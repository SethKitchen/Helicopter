//! Driven nonlinear 11-state time-march (5i): the coupled rigid body
//! `[u,w,q,θ,v,p,r,φ]` PLUS the Pitt–Peters inflow states `[λ₀,λ₁s,λ₁c]`,
//! integrated together with **time-varying control inputs**.
//!
//! This is the first sim with the dynamic inflow *in the loop* — the whole point
//! of building 5h first. With the inflow integrated, the rotor response carries
//! the correct timing (the 90° lag and the corrected off-axis sign), so the
//! driven cross-axis response is consistent with the `∂My/∂θ1c = +0.5` derivative
//! rather than the uniform-inflow `−3.2`.
//!
//! Validation (driven_validation.rs): control-effectiveness signs/magnitudes vs
//! the raw aero derivatives; the time-domain off-axis sign (and its contrast with
//! frozen inflow); and open-loop divergence at the linear-model rate — a control
//! pulse exciting the same unstable modes as an initial-condition perturbation.

use crate::control::{ControlSchedule, Trim};
use crate::driven_equilibrium::{equilibrium_state11, equilibrium_state11_at, model11, model11_at};
use crate::rigid_body::rigid_body_rates;
use crate::rk4::rk4_step_t;
use helisim_dynamics::{Inertia, inflow_rate, tail_thrust};
use helisim_flapping::Controls;
use helisim_trim::Aircraft;

/// State `[u, w, q, θ, v, p, r, φ, λ₀, λ₁s, λ₁c]`.
pub type State11 = [f64; 11];

/// Trimmed model inputs for the driven 11-state EOM.
pub struct Model11<'a> {
    pub ac: &'a Aircraft,
    pub collective: f64,
    pub tail_collective: f64,
    pub controls: Controls,
    pub j: Inertia,
}

/// Driven 11-state derivative at time `t` under control schedule `sched`. The four
/// pilot controls are trim + the schedule's deltas; the inflow is the integrated
/// `nu = [λ₀,λ₁s,λ₁c]` (not re-solved), with its Pitt–Peters rate appended.
pub fn state_derivative11(m: &Model11, sched: &dyn ControlSchedule, t: f64, s: &[f64]) -> Vec<f64> {
    deriv11(m, sched.deltas(t), s)
}

/// The 11-state derivative for explicit control deltas `[Δθ₀, Δθ1c, Δθ1s, Δθ_tail]`
/// from trim — the shared core, so a feedback controller (5j) can supply the deltas
/// the same way a time schedule (5i) does.
pub fn deriv11(m: &Model11, d: [f64; 4], s: &[f64]) -> Vec<f64> {
    let (u, w, q, theta) = (s[0], s[1], s[2], s[3]);
    let (v, p, r, phi) = (s[4], s[5], s[6], s[7]);
    let nu = [s[8], s[9], s[10]];
    let ac = m.ac;

    let collective = m.collective + d[0];
    let controls = Controls {
        theta_1c: m.controls.theta_1c + d[1],
        theta_1s: m.controls.theta_1s + d[2],
    };
    let tail_collective = m.tail_collective + d[3];

    let rotor = ac.main.with_collective(collective);
    let (nu_dot, main) = inflow_rate(
        &rotor,
        &ac.main_op,
        ac.main_airfoil.as_ref(),
        &ac.flap,
        ac.hub_height,
        &controls,
        [u, v, w],
        [p, q],
        nu,
        1.0,
    );
    let v_axial = v + p * ac.tail.height - r * ac.tail.arm;
    let t_tr = tail_thrust(ac, tail_collective, v_axial);

    let (xf, yf, zf) = (main.fx, main.fy + t_tr, main.fz);
    let lm = main.mx + ac.tail.height * t_tr;
    let mm = main.my;
    let nm = main.mz - ac.tail.arm * t_tr;

    let rb = rigid_body_rates(
        [u, w, q, theta, v, p, r, phi],
        [xf, yf, zf, lm, mm, nm],
        ac.mass,
        [m.j.i_xx, m.j.i_yy, m.j.i_zz],
    );

    vec![
        rb[0], rb[1], rb[2], rb[3], rb[4], rb[5], rb[6], rb[7], nu_dot[0], nu_dot[1], nu_dot[2],
    ]
}

/// Integrate the driven 11-state EOM from equilibrium + `perturbation` under
/// control schedule `sched`.
pub fn simulate11(
    ac: &Aircraft,
    j: Inertia,
    sched: &dyn ControlSchedule,
    perturbation: State11,
    dt: f64,
    t_end: f64,
) -> Vec<State11> {
    let m = model11(ac, j);
    let eq = equilibrium_state11(ac);
    let mut s: Vec<f64> = (0..11).map(|i| eq[i] + perturbation[i]).collect();
    let mut out = vec![to11(&s)];
    let mut t = 0.0;
    let n = (t_end / dt).round() as usize;
    for _ in 0..n {
        s = rk4_step_t(&s, t, dt, |tt, x| state_derivative11(&m, sched, tt, x));
        t += dt;
        out.push(to11(&s));
    }
    out
}

/// Numerically linearize the 11-state EOM about the hover equilibrium.
pub fn linearize11(ac: &Aircraft, j: Inertia) -> Vec<Vec<f64>> {
    linearize11_at(ac, j, [0.0, 0.0, 0.0])
}

/// Linearize about the equilibrium at a prescribed body velocity. Off the hover
/// wake-skew seam (`vel[0]>0`), this Jacobian SEES the λ₀↔λ₁c coupling the hover
/// linearization is blind to — the trustworthy oracle for SAS design.
pub fn linearize11_at(ac: &Aircraft, j: Inertia, vel: [f64; 3]) -> Vec<Vec<f64>> {
    let m = model11_at(ac, j, vel);
    let eq = equilibrium_state11_at(ac, vel);
    let h = 1e-6;
    let mut a = vec![vec![0.0; 11]; 11];
    for col in 0..11 {
        let mut xp = eq;
        let mut xm = eq;
        xp[col] += h;
        xm[col] -= h;
        let fp = state_derivative11(&m, &Trim, 0.0, &xp);
        let fm = state_derivative11(&m, &Trim, 0.0, &xm);
        for row in 0..11 {
            a[row][col] = (fp[row] - fm[row]) / (2.0 * h);
        }
    }
    a
}

/// Control-effectiveness matrix `B = ∂ẋ/∂u` (11×4) at the hover equilibrium.
pub fn control_matrix11(ac: &Aircraft, j: Inertia) -> [[f64; 4]; 11] {
    control_matrix11_at(ac, j, [0.0, 0.0, 0.0])
}

/// Control-effectiveness matrix at a prescribed body velocity.
pub fn control_matrix11_at(ac: &Aircraft, j: Inertia, vel: [f64; 3]) -> [[f64; 4]; 11] {
    let m = model11_at(ac, j, vel);
    let eq = equilibrium_state11_at(ac, vel);
    let mut b = [[0.0; 4]; 11];
    for ch in 0..4 {
        let h = 1e-6;
        let mut dp = [0.0; 4];
        let mut dm = [0.0; 4];
        dp[ch] = h;
        dm[ch] = -h;
        let fp = deriv11(&m, dp, &eq);
        let fm = deriv11(&m, dm, &eq);
        for row in 0..11 {
            b[row][ch] = (fp[row] - fm[row]) / (2.0 * h);
        }
    }
    b
}

fn to11(s: &[f64]) -> State11 {
    let mut a = [0.0; 11];
    a.copy_from_slice(&s[..11]);
    a
}
