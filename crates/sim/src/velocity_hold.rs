//! Velocity / position hold (5m): the outermost cascade loop, on the validated
//! inner stack — velocity error → attitude command → the 5k/5l PI attitude loop →
//! the 5j rate loop → controls.
//!
//! For hover-hold the velocity command is zero and the velocity-error integrator
//! IS the position (`∫u = x_position`), so proportional position feedback through
//! the integrator gives **zero steady-state velocity AND position error**: drift
//! arrested, position held. State vector grows to 15:
//! `[plant(11), z_θ, z_φ, ζ_u, ζ_v]` — attitude integrators + velocity/position
//! integrators.
//!
//! # Design discipline (the cascade rules, named before tuning)
//!
//! **Timescale separation:** each outer loop ~4× slower than the loop it commands
//! (within the textbook 3–5× rule): rate (fast) → attitude (mid) → velocity (slow).
//! The closed-loop eigenvalues must show three separated clusters; if the velocity
//! poles crowd the attitude poles the loops fight (a lightly-damped outer mode).
//! Validated by the cluster spread, not by eye.
//!
//! **Pre-computed tracking target:** 5l left a ~1.6 m/s velocity drift under a
//! sustained 0.6 N·m disturbance with attitude hold alone. The velocity loop must
//! drive THAT drift to ≈0 (and arrest position). The inner-loop residual is the
//! oracle — no external reference needed.
//!
//! **Seam discipline (3rd application):** design/validate off the seam (5 m/s,
//! χ differentiable, eigenvalues trustworthy), then confirm across the seam at
//! hover — where the outer loop, having authority over the slow drift, should
//! finally close the hover seam-residual the inner loops structurally leave.
//!
//! Scope: hold + steady velocity command. NOT trajectory guidance, mission
//! autopilot, or envelope protection.

use crate::control::Channel;
use crate::driven_equilibrium::{equilibrium_state11_at, model11_at};
use crate::driven_march::{Model11, deriv11};
use crate::rk4::rk4_step_t;
use crate::sim_setup::Sim11Setup;
use helisim_dynamics::Inertia;
use helisim_trim::Aircraft;

/// State `[u,w,q,θ,v,p,r,φ,λ₀,λ₁s,λ₁c, z_θ, z_φ, ζ_u, ζ_v]`.
pub type State15 = [f64; 15];

/// Full nested-cascade gains: inner rate, mid attitude (PI), outer velocity (PI).
#[derive(Clone, Copy, Debug)]
pub struct VelocityHold {
    pub gp: f64,
    pub gq: f64,
    pub gr: f64,
    pub k_theta: f64,
    pub k_phi: f64,
    pub ki_theta: f64,
    pub ki_phi: f64,
    pub k_u: f64,
    pub k_v: f64,
    pub ki_u: f64,
    pub ki_v: f64,
}

impl VelocityHold {
    /// The default nested stack: 5j rate (0.2/0.2/0.4), 5k/5l attitude (0.1 P,
    /// 0.3 I), and an outer velocity loop tuned ~4× slower than the attitude loop.
    pub fn hover_hold() -> Self {
        Self {
            gp: 0.2,
            gq: 0.2,
            gr: 0.4,
            k_theta: 0.1,
            k_phi: 0.1,
            ki_theta: 0.3,
            ki_phi: 0.3,
            k_u: 0.02,
            k_v: 0.02,
            ki_u: 0.005,
            ki_v: 0.005,
        }
    }
}

/// Closed-loop 15-state derivative about `eq`, holding velocity command `cmd =
/// [u_cmd, v_cmd]` (perturbations from `eq`), under external moment `disturb`.
pub fn deriv15(
    m: &Model11,
    vh: &VelocityHold,
    eq: &[f64; 11],
    j: Inertia,
    cmd: [f64; 2],
    disturb: [f64; 3],
    s: &[f64],
) -> Vec<f64> {
    let x = &s[..11];
    let (z_theta, z_phi, zeta_u, zeta_v) = (s[11], s[12], s[13], s[14]);

    // Outer loop: velocity error → attitude command (P + I; the integrator is position).
    let u_err = (x[0] - eq[0]) - cmd[0];
    let v_err = (x[4] - eq[4]) - cmd[1];
    let theta_cmd = vh.k_u * u_err + vh.ki_u * zeta_u;
    let phi_cmd = -(vh.k_v * v_err + vh.ki_v * zeta_v);

    // Mid loop: attitude tracking error about the commanded attitude.
    let theta_err = (x[3] - eq[3]) - theta_cmd;
    let phi_err = (x[7] - eq[7]) - phi_cmd;

    // Controls: rate damping + attitude PI tracking the command.
    let mut d = [0.0; 4];
    d[Channel::LonCyclic as usize] = -vh.gq * x[2] - vh.k_theta * theta_err - vh.ki_theta * z_theta;
    d[Channel::LatCyclic as usize] = -vh.gp * x[5] - vh.k_phi * phi_err - vh.ki_phi * z_phi;
    d[Channel::Pedal as usize] = vh.gr * x[6];

    let mut ds = deriv11(m, d, x);
    ds[5] += disturb[0] / j.i_xx;
    ds[2] += disturb[1] / j.i_yy;
    ds[6] += disturb[2] / j.i_zz;
    ds.push(theta_err); // ż_θ
    ds.push(phi_err); // ż_φ
    ds.push(u_err); // ζ̇_u  (= position rate for hover-hold)
    ds.push(v_err); // ζ̇_v
    ds
}

/// Integrate the 15-state hover/velocity-hold cascade about the equilibrium at
/// body velocity `vel`, holding command `cmd`, under disturbance `disturb`.
pub fn simulate15(
    setup: &Sim11Setup,
    vh: &VelocityHold,
    cmd: [f64; 2],
    disturb: [f64; 3],
    perturbation: State15,
    span: [f64; 2],
) -> Vec<State15> {
    let Sim11Setup { ac, j, vel } = *setup;
    let [dt, t_end] = span;
    let m = model11_at(ac, j, vel);
    let eq = equilibrium_state11_at(ac, vel);
    let mut s: Vec<f64> = (0..15)
        .map(|i| if i < 11 { eq[i] } else { 0.0 } + perturbation[i])
        .collect();
    let mut out = vec![to15(&s)];
    let mut t = 0.0;
    let n = (t_end / dt).round() as usize;
    for _ in 0..n {
        s = rk4_step_t(&s, t, dt, |_, x| deriv15(&m, vh, &eq, j, cmd, disturb, x));
        t += dt;
        out.push(to15(&s));
    }
    out
}

/// Numerically linearize the 15-state closed loop about the equilibrium (cmd = 0,
/// no disturbance) → 15×15 matrix; its eigenvalues are the cascade poles whose
/// cluster spread is the timescale-separation gate.
pub fn linearize15(ac: &Aircraft, j: Inertia, vel: [f64; 3], vh: &VelocityHold) -> Vec<Vec<f64>> {
    let m = model11_at(ac, j, vel);
    let eq = equilibrium_state11_at(ac, vel);
    let mut base = [0.0; 15];
    base[..11].copy_from_slice(&eq);
    let h = 1e-6;
    let mut a = vec![vec![0.0; 15]; 15];
    for col in 0..15 {
        let mut xp = base;
        let mut xm = base;
        xp[col] += h;
        xm[col] -= h;
        let fp = deriv15(&m, vh, &eq, j, [0.0; 2], [0.0; 3], &xp);
        let fm = deriv15(&m, vh, &eq, j, [0.0; 2], [0.0; 3], &xm);
        for row in 0..15 {
            a[row][col] = (fp[row] - fm[row]) / (2.0 * h);
        }
    }
    a
}

fn to15(s: &[f64]) -> State15 {
    let mut a = [0.0; 15];
    a.copy_from_slice(&s[..15]);
    a
}
