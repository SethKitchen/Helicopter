//! Nonlinear coupled 8-state time-march (5g): all of `[u,w,q,θ,v,p,r,φ]`
//! integrated with both rotors in the loop, on the now-validated general-state
//! aero. Gated against 5d (longitudinal) and the corrected lateral oracle.

use crate::rk4::rk4_step;
use helisim_dynamics::{Inertia, hover_collective_for_weight, main_rotor_full, tail_thrust};
use helisim_flapping::Controls;
use helisim_trim::{Aircraft, NewtonConfig, TrimCondition, trim};

const G: f64 = 9.80665;

/// State `[u, w, q, θ, v, p, r, φ]` (same order as the coupled linear model).
pub type State8 = [f64; 8];

/// Fixed model inputs for the 8-state EOM: trimmed rotors + mass/inertia.
pub struct Model8<'a> {
    pub ac: &'a Aircraft,
    pub collective: f64,
    pub tail_collective: f64,
    pub controls: Controls,
    pub j: Inertia,
}

/// Nonlinear 8-state body-axis derivative. Aero from the general-state main-rotor
/// model + tail; full rigid-body kinematics with inertia cross-terms and gravity.
pub fn state_derivative8(m: &Model8, s: &[f64]) -> Vec<f64> {
    let (u, w, q, theta) = (s[0], s[1], s[2], s[3]);
    let (v, p, r, phi) = (s[4], s[5], s[6], s[7]);
    let ac = m.ac;
    let rotor = ac.main.with_collective(m.collective);

    let main = main_rotor_full(
        &rotor,
        &ac.main_op,
        ac.main_airfoil.as_ref(),
        &ac.flap,
        ac.hub_height,
        &m.controls,
        [u, v, w],
        [p, q],
    );
    let v_axial = v + p * ac.tail.height - r * ac.tail.arm;
    let t_tr = tail_thrust(ac, m.tail_collective, v_axial);

    let xf = main.fx;
    let yf = main.fy + t_tr;
    let zf = main.fz;
    let lm = main.mx + ac.tail.height * t_tr;
    let mm = main.my;
    let nm = main.mz - ac.tail.arm * t_tr;

    let (mass, ixx, iyy, izz) = (ac.mass, m.j.i_xx, m.j.i_yy, m.j.i_zz);
    let (gx, gy, gz) = (
        -G * theta.sin(),
        G * theta.cos() * phi.sin(),
        G * theta.cos() * phi.cos(),
    );

    let udot = -(q * w - r * v) + gx + xf / mass;
    let vdot = -(r * u - p * w) + gy + yf / mass;
    let wdot = -(p * v - q * u) + gz + zf / mass;
    let pdot = (lm + (iyy - izz) * q * r) / ixx;
    let qdot = (mm + (izz - ixx) * r * p) / iyy;
    let rdot = (nm + (ixx - iyy) * p * q) / izz;
    let thetadot = q * phi.cos() - r * phi.sin();
    let phidot = p + (q * phi.sin() + r * phi.cos()) * theta.tan();

    vec![udot, wdot, qdot, thetadot, vdot, pdot, rdot, phidot]
}

/// The hover equilibrium for the 8-state EOM, as
/// `(collective, tail_collective, θ1c, θ1s, θ_e, φ_e)`. Solved by a full 6-variable
/// Newton so the equilibrium residual is at solver precision — essential because
/// the equilibrium is UNSTABLE: any residual is amplified by the growing modes,
/// so an approximate balance (e.g. ignoring the θ1c→pitch cross-coupling) would
/// look like a spurious divergence.
pub fn solve_equilibrium8(ac: &Aircraft) -> (f64, f64, f64, f64, f64, f64) {
    // Initial guess from the existing pieces.
    let coll0 = hover_collective_for_weight(ac);
    let tail0 = trim(ac, &TrimCondition::hover(), &NewtonConfig::default()).tail_collective;
    let mut x = [coll0, tail0, 0.0, 0.0, 0.0, 0.0];

    for _ in 0..20 {
        let r = equilibrium_residual(ac, &x);
        if r.iter().map(|v| v * v).sum::<f64>().sqrt() < 1e-11 {
            break;
        }
        let mut jac = vec![vec![0.0; 6]; 6];
        for c in 0..6 {
            let h = 1e-6 * (1.0 + x[c].abs());
            let mut xp = x;
            xp[c] += h;
            let rp = equilibrium_residual(ac, &xp);
            for row in 0..6 {
                jac[row][c] = (rp[row] - r[row]) / h;
            }
        }
        let dx = solve_lin(jac, r.iter().map(|v| -v).collect());
        for i in 0..6 {
            x[i] += dx[i];
        }
    }
    (x[0], x[1], x[2], x[3], x[4], x[5])
}

/// Residual `[u̇, ẇ, q̇, v̇, ṗ, ṙ]` at rest for `[coll, tail, θ1c, θ1s, θ_e, φ_e]`.
fn equilibrium_residual(ac: &Aircraft, x: &[f64; 6]) -> [f64; 6] {
    let m = Model8 {
        ac,
        collective: x[0],
        tail_collective: x[1],
        controls: Controls {
            theta_1c: x[2],
            theta_1s: x[3],
        },
        j: Inertia {
            mass: ac.mass,
            i_xx: 1.0,
            i_yy: 1.0,
            i_zz: 1.0,
        },
    };
    let d = state_derivative8(&m, &[0.0, 0.0, 0.0, x[4], 0.0, 0.0, 0.0, x[5]]);
    [d[0], d[1], d[2], d[4], d[5], d[6]] // u̇, ẇ, q̇, v̇, ṗ, ṙ
}

/// Gaussian elimination with partial pivoting for a dense system.
pub(crate) fn solve_lin(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Vec<f64> {
    let n = b.len();
    for col in 0..n {
        let mut piv = col;
        for r in (col + 1)..n {
            if a[r][col].abs() > a[piv][col].abs() {
                piv = r;
            }
        }
        a.swap(col, piv);
        b.swap(col, piv);
        let pivot = a[col].clone(); // fixed during the elimination below
        for r in (col + 1)..n {
            let f = a[r][col] / pivot[col];
            for (c, v) in a[r].iter_mut().enumerate().skip(col) {
                *v -= f * pivot[c];
            }
            b[r] -= f * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = b[i];
        for c in (i + 1)..n {
            s -= a[i][c] * x[c];
        }
        x[i] = s / a[i][i];
    }
    x
}

/// The equilibrium state (rest, with the trimmed pitch θ_e and roll φ_e attitudes).
pub fn equilibrium_state8(ac: &Aircraft) -> State8 {
    let (_, _, _, _, theta_e, phi_e) = solve_equilibrium8(ac);
    [0.0, 0.0, 0.0, theta_e, 0.0, 0.0, 0.0, phi_e]
}

/// Build the trimmed model.
pub fn model8(ac: &Aircraft, j: Inertia) -> Model8<'_> {
    let (collective, tail_collective, theta1c, theta1s, _, _) = solve_equilibrium8(ac);
    Model8 {
        ac,
        collective,
        tail_collective,
        controls: Controls {
            theta_1c: theta1c,
            theta_1s: theta1s,
        },
        j,
    }
}

/// Integrate the nonlinear 8-state EOM from the equilibrium plus `perturbation`.
pub fn simulate8(
    ac: &Aircraft,
    j: Inertia,
    perturbation: State8,
    dt: f64,
    t_end: f64,
) -> Vec<State8> {
    let m = model8(ac, j);
    let eq = equilibrium_state8(ac);
    let mut s: Vec<f64> = (0..8).map(|i| eq[i] + perturbation[i]).collect();
    let mut out = vec![to8(&s)];
    let n = (t_end / dt).round() as usize;
    for _ in 0..n {
        s = rk4_step(&s, dt, |x| state_derivative8(&m, x));
        out.push(to8(&s));
    }
    out
}

/// Numerically linearize the 8-state EOM about the equilibrium → 8×8 matrix.
pub fn linearize8(ac: &Aircraft, j: Inertia) -> Vec<Vec<f64>> {
    let m = model8(ac, j);
    let eq = equilibrium_state8(ac);
    let eqv: Vec<f64> = eq.to_vec();
    let h = 1e-5;
    let mut a = vec![vec![0.0; 8]; 8];
    for col in 0..8 {
        let mut xp = eqv.clone();
        let mut xm = eqv.clone();
        xp[col] += h;
        xm[col] -= h;
        let fp = state_derivative8(&m, &xp);
        let fm = state_derivative8(&m, &xm);
        for row in 0..8 {
            a[row][col] = (fp[row] - fm[row]) / (2.0 * h);
        }
    }
    a
}

fn to8(s: &[f64]) -> State8 {
    [s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7]]
}
