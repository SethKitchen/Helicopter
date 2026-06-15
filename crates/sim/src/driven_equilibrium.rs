//! Equilibria for the driven 11-state system: the trimmed steady state at a
//! prescribed body velocity (the time-march in [`crate::driven_march`] is a
//! separate concern). Hover is `vel=[0,0,0]`; a small forward `u` puts the
//! linearization point OFF the wake-skew seam (μ>0, χ differentiable) — the
//! trustworthy oracle for control design (5j–5l) that hover cannot provide.

use crate::control::Trim;
use crate::coupled_march::solve_lin;
use crate::driven_march::{Model11, State11, state_derivative11};
use helisim_dynamics::{Inertia, RotorAero, hover_collective_for_weight, quasi_static_inflow};
use helisim_flapping::Controls;
use helisim_trim::{Aircraft, NewtonConfig, TrimCondition, trim};

/// The hover equilibrium for the 11-state system. See [`solve_equilibrium11_at`].
pub fn solve_equilibrium11(ac: &Aircraft) -> (f64, f64, f64, f64, f64, f64, [f64; 3]) {
    solve_equilibrium11_at(ac, [0.0, 0.0, 0.0])
}

/// Equilibrium for a prescribed body velocity `vel = [u,v,w]`:
/// `(collective, tail, θ1c, θ1s, θ_e, φ_e, ν_e[3])`. Six-variable body Newton with
/// the main rotor through the **quasi-static three-state inflow** (steady ν makes
/// ν̇ = 0 by construction).
pub fn solve_equilibrium11_at(
    ac: &Aircraft,
    vel: [f64; 3],
) -> (f64, f64, f64, f64, f64, f64, [f64; 3]) {
    let coll0 = hover_collective_for_weight(ac);
    let tail0 = trim(ac, &TrimCondition::hover(), &NewtonConfig::default()).tail_collective;
    let mut x = [coll0, tail0, 0.0, 0.0, 0.0, 0.0];

    for _ in 0..40 {
        let r = equilibrium_residual(ac, &x, vel);
        if r.iter().map(|v| v * v).sum::<f64>().sqrt() < 1e-11 {
            break;
        }
        let mut jac = vec![vec![0.0; 6]; 6];
        for c in 0..6 {
            let h = 1e-6 * (1.0 + x[c].abs());
            let mut xp = x;
            xp[c] += h;
            let rp = equilibrium_residual(ac, &xp, vel);
            for row in 0..6 {
                jac[row][c] = (rp[row] - r[row]) / h;
            }
        }
        let dx = solve_lin(jac, r.iter().map(|v| -v).collect());
        for i in 0..6 {
            x[i] += dx[i];
        }
    }
    let nu_e = quasi_inflow(ac, &x, vel);
    (x[0], x[1], x[2], x[3], x[4], x[5], nu_e)
}

/// Steady 3-state inflow at the given controls/attitude and body velocity.
fn quasi_inflow(ac: &Aircraft, x: &[f64; 6], vel: [f64; 3]) -> [f64; 3] {
    let rotor = ac.main.with_collective(x[0]);
    let controls = Controls {
        theta_1c: x[2],
        theta_1s: x[3],
    };
    quasi_static_inflow(
        &RotorAero {
            rotor: &rotor,
            op: &ac.main_op,
            airfoil: ac.main_airfoil.as_ref(),
            props: &ac.flap,
            hub_height: ac.hub_height,
            controls: &controls,
        },
        vel,
        [0.0, 0.0],
    )
    .1
}

/// Residual `[u̇, ẇ, q̇, v̇, ṗ, ṙ]` at body velocity `vel`, zero rates, quasi-static inflow.
fn equilibrium_residual(ac: &Aircraft, x: &[f64; 6], vel: [f64; 3]) -> [f64; 6] {
    let m = Model11 {
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
    let nu = quasi_inflow(ac, x, vel);
    let s = [
        vel[0], vel[1], vel[2], x[4], 0.0, 0.0, 0.0, x[5], nu[0], nu[1], nu[2],
    ];
    let d = state_derivative11(&m, &Trim, 0.0, &s);
    [d[0], d[1], d[2], d[4], d[5], d[6]]
}

/// The hover equilibrium state vector (rest, trimmed attitudes, steady inflow).
pub fn equilibrium_state11(ac: &Aircraft) -> State11 {
    equilibrium_state11_at(ac, [0.0, 0.0, 0.0])
}

/// The equilibrium state vector at a prescribed body velocity.
pub fn equilibrium_state11_at(ac: &Aircraft, vel: [f64; 3]) -> State11 {
    let (_, _, _, _, te, pe, nu) = solve_equilibrium11_at(ac, vel);
    [
        vel[0], vel[1], vel[2], te, 0.0, 0.0, 0.0, pe, nu[0], nu[1], nu[2],
    ]
}

/// Build the trimmed driven model (hover).
pub fn model11(ac: &Aircraft, j: Inertia) -> Model11<'_> {
    model11_at(ac, j, [0.0, 0.0, 0.0])
}

/// Build the trimmed driven model at a prescribed body velocity.
pub fn model11_at(ac: &Aircraft, j: Inertia, vel: [f64; 3]) -> Model11<'_> {
    let (collective, tail_collective, t1c, t1s, _, _, _) = solve_equilibrium11_at(ac, vel);
    Model11 {
        ac,
        collective,
        tail_collective,
        controls: Controls {
            theta_1c: t1c,
            theta_1s: t1s,
        },
        j,
    }
}
