//! Couples the main-rotor aerodynamics ([`crate::full_aero`]) to the Pitt–Peters
//! inflow states ([`crate::pitt_peters`]).
//!
//! Two regimes:
//! * **Quasi-static** — the inflow is the fixed point `ν = [L]·C(ν)`; this is what
//!   the inner loop solved through 5g (generalised to all three states).
//! * **Dynamic** — the inflow is integrated: `ν̇ = f(ν, C(ν))`. The `lag` knob
//!   scales the apparent mass; `lag → 0` collapses the dynamics onto the
//!   quasi-static fixed point (the falsifiable τ→0 gate).

use crate::full_aero::{Forces6, InflowAero, assemble_forces};
use crate::pitt_peters::{inflow_derivative, steady_inflow_for};
use helisim_airfoil::Airfoil;
use helisim_flapping::{Controls, FlapProperties};
use helisim_rotor::{Operating, Rotor};

/// Main-rotor forces/moments at velocity `(u,v,w)` and rates `(p,q)` for an
/// **externally-supplied** linear inflow `[λ₀,λ₁s,λ₁c]` (the Pitt–Peters states),
/// plus the aerodynamic forcing the inflow responds to. The inflow is NOT solved
/// here — it is an integrated state of the dynamic-inflow model.
pub fn main_rotor_with_inflow(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    props: &FlapProperties,
    hub_height: f64,
    controls: &Controls,
    vel: [f64; 3],
    rates: [f64; 2],
    inflow: [f64; 3],
) -> (Forces6, InflowAero) {
    let vt = op.tip_speed(rotor.radius);
    let (mu_u, mu_v) = (vel[0] / vt, vel[1] / vt);
    let (p_bar, q_bar) = (rates[0] / op.omega, rates[1] / op.omega);
    // Fold the freestream-normal (heave) velocity into the mean inflow.
    let inflow = [inflow[0] - vel[2] / vt, inflow[1], inflow[2]];
    assemble_forces(
        rotor, op, airfoil, props, hub_height, mu_u, mu_v, p_bar, q_bar, controls, inflow,
    )
}

/// Mean inflow magnitude and advance ratio for the Pitt–Peters `[L]` matrix.
fn flow_scales(op: &Operating, rotor: &Rotor, vel: [f64; 3], nu0: f64) -> (f64, f64) {
    let vt = op.tip_speed(rotor.radius);
    let mu = (vel[0] * vel[0] + vel[1] * vel[1]).sqrt() / vt;
    let lambda = (nu0 - vel[2] / vt).abs().max(1e-3); // induced + climb
    (mu, lambda)
}

/// Quasi-static three-state inflow: the fixed point `ν = [L]·C(ν)`, plus the
/// resulting body forces and the aerodynamic forcing at convergence.
pub fn quasi_static_inflow(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    props: &FlapProperties,
    hub_height: f64,
    controls: &Controls,
    vel: [f64; 3],
    rates: [f64; 2],
) -> (Forces6, [f64; 3], InflowAero) {
    let mut nu = [0.05, 0.0, 0.0];
    let mut out = (Forces6::default(), InflowAero::default());
    for _ in 0..80 {
        out = main_rotor_with_inflow(
            rotor, op, airfoil, props, hub_height, controls, vel, rates, nu,
        );
        let c = [out.1.c_t, out.1.c_roll, out.1.c_pitch];
        let (mu, lambda) = flow_scales(op, rotor, vel, nu[0]);
        let target = steady_inflow_for(c, mu, lambda);
        let d: f64 = (0..3)
            .map(|i| (target[i] - nu[i]).abs())
            .fold(0.0, f64::max);
        for i in 0..3 {
            nu[i] += 0.5 * (target[i] - nu[i]); // relaxed fixed-point iteration
        }
        if d < 1e-12 {
            break;
        }
    }
    (out.0, nu, out.1)
}

/// One Pitt–Peters inflow rate `ν̇` for the current state `nu`, given the flight
/// condition. `lag` scales the apparent mass (1 = Pitt–Peters; →0 = quasi-static).
pub fn inflow_rate(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    props: &FlapProperties,
    hub_height: f64,
    controls: &Controls,
    vel: [f64; 3],
    rates: [f64; 2],
    nu: [f64; 3],
    lag: f64,
) -> ([f64; 3], Forces6) {
    let (forces, aero) = main_rotor_with_inflow(
        rotor, op, airfoil, props, hub_height, controls, vel, rates, nu,
    );
    let c = [aero.c_t, aero.c_roll, aero.c_pitch];
    let (mu, lambda) = flow_scales(op, rotor, vel, nu[0]);
    let d = inflow_derivative(nu, c, mu, lambda, op.omega, lag);
    (d, forces)
}

/// March the dynamic inflow (forward Euler, fixed `dt` seconds) from `nu0` for
/// `steps`, returning the final inflow and body forces. Used to show that as
/// `lag → 0` the dynamic inflow collapses onto [`quasi_static_inflow`].
pub fn march_inflow(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    props: &FlapProperties,
    hub_height: f64,
    controls: &Controls,
    vel: [f64; 3],
    rates: [f64; 2],
    nu0: [f64; 3],
    lag: f64,
    dt: f64,
    steps: usize,
) -> ([f64; 3], Forces6) {
    let mut nu = nu0;
    let mut forces = Forces6::default();
    for _ in 0..steps {
        let (d, f) = inflow_rate(
            rotor, op, airfoil, props, hub_height, controls, vel, rates, nu, lag,
        );
        forces = f;
        for i in 0..3 {
            nu[i] += dt * d[i];
        }
    }
    (nu, forces)
}
