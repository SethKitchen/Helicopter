//! Blade-element thrust/power integral with the flap motion folded in.
//!
//! Identical in spirit to the forward-flight integral, but the through-disk
//! velocity carries the flapping: `u_P = λ + x β' + μ β cosψ`. That is the whole
//! point of the coupling — on the advancing side the blade flaps up (`β' > 0`),
//! raising `u_P`, lowering the angle of attack, and shedding the excess lift the
//! rigid blade over-predicted.

use helisim_airfoil::Airfoil;
use helisim_flapping::Controls;
use helisim_rotor::Rotor;
use std::f64::consts::PI;

/// Azimuth-averaged loads at a fixed inflow and flap state.
#[derive(Clone, Copy, Default)]
pub struct Loads {
    /// Thrust coefficient.
    pub ct: f64,
    /// Power/torque coefficient (full torque integral).
    pub cp: f64,
    /// Profile-only power coefficient (the Cd term — always ≥ 0; the reliable
    /// part at high μ where the full torque integral becomes autorotative).
    pub cp_profile: f64,
    /// Rolling-moment coefficient.
    pub c_roll: f64,
    /// Pitching-moment coefficient.
    pub c_pitch: f64,
    /// Advancing-half mean thrust coefficient.
    pub advancing_ct: f64,
    /// Retreating-half mean thrust coefficient.
    pub retreating_ct: f64,
}

/// Flow condition for the blade-element integral: tip Mach, advance ratio, inflow.
#[derive(Clone, Copy, Debug)]
pub struct Flow {
    pub tip_mach: f64,
    pub mu: f64,
    pub lambda: f64,
}

/// Integrate the blade loads over azimuth and radius with flapping
/// `flap = [β0, β1c, β1s]` and cyclic pitch `controls`, at flow condition `flow`,
/// on a `grid = [n_azimuth, n_radial]`. Reverse-flow elements (`u_T<0`) are nulled.
pub fn integrate_with_flap(
    rotor: &Rotor,
    airfoil: &dyn Airfoil,
    flow: Flow,
    controls: &Controls,
    flap: [f64; 3],
    grid: [usize; 2],
) -> Loads {
    let Flow { tip_mach, mu, lambda } = flow;
    let [beta0, beta1c, beta1s] = flap;
    let [n_azimuth, n_radial] = grid;
    let x0 = rotor.root_cutout;
    let dx = (1.0 - x0) / n_radial as f64;
    let dpsi = 2.0 * PI / n_azimuth as f64;

    let mut acc = Loads::default();
    for j in 0..n_azimuth {
        let psi = (j as f64 + 0.5) * dpsi;
        let (sp, cp) = psi.sin_cos();
        let beta = beta0 - beta1c * cp - beta1s * sp;
        let beta_dot = beta1c * sp - beta1s * cp;
        let theta_cyclic = controls.theta_1c * cp + controls.theta_1s * sp;
        let mut ct_psi = 0.0;

        for i in 0..n_radial {
            let x = x0 + (i as f64 + 0.5) * dx;
            let u_t = x + mu * sp;
            if u_t <= 0.0 {
                continue; // reverse flow nulled
            }
            let u_p = lambda + x * beta_dot + mu * beta * cp;
            let u2 = u_t * u_t + u_p * u_p;
            let phi = u_p.atan2(u_t);
            let alpha = rotor.pitch(x) + theta_cyclic - phi;
            let mach = tip_mach * u2.sqrt();
            let (cl, cd) = airfoil.cl_cd(alpha, mach);

            let sigma = rotor.local_solidity(x);
            let (s, c) = phi.sin_cos();
            let w = 0.5 * sigma * u2;
            let dct = w * (cl * c - cd * s) * dx;
            ct_psi += dct;
            acc.cp += w * (cl * s + cd * c) * x * dx;
            acc.cp_profile += w * (cd * c) * x * dx;
            acc.c_roll += dct * x * sp;
            acc.c_pitch += dct * x * cp;
        }

        acc.ct += ct_psi;
        if sp > 0.0 {
            acc.advancing_ct += ct_psi;
        } else {
            acc.retreating_ct += ct_psi;
        }
    }

    let n = n_azimuth as f64;
    acc.ct /= n;
    acc.cp /= n;
    acc.cp_profile /= n;
    acc.c_roll /= n;
    acc.c_pitch /= n;
    acc.advancing_ct /= n / 2.0;
    acc.retreating_ct /= n / 2.0;
    acc
}
