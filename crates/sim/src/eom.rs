//! Nonlinear longitudinal equations of motion.
//!
//! Body axes (x fwd, z down), state `[u, w, q, θ]`, controls held at trim:
//!
//! ```text
//! u̇ = −q·w − g·sinθ + X_aero/m
//! ẇ =  q·u + g·cosθ + Z_aero/m
//! q̇ = M_aero / Iyy
//! θ̇ = q
//! ```
//!
//! The aerodynamic `X, Z, M` come from the quasi-static main-rotor solve
//! (`dynamics::longitudinal_main_aero`) at the current `(u, w, q)` — the rotor as
//! a callee inside the integration loop. Linearising these EOM about hover
//! (`u=w=q=0, θ≈0`) reproduces the 5c system matrix exactly, which is why the
//! time-march must reproduce the 5c eigenvalues.

use helisim_airfoil::Airfoil;
use helisim_dynamics::aero::longitudinal_main_aero;
use helisim_dynamics::RotorAero;
use helisim_flapping::Controls;
use helisim_flapping::FlapProperties;
use helisim_rotor::{Operating, Rotor};

const G: f64 = 9.80665;

/// Longitudinal state `[u, w, q, θ]`.
pub type LongState = [f64; 4];

/// All the fixed inputs the EOM needs (trimmed rotor + mass properties).
pub struct LongModel<'a> {
    /// Main rotor with the trim collective baked in.
    pub rotor: Rotor,
    /// Main rotor operating point.
    pub op: Operating,
    /// Sectional aero.
    pub airfoil: &'a dyn Airfoil,
    /// Flap properties.
    pub flap: FlapProperties,
    /// Trim cyclic controls.
    pub controls: Controls,
    /// Hub height above CG, m.
    pub hub_height: f64,
    /// Gross mass, kg.
    pub mass: f64,
    /// Pitch inertia, kg·m².
    pub i_yy: f64,
}

/// The nonlinear longitudinal state derivative `[u̇, ẇ, q̇, θ̇]`.
pub fn state_derivative(m: &LongModel, s: &[f64]) -> Vec<f64> {
    let (u, w, q, theta) = (s[0], s[1], s[2], s[3]);
    let aero = longitudinal_main_aero(
        &RotorAero {
            rotor: &m.rotor,
            op: &m.op,
            airfoil: m.airfoil,
            props: &m.flap,
            hub_height: m.hub_height,
            controls: &m.controls,
        },
        u,
        w,
        q,
    );
    let udot = -q * w - G * theta.sin() + aero.x_force / m.mass;
    let wdot = q * u + G * theta.cos() + aero.z_force / m.mass;
    let qdot = aero.pitch_moment / m.i_yy;
    let thetadot = q;
    vec![udot, wdot, qdot, thetadot]
}
