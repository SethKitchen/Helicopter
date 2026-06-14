//! Steady-flight trim for a single-main-rotor + tail-rotor helicopter.
//!
//! Trim finds the control inputs and fuselage attitude that make all forces and
//! moments balance for a steady flight condition (hover or steady level forward
//! flight). The six unknowns are
//!
//! ```text
//! θ₀     main-rotor collective
//! θ₁c    lateral cyclic
//! θ₁s    longitudinal cyclic
//! θ₀_tr  tail-rotor collective
//! θ_f    fuselage pitch attitude
//! φ_f    fuselage roll attitude
//! ```
//!
//! and the six equations are the body-axis force balance (X, Y, Z) and moment
//! balance (roll, pitch, yaw). This is a multidimensional root find — the
//! project's **third solver shape**, after monotone-residual bisection (hover /
//! forward inflow, pack current) and the harmonic-balance linear solve
//! (flapping). It is a small **Newton iteration with a numerically-estimated
//! Jacobian** ([`newton`]), and it *reuses everything*: hover BEMT, forward
//! inflow and flapping all become the residual function trim drives to zero.
//!
//! # Model
//!
//! * Main rotor thrust/torque/power from hover BEMT (μ≈0) or forward BEMT (μ>0);
//!   thrust acts perpendicular to the tip-path plane, tilted from the shaft by
//!   the flapping (β₁c, β₁s) the cyclic produces.
//! * Tail rotor modelled with the same hover BEMT — a thrust source for yaw
//!   balance and a real power draw.
//! * One-way inflow coupling carried over from the flapping milestone.
//! * Central or offset flap hinge; CG on the shaft a fixed height below the hub.
//!
//! # Scope (5a)
//!
//! Hover and steady **level** forward flight only — no maneuvers, turns, or
//! trimmed climbs (a climb analyzer already exists). Fuselage parasite drag is a
//! simple flat plate here; the full airframe/parasite model is milestone 5b.
//!
//! One concept per module:
//! * [`newton`]    — multidimensional Newton solver (numerical Jacobian).
//! * [`aircraft`]  — [`Aircraft`] definition (rotors, geometry, mass).
//! * [`condition`] — [`TrimCondition`] (hover / forward speed).
//! * [`residual`]  — the six force/moment residuals.
//! * [`solution`]  — [`TrimResult`].
//! * [`solver`]    — [`trim`].

pub mod aircraft;
pub mod condition;
pub mod newton;
pub mod residual;
pub mod solution;
pub mod solver;

pub use aircraft::{Aircraft, TailRotor};
pub use condition::TrimCondition;
pub use newton::{NewtonConfig, solve_newton};
pub use solution::TrimResult;
pub use solver::trim;

/// Standard gravity, m/s².
pub const G: f64 = 9.80665;
