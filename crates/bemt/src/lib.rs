//! Blade Element Momentum Theory (BEMT) solver for a single rotor in steady,
//! axial hover.
//!
//! # Method
//!
//! The blade is divided into annuli at `x = r/R`. At each station the local
//! induced inflow ratio `lambda` is found by enforcing that the thrust from
//! *blade element theory* (the section's own lift and drag) equals the thrust
//! from *momentum theory* (the momentum change of the air through that annulus).
//! The two couple through the inflow angle, so each station is solved for
//! `lambda` with a robust bracketed bisection.
//!
//! Nondimensionalisation (Leishman, *Principles of Helicopter Aerodynamics*),
//! velocities scaled by tip speed `Omega*R`:
//!
//! ```text
//! phi   = atan2(lambda, x)                                  inflow angle
//! alpha = theta(x) - phi                                    angle of attack
//! M     = M_tip * sqrt(x^2 + lambda^2)                      local Mach
//! dCT/dx = (sigma/2)(x^2+lambda^2)(Cl cos phi - Cd sin phi) blade element
//! dCT/dx = 4 F lambda^2 x                                   momentum (hover)
//! dCP/dx = (sigma/2)(x^2+lambda^2)(Cl sin phi + Cd cos phi) x
//! ```
//!
//! `F` is the Prandtl tip-loss factor. Blade-element thrust *decreases* with
//! `lambda` while momentum thrust *increases*, so the residual is monotone and
//! bisection finds the unique physical root. The span is integrated by the
//! trapezoidal rule from the root cutout to the tip. `C_P == C_Q` (power =
//! `Omega` * torque), so a single coefficient is reported.
//!
//! One concept per module:
//! * [`config`]   — solver settings ([`Config`]).
//! * [`tip_loss`] — Prandtl tip-loss factor.
//! * [`station`]  — per-station converged state ([`Station`]).
//! * [`solution`] — integrated result ([`HoverSolution`]).
//! * [`solver`]   — the [`solve_hover`] entry point.

pub mod config;
pub mod solution;
pub mod solver;
pub mod station;
pub mod tip_loss;

pub use config::Config;
pub use solution::HoverSolution;
pub use solver::solve_hover;
pub use station::Station;
pub use tip_loss::prandtl_tip_loss;
