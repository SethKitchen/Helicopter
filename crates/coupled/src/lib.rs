//! Two-way flap↔inflow coupling for forward flight.
//!
//! In the forward-flight milestone the blade was rigid and, in the flapping
//! milestone, flapping responded to a *frozen* inflow. That one-way coupling
//! over-predicts advancing-side thrust (the blade should flap up there, shedding
//! lift), which made high-μ trim collective/power collapse to unphysical values.
//!
//! This crate closes the loop. The three quantities are mutually dependent:
//!
//! ```text
//! inflow   λ        depends on the disk loading  (momentum)
//! loading  C_T(ψ,x) depends on the flapping      (β changes the local AoA)
//! flapping β        depends on the inflow        (flap moment uses λ)
//! ```
//!
//! [`solve_coupled`] iterates them to a joint fixed point: at each step it solves
//! the flapping for the current inflow, integrates the blade loads *with that
//! flapping folded into the through-disk velocity* `u_P = λ + xβ' + μβcosψ`, and
//! updates the inflow from momentum. The iteration is a contraction (more inflow
//! → less AoA → less thrust → less inflow), so it converges monotonically.
//!
//! The physical payoff: the converged loading is far more equalised between
//! advancing and retreating sides than the rigid result — the flap response
//! redistributes the loading, killing the over-prediction and restoring physical
//! high-μ power.
//!
//! One concept per module:
//! * [`config`]   — [`CoupledConfig`].
//! * [`loads`]    — blade-element thrust/power integral *with flapping*.
//! * [`solution`] — [`CoupledSolution`].
//! * [`solver`]   — [`solve_coupled`] (the flap↔inflow fixed point).

pub mod config;
pub mod loads;
pub mod solution;
pub mod solver;

pub use config::CoupledConfig;
pub use solution::CoupledSolution;
pub use solver::solve_coupled;
