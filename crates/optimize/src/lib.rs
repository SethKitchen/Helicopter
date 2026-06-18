//! Std-only optimization for design synthesis.
//!
//! The project's analysis stack answers "given design X, how does it perform?".
//! This crate supplies the inverse — "find the X that performs best" — as a new
//! solver shape: derivative-free [`nelder_mead::minimize`] over an [`Objective`],
//! soft inequality [`constraint::Penalized`] handling, and a [`pareto::pareto_front`]
//! for the genuinely multi-objective case (no single optimum exists).
//!
//! Every routine is validated against a problem with a KNOWN optimum / analytic
//! front (`tests/`), per the project rule that a "match" is a passing `#[test]`.

mod constraint;
mod nelder_mead;
mod objective;
mod pareto;

pub use constraint::{ConstraintFn, Penalized};
pub use nelder_mead::{NmOptions, NmResult, minimize};
pub use objective::{FnObjective, Objective};
pub use pareto::{dominates, pareto_front};
