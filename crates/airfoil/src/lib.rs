//! Sectional airfoil aerodynamics for the helisim BEMT solver.
//!
//! The [`Airfoil`] trait is the polymorphism boundary: the solver depends only on
//! the trait, never on a concrete model, so airfoils can be swapped freely
//! (analytic vs. tabulated) and new models added without touching the solver.
//!
//! One concept per module:
//! * [`airfoil`]  — the [`Airfoil`] trait.
//! * [`linear`]   — analytic linear-lift model with stall + compressibility.
//! * [`table`]    — piecewise-linear interpolation of a measured polar.

pub mod airfoil;
pub mod linear;
pub mod table;

pub use airfoil::Airfoil;
pub use linear::LinearAirfoil;
pub use table::TableAirfoil;
