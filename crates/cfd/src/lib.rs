//! **CFD** — a from-scratch viscous, incompressible 2-D Navier–Stokes core.
//!
//! Where the rest of the aero stack is reduced-order (blade-element/momentum,
//! finite-state inflow), this crate solves the actual Navier–Stokes equations on a
//! grid — std-only, zero dependencies, every operator hand-rolled — and validates
//! it against the gold-standard **Ghia, Ghia & Shin (1982)** lid-driven-cavity
//! benchmark (the way the FEA crate was validated against beam theory before being
//! applied).
//!
//! # Method
//!
//! The **vorticity–streamfunction** formulation (the one Ghia themselves used, so
//! the published ψ/ω *and* velocity data are directly comparable): a Poisson link
//! `∇²ψ = −ω` solved by SOR ([`poisson`]) coupled to an explicit vorticity-
//! transport march to steady state ([`cavity`]). Convection is upwinded (robust at
//! any cell-Reynolds number), diffusion central, wall vorticity from Thom's formula.
//!
//! # Scope & honesty
//!
//! * **Validated core, then application.** The steady solver is validated on the
//!   canonical Ghia benchmark; the *unsteady* solver against the exact Taylor–Green
//!   vortex ([`taylor_green`]); the **pressure field** — the quantity the
//!   streamfunction form drops, and the one forces need — is recovered via the
//!   pressure-Poisson equation ([`pressure`]); and a **body sits in the flow** —
//!   steady viscous flow past a circular cylinder on a body-fitted log-polar grid
//!   ([`cylinder`]), whose drag, wake length and separation angle match the
//!   Tritton/Dennis–Chang benchmark, with the forces from a surface integral. The
//!   remaining step toward airfoil `Cl/Cd` is a Joukowski conformal map of the same
//!   solver to an airfoil at incidence — named, not done.
//! * **First-order upwinding + uniform grid.** Accurate and robust; the absolute
//!   match to Ghia tightens with grid refinement (and a higher-order convection
//!   scheme), so the validation tolerances are honest about resolution.
//!
//! One concept per module:
//! * [`grid`]        — the uniform unit-square grid.
//! * [`poisson`]     — SOR solve of `∇²φ = rhs` (validated by a manufactured solution).
//! * [`cavity`]      — the lid-driven-cavity vorticity–streamfunction solver.
//! * [`pressure`]    — pressure-Poisson recovery from a velocity field (toward forces).
//! * [`taylor_green`]— exact unsteady-NS validation (Taylor–Green vortex decay).
//! * [`solution`]    — the flow field + Ghia-comparison diagnostics.

pub mod cavity;
pub mod cylinder;
pub mod cylinder_solution;
pub mod grid;
pub mod poisson;
pub mod polar_grid;
pub mod pressure;
pub mod solution;
pub mod taylor_green;

pub use cavity::{CavityConfig, solve_cavity};
pub use cylinder::{CylinderConfig, solve_cylinder};
pub use cylinder_solution::CylinderSolution;
pub use grid::Grid;
pub use poisson::{optimal_omega, sor_solve};
pub use polar_grid::PolarGrid;
pub use pressure::{pressure_source, recover_pressure, solve_pressure};
pub use solution::CavitySolution;
pub use taylor_green::TaylorGreen;
