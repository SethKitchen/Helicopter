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
//! * **Validated core, then application.** This first milestone establishes and
//!   validates the solver on the canonical benchmark. The natural follow-on is the
//!   primitive-variable (pressure) form for an airfoil section → viscous `Cl/Cd`,
//!   which needs the pressure field the streamfunction form omits — named, not done.
//! * **First-order upwinding + uniform grid.** Accurate and robust; the absolute
//!   match to Ghia tightens with grid refinement (and a higher-order convection
//!   scheme), so the validation tolerances are honest about resolution.
//!
//! One concept per module:
//! * [`grid`]     — the uniform unit-square grid.
//! * [`poisson`]  — SOR solve of `∇²φ = rhs` (validated by a manufactured solution).
//! * [`cavity`]   — the lid-driven-cavity vorticity–streamfunction solver.
//! * [`solution`] — the flow field + Ghia-comparison diagnostics.

pub mod cavity;
pub mod grid;
pub mod poisson;
pub mod solution;

pub use cavity::{CavityConfig, solve_cavity};
pub use grid::Grid;
pub use poisson::{optimal_omega, sor_solve};
pub use solution::CavitySolution;
