//! **FEA** — a minimal, std-only finite-element solver for the structural check.
//!
//! Upgrades the manufacture crate's single-section stress estimates to a real
//! field solution: a 1-D Euler-Bernoulli beam FEM that returns the deflection and
//! the bending-stress distribution along a part. The parts that matter are beams —
//! the tail boom (cantilever under tail thrust), the mast, and the rotor blade in
//! flap — so a beam solver is exactly the right tool, and it composes with the
//! existing closed-form checks as an *independent route* (FE vs beam theory).
//!
//! # Scope (deliberately bounded)
//! * 1-D Euler-Bernoulli beam elements (bending). Axial/torsion and 2-D/3-D shell
//!   FE are named, deferred — the rotor's structural drivers are bending and the
//!   (separately computed) centrifugal tension.
//! * Linear, static. No buckling, no geometric (tension) stiffening yet.
//!
//! # Validation
//! Cubic beam elements are *exact* for point loads, so the FE deflection equals
//! beam theory to machine precision (cantilever `PL³/3EI`, simply-supported
//! `PL³/48EI`); distributed loads converge with refinement (`qL⁴/8EI`). Those
//! closed forms are the oracle — a genuinely independent route from the algebraic
//! `M/Z` sizing it upgrades.
//!
//! One concept per module:
//! * [`linsolve`] — dense `Ax=b` (Gaussian elimination, partial pivoting).
//! * [`beam`]     — the beam model, assembly, solve, stress recovery.

pub mod beam;
pub mod cst;
pub mod linsolve;

pub use beam::{uniform_beam, Bc, Beam, BeamSolution, NodalLoad};
pub use cst::{rectangle_two_tris, Cst, CstSolution};
pub use linsolve::solve;
