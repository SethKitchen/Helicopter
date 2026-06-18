//! Inequality constraints by the (exterior) penalty method.
//!
//! Each constraint is a function `g(x)` interpreted as `g(x) ≤ 0` (feasible when
//! non-positive). [`Penalized`] wraps a base [`Objective`] and adds
//! `weight · Σ max(0, g_i(x))²` — a smooth exterior penalty that pushes the
//! minimizer back across a violated boundary. The result implements [`Objective`],
//! so the unconstrained simplex solver handles a constrained problem unchanged.
//!
//! Penalty (not a hard barrier) is the right tool here: our design constraints
//! (flare margin, envelope, weight closure) are soft engineering floors where a
//! small, reported violation is informative, and a large `weight` recovers the
//! constrained optimum to engineering tolerance on the smooth problems we pose.

use crate::objective::Objective;

/// One inequality constraint `g(x)`, interpreted as `g(x) ≤ 0` (feasible when
/// non-positive). Boxed so a heterogeneous list of constraints can be held.
pub type ConstraintFn = Box<dyn Fn(&[f64]) -> f64>;

/// A base objective augmented with inequality constraints `g_i(x) ≤ 0`.
pub struct Penalized<'a> {
    base: &'a dyn Objective,
    constraints: &'a [ConstraintFn],
    weight: f64,
}

impl<'a> Penalized<'a> {
    /// Wrap `base` with `constraints` (each `g_i(x) ≤ 0`) and a penalty `weight`.
    pub fn new(base: &'a dyn Objective, constraints: &'a [ConstraintFn], weight: f64) -> Self {
        Penalized {
            base,
            constraints,
            weight,
        }
    }

    /// Total constraint violation `Σ max(0, g_i(x))` at `x` (0 ⇔ feasible).
    pub fn violation(&self, x: &[f64]) -> f64 {
        self.constraints.iter().map(|g| g(x).max(0.0)).sum()
    }
}

impl Objective for Penalized<'_> {
    fn dim(&self) -> usize {
        self.base.dim()
    }

    fn value(&self, x: &[f64]) -> f64 {
        let penalty: f64 = self
            .constraints
            .iter()
            .map(|g| {
                let v = g(x).max(0.0);
                v * v
            })
            .sum();
        self.base.value(x) + self.weight * penalty
    }

    fn bounds(&self) -> Option<&[(f64, f64)]> {
        self.base.bounds()
    }
}
