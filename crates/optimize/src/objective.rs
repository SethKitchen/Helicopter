//! The optimization-problem boundary: a scalar objective over a real vector.
//!
//! Solvers depend on `&dyn Objective`, never a concrete problem — the polymorphism
//! boundary for this crate (cf. `Airfoil`, `ValidationCase`). A design problem
//! implements [`Objective`]; the optimizer ([`crate::nelder_mead::minimize`]) and the
//! constraint wrapper ([`crate::constraint::Penalized`]) work against the trait.

/// A scalar function to **minimize** over an `n`-dimensional real vector. Lower is
/// better. (Maximize by negating, or by minimizing a `−value` adapter.)
pub trait Objective {
    /// Number of design variables (the length every `x` slice must have).
    fn dim(&self) -> usize;

    /// Evaluate the objective at `x` (`x.len() == dim()`).
    fn value(&self, x: &[f64]) -> f64;

    /// Optional per-variable box bounds `[(lo, hi); dim]`; `None` = unbounded.
    /// The optimizer clamps trial points into these bounds before evaluating.
    fn bounds(&self) -> Option<&[(f64, f64)]> {
        None
    }
}

/// Adapt a plain closure into an [`Objective`], optionally with box bounds.
pub struct FnObjective<F: Fn(&[f64]) -> f64> {
    dim: usize,
    f: F,
    bounds: Option<Vec<(f64, f64)>>,
}

impl<F: Fn(&[f64]) -> f64> FnObjective<F> {
    /// An unbounded objective of dimension `dim`.
    pub fn new(dim: usize, f: F) -> Self {
        FnObjective {
            dim,
            f,
            bounds: None,
        }
    }

    /// A box-bounded objective; `bounds.len()` must equal `dim`.
    pub fn bounded(dim: usize, bounds: Vec<(f64, f64)>, f: F) -> Self {
        assert_eq!(bounds.len(), dim, "one (lo,hi) bound per variable");
        FnObjective {
            dim,
            f,
            bounds: Some(bounds),
        }
    }
}

impl<F: Fn(&[f64]) -> f64> Objective for FnObjective<F> {
    fn dim(&self) -> usize {
        self.dim
    }
    fn value(&self, x: &[f64]) -> f64 {
        (self.f)(x)
    }
    fn bounds(&self) -> Option<&[(f64, f64)]> {
        self.bounds.as_deref()
    }
}

/// Clamp `x` into `bounds` in place (no-op if `bounds` is `None`).
pub(crate) fn clamp_to_bounds(x: &mut [f64], bounds: Option<&[(f64, f64)]>) {
    if let Some(b) = bounds {
        for (xi, &(lo, hi)) in x.iter_mut().zip(b.iter()) {
            *xi = xi.clamp(lo, hi);
        }
    }
}
