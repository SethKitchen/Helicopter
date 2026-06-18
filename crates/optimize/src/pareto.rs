//! Pareto non-dominated front for multi-objective minimization.
//!
//! With competing objectives (endurance vs noise vs cost vs flare margin) there is
//! no single optimum — there is a *front* of designs none of which can be improved
//! in one objective without giving up another. A scalarized weighted rank collapses
//! that front into one number and hides the trade; this returns the front itself so
//! the trade can be inspected.
//!
//! Convention: every objective is **minimized**. Point `a` *dominates* `b` iff `a`
//! is ≤ `b` in every objective and strictly `<` in at least one. The non-dominated
//! set is the Pareto front.

/// Does `a` dominate `b`? (Minimization: `a` no worse in all, strictly better in
/// one.) Equal points do NOT dominate each other — both stay on the front.
pub fn dominates(a: &[f64], b: &[f64]) -> bool {
    debug_assert_eq!(a.len(), b.len(), "objective vectors must match in length");
    let mut strictly_better_somewhere = false;
    for (&ai, &bi) in a.iter().zip(b.iter()) {
        if ai > bi {
            return false; // worse in some objective ⇒ cannot dominate
        }
        if ai < bi {
            strictly_better_somewhere = true;
        }
    }
    strictly_better_somewhere
}

/// Indices of the non-dominated points in `points` (each a vector of objective
/// values to minimize). Order of returned indices is ascending. O(N²·m) — fine for
/// the design grids here (hundreds of points).
pub fn pareto_front(points: &[Vec<f64>]) -> Vec<usize> {
    let mut front = Vec::new();
    for (i, pi) in points.iter().enumerate() {
        let dominated = points
            .iter()
            .enumerate()
            .any(|(j, pj)| j != i && dominates(pj, pi));
        if !dominated {
            front.push(i);
        }
    }
    front
}
