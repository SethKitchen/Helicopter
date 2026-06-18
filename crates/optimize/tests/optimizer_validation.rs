//! The optimizer validated against problems with KNOWN optima — the oracle for a
//! solver is "does it find the answer we can compute by hand?". Without this, the
//! design recommender has no evidence it finds *an* optimum at all.

use helisim_optimize::{ConstraintFn, FnObjective, NmOptions, Penalized, minimize};

/// Sphere: f = Σ xᵢ², global min at the origin, value 0. The trivial sanity anchor.
#[test]
fn sphere_minimizes_to_origin() {
    let obj = FnObjective::new(3, |x| x.iter().map(|v| v * v).sum());
    let r = minimize(&obj, &[3.0, -2.0, 1.5], &NmOptions::default());
    assert!(r.converged, "should converge well inside max_iter");
    assert!(r.value < 1e-8, "sphere value {} not ≈0", r.value);
    for &xi in &r.x {
        assert!(xi.abs() < 1e-4, "coordinate {xi} not ≈0");
    }
}

/// Rosenbrock: f = (a−x)² + b(y−x²)², a=1, b=100. Global min at (1,1), value 0.
/// The classic curved, ill-conditioned valley — a real test of the simplex, not a
/// bowl. (Nelder–Mead is known to solve it; this gates that our implementation does.)
#[test]
fn rosenbrock_finds_the_banana_valley_minimum() {
    let obj = FnObjective::new(2, |x| {
        let (a, b) = (1.0, 100.0);
        let (px, py) = (x[0], x[1]);
        (a - px).powi(2) + b * (py - px * px).powi(2)
    });
    let r = minimize(&obj, &[-1.2, 1.0], &NmOptions::default());
    assert!(r.value < 1e-6, "Rosenbrock value {} not ≈0", r.value);
    assert!((r.x[0] - 1.0).abs() < 1e-2, "x={} not ≈1", r.x[0]);
    assert!((r.x[1] - 1.0).abs() < 1e-2, "y={} not ≈1", r.x[1]);
}

/// Box bounds active at the optimum: minimize (x−5)² on x∈[0,2]. The unconstrained
/// min (x=5) is outside the box, so the constrained optimum sits ON the boundary at
/// x=2 — gates that clamping honours an active bound rather than drifting past it.
#[test]
fn active_box_bound_is_respected() {
    let obj = FnObjective::bounded(1, vec![(0.0, 2.0)], |x| (x[0] - 5.0).powi(2));
    let r = minimize(&obj, &[0.5], &NmOptions::default());
    assert!(
        (r.x[0] - 2.0).abs() < 1e-6,
        "should sit on the bound x=2, got {}",
        r.x[0]
    );
}

/// Penalty-constrained problem with an ANALYTIC (KKT) solution: minimize x²+y²
/// subject to x+y ≥ 2 (i.e. g = 2−x−y ≤ 0). Lagrange/symmetry ⇒ optimum (1,1),
/// value 2, constraint active. Validates the [`Penalized`] wrapper recovers it.
#[test]
fn linearly_constrained_quadratic_hits_kkt_point() {
    let base = FnObjective::new(2, |x| x[0] * x[0] + x[1] * x[1]);
    let constraints: Vec<ConstraintFn> = vec![Box::new(|x: &[f64]| 2.0 - x[0] - x[1])]; // g ≤ 0
    let pen = Penalized::new(&base, &constraints, 1.0e5);

    let r = minimize(&pen, &[3.0, -1.0], &NmOptions::default());
    assert!((r.x[0] - 1.0).abs() < 1e-2, "x={} not ≈1", r.x[0]);
    assert!((r.x[1] - 1.0).abs() < 1e-2, "y={} not ≈1", r.x[1]);
    // Constraint satisfied to engineering tolerance (exterior penalty ⇒ a hair inside).
    assert!(pen.violation(&r.x) < 1e-2, "constraint not ≈satisfied");
    // True objective (not the penalized value) at the recovered point ≈ 2.
    assert!((r.x[0] * r.x[0] + r.x[1] * r.x[1] - 2.0).abs() < 5e-2);
}
