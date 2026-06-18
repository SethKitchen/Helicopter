//! The Pareto front validated against analytically-known dominance.

use helisim_optimize::{dominates, pareto_front};

/// Hand-computable dominance: with [1,1] present, every other listed point is
/// dominated by it (≤ in both objectives, strictly < in at least one), so the front
/// is exactly {[1,1]}. A falsifiable oracle — the indices are computed by hand.
#[test]
fn front_is_the_single_dominating_point() {
    let pts = vec![
        vec![1.0, 2.0], // dominated by [1,1] (equal x, worse y)
        vec![2.0, 1.0], // dominated by [1,1]
        vec![1.5, 1.5], // dominated by [1,1]
        vec![3.0, 3.0], // dominated by everything
        vec![1.0, 1.0], // dominates all the above
    ];
    assert_eq!(pareto_front(&pts), vec![4]);
}

/// A convex trade curve sampled on f1=x², f2=(x−2)² for x∈[0,2]: reducing one
/// objective always raises the other, so NO sampled point dominates another — the
/// entire set is non-dominated. Adding one clearly-bad point must be excluded.
#[test]
fn convex_tradeoff_keeps_all_then_drops_a_dominated_point() {
    let n = 21;
    let mut pts: Vec<Vec<f64>> = (0..n)
        .map(|i| {
            let x = 2.0 * i as f64 / (n as f64 - 1.0);
            vec![x * x, (x - 2.0).powi(2)]
        })
        .collect();
    assert_eq!(
        pareto_front(&pts).len(),
        n,
        "every trade-curve point is non-dominated"
    );

    // A point worse than the whole curve in both objectives must be dropped.
    pts.push(vec![100.0, 100.0]);
    let front = pareto_front(&pts);
    assert_eq!(front.len(), n, "the dominated point is excluded");
    assert!(
        !front.contains(&n),
        "index of the bad point not on the front"
    );
}

/// Ties and the dominance relation itself.
#[test]
fn dominance_relation_edge_cases() {
    assert!(
        dominates(&[1.0, 1.0], &[1.0, 2.0]),
        "equal in one, better in other"
    );
    assert!(dominates(&[1.0, 1.0], &[2.0, 2.0]), "better in both");
    assert!(
        !dominates(&[1.0, 2.0], &[2.0, 1.0]),
        "neither dominates (trade)"
    );
    assert!(
        !dominates(&[1.0, 1.0], &[1.0, 1.0]),
        "equal points do not dominate"
    );
    // Duplicates: both copies survive on the front (weak Pareto).
    let dup = vec![vec![1.0, 1.0], vec![1.0, 1.0]];
    assert_eq!(pareto_front(&dup), vec![0, 1]);
}
