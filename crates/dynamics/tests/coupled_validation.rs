//! Coupled 8-state validation (5e-ii): the decouple→known-pair / couple→new-modes
//! gate. With the cross-blocks zeroed the 8 eigenvalues must equal the union of
//! the independent 5c (longitudinal) and 5e-i (lateral) oracles; turning coupling
//! on shifts them into coupled modes neither 4-state model produces.

use helisim_dynamics::{
    Complex, Inertia, analyze_coupled_hover, analyze_hover_lateral, analyze_hover_longitudinal,
};
use helisim_trim::Aircraft;

fn inertia() -> Inertia {
    Inertia {
        mass: 8.0,
        i_xx: 0.4,
        i_yy: 0.8,
        i_zz: 1.0,
    }
}

/// Does `set` contain an eigenvalue within `tol` of `e`?
fn contains(set: &[Complex], e: Complex, tol: f64) -> bool {
    set.iter()
        .any(|c| (c.re - e.re).abs() < tol && (c.im - e.im).abs() < tol)
}

#[test]
fn decoupled_reproduces_both_oracles() {
    let ac = Aircraft::demo();
    let j = inertia();
    let lon = analyze_hover_longitudinal(&ac, j.i_yy);
    let lat = analyze_hover_lateral(&ac, j.i_xx, j.i_zz);
    let dec = analyze_coupled_hover(&ac, j, false);

    // Every 5c and 5e-i eigenvalue must appear in the decoupled 8-state set.
    for e in lon.eigenvalues.iter().chain(lat.eigenvalues.iter()) {
        assert!(
            contains(&dec.eigenvalues, *e, 1e-6),
            "decoupled set missing {:?}",
            e
        );
    }
    assert_eq!(dec.eigenvalues.len(), 8);
}

#[test]
fn coupling_shifts_the_modes() {
    let ac = Aircraft::demo();
    let j = inertia();
    let dec = analyze_coupled_hover(&ac, j, false);
    let cpl = analyze_coupled_hover(&ac, j, true);

    // At least one coupled eigenvalue is NOT in the decoupled set (new physics).
    let shifted = cpl
        .eigenvalues
        .iter()
        .any(|e| !contains(&dec.eigenvalues, *e, 1e-3));
    assert!(shifted, "coupling should shift modes");
}

#[test]
fn both_instabilities_persist_when_coupled() {
    // After the 5f sign fix the lateral instability is oscillatory (a lateral
    // phugoid), like the longitudinal one. Both survive coupling: the coupled
    // system has (at least) two unstable eigenvalues, and they are oscillatory.
    let cpl = analyze_coupled_hover(&Aircraft::demo(), inertia(), true);
    let n_unstable = cpl.eigenvalues.iter().filter(|e| e.re > 0.05).count();
    assert!(
        n_unstable >= 2,
        "expected ≥2 unstable eigenvalues, got {n_unstable}"
    );
    assert!(
        cpl.modes
            .iter()
            .any(|m| m.oscillatory && m.eigenvalue.re > 0.05),
        "unstable oscillatory mode(s) should persist"
    );
    // No unstable aperiodic divergence — that was the pre-5f sign artifact.
    assert!(
        !cpl.modes
            .iter()
            .any(|m| !m.oscillatory && m.eigenvalue.re > 0.05),
        "no spurious real divergence after the sign fix"
    );
}

#[test]
fn eigenvalues_are_conjugate_pairs() {
    let cpl = analyze_coupled_hover(&Aircraft::demo(), inertia(), true);
    let sum_im: f64 = cpl.eigenvalues.iter().map(|e| e.im).sum();
    assert!(sum_im.abs() < 1e-6, "imaginary parts should cancel");
}
