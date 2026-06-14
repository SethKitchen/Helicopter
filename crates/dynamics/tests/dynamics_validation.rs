//! Flight-dynamics validation.
//!
//! Headline (structural oracle, Padfield/Prouty): the hover longitudinal modes
//! must show the famous signature — stable subsidences plus an **unstable
//! low-frequency oscillatory mode** (the pitch–speed instability). A hovering
//! helicopter is open-loop unstable; reproducing that without putting it there is
//! the milestone landing.
//!
//! Cross-checks:
//! * Derivative signs (independent, force/moment-based — NOT power): `Mu > 0`
//!   (destabilizing), `Mq < 0` (pitch damping), `Zw < 0` (heave damping),
//!   `Xu < 0` (drag damping). The `Mu>0` + weak damping combination *is* what
//!   produces the instability.
//! * Analytic anchor for the new eigenvalue routine: the general solver's
//!   oscillatory eigenvalues match the roots of the hand-formed hovering cubic.

use helisim_dynamics::{analyze_hover_longitudinal, hovering_cubic, roots};
use helisim_trim::Aircraft;

const I_YY: f64 = 0.8; // pitch inertia of the demo aircraft, kg·m²

#[test]
fn derivative_signs_match_theory() {
    let a = analyze_hover_longitudinal(&Aircraft::demo(), I_YY);
    let d = a.derivatives;
    assert!(
        d.mu > 0.0,
        "Mu should be >0 (destabilizing speed stability): {}",
        d.mu
    );
    assert!(d.mq < 0.0, "Mq should be <0 (pitch damping): {}", d.mq);
    assert!(d.zw < 0.0, "Zw should be <0 (heave damping): {}", d.zw);
    assert!(d.xu < 0.0, "Xu should be <0 (drag damping): {}", d.xu);
}

#[test]
fn hover_has_the_unstable_oscillatory_signature() {
    let a = analyze_hover_longitudinal(&Aircraft::demo(), I_YY);
    // The headline: an unstable oscillatory mode exists.
    assert!(
        a.has_unstable_oscillation,
        "hover must have an unstable oscillatory mode"
    );

    // It is low-frequency / long-period (slow), as for the real hovering cubic.
    let osc = a
        .modes
        .iter()
        .find(|m| m.oscillatory && m.eigenvalue.re > 0.0)
        .unwrap();
    assert!(
        osc.period > 2.0,
        "unstable oscillation should be slow (period {:.1}s)",
        osc.period
    );

    // And there are stable (subsidence) modes too — not everything is unstable.
    assert!(
        a.modes.iter().any(|m| m.stable),
        "should also have stable subsidence modes"
    );
    // Heave subsidence (from Zw) should be clearly stable and reasonably fast.
    assert!(
        a.modes
            .iter()
            .any(|m| m.stable && !m.oscillatory && m.eigenvalue.re < -0.3),
        "expected a stable real subsidence (heave/Zw)"
    );
}

#[test]
fn eigenvalue_routine_matches_analytic_hovering_cubic() {
    // The new eigenvalue primitive, validated against the analytically-formed
    // longitudinal cubic: the 4×4's oscillatory pair must equal the cubic's
    // complex roots.
    let ac = Aircraft::demo();
    let a = analyze_hover_longitudinal(&ac, I_YY);
    let cubic = hovering_cubic(&a.derivatives, ac.mass, I_YY);
    let cubic_roots = roots(&cubic);

    let full_osc = a
        .modes
        .iter()
        .find(|m| m.oscillatory && m.eigenvalue.im > 0.0)
        .unwrap()
        .eigenvalue;
    let cubic_osc = cubic_roots
        .iter()
        .find(|r| r.im > 1e-6)
        .expect("hovering cubic should have a complex pair");

    assert!(
        (full_osc.re - cubic_osc.re).abs() < 0.05,
        "real parts: {} vs {}",
        full_osc.re,
        cubic_osc.re
    );
    assert!(
        (full_osc.im - cubic_osc.im).abs() < 0.05,
        "imag parts: {} vs {}",
        full_osc.im,
        cubic_osc.im
    );
}

#[test]
fn eigenvalues_come_in_conjugate_pairs() {
    // A real system matrix → complex eigenvalues in conjugate pairs.
    let a = analyze_hover_longitudinal(&Aircraft::demo(), I_YY);
    let sum_im: f64 = a.eigenvalues.iter().map(|e| e.im).sum();
    assert!(
        sum_im.abs() < 1e-6,
        "imaginary parts should cancel (conjugate pairs)"
    );
}

#[test]
fn char_poly_eigensolver_and_empty_matrix() {
    use helisim_dynamics::{char_poly, eigenvalues, eigenvalues_via_char_poly};
    // The characteristic-polynomial route (kept for small analytic anchors):
    // [[0,-1],[1,0]] has eigenvalues ±i.
    let a = vec![vec![0.0, -1.0], vec![1.0, 0.0]];
    let ev = eigenvalues_via_char_poly(&a);
    assert_eq!(ev.len(), 2);
    assert!(ev.iter().all(|e| e.re.abs() < 1e-6 && (e.im.abs() - 1.0).abs() < 1e-6));
    // char_poly of a 2×2 is degree-2: λ² − tr·λ + det = λ² + 1 here.
    let p = char_poly(&a);
    assert_eq!(p.len(), 3);
    // The empty-matrix guard returns no eigenvalues.
    assert!(eigenvalues(&[]).is_empty());
    let _ = roots(&[1.0, 0.0, 1.0]); // also exercise roots directly
}
