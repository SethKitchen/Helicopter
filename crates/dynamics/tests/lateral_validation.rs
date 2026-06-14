//! Lateral-directional hover validation (the 5e-i oracle).
//!
//! Two-layer, like 5c. Derivative signs against textbook: Lp<0 (roll damping),
//! Nr<0 (yaw damping, from the tail rotor as a dynamic element), Yv<0 (side-force
//! damping). Eigenvalue structure against the Padfield/Prouty lateral-hover
//! description: an unstable mode + a roll-sideslip oscillation + a subsidence.
//! Analytic anchor: the reduced roll-sideslip cubic must match the eigen-routine.
//!
//! Tail-rotor *height* is included → the tail thrust makes a roll moment as well
//! as yaw, the source of the roll-yaw coupling (named decision).

use helisim_dynamics::{analyze_hover_lateral, analyze_hover_longitudinal, lateral_cubic, roots};
use helisim_trim::Aircraft;

const I_XX: f64 = 0.4; // roll inertia
const I_ZZ: f64 = 1.0; // yaw inertia
const I_YY: f64 = 0.8; // pitch inertia

#[test]
fn lateral_derivative_signs_match_theory() {
    let lat = analyze_hover_lateral(&Aircraft::demo(), I_XX, I_ZZ);
    let d = lat.derivatives;
    assert!(d.lp < 0.0, "Lp (roll damping) should be <0: {}", d.lp);
    assert!(d.nr < 0.0, "Nr (yaw damping, tail) should be <0: {}", d.nr);
    assert!(d.yv < 0.0, "Yv (side-force damping) should be <0: {}", d.yv);
}

#[test]
fn roll_damping_matches_pitch_damping() {
    // The main rotor is axisymmetric: its roll damping equals its pitch damping.
    let ac = Aircraft::demo();
    let lat = analyze_hover_lateral(&ac, I_XX, I_ZZ);
    let lon = analyze_hover_longitudinal(&ac, I_YY);
    // Lp = Mq + small tail term; both clearly negative and close.
    assert!((lat.derivatives.lp - lon.derivatives.mq).abs() < 0.02);
}

#[test]
fn lateral_hover_has_unstable_oscillation_and_subsidences() {
    // Post-5f (correct Lv=−Mu sign): the lateral hover instability is an
    // *oscillation* — a lateral phugoid mirroring the longitudinal one — plus
    // stable roll/yaw subsidences. (The pre-5f sign bug had made it look like an
    // aperiodic divergence.)
    let lat = analyze_hover_lateral(&Aircraft::demo(), I_XX, I_ZZ);
    assert!(
        lat.has_unstable_oscillation,
        "lateral hover should be oscillatory-unstable"
    );
    assert!(
        lat.modes.iter().any(|m| m.stable && !m.oscillatory),
        "should have a stable subsidence"
    );
    // No spurious aperiodic divergence.
    assert!(
        !lat.modes
            .iter()
            .any(|m| !m.oscillatory && m.eigenvalue.re > 0.05),
        "no real divergence after the sign fix"
    );
}

#[test]
fn eigenvalues_match_analytic_roll_sideslip_cubic() {
    // The new eigen-routine, validated against the reducible lateral cubic.
    let ac = Aircraft::demo();
    let lat = analyze_hover_lateral(&ac, I_XX, I_ZZ);
    let cubic = roots(&lateral_cubic(&lat.derivatives, ac.mass, I_XX));

    // The roll-sideslip oscillation matches the 4×4 oscillatory pair (the primary
    // anchor — the cubic drops yaw, so it captures the roll-sideslip oscillation
    // and the roll subsidence, but not the separate yaw subsidence).
    let full_osc = lat
        .modes
        .iter()
        .find(|m| m.oscillatory && m.eigenvalue.im > 0.0)
        .unwrap()
        .eigenvalue;
    let cub_osc = cubic
        .iter()
        .find(|r| r.im > 1e-6)
        .expect("cubic complex pair");
    assert!(
        (full_osc.re - cub_osc.re).abs() < 0.05,
        "osc re: {} vs {}",
        full_osc.re,
        cub_osc.re
    );
    assert!(
        (full_osc.im - cub_osc.im).abs() < 0.05,
        "osc im: {} vs {}",
        full_osc.im,
        cub_osc.im
    );

    // The cubic's real root matches the nearest 4×4 real mode (the roll subsidence).
    let cub_real = cubic
        .iter()
        .find(|r| r.im.abs() < 1e-6)
        .expect("cubic real root")
        .re;
    let nearest = lat
        .modes
        .iter()
        .filter(|m| !m.oscillatory)
        .map(|m| (m.eigenvalue.re - cub_real).abs())
        .fold(f64::MAX, f64::min);
    assert!(
        nearest < 0.1,
        "cubic real root {cub_real} not near any 4×4 real mode"
    );
}

#[test]
fn eigenvalues_are_conjugate_pairs() {
    let lat = analyze_hover_lateral(&Aircraft::demo(), I_XX, I_ZZ);
    let sum_im: f64 = lat.eigenvalues.iter().map(|e| e.im).sum();
    assert!(sum_im.abs() < 1e-6);
}
