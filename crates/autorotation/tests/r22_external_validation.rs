//! External validation of the autorotation crate against published **Robinson
//! R22** autorotation reference speeds.
//!
//! Predictions and the (assumption-laden) parameter mapping were LOCKED before the
//! oracle was sourced — see `validation/AUTOROTATION_EXTERNAL_PREREG.md` and the
//! results in `validation/AUTOROTATION_EXTERNAL_RESULTS.md`.
//!
//! Oracle (Robinson R22 POH / type literature):
//!   * best glide (max distance): **75 KIAS**, glide ratio ≈ **4:1**
//!   * minimum rate of descent:   **53 KIAS**
//!
//! Source: Robinson R22 Pilot's Operating Handbook normal procedures + widely
//! republished R22 autorotation reference speeds
//! (robinsonstrapistorprod.blob.core.windows.net/.../r22_poh_4_98ca296aca.pdf;
//! corroborated by multiple R22 maneuver guides). Cited, not fabricated.
//!
//! Two of the input parameters (mean C_d0, flat-plate area f) are *assumptions*,
//! not sourced — so the clean, force/kinematics-based claims (speed ORDERING,
//! forward < vertical) are asserted tightly, while the power-derived magnitudes
//! (which carry the project's standing power-calibration caveat) are asserted only
//! to the pre-registered bands, with the error in the pre-registered direction.

use helisim_autorotation::{glide_polar, profile_power, steady_autorotation, G};
use std::f64::consts::PI;

const KT_TO_MS: f64 = 0.514444;

/// R22-like parameter mapping (locked in the prereg).
fn r22_polar() -> (helisim_autorotation::GlidePolar, f64) {
    let mass = 621.0_f64; // kg, ~1370 lb MGTOW
    let r = 3.84_f64; // m
    let rho = 1.225_f64;
    let vtip = 213.0_f64; // m/s (~530 rpm)
    let sigma = 0.060_f64;
    let cd0 = 0.010_f64; // assumed
    let f = 0.50_f64; // m², assumed
    let area = PI * r * r;
    let w = mass * G;
    let p0 = profile_power(rho, area, vtip, sigma, cd0);
    let vertical = steady_autorotation(w, rho, area, p0).descent_rate_ms;
    let speeds: Vec<f64> = (1..=140).map(|i| i as f64 * 0.5).collect();
    (glide_polar(w, rho, area, p0, f, &speeds), vertical)
}

#[test]
fn clean_claims_speed_ordering_and_forward_below_vertical() {
    // These need no calibrated input — a failure here would be a real bug.
    let (polar, vertical) = r22_polar();
    assert!(
        polar.best_glide.airspeed_ms > polar.min_sink.airspeed_ms,
        "best-glide speed must exceed min-sink speed (R22: 75 vs 53 KIAS)"
    );
    assert!(
        polar.min_sink.descent_rate_ms < vertical,
        "forward autorotation must descend slower than vertical"
    );
}

#[test]
fn min_sink_speed_matches_published_within_band() {
    // Published min-rate-of-descent speed 53 KIAS. Predicted ≈48 kt (−9%).
    let (polar, _) = r22_polar();
    let published = 53.0 * KT_TO_MS;
    let err = (polar.min_sink.airspeed_ms - published).abs() / published;
    assert!(err < 0.25, "min-sink speed err {:.0}% > 25%", err * 100.0);
}

#[test]
fn best_glide_speed_over_predicted_as_pre_registered() {
    // Published best-glide 75 KIAS. Model over-predicts (predicted DIRECTION):
    // the assumed flat-plate area pushes best glide fast. Within the band.
    let (polar, _) = r22_polar();
    let published = 75.0 * KT_TO_MS;
    assert!(
        polar.best_glide.airspeed_ms > published,
        "model best-glide should over-predict (pre-registered)"
    );
    let err = (polar.best_glide.airspeed_ms - published) / published;
    assert!(err < 0.30, "best-glide speed err {:.0}% > 30%", err * 100.0);
}

#[test]
fn best_glide_ratio_matches_published_within_band() {
    // Published glide ratio ≈4:1 (1 nm per 1500 ft). Glide ratio = 1/tan(angle).
    // A RATIO is relatively insensitive to absolute power calibration.
    let (polar, _) = r22_polar();
    let ratio = 1.0 / polar.best_glide.glide_angle_deg.to_radians().tan();
    let published = 4.0;
    let err = (ratio - published).abs() / published;
    assert!(err < 0.25, "glide ratio {ratio:.2}:1 vs 4:1, err {:.0}% > 25%", err * 100.0);
}
