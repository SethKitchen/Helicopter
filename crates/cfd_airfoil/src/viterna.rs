//! **Viterna–Corrigan post-stall extrapolation** — the standard way (used by AeroDyn
//! and most rotor codes) to extend a polar that is only known in the attached regime
//! out to deep stall, where the section behaves like a flat plate. A CFD or wind-tunnel
//! polar covers a few degrees around zero; a rotor blade — especially the inboard and
//! reverse-flow regions — sees the whole range, so the table must be completed.
//!
//! Given a stall anchor `(α_s, Cl_s, Cd_s)` and the deep-stall drag `Cd_max`, for
//! `α_s ≤ α ≤ 90°`:
//!
//! ```text
//! Cl(α) = (Cd_max/2)·sin 2α + A₂·cos²α/sin α
//! Cd(α) =  Cd_max·sin²α    + B₂·cos α
//! A₂ = (Cl_s − Cd_max·sin α_s·cos α_s)·sin α_s/cos²α_s
//! B₂ = (Cd_s − Cd_max·sin²α_s)/cos α_s
//! ```
//!
//! constructed to pass through the stall anchor and reach the flat-plate limits at
//! 90° (`Cl→0`, `Cd→Cd_max`). For a 2-D section `Cd_max ≈ 2.0` (a flat plate normal
//! to the flow). (Viterna & Corrigan, "Fixed Pitch Rotor Performance of Large HAWTs",
//! NASA/DOE, 1982.)

/// Lift and drag at post-stall angle `alpha` (radians, `α_stall ≤ α ≤ π/2`) from the
/// Viterna model anchored at `(alpha_stall, cl_stall, cd_stall)` with deep-stall drag
/// `cd_max`.
pub fn post_stall(
    alpha: f64,
    alpha_stall: f64,
    cl_stall: f64,
    cd_stall: f64,
    cd_max: f64,
) -> (f64, f64) {
    let (ss, cs) = (alpha_stall.sin(), alpha_stall.cos());
    let a2 = (cl_stall - cd_max * ss * cs) * ss / (cs * cs);
    let b2 = (cd_stall - cd_max * ss * ss) / cs;
    let (s, c) = (alpha.sin(), alpha.cos());
    let cl = 0.5 * cd_max * (2.0 * alpha).sin() + a2 * c * c / s;
    let cd = cd_max * s * s + b2 * c;
    (cl, cd)
}

/// Deep-stall drag for an aspect ratio (Viterna): `1.11 + 0.018·AR`, capped at the
/// 2-D flat-plate value `≈ 2.01` for high `AR`.
pub fn cd_max_for_aspect_ratio(ar: f64) -> f64 {
    if ar > 50.0 { 2.01 } else { 1.11 + 0.018 * ar }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn matches_the_stall_anchor_and_flat_plate_limits() {
        let (a_s, cl_s, cd_s, cd_max) = (14f64.to_radians(), 1.0, 0.05, 2.0);
        // Continuity: the model passes through the anchor at α_stall.
        let (cl0, cd0) = post_stall(a_s, a_s, cl_s, cd_s, cd_max);
        assert!(
            (cl0 - cl_s).abs() < 1e-9 && (cd0 - cd_s).abs() < 1e-9,
            "anchor continuity"
        );
        // Flat-plate limit at 90°: Cl→0, Cd→Cd_max.
        let (cl90, cd90) = post_stall(PI / 2.0 - 1e-9, a_s, cl_s, cd_s, cd_max);
        assert!(cl90.abs() < 1e-3, "Cl(90°) ≈ 0 (got {cl90})");
        assert!(
            (cd90 - cd_max).abs() < 1e-3,
            "Cd(90°) ≈ Cd_max (got {cd90})"
        );
        // Drag rises monotonically through the post-stall region.
        let (_, cd45) = post_stall(45f64.to_radians(), a_s, cl_s, cd_s, cd_max);
        assert!(
            cd_s < cd45 && cd45 < cd_max,
            "Cd monotone {cd_s} < {cd45} < {cd_max}"
        );
    }

    #[test]
    fn cd_max_caps_at_the_flat_plate_value() {
        assert!((cd_max_for_aspect_ratio(10.0) - 1.29).abs() < 1e-9);
        assert_eq!(cd_max_for_aspect_ratio(1000.0), 2.01);
    }
}
