//! NACA 0012 cross-section properties — the single source for the blade-section
//! geometry constants and integrals, so they can't drift between the structural
//! check, the joint-structural coupling, and the FEA.
//!
//! All are functions of the chord `c` (m): area `A ≈ 0.0822 c²`, max thickness
//! `t = 0.12 c`, and the flapwise second moment integrated from the real section.

use crate::airfoil_coords::naca4_half_thickness;

/// NACA 0012 cross-section area coefficient: `A ≈ AREA_COEFF · c²`.
pub const AREA_COEFF: f64 = 0.0822;
/// Max thickness as a fraction of chord (the "12" in 0012).
pub const MAX_THICKNESS_RATIO: f64 = 0.12;
/// Fraction of the section area treated as load-bearing in the **conservative**
/// structural-margin checks (the rest is non-structural skin/fill). The
/// material-comparison analysis in `actuation` deliberately uses the *full* area
/// (`structural_fraction = 1`) since it compares ratios, not absolute margins.
pub const STRUCTURAL_FRACTION: f64 = 0.5;

/// Full section area, m².
pub fn section_area(chord: f64) -> f64 {
    AREA_COEFF * chord * chord
}

/// Conservative load-bearing area for the structural-margin checks, m².
pub fn structural_area(chord: f64) -> f64 {
    STRUCTURAL_FRACTION * section_area(chord)
}

/// Max section thickness, m.
pub fn max_thickness(chord: f64) -> f64 {
    MAX_THICKNESS_RATIO * chord
}

/// NACA 0012 flapwise second moment of area about the chord line, m⁴, integrated
/// from the real section (`I = ∫ (2/3) y_t³ dx` per chordwise strip) — the
/// accurate value, well below the thin-rectangle `c·t³/12` over-estimate.
pub fn flap_inertia(chord: f64) -> f64 {
    let n = 400;
    let mut i = 0.0;
    let dx = chord / n as f64;
    for k in 0..n {
        let x = (k as f64 + 0.5) * dx;
        let yt = naca4_half_thickness(MAX_THICKNESS_RATIO, x / chord) * chord;
        i += (2.0 / 3.0) * yt.powi(3) * dx;
    }
    i
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Both section quantities scale as `c²` / `c⁴` respectively — the geometric
    /// oracle a reader can check.
    #[test]
    fn section_scaling() {
        assert!((section_area(0.08) / section_area(0.04) - 4.0).abs() < 1e-12);
        assert!((flap_inertia(0.08) / flap_inertia(0.04) - 16.0).abs() < 1e-6);
        assert!((max_thickness(0.05) - 0.006).abs() < 1e-12);
        // The integrated flap inertia is far below the thin-rectangle estimate.
        let c = 0.05;
        let rect = c * max_thickness(c).powi(3) / 12.0;
        assert!(flap_inertia(c) < 0.6 * rect, "NACA inertia ≪ rectangle");
    }
}
