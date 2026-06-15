//! Landing gear (skid type) — sized from the landing impact load.
//!
//! A model/light helicopter lands on tubular **skids** carried on struts. The
//! struts are sized from a hard-landing load `n_g · W` shared across the struts:
//! each strut is a cantilever taking its share as a lateral/bending load, and its
//! diameter follows the bending-stress rule `d = (32 M / π σ)^{1/3}` (same form as
//! the tail boom). Track and skid length come from the fuselage footprint plus a
//! stability margin, so the CG sits well inside the skid base.

use crate::part::{BuildPart, Source};
use std::f64::consts::PI;

/// Hard-landing vertical load factor (g) the gear is sized to absorb. ~3 g is a
/// conventional energy-absorption design point for light rotorcraft skids.
pub const LANDING_LOAD_FACTOR: f64 = 3.0;

/// A skid-type landing gear.
#[derive(Clone, Debug)]
pub struct LandingGearSpec {
    /// Skid tube length (fore-aft), m.
    pub skid_length_m: f64,
    /// Track (lateral skid-to-skid spacing), m.
    pub track_m: f64,
    /// Gear height (skid to fuselage hardpoint), m.
    pub height_m: f64,
    /// Strut / skid tube outer diameter, m (bending-sized).
    pub strut_diameter_m: f64,
    /// Number of struts.
    pub n_struts: usize,
    /// Total design landing load, N (`n_g · W`).
    pub landing_load_n: f64,
}

/// Size skid gear for a gross mass and fuselage footprint. `sigma_allow_pa` is the
/// strut material working stress (defaults to the Al value if you pass it; printed
/// nylon would pass a lower one).
pub fn landing_gear_for(
    gross_mass_kg: f64,
    fuselage_length_m: f64,
    fuselage_width_m: f64,
    sigma_allow_pa: f64,
) -> LandingGearSpec {
    let weight = gross_mass_kg * 9.80665;
    let landing_load = LANDING_LOAD_FACTOR * weight;
    let n_struts = 4;
    let height = (0.10 + 0.18 * fuselage_width_m).max(0.06);

    // Each strut carries its share as a cantilever of length = height; bending
    // moment M = (load/n) · height → d = (32 M / π σ)^{1/3}.
    let per_strut = landing_load / n_struts as f64;
    let moment = per_strut * height;
    let d = (32.0 * moment / (PI * sigma_allow_pa)).cbrt().max(0.003);

    LandingGearSpec {
        skid_length_m: (1.1 * fuselage_length_m).max(0.30),
        track_m: (1.6 * fuselage_width_m).max(0.20),
        height_m: height,
        strut_diameter_m: d,
        n_struts,
        landing_load_n: landing_load,
    }
}

impl BuildPart for LandingGearSpec {
    fn name(&self) -> &str {
        "landing gear (skids)"
    }
    fn material(&self) -> &str {
        "tubular struts + skids (printed nylon/CF or bent Al tube); energy-absorbing"
    }
    fn source(&self) -> Source {
        Source::Fabricated
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("skid length", self.skid_length_m * 1000.0),
            ("track", self.track_m * 1000.0),
            ("height", self.height_m * 1000.0),
            ("strut Ø", self.strut_diameter_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        vec![
            format!(
                "1. Make two skid tubes {:.0} mm long and {} struts Ø {:.1} mm, height {:.0} mm.",
                self.skid_length_m * 1000.0,
                self.n_struts,
                self.strut_diameter_m * 1000.0,
                self.height_m * 1000.0
            ),
            format!(
                "2. Set the track to {:.0} mm (CG must sit well inside the skid base).",
                self.track_m * 1000.0
            ),
            format!(
                "3. Size for a {:.0} N hard landing ({:.0} g) — let the struts flex to absorb energy, not snap.",
                self.landing_load_n, LANDING_LOAD_FACTOR
            ),
            "4. Bolt the struts to the fuselage skid hardpoints (bulkhead) and to the skids."
                .to_string(),
        ]
    }
    /// Skid gear envelope: skid length × track × height.
    fn bounding_box_mm(&self) -> (f64, f64, f64) {
        let mut a = [
            self.skid_length_m * 1000.0,
            self.track_m * 1000.0,
            self.height_m * 1000.0,
        ];
        a.sort_by(|p, q| q.total_cmp(p));
        (a[0], a[1], a[2])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::materials::SIGMA_ALLOW_AL;

    #[test]
    fn gear_grows_and_track_exceeds_width() {
        let g = landing_gear_for(3.5, 0.40, 0.18, SIGMA_ALLOW_AL);
        // Track wider than the fuselage (stability); positive sizing.
        assert!(g.track_m > 0.18);
        assert!(g.strut_diameter_m > 0.0);
        assert!((g.landing_load_n - 3.0 * 3.5 * 9.80665).abs() < 1e-6);
    }

    /// Strut diameter grows with landing load (heavier aircraft → thicker struts).
    #[test]
    fn strut_thickens_with_mass() {
        let small = landing_gear_for(3.5, 0.4, 0.18, SIGMA_ALLOW_AL);
        let big = landing_gear_for(700.0, 2.0, 1.0, SIGMA_ALLOW_AL);
        assert!(big.strut_diameter_m > small.strut_diameter_m);
    }

    #[test]
    fn bounding_box_is_skid_envelope() {
        let g = landing_gear_for(3.5, 0.4, 0.18, SIGMA_ALLOW_AL);
        let (l, _, _) = g.bounding_box_mm();
        assert!((l - g.skid_length_m * 1000.0).abs() < 1e-6);
    }
}
