//! Rotor hub + blade grips — holds the blades and sets the head type.
//!
//! A 2-blade rotor uses a **teetering** head (one flap hinge through the shaft);
//! 3+ blades use an articulated/rigid head with a grip per blade. The grips clamp
//! the blade root, so they are sized from the blade root chord and thickness; the
//! central bore fits the mast.

use crate::part::{BuildPart, Source};

/// A hub / grip specification (dimensions in metres).
#[derive(Clone, Debug)]
pub struct HubSpec {
    /// Number of grips (= number of blades).
    pub n_grips: usize,
    /// Head type description.
    pub head_type: &'static str,
    /// Central bore to fit the mast, m.
    pub bore_m: f64,
    /// Grip length (root clamp), m.
    pub grip_length_m: f64,
    /// Grip width (≈ blade chord), m.
    pub grip_width_m: f64,
    /// Grip internal height (≈ blade root thickness + clamp), m.
    pub grip_height_m: f64,
    /// Hub disc diameter, m (≈ twice the root cutout radius).
    pub hub_diameter_m: f64,
}

/// Build a hub from the blade root geometry and the mast bore.
pub fn hub_from_blade(
    n_blades: usize,
    root_chord_m: f64,
    root_thickness_m: f64,
    root_cutout_radius_m: f64,
    mast_diameter_m: f64,
) -> HubSpec {
    let head_type = if n_blades == 2 {
        "teetering (single central flap hinge)"
    } else {
        "articulated / rigid (one grip + pitch bearing per blade)"
    };
    HubSpec {
        n_grips: n_blades,
        head_type,
        bore_m: mast_diameter_m,
        grip_length_m: 1.5 * root_chord_m,
        grip_width_m: root_chord_m,
        grip_height_m: root_thickness_m + 0.004, // + clamp/wall
        hub_diameter_m: 2.0 * root_cutout_radius_m,
    }
}

impl BuildPart for HubSpec {
    fn name(&self) -> &str {
        "rotor hub + blade grips"
    }
    fn material(&self) -> &str {
        "machined/printed 6061-T6 or 7075 aluminium; steel pitch bolts; bearings purchased"
    }
    fn source(&self) -> Source {
        Source::Assembled
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("hub diameter", self.hub_diameter_m * 1000.0),
            ("bore", self.bore_m * 1000.0),
            ("grip length", self.grip_length_m * 1000.0),
            ("grip width", self.grip_width_m * 1000.0),
            ("grip height", self.grip_height_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        vec![
            format!("1. Head type: {} for {} blades.", self.head_type, self.n_grips),
            format!(
                "2. Machine the central hub disc Ø{:.0} mm with a Ø{:.1} mm bore for the mast \
                 (interference/keyed fit).",
                self.hub_diameter_m * 1000.0,
                self.bore_m * 1000.0
            ),
            format!(
                "3. Make {} grip(s): {:.0} × {:.0} × {:.1} mm pocket to clamp each blade root.",
                self.n_grips,
                self.grip_length_m * 1000.0,
                self.grip_width_m * 1000.0,
                self.grip_height_m * 1000.0
            ),
            "4. Press in the pitch (and flap, if teetering) bearings; fit the pitch horns."
                .to_string(),
            "5. Check each grip's pitch axis is concentric and free; set zero pitch.".to_string(),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn two_blades_is_teetering_three_is_articulated() {
        let t = hub_from_blade(2, 0.04, 0.005, 0.09, 0.006);
        let a = hub_from_blade(3, 0.04, 0.005, 0.09, 0.006);
        assert!(t.head_type.contains("teetering"));
        assert!(a.head_type.contains("articulated"));
        assert_eq!(a.n_grips, 3);
    }

    #[test]
    fn grips_scale_with_blade_root_and_bore_matches_mast() {
        let h = hub_from_blade(2, 0.05, 0.006, 0.10, 0.008);
        assert!((h.grip_width_m - 0.05).abs() < 1e-12);
        assert!((h.grip_length_m - 1.5 * 0.05).abs() < 1e-12);
        assert!((h.bore_m - 0.008).abs() < 1e-12);
        assert!((h.hub_diameter_m - 0.20).abs() < 1e-12);
    }
}
