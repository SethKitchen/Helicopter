//! Swashplate — converts non-rotating control inputs into rotating blade pitch.
//!
//! Two plates separated by a bearing: a non-rotating lower plate tilted/raised by
//! the servos, and a rotating upper plate driven with the mast that feeds the
//! pitch links. Sized proportionally to the rotor (outer diameter ~ a small
//! fraction of rotor radius) with the central bore on the mast.

use crate::part::{BuildPart, Source};

/// A swashplate specification (metres).
#[derive(Clone, Debug)]
pub struct SwashplateSpec {
    /// Outer diameter, m.
    pub outer_diameter_m: f64,
    /// Central bore (sliding on the mast), m.
    pub bore_m: f64,
    /// Number of pitch links (= number of blades).
    pub n_links: usize,
    /// Number of control inputs (collective + cyclic servos).
    pub n_servo_inputs: usize,
}

/// Size a swashplate from the rotor radius and mast diameter.
pub fn swashplate_for(rotor_radius_m: f64, mast_diameter_m: f64, n_blades: usize) -> SwashplateSpec {
    SwashplateSpec {
        outer_diameter_m: (0.15 * rotor_radius_m).max(2.5 * mast_diameter_m),
        bore_m: mast_diameter_m,
        n_links: n_blades,
        n_servo_inputs: 3, // 120° CCPM (collective + 2 cyclic)
    }
}

impl BuildPart for SwashplateSpec {
    fn name(&self) -> &str {
        "swashplate (rotating + stationary plates)"
    }
    fn material(&self) -> &str {
        "machined aluminium plates; purchased thrust + radial bearings; ball links"
    }
    fn source(&self) -> Source {
        Source::Assembled
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("outer diameter", self.outer_diameter_m * 1000.0),
            ("bore", self.bore_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        vec![
            format!(
                "1. Machine two Ø{:.0} mm plates with a Ø{:.1} mm bore; recess for the \
                 inter-plate bearing.",
                self.outer_diameter_m * 1000.0,
                self.bore_m * 1000.0
            ),
            "2. Press the bearing between plates; the lower stays fixed, the upper rotates."
                .to_string(),
            format!(
                "3. Fit {} pitch-link ball joints on the rotating plate and {} servo-input \
                 joints (120° CCPM) on the stationary plate.",
                self.n_links, self.n_servo_inputs
            ),
            "4. Add an anti-rotation guide for the stationary plate; check free tilt + slide."
                .to_string(),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bore_matches_mast_and_links_match_blades() {
        let s = swashplate_for(0.7, 0.008, 3);
        assert!((s.bore_m - 0.008).abs() < 1e-12);
        assert_eq!(s.n_links, 3);
        // OD is the larger of the radius fraction and the mast multiple.
        assert!(s.outer_diameter_m >= 2.5 * 0.008);
    }
}
