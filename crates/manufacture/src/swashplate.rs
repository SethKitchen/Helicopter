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
pub fn swashplate_for(
    rotor_radius_m: f64,
    mast_diameter_m: f64,
    n_blades: usize,
) -> SwashplateSpec {
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
                "1. Machine two Ø{:.0} mm plates, each with a Ø{:.1} mm bore, and a central \
                 ball/uniball that lets the assembly BOTH slide up/down the mast AND tilt in \
                 any direction (the bore is a sliding+gimballing fit on the mast, not a press fit).",
                self.outer_diameter_m * 1000.0,
                self.bore_m * 1000.0
            ),
            "2. Press a thrust+radial bearing between the plates: the LOWER (stationary) plate \
             never rotates; the UPPER (rotating) plate spins with the head. The bearing lets the \
             two share the same up/down position and tilt while one spins."
                .to_string(),
            format!(
                "3. ACTUATOR = {} servos (digital, metal-gear) — the correct choice: blade pitch \
                 is a continuous, bidirectional, position-held angle, which is exactly what a servo \
                 gives (a brushless ESC/motor cannot hold a precise angle). Mount the {} servos to \
                 the frame at 120° around the mast (CCPM); connect each by a threaded pushrod + \
                 ball link to a pickup on the STATIONARY plate.",
                self.n_servo_inputs, self.n_servo_inputs
            ),
            "4. How it MOVES (CCPM mixing, done in the flight controller): all 3 servos UP together \
             = the whole swashplate rises = COLLECTIVE (every blade gains pitch, more thrust). Two \
             servos differential = the plate TILTS = CYCLIC (blade pitch varies once per rev, tilting \
             the rotor disk). The FC mixes stick → 3 servo angles."
                .to_string(),
            format!(
                "5. Fit {} ball joints on the ROTATING plate; run a pitch link from each up to its \
                 blade-grip PITCH HORN. The rotating plate's tilt feeds a once-per-rev pitch change \
                 to each blade. IMPORTANT: lead the pitch horn ~90° around from the blade (gyroscopic \
                 precession — peak pitch input precedes peak flap by ~90°), so a fore-aft stick tilts \
                 the disk fore-aft, not sideways.",
                self.n_links
            ),
            "6. Drive the rotating plate with the mast via a SCISSOR/driver link (so it spins with \
             the head), and hold the stationary plate from spinning with a second anti-rotation \
             scissor to the frame. Check: free slide, free tilt, no bind through full collective + \
             full cyclic; set link lengths so zero stick = zero blade pitch at mid-collective."
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
