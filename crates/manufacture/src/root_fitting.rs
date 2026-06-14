//! Blade root fitting — the clamp/tang that joins the blade to the grip.
//!
//! The aerodynamic blade is shaped from the cutout to the tip; the inboard end
//! needs a solid fitting that the hub grip clamps and a bolt passes through. It is
//! sized from the blade root chord/thickness and the centrifugal load it carries
//! (the bolt diameter is checked in [`crate::structural`]).

use crate::part::{BuildPart, Source};

/// A blade root-fitting specification (metres).
#[derive(Clone, Debug)]
pub struct RootFitting {
    /// Number of fittings (one per blade).
    pub count: usize,
    /// Length of the fitting along the span (clamp overlap), m.
    pub length_m: f64,
    /// Width (≈ root chord), m.
    pub width_m: f64,
    /// Thickness (≈ blade root thickness), m.
    pub thickness_m: f64,
    /// Retention bolt diameter, m.
    pub bolt_diameter_m: f64,
}

/// Build a root fitting from blade root geometry and a chosen bolt size.
pub fn root_fitting_for(
    n_blades: usize,
    root_chord_m: f64,
    root_thickness_m: f64,
    bolt_diameter_m: f64,
) -> RootFitting {
    RootFitting {
        count: n_blades,
        length_m: 1.5 * root_chord_m,
        width_m: root_chord_m,
        thickness_m: root_thickness_m.max(bolt_diameter_m * 1.8),
        bolt_diameter_m,
    }
}

impl BuildPart for RootFitting {
    fn name(&self) -> &str {
        "blade root fittings"
    }
    fn material(&self) -> &str {
        "aluminium/steel tang bonded or bolted into the blade root"
    }
    fn source(&self) -> Source {
        Source::Fabricated
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("length", self.length_m * 1000.0),
            ("width", self.width_m * 1000.0),
            ("thickness", self.thickness_m * 1000.0),
            ("bolt Ø", self.bolt_diameter_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        vec![
            format!(
                "1. Make {} root tang(s): {:.0} × {:.0} × {:.1} mm, bonded/bolted into each \
                 blade root.",
                self.count,
                self.length_m * 1000.0,
                self.width_m * 1000.0,
                self.thickness_m * 1000.0
            ),
            format!(
                "2. Drill the Ø{:.1} mm retention bolt hole on the pitch axis (centrifugal \
                 load — see structural check).",
                self.bolt_diameter_m * 1000.0
            ),
            "3. Match-drill the grip; the bolt is the flap/feather pivot — use a close fit."
                .to_string(),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fitting_scales_with_root_and_has_min_thickness_for_bolt() {
        let f = root_fitting_for(3, 0.04, 0.002, 0.004);
        assert_eq!(f.count, 3);
        assert!((f.width_m - 0.04).abs() < 1e-12);
        // Thin blade root → thickness driven up to fit the bolt (1.8×Ø).
        assert!(f.thickness_m >= 0.004 * 1.8 - 1e-12);
    }
}
