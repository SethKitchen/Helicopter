//! Fuselage / canopy pod — houses the pack, motor and (for a manned craft) the
//! occupants, and carries the boom and skids.
//!
//! Modelled as a smooth lifting-body pod sized to enclose the powertrain tray with
//! a margin; the canopy is the upper half of the same shell. Exact internal layout
//! is the builder's, but the envelope and its geometry come from the design scale.

use crate::part::{BuildPart, Source};

/// A fuselage-pod specification (metres; semi-axes are half of each dimension).
#[derive(Clone, Debug)]
pub struct FuselageSpec {
    /// Overall length (x), m.
    pub length_m: f64,
    /// Overall width (y), m.
    pub width_m: f64,
    /// Overall height (z), m.
    pub height_m: f64,
}

/// Size a pod from the gross mass and rotor radius (envelope to enclose the
/// powertrain + payload with margin).
pub fn fuselage_for(gross_mass_kg: f64, rotor_radius_m: f64) -> FuselageSpec {
    // Footprint scales with mass^(1/3); length a fraction of the rotor radius too.
    let base = 0.12 * gross_mass_kg.cbrt();
    FuselageSpec {
        length_m: (2.2 * base).max(0.5 * rotor_radius_m),
        width_m: base.max(0.12 * rotor_radius_m),
        height_m: 1.1 * base.max(0.12 * rotor_radius_m),
    }
}

impl BuildPart for FuselageSpec {
    fn name(&self) -> &str {
        "fuselage / canopy pod"
    }
    fn material(&self) -> &str {
        "moulded composite shell / formed sheet (canopy = upper half)"
    }
    fn source(&self) -> Source {
        Source::Fabricated
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("length", self.length_m * 1000.0),
            ("width", self.width_m * 1000.0),
            ("height", self.height_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        vec![
            format!(
                "1. Make a plug/mould for the exported {:.0} × {:.0} × {:.0} mm smooth tapered pod.",
                self.length_m * 1000.0,
                self.width_m * 1000.0,
                self.height_m * 1000.0
            ),
            "2. Lay up the shell; split the upper half as a removable canopy.".to_string(),
            "3. Add bulkheads for the mast mount (top) and skid/boom hardpoints.".to_string(),
            "4. Cut access for the battery, wiring and cooling airflow.".to_string(),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pod_grows_with_mass() {
        let small = fuselage_for(3.5, 0.7);
        let big = fuselage_for(700.0, 4.0);
        assert!(big.length_m > small.length_m);
        assert!(big.width_m > small.width_m);
    }
}
