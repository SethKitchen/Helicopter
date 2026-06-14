//! Powertrain tray — mounts the motor, ESC and battery to the airframe.
//!
//! The motor and ESC are purchased, so their exact bolt patterns come from the
//! parts you buy; this part is the fabricated structure that carries them and the
//! pack. It is sized from the pack footprint (the largest item) with room for the
//! motor and controller alongside.

use crate::part::{BuildPart, Source};

/// A powertrain-tray specification (metres).
#[derive(Clone, Debug)]
pub struct MountSpec {
    /// Tray length, m.
    pub length_m: f64,
    /// Tray width, m.
    pub width_m: f64,
    /// Pack mass it carries, kg (drives the footprint).
    pub pack_mass_kg: f64,
}

/// Size a powertrain tray from the pack mass (a proxy for its footprint) and the
/// airframe scale (rotor radius).
pub fn mount_for(pack_mass_kg: f64, rotor_radius_m: f64) -> MountSpec {
    // Pack footprint scales with mass; tray a bit larger, and at least a sensible
    // fraction of the airframe length.
    let side = (0.06 * pack_mass_kg.sqrt()).max(0.10 * rotor_radius_m);
    MountSpec {
        length_m: 2.2 * side,
        width_m: side,
        pack_mass_kg,
    }
}

impl BuildPart for MountSpec {
    fn name(&self) -> &str {
        "powertrain tray (motor + ESC + pack mounts)"
    }
    fn material(&self) -> &str {
        "aluminium plate / CF sheet; purchased motor + ESC bolt to it"
    }
    fn source(&self) -> Source {
        Source::Fabricated
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("tray length", self.length_m * 1000.0),
            ("tray width", self.width_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        vec![
            format!(
                "1. Cut a {:.0} × {:.0} mm tray plate.",
                self.length_m * 1000.0,
                self.width_m * 1000.0
            ),
            "2. Drill the motor bolt circle to match your PURCHASED motor; mount it under the mast."
                .to_string(),
            "3. Add stand-offs for the ESC and a strapped bay for the battery (CG under the shaft)."
                .to_string(),
            format!(
                "4. Verify the tray carries the {:.1} kg pack with the CG on the rotor axis.",
                self.pack_mass_kg
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tray_grows_with_pack_mass() {
        let small = mount_for(0.5, 0.6);
        let big = mount_for(5.0, 0.6);
        assert!(big.width_m > small.width_m);
        assert!((big.length_m / big.width_m - 2.2).abs() < 1e-9);
    }
}
