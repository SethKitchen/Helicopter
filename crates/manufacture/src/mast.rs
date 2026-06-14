//! Rotor mast (drive shaft) — sized by torsion from the actual hover torque.
//!
//! The mast carries the rotor torque `Q = P_hover / Ω` to the head. For a solid
//! circular shaft the maximum shear stress is `τ = 16 Q / (π d³)`, so the minimum
//! diameter at an allowable working shear `τ_allow` is
//!
//! `d = (16 Q / (π τ_allow))^{1/3}`.
//!
//! That is textbook torsion (e.g. Shigley, *Mechanical Engineering Design*); it
//! makes the mast a *consequence* of the design's power and RPM, not a guess.

use crate::materials::TAU_ALLOW_AL;
use crate::part::{BuildPart, Source};
use std::f64::consts::PI;

/// A drive-mast specification.
#[derive(Clone, Debug)]
pub struct MastSpec {
    /// Transmitted torque, N·m.
    pub torque_nm: f64,
    /// Minimum solid-shaft diameter from torsion, m.
    pub min_diameter_m: f64,
    /// Recommended (rounded-up) diameter, m.
    pub diameter_m: f64,
    /// Mast length, m (head height above the gearbox/motor).
    pub length_m: f64,
}

/// Size a mast for a transmitted `torque_nm` (= P_hover/Ω) and a head height.
pub fn mast_for_torque(torque_nm: f64, head_height_m: f64) -> MastSpec {
    let d_min = (16.0 * torque_nm / (PI * TAU_ALLOW_AL)).cbrt();
    // Round up to the next millimetre for a real stock size.
    let d = (d_min * 1000.0).ceil() / 1000.0;
    MastSpec {
        torque_nm,
        min_diameter_m: d_min,
        diameter_m: d,
        length_m: head_height_m,
    }
}

impl BuildPart for MastSpec {
    fn name(&self) -> &str {
        "rotor mast (drive shaft)"
    }
    fn material(&self) -> &str {
        "6061-T6 aluminium round bar (or steel for higher torque)"
    }
    fn source(&self) -> Source {
        Source::Fabricated
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("diameter", self.diameter_m * 1000.0),
            ("length", self.length_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        vec![
            format!(
                "1. Cut a {:.0} mm length of {:.0} mm round bar (min by torsion {:.1} mm at \
                 {:.2} N·m).",
                self.length_m * 1000.0,
                self.diameter_m * 1000.0,
                self.min_diameter_m * 1000.0,
                self.torque_nm
            ),
            "2. Turn bearing journals and the head taper/spline at the top end."
                .to_string(),
            "3. Machine the drive coupling (key/spline) at the bottom for the motor/gearbox."
                .to_string(),
            "4. Verify runout < 0.05 mm; the mast must spin true.".to_string(),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn diameter_grows_with_torque() {
        let small = mast_for_torque(2.0, 0.2);
        let big = mast_for_torque(20.0, 0.2);
        assert!(big.min_diameter_m > small.min_diameter_m);
        // 10× torque → 10^(1/3) ≈ 2.15× diameter.
        assert!((big.min_diameter_m / small.min_diameter_m - 10f64.cbrt()).abs() < 1e-9);
    }

    #[test]
    fn satisfies_the_torsion_stress_limit() {
        let m = mast_for_torque(5.0, 0.3);
        let tau = 16.0 * m.torque_nm / (PI * m.min_diameter_m.powi(3));
        assert!((tau - TAU_ALLOW_AL).abs() / TAU_ALLOW_AL < 1e-9);
        // The rounded-up diameter is at least the minimum.
        assert!(m.diameter_m >= m.min_diameter_m);
    }
}
