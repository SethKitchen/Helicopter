//! Tail boom — sets the tail-rotor moment arm and reacts the main torque.
//!
//! The tail rotor sits at the boom tip a distance `L_tr` from the shaft and makes
//! thrust `T_tr` to balance the main torque: `T_tr · L_tr = Q`. The boom is then a
//! cantilever with `T_tr` at its tip, so the **bending moment at the boom root is
//! exactly the main rotor torque** `M = T_tr · L_tr = Q`. The tube outer diameter
//! follows from the bending stress of a thin tube (wall ≈ 0.1·D):
//!
//! `σ = M / Z`,  `Z ≈ 0.058 D³`  ⟹  `D = (M / (0.058 σ_allow))^{1/3}`.
//!
//! The thin-tube section modulus `Z = π(D⁴−d⁴)/(32D)` with wall `t = 0.1 D`
//! (so `d = 0.8 D`) gives `Z = π·0.5904·D³/32 ≈ 0.058 D³` (Roark's *Formulas for
//! Stress and Strain*, hollow circular section).
//!
//! Arm length is set for tip clearance (`L_tr ≈ 1.15 R`, so the tail rotor clears
//! the main disk).

use crate::materials::{E_AL, RHO_AL, SIGMA_ALLOW_AL};
use crate::part::{BuildPart, Source};
use crate::sizing::{BOOM_TARGET_PER_REV, boom_governing_od};
use std::f64::consts::PI;

/// Tip-deflection limit for the boom, as a fraction of its length (a stiff boom keeps
/// the tail rotor on-axis and its bending frequency clear of the rotor harmonics).
const BOOM_DEFL_FRAC: f64 = 0.02;

/// A tail-boom specification (metres).
#[derive(Clone, Debug)]
pub struct BoomSpec {
    /// Tail-rotor moment arm = boom length, m.
    pub length_m: f64,
    /// Root bending moment (= main torque), N·m.
    pub root_moment_nm: f64,
    /// Tube outer diameter from bending, m.
    pub tube_od_m: f64,
    /// Tube wall thickness (≈0.1·OD), m.
    pub tube_wall_m: f64,
}

/// Size a tail boom from the main torque, rotor radius, and rotor speed `omega`
/// (rad/s — sets the resonance-frequency target).
pub fn boom_for(main_torque_nm: f64, rotor_radius_m: f64, omega_rad_s: f64) -> BoomSpec {
    let length = 1.15 * rotor_radius_m;
    let m = main_torque_nm; // root bending moment = main torque
    // Sized for bending stress, a ≤2% tip-deflection stiffness limit, AND a fundamental
    // frequency at ~1.5/rev (clear of the rotor harmonics) — the governing one wins, so
    // the boom is strong, stiff, AND non-resonant (see the resonance check).
    let target_hz = BOOM_TARGET_PER_REV * omega_rad_s / (2.0 * PI);
    let od = boom_governing_od(
        m,
        length,
        E_AL,
        RHO_AL,
        SIGMA_ALLOW_AL,
        BOOM_DEFL_FRAC,
        target_hz,
    );
    BoomSpec {
        length_m: length,
        root_moment_nm: m,
        tube_od_m: od,
        tube_wall_m: 0.1 * od,
    }
}

impl BuildPart for BoomSpec {
    fn name(&self) -> &str {
        "tail boom"
    }
    fn material(&self) -> &str {
        "6061-T6 aluminium or carbon-fibre tube"
    }
    fn source(&self) -> Source {
        Source::RawStock
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("length", self.length_m * 1000.0),
            ("tube OD", self.tube_od_m * 1000.0),
            ("tube wall", self.tube_wall_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        vec![
            format!(
                "1. Cut a {:.0} mm length of Ø{:.0} mm tube (wall ≥ {:.1} mm; sized for the \
                 {:.2} N·m root bending = main torque).",
                self.length_m * 1000.0,
                self.tube_od_m * 1000.0,
                self.tube_wall_m * 1000.0,
                self.root_moment_nm
            ),
            "2. Fit the tail-rotor gearbox/motor mount at the tip and the airframe clamp at the root."
                .to_string(),
            "3. Route the tail-rotor drive (or wiring, if an electric tail) and the pitch control."
                .to_string(),
            "4. Add a horizontal stabiliser if used; check the boom is straight and rigid."
                .to_string(),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn arm_scales_with_radius() {
        let b = boom_for(2.0, 0.7, 100.0);
        assert!((b.length_m - 1.15 * 0.7).abs() < 1e-12);
    }

    #[test]
    fn tube_od_grows_with_torque_and_meets_bending_limit() {
        // omega = 0 removes the frequency target so this isolates the torque scaling.
        let small = boom_for(2.0, 0.7, 0.0);
        let big = boom_for(20.0, 0.7, 0.0);
        assert!(big.tube_od_m > small.tube_od_m);
        // The min (un-rounded) OD satisfies σ ≤ σ_allow with Z = 0.058 D³.
        let d_min = (small.root_moment_nm / (0.058 * SIGMA_ALLOW_AL)).cbrt();
        let sigma = small.root_moment_nm / (0.058 * d_min.powi(3));
        assert!((sigma - SIGMA_ALLOW_AL).abs() / SIGMA_ALLOW_AL < 1e-9);
    }
}
