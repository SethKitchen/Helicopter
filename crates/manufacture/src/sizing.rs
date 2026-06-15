//! Shared round-section sizing formulas — the single source for the boom bending
//! and mast torsion sizing (and the round-up-to-mm stock step), so a part and the
//! structural check that verifies it can never diverge (they used to be re-derived
//! in five places, two with the material allowable hard-coded as a literal).

use std::f64::consts::PI;

/// Thin-tube section-modulus coefficient `Z ≈ TUBE_Z_COEFF · D³` (wall 0.1·D).
pub const TUBE_Z_COEFF: f64 = 0.058;

/// Round a diameter (m) up to the next whole millimetre — a real stock size.
pub fn round_up_mm(d_m: f64) -> f64 {
    (d_m * 1000.0).ceil() / 1000.0
}

/// Minimum (un-rounded) boom tube outer diameter so bending `moment` stays within
/// `sigma_allow` (`σ = M / (Z_COEFF·D³)`), m.
pub fn boom_min_od_for_bending(moment_nm: f64, sigma_allow_pa: f64) -> f64 {
    (moment_nm / (TUBE_Z_COEFF * sigma_allow_pa)).cbrt()
}

/// Boom tube outer diameter sized for bending, rounded up to a mm stock size, m.
pub fn boom_od_for_bending(moment_nm: f64, sigma_allow_pa: f64) -> f64 {
    round_up_mm(boom_min_od_for_bending(moment_nm, sigma_allow_pa))
}

/// Bending stress in a boom tube of outer diameter `od` under `moment`, Pa.
pub fn boom_bending_stress(moment_nm: f64, od_m: f64) -> f64 {
    moment_nm / (TUBE_Z_COEFF * od_m.powi(3))
}

/// Minimum (un-rounded) solid mast diameter so torsion `torque` stays within
/// `tau_allow` (`τ = 16T / (π d³)`), m.
pub fn mast_min_dia_for_torsion(torque_nm: f64, tau_allow_pa: f64) -> f64 {
    (16.0 * torque_nm / (PI * tau_allow_pa)).cbrt()
}

/// Solid mast diameter sized for torsion, rounded up to a mm stock size, m.
pub fn mast_dia_for_torsion(torque_nm: f64, tau_allow_pa: f64) -> f64 {
    round_up_mm(mast_min_dia_for_torsion(torque_nm, tau_allow_pa))
}

/// Torsional shear stress in a solid mast of diameter `d` under `torque`, Pa.
pub fn mast_torsion_stress(torque_nm: f64, d_m: f64) -> f64 {
    16.0 * torque_nm / (PI * d_m.powi(3))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The sized OD/diameter, fed back through the stress formula, lands at exactly
    /// the allowable for the UN-rounded size (the sizing inverts the stress).
    #[test]
    fn sizing_inverts_stress() {
        let (m, sig) = (3.0, 90.0e6);
        let od = boom_min_od_for_bending(m, sig);
        assert!((boom_bending_stress(m, od) - sig).abs() / sig < 1e-9);
        let (t, tau) = (2.0, 55.0e6);
        let d = mast_min_dia_for_torsion(t, tau);
        assert!((mast_torsion_stress(t, d) - tau).abs() / tau < 1e-9);
        // Rounding up to mm only ever lowers the stress (adds margin).
        assert!(boom_bending_stress(m, boom_od_for_bending(m, sig)) <= sig);
        assert!(round_up_mm(0.0041) == 0.005);
    }
}
