//! Shared round-section sizing formulas — the single source for the boom bending
//! and mast torsion sizing (and the round-up-to-mm stock step), so a part and the
//! structural check that verifies it can never diverge (they used to be re-derived
//! in five places, two with the material allowable hard-coded as a literal).

use std::f64::consts::PI;

/// Thin-tube section-modulus coefficient `Z ≈ TUBE_Z_COEFF · D³` (wall 0.1·D).
pub const TUBE_Z_COEFF: f64 = 0.058;
/// Thin-tube second-moment coefficient `I ≈ TUBE_I_COEFF · D⁴` (wall 0.1·D):
/// `I = π(D⁴−d⁴)/64`, `d=0.8D` ⇒ `π·0.5904/64 ≈ 0.029`.
pub const TUBE_I_COEFF: f64 = 0.029;
/// Thin-tube cross-section area coefficient `A ≈ TUBE_A_COEFF · D²` (wall 0.1·D):
/// `A = π(D²−d²)/4`, `d=0.8D` ⇒ `π·0.36/4 ≈ 0.2827`.
pub const TUBE_A_COEFF: f64 = 0.2827;

/// Minimum boom tube OD so a cantilever tip load `p` (N) over `length` (m) deflects
/// no more than `defl_limit` (m): `δ = pL³/(3EI) ≤ limit`, `I = TUBE_I_COEFF·D⁴`, m.
pub fn boom_min_od_for_stiffness(p_n: f64, length_m: f64, e_pa: f64, defl_limit_m: f64) -> f64 {
    (p_n * length_m.powi(3) / (3.0 * e_pa * TUBE_I_COEFF * defl_limit_m)).powf(0.25)
}

/// Target boom fundamental as a multiple of 1/rev — placed midway between 1/rev and
/// 2/rev so it is maximally clear of the two strongest rotor harmonics.
pub const BOOM_TARGET_PER_REV: f64 = 1.5;

/// First clamped-free eigenvalue squared, `β₁² = 1.875104²` — for the boom resonance
/// sizing (cantilever fundamental `f₁ = (β₁²/2π)√(EI/μL⁴)`).
pub const CANTILEVER_BETA1_SQ: f64 = 3.516_015;

/// Minimum boom OD so its cantilever fundamental reaches `target_hz`. For a thin tube
/// `f₁ ∝ D` (both `I∝D⁴` and `A∝D²`), so `D = target_hz / k` with
/// `k = (β₁²/2π)·√(E·I_coeff / (ρ·A_coeff·L⁴))`, m.
pub fn boom_min_od_for_frequency(target_hz: f64, length_m: f64, e_pa: f64, rho: f64) -> f64 {
    let k = (CANTILEVER_BETA1_SQ / (2.0 * PI))
        * (e_pa * TUBE_I_COEFF / (rho * TUBE_A_COEFF * length_m.powi(4))).sqrt();
    if k <= 0.0 { 0.0 } else { target_hz / k }
}

/// Governing boom OD (rounded to mm): the LARGEST of the bending-stress, the
/// tip-deflection (`defl_frac` of length), AND the resonance-frequency (`target_freq_hz`)
/// requirements — so the boom is strong, stiff, AND clear of the rotor harmonics.
/// Replaces a stress-only boom that passes σ yet whips or resonates.
pub fn boom_governing_od(
    moment_nm: f64,
    length_m: f64,
    e_pa: f64,
    rho: f64,
    sigma_allow_pa: f64,
    defl_frac: f64,
    target_freq_hz: f64,
) -> f64 {
    let bend = boom_min_od_for_bending(moment_nm, sigma_allow_pa);
    let p = moment_nm / length_m; // tip load (moment = p·length for the cantilever)
    let stiff = boom_min_od_for_stiffness(p, length_m, e_pa, defl_frac * length_m);
    let freq = boom_min_od_for_frequency(target_freq_hz, length_m, e_pa, rho);
    round_up_mm(bend.max(stiff).max(freq))
}

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
