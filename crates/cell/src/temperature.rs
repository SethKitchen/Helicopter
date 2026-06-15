//! Temperature dependence of cell internal resistance.
//!
//! A cell's resistance rises sharply in the cold (slower charge-transfer kinetics
//! and electrolyte ionic conductivity) and falls modestly when warm. The standard
//! description is **Arrhenius**: `R(T) ∝ exp(Ea / (R_gas · T))`, referenced here to
//! 25 °C so the factor is exactly 1.0 at the datasheet rating temperature and the
//! base `R` (the measured 25 °C DCIR) is unchanged.
//!
//! ## Sourcing & honest scope
//! `Ea ≈ 27 kJ/mol` is the central measured area-specific-impedance activation
//! energy for NMC/NCA cathodes (Landesfeind et al. / OSTI ASI study report
//! 27–31 kJ/mol for NMC532/622/811/NCA). With it the model gives **≈7× at −20 °C**
//! vs 25 °C, squarely in the literature 5–10× band.
//!
//! This is a **single-Arrhenius, charge-transfer-dominated** model. It deliberately
//! omits (a) the temperature-*independent* ohmic offset (current collectors,
//! contacts — which do not follow Arrhenius), so it slightly **over-predicts** cold
//! resistance, and (b) the VTF/non-Arrhenius curvature near electrolyte freezing.
//! It is a representative parameter, not a per-cell fit — named, not fudged. Valid
//! roughly across the normal operating band (−20 … +60 °C).

/// Universal gas constant, J/(mol·K).
pub const R_GAS: f64 = 8.314;
/// Activation energy for the Arrhenius resistance rise, J/mol — NMC/NCA central
/// value (measured ASI activation energies 27–31 kJ/mol).
pub const ACTIVATION_ENERGY_J_PER_MOL: f64 = 27_000.0;
/// Reference temperature (25 °C) in kelvin — where the factor is 1.0.
pub const REFERENCE_TEMP_K: f64 = 298.15;

/// Multiplicative factor on the 25 °C internal resistance at `temp_c`. 1.0 at
/// 25 °C; ~7× at −20 °C; ~0.7× at 45 °C.
pub fn resistance_temp_factor(temp_c: f64) -> f64 {
    let t_k = temp_c + 273.15;
    (ACTIVATION_ENERGY_J_PER_MOL / R_GAS * (1.0 / t_k - 1.0 / REFERENCE_TEMP_K)).exp()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unit_at_reference() {
        assert!((resistance_temp_factor(25.0) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn cold_factor_in_literature_band() {
        // −20 °C should land in the cited 5–10× band (≈7× for Ea=27 kJ/mol).
        let f = resistance_temp_factor(-20.0);
        assert!((5.0..=10.0).contains(&f), "factor {f}");
    }

    #[test]
    fn warm_reduces_resistance() {
        assert!(resistance_temp_factor(45.0) < 1.0);
        assert!(resistance_temp_factor(45.0) > 0.5);
    }

    #[test]
    fn monotone_decreasing_in_temperature() {
        let mut prev = f64::INFINITY;
        for t in (-20..=60).step_by(5) {
            let f = resistance_temp_factor(t as f64);
            assert!(f < prev, "not monotone at {t}");
            prev = f;
        }
    }
}
