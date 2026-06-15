//! Broadband rotor noise — the random (non-tonal) sound from turbulent
//! trailing-edge / vortex shedding that forms the floor the Gutin tones sit on.
//!
//! Physics: a distributed dipole whose radiated acoustic power scales as
//! `blade_area · V_tip^6` (the sixth-power tip-speed law for rotor vortex noise;
//! Schlegel, King & Mull 1966 / Prouty, *Helicopter Performance, Stability, and
//! Control*), radiating to a `1/r²` intensity. This module models the broadband
//! OASPL **scaling** from those laws and anchors the **absolute** level to a single
//! supplied reference point [`BroadbandRef`].
//!
//! Honest scope (same discipline as the tonal model): the **scaling is the
//! validated content** — the tip-speed lever (+18 dB per doubling, 6th power),
//! +3 dB per blade-area doubling, −6 dB per distance doubling, and the Strouhal
//! peak frequency. The **absolute anchor is a REQUIRED input, not a fabricated
//! constant** — matching a published *measured* broadband SPL is the same
//! external-sourcing task flagged for Gutin, so no absolute level is invented here.

/// Sixth-power tip-speed law for rotor vortex/broadband noise power
/// (Schlegel-King-Mull / Prouty).
pub const VELOCITY_EXPONENT: f64 = 6.0;
/// Representative Strouhal number for trailing-edge / vortex shedding peak.
pub const STROUHAL: f64 = 0.1;

/// A measured (or assumed) broadband anchor: the overall level at one set of
/// reference conditions. The model scales *from* this point; supplying a real
/// measured value is what turns the scaling into an absolute prediction.
#[derive(Clone, Copy, Debug)]
pub struct BroadbandRef {
    /// Overall broadband SPL at the reference conditions, dB re 20 µPa.
    pub oaspl_db: f64,
    /// Reference tip speed, m/s.
    pub v_tip_ms: f64,
    /// Reference total blade planform area (all blades), m².
    pub blade_area_m2: f64,
    /// Reference observer distance, m.
    pub distance_m: f64,
}

/// Broadband OASPL (dB re 20 µPa) at the given tip speed, total blade area, and
/// observer distance — scaled from `reference` by the vortex-noise laws:
/// `+10·n·log₁₀(V/V₀)` (power ∝ V^n), `+10·log₁₀(A/A₀)` (power ∝ area),
/// `−20·log₁₀(r/r₀)` (intensity ∝ 1/r²).
pub fn broadband_oaspl_db(
    reference: &BroadbandRef,
    v_tip_ms: f64,
    blade_area_m2: f64,
    distance_m: f64,
) -> f64 {
    reference.oaspl_db
        + 10.0 * VELOCITY_EXPONENT * (v_tip_ms / reference.v_tip_ms).log10()
        + 10.0 * (blade_area_m2 / reference.blade_area_m2).log10()
        - 20.0 * (distance_m / reference.distance_m).log10()
}

/// Spectral peak frequency of the broadband noise, Hz — a Strouhal relation on the
/// blade max thickness (the shedding length scale): `f = St · V_tip / t`.
pub fn broadband_peak_hz(v_tip_ms: f64, blade_thickness_m: f64) -> f64 {
    STROUHAL * v_tip_ms / blade_thickness_m
}

/// Energy-sum two overall levels (dB) into a combined OASPL: the broadband floor
/// and the tonal (Gutin) level combine on a mean-square (energy) basis.
pub fn combined_oaspl_db(level_a_db: f64, level_b_db: f64) -> f64 {
    10.0 * (10f64.powf(level_a_db / 10.0) + 10f64.powf(level_b_db / 10.0)).log10()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn anchor() -> BroadbandRef {
        // An ARBITRARY reference purely to exercise the scaling — the test checks
        // the deltas, not this absolute value (which is the named external gap).
        BroadbandRef {
            oaspl_db: 60.0,
            v_tip_ms: 100.0,
            blade_area_m2: 0.1,
            distance_m: 10.0,
        }
    }

    /// The validated content: the vortex-noise scaling laws.
    #[test]
    fn broadband_scaling_laws() {
        let a = anchor();
        let base = broadband_oaspl_db(&a, 100.0, 0.1, 10.0);
        assert!(
            (base - 60.0).abs() < 1e-9,
            "at the reference, returns the reference"
        );
        // Doubling tip speed → +60·log10(2) ≈ +18.06 dB (6th-power law).
        let v2 = broadband_oaspl_db(&a, 200.0, 0.1, 10.0);
        assert!((v2 - base - 60.0 * 2f64.log10()).abs() < 1e-9);
        // Doubling blade area → +3 dB (power ∝ area).
        let a2 = broadband_oaspl_db(&a, 100.0, 0.2, 10.0);
        assert!((a2 - base - 10.0 * 2f64.log10()).abs() < 1e-9);
        // Doubling distance → −6 dB (1/r²).
        let r2 = broadband_oaspl_db(&a, 100.0, 0.1, 20.0);
        assert!((r2 - base + 20.0 * 2f64.log10()).abs() < 1e-9);
    }

    /// The Strouhal peak lands in the audible mid-band for a model rotor.
    #[test]
    fn peak_frequency_in_band() {
        // V_tip 125 m/s, max thickness 6 mm → St·V/t = 0.1·125/0.006 ≈ 2.08 kHz.
        let f = broadband_peak_hz(125.0, 0.006);
        assert!((1500.0..3000.0).contains(&f), "got {f} Hz");
    }

    /// Combining two equal levels adds 3 dB (energy sum) — the broadband floor and
    /// the tonal level combine correctly.
    #[test]
    fn combined_oaspl_energy_sum() {
        assert!((combined_oaspl_db(70.0, 70.0) - 73.0103).abs() < 1e-3);
        // A floor 10 dB below a tone barely moves the total (+0.41 dB).
        assert!((combined_oaspl_db(80.0, 70.0) - 80.4139).abs() < 1e-3);
    }
}
