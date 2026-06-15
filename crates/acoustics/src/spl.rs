//! Sound-pressure-level bookkeeping: pressures → decibels, and the harmonic sum.
//!
//! Acoustic level is referenced to `p_ref = 20 µPa` (the standard air reference):
//! `SPL = 20 log₁₀(p_rms / p_ref)` dB. Independent harmonics combine on an energy
//! (mean-square) basis, so the overall level uses
//! `p_total = √(Σ p_m²)`.

use crate::rotational::{gutin_harmonic_pressure, RotorNoise};
use crate::solution::{Harmonic, NoiseSpectrum};

/// Reference acoustic pressure, Pa (20 µPa).
pub const P_REF: f64 = 20e-6;

/// Sound pressure level of an rms pressure, dB re 20 µPa.
pub fn spl_db(p_rms: f64) -> f64 {
    20.0 * (p_rms.abs() / P_REF).log10()
}

/// Energy-sum of a set of rms pressures into an overall rms pressure.
pub fn combine_rms(pressures: &[f64]) -> f64 {
    pressures.iter().map(|p| p * p).sum::<f64>().sqrt()
}

/// Build the Gutin rotational-noise spectrum (harmonics `1..=n_harmonics`) for a
/// rotor and report each tone plus the overall level at the observer.
pub fn rotational_spectrum(n_harmonics: usize, src: &RotorNoise) -> NoiseSpectrum {
    let mut harmonics = Vec::with_capacity(n_harmonics);
    let mut pressures = Vec::with_capacity(n_harmonics);
    let f1 = src.blades as f64 * src.omega / (2.0 * std::f64::consts::PI);
    for m in 1..=n_harmonics {
        let p = gutin_harmonic_pressure(m, src);
        pressures.push(p);
        harmonics.push(Harmonic {
            m,
            frequency_hz: m as f64 * f1,
            pressure_pa: p,
            spl_db: spl_db(p),
        });
    }
    let overall = combine_rms(&pressures);
    NoiseSpectrum {
        harmonics,
        oaspl_db: spl_db(overall),
        observer_distance_m: src.distance,
        observer_angle_rad: src.theta,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tenfold increase in pressure is +20 dB.
    #[test]
    fn decibel_definition() {
        assert!((spl_db(P_REF) - 0.0).abs() < 1e-12);
        assert!((spl_db(10.0 * P_REF) - 20.0).abs() < 1e-12);
        assert!((spl_db(2.0 * P_REF) - 6.0206).abs() < 1e-3);
    }

    /// Equal-energy combination: two equal tones are +3 dB over one.
    #[test]
    fn energy_sum_is_three_db_for_two_equal_tones() {
        let p = 5.0 * P_REF;
        let total = combine_rms(&[p, p]);
        assert!((spl_db(total) - spl_db(p) - 3.0103).abs() < 1e-3);
    }
}
