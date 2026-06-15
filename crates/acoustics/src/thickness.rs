//! Thickness noise — the tip-Mach "master knob" for rotor loudness.
//!
//! Thickness (volume-displacement) noise comes from the blade physically pushing
//! air aside as it sweeps past; unlike loading noise it does not depend on how
//! hard the rotor is working, only on blade volume and how fast the tip moves.
//! Its full evaluation is the Ffowcs Williams–Hawkings / Farassat 1A thickness
//! integral — out of scope here. What *is* in scope, and what matters for design,
//! is its **scaling**: in the subsonic compact regime the radiated thickness-noise
//! acoustic pressure grows roughly as the cube of the tip Mach number,
//! `p_thick ∝ M_tip³` (and far steeper as the tip approaches transonic, where
//! delocalised shock noise sets in).
//!
//! That cube is why **tip speed is the dominant acoustic design lever**: a sound
//! pressure `∝ M³` is a sound *level* `∝ 60 log₁₀ M`, so a 10 % cut in tip speed
//! buys ≈ 2.7 dB with no change to thrust — the cheapest noise reduction
//! available, traded against the higher torque/heavier blades a slower rotor
//! needs. This module exposes the lever as a *relative* indicator (validated only
//! for its monotone steep growth), not an absolute SPL; the quantitative tonal
//! level comes from [`crate::rotational`].
//!
//! Source for the `M³` subsonic scaling: Gutin/Deming thickness-noise theory and
//! the compact-source limit (Leishman, *Principles of Helicopter Aerodynamics*
//! 2nd ed., §8.4–8.5).

/// Relative thickness-noise pressure index, `∝ M_tip³` — a dimensionless growth
/// indicator normalised to 1 at `M_tip = 1` (sonic). Use ratios, not the absolute
/// value.
pub fn thickness_noise_index(m_tip: f64) -> f64 {
    m_tip.powi(3)
}

/// Change in thickness-noise level (dB) when moving the tip Mach from
/// `m_tip_ref` to `m_tip`, in the subsonic `∝ M³` regime:
/// `ΔSPL = 60 log₁₀(M / M_ref)`.
pub fn thickness_noise_db_delta(m_tip_ref: f64, m_tip: f64) -> f64 {
    60.0 * (m_tip / m_tip_ref).log10()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Doubling tip Mach raises the pressure index 8× (third power).
    #[test]
    fn index_is_cubic() {
        assert!((thickness_noise_index(0.6) - 0.216).abs() < 1e-12);
        assert!((thickness_noise_index(1.2) / thickness_noise_index(0.6) - 8.0).abs() < 1e-9);
    }

    /// A 10 % tip-speed reduction is worth ≈ 2.7 dB of thickness noise.
    #[test]
    fn ten_percent_slower_buys_a_couple_db() {
        let d = thickness_noise_db_delta(0.6, 0.54); // -10%
        assert!(d < 0.0);
        assert!((d + 2.74).abs() < 0.1, "got {d} dB");
    }

    /// Monotone increasing in tip Mach.
    #[test]
    fn monotone_in_tip_mach() {
        assert!(thickness_noise_index(0.4) < thickness_noise_index(0.5));
        assert!(thickness_noise_index(0.5) < thickness_noise_index(0.7));
    }
}
