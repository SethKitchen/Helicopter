//! Gutin rotational (loading) noise — the tonal sound of steady blade loading
//! sweeping around the disk.
//!
//! A rotor in steady flight carries a fixed thrust `T` and torque `Q`; to a
//! stationary observer those loads rotate, radiating tones at the blade-passage
//! frequency `B Ω / 2π` and its harmonics. Gutin (1936) solved this for a compact
//! rotating source. The rms acoustic pressure of the `m`-th harmonic at an
//! observer a distance `s` from the hub, at angle `θ` from the rotor axis, is
//!
//! `p_m = (m B Ω)/(2√2 π a₀ s) · [ T cosθ − a₀ Q /(Ω R_e²) ] · J_{mB}( m B Ω R_e sinθ / a₀ )`
//!
//! where `B` = blades, `Ω` = rotational speed, `a₀` = speed of sound, `R_e` ≈
//! 0.8 R the effective loading radius, and `J_{mB}` the Bessel function of the
//! first kind ([`crate::bessel`]). Two physical features fall straight out and are
//! validated in `tests/`:
//!
//! * **On-axis null** — at `θ = 0`, `sinθ = 0` so `J_{mB}(0) = 0` for `mB ≥ 1`:
//!   no rotational noise radiates along the shaft axis. The tone is loudest near
//!   the disk plane.
//! * **Harmonic decay** — for subsonic tips the Bessel argument is below the order
//!   `mB`, where `J_{mB}` falls off fast, so higher harmonics are progressively
//!   quieter. (As the tip approaches sonic the argument catches the order and the
//!   harmonics stop decaying — the physical reason fast tips are so much louder.)
//!
//! Source: L. Gutin, *On the sound field of a rotating propeller* (1936; NACA TM
//! 1195); see also Leishman, *Principles of Helicopter Aerodynamics* (2nd ed.,
//! §8.4). This is **loading** noise only; thickness noise is [`crate::thickness`].

use crate::bessel::bessel_j;

/// Signed rms acoustic pressure (Pa) of the `m`-th blade-passage harmonic from
/// Gutin's formula. `omega` rad/s, `sound_speed` & all lengths SI, `theta`
/// radians from the rotor axis, `r_eff` the effective loading radius.
pub fn gutin_harmonic_pressure(
    m: usize,
    blades: usize,
    omega: f64,
    sound_speed: f64,
    distance: f64,
    thrust: f64,
    torque: f64,
    r_eff: f64,
    theta: f64,
) -> f64 {
    let mb = (m * blades) as f64;
    let amp = mb * omega / (2.0 * std::f64::consts::SQRT_2 * std::f64::consts::PI * sound_speed * distance);
    let bracket = thrust * theta.cos() - sound_speed * torque / (omega * r_eff * r_eff);
    let arg = mb * omega * r_eff * theta.sin() / sound_speed;
    amp * bracket * bessel_j(m * blades, arg)
}

/// Blade-passage frequency `B Ω / 2π`, Hz (the `m = 1` tone frequency).
pub fn blade_passage_frequency(blades: usize, omega: f64) -> f64 {
    blades as f64 * omega / (2.0 * std::f64::consts::PI)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn case(theta: f64) -> f64 {
        // 2-blade, Ω=50 rad/s, a₀=340, s=50 m, T=9810 N, Q=4000 N·m, R_e=3.2 m.
        gutin_harmonic_pressure(1, 2, 50.0, 340.0, 50.0, 9810.0, 4000.0, 3.2, theta)
    }

    #[test]
    fn on_axis_null() {
        assert!(case(0.0).abs() < 1e-12, "rotational noise must vanish on axis");
    }

    #[test]
    fn directivity_peaks_off_axis() {
        // Rotational noise is null on the axis, rises off it, and — because the
        // torque term opposes the thrust term and flips the bracket sign as the
        // observer nears the disk plane — peaks at an intermediate angle rather
        // than in the plane itself. So the mid-angle tone is louder than both a
        // near-axis and a near-plane observer.
        let near_axis = case(10f64.to_radians()).abs();
        let mid = case(45f64.to_radians()).abs();
        let near_plane = case(85f64.to_radians()).abs();
        assert!(mid > near_axis && mid > near_plane);
    }

    #[test]
    fn higher_harmonics_decay_for_subsonic_tip() {
        let theta = 80f64.to_radians();
        let p1 = gutin_harmonic_pressure(1, 2, 50.0, 340.0, 50.0, 9810.0, 4000.0, 3.2, theta).abs();
        let p2 = gutin_harmonic_pressure(2, 2, 50.0, 340.0, 50.0, 9810.0, 4000.0, 3.2, theta).abs();
        let p3 = gutin_harmonic_pressure(3, 2, 50.0, 340.0, 50.0, 9810.0, 4000.0, 3.2, theta).abs();
        // Tip Mach = 50*3.2/340 ≈ 0.47, well subsonic → strong decay.
        assert!(p2 < p1 && p3 < p2);
    }

    #[test]
    fn blade_passage_frequency_value() {
        // 2 blades at Ω = 2π·5 rad/s (5 rev/s) → 10 Hz.
        let f = blade_passage_frequency(2, 2.0 * PI * 5.0);
        assert!((f - 10.0).abs() < 1e-9);
    }
}
