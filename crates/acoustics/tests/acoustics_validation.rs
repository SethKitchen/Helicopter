//! Acoustics validation.
//!
//! The Bessel routine is anchored against tabulated zeros/values/recurrence (unit
//! tests in `bessel.rs`); Gutin's on-axis null, directivity and harmonic decay,
//! and the thickness `M³` lever are checked in their modules. Here we exercise the
//! assembled spectrum end-to-end and confirm the two design-level statements the
//! crate exists to make: rotor noise is directional, and **tip speed is the
//! master knob**.
//!
//! No external measured-SPL oracle is asserted — matching one published rotor's
//! noise with full geometry is a sourced future step (see crate docs), and the
//! project rule is to never fabricate an oracle value.

use helisim_acoustics::*;

/// Representative rotor: 2 blades, Ω = 50 rad/s, R_e = 3.2 m (≈0.8·4 m),
/// T = 9810 N, Q = 4000 N·m, observer 50 m away.
fn src(omega: f64, theta_deg: f64) -> RotorNoise {
    RotorNoise {
        blades: 2,
        omega,
        sound_speed: 340.0,
        distance: 50.0,
        thrust: 9810.0,
        torque: 4000.0,
        r_eff: 3.2,
        theta: theta_deg.to_radians(),
    }
}

fn spectrum_at(theta_deg: f64) -> NoiseSpectrum {
    rotational_spectrum(6, &src(50.0, theta_deg))
}

#[test]
fn spectrum_is_directional_and_finite() {
    let near_axis = spectrum_at(10.0);
    let mid = spectrum_at(45.0);
    // Gutin directivity: the off-axis (mid-angle) observer is louder than one
    // near the axis, and the level is physical.
    assert!(mid.oaspl_db > near_axis.oaspl_db);
    assert!(mid.oaspl_db.is_finite() && mid.oaspl_db > 0.0);
    // Fundamental dominates the energy sum for a subsonic tip.
    let h = &mid.harmonics;
    assert!(h[0].pressure_pa.abs() > h[1].pressure_pa.abs());
    assert_eq!(h[0].m, 1);
}

#[test]
fn fundamental_frequency_tracks_blade_count_and_rpm() {
    let s = spectrum_at(80.0);
    // f1 = B·Ω/2π = 2·50/2π ≈ 15.92 Hz; harmonic m has m·f1.
    assert!((s.harmonics[0].frequency_hz - 2.0 * 50.0 / (2.0 * std::f64::consts::PI)).abs() < 1e-6);
    assert!((s.harmonics[2].frequency_hz / s.harmonics[0].frequency_hz - 3.0).abs() < 1e-9);
}

#[test]
fn faster_tip_is_louder_loading_noise() {
    // Raising Ω (tip speed) at the same observer raises the loading-noise level.
    let slow = rotational_spectrum(6, &src(45.0, 80.0));
    let fast = rotational_spectrum(6, &src(55.0, 80.0));
    assert!(fast.oaspl_db > slow.oaspl_db);
}

#[test]
fn tip_speed_is_the_master_noise_knob() {
    // The thickness-noise lever: a 15% tip-speed cut is worth several dB, with no
    // change in thrust — the cheapest quieting available.
    let delta = thickness_noise_db_delta(0.60, 0.51); // -15%
    assert!(delta < -4.0 && delta > -5.0, "got {delta} dB");
}
