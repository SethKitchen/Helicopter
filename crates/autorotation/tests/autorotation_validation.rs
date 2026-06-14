//! Autorotation validation.
//!
//! The descent-regime inflow closed forms are anchored by their own momentum
//! quadratics (unit tests in `inflow.rs`). The integrated result — the steady
//! vertical-autorotation descent rate — is validated here against the **measured
//! ideal-autorotation band** `V_d/v_h ≈ [1.7, 2.0]` reported from axial-descent
//! flight test and the induced-velocity curve (Leishman, *Principles of
//! Helicopter Aerodynamics* 2nd ed., §2.13–2.14; Prouty, *Helicopter Performance,
//! Stability and Control*). This is a measured-band oracle in the spirit of the
//! Harrington figure-of-merit check — the autorotation equilibrium sits in the
//! vortex-ring/turbulent-wake regime where momentum theory is invalid, so a
//! closed form is unavailable *by physics*, and the measured band is the honest
//! oracle.

use helisim_autorotation::{
    autorotation_index, flare_height_equivalent, forward_induced_velocity, glide_polar,
    hover_induced_velocity, profile_power, steady_autorotation, G,
};
use std::f64::consts::PI;

/// A representative light single-rotor helicopter (order-of-magnitude realistic):
/// ~1000 kg, 4 m radius, tip speed ~190 m/s, σ≈0.07, C_d0≈0.010.
fn representative_case() -> (f64, f64, f64, f64) {
    let mass_kg = 1000.0;
    let radius = 4.0;
    let rho = 1.225;
    let area = PI * radius * radius;
    let tip_speed = 190.0;
    let solidity = 0.07;
    let cd0 = 0.010;
    let thrust = mass_kg * G;
    let p0 = profile_power(rho, area, tip_speed, solidity, cd0);
    (thrust, rho, area, p0)
}

#[test]
fn ideal_autorotation_descent_ratio_in_measured_band() {
    let (thrust, rho, area, p0) = representative_case();
    let sol = steady_autorotation(thrust, rho, area, p0);
    assert!(
        (1.7..=2.0).contains(&sol.descent_ratio),
        "V_d/v_h = {:.3} outside measured ideal-autorotation band [1.7, 2.0]",
        sol.descent_ratio
    );
    // The descent must supply real induced + profile power → descent rate exceeds
    // the hover induced velocity.
    assert!(sol.descent_rate_ms > sol.hover_induced_velocity_ms);
}

#[test]
fn zero_profile_recovers_pure_induced_ideal_autorotation() {
    // With no profile power the balance is V_d = v_i alone; the fixed point of
    // d = f(-d) is the pure-induced ideal autorotation, ≈ 1.79 v_h on the
    // measured curve — the textbook "≈ 1.8 v_h" ideal value.
    let (thrust, rho, area, _) = representative_case();
    let sol = steady_autorotation(thrust, rho, area, 0.0);
    assert!(
        (1.75..=1.85).contains(&sol.descent_ratio),
        "pure-induced ideal autorotation V_d/v_h = {:.3}, expected ≈ 1.79",
        sol.descent_ratio
    );
}

#[test]
fn profile_power_raises_the_descent_rate() {
    // More profile drag → the descent must supply more power → faster descent.
    let (thrust, rho, area, p0) = representative_case();
    let slow = steady_autorotation(thrust, rho, area, 0.0);
    let fast = steady_autorotation(thrust, rho, area, p0);
    let faster = steady_autorotation(thrust, rho, area, 2.0 * p0);
    assert!(fast.descent_rate_ms > slow.descent_rate_ms);
    assert!(faster.descent_rate_ms > fast.descent_rate_ms);
}

#[test]
fn descent_rate_fpm_conversion_and_positive() {
    let (thrust, rho, area, p0) = representative_case();
    let sol = steady_autorotation(thrust, rho, area, p0);
    assert!(sol.descent_rate_ms > 0.0);
    assert!((sol.descent_rate_fpm() / sol.descent_rate_ms - 196.85).abs() < 0.01);
}

#[test]
fn forward_induced_velocity_limits() {
    let v_h = 8.0;
    // Hover (V=0): v_i = v_h.
    assert!((forward_induced_velocity(v_h, 0.0) - v_h).abs() < 1e-6);
    // High speed: v_i → v_h²/V.
    let v = 60.0;
    assert!((forward_induced_velocity(v_h, v) - v_h * v_h / v).abs() < 1e-2);
}

#[test]
fn forward_autorotation_is_much_gentler_than_vertical() {
    // Same representative rotor; forward flight roughly halves the descent rate —
    // the reason a forced landing is flown with airspeed, not straight down.
    let (thrust, rho, area, p0) = representative_case();
    let vertical = steady_autorotation(thrust, rho, area, p0);

    let flat_plate = 1.0; // m², representative light-helicopter parasite area
    let speeds: Vec<f64> = (1..=60).map(|i| i as f64).collect();
    let polar = glide_polar(thrust, rho, area, p0, flat_plate, &speeds);

    // Minimum sink in forward flight is well below the vertical autorotation rate.
    assert!(polar.min_sink.descent_rate_ms < 0.7 * vertical.descent_rate_ms);
    assert!(polar.min_sink.airspeed_ms > 0.0);

    // Best-glide (shallowest angle) occurs at a *higher* speed than minimum sink —
    // the textbook ordering of the two autorotation reference speeds.
    assert!(polar.best_glide.airspeed_ms > polar.min_sink.airspeed_ms);
    // ...and at best glide the descent is faster than at min-sink (further down
    // the bucket toward higher speed), but the angle is shallower.
    assert!(polar.best_glide.descent_rate_ms >= polar.min_sink.descent_rate_ms);
    assert!(polar.best_glide.glide_angle_deg < 90.0 && polar.best_glide.glide_angle_deg > 0.0);
}

#[test]
fn min_sink_rate_is_consistent_with_hover_induced_scale() {
    // Sanity: the forward min-sink rate is a small multiple of v_h, not absurd.
    let (thrust, rho, area, p0) = representative_case();
    let v_h = hover_induced_velocity(thrust, rho, area);
    let speeds: Vec<f64> = (1..=60).map(|i| i as f64).collect();
    let polar = glide_polar(thrust, rho, area, p0, 1.0, &speeds);
    assert!(polar.min_sink.descent_rate_ms > 0.0);
    assert!(polar.min_sink.descent_rate_ms < 2.0 * v_h);
}

#[test]
fn flare_energy_margin_is_positive_and_index_orders_rotors() {
    // Stored rotor energy gives a positive flare-height margin, and for equal
    // stored energy the larger disk (lower disk loading) is more
    // autorotation-capable.
    let weight = 1000.0 * G;
    let inertia = 1500.0; // kg·m², representative main-rotor polar inertia
    let omega = 190.0 / 4.0; // tip speed / radius, rad/s
    let h = flare_height_equivalent(inertia, omega, weight);
    assert!(h > 0.0);

    let big = autorotation_index(inertia, omega, weight, PI * 4.0 * 4.0);
    let small = autorotation_index(inertia, omega, weight, PI * 3.0 * 3.0);
    assert!(big > small);
}

#[test]
fn glide_polar_skips_nonpositive_airspeeds() {
    // The glide-polar builder ignores v ≤ 0 (the continue branch) and still finds
    // the min-sink / best-glide at the positive speeds.
    let (thrust, rho, area, p0) = representative_case();
    let speeds = [-5.0, 0.0, 10.0, 20.0, 30.0, 40.0];
    let polar = glide_polar(thrust, rho, area, p0, 1.0, &speeds);
    assert!(polar.min_sink.airspeed_ms > 0.0);
    assert!(polar.best_glide.airspeed_ms > 0.0);
}
