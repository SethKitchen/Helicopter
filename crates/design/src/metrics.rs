//! Evaluate a design point: compose the validated cores into the priority vector.
//!
//! This module adds **no new physics**. It trims the rotor to hover with the
//! validated hover BEMT ([`helisim_mission::trim_hover_collective`]), reads the
//! autorotation margin from [`helisim_autorotation`], and the radiated noise from
//! [`helisim_acoustics`]. Its only job — and the thing its tests check — is that it
//! wires those trusted models together correctly and surfaces the design
//! tensions: a bigger disk is quieter, more efficient and safer in autorotation
//! but heavier and slower; a faster tip is lighter and cheaper but louder.

use crate::candidate::DesignCandidate;
use crate::report::DesignReport;
use helisim_acoustics::{P_REF, rotational_spectrum};
use helisim_airfoil::Airfoil;
use helisim_autorotation::{
    G, assess_vertical, autorotation_index, decay_time_constant_power, flare_height_equivalent,
    glide_polar, profile_power, steady_autorotation,
};
use helisim_bemt::Config;
use helisim_cost::{AircraftSpec, UnitCosts, build_bom, summarize};
use helisim_mission::trim_hover_collective;

/// Effective loading radius for Gutin noise, as a fraction of rotor radius.
const R_EFF_FRACTION: f64 = 0.8;
/// Number of blade-passage harmonics to sum for the overall noise level.
const N_HARMONICS: usize = 6;
/// Minimum controllable rotor speed for the flare, as a fraction of nominal.
const OMEGA_MIN_FRAC: f64 = 0.7;
/// Assumed pilot/sensor reaction delay before the flare, s.
const REACTION_DELAY_S: f64 = 1.0;
/// Acceptable touchdown descent rate, m/s.
const SAFE_TOUCHDOWN_MS: f64 = 2.0;

/// Compute the full [`DesignReport`] for a candidate against an airfoil and BEMT
/// config. Safety (autorotation) is computed even when the rotor cannot hover, so
/// an infeasible point still reports its descent/flare physics.
pub fn evaluate(c: &DesignCandidate, airfoil: &dyn Airfoil, cfg: &Config) -> DesignReport {
    let op = c.operating();
    let area = c.disk_area();
    let weight = c.gross_mass_kg * G;
    let omega = c.omega();
    let tip_mach = c.tip_speed_ms / op.sound_speed;

    // --- priority 1: autorotation (independent of being able to hover) ---
    let p0 = profile_power(op.rho, area, c.tip_speed_ms, c.solidity(), c.blade_cd0);
    let auto = steady_autorotation(weight, op.rho, area, p0);
    let flare_height_m = flare_height_equivalent(c.rotor_inertia, omega, weight);
    let auto_index = autorotation_index(c.rotor_inertia, omega, weight, area);
    let flare = assess_vertical(
        weight,
        c.gross_mass_kg,
        c.rotor_inertia,
        omega,
        OMEGA_MIN_FRAC,
        op.rho,
        area,
        p0,
        REACTION_DELAY_S,
        SAFE_TOUCHDOWN_MS,
    );

    // Forward-flight glide polar (the realistic, gentler case). Speeds 0.5..40 m/s
    // span the bucket for model through light-helicopter scales.
    let speeds: Vec<f64> = (1..=80).map(|i| i as f64 * 0.5).collect();
    let polar = glide_polar(weight, op.rho, area, p0, c.flat_plate_area_m2, &speeds);

    let mut report = DesignReport {
        hover_feasible: false,
        collective_deg: f64::NAN,
        autorotation_descent_fpm: auto.descent_rate_fpm(),
        autorotation_ratio: auto.descent_ratio,
        forward_min_sink_fpm: polar.min_sink.descent_rate_ms * 196.850393,
        forward_min_sink_speed_ms: polar.min_sink.airspeed_ms,
        best_glide_speed_ms: polar.best_glide.airspeed_ms,
        best_glide_angle_deg: polar.best_glide.glide_angle_deg,
        flare_height_m,
        autorotation_index: auto_index,
        flare_margin: flare.flare_margin,
        can_flare: flare.can_flare,
        rotor_decay_time_s: f64::NAN,
        hover_shaft_power_w: f64::NAN,
        hover_elec_power_w: f64::NAN,
        endurance_min: 0.0,
        figure_of_merit: f64::NAN,
        disk_loading: weight / area,
        power_loading: f64::NAN,
        oaspl_db: f64::NAN,
        blade_passage_hz: c.n_blades as f64 * omega / (2.0 * std::f64::consts::PI),
        tip_mach,
        total_cost: f64::NAN,
        vertical_integration_index: f64::NAN,
        purchased_cost_fraction: f64::NAN,
    };

    // --- hover trim → power, airtime, efficiency, noise ---
    let Some((theta, sol)) = trim_hover_collective(&c.rotor(), &op, airfoil, weight, cfg) else {
        return report; // infeasible: safety physics already filled, rest stays NaN
    };
    let shaft_power = sol.power;
    let elec_power = shaft_power / c.powertrain_eta;
    let usable_wh = c.pack_energy_wh * c.usable_fraction;
    let endurance_min = if elec_power > 0.0 {
        usable_wh / elec_power * 60.0
    } else {
        0.0
    };

    // Noise: the hover torque Q = P/Ω rotates with the loading (Gutin).
    let torque = shaft_power / omega;
    let spectrum = rotational_spectrum(
        N_HARMONICS,
        c.n_blades,
        omega,
        op.sound_speed,
        c.observer_distance_m,
        weight,
        torque,
        R_EFF_FRACTION * c.radius_m,
        c.observer_angle_deg.to_radians(),
    );

    // Cost + vertical integration (priorities #2/#3). Documented mass split of the
    // gross mass + installed power = 2× hover (climb margin).
    let m = c.gross_mass_kg;
    let spec = AircraftSpec {
        n_blades: c.n_blades,
        blade_mass_kg: 0.03 * m / c.n_blades as f64,
        hub_mass_kg: 0.05 * m,
        structure_mass_kg: 0.40 * m,
        powertrain_mass_kg: 0.12 * m,
        motor_power_kw: 2.0 * shaft_power / 1000.0,
        pack_energy_wh: c.pack_energy_wh,
        pack_mass_kg: 0.25 * m,
    };
    let cost = summarize(&build_bom(&spec, &UnitCosts::default()));

    report.hover_feasible = true;
    report.collective_deg = theta.to_degrees();
    report.total_cost = cost.total_cost;
    report.vertical_integration_index = cost.vertical_integration_index;
    report.purchased_cost_fraction = cost.purchased_cost_fraction;
    report.rotor_decay_time_s =
        decay_time_constant_power(c.rotor_inertia, omega, OMEGA_MIN_FRAC * omega, shaft_power);
    report.hover_shaft_power_w = shaft_power;
    report.hover_elec_power_w = elec_power;
    report.endurance_min = endurance_min;
    report.figure_of_merit = sol.figure_of_merit;
    report.power_loading = weight / shaft_power;
    // Guard against a degenerate all-zero spectrum producing -inf dB.
    report.oaspl_db = if spectrum.oaspl_db.is_finite() {
        spectrum.oaspl_db
    } else {
        20.0 * (f64::MIN_POSITIVE / P_REF).log10()
    };
    report
}
