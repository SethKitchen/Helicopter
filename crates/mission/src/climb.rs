//! Sustained-climb thermal assessment — the safety-relevant question: does a
//! few minutes of full-power climb push the cells out of their safe band, even
//! when the current is within the C-rate rating?
//!
//! Climb power is modelled first-principles as hover power plus the rate of work
//! done against gravity: `P_mech_climb = P_hover + W · V_climb`. (A proper climb
//! BEMT arrives with the forward-flight milestone; this energy bound is the right
//! first cut for a thermal-feasibility check.)

use crate::G;
use crate::electrical::solve_pack_current;
use crate::endurance::{MissionConfig, simulate_discharge_thermal};
use crate::hover_trim::trim_hover_collective;
use helisim_airfoil::Airfoil;
use helisim_bemt::Config;
use helisim_pack::Pack;
use helisim_powertrain::Powertrain;
use helisim_rotor::{Operating, Rotor};
use helisim_thermal::{Cooling, ThermalLimits, ThermalStatus};

/// Outcome of a sustained-climb thermal check.
#[derive(Clone, Copy, Debug)]
pub struct ClimbReport {
    /// Whether the rotor can produce the climb power at all.
    pub feasible: bool,
    /// Commanded climb rate, m/s.
    pub climb_rate_mps: f64,
    /// Climb duration analysed, s.
    pub duration_s: f64,
    /// Mechanical climb power, W.
    pub mech_power_w: f64,
    /// Electrical climb power, W.
    pub elec_power_w: f64,
    /// Per-cell C-rate in the climb (1/h).
    pub cell_c_rate: f64,
    /// Whether that C-rate is within the cell's continuous rating.
    pub within_c_rating: bool,
    /// Peak cell temperature reached during the climb, °C.
    pub peak_temp_c: f64,
    /// Time into the climb at which the absolute max temp was first exceeded, s.
    pub time_to_over_temp_s: Option<f64>,
    /// Thermal status of the peak temperature.
    pub thermal_status: ThermalStatus,
}

/// Assess a sustained climb at `climb_rate_mps` held for `duration_s`.
#[allow(clippy::too_many_arguments)]
pub fn analyze_climb(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    pack: &Pack,
    powertrain: &dyn Powertrain,
    gross_mass_kg: f64,
    climb_rate_mps: f64,
    duration_s: f64,
    cooling: &dyn Cooling,
    limits: ThermalLimits,
    bemt_cfg: &Config,
    mission_cfg: &MissionConfig,
) -> ClimbReport {
    let weight = gross_mass_kg * G;

    let infeasible = |status| ClimbReport {
        feasible: false,
        climb_rate_mps,
        duration_s,
        mech_power_w: f64::NAN,
        elec_power_w: f64::NAN,
        cell_c_rate: f64::NAN,
        within_c_rating: false,
        peak_temp_c: f64::NAN,
        time_to_over_temp_s: None,
        thermal_status: status,
    };

    // Hover power at this weight (climb adds to it).
    let Some((_, sol)) = trim_hover_collective(rotor, op, airfoil, weight, bemt_cfg) else {
        return infeasible(ThermalStatus::Safe);
    };

    // Climb power = hover power + rate of work against gravity.
    let mech_power_w = sol.power + weight * climb_rate_mps;
    let elec_power_w = powertrain.electrical_power(mech_power_w);

    // Coupled hover-point current at the start of the climb (full charge).
    let Some(state) = solve_pack_current(pack, mission_cfg.start_soc, elec_power_w) else {
        return infeasible(ThermalStatus::OverTemp); // pack cannot source the power
    };
    let cell_c_rate = pack.cell_c_rate(state.pack_current);
    let within_c_rating = cell_c_rate <= pack.continuous_c_rating();

    // Thermal transient over the climb duration.
    let (_, thermal) = simulate_discharge_thermal(
        pack,
        elec_power_w,
        Some(duration_s),
        cooling,
        limits,
        mission_cfg,
    );

    ClimbReport {
        feasible: true,
        climb_rate_mps,
        duration_s,
        mech_power_w,
        elec_power_w,
        cell_c_rate,
        within_c_rating,
        peak_temp_c: thermal.peak_temp_c,
        time_to_over_temp_s: thermal.time_to_over_temp_s,
        thermal_status: limits.classify(thermal.peak_temp_c),
    }
}
