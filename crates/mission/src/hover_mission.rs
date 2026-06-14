//! The end-to-end hover analysis: rotor + pack + powertrain + gross mass →
//! "can it hover, at what C-rate, for how long".

use crate::G;
use crate::electrical::solve_pack_current;
use crate::endurance::{EnduranceResult, MissionConfig, simulate_discharge_thermal};
use crate::hover_trim::trim_hover_collective;
use helisim_airfoil::Airfoil;
use helisim_bemt::Config;
use helisim_pack::Pack;
use helisim_powertrain::Powertrain;
use helisim_rotor::{Operating, Rotor};
use helisim_thermal::{Cooling, ThermalLimits, ThermalStatus};

/// Everything the end-to-end chain produces for one hover design point.
#[derive(Clone, Debug)]
pub struct HoverReport {
    /// Whether the rotor can be trimmed to hover the gross mass at this RPM.
    pub hover_feasible: bool,
    /// Gross mass, kg.
    pub gross_mass_kg: f64,
    /// Required hover thrust (= weight), N.
    pub required_thrust_n: f64,
    /// Trimmed collective, degrees.
    pub collective_deg: f64,
    /// Rotor figure of merit at the hover point.
    pub figure_of_merit: f64,
    /// Mechanical (shaft) power, W.
    pub mech_power_w: f64,
    /// Electrical power drawn from the pack, W.
    pub elec_power_w: f64,
    /// Pack terminal voltage in hover at full charge, V.
    pub hover_pack_voltage: f64,
    /// Pack current in hover at full charge, A.
    pub hover_pack_current: f64,
    /// Per-cell C-rate in hover (1/h).
    pub hover_cell_c_rate: f64,
    /// Cell continuous C-rate rating (1/h).
    pub continuous_c_rating: f64,
    /// Whether the hover current is within the pack's continuous rating.
    pub within_continuous_rating: bool,
    /// Pack mass, kg.
    pub pack_mass_kg: f64,
    /// Hover endurance result.
    pub endurance: EnduranceResult,
    /// Peak cell temperature over a full hover discharge, °C.
    pub hover_peak_temp_c: f64,
    /// Thermal status of the hover peak temperature.
    pub hover_thermal_status: ThermalStatus,
}

/// Run the full chain for a hover design point. `rotor` carries the geometry
/// (collective is overwritten by trim); `op` sets RPM/air; `airfoil` the section
/// aero; `pack`/`powertrain` the electrical side; `gross_mass_kg` the weight to
/// support.
#[allow(clippy::too_many_arguments)]
pub fn analyze_hover(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    pack: &Pack,
    powertrain: &dyn Powertrain,
    gross_mass_kg: f64,
    cooling: &dyn Cooling,
    limits: ThermalLimits,
    bemt_cfg: &Config,
    mission_cfg: &MissionConfig,
) -> HoverReport {
    let required_thrust_n = gross_mass_kg * G;
    let pack_mass_kg = pack.mass_kg();

    // 1. Trim the rotor so thrust = weight; get mechanical power.
    let trim = trim_hover_collective(rotor, op, airfoil, required_thrust_n, bemt_cfg);
    let Some((theta, sol)) = trim else {
        return HoverReport {
            hover_feasible: false,
            gross_mass_kg,
            required_thrust_n,
            collective_deg: f64::NAN,
            figure_of_merit: f64::NAN,
            mech_power_w: f64::NAN,
            elec_power_w: f64::NAN,
            hover_pack_voltage: f64::NAN,
            hover_pack_current: f64::NAN,
            hover_cell_c_rate: f64::NAN,
            continuous_c_rating: pack.continuous_c_rating(),
            within_continuous_rating: false,
            pack_mass_kg,
            endurance: EnduranceResult {
                feasible: false,
                endurance_min: 0.0,
                mean_pack_current: 0.0,
                peak_pack_current: 0.0,
                peak_cell_c_rate: 0.0,
                energy_delivered_wh: 0.0,
                end_soc: mission_cfg.start_soc,
            },
            hover_peak_temp_c: f64::NAN,
            hover_thermal_status: ThermalStatus::Safe,
        };
    };

    // 2. Mechanical -> electrical power.
    let mech_power_w = sol.power;
    let elec_power_w = powertrain.electrical_power(mech_power_w);

    // 3. Coupled hover operating point at the starting state of charge.
    let hover = solve_pack_current(pack, mission_cfg.start_soc, elec_power_w);
    let (hover_pack_voltage, hover_pack_current) = match hover {
        Some(s) => (s.terminal_voltage, s.pack_current),
        None => (f64::NAN, f64::NAN),
    };
    let hover_cell_c_rate = pack.cell_c_rate(hover_pack_current);
    let continuous_c_rating = pack.continuous_c_rating();
    let within_continuous_rating =
        hover_pack_current.is_finite() && hover_cell_c_rate <= continuous_c_rating;

    // 4. Endurance + hover thermal transient (run to cut-off).
    let (endurance, thermal) =
        simulate_discharge_thermal(pack, elec_power_w, None, cooling, limits, mission_cfg);

    HoverReport {
        hover_feasible: true,
        gross_mass_kg,
        required_thrust_n,
        collective_deg: theta.to_degrees(),
        figure_of_merit: sol.figure_of_merit,
        mech_power_w,
        elec_power_w,
        hover_pack_voltage,
        hover_pack_current,
        hover_cell_c_rate,
        continuous_c_rating,
        within_continuous_rating,
        pack_mass_kg,
        endurance,
        hover_peak_temp_c: thermal.peak_temp_c,
        hover_thermal_status: limits.classify(thermal.peak_temp_c),
    }
}
