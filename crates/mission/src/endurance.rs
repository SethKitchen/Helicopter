//! Constant-power discharge integration → endurance and (optionally) cell
//! temperature.
//!
//! Holding mechanical power constant, the pack is a constant-power load. At each
//! step the coupled current/voltage is solved ([`crate::electrical`]), charge is
//! removed, and — when a thermal model is supplied — the per-cell I²R heat is fed
//! into the lumped thermal balance. The loop runs to the cut-off voltage (or an
//! optional time cap, for a fixed-duration climb).

use crate::electrical::{solve_pack_current, solve_pack_current_at};
use helisim_pack::Pack;
use helisim_thermal::{Cooling, LumpedThermalCell, ThermalLimits};

/// Integration settings.
#[derive(Clone, Copy, Debug)]
pub struct MissionConfig {
    /// Time step, seconds.
    pub dt_seconds: f64,
    /// Starting state of charge in (0, 1].
    pub start_soc: f64,
    /// Stop terminal voltage as a multiple of cut-off (1.0 = exactly cut-off).
    pub cutoff_margin: f64,
    /// Ambient temperature for the thermal model, °C.
    pub ambient_c: f64,
    /// Use the tracked cell temperature for pack resistance (closing the
    /// temperature→R→sag→heat→temperature loop). **Default `false`** — the default
    /// thermal/safety check uses the conservative 25 °C resistance (warming a cell
    /// LOWERS its R, which would *under*-predict self-heating, an optimistic bias
    /// not wanted in an overheat check). Enable to analyse a hot/cold pack.
    pub temp_dependent_resistance: bool,
}

impl Default for MissionConfig {
    fn default() -> Self {
        MissionConfig {
            dt_seconds: 1.0,
            start_soc: 1.0,
            cutoff_margin: 1.0,
            ambient_c: 25.0,
            temp_dependent_resistance: false,
        }
    }
}

/// Energy/endurance outcome of a discharge.
#[derive(Clone, Copy, Debug)]
pub struct EnduranceResult {
    /// Whether the pack could supply the demand for any time at all.
    pub feasible: bool,
    /// Discharge duration, minutes.
    pub endurance_min: f64,
    /// Mean pack current, amps.
    pub mean_pack_current: f64,
    /// Peak pack current (at the end / lowest voltage), amps.
    pub peak_pack_current: f64,
    /// Peak per-cell C-rate (1/h).
    pub peak_cell_c_rate: f64,
    /// Energy delivered to the load, watt-hours.
    pub energy_delivered_wh: f64,
    /// State of charge when discharge stopped.
    pub end_soc: f64,
}

/// Cell-temperature outcome of a discharge.
#[derive(Clone, Copy, Debug)]
pub struct ThermalResult {
    /// Peak cell temperature reached, °C.
    pub peak_temp_c: f64,
    /// Cell temperature when the discharge stopped, °C.
    pub final_temp_c: f64,
    /// Time to first exceed the recommended limit, s (None if never).
    pub time_to_warn_s: Option<f64>,
    /// Time to first exceed the absolute maximum, s (None if never).
    pub time_to_over_temp_s: Option<f64>,
    /// True if the peak stayed at or below the absolute maximum.
    pub within_limits: bool,
}

/// Optional thermal inputs for the discharge integrator.
struct ThermalInputs<'a> {
    lump: LumpedThermalCell,
    cooling: &'a dyn Cooling,
    limits: ThermalLimits,
}

/// Core integrator: discharge `pack` at constant `p_elec` watts until cut-off or
/// `max_time_s` (if given). With `thermal` supplied, also tracks cell temperature.
fn run_discharge(
    pack: &Pack,
    p_elec: f64,
    max_time_s: Option<f64>,
    cfg: &MissionConfig,
    thermal: Option<ThermalInputs>,
) -> (EnduranceResult, Option<ThermalResult>) {
    let dt_h = cfg.dt_seconds / 3600.0;
    let capacity_ah = pack.capacity_ah();
    let stop_v = pack.cutoff_voltage() * cfg.cutoff_margin;
    let cap_time = max_time_s.unwrap_or(86_400.0);

    let mut soc = cfg.start_soc;
    let mut time_s = 0.0;
    let mut current_integral = 0.0;
    let mut peak_current = 0.0_f64;
    let mut energy_wh = 0.0;

    let mut temp = cfg.ambient_c;
    let mut peak_temp = cfg.ambient_c;
    let mut time_to_warn = None;
    let mut time_to_over = None;

    loop {
        // When a thermal model is tracking cell temperature, solve the current at
        // the CURRENT temperature (cold pack ⇒ higher R ⇒ more sag/current) — this
        // closes the temperature→resistance→sag→heat→temperature loop. Without a
        // thermal model, fall back to the 25 °C solve (unchanged behaviour).
        let state = if cfg.temp_dependent_resistance && thermal.is_some() {
            solve_pack_current_at(pack, soc, p_elec, temp)
        } else {
            solve_pack_current(pack, soc, p_elec)
        };
        let Some(state) = state else { break };
        if state.terminal_voltage <= stop_v || soc <= 0.0 || time_s >= cap_time {
            break;
        }

        let i = state.pack_current;
        peak_current = peak_current.max(i);
        let soc_at_solve = soc; // resistance evaluated at the SoC the current was solved at
        let dq = i * dt_h;
        current_integral += i * cfg.dt_seconds;
        energy_wh += state.terminal_voltage * i * dt_h;
        soc -= dq / capacity_ah;

        // Thermal step (per-cell I²R into the lumped balance), with the resistance
        // at the tracked temperature so warming reduces R (self-limiting heating).
        if let Some(th) = thermal.as_ref() {
            let i_cell = pack.cell_current(i);
            let r_cell = if cfg.temp_dependent_resistance {
                pack.cell_resistance_at(soc_at_solve, temp)
            } else {
                pack.cell_resistance(soc_at_solve)
            };
            // Joule (I²R) + reversible (entropic) heat; the latter is 0 unless the
            // cell carries a measured ∂OCV/∂T.
            let q_gen =
                i_cell * i_cell * r_cell + pack.cell_reversible_heat(soc_at_solve, i_cell, temp);
            temp = th.lump.step(temp, q_gen, th.cooling, cfg.dt_seconds);
            peak_temp = peak_temp.max(temp);
            if time_to_warn.is_none() && temp > th.limits.warn_c {
                time_to_warn = Some(time_s);
            }
            if time_to_over.is_none() && temp > th.limits.max_c {
                time_to_over = Some(time_s);
            }
        }

        time_s += cfg.dt_seconds;
    }

    let feasible = time_s > 0.0;
    let endurance = EnduranceResult {
        feasible,
        endurance_min: time_s / 60.0,
        mean_pack_current: if feasible {
            current_integral / time_s
        } else {
            0.0
        },
        peak_pack_current: peak_current,
        peak_cell_c_rate: pack.cell_c_rate(peak_current),
        energy_delivered_wh: energy_wh,
        end_soc: soc.max(0.0),
    };

    let thermal_result = thermal.map(|th| ThermalResult {
        peak_temp_c: peak_temp,
        final_temp_c: temp,
        time_to_warn_s: time_to_warn,
        time_to_over_temp_s: time_to_over,
        within_limits: peak_temp <= th.limits.max_c,
    });

    (endurance, thermal_result)
}

/// Discharge to cut-off at constant power; energy/endurance only.
pub fn simulate_hover_endurance(pack: &Pack, p_elec: f64, cfg: &MissionConfig) -> EnduranceResult {
    run_discharge(pack, p_elec, None, cfg, None).0
}

/// Discharge at constant power with cell-temperature tracking, optionally capped
/// at `max_time_s` (e.g. a fixed-duration climb). Returns both outcomes.
pub fn simulate_discharge_thermal(
    pack: &Pack,
    p_elec: f64,
    max_time_s: Option<f64>,
    cooling: &dyn Cooling,
    limits: ThermalLimits,
    cfg: &MissionConfig,
) -> (EnduranceResult, ThermalResult) {
    let lump = LumpedThermalCell::new(
        pack.cell_heat_capacity(),
        pack.cell_surface_area(),
        cfg.ambient_c,
    );
    let (e, t) = run_discharge(
        pack,
        p_elec,
        max_time_s,
        cfg,
        Some(ThermalInputs {
            lump,
            cooling,
            limits,
        }),
    );
    (e, t.unwrap())
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_cell::TheveninCell;
    use helisim_thermal::Convective;

    fn pack() -> Pack {
        Pack::new(Box::new(TheveninCell::samsung_25r()), 6, 3)
    }

    #[test]
    fn lower_power_lasts_longer() {
        let p = pack();
        let cfg = MissionConfig::default();
        let a = simulate_hover_endurance(&p, 300.0, &cfg);
        let b = simulate_hover_endurance(&p, 600.0, &cfg);
        assert!(a.feasible && b.feasible);
        assert!(a.endurance_min > b.endurance_min);
    }

    #[test]
    fn endurance_roughly_matches_energy_over_power() {
        let p = pack();
        let cfg = MissionConfig::default();
        let r = simulate_hover_endurance(&p, 400.0, &cfg);
        let hours = r.energy_delivered_wh / 400.0;
        assert!((hours * 60.0 - r.endurance_min).abs() < 0.5);
    }

    #[test]
    fn higher_power_runs_hotter() {
        let p = pack();
        let cfg = MissionConfig::default();
        let cooling = Convective::natural_air();
        let limits = ThermalLimits::default();
        let (_, lo) = simulate_discharge_thermal(&p, 300.0, None, &cooling, limits, &cfg);
        let (_, hi) = simulate_discharge_thermal(&p, 1500.0, None, &cooling, limits, &cfg);
        assert!(hi.peak_temp_c > lo.peak_temp_c);
    }
}
