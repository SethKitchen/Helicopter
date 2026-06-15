//! Parametric pack sizing — the heart of the "scales from model to human" goal.
//! Given a cell and a [`Target`] of (bus voltage, peak power, required energy),
//! pick the smallest series/parallel topology that meets all three *while keeping
//! every cell inside its true continuous-current limit*. The exact same call sizes
//! a 22 V / 1 kW model pack and a 700 V / 200 kW human-scale pack — only the target
//! changes. No fixed aircraft is baked in.
//!
//! The S/P scaling is identical to [`helisim_pack::Pack`]; sizing just inverts it
//! (solve for S and P from the requirements instead of reporting a chosen pack).

use helisim_cell::Cell;

/// What the pack must deliver. All three are hard requirements; sizing satisfies
/// the binding one.
#[derive(Clone, Copy, Debug)]
pub struct Target {
    /// Minimum nominal bus voltage, V (sets series count).
    pub bus_voltage_v: f64,
    /// Peak electrical power the pack must source, W (sets parallel via current).
    pub peak_power_w: f64,
    /// Energy that must be stored (nominal), Wh (can also set parallel).
    pub energy_wh: f64,
}

/// Which requirement set the parallel count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Limiting {
    /// Peak-power (current) requirement dominated.
    Power,
    /// Stored-energy requirement dominated.
    Energy,
}

/// A sized pack and the per-cell duty it implies.
#[derive(Clone, Debug)]
pub struct PackSizing {
    pub series: usize,
    pub parallel: usize,
    pub cell_count: usize,
    pub mass_kg: f64,
    /// Nominal bus voltage, V (`S × cell nominal`).
    pub bus_nominal_v: f64,
    /// Stored nominal energy, Wh (`S·P × cell nominal Wh`).
    pub energy_wh: f64,
    /// Per-cell current at peak power, A.
    pub peak_cell_current_a: f64,
    /// Per-cell C-rate at peak power, 1/h.
    pub peak_cell_c_rate: f64,
    /// `peak_cell_current / true_continuous` — ≤1 means inside the safe envelope.
    pub current_utilisation: f64,
    /// Per-cell ohmic heat at peak, `I²R` (W) — the thermal-load proxy.
    pub peak_cell_heat_w: f64,
    /// Which requirement bound the parallel count.
    pub limiting: Limiting,
}

/// Size a pack of `cell` (continuous-limited to `true_continuous_a` per cell) to
/// meet `target`.
pub fn size_for_target(cell: &dyn Cell, true_continuous_a: f64, target: Target) -> PackSizing {
    let v_nom = cell.nominal_voltage();
    let cap_ah = cell.capacity_ah();
    let r = cell.internal_resistance(0.5);
    let cell_wh = v_nom * cap_ah;
    // Peak bursts are sized at a mid-SoC operating point (where the OCV ≈ nominal).
    let ocv_peak = cell.ocv(0.5);

    // Series sets the bus voltage.
    let series = (target.bus_voltage_v / v_nom).ceil().max(1.0) as usize;
    let bus_nominal_v = series as f64 * v_nom;

    // Parallel is the max of what power and energy each demand.
    // Power: under a peak load the cell voltage SAGS to `OCV − I·R`, so the power a
    // cell can source AT its continuous-current limit is `(OCV − I_cont·R)·I_cont`,
    // NOT the un-sagged `V_nom·I_cont`. Sizing against the sagged value is the
    // conservative (correct) choice — using nominal under-counts current and can
    // certify a pack that actually exceeds the cell limit.
    let p_cell_at_limit = ((ocv_peak - true_continuous_a * r) * true_continuous_a).max(1e-9);
    let p_for_power = (target.peak_power_w / (series as f64 * p_cell_at_limit)).ceil();
    // Energy: total stored = S·P·cell_wh ≥ energy_wh.
    let p_for_energy = (target.energy_wh / (series as f64 * cell_wh)).ceil();

    let parallel = p_for_power.max(p_for_energy).max(1.0) as usize;
    let limiting = if p_for_power >= p_for_energy {
        Limiting::Power
    } else {
        Limiting::Energy
    };

    let p = parallel as f64;
    // True per-cell current at the peak constant-power load, including sag: each
    // cell delivers `P_cell = P_peak/(S·P)` at `V = OCV − I·R`, so
    // `r·I² − OCV·I + P_cell = 0 ⇒ I = (OCV − √(OCV² − 4r·P_cell))/(2r)`.
    let p_cell = target.peak_power_w / (series as f64 * p);
    let disc = (ocv_peak * ocv_peak - 4.0 * r * p_cell).max(0.0);
    let peak_cell_current_a = (ocv_peak - disc.sqrt()) / (2.0 * r);
    PackSizing {
        series,
        parallel,
        cell_count: series * parallel,
        mass_kg: (series * parallel) as f64 * cell.mass_kg(),
        bus_nominal_v,
        energy_wh: series as f64 * p * cell_wh,
        peak_cell_current_a,
        peak_cell_c_rate: peak_cell_current_a / cap_ah,
        current_utilisation: peak_cell_current_a / true_continuous_a,
        peak_cell_heat_w: peak_cell_current_a * peak_cell_current_a * r,
        limiting,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_cell::{bak_45d, eve_40pl, molicel_p50b, true_continuous_current};

    fn tc(name: &str) -> f64 {
        true_continuous_current(name).unwrap()
    }

    /// The same function sizes both ends of the scale, and sizing always keeps the
    /// cell inside its continuous limit (utilisation ≤ 1).
    #[test]
    fn scales_model_to_human_within_envelope() {
        let cell = molicel_p50b();
        let model = Target {
            bus_voltage_v: 22.0,
            peak_power_w: 1500.0,
            energy_wh: 150.0,
        };
        let human = Target {
            bus_voltage_v: 700.0,
            peak_power_w: 200_000.0,
            energy_wh: 50_000.0,
        };
        let sm = size_for_target(&cell, tc("Molicel P50B"), model);
        let sh = size_for_target(&cell, tc("Molicel P50B"), human);
        assert!(sm.current_utilisation <= 1.0 + 1e-9);
        assert!(sh.current_utilisation <= 1.0 + 1e-9);
        // Human pack is vastly bigger but built from the same cell.
        assert!(sh.cell_count > 100 * sm.cell_count);
        assert!(sh.bus_nominal_v >= 700.0 && sm.bus_nominal_v >= 22.0);
    }

    /// Power-dominated target: the cell with the highest TRUE continuous rating
    /// (EVE 40PL, 70 A) needs the fewest parallel strings, so the lightest pack —
    /// even though it stores less per cell than the BAK 45D.
    #[test]
    fn power_dominated_favours_high_continuous_cell() {
        let target = Target {
            bus_voltage_v: 100.0,
            peak_power_w: 80_000.0,
            energy_wh: 200.0, // tiny energy → power binds
        };
        let eve = size_for_target(&eve_40pl(), tc("EVE 40PL"), target);
        let bak = size_for_target(&bak_45d(), tc("BAK 45D"), target);
        assert_eq!(eve.limiting, Limiting::Power);
        assert_eq!(bak.limiting, Limiting::Power);
        assert!(
            eve.parallel < bak.parallel,
            "eve {} bak {}",
            eve.parallel,
            bak.parallel
        );
        assert!(eve.mass_kg < bak.mass_kg);
    }

    /// Sag-aware sizing is conservative: the reported peak per-cell current
    /// (constant-power, with sag) exceeds the naive un-sagged `P/(S·P·V_nom)`
    /// estimate, yet the pack is sized so it still stays inside the continuous
    /// limit (utilisation ≤ 1) — i.e. the optimism is removed, not papered over.
    #[test]
    fn peak_current_is_sag_aware_and_still_within_limit() {
        let cell = eve_40pl();
        let target = Target {
            bus_voltage_v: 100.0,
            peak_power_w: 60_000.0,
            energy_wh: 500.0, // power binds
        };
        let s = size_for_target(&cell, tc("EVE 40PL"), target);
        assert_eq!(s.limiting, Limiting::Power);
        // The naive (un-sagged) per-cell current the old code reported.
        let naive = target.peak_power_w / (s.bus_nominal_v * s.parallel as f64);
        assert!(
            s.peak_cell_current_a > naive,
            "sag must raise current above the nominal estimate ({} vs {})",
            s.peak_cell_current_a,
            naive
        );
        // ...but sizing kept it inside the cell's continuous rating.
        assert!(s.current_utilisation <= 1.0 + 1e-9);
    }

    /// Energy-dominated target: the highest-capacity cell (P50B, 5.0 Ah) needs the
    /// fewest parallel strings and the lightest pack.
    #[test]
    fn energy_dominated_favours_high_capacity_cell() {
        let target = Target {
            bus_voltage_v: 100.0,
            peak_power_w: 2_000.0, // tiny power → energy binds
            energy_wh: 40_000.0,
        };
        let p50 = size_for_target(&molicel_p50b(), tc("Molicel P50B"), target);
        let eve = size_for_target(&eve_40pl(), tc("EVE 40PL"), target);
        assert_eq!(p50.limiting, Limiting::Energy);
        assert!(
            p50.parallel < eve.parallel,
            "p50 {} eve {}",
            p50.parallel,
            eve.parallel
        );
        assert!(p50.mass_kg < eve.mass_kg);
    }
}
