//! Four-cell benchmark: run every cell in the library through [`size_for_target`]
//! for one [`Target`] and tabulate the trade. This is where the apples-to-apples
//! comparison the user asked for lands — and it is *protection-aware*, sizing
//! against each cell's TRUE continuous rating, not its flattering datasheet label.

use crate::sizing::{PackSizing, Target, size_for_target};
use helisim_cell::{benchmark_cells, true_continuous_current};

/// One cell's result for a given target.
#[derive(Clone, Debug)]
pub struct BenchmarkRow {
    pub name: &'static str,
    /// True (de-rated) continuous current the pack was sized against, A.
    pub true_continuous_a: f64,
    pub sizing: PackSizing,
    /// Gravimetric energy density of the sized pack (cells only), Wh/kg.
    pub pack_wh_per_kg: f64,
}

/// Benchmark all four library cells against `target`, sorted lightest pack first.
pub fn run_benchmark(target: Target) -> Vec<BenchmarkRow> {
    let mut rows: Vec<BenchmarkRow> = benchmark_cells()
        .into_iter()
        .map(|(name, cell)| {
            let tc = true_continuous_current(name).expect("library cell has a true rating");
            let sizing = size_for_target(cell.as_ref(), tc, target);
            let pack_wh_per_kg = sizing.energy_wh / sizing.mass_kg;
            BenchmarkRow {
                name,
                true_continuous_a: tc,
                sizing,
                pack_wh_per_kg,
            }
        })
        .collect();
    rows.sort_by(|a, b| a.sizing.mass_kg.partial_cmp(&b.sizing.mass_kg).unwrap());
    rows
}

/// The lightest pack for `target` and the cell that produced it.
pub fn best_by_mass(target: Target) -> BenchmarkRow {
    run_benchmark(target).into_iter().next().unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sizing::Limiting;

    /// Every sized pack meets the target and stays inside the safe envelope.
    #[test]
    fn all_rows_meet_target_and_envelope() {
        let target = Target {
            bus_voltage_v: 400.0,
            peak_power_w: 120_000.0,
            energy_wh: 30_000.0,
        };
        for r in run_benchmark(target) {
            assert!(r.sizing.bus_nominal_v >= target.bus_voltage_v);
            assert!(r.sizing.energy_wh >= target.energy_wh - 1e-6);
            assert!(
                r.sizing.current_utilisation <= 1.0 + 1e-9,
                "{} util",
                r.name
            );
        }
    }

    /// The benchmark reproduces the energy-vs-power split the cells embody:
    /// energy-bound targets crown the P50B; power-bound targets crown the EVE 40PL.
    #[test]
    fn winner_follows_the_binding_requirement() {
        let energy_bound = Target {
            bus_voltage_v: 350.0,
            peak_power_w: 5_000.0,
            energy_wh: 60_000.0,
        };
        assert_eq!(best_by_mass(energy_bound).name, "Molicel P50B");

        let power_bound = Target {
            bus_voltage_v: 350.0,
            peak_power_w: 250_000.0,
            energy_wh: 500.0,
        };
        assert_eq!(best_by_mass(power_bound).name, "EVE 40PL");
    }

    /// Honesty check: against the BAK 45D's true 30 A rating a power-bound target
    /// needs more copper than its 60 A *label* would suggest — sizing on the label
    /// would under-build the pack. We assert the label would have cut the parallel
    /// count, i.e. the de-rate is load-bearing, not cosmetic.
    #[test]
    fn label_rating_would_underbuild() {
        use crate::sizing::size_for_target;
        use helisim_cell::{Cell, bak_45d};
        let target = Target {
            bus_voltage_v: 200.0,
            peak_power_w: 100_000.0,
            energy_wh: 1_000.0,
        };
        let cell = bak_45d();
        let honest = size_for_target(&cell, 30.0, target); // true continuous
        let label = size_for_target(&cell, cell.max_continuous_current(), target); // 60 A label
        assert_eq!(honest.limiting, Limiting::Power);
        assert!(
            honest.parallel > label.parallel,
            "honest {} label {}",
            honest.parallel,
            label.parallel
        );
    }
}
