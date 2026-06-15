//! Benchmark cell library: four modern high-power **21700** cells, each built as
//! a [`TheveninCell`] from *sourced* datasheet + independent-bench-test numbers.
//!
//! These are an apples-to-apples set — all tabless (P50B steel-can) 21700 NMC/NCM
//! high-power cells — chosen to benchmark a battery + BMS that must scale from a
//! model helicopter to a human-carrying one. Like the Samsung 25R oracle, each
//! cell fixes only TWO fitted/measured parameters (the OCV curve and the series
//! resistance `R`); everything else (capacity, voltage window, mass, current
//! rating) is a published datasheet number a reader can hand-check against the
//! cited source.
//!
//! ## Sourcing (never fabricated)
//! * Capacity / voltage window / mass / label current: manufacturer datasheets.
//! * `R`: **measured DC internal resistance** from Battery Mooch's bench tests
//!   (E-Cigarette Forum / BudgetLightForum), NOT the datasheet's AC-1 kHz
//!   impedance. The AC figure (3–5 mΩ) is an under-estimate of the resistance a
//!   sustained load sees; the same reason the 25R model fits `R` (~21 mΩ) above
//!   its 14.8 mΩ DCIR. Here we use the *measured DCIR* directly as `R0`, so the
//!   single-`R` model slightly UNDER-predicts sustained-load sag (it omits the
//!   polarisation the 25R fit lumped in) — documented, not fudged.
//!
//! ## Honesty caveat — "max continuous" is thermal-cutoff-limited
//! Every datasheet's headline continuous current is rated *with an 80 °C cell
//! cutoff*, not a true-to-empty continuous rating. Independent testing puts the
//! true continuous rating well below the label (JP40 ~45 A vs 60 A label; BAK 45D
//! ~30 A vs 60 A). [`max_continuous_current`](crate::Cell::max_continuous_current)
//! holds the **datasheet label** (the sourceable number); the true continuous
//! rating lives in [`true_continuous_current`] and is what the BMS/benchmark
//! layer uses for an honest margin. This is the project's own thermal-track
//! finding ("the 75 °C limit bites before the C-rate limit") showing up directly
//! in the vendor ratings.
//!
//! ## Shared OCV curve
//! All four are the same NMC/NCM-graphite chemistry family, so they share one
//! representative OCV-SoC shape ([`representative_nmc_ocv`]) anchored to the
//! common 4.20 V / 2.50 V window and ~3.6 V mean — the same representative-shape
//! approach used for the 25R. Only `R`, capacity, mass and current rating
//! distinguish them; that is the first-cut modelling decision (a per-cell
//! measured OCV curve would refine it).
//!
//! NAMED GAP (not fabricated): per-cell measured OCV-SoC curves are not available
//! in clean numeric form (About:Energy/Voltt is login-gated; datasheet discharge
//! curves are published only as graphs), so the shared curve is retained. The API
//! already supports per-cell curves — [`TheveninCell::new`] takes one — so this is
//! a sourcing gap, not a structural one. See `crates/bms/tests/battery_external_validation.rs`.

use crate::cell::{Cell, SURFACE_AREA_21700_M2};
use crate::thevenin::TheveninCell;

/// Representative NMC/NCM-graphite OCV-SoC curve, `(soc, ocv)` ascending, anchored
/// to a 4.20 V full / 2.50 V empty window with a ~3.6 V mean. Shared by the
/// benchmark cells (same chemistry family); identical shape to the validated 25R.
pub fn representative_nmc_ocv() -> Vec<(f64, f64)> {
    vec![
        (0.00, 2.50),
        (0.05, 3.10),
        (0.10, 3.30),
        (0.20, 3.45),
        (0.30, 3.52),
        (0.40, 3.58),
        (0.50, 3.64),
        (0.60, 3.72),
        (0.70, 3.82),
        (0.80, 3.93),
        (0.90, 4.05),
        (1.00, 4.20),
    ]
}

/// **Molicel INR-21700-P50B** — the energy/cycle-life leader of the set.
///
/// Source: Molicel INR-21700-P50B datasheet — 5.0 Ah typ (4.85 Ah min), 3.6 V
/// nominal, 4.2/2.5 V window, 60 A max continuous (80 °C cutoff; ~35 A true
/// continuous to 2.5 V per Battery Mooch), 25 A charge, 71 g max (~70 g),
/// 257 Wh/kg. Measured DCIR ≈ **9.5 mΩ** (Battery Mooch) — the highest of the
/// four, but it holds the steadiest voltage at extreme power and has the longest
/// high-power cycle life. Released 2024 (Taiwan).
pub fn molicel_p50b() -> TheveninCell {
    TheveninCell::new(
        &representative_nmc_ocv(),
        0.0095,
        5.0,
        3.6,
        2.5,
        60.0,
        0.070,
    )
    .with_surface_area(SURFACE_AREA_21700_M2)
}

/// **Ampace INR21700-JP40** — one of the highest power-density cells available.
///
/// Source: Ampace JP40 (a.k.a. LT22710A) datasheet — 4.0 Ah typ (3.9 Ah min),
/// 3.6 V nominal, 4.2/2.5 V window, 60 A max continuous (80 °C cutoff; ~45 A true
/// continuous per bench testing), 140 A / 5 s pulse, 70 g, 215 Wh/kg, AC-1 kHz
/// impedance <4 mΩ. Measured DCIR ≈ **5.4 mΩ** (Battery Mooch).
pub fn ampace_jp40() -> TheveninCell {
    TheveninCell::new(
        &representative_nmc_ocv(),
        0.0054,
        4.0,
        3.6,
        2.5,
        60.0,
        0.070,
    )
    .with_surface_area(SURFACE_AREA_21700_M2)
}

/// **BAK N21700-45D** — mid-capacity, weakest *true* continuous rating of the set.
///
/// Source: BAK INR2170-45D datasheet — 4.5 Ah typ (4.4 Ah rated), 3.6 V nominal,
/// 4.2/2.5 V window, 60 A max discharge **with 80 °C cutoff / 30 A without** (the
/// datasheet states both — true continuous ≈ 30 A), ≤69 g, AC-1 kHz ≤5 mΩ.
/// Measured DCIR ≈ **6.0 mΩ** (5.6 & 6.3 mΩ, two cells, Battery Mooch).
pub fn bak_45d() -> TheveninCell {
    TheveninCell::new(
        &representative_nmc_ocv(),
        0.0060,
        4.5,
        3.6,
        2.5,
        60.0,
        0.069,
    )
    .with_surface_area(SURFACE_AREA_21700_M2)
}

/// **EVE INR21700-40PL** — lowest impedance / highest label current of the set.
///
/// Source: EVE 40PL datasheet — 4.0 Ah, 3.6 V nominal, 4.2/2.5 V window, 70 A max
/// continuous (80 °C cutoff), ~67 g, 215–218 Wh/kg, AC-1 kHz ≤5 mΩ. Measured DCIR
/// ≈ **5.1 mΩ** (Battery Mooch) — the lowest of the four. Released 2023 (China).
pub fn eve_40pl() -> TheveninCell {
    TheveninCell::new(
        &representative_nmc_ocv(),
        0.0051,
        4.0,
        3.6,
        2.5,
        70.0,
        0.067,
    )
    .with_surface_area(SURFACE_AREA_21700_M2)
}

/// True (de-rated) continuous current ratings, amps — the honest number behind
/// the thermal-cutoff-limited datasheet label. Sourced from independent bench
/// testing (Battery Mooch); where no separate de-rate was published the label is
/// kept. Used by the BMS protection envelope and the benchmark margin so the
/// comparison does not flatter a cell on a number it can't truly sustain.
///
/// * P50B  35 A (Battery Mooch's "true continuous to 2.5 V" estimate, vs 60 A label)
/// * JP40  45 A (vs 60 A label)
/// * BAK45D 30 A (vs 60 A label — the datasheet's own "without cutoff" figure)
/// * 40PL  70 A (label; lowest impedance, no published de-rate)
pub fn true_continuous_current(name: &str) -> Option<f64> {
    match name {
        "Molicel P50B" => Some(35.0),
        "Ampace JP40" => Some(45.0),
        "BAK 45D" => Some(30.0),
        "EVE 40PL" => Some(70.0),
        _ => None,
    }
}

/// Manufacturer **maximum continuous charge current**, A (datasheets) — the BMS
/// charge ceiling. Standard/recommended charge is gentler (≈0.5C, longer life);
/// charging to these caps is fast-charge territory that needs cooling.
/// * P50B  25 A (Molicel — also 5C fast-charge capable)
/// * JP40   8 A (Ampace, 2C rated charge)
/// * BAK45D 13.2 A (BAK datasheet; standard 2.2 A)
/// * 40PL  15 A (EVE datasheet; standard 2 A)
pub fn max_charge_current(name: &str) -> Option<f64> {
    match name {
        "Molicel P50B" => Some(25.0),
        "Ampace JP40" => Some(8.0),
        "BAK 45D" => Some(13.2),
        "EVE 40PL" => Some(15.0),
        _ => None,
    }
}

/// The four benchmark cells with their display names, boxed behind the [`Cell`]
/// trait so callers (pack, BMS, benchmark) stay chemistry-agnostic.
pub fn benchmark_cells() -> Vec<(&'static str, Box<dyn Cell>)> {
    vec![
        ("Molicel P50B", Box::new(molicel_p50b())),
        ("Ampace JP40", Box::new(ampace_jp40())),
        ("BAK 45D", Box::new(bak_45d())),
        ("EVE 40PL", Box::new(eve_40pl())),
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DOCUMENTED NUMBERS — each row is a published datasheet value (capacity,
    /// nominal/cutoff voltage, label current, mass) plus the Battery-Mooch
    /// measured DCIR. A reader can hand-check every field against the citation in
    /// the constructor's doc comment.
    #[test]
    fn documented_datasheet_numbers() {
        // (tag, cell, cap_ah, v_nom, v_cut, i_label, mass_kg, r_ohm)
        let rows = [
            ("P50B", molicel_p50b(), 5.0, 3.6, 2.5, 60.0, 0.070, 0.0095),
            ("JP40", ampace_jp40(), 4.0, 3.6, 2.5, 60.0, 0.070, 0.0054),
            ("BAK45D", bak_45d(), 4.5, 3.6, 2.5, 60.0, 0.069, 0.0060),
            ("40PL", eve_40pl(), 4.0, 3.6, 2.5, 70.0, 0.067, 0.0051),
        ];
        for (tag, c, cap, vn, vc, il, m, r) in rows {
            assert!((c.capacity_ah() - cap).abs() < 1e-9, "{tag} cap");
            assert!((c.nominal_voltage() - vn).abs() < 1e-9, "{tag} vnom");
            assert!((c.cutoff_voltage() - vc).abs() < 1e-9, "{tag} vcut");
            assert!((c.max_continuous_current() - il).abs() < 1e-9, "{tag} I");
            assert!((c.mass_kg() - m).abs() < 1e-9, "{tag} mass");
            assert!((c.internal_resistance(0.5) - r).abs() < 1e-9, "{tag} R");
        }
    }

    /// The shared OCV curve hits the common 4.20/2.50 window and is monotonic.
    #[test]
    fn ocv_window_and_monotonic() {
        for (_name, c) in benchmark_cells() {
            assert!((c.ocv(1.0) - 4.20).abs() < 1e-9);
            assert!((c.ocv(0.0) - 2.50).abs() < 1e-9);
            let mut prev = -1.0;
            for k in 0..=100 {
                let v = c.ocv(k as f64 / 100.0);
                assert!(v >= prev - 1e-9);
                prev = v;
            }
        }
    }

    /// The energy-vs-power ordering the benchmark rests on: the P50B carries the
    /// most charge; the EVE 40PL has the lowest resistance (least sag / heat).
    #[test]
    fn energy_vs_power_ordering() {
        let cells = benchmark_cells();
        let cap = |n: &str| cells.iter().find(|c| c.0 == n).unwrap().1.capacity_ah();
        let r = |n: &str| {
            cells
                .iter()
                .find(|c| c.0 == n)
                .unwrap()
                .1
                .internal_resistance(0.5)
        };
        // Energy leader.
        for other in ["Ampace JP40", "BAK 45D", "EVE 40PL"] {
            assert!(cap("Molicel P50B") > cap(other), "P50B energy vs {other}");
        }
        // Lowest-impedance leader (least voltage sag / heat per amp).
        for other in ["Molicel P50B", "Ampace JP40", "BAK 45D"] {
            assert!(r("EVE 40PL") < r(other), "40PL R vs {other}");
        }
    }

    /// The label continuous rating over-states what the cell truly sustains — the
    /// thermal-cutoff caveat, made falsifiable: true ≤ label for every cell, and
    /// strictly below it for the two with published de-rates.
    #[test]
    fn true_continuous_at_most_label() {
        for (name, c) in benchmark_cells() {
            let truec = true_continuous_current(name).unwrap();
            assert!(
                truec <= c.max_continuous_current() + 1e-9,
                "{name} true<=label"
            );
        }
        assert!(true_continuous_current("Ampace JP40").unwrap() < 60.0);
        assert!(true_continuous_current("BAK 45D").unwrap() < 60.0);
    }

    /// The temperature-aware resistance is the base value at 25 °C and rises in
    /// the cold (Arrhenius) — so a winter pack sags and self-heats more.
    #[test]
    fn resistance_rises_in_the_cold() {
        let c = eve_40pl();
        let r25 = c.internal_resistance(0.5);
        assert!((c.internal_resistance_at(0.5, 25.0) - r25).abs() < 1e-12);
        assert!(c.internal_resistance_at(0.5, -20.0) > 5.0 * r25);
        assert!(c.internal_resistance_at(0.5, 45.0) < r25);
    }

    /// Charge-current ratings are sourced and never exceed the (higher) discharge
    /// label — charge is the gentler direction.
    #[test]
    fn charge_ratings_present_and_below_discharge_label() {
        for (name, c) in benchmark_cells() {
            let chg = max_charge_current(name).unwrap();
            assert!(chg > 0.0, "{name} charge rating");
            assert!(
                chg <= c.max_continuous_current(),
                "{name} charge ≤ discharge label"
            );
        }
        assert_eq!(max_charge_current("Molicel P50B"), Some(25.0));
        assert_eq!(max_charge_current("Ampace JP40"), Some(8.0));
    }
}
