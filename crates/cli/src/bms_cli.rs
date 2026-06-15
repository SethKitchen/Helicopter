//! `bms` subcommand: benchmark the four high-power 21700 cells (Molicel P50B,
//! Ampace JP40, BAK 45D, EVE 40PL) and demonstrate the BMS layer (protection
//! envelope, SoC estimation, balancing). Sizing is protection-aware — every pack
//! is built against the cell's TRUE continuous rating, not its datasheet label.
//!
//! The parametric [`Target`] makes the "scales from model to human" goal concrete:
//! the same code sizes a small model pack and a human-scale HV pack.

use helisim_bms::{
    Target, ThermalEnvelope, balancing::CellSpread, protection::ProtectionLimits, run_benchmark,
    sizing::Limiting, soc_estimator::SocEstimator,
};
use helisim_cell::{Cell, benchmark_cells, true_continuous_current};
use helisim_thermal::Convective;

fn limiting_str(l: Limiting) -> &'static str {
    match l {
        Limiting::Power => "power",
        Limiting::Energy => "energy",
    }
}

fn print_cells() {
    println!("=== Benchmark cells (sourced datasheet + measured DCIR) ===");
    println!(
        "{:>14} {:>7} {:>7} {:>9} {:>9} {:>7} {:>7}",
        "cell", "cap Ah", "Vnom", "I_label", "I_true", "DCIR mΩ", "mass g"
    );
    for (name, c) in benchmark_cells() {
        let truec = true_continuous_current(name).unwrap();
        println!(
            "{:>14} {:>6.1} {:>6.1}V {:>7.0}A {:>7.0}A {:>7.1} {:>6.0}",
            name,
            c.capacity_ah(),
            c.nominal_voltage(),
            c.max_continuous_current(),
            truec,
            c.internal_resistance(0.5) * 1000.0,
            c.mass_kg() * 1000.0,
        );
    }
    println!(
        "Note: I_label is the datasheet's 80°C-cutoff-limited rating; I_true is the\n\
         independently-measured continuous rating (Battery Mooch). The BMS sizes on\n\
         I_true — the label is not a safety number."
    );
}

fn print_benchmark(title: &str, target: Target) {
    println!("\n=== Benchmark: {title} ===");
    println!(
        "  target: ≥{:.0} V bus, {:.1} kW peak, {:.2} kWh stored",
        target.bus_voltage_v,
        target.peak_power_w / 1000.0,
        target.energy_wh / 1000.0
    );
    println!(
        "{:>14} {:>7} {:>7} {:>8} {:>9} {:>7} {:>6} {:>8}",
        "cell", "S", "P", "cells", "mass kg", "Wh/kg", "util", "limit"
    );
    let rows = run_benchmark(target);
    for r in &rows {
        let s = &r.sizing;
        println!(
            "{:>14} {:>7} {:>7} {:>8} {:>9.1} {:>7.0} {:>5.0}% {:>8}",
            r.name,
            s.series,
            s.parallel,
            s.cell_count,
            s.mass_kg,
            r.pack_wh_per_kg,
            s.current_utilisation * 100.0,
            limiting_str(s.limiting),
        );
    }
    let best = &rows[0];
    println!(
        "  -> lightest: {} ({:.1} kg, {} bound)",
        best.name,
        best.sizing.mass_kg,
        limiting_str(best.sizing.limiting)
    );
}

fn print_bms_demo() {
    println!("\n=== BMS layer demo (cell-agnostic; scales by cell count) ===");
    let cell = helisim_cell::ampace_jp40();
    let limits =
        ProtectionLimits::from_cell(&cell, true_continuous_current("Ampace JP40").unwrap(), 60.0);
    println!(
        "Protection (JP40): window {:.1}–{:.1} V, {:.0} A cont, {:.0}°C",
        limits.v_min, limits.v_max, limits.i_continuous, limits.t_max
    );
    for &(v, i, t) in &[
        (3.6, 30.0, 25.0),
        (3.6, 50.0, 25.0),
        (2.4, 10.0, 25.0),
        (3.6, 10.0, 70.0),
    ] {
        println!(
            "  check(V={v:.1}, I={i:.0}, T={t:.0}°C) -> {:?}",
            limits.check(v, i, t)
        );
    }

    // SoC estimator: coulomb-count drift, then OCV re-anchor corrects it.
    let mut est = SocEstimator::new(0.40, cell.capacity_ah()); // seeded wrong (drift)
    let v_rest = cell.ocv(0.60); // truth is 60%
    println!(
        "SoC estimator: drifted estimate {:.0}% -> OCV re-anchor at {:.3} V ...",
        est.soc() * 100.0,
        v_rest
    );
    est.reanchor_from_ocv(v_rest, &cell);
    println!("  corrected to {:.1}% (truth 60%)", est.soc() * 100.0);

    // Balancing: an imbalanced string strands capacity until passively balanced.
    let mut s = CellSpread::new(vec![0.95, 0.90, 0.60, 0.80]);
    println!(
        "Balancing: spread {:.0}% strands {:.0}% capacity (weakest cell limits discharge to {:.0}%)",
        s.spread() * 100.0,
        s.stranded_fraction() * 100.0,
        s.dischargeable_fraction() * 100.0
    );
    for _ in 0..1000 {
        s.passive_balance_step(0.001);
    }
    println!(
        "  after passive balancing: spread {:.2}%",
        s.spread() * 100.0
    );
}

fn print_thermal_tabless() {
    println!("\n=== Tabless / thermal: true-continuous EMERGES from the sim ===");
    println!(
        "  2-node (core+surface) thermal + temperature-dependent R. The tabless\n\
         advantage flows through the cell's low measured R (less I²R heat), NOT a\n\
         lower radial conduction (which is chemistry/geometry-set, tab-independent)."
    );
    let env = ThermalEnvelope::for_21700(25.0, 80.0);
    let natural = Convective::natural_air();
    println!(
        "{:>14} {:>10} {:>12} {:>12} {:>9}",
        "cell", "steady A", "core-tran A", "surf-tran A", "rating A"
    );
    for (name, c) in benchmark_cells() {
        let steady = env.steady_continuous(c.as_ref(), &natural);
        let core = env.discharge_continuous_core(c.as_ref(), &natural);
        let surf = env.discharge_continuous(c.as_ref(), &natural);
        let rating = true_continuous_current(name).unwrap();
        println!("{name:>14} {steady:>9.0} {core:>11.0} {surf:>11.0} {rating:>8.0}");
    }
    println!(
        "  Finding: the steady still-air SURFACE limit reproduces JP40 (47 vs 45 A)\n\
         and P50B (36 vs 35 A) to ~4%, emergent. The full-discharge SURFACE limit is\n\
         meaningless at high rate (skin lags core, empties in ~1 min) — the CORE is\n\
         the safety node. Temp-dependent R is load-bearing (R(80°C)≈0.18·R(25°C))."
    );
    // Cold penalty (Arrhenius R).
    let cell = helisim_cell::eve_40pl();
    let r25 = cell.internal_resistance_at(0.5, 25.0) * 1000.0;
    let rcold = cell.internal_resistance_at(0.5, -20.0) * 1000.0;
    println!(
        "  Cold penalty (EVE 40PL): R {r25:.1} mΩ @25°C -> {rcold:.1} mΩ @-20°C ({:.1}× sag/heat).",
        rcold / r25
    );
}

pub fn run() {
    println!("helisim — battery + BMS benchmark (zero dependencies, std only)\n");
    print_cells();
    print_thermal_tabless();

    // Two ends of the scaling axis, same parametric sizing call.
    print_benchmark(
        "model helicopter pack (≈22 V, 1.5 kW peak, 0.15 kWh)",
        Target {
            bus_voltage_v: 22.0,
            peak_power_w: 1_500.0,
            energy_wh: 150.0,
        },
    );
    print_benchmark(
        "human-scale helicopter pack (≈700 V, 200 kW peak, 50 kWh)",
        Target {
            bus_voltage_v: 700.0,
            peak_power_w: 200_000.0,
            energy_wh: 50_000.0,
        },
    );

    print_bms_demo();

    println!(
        "\nInsight: the winner follows the binding requirement — the high-capacity P50B\n\
         leads energy-bound packs; the low-impedance/high-current EVE 40PL leads\n\
         power-bound packs. The same BMS + sizing scales from the model to the\n\
         human aircraft by changing only the target, not the code."
    );
}
