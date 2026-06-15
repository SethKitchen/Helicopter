//! `battery-build` subcommand: emit an exact, purchasable build for a battery pack
//! + BMS — the bill of materials (quantities, sourced prices, buy links), one-time
//! tools, and the step-by-step assembly procedure. Sized parametrically, so the
//! same path prints a buildable model pack and a (distributed-BMS) human-scale pack.

use helisim_bms::components::Buildability;
use helisim_bms::{build_pack, size_for_target, PackBuild, Target};
use helisim_cell::{ampace_jp40, bak_45d, benchmark_cells, eve_40pl, molicel_p50b, Cell};

fn cell_by_name(name: &str) -> Box<dyn Cell> {
    match name {
        "Molicel P50B" => Box::new(molicel_p50b()),
        "Ampace JP40" => Box::new(ampace_jp40()),
        "BAK 45D" => Box::new(bak_45d()),
        _ => Box::new(eve_40pl()),
    }
}

fn build_kind(b: Buildability) -> &'static str {
    match b {
        Buildability::Purchased => "buy",
        Buildability::RawStock => "stock",
        Buildability::Tool => "tool",
    }
}

fn print_build(title: &str, b: &PackBuild) {
    println!("\n========================================================");
    println!("{title}");
    println!("========================================================");
    println!(
        "Pack: {} — {}S{}P = {} cells | {:.1} V nom | {:.1} Ah | {:.0} Wh | {:.2} kg | peak {:.0} A",
        b.cell_name,
        b.series,
        b.parallel,
        b.cell_count,
        b.nominal_v,
        b.capacity_ah,
        b.energy_wh,
        b.mass_kg,
        b.peak_current_a,
    );
    if b.is_distributed_bms() {
        println!("BMS: DISTRIBUTED (master + slave modules) — {}S exceeds a single integrated board.", b.series);
    } else {
        println!("BMS: single integrated smart BMS (Li-ion variant).");
    }

    println!("\n--- SHOPPING LIST (per pack) ---");
    println!("{:<46}{:>6} {:>6} {:>9} {:>9}  {}", "item", "qty", "type", "unit $", "line $", "source");
    for l in &b.lines {
        println!(
            "{:<46}{:>6.0} {:>6} {:>9.2} {:>9.2}  {} {}",
            truncate(&l.item, 46),
            l.qty,
            build_kind(l.buildability),
            l.unit_price.usd,
            l.line_total_usd(),
            l.unit_price.retailer,
            if l.unit_price.url.is_empty() { "" } else { l.unit_price.url },
        );
    }
    println!("{:<46}{:>30.2}", "PARTS TOTAL (USD)", b.parts_total_usd());

    println!("\n--- ONE-TIME TOOLS ---");
    for l in &b.tools {
        println!(
            "{:<46}{:>6.0} {:>6} {:>9.2} {:>9.2}  {} {}",
            truncate(&l.item, 46),
            l.qty,
            build_kind(l.buildability),
            l.unit_price.usd,
            l.line_total_usd(),
            l.unit_price.retailer,
            if l.unit_price.url.is_empty() { "" } else { l.unit_price.url },
        );
    }
    println!("{:<46}{:>30.2}", "TOOLS TOTAL (USD)", b.tools_total_usd());

    println!("\n--- BUILD INSTRUCTIONS ---");
    for step in &b.instructions {
        println!("  {step}");
    }
}

fn truncate(s: &str, n: usize) -> String {
    if s.chars().count() <= n {
        s.to_string()
    } else {
        let t: String = s.chars().take(n - 1).collect();
        format!("{t}…")
    }
}

pub fn run() {
    println!("helisim — battery pack + BMS build (zero deps; prices sourced 2026-06-15, representative & overridable)");
    println!(
        "PRICE CAVEAT: every $ is a representative figure from a named retailer on the capture date,\n\
         NOT a quote — battery/hobby prices fluctuate; confirm on the live listing. The BOM structure\n\
         (what & how many) is the durable part."
    );

    // Model-scale pack: size to a small target, build with the benchmark-winning cell.
    let model_target = Target {
        bus_voltage_v: 22.0,
        peak_power_w: 1_500.0,
        energy_wh: 150.0,
    };
    let model_cell_name = lightest_cell_for(model_target);
    let model_cell = cell_by_name(model_cell_name);
    let ms = size_for_target(model_cell.as_ref(), helisim_cell::true_continuous_current(model_cell_name).unwrap(), model_target);
    let model_peak_a = model_target.peak_power_w / ms.bus_nominal_v;
    let model = build_pack(model_cell_name, model_cell.as_ref(), ms.series, ms.parallel, model_peak_a);
    print_build("MODEL HELICOPTER PACK (buyable today)", &model);

    // Human-scale pack: same call, distributed BMS emerges.
    let human_target = Target {
        bus_voltage_v: 700.0,
        peak_power_w: 200_000.0,
        energy_wh: 50_000.0,
    };
    let human_cell_name = lightest_cell_for(human_target);
    let human_cell = cell_by_name(human_cell_name);
    let hs = size_for_target(human_cell.as_ref(), helisim_cell::true_continuous_current(human_cell_name).unwrap(), human_target);
    let human_peak_a = human_target.peak_power_w / hs.bus_nominal_v;
    let human = build_pack(human_cell_name, human_cell.as_ref(), hs.series, hs.parallel, human_peak_a);
    print_build("HUMAN-SCALE HELICOPTER PACK (parametric; distributed BMS)", &human);

    println!(
        "\nNote: pass a different cell or target in code (build_pack / size_for_target) to regenerate\n\
         either list. Cells dominate the model bill; for the human pack the cells + distributed BMS\n\
         channels dominate. The human pack is a custom traction-pack build, not a single-board kit."
    );
}

/// The lightest cell for a target (reuses the benchmark sizing).
fn lightest_cell_for(target: Target) -> &'static str {
    let mut best: Option<(&'static str, f64)> = None;
    for (name, cell) in benchmark_cells() {
        let tc = helisim_cell::true_continuous_current(name).unwrap();
        let s = size_for_target(cell.as_ref(), tc, target);
        if best.map(|(_, m)| s.mass_kg < m).unwrap_or(true) {
            best = Some((name, s.mass_kg));
        }
    }
    best.unwrap().0
}
