//! `battery-build` subcommand: emit an exact, purchasable build for a battery pack
//! + BMS — the bill of materials (quantities, sourced prices, buy links), one-time
//! tools, and the step-by-step assembly procedure.
//!
//! The pack is sized to the **real electrical load it must carry**: the motor (via
//! its ESC) PLUS the control-surface actuators (swashplate + tail servos, via an HV
//! BEC) PLUS the avionics rail — computed by [`helisim_actuation::power_budget`]
//! from the selected actuation hardware. The build then lists the power-distribution
//! parts (ESC, HV BEC, connectors, fuse) and the instructions that feed pack power
//! to the motor and the actuators.

use helisim_actuation::loads::MOTOR_POWER_MARGIN;
use helisim_actuation::{power_budget, select_actuation, ActuationPlan, PowerBudget};
use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_bms::components::{Buildability, ESC, HV_BEC};
use helisim_bms::{build_pack, size_for_target, PackBuild, Target};
use helisim_cell::{
    ampace_jp40, bak_45d, benchmark_cells, eve_40pl, molicel_p50b, true_continuous_current, Cell,
};
use helisim_design::{evaluate, DesignCandidate};
use helisim_actuation::motor::pack_connector;

/// Target hover endurance the pack energy is sized for, minutes.
const TARGET_ENDURANCE_MIN: f64 = 12.0;

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

fn truncate(s: &str, n: usize) -> String {
    if s.chars().count() <= n {
        s.to_string()
    } else {
        let t: String = s.chars().take(n - 1).collect();
        format!("{t}…")
    }
}

fn print_build(title: &str, b: &PackBuild) {
    println!("\n========================================================");
    println!("{title}");
    println!("========================================================");
    println!(
        "Pack: {} — {}S{}P = {} cells | {:.1} V nom | {:.1} Ah | {:.0} Wh | {:.2} kg | peak {:.0} A",
        b.cell_name, b.series, b.parallel, b.cell_count, b.nominal_v, b.capacity_ah, b.energy_wh, b.mass_kg, b.peak_current_a,
    );
    if b.is_distributed_bms() {
        println!("BMS: DISTRIBUTED (master + slave modules) — {}S exceeds a single integrated board.", b.series);
    } else {
        println!("BMS: single integrated smart BMS (Li-ion variant).");
    }
    println!("\n--- SHOPPING LIST (per pack) ---");
    println!("{:<46}{:>6} {:>6} {:>9} {:>9}  {}", "item", "qty", "type", "unit $", "line $", "source");
    for l in &b.lines {
        print_line(&l.item, l.qty, build_kind(l.buildability), l.unit_price.usd, l.line_total_usd(), l.unit_price.retailer, l.unit_price.url);
    }
    println!("{:<46}{:>30.2}", "PARTS TOTAL (USD)", b.parts_total_usd());
    println!("\n--- ONE-TIME TOOLS ---");
    for l in &b.tools {
        print_line(&l.item, l.qty, build_kind(l.buildability), l.unit_price.usd, l.line_total_usd(), l.unit_price.retailer, l.unit_price.url);
    }
    println!("{:<46}{:>30.2}", "TOOLS TOTAL (USD)", b.tools_total_usd());
    println!("\n--- BUILD INSTRUCTIONS ---");
    for step in &b.instructions {
        println!("  {step}");
    }
}

#[allow(clippy::too_many_arguments)]
fn print_line(item: &str, qty: f64, kind: &str, unit: f64, line: f64, retailer: &str, url: &str) {
    println!(
        "{:<46}{:>6.0} {:>6} {:>9.2} {:>9.2}  {} {}",
        truncate(item, 46),
        qty,
        kind,
        unit,
        line,
        retailer,
        if url.is_empty() { "" } else { url }
    );
}

/// Print the power-budget calculation and the power-distribution parts + wiring
/// that feed pack power to the motor and the control-surface actuators.
fn print_power_feed(act: &ActuationPlan, budget: &PowerBudget) {
    println!("\n--- POWER BUDGET (what the pack must feed: motor + control-surface actuators) ---");
    for line in budget.explain() {
        println!("  {line}");
    }

    let esc_a = (budget.motor_current_a * 1.2).ceil();
    let bec_a = budget.servo_peak_current_a.ceil();
    let conn = pack_connector(budget.motor_current_a);
    println!("\n--- POWER DISTRIBUTION (pack → motor + actuators) ---");
    println!("{:<46}{:>6} {:>6} {:>9} {:>9}  {}", "item", "qty", "type", "unit $", "line $", "source");
    print_line(&format!("Brushless heli ESC, ≥{esc_a:.0} A, {}S", act.cells), 1.0, "buy", ESC.usd, ESC.usd, ESC.retailer, ESC.url);
    print_line(&format!("HV BEC for servos, ≥{bec_a:.0} A @ {:.1} V", budget.servo_voltage_v), 1.0, "buy", HV_BEC.usd, HV_BEC.usd, HV_BEC.retailer, HV_BEC.url);
    print_line(&format!("Pack→ESC main connector ({conn})"), 1.0, "buy", 0.0, 0.0, "(included with main lead)", "");
    println!(
        "  (the motor, servos and their prices/links are in the `helisim build` actuation section)"
    );

    println!("\n--- POWER-FEED INSTRUCTIONS (motor + actuators) ---");
    for step in act.power_and_connections() {
        if step.starts_with('—') {
            println!("  {step}");
        } else {
            println!("    {step}");
        }
    }
    println!(
        "  • Fuse the pack POSITIVE output (≥ pack peak {:.0} A, < wire/connector limit) before it splits to the ESC and the HV BEC.",
        budget.pack_peak_current_a
    );
    println!(
        "  • Wire the HV BEC INPUT across the pack (post-fuse), set its output to the servo rail ({:.1} V); never run digital HV servos off a 5 V receiver rail.",
        budget.servo_voltage_v
    );
}

/// Size a pack to a fixed series count (= the motor's cell count) and a current +
/// energy demand, returning (series, parallel).
fn size_to_motor_rail(cell: &dyn Cell, cell_name: &str, series: usize, peak_a: f64, energy_wh: f64) -> (usize, usize) {
    let tc = true_continuous_current(cell_name).unwrap();
    let p_power = (peak_a / tc).ceil();
    let cell_wh = cell.nominal_voltage() * cell.capacity_ah();
    let p_energy = (energy_wh / (series as f64 * cell_wh)).ceil();
    (series, p_power.max(p_energy).max(1.0) as usize)
}

pub fn run() {
    println!("helisim — battery pack + BMS build (zero deps; prices sourced 2026-06-15, representative & overridable)");
    println!(
        "PRICE CAVEAT: every $ is a representative figure from a named retailer on the capture date,\n\
         NOT a quote — battery/hobby prices fluctuate; confirm on the live listing."
    );

    // ----- MODEL PACK: sized to the actuation power budget (motor + actuators) -----
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    let report = evaluate(&base, &af, &cfg);
    let act = select_actuation(&base, &report);
    let budget = power_budget(&act);

    println!("\n########################################################");
    println!("# MODEL HELICOPTER — powering the motor + control surfaces");
    println!("########################################################");
    print_power_feed(&act, &budget);

    // Energy for the target hover endurance: hover (not peak) electrical power.
    let hover_pack_w =
        budget.motor_power_w / MOTOR_POWER_MARGIN + budget.bec_input_power_w + budget.avionics_power_w;
    let energy_wh = hover_pack_w * TARGET_ENDURANCE_MIN / 60.0;

    // Pack series = the motor's cell count (the ESC is configured for it); the
    // benchmark-winning cell sizes the rest for current + energy.
    let target = Target {
        bus_voltage_v: budget.pack_voltage_v,
        peak_power_w: budget.total_pack_power_w,
        energy_wh,
    };
    let cell_name = lightest_cell_for(target);
    let cell = cell_by_name(cell_name);
    let series = act.cells.max(1) as usize;
    let (series, parallel) =
        size_to_motor_rail(cell.as_ref(), cell_name, series, budget.pack_peak_current_a, energy_wh);
    let model = build_pack(cell_name, cell.as_ref(), series, parallel, budget.pack_peak_current_a);
    print_build(
        &format!(
            "MODEL PACK (sized to the {:.0} W motor+actuator budget, {:.0}-min hover)",
            budget.total_pack_power_w, TARGET_ENDURANCE_MIN
        ),
        &model,
    );

    // ----- HUMAN-SCALE PACK: parametric (distributed BMS) -----
    let human_target = Target {
        bus_voltage_v: 700.0,
        peak_power_w: 200_000.0,
        energy_wh: 50_000.0,
    };
    let human_cell_name = lightest_cell_for(human_target);
    let human_cell = cell_by_name(human_cell_name);
    let hs = size_for_target(human_cell.as_ref(), true_continuous_current(human_cell_name).unwrap(), human_target);
    let human_peak_a = human_target.peak_power_w / hs.bus_nominal_v;
    let human = build_pack(human_cell_name, human_cell.as_ref(), hs.series, hs.parallel, human_peak_a);
    print_build("HUMAN-SCALE PACK (parametric; distributed BMS; actuation beyond catalogue)", &human);

    println!(
        "\nNote: the MODEL pack is sized to the motor+actuator power budget above (series = motor cell\n\
         count; parallel for current + {:.0}-min hover). NMC cells are 3.6 V nominal vs LiPo 3.7 V — the\n\
         series count matches the ESC's cell setting. The human pack is a custom traction-pack build.",
        TARGET_ENDURANCE_MIN
    );
}

/// The lightest cell for a target (reuses the benchmark sizing).
fn lightest_cell_for(target: Target) -> &'static str {
    let mut best: Option<(&'static str, f64)> = None;
    for (name, cell) in benchmark_cells() {
        let tc = true_continuous_current(name).unwrap();
        let s = size_for_target(cell.as_ref(), tc, target);
        if best.map(|(_, m)| s.mass_kg < m).unwrap_or(true) {
            best = Some((name, s.mass_kg));
        }
    }
    best.unwrap().0
}
