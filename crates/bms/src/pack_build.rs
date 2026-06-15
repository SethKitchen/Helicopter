//! Turn a sized topology into a **buildable pack + BMS**: an exact bill of
//! materials (with quantities, sourced prices and buy links), the one-time tool
//! list, and a safety-forward, step-by-step assembly procedure.
//!
//! Scales from model to human by the same call: the BMS line-up switches from a
//! single integrated smart BMS (small series counts) to a distributed master +
//! slave modules + contactor + current sensor (large series counts), because no
//! off-the-shelf single board spans a ~195S traction pack.
//!
//! This is the `manufacture`/`cost` pattern applied to the battery: real geometry
//! of *what to buy and do*, with the dollar figures carrying the explicit
//! representative-price caveat from [`crate::components`].

use crate::components::{
    cell_price, BomLine, Buildability, UnitPrice, BALANCE_CHARGER, BALANCE_LEAD_KIT, BMS_MASTER,
    BMS_SLAVE_MODULE, CELL_HOLDER_EACH, CONTACTOR, COPPER_BUSBAR_SET, CURRENT_SENSOR,
    FISH_PAPER_ROLL, FUSE_AND_HOLDER, KAPTON_ROLL, MAIN_LEAD_HEAVY, MAIN_LEAD_XT90, MULTIMETER,
    NICKEL_ROLL_5M, SMART_BMS, SPOT_WELDER,
};
use helisim_cell::Cell;

/// A complete, buildable pack: bill of materials, one-time tools, and assembly steps.
#[derive(Clone, Debug)]
pub struct PackBuild {
    pub cell_name: String,
    pub series: usize,
    pub parallel: usize,
    pub cell_count: usize,
    pub nominal_v: f64,
    pub capacity_ah: f64,
    pub energy_wh: f64,
    pub mass_kg: f64,
    pub peak_current_a: f64,
    /// Consumed-per-pack parts (the buy-list).
    pub lines: Vec<BomLine>,
    /// One-time tools (not consumed per pack).
    pub tools: Vec<BomLine>,
    /// Step-by-step assembly procedure.
    pub instructions: Vec<String>,
}

impl PackBuild {
    /// Sum of the per-pack parts (the cost to build one pack), USD.
    pub fn parts_total_usd(&self) -> f64 {
        self.lines.iter().map(BomLine::line_total_usd).sum()
    }
    /// Sum of the one-time tools, USD.
    pub fn tools_total_usd(&self) -> f64 {
        self.tools.iter().map(BomLine::line_total_usd).sum()
    }
    /// True when the BMS is the distributed (master + slaves) kind.
    pub fn is_distributed_bms(&self) -> bool {
        self.series > MAX_INTEGRATED_SERIES
    }
}

/// Above this many series cells, a single integrated smart BMS is not available,
/// so the build switches to a distributed master/slave architecture.
pub const MAX_INTEGRATED_SERIES: usize = 24;
/// Series cells one distributed slave module monitors.
const SERIES_PER_SLAVE: usize = 16;
/// Pack peak current (A) above which 0.15 mm nickel strip is replaced by copper
/// busbar for the series links (which carry full pack current).
const NICKEL_MAX_PACK_A: f64 = 120.0;
/// Pack peak current (A) above which the XT90/12 AWG main lead is replaced by a
/// heavy cable + high-current connector.
const XT90_MAX_A: f64 = 90.0;

fn line(item: impl Into<String>, qty: f64, unit: &'static str, p: UnitPrice, b: Buildability, note: impl Into<String>) -> BomLine {
    BomLine {
        item: item.into(),
        qty,
        unit,
        unit_price: p,
        buildability: b,
        note: note.into(),
    }
}

/// Select the BMS line(s) for `series` cells carrying `peak_current_a`.
fn bms_lines(series: usize, peak_current_a: f64) -> Vec<BomLine> {
    if series <= MAX_INTEGRATED_SERIES {
        vec![line(
            format!("Smart BMS, {series}S Li-ion, ≥{:.0} A", peak_current_a.ceil()),
            1.0,
            "ea",
            SMART_BMS,
            Buildability::Purchased,
            "Li-ion variant (NOT LiFePO4); current rating ≥ pack peak; Bluetooth/UART for monitoring",
        )]
    } else {
        let slaves = series.div_ceil(SERIES_PER_SLAVE);
        vec![
            line(
                format!("BMS slave module ({SERIES_PER_SLAVE}S monitoring)"),
                slaves as f64,
                "ea",
                BMS_SLAVE_MODULE,
                Buildability::Purchased,
                format!("{slaves} modules cover {series}S; one per ~{SERIES_PER_SLAVE} series groups"),
            ),
            line("BMS master controller", 1.0, "ea", BMS_MASTER, Buildability::Purchased, "aggregates slaves; runs protection/contactor logic"),
            line(format!("Main HV contactor, ≥{:.0} A", peak_current_a.ceil()), 1.0, "ea", CONTACTOR, Buildability::Purchased, "main disconnect commanded by the BMS"),
            line("Hall current sensor", 1.0, "ea", CURRENT_SENSOR, Buildability::Purchased, "pack current feedback for the master"),
        ]
    }
}

/// Build a complete pack + BMS for `cell` in a `series`×`parallel` topology drawing
/// up to `peak_current_a`.
pub fn build_pack(
    cell_name: &str,
    cell: &dyn Cell,
    series: usize,
    parallel: usize,
    peak_current_a: f64,
) -> PackBuild {
    let cell_count = series * parallel;
    // Nickel: each series group needs busing across its P cells plus a series link;
    // ~0.06 m of strip per cell is a sound first-cut (both ends, parallel + series).
    let nickel_m = cell_count as f64 * 0.06;
    let nickel_rolls = (nickel_m / 5.0).ceil().max(1.0);
    let balance_kits = (series as f64 / 12.0).ceil().max(1.0); // a kit ~ up to 12 taps

    let mut lines = vec![line(
        format!("{cell_name} 21700 cell"),
        cell_count as f64,
        "ea",
        cell_price(cell_name),
        Buildability::Purchased,
        format!("{series}S × {parallel}P; match cell voltages within 0.02 V before welding"),
    )];
    lines.extend(bms_lines(series, peak_current_a));
    // Interconnect scales with the series-link current (= full pack peak): thin
    // nickel for low current, copper busbar for a high-current traction pack.
    if peak_current_a <= NICKEL_MAX_PACK_A {
        lines.push(line(
            "Pure nickel strip 0.15 mm × 8 mm (5 m roll)",
            nickel_rolls,
            "roll",
            NICKEL_ROLL_5M,
            Buildability::RawStock,
            format!("≈{nickel_m:.1} m needed; stack/parallel strips for current (1P strip ≈ 25–30 A)"),
        ));
    } else {
        lines.push(line(
            "Copper busbar / thick interconnect set",
            1.0,
            "set",
            COPPER_BUSBAR_SET,
            Buildability::RawStock,
            format!("series links carry the full {:.0} A pack current — 0.15 mm nickel cannot; use busbar/laser-weld", peak_current_a),
        ));
    }
    lines.push(line("Balance-lead wiring kit", balance_kits, "kit", BALANCE_LEAD_KIT, Buildability::RawStock, format!("{} balance taps (B0..B{series})", series + 1)));
    // Main lead scales with current: XT90/12 AWG up to ~90 A, heavy gauge above.
    if peak_current_a <= XT90_MAX_A {
        lines.push(line("Main lead: 12 AWG silicone wire + XT90", 1.0, "set", MAIN_LEAD_XT90, Buildability::RawStock, "XT90 ≈ 90 A continuous; size gauge to peak current"));
    } else {
        lines.push(line(format!("Heavy main cable + HV connector (≥{:.0} A)", peak_current_a.ceil()), 1.0, "set", MAIN_LEAD_HEAVY, Buildability::RawStock, "e.g. 2 AWG + Anderson/HV connector — XT90/12 AWG cannot carry this"));
    }
    lines.push(line(format!("Inline fuse + holder, ≥{:.0} A", peak_current_a.ceil()), 1.0, "ea", FUSE_AND_HOLDER, Buildability::Purchased, "on the positive output, between pack and load"));
    lines.push(line("Fish/barley paper insulation roll", 1.0, "roll", FISH_PAPER_ROLL, Buildability::RawStock, "positive-end insulator rings + layer between groups"));
    lines.push(line("Kapton tape roll", 1.0, "roll", KAPTON_ROLL, Buildability::RawStock, "secure leads, insulate nickel edges"));
    lines.push(line("21700 cell holder / spacer", cell_count as f64, "ea", CELL_HOLDER_EACH, Buildability::Purchased, "fixes cell pitch; keeps cans from shorting"));

    let tools = vec![
        line("Battery spot welder", 1.0, "ea", SPOT_WELDER, Buildability::Tool, "weld nickel — do NOT solder directly to cans (heat damages cells)"),
        line("Smart balance charger", 1.0, "ea", BALANCE_CHARGER, Buildability::Tool, "first charge + balancing at low current"),
        line("Multimeter", 1.0, "ea", MULTIMETER, Buildability::Tool, "cell matching and per-tap voltage checks"),
    ];

    let instructions = assembly_steps(cell_name, series, parallel, peak_current_a);

    PackBuild {
        cell_name: cell_name.to_string(),
        series,
        parallel,
        cell_count,
        nominal_v: series as f64 * cell.nominal_voltage(),
        capacity_ah: parallel as f64 * cell.capacity_ah(),
        energy_wh: series as f64 * cell.nominal_voltage() * parallel as f64 * cell.capacity_ah(),
        mass_kg: cell_count as f64 * cell.mass_kg(),
        peak_current_a,
        lines,
        tools,
        instructions,
    }
}

fn assembly_steps(cell_name: &str, series: usize, parallel: usize, peak_a: f64) -> Vec<String> {
    let distributed = series > MAX_INTEGRATED_SERIES;
    let heavy = peak_a > NICKEL_MAX_PACK_A;
    let mut s = vec![
        "SAFETY FIRST: Li-ion cells store large energy and can vent/ignite if shorted, \
         over-charged or punctured. Work one connection at a time, insulate tools, never \
         bridge a parallel group while welding the next. Have a metal/sand bucket nearby."
            .to_string(),
        format!(
            "1. Sort & match: measure every {cell_name} cell with the multimeter. Group into \
             {series} parallel-groups of {parallel}; cells in a group must match within 0.02 V \
             (paralleling unmatched cells dumps current between them)."
        ),
        "2. Insulate positive ends: apply a fish-paper ring to each cell's positive end \
         (the rim shorts easily to nickel)."
            .to_string(),
        format!(
            "3. Lay out the {series}S×{parallel}P array in the cell holders, alternating \
             orientation so each series step is a short hop (series link from one group's + \
             bus to the next group's − bus)."
        ),
        if heavy {
            format!(
                "4. Join the parallel buses first across the {parallel} cells of each group with \
                 COPPER BUSBAR (laser/resistance weld). Each bus carries the full ≈{:.0} A pack \
                 current — 0.15 mm nickel cannot; size the busbar to current and temperature rise.",
                peak_a
            )
        } else {
            format!(
                "4. Spot-weld the parallel buses first: weld nickel across the {parallel} cells of \
                 each group (same polarity). Use stacked/parallel nickel strips so the bus carries \
                 the current (1P 0.15 mm nickel ≈ 25–30 A; peak here ≈ {:.0} A pack).",
                peak_a
            )
        },
        "5. Spot-weld the series links between adjacent groups (group + bus → next group − bus). \
         After this the pack reads full nominal voltage end-to-end."
            .to_string(),
        format!(
            "6. Attach balance leads at EVERY series node in order: B0 = pack −, then B1..B{series} \
             at each successive group junction, B{series} = pack +. Getting the order/positions \
             right is critical — a mis-wired balance tap can destroy the BMS."
        ),
    ];
    if distributed {
        s.push(format!(
            "7. Wire the DISTRIBUTED BMS: connect each slave module to its ~{}-series block's \
             balance taps; daisy-chain slaves to the master; install the Hall current sensor on \
             the main negative; wire the master to command the main contactor.",
            SERIES_PER_SLAVE
        ));
        s.push("8. Install the main contactor and the inline fuse on the positive output, then \
                the heavy main cable + HV connector. Fuse rating ≥ peak current, < cable/busbar limit."
            .to_string());
    } else {
        s.push("7. Connect the smart BMS: B− to pack negative FIRST, then the balance connector \
                (B1..Bn in order), then P+/C+ to pack positive last. Use the Li-ion variant."
            .to_string());
        s.push(format!(
            "8. Install the inline fuse (≥{:.0} A) on the positive output and solder the XT90 to \
             the 12 AWG main leads (off-pack, so cell heat is never an issue).",
            peak_a.ceil()
        ));
    }
    s.push("9. Insulate & secure: cover exposed nickel/series links with kapton, layer fish paper \
            between groups, fix all leads, then close the enclosure."
        .to_string());
    s.push("10. First-power checks: confirm pack voltage = nominal; verify each balance tap rises \
            monotonically (B0<B1<...<Bn); balance-charge at LOW current; confirm the BMS trips on \
            over-current and over-temperature before field use."
        .to_string());
    s.push("11. Aircraft integration: wire automatic power-loss detection + instant collective \
            drop — the model rotor's stored energy gives only ~0.5 s of usable RPM after power \
            loss (see the autorotation track), far faster than a human can react."
        .to_string());
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_cell::{eve_40pl, molicel_p50b};

    fn model_pack() -> PackBuild {
        build_pack("EVE 40PL", &eve_40pl(), 7, 2, 60.0)
    }

    #[test]
    fn bom_quantities_scale_with_topology() {
        let b = model_pack();
        let cells = b.lines.iter().find(|l| l.item.contains("cell")).unwrap();
        assert_eq!(cells.qty as usize, 14); // 7S2P
        assert_eq!(b.cell_count, 14);
        // Electrical summary is consistent with the cells.
        assert!((b.nominal_v - 7.0 * 3.6).abs() < 1e-9);
        assert!((b.capacity_ah - 2.0 * 4.0).abs() < 1e-9);
    }

    #[test]
    fn parts_total_sums_lines_and_is_positive() {
        let b = model_pack();
        let manual: f64 = b.lines.iter().map(|l| l.qty * l.unit_price.usd).sum();
        assert!((b.parts_total_usd() - manual).abs() < 1e-9);
        assert!(b.parts_total_usd() > 0.0);
        // Cells dominate a small pack's bill.
        let cell_line = b.lines.iter().find(|l| l.item.contains("cell")).unwrap();
        assert!(cell_line.line_total_usd() > 0.0);
    }

    #[test]
    fn small_pack_uses_integrated_bms_large_uses_distributed() {
        let small = build_pack("EVE 40PL", &eve_40pl(), 7, 2, 60.0);
        assert!(!small.is_distributed_bms());
        assert_eq!(small.lines.iter().filter(|l| l.item.contains("BMS")).count(), 1);

        let big = build_pack("Molicel P50B", &molicel_p50b(), 195, 15, 290.0);
        assert!(big.is_distributed_bms());
        // master + slaves + contactor + current sensor = several BMS-related lines.
        let bms_related = big
            .lines
            .iter()
            .filter(|l| l.item.contains("BMS") || l.item.contains("contactor") || l.item.contains("sensor"))
            .count();
        assert!(bms_related >= 4, "distributed BMS should have ≥4 lines, got {bms_related}");
        // 195S needs ceil(195/16)=13 slave modules.
        let slaves = big.lines.iter().find(|l| l.item.contains("slave")).unwrap();
        assert_eq!(slaves.qty as usize, 13);
    }

    #[test]
    fn interconnect_and_main_lead_scale_with_current() {
        // Model (60 A): thin nickel + XT90.
        let small = build_pack("EVE 40PL", &eve_40pl(), 7, 2, 60.0);
        assert!(small.lines.iter().any(|l| l.item.contains("nickel strip")));
        assert!(small.lines.iter().any(|l| l.item.contains("XT90")));
        // Human (285 A): copper busbar + heavy cable, NOT nickel/XT90.
        let big = build_pack("Molicel P50B", &molicel_p50b(), 195, 15, 285.0);
        assert!(big.lines.iter().any(|l| l.item.contains("busbar")));
        assert!(big.lines.iter().any(|l| l.item.contains("Heavy main cable")));
        assert!(!big.lines.iter().any(|l| l.item.contains("XT90")));
        assert!(!big.lines.iter().any(|l| l.item.contains("nickel strip")));
    }

    #[test]
    fn instructions_are_present_and_safety_led() {
        let b = model_pack();
        assert!(b.instructions.len() >= 10);
        assert!(b.instructions[0].contains("SAFETY"));
        // The balance-tap-order warning must be present (a destroy-the-BMS hazard).
        assert!(b.instructions.iter().any(|s| s.contains("balance") && s.contains("order")));
        // The model-heli power-loss safety tie-in is included.
        assert!(b.instructions.iter().any(|s| s.contains("power-loss")));
    }
}
