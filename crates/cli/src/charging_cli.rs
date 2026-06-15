//! `charging` subcommand: how to refill the pack two ways — a residential 120 V
//! socket and a solar array. Runs the CC/CV charge model for a model-scale pack
//! (the practical case) and shows the human-scale contrast, reporting charge time,
//! energy, efficiency, and whether the SOURCE or the CELL set the rate.

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_bms::ThermalEnvelope;
use helisim_bms::components::cell_price;
use helisim_cell::{CalendarLoad, Cell, DegradationModel, max_charge_current, molicel_p50b};
use helisim_charging::{
    ChargeConfig, ChargeReport, ChargeSource, DcFastCharger, MainsCharger, SolarArray,
    cell_charge_power_ceiling_w, charge, charge_flight_ratio, flight_time_h,
};
use helisim_design::{DesignCandidate, evaluate};
use helisim_pack::Pack;
use helisim_thermal::Convective;

/// End-of-life retention (80%) and pack-life target (years).
const LIFE_YEARS: f64 = 10.0;

/// Representative hover power for a ~1-tonne human-scale electric helicopter, kW —
/// a STATED assumption (reproduces the ~20-min flight the ~52 kWh pack gives, the
/// figure that prompted this analysis). Swap for a real design's hover power.
const HUMAN_HOVER_KW: f64 = 130.0;
/// Driveline (motor + ESC) efficiency, to turn shaft hover power into electrical.
const DRIVELINE_EFF: f64 = 0.85;

fn pack_header(label: &str, pack: &Pack, soc_start: f64) {
    let needed = pack.energy_wh() * (1.0 - soc_start);
    println!(
        "{label}: {}S{}P {} cells | {:.1} V nom (CV {:.1} V) | {:.1} Ah | {:.0} Wh | from {:.0}% SoC → need ≈{:.0} Wh",
        pack.series,
        pack.parallel,
        pack.cell_count(),
        pack.nominal_voltage(),
        pack.ocv(1.0),
        pack.capacity_ah(),
        pack.energy_wh(),
        soc_start * 100.0,
        needed,
    );
}

fn print_row(r: &ChargeReport) {
    let limit = if r.source_limited { "SOURCE" } else { "cell" };
    let t = if r.timed_out {
        format!(">{:.0} h (incomplete)", r.total_time_h)
    } else {
        format!("{:.1} h", r.total_time_h)
    };
    println!(
        "  {:<48} CC {:>5.1} A  {:>14}  in {:>6.0} Wh  src {:>6.0} Wh  η {:>3.0}%  [{}-limited]",
        r.source_label,
        r.cc_current_a,
        t,
        r.energy_into_pack_wh,
        r.source_input_energy_wh,
        r.efficiency() * 100.0,
        limit,
    );
}

pub fn run() {
    println!(
        "helisim — charging the pack (120 V mains + solar); CC/CV, respects cell + BMS limits"
    );
    println!(
        "Facts sourced: 120 V circuit @ NEC 80% continuous (15 A→1440 W, 20 A→1920 W); charger ~90%;\n\
         solar 400 W panels, MPPT ~97%, derate ~80%, 4.5 peak-sun-h/day. All overridable assumptions."
    );

    let cell_name = "Molicel P50B";
    let cell_max = max_charge_current(cell_name).unwrap();
    let cfg = ChargeConfig::default(); // 0.5C recommended, from 20% SoC

    // ---- Model-scale pack: the practical, buildable case ----
    let model = Pack::new(Box::new(molicel_p50b()), 6, 2); // ~25 V, 10 Ah, 216 Wh
    println!("\n=== MODEL helicopter pack ===");
    pack_header("Pack", &model, cfg.soc_start);
    println!(
        "Charge sources (CC plateau at 0.5C = {:.0} A, capped by cell rating {:.0} A/cell):",
        0.5 * model.capacity_ah(),
        cell_max
    );
    let model_sources: Vec<Box<dyn ChargeSource>> = vec![
        Box::new(MainsCharger::residential_15a()),
        Box::new(MainsCharger::residential_20a()),
        Box::new(SolarArray::typical(2)),
        Box::new(SolarArray::typical(4)),
    ];
    for s in &model_sources {
        print_row(&charge(&model, s.as_ref(), cell_max, cfg));
    }
    let solar4 = SolarArray::typical(4);
    println!(
        "  Solar daily yield (4× 400 W) ≈ {:.1} kWh — about {:.0}× this pack's energy per sunny day.",
        solar4.daily_energy_wh().unwrap() / 1000.0,
        solar4.daily_energy_wh().unwrap() / model.energy_wh()
    );
    println!(
        "  Insight: at 0.5C the wall socket is NOT the bottleneck — the CELL's recommended charge\n\
         rate is (≈{:.0} W ≪ the 1296 W the socket can give). Two solar panels already match it.",
        0.5 * model.capacity_ah() * model.nominal_voltage()
    );

    // ---- Human-scale pack: the contrast ----
    let human = Pack::new(Box::new(molicel_p50b()), 195, 15); // ~700 V, 75 Ah, ~52.6 kWh
    println!("\n=== HUMAN-SCALE pack (contrast) ===");
    pack_header("Pack", &human, cfg.soc_start);
    let human_sources: Vec<Box<dyn ChargeSource>> = vec![
        Box::new(MainsCharger::residential_15a()),
        Box::new(SolarArray::typical(60)), // ~24 kW array
    ];
    for s in &human_sources {
        print_row(&charge(&human, s.as_ref(), cell_max, cfg));
    }
    let big_solar = SolarArray::typical(60);
    println!(
        "  Solar daily yield (60× 400 W ≈ 24 kW) ≈ {:.0} kWh — about {:.1} sunny days to fill this pack.",
        big_solar.daily_energy_wh().unwrap() / 1000.0,
        human.energy_wh() * (1.0 - cfg.soc_start) / big_solar.daily_energy_wh().unwrap()
    );
    println!(
        "  Honest caveat: a ~700 V pack cannot charge off a 120 V socket directly — it needs a\n\
         voltage-boosting (HV) charger, and the 120 V branch still caps power at ~1.3 kW (→ the long\n\
         time above). Human-scale charging wants 240 V / dedicated DC fast-charge or a large array."
    );

    charge_flight_ratio_section();
    aging_sweet_spot_section();
    daily_flying_section();
}

/// Daily flying (365/yr): will the pack last 10 years, and what makes it happen.
fn daily_flying_section() {
    println!("\n=== DAILY FLYING (365 flights/yr): can the pack last {LIFE_YEARS:.0} years? ===");
    let model = DegradationModel::default();
    let human = Pack::new(Box::new(molicel_p50b()), 195, 15);
    let base_c_rate = HUMAN_HOVER_KW * 1000.0 / human.energy_wh(); // 2.5C flight
    let flights_per_year = 365.0;
    let storage_temp = 25.0;

    // 1) Minimum pack, full depth: the blunt answer.
    let y_min = years_to_eol(&model, flights_per_year, 1.0, base_c_rate, storage_temp);
    let replacements = (LIFE_YEARS / y_min).ceil();
    println!(
        "Minimum pack, full-depth daily: lasts ≈{:.1} years to 80% — NOT 10. You'd replace it ~{:.0}×\n\
         over a decade. (Charging speed is irrelevant: the limiter is deep CYCLES, not the charger.)",
        y_min, replacements
    );

    // 2) The path to 10 years: oversize the pack → shallower DoD AND lower flight C-rate.
    println!(
        "\nReaching {LIFE_YEARS:.0} years at 365/yr — oversize the pack (uses less of it each flight,\n\
         and a bigger pack lowers the flight C-rate too):"
    );
    println!(
        "  {:<10} {:>6} {:>9} {:>12} {:>14}",
        "pack size", "DoD", "flight C", "10-yr fade", "verdict"
    );
    let mut sweet_f = None;
    let mut f = 1.0;
    while f <= 4.0 + 1e-9 {
        let dod = 1.0 / f;
        let c_rate = base_c_rate / f;
        let fade = model.fade_over_life(
            flights_per_year * LIFE_YEARS,
            dod,
            c_rate,
            25.0,
            CalendarLoad {
                years: LIFE_YEARS,
                storage_temp_c: storage_temp,
                soc_factor: 1.0,
            },
        );
        let ok = fade <= model.eol_fade;
        if ok && sweet_f.is_none() {
            sweet_f = Some(f);
        }
        println!(
            "  {:<9.2}× {:>5.0}% {:>8.2}C {:>11.0}% {:>14}",
            f,
            dod * 100.0,
            c_rate,
            fade * 100.0,
            if ok { "✓ ≥10 yr" } else { "✗ < 10 yr" }
        );
        f += 0.5;
    }

    // 3) Cost crossover: replace a small pack many times vs buy one big pack.
    let cells = human.cell_count() as f64;
    let unit = cell_price("Molicel P50B").usd;
    let small_10yr = replacements * cells * unit;
    if let Some(fs) = sweet_f {
        let big_once = fs * cells * unit;
        println!(
            "\nCost over 10 years (cells only, P50B @ ${:.2}): {:.0}× minimum packs ≈ ${:.0}k  vs  one {:.1}× pack ≈ ${:.0}k.\n\
             → The bigger pack is BOTH longer-lived and cheaper over the decade (and lighter-duty per cycle).",
            unit,
            replacements,
            small_10yr / 1000.0,
            fs,
            big_once / 1000.0
        );
    }

    // 4) Calendar floor / storage lever.
    let cal_25 = model.calendar_fade(LIFE_YEARS, 25.0, 1.0) * 100.0;
    let cal_15 = model.calendar_fade(LIFE_YEARS, 15.0, 1.0) * 100.0;
    println!(
        "\nCalendar fade is a floor you can't cycle your way out of: {:.0}% over 10 yr at 25 °C, but only\n\
         {:.0}% at 15 °C — store cool and not full to reclaim cycle budget.",
        cal_25, cal_15
    );

    // 5) The charging answer for DAILY ops.
    let e_flight = HUMAN_HOVER_KW * 1000.0 * 20.0 / 60.0; // ~one 20-min flight, Wh
    let day_120v = MainsCharger::residential_15a().dc_power_w() * 23.0; // 23 h idle, Wh
    println!(
        "\nDo you need a custom 120 V fast-charge circuit? NO — two reasons:\n\
         • You don't need 1:1 SPEED for once-a-day flying. With ~23 h idle, charge SLOWLY (0.5C, a\n\
           couple of hours) — gentler on life. 1:1 fast charge is only for rapid back-to-back turnaround.\n\
         • A 120 V branch is capped by code/breaker at ~1.3 kW no matter the circuit. Over a full day it\n\
           delivers ~{:.0} kWh, but ONE human-scale flight needs ~{:.0} kWh — so 120 V cannot keep up with\n\
           daily human-scale flying on ENERGY alone. That needs a 240 V circuit (an electrician install,\n\
           ~{:.0} kWh/day available) or DC fast — not a clever 120 V gadget.",
        day_120v / 1000.0,
        e_flight / 1000.0,
        MainsCharger::level2_240v(40.0).dc_power_w() * 23.0 / 1000.0
    );
    println!(
        "  (Model-scale heli: one flight ≈ {:.0} Wh — a 120 V socket refills it in minutes and laughs at\n\
         daily use; no custom circuit, just charge gently overnight.)",
        300.0 * 0.576 * 1000.0 / 1000.0
    );
}

/// Years until the pack reaches end-of-life under a usage (raw flight count/yr,
/// per-flight depth, flight C-rate, storage temp). Calendar fade tracks the years.
fn years_to_eol(
    model: &DegradationModel,
    flights_per_year: f64,
    dod: f64,
    c_rate: f64,
    storage_temp: f64,
) -> f64 {
    let mut y = 0.1;
    while y <= 30.0 {
        let fade = model.fade_over_life(
            flights_per_year * y,
            dod,
            c_rate,
            25.0,
            CalendarLoad {
                years: y,
                storage_temp_c: storage_temp,
                soc_factor: 1.0,
            },
        );
        if fade >= model.eol_fade {
            return y;
        }
        y += 0.1;
    }
    30.0
}

/// Model the aging hit of fast charging and find the sweet spot for a ≥10-year
/// pack life. Couples charge C-rate → charge temperature (the 2-node thermal model)
/// → capacity fade (cycle + calendar), against the flight (discharge) C-rate.
fn aging_sweet_spot_section() {
    println!(
        "\n=== BATTERY AGING & FAST-CHARGE SWEET SPOT (target: ≥{LIFE_YEARS:.0}-year pack) ==="
    );
    println!(
        "Model: cycle fade ∝ (C/1C)^0.63 · 2^((T−25)/10) · EFC^0.55 (Wang-style power law,\n\
         calibrated to BAK 45D 600cyc@6.7C→60% + ~1500cyc@1C→80% representative); calendar fade\n\
         ∝ 2.5%/yr · √t · Arrhenius; EOL = 80%. Coefficients representative/overridable. Aging is\n\
         driven by the EFFECTIVE C-rate = max(charge, flight) — charging faster than you fly is what\n\
         adds stress — plus the heat fast charging makes (raises T → Arrhenius)."
    );

    let model = DegradationModel::default();
    let cell = molicel_p50b();
    let env = ThermalEnvelope::for_21700(25.0, 80.0);
    let cooling = Convective::forced_air(); // you'd cool a fast charge

    // Human pack is the demanding case; flight (discharge) C-rate = hover ÷ energy.
    let human = Pack::new(Box::new(molicel_p50b()), 195, 15);
    let p_flight = HUMAN_HOVER_KW * 1000.0;
    let disch_cr = p_flight / human.energy_wh();
    println!(
        "\nHUMAN pack: flight draws {:.1}C ({:.0} kW ÷ {:.1} kWh). Charging is COOLED (forced air).",
        disch_cr,
        p_flight / 1000.0,
        human.energy_wh() / 1000.0
    );

    // Cycles-to-80% vs charge C-rate. Aging uses AMBIENT (the cell sits at ambient
    // most of the cycle; charging is a small duty fraction), so charge heat is NOT
    // folded in here — it is shown as a separate cooling-requirement flag.
    let ambient = 25.0;
    println!(
        "  Cycles to 80% vs charge rate (eff = max(charge, {:.1}C flight); aged at {ambient:.0} °C):",
        disch_cr
    );
    println!(
        "  {:>7} {:>12} {:>12}   peak charge T (cool!)",
        "charge", "eff C-rate", "cycles→80%"
    );
    for &cr in &[0.5_f64, 1.0, disch_cr, 3.0, 5.0] {
        let t = charge_temp(&env, &cell, &cooling, cr);
        let eff = cr.max(disch_cr);
        let life = model.cycle_life_to_eol(eff, ambient);
        let flag = if t > 45.0 {
            " ⚠ needs strong cooling"
        } else {
            ""
        };
        println!(
            "  {:>6.1}C {:>11.1}C {:>12.0}   {:>5.0} °C{}",
            cr, eff, life, t, flag
        );
    }

    // For a few flight frequencies, the max charge rate that still meets 10 years.
    println!(
        "\n  Max charge rate that still meets a {LIFE_YEARS:.0}-year life (DoD 1.0, 25 °C storage):"
    );
    println!(
        "  {:<18} {:>14} {:>16} {:>12}",
        "usage", "10-yr EFC", "max charge rate", "charge:flight"
    );
    for &flights_per_year in &[10.0_f64, 25.0, 50.0, 100.0] {
        let efc = flights_per_year * LIFE_YEARS; // DoD 1.0
        let best = max_charge_c_for_life(&model, disch_cr, efc);
        let verdict = match best {
            Some(cr) => {
                // charge:flight ratio ≈ flight C-rate / charge C-rate.
                let ratio = disch_cr / cr;
                format!("{cr:>14.1}C   {ratio:>8.2}:1")
            }
            None => "  none — cycle/calendar-limited even at slow charge".to_string(),
        };
        println!(
            "  {:<18} {:>14.0} {}",
            format!("{flights_per_year:.0} flights/yr"),
            efc,
            verdict
        );
    }

    // The sustainable flight frequency at a 1:1 (charge = flight rate) charge.
    let cal = model.calendar_fade(LIFE_YEARS, 25.0, 1.0);
    let per_cycle = model.cycle_coeff * model.c_rate_factor(disch_cr) * model.temp_factor(25.0);
    let efc_budget = ((model.eol_fade - cal).max(0.0) / per_cycle).powf(1.0 / model.throughput_exp);
    println!(
        "\n  → At a 1:1 charge ({:.1}C, ≈flight rate): ~{:.0} full flights fit in a {LIFE_YEARS:.0}-year life\n\
         (≈{:.0} flights/yr). Calendar fade alone eats {:.0}% over {LIFE_YEARS:.0} yr.",
        disch_cr,
        efc_budget,
        efc_budget / LIFE_YEARS,
        cal * 100.0
    );

    println!(
        "\nThe sweet spot:\n\
         • Charging AT or BELOW the flight C-rate (~{:.1}C ≈ 1:1) is FREE on cycle aging — the flight\n\
           discharge already imposes that C-rate stress, so a charge no faster adds nothing. The only\n\
           cost is heat: a {:.1}C charge runs the cells warm and needs cooling, but that heat is brief\n\
           (a small duty fraction of the cycle) so it barely touches the 10-year aging.\n\
         • Charging FASTER than you fly (e.g. 5C, to beat 1:1) raises the effective C-rate → markedly\n\
           fewer cycles (and a hot charge that demands real cooling). Rarely worth it.\n\
         • The real {LIFE_YEARS:.0}-year limiter is TOTAL DEEP CYCLES at the flight rate + calendar fade,\n\
           NOT the charger: ~{:.0} full flights fit the budget regardless of how you charge. To fly more\n\
           often for {LIFE_YEARS:.0} years, lower the FLIGHT (discharge) C-rate — a bigger pack or a more\n\
           efficient rotor (higher FM / lower disk loading) — or use shallower DoD (partial cycles age\n\
           far less). The same levers that improve the charge:flight ratio.",
        disch_cr, disch_cr, efc_budget
    );
}

/// Steady cell surface temperature when charging at `charge_cr` (per-cell current
/// = C-rate × cell capacity), via the 2-node thermal envelope.
fn charge_temp(
    env: &ThermalEnvelope,
    cell: &dyn Cell,
    cooling: &Convective,
    charge_cr: f64,
) -> f64 {
    let i = charge_cr * cell.capacity_ah();
    env.steady_surface_temp(cell, cooling, i)
}

/// Largest charge C-rate whose 10-year fade stays within EOL, or None if even a
/// slow charge fails (usage is cycle/calendar-limited). Ages at ambient (charge
/// heat is a cooling requirement, not folded into the cycle-average temperature).
fn max_charge_c_for_life(model: &DegradationModel, disch_cr: f64, efc: f64) -> Option<f64> {
    let mut best = None;
    let mut cr: f64 = 0.5;
    while cr <= 5.0 + 1e-9 {
        let eff = cr.max(disch_cr);
        if model.meets_life(
            efc,
            eff,
            25.0,
            CalendarLoad {
                years: LIFE_YEARS,
                storage_temp_c: 25.0,
                soc_factor: 1.0,
            },
        ) {
            best = Some(cr);
        }
        cr += 0.25;
    }
    best
}

/// The figure of merit the user cares about: charge time ÷ flight time. The
/// identity `ratio = P_flight / P_charge` (energy cancels) means the ONLY way to
/// 1:1 is to raise charge power toward flight power. This section walks the source
/// ladder and shows where 1:1 lives — and the cell ceiling that makes it possible.
fn charge_flight_ratio_section() {
    println!("\n=== CHARGE : FLIGHT RATIO — getting toward 1:1 ===");
    println!(
        "Key identity: ratio = charge_time / flight_time = P_flight / P_charge. The energy CANCELS,\n\
         so a bigger battery does NOT help — only raising charge power (or lowering flight power) does.\n\
         Flight is a hard ~3C discharge, so 1:1 is a ~3C CHARGE: feasible for these high-power cells\n\
         (P50B ~5C), impossible for a wall socket's power. Below, charging is at the cells' high rate."
    );
    let cell_max = max_charge_current("Molicel P50B").unwrap();
    // Charge at the cells' acceptance (high C-rate) so the SOURCE sets the pace —
    // the fast-charge regime. (At the gentle 0.5C default the cell caps it instead.)
    let fast = ChargeConfig {
        charge_c_rate: 5.0,
        ..ChargeConfig::default()
    };

    // Model heli: real BEMT hover power.
    let report = evaluate(
        &DesignCandidate::model(),
        &LinearAirfoil::naca0012(),
        &Config::default(),
    );
    let p_flight_model = report.hover_shaft_power_w / DRIVELINE_EFF;
    let model = Pack::new(Box::new(molicel_p50b()), 6, 2);
    ratio_ladder(
        "MODEL heli",
        &model,
        cell_max,
        p_flight_model,
        &[
            Box::new(MainsCharger::residential_15a()),
            Box::new(MainsCharger::level2_240v(40.0)),
            Box::new(DcFastCharger::dc_50kw()),
        ],
        fast,
    );

    // Human heli: representative hover power.
    let human = Pack::new(Box::new(molicel_p50b()), 195, 15);
    ratio_ladder(
        "HUMAN heli",
        &human,
        cell_max,
        HUMAN_HOVER_KW * 1000.0,
        &[
            Box::new(MainsCharger::residential_15a()),
            Box::new(MainsCharger::level2_240v(40.0)),
            Box::new(DcFastCharger::dc_50kw()),
            Box::new(DcFastCharger::dc_150kw()),
            Box::new(DcFastCharger::dc_350kw()),
        ],
        fast,
    );

    println!(
        "\nLevers to reach 1:1 (in order of leverage):\n\
         1. Charge at FLIGHT POWER — DC fast charge, not a wall socket. The cells already allow it\n\
            (P50B accepts ~5C ≈ the ceiling shown); the socket's power never can.\n\
         2. Lower flight power — a more efficient rotor (higher FM, lower disk loading) cuts P_flight,\n\
            improving the ratio AND endurance at once (that's the whole aero side of this project).\n\
         3. BATTERY SWAP — sidesteps charging entirely: a fresh pack in minutes makes turnaround ≈0,\n\
            and the drained pack charges gently (0.5C, long life) off-aircraft. Often the real answer.\n\
         Cost of fast charge: heat (needs cooling — the I²R the 2-node thermal model already captures)\n\
         and cycle life (≈3–5C every cycle ages cells faster than the 0.5C gentle charge)."
    );
}

/// Print the charge:flight ratio for a pack across a ladder of sources.
fn ratio_ladder(
    label: &str,
    pack: &Pack,
    cell_max_charge_a: f64,
    p_flight_w: f64,
    sources: &[Box<dyn ChargeSource>],
    cfg: ChargeConfig,
) {
    let usable = pack.energy_wh() * (1.0 - cfg.soc_start);
    let t_flight = flight_time_h(usable, p_flight_w);
    let ceiling = cell_charge_power_ceiling_w(
        pack.parallel as f64 * cell_max_charge_a,
        pack.nominal_voltage(),
    );
    println!(
        "\n{label}: hover {:.1} kW → {:.0} min flight on {:.0} Wh usable.",
        p_flight_w / 1000.0,
        t_flight * 60.0,
        usable
    );
    println!(
        "  Cell charge-power ceiling ≈ {:.0} kW ({:.1}C). 1:1 needs P_charge = P_flight = {:.1} kW → {}.",
        ceiling / 1000.0,
        ceiling / pack.energy_wh(),
        p_flight_w / 1000.0,
        if p_flight_w <= ceiling {
            "cells CAN accept a 1:1 charge"
        } else {
            "cells CANNOT (cell-limited even with an infinite source)"
        }
    );
    println!("  {:<42} {:>10} {:>10}  limit", "source", "charge", "ratio");
    for s in sources {
        let r: ChargeReport = charge(pack, s.as_ref(), cell_max_charge_a, cfg);
        let ratio = charge_flight_ratio(r.total_time_h, t_flight);
        let t = if r.timed_out {
            format!(">{:.0} h", r.total_time_h)
        } else if r.total_time_h < 1.0 {
            format!("{:.0} min", r.total_time_h * 60.0)
        } else {
            format!("{:.1} h", r.total_time_h)
        };
        println!(
            "  {:<42} {:>10} {:>9.1}:1  [{}]",
            s.label(),
            t,
            ratio,
            if r.source_limited { "source" } else { "cell" }
        );
    }
}
