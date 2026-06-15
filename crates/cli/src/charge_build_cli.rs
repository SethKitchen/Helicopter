//! `charge-build` subcommand: the integrated answer to "fold 10-year life into the
//! design and strive for 1:1 charging".
//!
//! For the model heli it sizes the pack for a **10-year, daily-flying** life
//! (oversized for shallow DoD), **propagates the heavier pack back into the hover
//! power** (momentum scaling, iterated to closure), emits the updated pack BOM, and
//! lists the **charging equipment** (120 V / 240 V / solar / DC-fast) with the
//! charge:flight ratio each reaches. The human-scale case shows what 1:1 takes.
//!
//! Key synthesis: the same oversize that buys 10-year life lowers the flight
//! C-rate, so a 1:1 charge becomes a GENTLE low-C charge — life and fast turnaround
//! reinforce each other.

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_bms::{LifeSizing, PackBuild, build_pack, size_for_life};
use helisim_cell::{Cell, DegradationModel, molicel_p50b};
use helisim_charging::{ChargeKit, kit_120v, kit_240v, kit_dc_fast, kit_solar};
use helisim_design::{DesignCandidate, evaluate, size_for_daily_life};

/// Upsizing assumptions (representative, overridable): figure of merit, empty-mass
/// fraction (structure+motor+avionics, of gross), profile-power factor, useful
/// payload to carry, and the disk loadings to consider.
const FM: f64 = 0.72;
const EMPTY_FRACTION: f64 = 0.40;
const PROFILE_FACTOR: f64 = 1.15;
const PAYLOAD_KG: f64 = 1.0;
const DISK_LOADINGS: [f64; 5] = [15.0, 25.0, 40.0, 60.0, 90.0];

/// Representative pack-level specific energy (P50B cell 257 Wh/kg × ~0.70
/// packaging), Wh/kg — a stated, overridable assumption.
const PACK_WH_PER_KG: f64 = 180.0;
const LIFE_YEARS: f64 = 10.0;
const FLIGHTS_PER_YEAR: f64 = 365.0;
const STORAGE_TEMP_C: f64 = 25.0;
const DRIVELINE_ETA: f64 = 0.85;

fn print_pack_bom(pack: &PackBuild) {
    println!(
        "\nPack BOM: {}S{}P = {} cells, {:.0} Wh, {:.2} kg, parts ≈ ${:.0} (+ tools ${:.0})",
        pack.series,
        pack.parallel,
        pack.cell_count,
        pack.energy_wh,
        pack.mass_kg,
        pack.parts_total_usd(),
        pack.tools_total_usd()
    );
    for l in &pack.lines {
        println!(
            "    {:<44} {:>4.0} × ${:>7.2}",
            l.item, l.qty, l.unit_price.usd
        );
    }
}

/// Option B: size an UPSIZED aircraft that closes the 10-yr daily-flight pack.
/// Sweeps disk loading (rotor size), recommends the lightest design that closes
/// with margin, and emits its pack BOM + charging.
fn recommend_upsize(deg: &DegradationModel, flight_time_h: f64) {
    // The life-limited DoD for daily flying (from the cell aging model).
    let dod = size_for_life(
        deg,
        1000.0,
        flight_time_h,
        FLIGHTS_PER_YEAR,
        LIFE_YEARS,
        STORAGE_TEMP_C,
        PACK_WH_PER_KG,
    )
    .dod;
    println!(
        "\nOption B — UPSIZE so the 10-yr daily pack closes (recommended). Design for a battery-heavy\n\
         airframe (empty {:.0}% of gross) at LOW disk loading; the {:.0}-min daily life needs DoD {:.0}%\n\
         (pack ≈{:.1}× the energy-minimum). Useful payload {:.1} kg; FM {:.2}, {:.0} Wh/kg pack.",
        EMPTY_FRACTION * 100.0,
        flight_time_h * 60.0,
        dod * 100.0,
        1.0 / dod,
        PAYLOAD_KG,
        FM,
        PACK_WH_PER_KG
    );
    println!(
        "  {:>8} {:>8} {:>8} {:>9} {:>9} {:>10}",
        "DL N/m²", "rotor", "gross", "hover", "pack", "closes?"
    );
    let mut best: Option<helisim_design::UpsizeResult> = None;
    for &dl in &DISK_LOADINGS {
        match size_for_daily_life(
            PAYLOAD_KG,
            EMPTY_FRACTION,
            dl,
            FM,
            DRIVELINE_ETA,
            PROFILE_FACTOR,
            flight_time_h,
            dod,
            PACK_WH_PER_KG,
        ) {
            Some(r) => {
                println!(
                    "  {:>8.0} {:>6.2} m {:>6.1} kg {:>7.0} W {:>7.0} Wh   ✓ ({:.0}% pack)",
                    dl,
                    r.rotor_radius_m,
                    r.gross_kg,
                    r.hover_power_elec_w,
                    r.pack_energy_wh,
                    r.pack_fraction * 100.0
                );
                // "Works well" = the most battery-heavy design (largest pack
                // fraction → most life margin) that's still a sane model size
                // (≤8 kg gross, ≤1.5 m rotor).
                if r.gross_kg <= 8.0
                    && r.rotor_radius_m <= 1.5
                    && best
                        .map(|b| r.pack_fraction > b.pack_fraction)
                        .unwrap_or(true)
                {
                    best = Some(r);
                }
            }
            None => println!(
                "  {:>8.0} {:>33}   ✗ won't close (pack+structure fill the aircraft)",
                dl, ""
            ),
        }
    }

    let Some(r) = best else {
        println!(
            "  → No disk loading in range closes with a ≤1.5 m rotor — lower the disk loading further."
        );
        return;
    };
    println!(
        "\n  ★ Recommended: {:.1} kg gross, {:.2} m rotor (DL {:.0} N/m²), {:.0} W hover, {:.0} Wh pack\n\
        ({:.1} kg = {:.0}% of gross), {:.0}-min sortie, daily for 10 years.\n\
        The real fix is a REDESIGN, not just mass: ~{:.1}× the base mass and a {:.0}% bigger rotor, but\n\
        the key change is making it BATTERY-HEAVY ({:.0}% pack vs the base's ~19%) at low disk loading.\n\
        Bigger payload → bigger aircraft (gross scales ~linearly with payload).",
        r.gross_kg,
        r.rotor_radius_m,
        r.disk_loading_n_m2,
        r.hover_power_elec_w,
        r.pack_energy_wh,
        r.pack_mass_kg,
        r.pack_fraction * 100.0,
        r.flight_time_h * 60.0,
        r.gross_kg / 3.5,
        (r.rotor_radius_m / 0.6 - 1.0) * 100.0,
        r.pack_fraction * 100.0
    );

    // Pack BOM for the recommended pack.
    let cell = molicel_p50b();
    let cell_wh = cell.nominal_voltage() * cell.capacity_ah();
    let series = 6usize;
    let parallel = ((r.pack_energy_wh / (series as f64 * cell_wh)).ceil() as usize).max(1);
    let peak_a = r.hover_power_elec_w / (series as f64 * cell.nominal_voltage());
    let pack = build_pack("Molicel P50B", &cell, series, parallel, peak_a);
    print_pack_bom(&pack);

    println!(
        "\n  Charging the upsized model (toward 1:1 at {:.0} W flight power):",
        r.hover_power_elec_w
    );
    print_kit(&kit_120v(r.hover_power_elec_w));
    print_kit(&kit_solar(r.hover_power_elec_w));
    println!(
        "  → Flight power is still small, so a 120 V outlet exceeds it (≤1:1) and a few panels match it;\n\
         charge gently overnight for best life. (First-cut momentum + mass-fraction sizing — confirm\n\
         structure mass and FM before building.)"
    );
}

/// Years to 80% for full-DoD daily flying at `c_rate` (calendar tracks the years).
fn years_to_eol(deg: &DegradationModel, c_rate: f64, _flight_time_h: f64) -> f64 {
    let mut y = 0.1;
    while y <= 30.0 {
        let fade = deg.fade_over_life(
            FLIGHTS_PER_YEAR * y,
            1.0,
            c_rate,
            25.0,
            y,
            STORAGE_TEMP_C,
            1.0,
        );
        if fade >= deg.eol_fade {
            return y;
        }
        y += 0.1;
    }
    30.0
}

fn print_kit(k: &ChargeKit) {
    let unity = if k.reaches_unity() {
        "✓ ~1:1"
    } else {
        "below 1:1"
    };
    println!(
        "  {:<28} {:>8.1} kW  ratio {:>6.1}:1 [{}]   equip ≈ ${:.0}",
        k.source,
        k.charge_power_w / 1000.0,
        k.charge_flight_ratio,
        unity,
        k.equipment_total_usd()
    );
    for e in &k.equipment {
        println!(
            "      - {:<38} {:>3.0}× ${:>8.2}  ({})",
            e.item, e.qty, e.unit_price_usd, e.note
        );
    }
}

/// Iterate the pack-mass ↔ hover-power loop for the model heli. Returns
/// (gross_mass, hover_elec_w, flight_time_h, life_sizing, closed).
fn size_model_heli(deg: &DegradationModel) -> (f64, f64, f64, LifeSizing, bool) {
    let base = DesignCandidate::model();
    let report = evaluate(&base, &LinearAirfoil::naca0012(), &Config::default());
    let p0_shaft = report.hover_shaft_power_w;
    let m0 = base.gross_mass_kg;
    let base_pack_mass = base.pack_energy_wh / PACK_WH_PER_KG;
    let non_pack = m0 - base_pack_mass;
    // Base flight duration to preserve (the "fly" time per sortie).
    let flight_time_h =
        (base.pack_energy_wh * base.usable_fraction) / (p0_shaft / DRIVELINE_ETA) / 1.0;

    let mut m = m0;
    let mut last = size_for_life(
        deg,
        1.0,
        flight_time_h,
        FLIGHTS_PER_YEAR,
        LIFE_YEARS,
        STORAGE_TEMP_C,
        PACK_WH_PER_KG,
    );
    let mut p_elec = p0_shaft / DRIVELINE_ETA;
    let mut closed = false;
    for _ in 0..200 {
        // Hover power grows with weight^1.5 (momentum theory).
        let p_shaft = p0_shaft * (m / m0).powf(1.5);
        p_elec = p_shaft / DRIVELINE_ETA;
        let flight_energy = p_elec * flight_time_h;
        last = size_for_life(
            deg,
            flight_energy,
            flight_time_h,
            FLIGHTS_PER_YEAR,
            LIFE_YEARS,
            STORAGE_TEMP_C,
            PACK_WH_PER_KG,
        );
        let m_new = non_pack + last.pack_mass_kg;
        if (m_new - m).abs() < 0.005 {
            closed = true;
            m = m_new;
            break;
        }
        if m_new > 5.0 * m0 {
            m = m_new;
            break; // diverging — won't close
        }
        m += 0.5 * (m_new - m); // damped
    }
    (m, p_elec, flight_time_h, last, closed)
}

pub fn run() {
    println!("helisim — charge-build: 10-year-life pack + 1:1 charging equipment (zero deps)");
    println!(
        "Constraints folded in: ≥{LIFE_YEARS:.0}-year pack at {FLIGHTS_PER_YEAR:.0} flights/yr, and\n\
         charging sized toward 1:1 (charge power = flight power). Prices representative/overridable."
    );
    let deg = DegradationModel::default();

    // ---------------- MODEL HELI ----------------
    println!("\n########################################################");
    println!("# MODEL HELI — fold in 10-yr daily life (mass→power propagated)");
    println!("########################################################");
    let base = DesignCandidate::model();
    let cell = molicel_p50b();
    let cell_wh = cell.nominal_voltage() * cell.capacity_ah();
    let (m, p_elec, flight_time_h, ls, closed) = size_model_heli(&deg);

    if closed {
        println!(
            "Heli recalculated: gross {:.2} kg (was {:.2}); hover {:.0} W elec; flight {:.0} min/sortie.",
            m,
            base.gross_mass_kg,
            p_elec,
            flight_time_h * 60.0
        );
        println!(
            "Life-sized pack: oversize {:.1}× (DoD {:.0}%), flight {:.2}C, capacity {:.0} Wh, mass {:.2} kg,\n\
             10-yr fade {:.0}% → MEETS 10 yr (mass loop converged).",
            ls.oversize,
            ls.dod * 100.0,
            ls.flight_c_rate,
            ls.capacity_wh,
            ls.pack_mass_kg,
            ls.fade_over_life * 100.0
        );
        let series = 6usize;
        let parallel = ((ls.capacity_wh / (series as f64 * cell_wh)).ceil() as usize).max(1);
        let peak_a = p_elec / (series as f64 * cell.nominal_voltage());
        let pack: PackBuild = build_pack("Molicel P50B", &cell, series, parallel, peak_a);
        print_pack_bom(&pack);
        println!(
            "\nCharging equipment (toward 1:1 at {:.0} W flight power):",
            p_elec
        );
        print_kit(&kit_120v(p_elec));
        print_kit(&kit_solar(p_elec));
    } else {
        // The honest finding: the constraint is INFEASIBLE on a tiny airframe.
        println!(
            "⚠ INFEASIBLE: folding a 10-yr DAILY-flight life into a {:.1} kg airframe does NOT close.\n\
             A 10-yr/365-per-yr pack needs ~{:.1}× oversize (~{:.0} Wh, ~{:.1} kg) — heavier than the\n\
             whole base aircraft — so adding it raises hover power, which needs still more pack: the\n\
             mass spirals past ~{:.0} kg without a fixed point. The cells are simply too heavy for that\n\
             duty cycle on a model-scale rotor.",
            base.gross_mass_kg, ls.oversize, ls.capacity_wh, ls.pack_mass_kg, m
        );
        // Practical answer: build the energy-minimum pack and replace it periodically.
        let base_c_rate = 1.0 / flight_time_h; // full-DoD flight C-rate at base size
        let life_min = years_to_eol(&deg, base_c_rate, flight_time_h);
        let replacements = (LIFE_YEARS / life_min).ceil();
        let series = 6usize;
        let parallel = ((base.pack_energy_wh / (series as f64 * cell_wh)).ceil() as usize).max(1);
        let p_elec_base = (evaluate(&base, &LinearAirfoil::naca0012(), &Config::default())
            .hover_shaft_power_w)
            / DRIVELINE_ETA;
        let peak_a = p_elec_base / (series as f64 * cell.nominal_voltage());
        let pack: PackBuild = build_pack("Molicel P50B", &cell, series, parallel, peak_a);
        println!(
            "\nOption A — keep it 3.5 kg and REPLACE the cheap pack: the energy-minimum pack ({}S{}P,\n\
             ${:.0}) lasts ≈{:.1} yr at daily full-DoD flying (≈{:.0} packs over a decade).",
            pack.series,
            pack.parallel,
            pack.parts_total_usd(),
            life_min,
            replacements
        );
        let _ = p_elec_base;

        // Option B — UPSIZE the aircraft so the 10-yr daily pack actually closes.
        recommend_upsize(&deg, flight_time_h);
    }

    // ---------------- HUMAN HELI ----------------
    println!("\n########################################################");
    println!("# HUMAN HELI — what 1:1 charging takes");
    println!("########################################################");
    let human_flight_w = 130_000.0; // representative hover power (stated)
    println!(
        "Flight (hover) power ≈ {:.0} kW. 1:1 needs the charger to deliver that.",
        human_flight_w / 1000.0
    );
    print_kit(&kit_120v(human_flight_w));
    print_kit(&kit_240v(human_flight_w));
    print_kit(&kit_dc_fast(human_flight_w));
    print_kit(&kit_solar(human_flight_w));
    println!(
        "\n  → Only DC FAST CHARGE reaches 1:1 for a human-scale pack (120/240 V are branch-capped).\n\
         Because the 10-yr pack is oversized ~3× (flight ≈0.8C), that 1:1 charge is only ~0.8C — gentle.\n\
         For DAILY ops you can instead trickle the daily energy off 240 V overnight (kinder to life);\n\
         reserve DC fast for rapid back-to-back turnaround."
    );
}
