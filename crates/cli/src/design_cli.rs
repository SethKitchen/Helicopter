//! `design` subcommand: the model-scale sizing study. Reports the starter design
//! point against the priority vector (safety → airtime → efficiency → noise),
//! then sweeps rotor radius at fixed tip speed to surface the central disk-loading
//! trade and recommend a point from the priority *ordering*.

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_cost::{build_bom, summarize, AircraftSpec, UnitCosts};
use helisim_design::{evaluate, recommend, sweep_radius, DesignCandidate, DesignReport, DesignSpace};
use helisim_manufacture::blade_from_design;

pub fn run() {
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();

    println!("helisim — model-scale sizing study (composes the validated cores, zero deps)\n");

    // Lead with the RECOMMENDATION — the tool suggests a target, it doesn't just
    // evaluate one. Search the space, enforce safety, rank by priority order.
    let space = DesignSpace::model_default();
    println!("=== RECOMMENDED model design (searched + ranked by priority order) ===");
    match recommend(&space, &base, &af, &cfg) {
        Some(rec) => {
            for line in &rec.rationale {
                println!("  {line}");
            }
            println!("  Top alternatives:");
            println!(
                "  {:>2}b R={:>4} σ={:>5} V={:>3}  flareM endur  cost   VII  OASPL  score",
                "", "", "", ""
            );
            for sc in rec.ranked.iter().take(4) {
                let c = &sc.candidate;
                let r = &sc.report;
                println!(
                    "  {:>2}b R={:.2} σ={:.3} V={:>3.0}  {:>5.2} {:>5.1}m ${:>4.0} {:>4.0}% {:>4.1}dB {:>6.2}",
                    c.n_blades, c.radius_m, c.solidity(), c.tip_speed_ms,
                    r.flare_margin, r.endurance_min, r.total_cost,
                    r.vertical_integration_index * 100.0, r.oaspl_db, sc.score
                );
            }
            emit_blade_build(&rec.best.candidate);
        }
        None => println!("  No design met the constraints; relax the safety floor or widen the grid."),
    }
    println!();
    println!(
        "Starter point: {:.1} kg gross, {} blades, R={:.2} m, c={:.3} m, V_tip={:.0} m/s, NACA0012",
        base.gross_mass_kg, base.n_blades, base.radius_m, base.chord_m, base.tip_speed_ms
    );
    println!(
        "Pack: {:.0} Wh usable×{:.0}% @ η={:.0}%   |   noise observer {:.0} m @ {:.0}° off axis\n",
        base.pack_energy_wh,
        base.usable_fraction * 100.0,
        base.powertrain_eta * 100.0,
        base.observer_distance_m,
        base.observer_angle_deg,
    );

    let r = evaluate(&base, &af, &cfg);
    print_point(&base, &r);
    print_cost(&base, &r);

    println!(
        "\n=== Radius sweep at fixed V_tip={:.0} m/s ({:.1} kg) — the disk-loading trade ===",
        base.tip_speed_ms, base.gross_mass_kg
    );
    println!(
        "{:>6} {:>6} {:>7} {:>7} {:>7} {:>7} {:>8} {:>7} {:>6}",
        "R(m)", "RPM", "DL", "P_sh", "endur", "autoVd", "OASPL", "flareM", "ok?"
    );
    let radii = [0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00];
    let pts = sweep_radius(&base, &radii, &af, &cfg);
    for p in &pts {
        let rep = p.report;
        if !rep.hover_feasible {
            println!("{:>5.2}   cannot hover at this tip speed", p.radius_m);
            continue;
        }
        let rpm = (base.tip_speed_ms / p.radius_m) * 60.0 / (2.0 * std::f64::consts::PI);
        println!(
            "{:>6.2} {:>6.0} {:>6.1}N {:>6.0}W {:>6.1}m {:>5.0}fpm {:>5.1}dB {:>6.2} {:>5}",
            p.radius_m,
            rpm,
            rep.disk_loading,
            rep.hover_shaft_power_w,
            rep.endurance_min,
            rep.autorotation_descent_fpm,
            rep.oaspl_db,
            rep.flare_margin,
            if rep.can_flare { "yes" } else { "NO" },
        );
    }

    // Capstone: the same validated stack on a human-scale 2-pax point, to show
    // how the priorities — especially the safety findings — scale from the model.
    println!("\n=== Scale-up: human-scale 2-passenger point (same stack, defensible assumptions) ===");
    let human = DesignCandidate::human_scale_2pax();
    let hr = evaluate(&human, &af, &cfg);
    println!(
        "  {:.0} kg, R={:.1} m, V_tip={:.0} m/s, {:.0} kWh pack",
        human.gross_mass_kg, human.radius_m, human.tip_speed_ms, human.pack_energy_wh / 1000.0
    );
    if hr.hover_feasible {
        println!(
            "  hover {:.0} kW, endurance {:.0} min, FM {:.2}, OASPL {:.0} dB @ 150 m",
            hr.hover_shaft_power_w / 1000.0, hr.endurance_min, hr.figure_of_merit, hr.oaspl_db
        );
        println!(
            "  SAFETY scale-up: rotor-decay reaction {:.1} s (vs {:.2} s model), flare margin {:.2} ({}),",
            hr.rotor_decay_time_s, r.rotor_decay_time_s, hr.flare_margin,
            if hr.can_flare { "OK" } else { "FAILS" }
        );
        println!(
            "  forward auto min-sink {:.0} fpm @ {:.0} m/s, best glide {:.1}°.",
            hr.forward_min_sink_fpm, hr.forward_min_sink_speed_ms, hr.best_glide_angle_deg
        );
        print_cost(&human, &hr);
        println!(
            "  → The model's ~0.5 s decay window stretches to ~{:.0} s at human scale (stored rotor\n  \
             energy ∝ size⁵-ish vs power ∝ size³·⁵), and the COTS-avionics cost share shrinks as\n  \
             self-buildable structure grows — both priorities (safety, vertical integration) ease\n  \
             with scale. The aero/efficiency sweet-spot logic is unchanged; only the numbers move.",
            hr.rotor_decay_time_s
        );
    } else {
        println!("  (infeasible at these assumptions — adjust tip speed / radius)");
    }

    println!(
        "\nReading by priority order (what the sweep ACTUALLY shows — not a monotone story):\n  \
         4. Airtime  — hover power is NOT monotone in radius at fixed V_tip: it bottoms\n     \
         out near R≈0.7 (endurance peaks there) and rises again as the blades grow\n     \
         draggy. There is a genuine optimum, not 'bigger is better'.\n  \
         1a. Safety (descent) — the autorotation descent RATE also minimises near R≈0.6,\n     \
         then worsens: profile power (∝R at fixed V_tip) overtakes the shrinking induced\n     \
         power, driving the rotor deep into the windmill-brake state.\n  \
         1b. Safety (flare)   — the flareM column does the OPPOSITE: at fixed V_tip a\n     \
         bigger disk spins SLOWER (Ω=V_tip/R), so for fixed rotor inertia the stored\n     \
         flare energy ½IΩ² FALLS — the margin shrinks toward the 'NO' cliff. This is the\n     \
         binding tension: efficiency/noise want a big disk, flare energy wants RPM.\n  \
         6. Noise    — the only cleanly monotone column: bigger disk (lower RPM at fixed\n     \
         V_tip) is steadily quieter. Tip speed is the independent knob — a 10%% V_tip cut\n     \
         buys ~2.7 dB with no thrust change, traded against higher torque/heavier blades.\n\n\
         Recommendation from the priority ORDER: SAFETY leads, and the two safety metrics\n\
         pull opposite ways on radius — so the disk can't just grow. A point near\n\
         R≈0.6-0.65 m balances a gentle-ish descent against an adequate flare margin,\n\
         with airtime near its optimum; noise is then bought by trimming V_tip, NOT by\n\
         growing R into the flare-margin cliff. The model-scale reality the study makes\n\
         concrete: a small rotor's flare margin is thin (raise rotor inertia / RPM, or\n\
         accept a higher safe-touchdown design) — flare energy is THE rotor-sizing driver."
    );
}

/// Emit the buildable blade geometry + step-by-step shaping for a design — the
/// first concrete "here is the shape to make" output.
fn emit_blade_build(c: &DesignCandidate) {
    let b = blade_from_design(c, 0.0);
    println!("\n  --- BLADE build (recommended design) ---");
    println!(
        "  {} ×{}, span {:.0} mm, chord {:.0} mm, max thickness {:.1} mm @ 30% chord",
        b.airfoil, b.n_blades, b.span_m * 1000.0, b.chord_m * 1000.0, b.max_thickness_m * 1000.0
    );
    for step in b.instructions() {
        println!("    {step}");
    }
}

/// Cost + vertical-integration view (priorities #2 and #3). Builds a BOM from a
/// documented mass split and the computed hover power; costs are a PARAMETRIC
/// model with representative default inputs — override with real quotes.
fn print_cost(c: &DesignCandidate, r: &DesignReport) {
    if !r.hover_feasible {
        return;
    }
    // Documented mass split of the gross mass + installed power = 2× hover (climb
    // margin). Override-able; representative, not authoritative.
    let m = c.gross_mass_kg;
    let spec = AircraftSpec {
        n_blades: c.n_blades,
        blade_mass_kg: 0.03 * m / c.n_blades as f64,
        hub_mass_kg: 0.05 * m,
        structure_mass_kg: 0.40 * m,
        powertrain_mass_kg: 0.12 * m,
        motor_power_kw: 2.0 * r.hover_shaft_power_w / 1000.0,
        pack_energy_wh: c.pack_energy_wh,
        pack_mass_kg: 0.25 * m,
    };
    let costs = UnitCosts::default();
    let cr = summarize(&build_bom(&spec, &costs));
    println!("  [cost/build] total ≈ ${:.0} (PARAMETRIC — representative unit costs, override w/ quotes)", cr.total_cost);
    print!("               by subsystem:");
    for (s, cost) in &cr.by_subsystem {
        print!(" {s} ${cost:.0};");
    }
    println!();
    println!(
        "  [vert-integ] self-build index {:.0}% (1=all self-made); ${:.0} ({:.0}%) is irreducible buy",
        cr.vertical_integration_index * 100.0,
        cr.purchased_cost,
        cr.purchased_cost_fraction * 100.0,
    );
    print!("               must buy:");
    for (n, cost) in &cr.buy_items {
        print!(" {n} ${cost:.0};");
    }
    println!();
}

fn print_point(c: &DesignCandidate, r: &DesignReport) {
    println!("=== Starter point evaluated ===");
    if !r.hover_feasible {
        println!("  INFEASIBLE: cannot hover {:.1} kg at this rotor/tip speed.", c.gross_mass_kg);
        return;
    }
    println!(
        "  [safety]     vertical auto {:.0} fpm (V_d/v_h={:.2}); forward min-sink {:.0} fpm @ {:.1} m/s,",
        r.autorotation_descent_fpm, r.autorotation_ratio, r.forward_min_sink_fpm, r.forward_min_sink_speed_ms
    );
    println!(
        "               best glide {:.1}° @ {:.1} m/s; flare margin {:.2} ({}), flare-height {:.2} m",
        r.best_glide_angle_deg,
        r.best_glide_speed_ms,
        r.flare_margin,
        if r.can_flare { "energy bound MET" } else { "FAILS bound" },
        r.flare_height_m,
    );
    println!(
        "               rotor-decay reaction time after power loss: {:.2} s (E_flare/P_hover, worst case)",
        r.rotor_decay_time_s,
    );
    println!(
        "  [airtime]    endurance {:.1} min   (P {:.0} W mech → {:.0} W elec)",
        r.endurance_min, r.hover_shaft_power_w, r.hover_elec_power_w
    );
    println!(
        "  [efficiency] FM {:.3}, power loading {:.3} N/W, disk loading {:.1} N/m², collective {:.2}°",
        r.figure_of_merit, r.power_loading, r.disk_loading, r.collective_deg
    );
    println!(
        "  [noise]      OASPL {:.1} dB @ obs, blade-passage {:.0} Hz, tip Mach {:.2}",
        r.oaspl_db, r.blade_passage_hz, r.tip_mach
    );
}
