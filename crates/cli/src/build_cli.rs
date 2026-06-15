//! `build` subcommand: recommend a design, then emit the COMPLETE build package —
//! every part (sized from the design), the assembly sequence, and exported
//! STL/DXF geometry files. The end-to-end realisation of the project's goal:
//! physics → recommended design → "make exactly this".

use helisim_actuation::select_actuation;
use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_design::{DesignCandidate, DesignSpace, recommend};
use helisim_manufacture::{
    aircraft_to_step_ap203, aircraft_to_stl, airfoil_to_dxf, blade_from_design, blade_to_step_brep,
    build_package, check_structure, hardware_schedule, lofted_blade_to_stl, run_fea,
};
use std::fs;

pub fn run() {
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    let space = DesignSpace::model_default();

    println!("helisim — full build package (recommend → size every part → export files)\n");

    let Some(rec) = recommend(&space, &base, &af, &cfg) else {
        println!("No design met the constraints; relax the safety floor or widen the grid.");
        return;
    };
    let c = rec.best.candidate;
    let report = rec.best.report;

    println!("=== Recommended design ===");
    for line in &rec.rationale {
        println!("  {line}");
    }

    let pkg = build_package(&c, &report);

    println!("\n=== Parts ({}) ===", pkg.parts.len());
    for part in &pkg.parts {
        let dims: Vec<String> = part
            .key_dimensions_mm()
            .iter()
            .map(|(k, v)| format!("{k} {v:.1}mm"))
            .collect();
        println!(
            "\n• {} [{}] — {}",
            part.name(),
            part.source().label(),
            part.material()
        );
        println!("  dims: {}", dims.join(", "));
        for step in part.build_steps() {
            println!("    {step}");
        }
    }

    println!("\n=== Assembly sequence ===");
    for step in &pkg.assembly_steps {
        println!("  {step}");
    }

    // Structural proof: real flight-load margins (centrifugal-dominated).
    println!("\n=== Structural proof (margins of safety) ===");
    let structure = check_structure(&c, &report, 40.0e6, 200.0e6);
    println!(
        "  blade centrifugal force {:.0} N → min retention bolt Ø {:.1} mm",
        structure.blade_centrifugal_n,
        structure.min_bolt_diameter_m * 1000.0
    );
    for it in &structure.items {
        println!(
            "  {:<10} {:<26} σ {:>6.1} / {:>6.1} MPa  MS {:+.2}  {}",
            it.part,
            it.load,
            it.actual_mpa,
            it.allowable_mpa,
            it.margin_of_safety,
            if it.ok { "OK" } else { "FAIL" }
        );
    }
    println!(
        "  → {} (min MS {:+.2}). Closed-form section check; FEA below refines it.",
        if structure.all_pass {
            "all pass"
        } else {
            "REVIEW: a part fails"
        },
        structure.min_margin
    );

    // FEA: beam field solution (deflection + stress), cross-checked vs closed form.
    println!("\n=== FEA (beam finite element — deflection + stress, vs closed form) ===");
    let fea = run_fea(&c, &report);
    for part in [&fea.boom, &fea.blade] {
        let stiff = match part.tip_deflection_stiffened_m {
            Some(s) => format!(" → {:.2} mm spun-up (centrifugal stiffening)", s * 1000.0),
            None => String::new(),
        };
        println!(
            "  {:<12} tip deflection {:>6.2} mm{}  |  FE σ {:>5.1} vs closed-form {:>5.1} MPa [{}]",
            part.name,
            part.tip_deflection_m * 1000.0,
            stiff,
            part.fe_stress_mpa,
            part.closed_form_stress_mpa,
            if part.routes_agree {
                "agree"
            } else {
                "MISMATCH"
            }
        );
    }

    // Hardware: standard fasteners + bearings selected by load.
    println!("\n=== Hardware schedule (standard parts selected by load) ===");
    for h in hardware_schedule(&c, &report) {
        println!("  {:<24} {:<6} — {}", h.joint, h.part, h.detail);
    }

    // Actuation: real motor + swashplate/tail servos selected by design load.
    println!("\n=== Actuation (motor + servos selected by load) ===");
    let act = select_actuation(&c, &report);
    match &act.motor.part {
        Some(m) => println!(
            "  motor      {:<14} {:>4} Kv, {:>4.0} g — cont {:>4.0} W ≥ demand {:>4.0} W; \
             {:.0} A on {}S (≤ {:.0} A), gear {:.0}:1",
            m.name,
            m.kv as i64,
            m.mass_g,
            m.max_cont_power_w,
            act.motor_power_demand_w,
            act.motor_current_a,
            act.cells,
            m.max_cont_current_a,
            act.gear_ratio,
        ),
        None => println!(
            "  motor      beyond catalogue — demand {:.0} W; est. {:.0} g",
            act.motor_power_demand_w, act.motor.mass_g
        ),
    }
    if let Some(s) = &act.cyclic_servo.part {
        println!(
            "  cyclic     {}× {:<8} {:>5.2} N·m ≥ demand {:.2} N·m (propeller moment), {:.0} g ea, {:.3} s/60°",
            act.n_cyclic_servos,
            s.name,
            s.stall_torque_nm,
            act.servo_demand_nm,
            s.mass_g,
            s.speed_s_per_60
        );
    }
    if let Some(s) = &act.tail_servo.part {
        println!(
            "  tail       1× {:<8} {:>5.2} N·m ≥ demand {:.2} N·m, {:.0} g, {:.3} s/60°",
            s.name, s.stall_torque_nm, act.tail_demand_nm, s.mass_g, s.speed_s_per_60
        );
    }
    println!(
        "  → total actuation mass {:.2} kg (motor {:.2} + servos {:.2})",
        act.total_mass_kg,
        act.motor_mass_kg(),
        act.servo_mass_kg()
    );
    for n in &act.notes {
        println!("  {n}");
    }

    // Purchasable parts: specific part, price, and direct purchase link.
    println!("\n  Purchase list (specific buyable parts — price + direct link):");
    for line in act.purchase_lines() {
        println!("    {line}");
    }

    // Power & connections: pack/ESC/BEC, connectors, signal wiring.
    println!("\n  Power & connections:");
    for step in act.power_and_connections() {
        if step.starts_with('—') {
            println!("    {step}");
        } else {
            println!("      {step}");
        }
    }

    // Connect to structure: which heli structure each part mounts to and how.
    println!("\n  Connect to structure:");
    for step in act.structural_connections() {
        if step.starts_with('—') {
            println!("    {step}");
        } else {
            println!("      {step}");
        }
    }

    // Setup & use: install / centre / endpoint / run-in sequence.
    println!("\n  Setup & use:");
    for step in act.build_instructions() {
        if step.starts_with('—') {
            println!("    {step}");
        } else {
            println!("      {step}");
        }
    }

    // Export geometry files.
    let blade = blade_from_design(&c, 0.0);
    let blade_stl = lofted_blade_to_stl(&blade, 24, 80); // lofted (handles taper/twist)
    let dxf = airfoil_to_dxf(&blade.section_contour_mm(120));
    let aircraft_stl = aircraft_to_stl(&c, &report);
    let blade_step = blade_to_step_brep(&blade, 24, 60); // real B-rep solid
    let aircraft_step = aircraft_to_step_ap203(&c, &report); // whole-assembly AP203 B-rep
    let dir = "build_output";
    println!("\n=== Geometry files ===");
    match fs::create_dir_all(dir) {
        Ok(()) => {
            let files = [
                ("blade.stl", &blade_stl, "lofted blade solid (printable)"),
                (
                    "blade_section.dxf",
                    &dxf,
                    "NACA 0012 section profile (cuttable)",
                ),
                ("aircraft.stl", &aircraft_stl, "full-aircraft assembly mesh"),
                (
                    "blade.step",
                    &blade_step,
                    "B-rep SOLID (MANIFOLD_SOLID_BREP)",
                ),
                (
                    "aircraft.step",
                    &aircraft_step,
                    "whole-aircraft B-rep, full AP203 (multi-solid)",
                ),
            ];
            for (name, content, desc) in files {
                let path = format!("{dir}/{name}");
                let _ = fs::write(&path, content);
                println!("  wrote {path} ({} bytes) — {desc}", content.len());
            }
        }
        Err(e) => println!("  could not create {dir}/: {e}"),
    }

    println!(
        "\nNote: dimensions are a first-cut from the model's stress/torsion sizing — \
         confirm critical parts (mast, grips, blade root) before flight."
    );
}
