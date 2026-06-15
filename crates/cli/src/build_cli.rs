//! `build` subcommand: recommend a design, then emit the COMPLETE build package —
//! every part (sized from the design), the assembly sequence, and exported
//! STL/DXF geometry files. The end-to-end realisation of the project's goal:
//! physics → recommended design → "make exactly this".

use helisim_actuation::{
    actuation_power_w, onyx_pro_structural, options_as_materials, recommend_materials,
    select_actuation, services, source_for,
};
use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_design::{DesignCandidate, DesignSpace, recommend};
use helisim_manufacture::mesh::tris_to_stl;
use helisim_manufacture::{
    aircraft_to_step_ap203, aircraft_to_stl, airfoil_to_dxf, blade_from_design, blade_joint_effect,
    blade_splice_plate, blade_split_meshes, blade_to_step_brep, build_package, check_structure,
    hardware_schedule, lofted_blade_to_stl, onyx_pro, plan_prints, plan_split, recommend_printer,
    run_fea, smallest_fitting,
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

    // Control-surface material analysis: torque/power to deflect the printed
    // surfaces, and the best Markforged Onyx Pro material (stability/weight/bend).
    println!("\n=== Control-surface material (Markforged Onyx Pro: Onyx vs Onyx+Fiberglass) ===");
    let m_c = act.servo_demand_nm;
    let mat_report = recommend_materials(&c, m_c, &onyx_pro_structural());
    let slew = act
        .cyclic_servo
        .part
        .as_ref()
        .map(|s| s.speed_s_per_60)
        .unwrap_or(0.06);
    println!(
        "  Actuation: {:.3} N·m to deflect a blade (feathering/propeller moment), \
         ≈ {:.1} W to slew it (60° in {:.3} s).",
        m_c,
        actuation_power_w(m_c, slew),
        slew
    );
    println!("  Per printed control surface — governing mode → lightest adequate material:");
    for p in &mat_report.parts {
        let pick = p.chosen.unwrap_or("NONE (beyond Onyx Pro materials)");
        println!("    {:<18} {} → use {}", p.part, p.detail, pick);
    }
    println!(
        "  Onyx 3.0 GPa/71 MPa/1.2 g·cm⁻³ vs Onyx+Fiberglass 22 GPa/200 MPa/1.5 — Fiberglass buys ~7× \
         stiffness & ~2.8× strength for ~+25% weight (Markforged Composites Datasheet)."
    );
    println!(
        "  → Verdict: print the blade and swashplate in Onyx+Fiberglass (the blade for torsional \
         control authority — spun-up flap is centrifugal-tension-dominated, not modulus-dominated; \
         the swashplate for bending stiffness); neat Onyx suffices for the low-load pitch links."
    );

    // Outsourced manufacturing: services + the material pick across in-house and
    // service options (so the recommendation reflects the wider service menu).
    println!("\n=== Outsourced manufacturing (services + cross-source material pick) ===");
    println!("  Services (instant-quote; exact cost is geometry-based — upload CAD to each):");
    for s in services() {
        println!(
            "    {:<16} {:<24} [{}] {} — {}",
            s.name,
            s.model,
            s.cost.label(),
            s.processes,
            s.quote_url
        );
    }
    println!("  Best material per control surface across in-house + service options:");
    let cross = recommend_materials(&c, m_c, &options_as_materials());
    for p in &cross.parts {
        match p.chosen.and_then(source_for) {
            Some(src) => println!(
                "    {:<18} → {} [{}] via {} ({})",
                p.part,
                src.material.name,
                src.cost.label(),
                src.process,
                src.where_made
            ),
            None => println!(
                "    {:<18} → NONE adequate (needs metal / continuous fiber)",
                p.part
            ),
        }
    }
    println!(
        "  Finding: service SLS/MJF polymers (PA12 1.5, glass 3, carbon 5 GPa) are all LESS stiff \
         than in-house continuous Fiberglass (22 GPa); only CNC metal (Al 69 GPa) beats it. So the \
         stiff surfaces (blade, swashplate) want in-house Fiberglass or outsourced CNC metal, while \
         the low-load links are cheapest as outsourced SLS nylon. Costs are quote-based — compare \
         Protolabs (fast in-house), Xometry (widest menu), and Craftcloud/PCBWay (budget)."
    );

    // Print planning: fit each part to the in-house Onyx Pro bed, split what
    // overflows, and pick each split joint's fastening (snap vs bolt by load).
    println!("\n=== Print planning (fit to build volume → split → joints) ===");
    let bed = onyx_pro();
    println!(
        "  In-house bed: {} ({:.0}×{:.0}×{:.0} mm). Per part: envelope, fit, pieces, joint:",
        bed.name, bed.x_mm, bed.y_mm, bed.z_mm
    );
    for sp in plan_prints(&c, &report, &bed) {
        let (l, w, h) = sp.bbox_mm;
        let fit = if sp.fits {
            "fits".to_string()
        } else {
            match smallest_fitting(sp.bbox_mm) {
                Some(v) => format!("OVERFLOWS → {} pcs (or 1 pc on {})", sp.pieces, v.name),
                None => format!(
                    "OVERFLOWS → {} pcs (exceeds every bed; tube stock?)",
                    sp.pieces
                ),
            }
        };
        let joint = if sp.joints > 0 {
            format!(
                "{} joint(s): {} @ {:.0} N",
                sp.joints,
                sp.joint.label(),
                sp.joint_load_n
            )
        } else {
            "—".to_string()
        };
        println!(
            "    {:<26} {:>4.0}×{:>3.0}×{:>3.0} mm  {:<34} {}",
            sp.part, l, w, h, fit, joint
        );
    }
    println!(
        "  (Piece count is a conservative solid-grid estimate; open-frame parts — the boom and \
         landing-gear skids — are naturally built as separate tubes, not sliced from a block.)"
    );
    match recommend_printer(&c, &report) {
        (Some(v), part) => println!(
            "  → A single {} bed prints every part whole (limiting part: {}).",
            v.name, part
        ),
        (None, part) => println!(
            "  → No single bed prints everything whole — the {} exceeds the largest service bed \
             (600 mm SLS); split it or use cut-to-length tube/rod stock. The blade needs the SLS \
             bed (or a 2-piece bolted-spar split for the Onyx Pro); links/fairings snap-fit.",
            part
        ),
    }

    // Joining the pieces — woven into the construction instructions.
    println!("\n  Joining the printed pieces (how to put them together):");
    for sp in plan_prints(&c, &report, &bed) {
        let steps = sp.join_instructions();
        if !steps.is_empty() {
            println!("    {} ({} pcs):", sp.part, sp.pieces);
            for s in &steps {
                println!("      - {s}");
            }
        }
    }

    // Splitting effects fed back into the physics (the blade split is the one that
    // matters — it adds mass at radius, so inertia/flare/gross-mass move, and the
    // joint becomes a structural section to check).
    println!("\n  Splitting effects on the physics (blade joint, Onyx Pro bed):");
    let bje = blade_joint_effect(&c, &bed, 200.0e6, 22.0e9); // Onyx+Fiberglass tensile / flex modulus
    if bje.joints == 0 {
        println!("    Blade prints whole on this bed — no joint, no structural/inertial effect.");
    } else {
        println!(
            "    {} joint(s); innermost at r={:.2} m carries {:.0} N centrifugal → net-section MS {:+.2} \
             ({}× {} bolts).",
            bje.joints,
            bje.joint_radius_m,
            bje.cf_at_joint_n,
            bje.net_section_margin - 1.0,
            bje.bolts_per_joint,
            bje.bolt.clone().unwrap_or_else(|| "—".into())
        );
        println!(
            "    Rotor inertia {:.4} → {:.4} kg·m² (+{:.1}% flare energy ½IΩ²); gross mass +{:.0} g \
             (joint hardware). Lock number drops with I (slower flap).",
            bje.base_inertia,
            bje.new_inertia,
            bje.flare_delta_pct,
            bje.gross_mass_delta_kg * 1000.0
        );
        println!(
            "    Splice stiffness-knockdown (η={:.0}%): +{:.2} mm flap tip deflection. Joint surface-step \
             drag ({:.1} mm step): +{:.2} W rotor profile power ({:.2}% of hover).",
            70.0,
            bje.flap_deflection_penalty_mm,
            0.2,
            bje.drag_power_penalty_w,
            100.0 * bje.drag_power_penalty_w / report.hover_shaft_power_w.max(1.0)
        );
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

            // Split-piece geometry: if the blade overflows the in-house bed, emit
            // each printable PIECE (clean capped sub-loft) plus the bolted spar
            // SPLICE PLATE as its own watertight solid with real holes — the STL
            // set fully reflects the split + its fastening, no manual CAD step.
            let blade_sp = plan_split(&blade, &onyx_pro(), bje.cf_at_joint_n.max(1.0));
            if blade_sp.pieces > 1 {
                let meshes = blade_split_meshes(&blade, blade_sp.pieces, 60, 24);
                for (i, m) in meshes.iter().enumerate() {
                    let stl = tris_to_stl(&format!("blade_piece_{}", i + 1), m);
                    let path = format!("{dir}/blade_piece_{}.stl", i + 1);
                    let _ = fs::write(&path, &stl);
                    println!(
                        "  wrote {path} ({} bytes) — printable blade piece {}/{} (clean capped solid)",
                        stl.len(),
                        i + 1,
                        blade_sp.pieces
                    );
                }
                let plate = blade_splice_plate(&blade, 2.0, 2); // M2 spar bolts
                let pstl = tris_to_stl("splice_plate", &plate);
                let ppath = format!("{dir}/splice_plate.stl");
                let _ = fs::write(&ppath, &pstl);
                println!(
                    "  wrote {ppath} ({} bytes) — bolted spar splice plate (real through-holes), {}",
                    pstl.len(),
                    blade_sp.joint.label()
                );
            }
        }
        Err(e) => println!("  could not create {dir}/: {e}"),
    }

    println!(
        "\nNote: dimensions are a first-cut from the model's stress/torsion sizing — \
         confirm critical parts (mast, grips, blade root) before flight."
    );
}
