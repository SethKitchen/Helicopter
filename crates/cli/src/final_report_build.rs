//! Build/BOM half of the final report: the buildable artifacts — part build steps +
//! assembly + structural margins, the airframe cost BOM, the priced battery shopping
//! list, the charging kits, and the exported geometry files. Split from
//! `final_report_cli` to keep each file within the size limit; the section writers are
//! `pub(crate)` and append Markdown to the shared buffer.

use std::fmt::Write as _;
use std::fs;

use helisim_actuation::ActuationPlan;
use helisim_actuation::{PowerBudget, power_budget};
use helisim_bms::{PackBuild, Target, build_pack, size_for_target};
use helisim_cell::{
    Cell, ampace_jp40, bak_45d, benchmark_cells, eve_40pl, molicel_p50b, true_continuous_current,
};
use helisim_charging::kit_120v;
use helisim_charging::kit_solar;
use helisim_cost::{AircraftSpec, UnitCosts, build_bom, summarize};
use helisim_design::{DesignCandidate, DesignReport};
use helisim_manufacture::{
    aircraft_to_step_ap203, aircraft_to_stl, airfoil_to_dxf, analyze_blade_fatigue,
    analyze_bond_creep, analyze_fatigue, analyze_resonance, analyze_root_hole, analyze_root_solid,
    analyze_thermal_softening, assembly_svg, blade_from_design_tapered, blade_section_svg,
    blade_to_step_brep, build_package_lofted, check_structure, hardware_schedule,
    lofted_blade_to_stl, retention_bolt, rotor_head_svg, select_bearing, shopping_list,
    shopping_total, swashplate_svg,
};

use crate::final_report_cli::OUT_DIR;

/// Build package (report §3): parts + assembly + structural margins + FEA + hardware.
/// The blade carries the optimized washout `twist_deg` + `taper_ratio` (printed from
/// the loft, so the twist costs nothing to make).
pub(crate) fn build_section(
    md: &mut String,
    c: &DesignCandidate,
    rep: &DesignReport,
    twist_deg: f64,
    taper_ratio: f64,
) {
    let pkg = build_package_lofted(c, rep, twist_deg, taper_ratio);
    let _ = writeln!(md, "## 3. Build package — parts ({})\n", pkg.parts.len());
    for part in &pkg.parts {
        let dims: Vec<String> = part
            .key_dimensions_mm()
            .iter()
            .map(|(k, v)| format!("{k} {v:.1} mm"))
            .collect();
        let _ = writeln!(
            md,
            "### {} [{}] — {}",
            part.name(),
            part.source().label(),
            part.material()
        );
        let _ = writeln!(md, "- Dimensions: {}", dims.join(", "));
        for step in part.build_steps() {
            let _ = writeln!(md, "- {step}");
        }
        let _ = writeln!(md);
    }

    let _ = writeln!(md, "### Assembly sequence\n");
    for step in &pkg.assembly_steps {
        let _ = writeln!(md, "1. {step}");
    }

    let s = check_structure(c, rep, 40.0e6, 200.0e6);
    let _ = writeln!(md, "\n### Structural margins (flight loads)\n");
    let _ = writeln!(
        md,
        "Blade centrifugal force {:.0} N → min retention bolt Ø {:.1} mm.\n",
        s.blade_centrifugal_n,
        s.min_bolt_diameter_m * 1000.0
    );
    let _ = writeln!(md, "| Part | Load | σ (MPa) | Allowable | MS | |");
    let _ = writeln!(md, "|---|---|---|---|---|---|");
    for it in &s.items {
        let _ = writeln!(
            md,
            "| {} | {} | {:.1} | {:.1} | {:+.2} | {} |",
            it.part,
            it.load,
            it.actual_mpa,
            it.allowable_mpa,
            it.margin_of_safety,
            if it.ok { "OK" } else { "FAIL" }
        );
    }
    let _ = writeln!(
        md,
        "\n{} (min MS {:+.2}). FEA refines deflection.\n",
        if s.all_pass {
            "All pass"
        } else {
            "REVIEW: a part fails"
        },
        s.min_margin
    );

    let fea = helisim_manufacture::run_fea(c, rep);
    let _ = writeln!(md, "### FEA (beam deflection, FE vs closed form)\n");
    let _ = writeln!(
        md,
        "_As-built blade stiffness: material {:.0} GPa × infill knockdown (shell fraction {:.2}) → \
         effective **{:.1} GPa** — the deflection below uses this, NOT a solid laminate._",
        fea.blade_material_e_gpa, fea.blade_wall_fraction, fea.blade_effective_e_gpa
    );
    let span_mm = (c.radius_m - c.root_cutout * c.radius_m) * 1000.0;
    if fea.blade.tip_deflection_m * 1000.0 > span_mm {
        let _ = writeln!(
            md,
            "_(The non-rotating blade static deflection exceeds the span — a linear-beam \
             extrapolation past its valid range. It just means a printed blade is far too floppy \
             UNSUPPORTED; the **spun-up** value is the real flight stiffness — centrifugal tension \
             is what makes the blade rigid.)_"
        );
    }
    for part in [&fea.boom, &fea.blade] {
        let stiff = match part.tip_deflection_stiffened_m {
            Some(sft) => format!(" → {:.1} mm spun-up (centrifugal stiffening)", sft * 1000.0),
            None => String::new(),
        };
        let _ = writeln!(
            md,
            "- **{}**: tip deflection {:.1} mm{}; FE σ {:.1} vs closed-form {:.1} MPa [{}]",
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
    // Resonance (Campbell/fan) check — a flexible printed part on a per-rev harmonic
    // shakes the aircraft apart even with positive static margins.
    let res = analyze_resonance(c, rep);
    let _ = writeln!(
        md,
        "\n### Resonance (Campbell check — natural freq vs rotor harmonics)\n"
    );
    let _ = writeln!(
        md,
        "Rotor 1/rev = {:.0} Hz ({:.0} rpm); blade-passage = {}/rev. A part is unsafe if its \
         natural frequency sits within ±10% of any harmonic.\n",
        res.rotor_hz,
        res.rotor_hz * 60.0,
        res.n_blades
    );
    let _ = writeln!(md, "| Part | Natural freq | Per-rev order | Clear? |");
    let _ = writeln!(md, "|---|---|---|---|");
    let _ = writeln!(
        md,
        "| Blade flap (spun-up) | {:.0} Hz | {:.1}/rev | {} |",
        res.blade_flap_hz,
        res.blade_per_rev,
        if res.blade_resonant { "**NO**" } else { "yes" }
    );
    let _ = writeln!(
        md,
        "| Tail boom (bending) | {:.0} Hz | {:.1}/rev | {} |",
        res.boom_hz,
        res.boom_per_rev,
        if res.boom_resonant || res.boom_per_rev < 1.0 {
            "**NO**"
        } else {
            "yes"
        }
    );
    for n in &res.notes {
        let _ = writeln!(md, "- {n}");
    }
    let _ = writeln!(
        md,
        "→ {}\n",
        if res.feasible {
            "All parts clear the rotor harmonics."
        } else {
            "REVIEW: a part is near a harmonic — apply the retune/replacement above before flight."
        }
    );

    // Bonded-root stress concentration at the bolt hole (CST + closed-form Kt).
    let hole = analyze_root_hole(c);
    let _ = writeln!(md, "\n### Bonded-root stress concentration (bolt hole)\n");
    let _ = writeln!(
        md,
        "_(This subsection analyses the **bonded-doubler** root used by the recommended SLS-route \
         blade; a smaller desktop continuous-fiber blade instead carries `F_cf` in the fiber loop — \
         see the `root fiber loop` row in the margins table.)_\n"
    );
    let _ = writeln!(
        md,
        "The net-section margin above is a nominal `F/A`; the hole concentrates it. Net-section \
         Kt (Heywood, d/w = {:.2}) = **{:.2}** (CST plane-stress FE confirms a concentration, \
         Kt_FE ≈ {:.1} — coarse, under-predicts). Peak hole stress {:.1} MPa vs {:.0} MPa allowable \
         → MS {:+.2} ({}).\n",
        hole.d_over_w,
        hole.kt_closed_form,
        hole.kt_fe,
        hole.peak_stress_mpa,
        hole.allowable_mpa,
        hole.margin_of_safety,
        if hole.ok { "OK" } else { "FAIL" }
    );
    let solid = analyze_root_solid(c);
    let _ = writeln!(
        md,
        "**3-D solid FE** (linear tetrahedra): the analytical Kt {:.2} is **bracketed** by the two \
         FE routes (coarse CST {:.1} under, finer 3-D {:.1} over — {}); through-thickness σ ratio \
         {:.2} (≈1 ⇒ the thin doubler is plane-stress, no through-thickness gradient). Bolt bearing \
         (cosine contact distribution, peak 4/π × average): {:.0} MPa vs {:.0} MPa → {}.\n",
        solid.kt_closed_form,
        solid.kt_cst,
        solid.kt_3d,
        if solid.routes_agree {
            "consistent"
        } else {
            "REVIEW"
        },
        solid.through_thickness_ratio,
        solid.bearing_peak_mpa,
        solid.bearing_allowable_mpa,
        if solid.bearing_ok { "OK" } else { "FAIL" }
    );

    // Fatigue over the 10-year service life (GAG + per-rev, Basquin + Miner).
    let fat = analyze_fatigue(c, rep, 365.0, 10.0, 20.0);
    let _ = writeln!(
        md,
        "### Fatigue — root doubler over 10 years (Basquin S-N + Miner)\n"
    );
    let _ = writeln!(
        md,
        "| Spectrum | Cycles | Stress | Allowable cycles | Damage |"
    );
    let _ = writeln!(md, "|---|---|---|---|---|");
    let _ = writeln!(
        md,
        "| GAG (per flight) | {:.0} | {:.1} MPa peak | {:.1e} | {:.3} |",
        fat.gag_cycles, fat.gag_peak_mpa, fat.gag_allowable, fat.gag_damage
    );
    let _ = writeln!(
        md,
        "| Per-rev (1/rev) | {:.1e} | {:.1} MPa alt | {:.1e} | {:.3} |",
        fat.hcf_cycles, fat.hcf_alt_mpa, fat.hcf_allowable, fat.hcf_damage
    );
    let life = if fat.predicted_life_years > 1000.0 {
        "≫1000 years (stresses are below the fatigue limit)".to_string()
    } else {
        format!("{:.0} years", fat.predicted_life_years)
    };
    let _ = writeln!(
        md,
        "\nAluminium doubler Miner damage over 10 yr = **{:.3}** → fatigue life **{life}** ({}).\n",
        fat.total_damage,
        if fat.meets_life {
            "meets the 10-year target"
        } else {
            "BELOW target — upsize the doubler/bolt"
        }
    );

    // Blade SPAR (polymer) per-rev fatigue — a separate S-N from the aluminium.
    let bf = analyze_blade_fatigue(c, 365.0, 10.0, 20.0);
    let bl = if bf.predicted_life_years > 1000.0 {
        "≫1000 years".to_string()
    } else {
        format!("{:.0} years", bf.predicted_life_years)
    };
    let _ = writeln!(
        md,
        "**Blade spar (nylon) HCF**: 1-g flap {:.0} MPa → per-rev alternating {:.1} MPa (after \
         flap-hinge relief) over {:.1e} cycles; polymer S-N damage {:.3} → blade fatigue life \
         **{bl}** ({}).\n",
        bf.flap_1g_mpa,
        bf.alt_mpa,
        bf.cycles,
        bf.damage,
        if bf.meets_life {
            "meets the target"
        } else {
            "BELOW target — thicken the root / add a flap hinge"
        }
    );

    // Epoxy-bond creep under the sustained centrifugal load over the loaded hours.
    let creep = analyze_bond_creep(c, 35.0, 365.0, 10.0, 20.0);
    let _ = writeln!(
        md,
        "**Epoxy-bond creep**: {:.2} MPa sustained shear over {:.0} loaded hours at {:.0} °C; \
         creep-rupture allowable {:.2} MPa (static × time {:.2} × temp {:.2}) → MS {:+.2} ({}).\n",
        creep.sustained_shear_mpa,
        creep.loaded_hours,
        creep.temp_c,
        creep.creep_allowable_mpa,
        creep.time_factor,
        creep.temp_factor,
        creep.margin_of_safety,
        if creep.ok {
            "OK"
        } else {
            "FAIL — bigger bond area or a higher-temp adhesive"
        }
    );

    // Thermal softening of the printed blade (sun-baked).
    let th = analyze_thermal_softening(c, 35.0);
    let _ = writeln!(
        md,
        "### Thermal softening — blade in the sun (35 °C ambient)\n"
    );
    let _ = writeln!(
        md,
        "Solar balance → blade ≈ **{:.0} °C**; printed-nylon retention there: modulus {:.0}%, \
         strength {:.0}%. Centrifugal root stress {:.1} MPa vs hot allowable {:.1} MPa → MS {:+.2} \
         ({}). {}\n",
        th.blade_temp_c,
        th.modulus_retention * 100.0,
        th.strength_retention * 100.0,
        th.root_stress_mpa,
        th.hot_allowable_mpa,
        th.margin_of_safety,
        if th.ok { "OK" } else { "FAIL" },
        if th.ok {
            "Use a light-coloured (low-α) blade or a higher-Tg material for hot climates."
        } else {
            "REPLACE: a higher-Tg material (e.g. PA-CF over neat nylon) or a light colour / shade is required."
        }
    );

    let _ = writeln!(md, "### Hardware schedule (standard parts by load)\n");
    for h in hardware_schedule(c, rep) {
        let _ = writeln!(md, "- **{}**: {} — {}", h.joint, h.part, h.detail);
    }
    let _ = writeln!(md);
}

/// 4. Cost bill of materials + actuation purchase list.
pub(crate) fn cost_section(
    md: &mut String,
    c: &DesignCandidate,
    rep: &DesignReport,
    act: &ActuationPlan,
) {
    let m = c.gross_mass_kg;
    let spec = AircraftSpec {
        n_blades: c.n_blades,
        blade_mass_kg: 0.03 * m / c.n_blades as f64,
        hub_mass_kg: 0.05 * m,
        structure_mass_kg: 0.40 * m,
        powertrain_mass_kg: 0.12 * m,
        motor_power_kw: 2.0 * rep.hover_shaft_power_w / 1000.0,
        pack_energy_wh: c.pack_energy_wh,
        pack_mass_kg: 0.25 * m,
    };
    let cost = summarize(&build_bom(&spec, &UnitCosts::default()));
    let _ = writeln!(md, "## 4. Cost — airframe bill of materials\n");
    let _ = writeln!(
        md,
        "Parametric (representative unit costs, overridable). Total **${:.0}**, \
         vertical-integration index **{:.0}%** (cost-weighted self-build fraction), \
         purchased-cost fraction {:.0}%.\n",
        cost.total_cost,
        cost.vertical_integration_index * 100.0,
        cost.purchased_cost_fraction * 100.0
    );
    let _ = writeln!(md, "| Subsystem | Cost |");
    let _ = writeln!(md, "|---|---|");
    for (sub, cst) in &cost.by_subsystem {
        let _ = writeln!(md, "| {sub} | ${cst:.0} |");
    }
    let _ = writeln!(md, "\nIrreducible buy-items (cannot be self-made):\n");
    for (name, cst) in cost.buy_items.iter().take(6) {
        let _ = writeln!(md, "- {name} — ${cst:.0}");
    }
    let _ = writeln!(
        md,
        "\n### Actuation (motor + servos selected by load — buy links)\n"
    );
    for line in act.purchase_lines() {
        let _ = writeln!(md, "- {line}");
    }
    let _ = writeln!(md, "\nTotal actuation mass {:.2} kg.\n", act.total_mass_kg);

    // Hardware + consumables (every fitting the build steps call for, buyable).
    let shop = shopping_list(c, rep);
    let _ = writeln!(
        md,
        "### Hardware & consumables (buyable — direct product links)\n"
    );
    let _ = writeln!(
        md,
        "Each line is a **direct product page** (specific Amazon listing / retailer / service), \
         captured 2026-06-17 — **click for the live price** (Amazon/retailer prices change and are \
         not machine-readable here, so the $ shown is a representative estimate; the cut/print \
         SERVICES are per-job instant quotes from uploading the file). Size-dependent fasteners fall \
         back to an exact-size search if your design selects a non-default bolt/bearing.\n"
    );
    let _ = writeln!(md, "| Item | Qty | Unit $ | Source | For |");
    let _ = writeln!(md, "|---|---|---|---|---|");
    for it in &shop {
        let _ = writeln!(
            md,
            "| {} | {:.0} | {:.2} | [{}]({}) | {} |",
            it.item, it.qty, it.usd, it.retailer, it.url, it.note
        );
    }
    let _ = writeln!(
        md,
        "| **TOTAL** | | **${:.2}** | | |",
        shopping_total(&shop)
    );
    let _ = writeln!(md);
}

/// 5. Battery pack + BMS shopping list (priced, buy links) + assembly.
pub(crate) fn battery_section(
    md: &mut String,
    c: &DesignCandidate,
    act: &ActuationPlan,
) -> PackBuild {
    let budget: PowerBudget = power_budget(act);
    let energy_wh = c.pack_energy_wh.max(10.0); // the mission+life-sized nameplate
    let target = Target {
        bus_voltage_v: budget.pack_voltage_v,
        peak_power_w: budget.total_pack_power_w,
        energy_wh,
    };
    let cell_name = lightest_cell_for(target);
    let cell = cell_by_name(cell_name);
    let series = act.cells.max(1) as usize;
    let tc = true_continuous_current(cell_name).unwrap();
    let p_power = (budget.pack_peak_current_a / tc).ceil();
    let cell_wh = cell.nominal_voltage() * cell.capacity_ah();
    let p_energy = (energy_wh / (series as f64 * cell_wh)).ceil();
    let parallel = p_power.max(p_energy).max(1.0) as usize;
    let pack = build_pack(
        cell_name,
        cell.as_ref(),
        series,
        parallel,
        budget.pack_peak_current_a,
    );

    let _ = writeln!(md, "## 5. Battery pack + BMS shopping list\n");
    let _ = writeln!(
        md,
        "Pack: **{} — {}S{}P** ({} cells), {:.1} V nom, {:.1} Ah, {:.0} Wh, {:.2} kg, peak {:.0} A. \
         Sized to the {:.0} W motor+actuator budget and the 10-year life-pack capacity.\n",
        pack.cell_name,
        pack.series,
        pack.parallel,
        pack.cell_count,
        pack.nominal_v,
        pack.capacity_ah,
        pack.energy_wh,
        pack.mass_kg,
        pack.peak_current_a,
        budget.total_pack_power_w
    );
    let _ = writeln!(md, "| Item | Qty | Unit $ | Line $ | Source |");
    let _ = writeln!(md, "|---|---|---|---|---|");
    for l in &pack.lines {
        let _ = writeln!(
            md,
            "| {} | {:.0} | {:.2} | {:.2} | {} {} |",
            l.item,
            l.qty,
            l.unit_price.usd,
            l.line_total_usd(),
            l.unit_price.retailer,
            l.unit_price.url
        );
    }
    let _ = writeln!(
        md,
        "| **PARTS TOTAL** | | | **${:.2}** | |",
        pack.parts_total_usd()
    );
    let _ = writeln!(md, "\nOne-time tools:\n");
    for l in &pack.tools {
        let _ = writeln!(
            md,
            "- {} ×{:.0} — ${:.2} ({})",
            l.item,
            l.qty,
            l.line_total_usd(),
            l.unit_price.retailer
        );
    }
    let _ = writeln!(md, "\n### Pack assembly instructions\n");
    for step in &pack.instructions {
        let _ = writeln!(md, "1. {step}");
    }
    let _ = writeln!(md);
    pack
}

/// 6. Charging — the 1:1 consequence of the life oversize.
pub(crate) fn charging_section(md: &mut String, rep: &DesignReport) {
    let flight_w = rep.hover_elec_power_w;
    let _ = writeln!(
        md,
        "## 6. Charging (toward 1:1 — charge power = flight power)\n"
    );
    let _ = writeln!(
        md,
        "Flight (hover) electrical power **{flight_w:.0} W**. Because the 10-year life-pack is \
         oversized, the flight C-rate is low, so a 1:1 charge (charge as fast as you fly) is a \
         *gentle* low-C charge — life and fast turnaround reinforce each other. Kits sized to it:\n"
    );
    let _ = writeln!(
        md,
        "| Source | Charge power | Charge:flight | Meets 1:1? | Equipment $ |"
    );
    let _ = writeln!(md, "|---|---|---|---|---|");
    for k in [kit_120v(flight_w), kit_solar(flight_w)] {
        let _ = writeln!(
            md,
            "| {} | {:.0} W | {:.1}:1 | {} | ${:.0} |",
            k.source,
            k.charge_power_w,
            k.charge_flight_ratio,
            if k.reaches_unity() {
                "✓"
            } else {
                "below 1:1"
            },
            k.equipment_total_usd()
        );
    }
    let _ = writeln!(
        md,
        "\nFor best life, trickle the daily energy overnight; a standard 120 V outlet already \
         meets or exceeds this model's flight power.\n"
    );
}

/// 8. Geometry files — write them and list in the report.
pub(crate) fn geometry_section(
    md: &mut String,
    c: &DesignCandidate,
    rep: &DesignReport,
    twist_deg: f64,
    taper_ratio: f64,
) -> Vec<(String, usize, &'static str)> {
    let blade = blade_from_design_tapered(c, twist_deg, taper_ratio);
    // Head hardware for the rotor-head diagram (same selectors as the schedule).
    let omega = c.omega();
    let span = c.radius_m - c.root_cutout * c.radius_m;
    let m_blade = c.blade_areal_density_kg_m2 * c.chord_m * span;
    let r_cg = 0.5 * (c.radius_m + c.root_cutout * c.radius_m);
    let f_cf = omega * omega * m_blade * r_cg;
    let bolt_mm = retention_bolt(f_cf).diameter_mm;
    let brg = select_bearing(3.0, f_cf, 1.5)
        .map(|b| b.name)
        .unwrap_or("623");

    let mut written: Vec<(String, usize, &'static str)> = Vec::new();
    let _ = fs::create_dir_all(OUT_DIR);
    let payloads: [(&str, String, &'static str); 9] = [
        (
            "blade.stl",
            lofted_blade_to_stl(&blade, 24, 80),
            "lofted blade solid (printable)",
        ),
        (
            "blade_section.dxf",
            airfoil_to_dxf(&blade.section_contour_mm(120)),
            "NACA section profile (cuttable)",
        ),
        (
            "aircraft.stl",
            aircraft_to_stl(c, rep),
            "full-aircraft assembly mesh",
        ),
        (
            "blade.step",
            blade_to_step_brep(&blade, 24, 60),
            "B-rep SOLID (MANIFOLD_SOLID_BREP)",
        ),
        (
            "aircraft.step",
            aircraft_to_step_ap203(c, rep),
            "whole-aircraft AP203 B-rep",
        ),
        (
            "blade_section.svg",
            blade_section_svg(&blade),
            "DIAGRAM: blade section + spar + dims (§3)",
        ),
        (
            "rotor_head.svg",
            rotor_head_svg(bolt_mm, brg),
            "DIAGRAM: rotor-head load path (§3/§7)",
        ),
        (
            "swashplate.svg",
            swashplate_svg(c.n_blades),
            "DIAGRAM: swashplate/CCPM control (§7)",
        ),
        (
            "aircraft.svg",
            assembly_svg(c, rep),
            "DIAGRAM: aircraft layout (side elevation)",
        ),
    ];
    let _ = writeln!(
        md,
        "## 8. Geometry + diagram files (written to `{OUT_DIR}/`)\n"
    );
    let _ = writeln!(md, "| File | Bytes | Description |");
    let _ = writeln!(md, "|---|---|---|");
    for (name, content, desc) in payloads {
        let path = format!("{OUT_DIR}/{name}");
        let _ = fs::write(&path, &content);
        let _ = writeln!(md, "| `{name}` | {} | {desc} |", content.len());
        written.push((name.to_string(), content.len(), desc));
    }
    let _ = writeln!(
        md,
        "\nImport the `.step` into any CAD, the `.stl` into a slicer, the `.dxf` into a 2-D cutter. \
         Open the **`.svg` diagrams in any browser** — they show the blade section, the rotor-head \
         load path, and the swashplate/CCPM linkage that the build steps (§3, §7) describe.\n"
    );
    let _ = writeln!(
        md,
        "**Geometry honesty:** `blade.stl` is the WHOLE outer blade solid — it is printed in one \
         piece (desktop, or via the SLS service in §4), so there is no internal spar channel to \
         model and the retention-bolt hole is PRINTED (an undersized pilot, then reamed to size — \
         never drilled from solid, which would delaminate the root). The bolt bears on a BONDED \
         steel bushing (not a press-fit — a polymer interference fit relaxes). The centrifugal load \
         path then follows the print route: a desktop continuous-fiber blade winds the fiber as a \
         LOOP around the bushing (F_cf in fiber tension); the larger SLS blade (chopped fiber, no \
         loop) uses bonded aluminium doublers. Every such part, and the tools/services to make the \
         cuts, is in the §4 shopping list with a link.\n"
    );
    written
}

// --- battery cell helpers (mirror battery-build) ---

fn cell_by_name(name: &str) -> Box<dyn Cell> {
    match name {
        "Molicel P50B" => Box::new(molicel_p50b()),
        "Ampace JP40" => Box::new(ampace_jp40()),
        "BAK 45D" => Box::new(bak_45d()),
        _ => Box::new(eve_40pl()),
    }
}

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
