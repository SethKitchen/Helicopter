//! Data export for the web design studio: turn the recommended design into the
//! JSON the browser UI renders. Two files:
//!
//! * `geometry.json` — every aircraft solid as flat-shaded triangle data
//!   (positions + per-vertex normals), one entry per pickable part.
//! * `manifest.json` — the design summary, per-part build metadata (material,
//!   source, key dimensions, build steps), the structural margins, the FEA field,
//!   and the assembly sequence.
//!
//! JSON is hand-written (zero dependencies), the same discipline as the project's
//! hand-rolled STL/STEP/DXF writers. The geometry reuses the *validated*
//! [`helisim_manufacture::mesh`] solids — the UI shows exactly what `build` makes.

use crate::studio_scene::studio_scene;
use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_design::{DesignCandidate, DesignReport, DesignSpace, ScoredCandidate, recommend};
use helisim_manufacture::mesh::Tri;
use helisim_manufacture::{build_package, check_structure, run_fea};

/// The two JSON documents the UI needs.
pub struct Bundle {
    pub geometry: String,
    pub manifest: String,
}

/// Escape a string for embedding in JSON.
fn esc(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

/// `"key":"value"` JSON string field.
fn fs_(key: &str, val: &str) -> String {
    format!("\"{}\":\"{}\"", key, esc(val))
}

/// JSON-safe number: 0.0 for non-finite (NaN/Inf aren't valid JSON).
fn nan0(x: f64) -> f64 {
    if x.is_finite() { x } else { 0.0 }
}

/// Flat-shaded triangle data for one mesh: positions and per-vertex normals as
/// flat f64 arrays (3 vertices/triangle, normal repeated per vertex).
fn mesh_arrays(tris: &[Tri]) -> (String, String) {
    let mut pos = String::from("[");
    let mut nrm = String::from("[");
    for (i, t) in tris.iter().enumerate() {
        if i > 0 {
            pos.push(',');
            nrm.push(',');
        }
        let (a, b, c) = (t.0, t.1, t.2);
        // Facet normal (flat shading); unit, 0 if degenerate.
        let (ux, uy, uz) = (b.x - a.x, b.y - a.y, b.z - a.z);
        let (vx, vy, vz) = (c.x - a.x, c.y - a.y, c.z - a.z);
        let (mut nx, mut ny, mut nz) = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        if len > 0.0 {
            nx /= len;
            ny /= len;
            nz /= len;
        }
        pos.push_str(&format!(
            "{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2},{:.2}",
            a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z
        ));
        let n = format!("{nx:.4},{ny:.4},{nz:.4}");
        nrm.push_str(&format!("{n},{n},{n}"));
    }
    pos.push(']');
    nrm.push(']');
    (pos, nrm)
}

/// Classify a [`build_package`] spec name into the studio's group vocabulary, so
/// the 3D part finds its real material / dimensions / build steps. Order matters:
/// "rotor hub + blade grips" contains "blade", so hub is matched before blade.
fn group_for_name(name: &str) -> &'static str {
    let n = name.to_lowercase();
    let has = |kw: &str| n.contains(kw);
    if has("fuselage") || has("canopy") {
        "fuselage"
    } else if has("tail rotor") {
        "tail_rotor"
    } else if has("boom") {
        "boom"
    } else if has("swashplate") {
        "swashplate"
    } else if has("hub") || has("grip") {
        "hub"
    } else if has("mast") {
        "mast"
    } else if has("blade") {
        "blade"
    } else if has("landing") || has("skid") {
        "landing_gear"
    } else if has("tray") || has("powertrain") || has("mount") || has("motor") {
        "motor"
    } else {
        ""
    }
}

/// A denser search grid for the studio than [`DesignSpace::model_default`]'s 120
/// corners — finer steps in every dimension so the displayed "best" is chosen from
/// a far larger candidate set. (model_default is left untouched: its winner is
/// pinned by the design-crate tests.) The continuous Nelder–Mead optimiser
/// (`optimize_design`, used by `final-report`) goes further still — the true
/// optimum *between* grid points, not just the best corner.
fn studio_space() -> DesignSpace {
    let step = |lo: f64, hi: f64, dx: f64| {
        let n = ((hi - lo) / dx).round() as usize;
        (0..=n).map(|i| lo + i as f64 * dx).collect::<Vec<_>>()
    };
    DesignSpace {
        blade_counts: vec![2, 3, 4, 5],
        radii_m: step(0.40, 0.80, 0.05),        // 9
        tip_speeds_ms: step(90.0, 150.0, 10.0), // 7
        solidities: step(0.05, 0.11, 0.01),     // 7
        min_flare_margin: 1.5,
        min_endurance_min: 10.0,
        max_tip_mach: 0.55,
        envelope: None,
        sizing: None,
    }
}

/// Build the geometry + manifest bundle from the recommended design.
/// Returns `None` if no design meets the constraints.
pub fn build_bundle() -> Option<Bundle> {
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    let space = studio_space();

    let rec = recommend(&space, &base, &af, &cfg)?;
    let c = rec.best.candidate;
    let report = rec.best.report;

    let geometry = geometry_json(&c, &report);
    let manifest = manifest_json(
        &c,
        &report,
        &rec.rationale,
        rec.n_evaluated,
        rec.n_feasible,
        &rec.ranked,
        &rec.pareto,
    );
    Some(Bundle { geometry, manifest })
}

fn geometry_json(c: &DesignCandidate, report: &DesignReport) -> String {
    let parts = studio_scene(c, report);

    let mut out = String::from("{\n  \"units\":\"mm\",\n  \"parts\":[\n");
    for (i, p) in parts.iter().enumerate() {
        let (pos, nrm) = mesh_arrays(&p.tris);
        if i > 0 {
            out.push_str(",\n");
        }
        out.push_str(&format!(
            "    {{{},{},{},\"smooth\":{},\"tris\":{},\"positions\":{},\"normals\":{}}}",
            fs_("id", &p.id),
            fs_("name", &p.name),
            fs_("group", p.group),
            p.smooth,
            p.tris.len(),
            pos,
            nrm
        ));
    }
    out.push_str("\n  ]\n}\n");
    out
}

fn manifest_json(
    c: &DesignCandidate,
    report: &DesignReport,
    rationale: &[String],
    n_evaluated: usize,
    n_feasible: usize,
    ranked: &[ScoredCandidate],
    pareto: &[ScoredCandidate],
) -> String {
    let pkg = build_package(c, report);
    let structure = check_structure(c, report, 40.0e6, 200.0e6);
    let fea = run_fea(c, report);
    let omega = c.omega();

    let mut out = String::from("{\n");

    // --- design summary ---
    out.push_str("  \"design\":{\n");
    let d = [
        ("blades", c.n_blades as f64, ""),
        ("radius_m", c.radius_m, "m"),
        ("tip_speed_mps", c.tip_speed_ms, "m/s"),
        ("solidity", c.solidity(), ""),
        ("rpm", omega * 60.0 / (2.0 * std::f64::consts::PI), "rpm"),
        ("gross_mass_kg", c.gross_mass_kg, "kg"),
        ("figure_of_merit", report.figure_of_merit, ""),
        ("hover_shaft_power_w", report.hover_shaft_power_w, "W"),
        ("hover_endurance_min", report.endurance_min, "min"),
        ("flare_margin", report.flare_margin, ""),
        ("oaspl_db", report.oaspl_db, "dB"),
        ("candidates_evaluated", n_evaluated as f64, ""),
        ("candidates_feasible", n_feasible as f64, ""),
    ];
    for (i, (k, v, u)) in d.iter().enumerate() {
        if i > 0 {
            out.push_str(",\n");
        }
        out.push_str(&format!(
            "    \"{k}\":{{\"value\":{:.4},\"unit\":\"{u}\"}}",
            if v.is_finite() { *v } else { 0.0 }
        ));
    }
    out.push_str("\n  },\n");

    // --- rationale ---
    out.push_str("  \"rationale\":[");
    for (i, line) in rationale.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str(&format!("\"{}\"", esc(line)));
    }
    out.push_str("],\n");

    // --- per-part build specs (keyed by group keyword so the 3D part finds it) ---
    out.push_str("  \"parts\":[\n");
    for (i, part) in pkg.parts.iter().enumerate() {
        if i > 0 {
            out.push_str(",\n");
        }
        // Which 3D group does this spec correspond to (if any)?
        let name = part.name();
        let group = group_for_name(name);
        let dims: String = part
            .key_dimensions_mm()
            .iter()
            .enumerate()
            .map(|(j, (k, v))| {
                format!(
                    "{}{{\"label\":\"{}\",\"mm\":{:.2}}}",
                    if j > 0 { "," } else { "" },
                    esc(k),
                    v
                )
            })
            .collect();
        let steps: String = part
            .build_steps()
            .iter()
            .enumerate()
            .map(|(j, s)| format!("{}\"{}\"", if j > 0 { "," } else { "" }, esc(s)))
            .collect();
        out.push_str(&format!(
            "    {{{},{},{},{},\"dims\":[{}],\"steps\":[{}]}}",
            fs_("name", name),
            fs_("group", group),
            fs_("material", part.material()),
            fs_("source", part.source().label()),
            dims,
            steps
        ));
    }
    out.push_str("\n  ],\n");

    // --- structural margins ---
    out.push_str(&format!(
        "  \"structure\":{{\"blade_centrifugal_n\":{:.1},\"min_bolt_d_mm\":{:.2},\"all_pass\":{},\"min_margin\":{:.3},\"items\":[\n",
        structure.blade_centrifugal_n,
        structure.min_bolt_diameter_m * 1000.0,
        structure.all_pass,
        if structure.min_margin.is_finite() { structure.min_margin } else { 0.0 }
    ));
    for (i, it) in structure.items.iter().enumerate() {
        if i > 0 {
            out.push_str(",\n");
        }
        out.push_str(&format!(
            "    {{{},{},\"actual_mpa\":{:.2},\"allowable_mpa\":{:.2},\"ms\":{:.3},\"ok\":{}}}",
            fs_("part", it.part),
            fs_("load", &it.load),
            it.actual_mpa,
            it.allowable_mpa,
            it.margin_of_safety,
            it.ok
        ));
    }
    out.push_str("\n  ]},\n");

    // --- FEA field (beam tip deflection + stress, per part) ---
    out.push_str("  \"fea\":[\n");
    for (i, part) in [&fea.boom, &fea.blade].iter().enumerate() {
        if i > 0 {
            out.push_str(",\n");
        }
        let stiff = match part.tip_deflection_stiffened_m {
            Some(s) => format!("{:.4}", s * 1000.0),
            None => "null".to_string(),
        };
        out.push_str(&format!(
            "    {{{},\"tip_deflection_mm\":{:.4},\"tip_deflection_stiffened_mm\":{},\"fe_stress_mpa\":{:.3},\"closed_form_stress_mpa\":{:.3},\"routes_agree\":{}}}",
            fs_("name", part.name),
            part.tip_deflection_m * 1000.0,
            stiff,
            part.fe_stress_mpa,
            part.closed_form_stress_mpa,
            part.routes_agree
        ));
    }
    out.push_str("\n  ],\n");

    // --- mass / balance: components → CG → trim-attitude effect ---
    let bal = crate::mass_properties::balance(c, report);
    out.push_str(&format!(
        "  \"balance\":{{\"total_mass_kg\":{:.3},\"cg_mm\":[{:.1},{:.1},{:.1}],\"inertia_kg_m2\":[{:.5},{:.5},{:.5}],\"cg_offset_m\":{:.4},\"trim_pitch_deg\":{:.3},\"trim_pitch_centered_deg\":{:.3},\"dpitch_dcg_deg_per_m\":{:.3},\"converged\":{},\"components\":[\n",
        bal.total_mass_kg,
        bal.cg_mm[0], bal.cg_mm[1], bal.cg_mm[2],
        bal.i_xx, bal.i_yy, bal.i_zz,
        bal.cg_offset_m,
        nan0(bal.trim_pitch_deg),
        nan0(bal.trim_pitch_centered_deg),
        nan0(bal.dpitch_dcg_deg_per_m),
        bal.converged,
    ));
    for (i, p) in bal.parts.iter().enumerate() {
        if i > 0 {
            out.push_str(",\n");
        }
        out.push_str(&format!(
            "    {{{},{},{},\"mass_kg\":{:.4},\"cg_mm\":[{:.1},{:.1},{:.1}]}}",
            fs_("id", &p.id),
            fs_("name", &p.name),
            fs_("group", p.group),
            p.mass_kg,
            p.centroid_mm[0],
            p.centroid_mm[1],
            p.centroid_mm[2]
        ));
    }
    out.push_str("\n  ]");
    if let Some(stab) = &bal.stability {
        out.push_str(&format!(
            ",\"stability\":{{\"unstable\":{},\"has_unstable_oscillation\":{},\"derivatives\":{{\"mu\":{:.6},\"mq\":{:.6},\"zw\":{:.6},\"xu\":{:.6}}},\"modes\":[",
            stab.unstable,
            stab.has_unstable_oscillation,
            nan0(stab.mu),
            nan0(stab.mq),
            nan0(stab.zw),
            nan0(stab.xu),
        ));
        for (i, m) in stab.modes.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&format!(
                "{{\"re\":{:.6},\"im\":{:.6},\"stable\":{},\"oscillatory\":{},\"period_s\":{:.4},\"t_half_double_s\":{:.4}}}",
                nan0(m.re),
                nan0(m.im),
                m.stable,
                m.oscillatory,
                nan0(m.period_s),
                nan0(m.t_half_double_s),
            ));
        }
        out.push_str("]}");
    }
    let avionics = bal.parts.iter().find(|p| p.group == "avionics");
    if let Some(a) = avionics {
        let dx = (a.centroid_mm[0] - bal.cg_mm[0]) / 1000.0;
        let dz = (a.centroid_mm[2] - bal.cg_mm[2]) / 1000.0;
        out.push_str(&format!(
            ",\"avionics_effect\":{{\"mass_kg\":{:.4},\"cg_mm\":[{:.1},{:.1},{:.1}],\"pitch_inertia_delta_kg_m2\":{:.6},\"cg_shift_if_moved_100mm_aft_mm\":{:.3},\"fea_mount_load_n\":{:.3},\"cfd_frontal_area_m2\":{:.6}}}",
            a.mass_kg,
            a.centroid_mm[0], a.centroid_mm[1], a.centroid_mm[2],
            a.mass_kg * (dx * dx + dz * dz),
            (a.mass_kg * 100.0 / bal.total_mass_kg.max(1e-9)),
            a.mass_kg * 9.80665 * 6.0,
            0.16 * 0.34 * 0.01,
        ));
    }
    out.push_str("},\n");

    // --- CFD rendering metadata: analytic slices for the studio view ---
    let disk_loading = c.gross_mass_kg * 9.80665 / (std::f64::consts::PI * c.radius_m * c.radius_m);
    let tip_re = 1.225 * c.tip_speed_ms * c.chord_m / 1.81e-5;
    out.push_str(&format!(
        "  \"cfd\":{{\"disk_loading_pa\":{:.3},\"tip_re\":{:.1},\"wake_radius_m\":{:.4},\"downwash_mps\":{:.4},\"avionics_bluffness\":{:.4}}},\n",
        disk_loading,
        tip_re,
        c.radius_m,
        (disk_loading / (2.0 * 1.225)).sqrt(),
        avionics.map(|a| a.mass_kg / bal.total_mass_kg.max(1e-9)).unwrap_or(0.0),
    ));

    // --- optimality samples for the in-studio plot ---
    out.push_str("  \"optimality\":{\"ranked\":[");
    for (i, s) in ranked.iter().take(36).enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str(&format!(
            "{{\"rank\":{},\"score\":{:.5},\"radius_m\":{:.4},\"tip_speed_mps\":{:.3},\"fm\":{:.5},\"endurance_min\":{:.4},\"flare_margin\":{:.4},\"oaspl_db\":{:.4},\"pareto\":{}}}",
            i + 1,
            nan0(s.score),
            s.candidate.radius_m,
            s.candidate.tip_speed_ms,
            nan0(s.report.figure_of_merit),
            nan0(s.report.endurance_min),
            nan0(s.report.flare_margin),
            nan0(s.report.oaspl_db),
            pareto.iter().any(|p| {
                p.candidate.n_blades == s.candidate.n_blades
                    && (p.candidate.radius_m - s.candidate.radius_m).abs() < 1e-9
                    && (p.candidate.tip_speed_ms - s.candidate.tip_speed_ms).abs() < 1e-9
                    && (p.candidate.solidity() - s.candidate.solidity()).abs() < 1e-9
            }),
        ));
    }
    out.push_str("]},\n");

    // --- build tutorials: procedural operations that deserve animation ---
    out.push_str(
        "  \"tutorials\":[\
         {\"id\":\"ream\",\"title\":\"Ream a printed pilot hole\",\"definition\":\"Reaming means using a straight fluted finishing tool to bring an undersized printed pilot hole to final diameter. It removes a tiny, controlled amount of material and makes a round, concentric, close-fit bore; it is not the same as drilling from solid.\",\"steps\":[\
         \"Clamp the printed root so the pilot hole is vertical and supported on both faces.\",\
         \"Align the reamer with the printed pilot hole; keep it coaxial with the pitch axis.\",\
         \"Turn slowly while feeding through the pilot. Let the flutes cut; do not wobble or force it.\",\
         \"Clear chips, test the steel bushing, and stop when the bushing slides in without slop.\"\
         ]},\
         {\"id\":\"bond\",\"title\":\"Bond the bushing and doublers\",\"definition\":\"Bonding means scuffing, degreasing, applying structural epoxy, clamping, and letting the joint fully cure so load transfers through the adhesive layer instead of a loose press fit.\",\"steps\":[\
         \"Scuff the bushing and doubler bond faces; wipe away dust and degrease.\",\
         \"Wet the bore and metal faces with structural epoxy, keeping adhesive continuous but thin.\",\
         \"Insert the steel bushing through the reamed root and seat the doublers on both faces.\",\
         \"Clamp flat until full cure, then pass the bolt through the steel bushing, not bare plastic.\"\
         ]}\
         ],\n",
    );

    // --- assembly sequence ---
    out.push_str("  \"assembly\":[");
    for (i, step) in pkg.assembly_steps.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        out.push_str(&format!("\"{}\"", esc(step)));
    }
    out.push_str("]\n}\n");
    out
}
