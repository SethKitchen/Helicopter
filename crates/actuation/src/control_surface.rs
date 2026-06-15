//! How much the printed **control surfaces** bend under the actuation/flight load,
//! and which Markforged material is best (stability vs weight vs bend).
//!
//! The servo supplies a control torque (the feathering "propeller moment" from
//! [`crate::loads`]) to pitch the blade; that torque + the flight loads are
//! reacted by printed parts whose **bending** matters: if the part flexes, the
//! pitch input is lost. For each control-path part we compute, per material, the
//! deflection (the "bend"), the stress margin (stability), and the mass (weight),
//! then pick the **lightest adequate** material (the rule the rest of the crate
//! uses for parts).
//!
//! Two part classes:
//! * **Non-rotating** (swashplate arm, pitch link) — closed-form Euler-Bernoulli
//!   bending (`δ=PL³/3EI`) and Euler buckling (`P_cr=π²EI/L²`); modulus governs.
//! * **Rotor blade** — modelled with the validated [`helisim_fea`] beam INCLUDING
//!   **centrifugal tension stiffening** (the dominant effect — a static cantilever
//!   over-predicts flap deflection several-fold). Spun-up flap is therefore tension
//!   -dominated and nearly material-insensitive (the taut-string limit); the
//!   material-sensitive blade modes are **torsional wind-up** under the feathering
//!   moment (lost pitch authority, `Δθ=M_c L/GJ`) and the centrifugal+bending
//!   **stress** margin. Closed-form deflection is cross-checked against the FEA
//!   beam in the tests (the two-routes rule).

use crate::material::PrintMaterial;
use helisim_design::DesignCandidate;
use helisim_fea::beam::{Bc, Beam};
use std::f64::consts::PI;

/// Pitch-horn / control-arm radius, m (servo moment → link force `F = M/r`).
pub const R_HORN_M: f64 = 0.012;
/// Required bending-stress safety factor (strength margin).
pub const STRENGTH_SF: f64 = 2.0;
/// Required Euler-buckling safety factor for slender links.
pub const BUCKLING_SF: f64 = 3.0;
/// Max acceptable blade torsional wind-up under the feathering moment, deg
/// (control authority: this much commanded pitch is lost to twist).
pub const WINDUP_LIMIT_DEG: f64 = 0.5;

// ---------------------------------------------------------------------------
// Non-rotating control parts (swashplate arm, pitch link)
// ---------------------------------------------------------------------------

/// One printed non-rotating control-path part: a beam section + its load.
#[derive(Clone, Copy, Debug)]
pub struct ControlPart {
    pub name: &'static str,
    pub length_m: f64,
    pub area_m2: f64,
    pub i_m4: f64,
    pub c_m: f64,
    /// Transverse load, N (bending). 0 if loaded axially only.
    pub transverse_n: f64,
    /// Axial compression, N (Euler buckling). 0 if none.
    pub axial_n: f64,
    /// Max acceptable deflection as a fraction of length (control stiffness).
    pub deflection_limit_frac: f64,
}

/// Per-material analysis of one non-rotating part.
#[derive(Clone, Debug)]
pub struct PartAnalysis {
    pub part: &'static str,
    pub material: &'static str,
    pub deflection_mm: f64,
    pub deflection_frac: f64,
    pub stress_mpa: f64,
    pub strength_margin: f64,
    pub buckling_margin: Option<f64>,
    pub mass_g: f64,
    pub adequate: bool,
}

/// Analyse one non-rotating part in one material (closed-form cantilever + Euler).
pub fn analyze(part: &ControlPart, mat: &PrintMaterial) -> PartAnalysis {
    let e = mat.e_pa();
    let i = part.i_m4;
    let l = part.length_m;

    let (deflection_m, moment_nm) = if part.transverse_n > 0.0 {
        (
            part.transverse_n * l.powi(3) / (3.0 * e * i),
            part.transverse_n * l,
        )
    } else {
        (0.0, 0.0)
    };
    let stress_pa = if i > 0.0 {
        moment_nm * part.c_m / i
    } else {
        0.0
    };
    let strength_margin = if stress_pa > 0.0 {
        mat.strength_pa() / stress_pa
    } else {
        f64::INFINITY
    };
    let deflection_frac = if l > 0.0 { deflection_m / l } else { 0.0 };
    let buckling_margin = if part.axial_n > 0.0 {
        Some(PI * PI * e * i / (l * l) / part.axial_n)
    } else {
        None
    };

    let strength_ok = strength_margin >= STRENGTH_SF;
    let deflection_ok = part.transverse_n <= 0.0 || deflection_frac <= part.deflection_limit_frac;
    let buckling_ok = buckling_margin.map(|m| m >= BUCKLING_SF).unwrap_or(true);

    PartAnalysis {
        part: part.name,
        material: mat.name,
        deflection_mm: deflection_m * 1000.0,
        deflection_frac,
        stress_mpa: stress_pa / 1e6,
        strength_margin,
        buckling_margin,
        mass_g: mat.density_kg_m3() * part.area_m2 * l * 1000.0,
        adequate: strength_ok && deflection_ok && buckling_ok,
    }
}

/// The non-rotating control parts from a design + per-blade control moment `M_c`.
pub fn control_parts(c: &DesignCandidate, control_moment_nm: f64) -> Vec<ControlPart> {
    let r = c.radius_m;
    let link_force = control_moment_nm / R_HORN_M;

    let (sw_b, sw_h) = (0.020, 0.004);
    let swash = ControlPart {
        name: "swashplate arm",
        length_m: 0.12 * r,
        area_m2: sw_b * sw_h,
        i_m4: sw_b * sw_h.powi(3) / 12.0,
        c_m: sw_h / 2.0,
        transverse_n: link_force,
        axial_n: 0.0,
        deflection_limit_frac: 0.02,
    };
    let link_d = 0.004;
    let pitch_link = ControlPart {
        name: "pitch link/pushrod",
        length_m: 0.08 * r,
        area_m2: link_d * link_d,
        i_m4: link_d.powi(4) / 12.0,
        c_m: link_d / 2.0,
        transverse_n: 0.0,
        axial_n: link_force,
        deflection_limit_frac: 0.02,
    };
    vec![swash, pitch_link]
}

// ---------------------------------------------------------------------------
// Rotor blade (centrifugally stiffened flap + torsional wind-up)
// ---------------------------------------------------------------------------

/// Per-material analysis of the rotor blade — the aerodynamic control surface.
#[derive(Clone, Debug)]
pub struct BladeAnalysis {
    pub material: &'static str,
    /// Spun-up flap deflection (centrifugally stiffened), mm.
    pub flap_deflection_mm: f64,
    /// Flap deflection as a fraction of span.
    pub flap_frac: f64,
    /// Torsional wind-up under the feathering moment, deg (lost pitch authority).
    pub windup_deg: f64,
    /// Centrifugal tensile stress, MPa.
    pub centrifugal_mpa: f64,
    /// Combined (centrifugal + flap-bending) stress, MPa.
    pub total_stress_mpa: f64,
    /// Strength margin `flex_strength / σ_total`.
    pub strength_margin: f64,
    pub mass_g: f64,
    pub adequate: bool,
}

/// Analyse the rotor blade in one material, with centrifugal stiffening (flap via
/// the [`helisim_fea`] beam with tension) and torsional wind-up.
pub fn analyze_blade(
    c: &DesignCandidate,
    control_moment_nm: f64,
    mat: &PrintMaterial,
) -> BladeAnalysis {
    let omega = c.omega();
    let r = c.radius_m;
    let root_r = c.root_cutout * r;
    let span = r - root_r;
    let chord = c.chord_m;
    let t = 0.12 * chord; // NACA 0012 max thickness
    let area = 0.0822 * chord * chord; // NACA 0012 section area
    let i_flap = chord * t.powi(3) / 12.0;
    let z = i_flap / (t / 2.0);
    let mu = c.blade_areal_density_kg_m2 * chord; // mass per unit span
    let m_blade = mu * span;
    let r_cg = 0.5 * (r + root_r);
    let f_cf = omega * omega * m_blade * r_cg; // centrifugal tension at the root
    let lift_per_blade = c.gross_mass_kg * 9.80665 / c.n_blades as f64;
    let q = lift_per_blade / span; // distributed flap load

    // FEA beam: cantilever from the root, EI from the material, per-element
    // centrifugal tension T(r) = ω²μ(R² − r²)/2 (geometric stiffening).
    let n_el = 12;
    let nodes_x: Vec<f64> = (0..=n_el).map(|i| span * i as f64 / n_el as f64).collect();
    let ei = vec![mat.e_pa() * i_flap; n_el];
    let section_modulus = vec![z; n_el];
    let tension: Vec<f64> = (0..n_el)
        .map(|e| {
            let r_mid = root_r + span * (e as f64 + 0.5) / n_el as f64;
            (omega * omega * mu * (r * r - r_mid * r_mid) / 2.0).max(0.0)
        })
        .collect();
    let beam = Beam {
        nodes_x,
        ei,
        section_modulus,
        tension,
    };
    let dist = beam.uniform_load_vector(q);
    let sol = beam.solve(&[], Some(&dist), &[Bc::Clamped(0)]);
    let (flap_m, bending_pa) = match sol {
        Some(s) => (s.max_deflection_m, s.max_stress_pa),
        None => (f64::NAN, f64::NAN),
    };

    // Torsional wind-up Δθ = M_c L / (G J), J ≈ (1/3) chord t³ (thin solid section).
    // Upper bound (the feathering moment taken at the tip; distributed → less).
    let j = chord * t.powi(3) / 3.0;
    let windup_rad = control_moment_nm * span / (mat.shear_modulus_pa() * j);
    let windup_deg = windup_rad * 180.0 / PI;

    let centrifugal_pa = f_cf / area;
    let total_pa = centrifugal_pa + bending_pa;
    let strength_margin = mat.strength_pa() / total_pa;
    let flap_frac = flap_m / span;

    let adequate =
        flap_frac <= 0.05 && windup_deg <= WINDUP_LIMIT_DEG && strength_margin >= STRENGTH_SF;

    BladeAnalysis {
        material: mat.name,
        flap_deflection_mm: flap_m * 1000.0,
        flap_frac,
        windup_deg,
        centrifugal_mpa: centrifugal_pa / 1e6,
        total_stress_mpa: total_pa / 1e6,
        strength_margin,
        mass_g: mat.density_kg_m3() * area * span * 1000.0,
        adequate,
    }
}

// ---------------------------------------------------------------------------
// Study + recommendation
// ---------------------------------------------------------------------------

/// The recommended material for one part: the lightest adequate one (or `None`).
#[derive(Clone, Debug)]
pub struct PartChoice {
    pub part: &'static str,
    pub chosen: Option<&'static str>,
    pub detail: String,
}

/// The full control-surface material study.
#[derive(Clone, Debug)]
pub struct ControlMaterialReport {
    /// Actuation torque the servo supplies to deflect the surface, N·m (= `M_c`).
    pub actuation_torque_nm: f64,
    /// Per-part material choice + a one-line detail.
    pub parts: Vec<PartChoice>,
    /// Per-material blade analyses (lightest first).
    pub blade: Vec<BladeAnalysis>,
    /// The chosen blade material (lightest adequate).
    pub blade_choice: Option<&'static str>,
}

/// Mechanical power to slew the control surface, W: `P = M_c · ω`, with
/// `ω = (π/3)/t_60` from the servo's 60° transit time.
pub fn actuation_power_w(control_moment_nm: f64, servo_s_per_60: f64) -> f64 {
    if servo_s_per_60 <= 0.0 {
        return 0.0;
    }
    control_moment_nm * (PI / 3.0) / servo_s_per_60
}

/// Run the control-surface material study: for every part, analyse each candidate
/// material and pick the **lightest adequate** one.
pub fn recommend_materials(
    c: &DesignCandidate,
    control_moment_nm: f64,
    materials: &[PrintMaterial],
) -> ControlMaterialReport {
    let mut parts = Vec::new();

    // Blade (centrifugally stiffened + torsion).
    let blade: Vec<BladeAnalysis> = materials
        .iter()
        .map(|m| analyze_blade(c, control_moment_nm, m))
        .collect();
    let blade_choice = blade
        .iter()
        .filter(|a| a.adequate)
        .min_by(|a, b| a.mass_g.total_cmp(&b.mass_g))
        .map(|a| a.material);
    let blade_lead = &blade[0];
    parts.push(PartChoice {
        part: "rotor blade",
        chosen: blade_choice,
        detail: format!(
            "spun-up flap {:.1} mm = {:.1}% (centrifugally stiffened, ≈material-independent), \
             wind-up {:.2}° ({}, limit {:.1}°), σ {:.0} MPa",
            blade_lead.flap_deflection_mm,
            blade_lead.flap_frac * 100.0,
            blade_lead.windup_deg,
            blade_lead.material,
            WINDUP_LIMIT_DEG,
            blade_lead.total_stress_mpa
        ),
    });

    // Non-rotating parts.
    for part in control_parts(c, control_moment_nm) {
        let by_material: Vec<PartAnalysis> = materials.iter().map(|m| analyze(&part, m)).collect();
        let chosen = by_material
            .iter()
            .filter(|a| a.adequate)
            .min_by(|a, b| a.mass_g.total_cmp(&b.mass_g))
            .map(|a| a.material);
        let lead = &by_material[0];
        let detail = match lead.buckling_margin {
            Some(b) => format!("buckling ×{b:.1} ({})", lead.material),
            None => format!(
                "bend {:.1} mm = {:.1}% ({}, limit {:.0}%)",
                lead.deflection_mm,
                lead.deflection_frac * 100.0,
                lead.material,
                part.deflection_limit_frac * 100.0
            ),
        };
        parts.push(PartChoice {
            part: part.name,
            chosen,
            detail,
        });
    }

    ControlMaterialReport {
        actuation_torque_nm: control_moment_nm,
        parts,
        blade,
        blade_choice,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::material::{onyx, onyx_fiberglass, onyx_pro_structural};

    /// Closed-form cantilever deflection matches the independent FEA beam (the
    /// two-routes rule — `helisim_fea` is validated against beam theory itself).
    #[test]
    fn deflection_matches_fea_beam() {
        use helisim_fea::beam::{NodalLoad, uniform_beam};
        let l: f64 = 0.1;
        let b: f64 = 0.004;
        let i = b * b.powi(3) / 12.0;
        let e = onyx().e_pa();
        let p: f64 = 10.0;
        let closed = p * l.powi(3) / (3.0 * e * i);
        let z = i / (b / 2.0);
        let beam = uniform_beam(l, 8, e * i, z);
        let sol = beam
            .solve(
                &[NodalLoad {
                    node: 8,
                    force: p,
                    moment: 0.0,
                }],
                None,
                &[Bc::Clamped(0)],
            )
            .unwrap();
        assert!((sol.max_deflection_m - closed).abs() / closed < 1e-6);
    }

    /// Centrifugal stiffening is the dominant effect: the spun-up flap deflection
    /// is FAR below the naive static cantilever, and it is nearly material-
    /// independent (the taut-string limit) — so the blade material is NOT chosen
    /// on flap stiffness.
    #[test]
    fn blade_flap_is_tension_dominated_and_material_insensitive() {
        let c = DesignCandidate::model();
        let m_c = 0.16;
        let onyx_b = analyze_blade(&c, m_c, &onyx());
        let fg_b = analyze_blade(&c, m_c, &onyx_fiberglass());

        // Static cantilever (no tension) would deflect ~hundreds of mm; spun-up is
        // tens of mm or less.
        let static_onyx = {
            let span = c.radius_m - c.root_cutout * c.radius_m;
            let t = 0.12 * c.chord_m;
            let i = c.chord_m * t.powi(3) / 12.0;
            let lift = c.gross_mass_kg * 9.80665 / c.n_blades as f64;
            lift * span.powi(3) / (8.0 * onyx().e_pa() * i) * 1000.0
        };
        assert!(
            onyx_b.flap_deflection_mm < 0.5 * static_onyx,
            "centrifugal stiffening must help a lot"
        );
        // Tension-dominated ⇒ the 7× modulus gap shrinks to a small flap-deflection gap.
        let ratio = onyx_b.flap_deflection_mm / fg_b.flap_deflection_mm;
        assert!(
            ratio < 3.0,
            "flap is tension-dominated, not modulus-dominated (got {ratio})"
        );
    }

    /// The material-sensitive blade mode is TORSIONAL wind-up: neat Onyx loses too
    /// much commanded pitch to twist, Fiberglass does not — so the blade is chosen
    /// for torsional (control-authority) stiffness, and lands on Fiberglass.
    #[test]
    fn blade_material_driven_by_torsional_windup() {
        let c = DesignCandidate::model();
        let report = recommend_materials(&c, 0.16, &onyx_pro_structural());
        let onyx_b = &report.blade[0];
        let fg_b = &report.blade[1];
        assert!(
            onyx_b.windup_deg > fg_b.windup_deg * 3.0,
            "Onyx twists much more"
        );
        assert!(
            onyx_b.windup_deg > WINDUP_LIMIT_DEG,
            "neat Onyx exceeds the wind-up limit"
        );
        assert!(
            fg_b.windup_deg <= WINDUP_LIMIT_DEG,
            "Fiberglass is within it"
        );
        assert_eq!(report.blade_choice, Some("Onyx+Fiberglass"));
    }

    /// The lightly-loaded pitch link is fine in neat Onyx (buckling margin); the
    /// swashplate is stiffness-governed and steps up to Fiberglass.
    #[test]
    fn link_is_onyx_swashplate_is_fiberglass() {
        let c = DesignCandidate::model();
        let report = recommend_materials(&c, 0.16, &onyx_pro_structural());
        let link = report
            .parts
            .iter()
            .find(|p| p.part == "pitch link/pushrod")
            .unwrap();
        assert_eq!(link.chosen, Some("Onyx"));
        let swash = report
            .parts
            .iter()
            .find(|p| p.part == "swashplate arm")
            .unwrap();
        assert_eq!(swash.chosen, Some("Onyx+Fiberglass"));
    }

    #[test]
    fn actuation_power_positive() {
        let p = actuation_power_w(0.16, 0.06);
        assert!(p > 1.0 && p < 3.0);
    }
}
