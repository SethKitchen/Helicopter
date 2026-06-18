//! Structural proof — turn the first-cut part sizing into a margin-of-safety check
//! under real flight loads.
//!
//! Sizing (mast by torsion, boom by bending) picks a dimension; this module does
//! the converse — computes the *actual* stress under load and the **margin of
//! safety** `MS = σ_allow/σ_actual − 1` (positive = passes). The dominant and
//! easily-overlooked rotor load is **blade centrifugal tension**: every blade is
//! flung outward with
//!
//! `F_cf = ∫ ω² r dm = ω² m_blade r_cg`,  `r_cg = (R + R_root)/2`,
//!
//! which for a fast or heavy rotor is large and sets the blade root and retention
//! bolt. We report `F_cf`, the blade-root tensile margin, the mast torsion margin,
//! the boom bending margin, and the minimum retention-bolt diameter.
//!
//! All allowables already fold in a safety factor (see [`crate::materials`]), so
//! `MS ≥ 0` means the part meets the working stress. This is a first-principles
//! check, not an FEA — it catches the obvious failures (a centrifugally-overloaded
//! root) and gives the magnitudes; detailed parts still want FEA/proof-load.

use crate::blade::{RootLoadPath, root_load_path};
use crate::materials::{
    SIGMA_ALLOW_AL, SIGMA_ALLOW_CFF_GLASS, SIGMA_BEARING_AL, SIGMA_COMPR_CFF_GLASS, TAU_ALLOW_AL,
    TAU_ALLOW_EPOXY,
};
use crate::naca_section::structural_area;
use crate::sizing::{boom_bending_stress, mast_dia_for_torsion, mast_torsion_stress};
use helisim_design::{DesignCandidate, DesignReport};
use std::f64::consts::PI;

/// Steel retention-bushing wall thickness, m (the sleeve OD = bolt Ø + 2·wall).
const BUSHING_WALL_M: f64 = 0.001;

/// One part's load and margin.
#[derive(Clone, Debug)]
pub struct MarginItem {
    /// Part name.
    pub part: &'static str,
    /// Load description.
    pub load: String,
    /// Actual stress, MPa.
    pub actual_mpa: f64,
    /// Allowable (working) stress, MPa.
    pub allowable_mpa: f64,
    /// Margin of safety `allowable/actual − 1`.
    pub margin_of_safety: f64,
    /// Whether it passes (`MS ≥ 0`).
    pub ok: bool,
}

fn margin(part: &'static str, load: String, actual_pa: f64, allow_pa: f64) -> MarginItem {
    let ms = if actual_pa > 0.0 {
        allow_pa / actual_pa - 1.0
    } else {
        f64::INFINITY
    };
    MarginItem {
        part,
        load,
        actual_mpa: actual_pa / 1e6,
        allowable_mpa: allow_pa / 1e6,
        margin_of_safety: ms,
        ok: ms >= 0.0,
    }
}

/// The assembled structural check.
#[derive(Clone, Debug)]
pub struct StructuralReport {
    /// Per-part margins.
    pub items: Vec<MarginItem>,
    /// Blade centrifugal force, N (the headline rotor load).
    pub blade_centrifugal_n: f64,
    /// Minimum blade-retention bolt diameter for the centrifugal load, m.
    pub min_bolt_diameter_m: f64,
    /// The worst (minimum) margin of safety.
    pub min_margin: f64,
    /// Whether every check passes.
    pub all_pass: bool,
}

/// Run the structural check for a design. `blade_tensile_allow_pa` is the blade
/// material working tensile stress (SF included); `bolt_shear_allow_pa` the
/// retention-bolt working shear (SF included).
pub fn check_structure(
    c: &DesignCandidate,
    report: &DesignReport,
    blade_tensile_allow_pa: f64,
    bolt_shear_allow_pa: f64,
) -> StructuralReport {
    let omega = c.omega();
    let root_radius = c.root_cutout * c.radius_m;
    let span = c.radius_m - root_radius;

    // --- blade centrifugal tension ---
    let m_blade = c.blade_areal_density_kg_m2 * c.chord_m * span;
    let r_cg = 0.5 * (c.radius_m + root_radius);
    let f_cf = omega * omega * m_blade * r_cg;
    let a_root = structural_area(c.chord_m);
    let sigma_root = f_cf / a_root;

    // Minimum retention bolt (double shear): F_cf = 2 · (π d²/4) · τ_allow.
    let min_bolt_d = (2.0 * f_cf / (PI * bolt_shear_allow_pa)).sqrt();

    let mut items = vec![margin(
        "blade root",
        format!("centrifugal tension {f_cf:.0} N"),
        sigma_root,
        blade_tensile_allow_pa,
    )];

    // --- AS-BUILT root joint: the load path that actually carries F_cf ---
    // The retention bolt bears on a BONDED steel bushing (never a press-fit — a polymer
    // interference fit stress-relaxes and loosens). How F_cf gets from the blade into
    // the bushing depends on the print route (see [`crate::root_load_path`]):
    let bolt_d = min_bolt_d.max(0.003); // retention bolt (≥ M3)
    match root_load_path(span) {
        // Desktop continuous-fiber route: the fiber tow loops the bushing, carrying F_cf
        // in fiber TENSION (two legs across the root) bearing on the steel bushing. The
        // amount of fiber is NOT assumed — the required area falls out of the load and the
        // sourced strength (A_req = F_cf/σ_allow). Reporting the section-average stress
        // F_cf/a_root against the fiber allowable makes the margin the FEASIBILITY headroom:
        // MS = a_root/A_req − 1, i.e. the loop needs a fraction 1/(MS+1) of the root section
        // to be fiber — buildable as long as that fraction is < 1 (MS ≥ 0).
        RootLoadPath::FiberLoop => {
            items.push(margin(
                "root fiber loop",
                "fiber tension (req. frac. of root section)".to_string(),
                f_cf / a_root,
                SIGMA_ALLOW_CFF_GLASS,
            ));
            // Fiber bearing on the steel bushing — a COMPRESSION on the fiber (datasheet
            // compressive allowable). Bearing area = bushing OD × the root boss length: the
            // root is a local THICKENED boss around the bushing, sized (as in
            // [`crate::root_fitting`]) to at least 1.8·bolt Ø — not the thin running airfoil.
            // The bushing OD is itself SIZED to keep the fiber bearing within the allowable
            // (as the bolt Ø is sized to its shear), floored at bolt Ø + 2·wall and rounded
            // up to the next mm — a thicker steel sleeve simply spreads the contact.
            let boss_length = (0.12 * c.chord_m).max(1.8 * bolt_d);
            let od_for_bearing = f_cf / (SIGMA_COMPR_CFF_GLASS * boss_length);
            let bushing_od =
                ((bolt_d + 2.0 * BUSHING_WALL_M).max(od_for_bearing) * 1000.0).ceil() / 1000.0;
            let a_bear = bushing_od * boss_length;
            items.push(margin(
                "root bushing bearing",
                "fiber on bushing (compression)".to_string(),
                f_cf / a_bear,
                SIGMA_COMPR_CFF_GLASS,
            ));
        }
        // SLS/molded route: bonded aluminium doublers carry F_cf into metal. Three ways
        // the joint can fail: the epoxy bond (shear), the doublers (net-section tension
        // at the bolt hole), and the bolt bearing on the doublers.
        RootLoadPath::BondedDoublers => {
            let doubler_l = 1.5 * c.chord_m; // bond overlap (= root-fitting length)
            let doubler_w = c.chord_m; // ≈ root chord
            let doubler_t = 0.003; // 3 mm (1/8") flat bar, per plate
            // Epoxy bond: F_cf sheared over both doubler footprints.
            let a_bond = 2.0 * doubler_l * doubler_w;
            items.push(margin(
                "root epoxy bond",
                "doubler bond, shear".to_string(),
                f_cf / a_bond,
                TAU_ALLOW_EPOXY,
            ));
            // Doubler net-section tension (two plates, minus the bolt hole).
            let a_net = 2.0 * (doubler_w - bolt_d).max(1e-4) * doubler_t;
            items.push(margin(
                "root doublers",
                "net-section tension".to_string(),
                f_cf / a_net,
                SIGMA_ALLOW_AL,
            ));
            // Bolt bearing on the doublers (two plates).
            let a_bear = 2.0 * bolt_d * doubler_t;
            items.push(margin(
                "root bolt bearing",
                "bolt on doublers".to_string(),
                f_cf / a_bear,
                SIGMA_BEARING_AL,
            ));
        }
    }

    // --- mast torsion (at the sized diameter) ---
    if report.hover_shaft_power_w.is_finite() && omega > 0.0 {
        let torque = report.hover_shaft_power_w / omega;
        // Sized diameter (rounded up to mm) — recompute to match the part.
        let d = mast_dia_for_torsion(torque, TAU_ALLOW_AL);
        let tau = mast_torsion_stress(torque, d);
        items.push(margin(
            "mast",
            format!("torsion {torque:.2} N·m"),
            tau,
            TAU_ALLOW_AL,
        ));

        // --- boom bending at the GOVERNING (stress / stiffness / resonance) OD ---
        let boom_len = 1.15 * c.radius_m;
        let target_hz = crate::sizing::BOOM_TARGET_PER_REV * omega / (2.0 * PI);
        let od = crate::sizing::boom_governing_od(
            torque,
            boom_len,
            crate::materials::E_AL,
            crate::materials::RHO_AL,
            SIGMA_ALLOW_AL,
            0.02,
            target_hz,
        );
        let sigma_boom = boom_bending_stress(torque, od);
        items.push(margin(
            "tail boom",
            format!("bending {torque:.2} N·m"),
            sigma_boom,
            SIGMA_ALLOW_AL,
        ));
    }

    let min_margin = items
        .iter()
        .map(|i| i.margin_of_safety)
        .fold(f64::INFINITY, f64::min);
    let all_pass = items.iter().all(|i| i.ok);
    StructuralReport {
        items,
        blade_centrifugal_n: f_cf,
        min_bolt_diameter_m: min_bolt_d,
        min_margin,
        all_pass,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_airfoil::LinearAirfoil;
    use helisim_bemt::Config;
    use helisim_design::evaluate;

    fn report_for(c: &DesignCandidate) -> DesignReport {
        evaluate(c, &LinearAirfoil::naca0012(), &Config::default())
    }

    #[test]
    fn model_blade_passes_centrifugal_with_margin() {
        let c = DesignCandidate::model();
        let r = report_for(&c);
        // Working allowables (documented): blade tensile 40 MPa — a conservative
        // value for a glass/epoxy laminate (ultimate ~300–400 MPa ÷ ~8 SF for a
        // first cut; CMH-17 / typical wet-layup data); bolt shear 200 MPa
        // (ISO 898-1 class-8.8 ultimate 480 MPa ÷ 2.4).
        let s = check_structure(&c, &r, 40e6, 200e6);
        assert!(s.blade_centrifugal_n > 0.0);
        assert!(s.all_pass, "model should pass: min MS {}", s.min_margin);
        assert!(s.min_bolt_diameter_m > 0.0);
    }

    #[test]
    fn centrifugal_force_grows_with_rpm_squared() {
        let c = DesignCandidate::model();
        let r = report_for(&c);
        let s1 = check_structure(&c, &r, 40e6, 200e6);
        let mut c2 = c;
        c2.tip_speed_ms *= 2.0; // doubles omega → 4× centrifugal force
        let s2 = check_structure(&c2, &report_for(&c2), 40e6, 200e6);
        assert!((s2.blade_centrifugal_n / s1.blade_centrifugal_n - 4.0).abs() < 0.01);
    }

    #[test]
    fn report_includes_the_as_built_root_joint_checks() {
        let c = DesignCandidate::model();
        let r = report_for(&c);
        let s = check_structure(&c, &r, 40e6, 200e6);
        // The as-built metal load path is checked: epoxy bond, doubler net-section,
        // and bolt bearing — not just an idealized bolt.
        for p in ["root epoxy bond", "root doublers", "root bolt bearing"] {
            assert!(
                s.items.iter().any(|i| i.part == p),
                "missing joint check: {p}"
            );
        }
        // For the small model these all pass with margin.
        assert!(
            s.all_pass,
            "model joint should pass, min MS {}",
            s.min_margin
        );
    }

    #[test]
    fn a_small_blade_uses_the_fiber_loop_root_path() {
        // A short-span blade fits a desktop continuous-fiber bed → the root carries
        // F_cf as a fiber LOOP in tension, not bonded doublers.
        let mut c = DesignCandidate::model();
        c.radius_m = 0.25; // span ≈ 0.21 m ≤ 0.32 m desktop bed
        let r = report_for(&c);
        let s = check_structure(&c, &r, 40e6, 200e6);
        assert!(
            s.items.iter().any(|i| i.part == "root fiber loop"),
            "small blade should use the fiber-loop root"
        );
        assert!(s.items.iter().any(|i| i.part == "root bushing bearing"));
        // It must NOT report the bonded-doubler checks (different route).
        assert!(!s.items.iter().any(|i| i.part == "root doublers"));
        assert!(
            s.all_pass,
            "fiber-loop root should pass, min MS {}",
            s.min_margin
        );
    }

    #[test]
    fn a_too_weak_blade_material_fails_the_check() {
        let c = DesignCandidate::model();
        let r = report_for(&c);
        // Absurdly weak material → the centrifugal check must fail.
        let s = check_structure(&c, &r, 0.05e6, 200e6);
        assert!(!s.all_pass);
        assert!(s.items.iter().any(|i| i.part == "blade root" && !i.ok));
    }
}
