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

use crate::materials::{SIGMA_ALLOW_AL, TAU_ALLOW_AL};
use helisim_design::{DesignCandidate, DesignReport};
use std::f64::consts::PI;

/// NACA 0012 cross-section area coefficient: `A ≈ 0.0822 c²`.
const SECTION_AREA_COEFF: f64 = 0.0822;
/// Fraction of the section area treated as load-bearing (conservative).
const STRUCTURAL_FRACTION: f64 = 0.5;

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
    let ms = if actual_pa > 0.0 { allow_pa / actual_pa - 1.0 } else { f64::INFINITY };
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
    let a_root = STRUCTURAL_FRACTION * SECTION_AREA_COEFF * c.chord_m * c.chord_m;
    let sigma_root = f_cf / a_root;

    // Minimum retention bolt (double shear): F_cf = 2 · (π d²/4) · τ_allow.
    let min_bolt_d = (2.0 * f_cf / (PI * bolt_shear_allow_pa)).sqrt();

    let mut items = vec![margin(
        "blade root",
        format!("centrifugal tension {f_cf:.0} N"),
        sigma_root,
        blade_tensile_allow_pa,
    )];

    // --- mast torsion (at the sized diameter) ---
    if report.hover_shaft_power_w.is_finite() && omega > 0.0 {
        let torque = report.hover_shaft_power_w / omega;
        // Sized diameter (rounded up to mm) — recompute to match the part.
        let d_min = (16.0 * torque / (PI * TAU_ALLOW_AL)).cbrt();
        let d = (d_min * 1000.0).ceil() / 1000.0;
        let tau = 16.0 * torque / (PI * d.powi(3));
        items.push(margin("mast", format!("torsion {torque:.2} N·m"), tau, TAU_ALLOW_AL));

        // --- boom bending (root moment = main torque) ---
        let od_min = (torque / (0.058 * SIGMA_ALLOW_AL)).cbrt();
        let od = (od_min * 1000.0).ceil() / 1000.0;
        let sigma_boom = torque / (0.058 * od.powi(3));
        items.push(margin("tail boom", format!("bending {torque:.2} N·m"), sigma_boom, SIGMA_ALLOW_AL));
    }

    let min_margin = items.iter().map(|i| i.margin_of_safety).fold(f64::INFINITY, f64::min);
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
    fn a_too_weak_blade_material_fails_the_check() {
        let c = DesignCandidate::model();
        let r = report_for(&c);
        // Absurdly weak material → the centrifugal check must fail.
        let s = check_structure(&c, &r, 0.05e6, 200e6);
        assert!(!s.all_pass);
        assert!(s.items.iter().any(|i| i.part == "blade root" && !i.ok));
    }
}
