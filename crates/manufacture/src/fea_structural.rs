//! FEA-backed structural check — upgrades the single-section `M/Z` estimates to a
//! beam finite-element field solution, and cross-checks the two routes.
//!
//! The beam parts (tail boom, rotor blade in flap) are solved with the
//! [`helisim_fea`] Euler-Bernoulli solver: the boom as a cantilever under the
//! tail-thrust tip load, the blade as a cantilever under its distributed lift.
//! The FE result is reported alongside the closed-form value — agreement between
//! the two *independent* routes (FE vs beam theory) is the validation, and the FE
//! adds the **deflection** (stiffness), which the section-stress check cannot give.
//!
//! Section properties are taken from the real geometry: a thin tube for the boom,
//! and the NACA 0012 flapwise inertia integrated from the section for the blade.

use helisim_design::{DesignCandidate, DesignReport};
use helisim_fea::{Bc, NodalLoad, uniform_beam};
use std::f64::consts::PI;

/// Young's modulus of 6061-T6 aluminium, Pa.
const E_AL: f64 = 69.0e9;
/// Representative blade Young's modulus (glass/carbon laminate), Pa.
const E_BLADE: f64 = 30.0e9;

/// One part's FEA result with its closed-form cross-check.
#[derive(Clone, Debug)]
pub struct FeaPart {
    pub name: &'static str,
    /// Peak deflection from FE, m (the new information).
    pub tip_deflection_m: f64,
    /// Peak deflection WITH centrifugal (geometric) stiffening, m — the real
    /// rotating-blade stiffness. `None` for non-rotating parts.
    pub tip_deflection_stiffened_m: Option<f64>,
    /// Peak bending stress from FE, MPa.
    pub fe_stress_mpa: f64,
    /// Closed-form bending stress, MPa (independent route).
    pub closed_form_stress_mpa: f64,
    /// Whether FE and closed form agree to 2%.
    pub routes_agree: bool,
}

/// The FEA structural report.
#[derive(Clone, Debug)]
pub struct FeaReport {
    pub boom: FeaPart,
    pub blade: FeaPart,
}

/// Tube second moment of area `I = π(D⁴−d⁴)/64`, m⁴.
fn tube_inertia(od: f64, wall: f64) -> f64 {
    let id = od - 2.0 * wall;
    PI * (od.powi(4) - id.powi(4)) / 64.0
}

/// NACA 0012 flapwise second moment of area about the chord line, m⁴. Delegates
/// to the shared [`crate::naca_section::flap_inertia`] so the section geometry has
/// one source of truth (kept under this name for the public API).
pub fn naca0012_flap_inertia(chord: f64) -> f64 {
    crate::naca_section::flap_inertia(chord)
}

/// Run the FEA structural check for a design.
pub fn run_fea(c: &DesignCandidate, report: &DesignReport) -> FeaReport {
    let omega = c.omega();
    let torque = if report.hover_shaft_power_w.is_finite() && omega > 0.0 {
        report.hover_shaft_power_w / omega
    } else {
        1.0
    };

    // --- tail boom: cantilever, tip load = tail thrust ---
    let boom_len = 1.15 * c.radius_m;
    let tail_thrust = torque / boom_len;
    let od = crate::sizing::boom_od_for_bending(torque, crate::materials::SIGMA_ALLOW_AL);
    let wall = 0.1 * od;
    let i_boom = tube_inertia(od, wall);
    let z_boom = i_boom / (od / 2.0);
    let beam = uniform_beam(boom_len, 8, E_AL * i_boom, z_boom);
    let sol = beam
        .solve(
            &[NodalLoad {
                node: 8,
                force: -tail_thrust,
                moment: 0.0,
            }],
            None,
            &[Bc::Clamped(0)],
        )
        .unwrap();
    let cf_boom = tail_thrust * boom_len / z_boom;
    let boom = FeaPart {
        name: "tail boom",
        tip_deflection_m: sol.max_deflection_m,
        tip_deflection_stiffened_m: None,
        fe_stress_mpa: sol.max_stress_pa / 1e6,
        closed_form_stress_mpa: cf_boom / 1e6,
        routes_agree: (sol.max_stress_pa - cf_boom).abs() / cf_boom < 0.02,
    };

    // --- rotor blade: cantilever, distributed lift ---
    let span = c.radius_m - c.root_cutout * c.radius_m;
    let lift_per_blade = c.gross_mass_kg * 9.80665 / c.n_blades as f64;
    let q = lift_per_blade / span; // mean lift per unit span
    let i_blade = naca0012_flap_inertia(c.chord_m);
    let z_blade = i_blade / (0.06 * c.chord_m);
    let n_el = 12;
    let mut bbeam = uniform_beam(span, n_el, E_BLADE * i_blade, z_blade);
    let dist = bbeam.uniform_load_vector(q);
    let bsol = bbeam.solve(&[], Some(&dist), &[Bc::Clamped(0)]).unwrap();
    let cf_blade = (q * span * span / 2.0) / z_blade; // uniform-load root stress

    // Centrifugal (geometric) stiffening: tension at radius r is the pull of all
    // outboard mass, T(r) = ω²·μ·(R²−r²)/2 (μ = blade mass per unit span). Set the
    // per-element tension at each element midpoint and re-solve.
    let root_radius = c.root_cutout * c.radius_m;
    let mu = c.blade_areal_density_kg_m2 * c.chord_m;
    bbeam.tension = (0..n_el)
        .map(|e| {
            let r_mid = root_radius + span * (e as f64 + 0.5) / n_el as f64;
            omega * omega * mu * (c.radius_m * c.radius_m - r_mid * r_mid) / 2.0
        })
        .collect();
    let bsol_stiff = bbeam.solve(&[], Some(&dist), &[Bc::Clamped(0)]).unwrap();

    let blade = FeaPart {
        name: "blade (flap)",
        tip_deflection_m: bsol.max_deflection_m,
        tip_deflection_stiffened_m: Some(bsol_stiff.max_deflection_m),
        fe_stress_mpa: bsol.max_stress_pa / 1e6,
        closed_form_stress_mpa: cf_blade / 1e6,
        routes_agree: (bsol.max_stress_pa - cf_blade).abs() / cf_blade < 0.05,
    };

    FeaReport { boom, blade }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_airfoil::LinearAirfoil;
    use helisim_bemt::Config;
    use helisim_design::evaluate;

    fn cr() -> (DesignCandidate, DesignReport) {
        let c = DesignCandidate::model();
        let r = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
        (c, r)
    }

    #[test]
    fn flap_inertia_scales_as_chord_to_the_fourth() {
        let i1 = naca0012_flap_inertia(0.04);
        let i2 = naca0012_flap_inertia(0.08);
        assert!((i2 / i1 - 16.0).abs() < 1e-6); // 2^4
        assert!(i1 > 0.0);
    }

    #[test]
    fn fe_and_closed_form_routes_agree() {
        let (c, r) = cr();
        let fea = run_fea(&c, &r);
        // The boom (point load) is exact; the blade (uniform load) within 5%.
        assert!(
            fea.boom.routes_agree,
            "boom FE {} vs CF {}",
            fea.boom.fe_stress_mpa, fea.boom.closed_form_stress_mpa
        );
        assert!(
            fea.blade.routes_agree,
            "blade FE {} vs CF {}",
            fea.blade.fe_stress_mpa, fea.blade.closed_form_stress_mpa
        );
    }

    #[test]
    fn fea_reports_a_finite_deflection() {
        let (c, r) = cr();
        let fea = run_fea(&c, &r);
        assert!(fea.boom.tip_deflection_m >= 0.0 && fea.boom.tip_deflection_m.is_finite());
        assert!(fea.blade.tip_deflection_m >= 0.0 && fea.blade.tip_deflection_m.is_finite());
    }

    #[test]
    fn centrifugal_stiffening_makes_the_blade_much_stiffer() {
        // The spinning blade's tension stiffening must dramatically reduce the
        // static (non-rotating) flap deflection — the real rotating-blade behaviour.
        let (c, r) = cr();
        let fea = run_fea(&c, &r);
        let stiff = fea.blade.tip_deflection_stiffened_m.unwrap();
        assert!(stiff < fea.blade.tip_deflection_m);
        assert!(
            stiff < 0.25 * fea.blade.tip_deflection_m,
            "stiffening should be large"
        );
    }
}
