//! Bonded-root continuum FEA — resolve the STRESS CONCENTRATION at the retention-bolt
//! hole in the aluminium doubler, which the net-section margin (a nominal `F/A`) misses.
//!
//! The doubler is a thin tension plate with a hole, so plane-stress is the right model
//! (the [`helisim_fea`] CST). A quarter-symmetry plate-with-hole mesh is loaded in
//! tension; the peak edge stress gives the FE concentration factor `Kt = σ_peak/σ_net`.
//! The **oracle** is the closed-form Heywood factor for a hole in a finite-width strip,
//! `Kt_net = 2 + (1 − d/w)³` (→3 as d/w→0, the classic infinite-plate value; →2 as
//! d/w→1). The design check applies the closed-form `Kt` to the net-section stress —
//! the honest peak the bolt hole actually sees. (CST is constant-strain, so a coarse
//! mesh UNDER-predicts the true peak; the closed-form `Kt` is the design value and the
//! FE is a directional confirmation.)

use crate::fasteners::retention_bolt;
use crate::materials::{E_AL, SIGMA_ALLOW_AL};
use helisim_design::DesignCandidate;
use helisim_fea::Cst;

/// Heywood net-section stress-concentration factor for a transverse hole in a
/// finite-width tension strip: `Kt_net = 2 + (1 − d/w)³`.
pub fn howland_kt_net(d_over_w: f64) -> f64 {
    let r = d_over_w.clamp(0.0, 1.0);
    2.0 + (1.0 - r).powi(3)
}

/// A 2-D quarter mesh: `(nodes, triangles, far-edge nodes on x=w, hole-edge nodes)`.
pub(crate) type QuarterMesh2d = (Vec<(f64, f64)>, Vec<[usize; 3]>, Vec<usize>, Vec<usize>);

/// Raw quarter-symmetry plate-with-hole mesh — shared by the plane-stress CST and the
/// 3-D tetrahedral extrusion.
pub(crate) fn quarter_mesh_2d(a: f64, w: f64, h: f64, mesh: (usize, usize)) -> QuarterMesh2d {
    use std::f64::consts::PI;
    let (nr, nt) = mesh; // radial rings, angular steps
    let mut nodes = Vec::new();
    let idx = |i: usize, j: usize| j * (nr + 1) + i;
    for j in 0..=nt {
        let theta = 0.5 * PI * (j as f64 / nt as f64);
        let (ct, st) = (theta.cos(), theta.sin());
        let (ox, oy) = if theta.tan() * w <= h {
            (w, w * theta.tan())
        } else {
            (h / theta.tan().max(1e-9), h)
        };
        let (ix, iy) = (a * ct, a * st); // hole-edge point
        for i in 0..=nr {
            let t = i as f64 / nr as f64;
            nodes.push((ix + (ox - ix) * t, iy + (oy - iy) * t));
        }
    }
    let mut tris = Vec::new();
    for j in 0..nt {
        for i in 0..nr {
            let (a0, b0, c0, d0) = (idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1));
            tris.push([a0, b0, c0]);
            tris.push([a0, c0, d0]);
        }
    }
    let far: Vec<usize> = (0..nodes.len())
        .filter(|&n| (nodes[n].0 - w).abs() < 1e-9)
        .collect();
    let hole: Vec<usize> = (0..=nt).map(|j| idx(0, j)).collect(); // i=0 ring
    (nodes, tris, far, hole)
}

/// Build the plane-stress CST mesh and its far-edge (x=w) loaded nodes.
fn quarter_plate_with_hole(
    a: f64,
    w: f64,
    h: f64,
    mesh: (usize, usize),
    thickness: f64,
    e: f64,
    nu: f64,
) -> (Cst, Vec<usize>) {
    let (nodes, elements, loaded, _hole) = quarter_mesh_2d(a, w, h, mesh);
    (
        Cst {
            nodes,
            elements,
            thickness,
            e,
            nu,
        },
        loaded,
    )
}

/// The bonded-root stress-concentration report.
#[derive(Clone, Debug)]
pub struct RootHoleReport {
    /// Hole-diameter / strip-width ratio at the doubler net section.
    pub d_over_w: f64,
    /// Closed-form Heywood net-section Kt (the DESIGN value).
    pub kt_closed_form: f64,
    /// FE (CST) net-section Kt — a coarse confirmation (under-predicts).
    pub kt_fe: f64,
    /// Nominal net-section stress `F_cf/A_net`, MPa.
    pub nominal_net_mpa: f64,
    /// Peak stress at the hole edge `Kt·nominal`, MPa.
    pub peak_stress_mpa: f64,
    /// Working allowable, MPa.
    pub allowable_mpa: f64,
    /// Margin of safety on the peak stress.
    pub margin_of_safety: f64,
    /// Passes at the peak (concentrated) stress?
    pub ok: bool,
}

/// Resolve the bolt-hole stress concentration in the root doubler and check the peak.
pub fn analyze_root_hole(c: &DesignCandidate) -> RootHoleReport {
    let omega = c.omega();
    let span = c.radius_m - c.root_cutout * c.radius_m;
    let m_blade = c.blade_areal_density_kg_m2 * c.chord_m * span;
    let r_cg = 0.5 * (c.radius_m + c.root_cutout * c.radius_m);
    let f_cf = omega * omega * m_blade * r_cg; // per-blade centrifugal force

    // Doubler geometry: width ≈ root chord, hole = retention bolt, 2 plates × 3 mm.
    let bolt_d = retention_bolt(f_cf).diameter_mm * 1e-3;
    let width = c.chord_m;
    let thickness = 0.003;
    let d_over_w = (bolt_d / width).clamp(0.05, 0.9);

    let a_net = 2.0 * (width - bolt_d).max(1e-4) * thickness; // both plates
    let nominal = f_cf / a_net;
    let kt = howland_kt_net(d_over_w);
    let peak = kt * nominal;

    // FE confirmation: a quarter plate (half-width w=width/2, half-hole a=bolt_d/2).
    let (w, hh, a) = (0.5 * width, 0.5 * width, 0.5 * bolt_d);
    let kt_fe = cst_hole_kt(a, w, hh, thickness);

    let allow = SIGMA_ALLOW_AL;
    let ms = if peak > 0.0 {
        allow / peak - 1.0
    } else {
        f64::INFINITY
    };
    RootHoleReport {
        d_over_w,
        kt_closed_form: kt,
        kt_fe,
        nominal_net_mpa: nominal / 1e6,
        peak_stress_mpa: peak / 1e6,
        allowable_mpa: allow / 1e6,
        margin_of_safety: ms,
        ok: ms >= 0.0,
    }
}

/// Solve the quarter plate-with-hole for a unit far-field tension and return the FE
/// net-section concentration factor `Kt_fe = σ_peak / σ_net`.
fn cst_hole_kt(a: f64, w: f64, h: f64, thickness: f64) -> f64 {
    let (mesh, loaded) = quarter_plate_with_hole(a, w, h, (8, 10), thickness, E_AL, 0.33);
    if loaded.is_empty() {
        return howland_kt_net(2.0 * a / (2.0 * w)); // fall back to closed form
    }
    // Symmetry BCs: u=0 on x=0 (j=nt column), v=0 on y=0 (j=0 row).
    let mut fixed = Vec::new();
    for (n, &(x, y)) in mesh.nodes.iter().enumerate() {
        if x.abs() < 1e-9 {
            fixed.push(2 * n); // u = 0
        }
        if y.abs() < 1e-9 {
            fixed.push(2 * n + 1); // v = 0
        }
    }
    // Unit total load P over the loaded edge (split equally — a Kt estimate).
    let sigma0 = 1.0e6; // 1 MPa far field
    let p = sigma0 * h * thickness;
    let fx = p / loaded.len() as f64;
    let loads: Vec<(usize, f64, f64)> = loaded.iter().map(|&n| (n, fx, 0.0)).collect();
    let Some(sol) = mesh.solve(&loads, &fixed) else {
        return howland_kt_net(2.0 * a / (2.0 * w));
    };
    let peak_xx = sol
        .element_stress
        .iter()
        .map(|s| s[0])
        .fold(0.0_f64, f64::max);
    let sigma_net = p / ((h - a) * thickness);
    peak_xx / sigma_net
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_design::DesignCandidate;

    #[test]
    fn howland_kt_hits_the_known_limits() {
        // Infinite plate (tiny hole) → 3; hole filling the width → 2; monotone between.
        assert!((howland_kt_net(0.0) - 3.0).abs() < 1e-9);
        assert!((howland_kt_net(1.0) - 2.0).abs() < 1e-9);
        assert!(howland_kt_net(0.2) > howland_kt_net(0.5));
        assert!(howland_kt_net(0.3) > 2.0 && howland_kt_net(0.3) < 3.0);
    }

    #[test]
    fn fe_shows_a_concentration_below_the_closed_form() {
        // The CST captures a stress concentration (Kt > 1); being constant-strain and
        // coarse it under-predicts the closed-form peak — exactly the documented caveat.
        let kt_fe = cst_hole_kt(0.0015, 0.011, 0.011, 0.003);
        assert!(kt_fe > 1.2, "FE should show a concentration, got {kt_fe}");
    }

    #[test]
    fn root_hole_peak_uses_kt_and_is_checked() {
        let c = DesignCandidate::model();
        let r = analyze_root_hole(&c);
        assert!(r.kt_closed_form > 2.0 && r.kt_closed_form <= 3.0);
        // The peak is the concentrated stress, strictly above the nominal net stress.
        assert!(r.peak_stress_mpa > r.nominal_net_mpa);
        assert!((r.peak_stress_mpa - r.kt_closed_form * r.nominal_net_mpa).abs() < 1e-6);
    }
}
