//! Whole-aircraft geometry export: position every solid into one STL, and write a
//! valid STEP wireframe of the key section curves.
//!
//! The single-part exports ([`crate::export`]) make each component; this places
//! them — fuselage at the origin, mast up the z-axis, blades radiating from the
//! hub, boom reaching aft — into one assembly STL you can view or print as a unit.
//! STEP is emitted as a **wireframe** (section polylines): a valid ISO-10303-21
//! file that opens in CAD, but geometry-as-curves, not a solid B-rep (the full
//! B-rep STEP is a named, larger effort).

use crate::airfoil_coords::naca00xx_contour;
use crate::blade::{blade_from_design, BladeSpec};
use crate::fuselage::fuselage_for;
use crate::mesh::{cylinder_z, ellipsoid, lofted_blade_tris, place, tris_to_stl, Tri, Vec3};
use helisim_design::{DesignCandidate, DesignReport};
use std::f64::consts::PI;

/// The positioned closed-manifold meshes of the aircraft's main solids, each named
/// — shared by the STL (flattened) and the AP203 B-rep assembly (per-solid).
/// Positions: fuselage pod at origin, mast up +z to the hub, `n_blades` blades
/// radiating from the hub, the tail boom reaching aft (−x).
pub fn aircraft_parts(c: &DesignCandidate, report: &DesignReport) -> Vec<(&'static str, Vec<Tri>)> {
    let blade = blade_from_design(c, 0.0);
    let r_mm = c.radius_m * 1000.0;
    let root_r_mm = blade.root_radius_m * 1000.0;
    let hub_z = (0.20 * c.radius_m + 0.05) * 1000.0;

    let omega = c.omega();
    let torque = if report.hover_shaft_power_w.is_finite() && omega > 0.0 {
        report.hover_shaft_power_w / omega
    } else {
        1.0
    };
    let mast_d = ((16.0 * torque / (PI * 55.0e6)).cbrt() * 1000.0).ceil();
    let boom_len = 1.15 * r_mm;
    let boom_od = ((torque / (0.058 * 90.0e6)).cbrt() * 1000.0).ceil();
    let fus = fuselage_for(c.gross_mass_kg, c.radius_m);

    let mut parts: Vec<(&'static str, Vec<Tri>)> = Vec::new();
    parts.push(("fuselage", ellipsoid(fus.length_m * 500.0, fus.width_m * 500.0, fus.height_m * 500.0, 10, 16)));
    parts.push(("mast", cylinder_z(mast_d * 0.5, hub_z, 16)));

    let blade_tris = lofted_blade_tris(&blade, 12, 40);
    for k in 0..c.n_blades {
        let az = 2.0 * PI * k as f64 / c.n_blades as f64;
        let laid = place(&blade_tris, 0.0, -PI / 2.0, Vec3::new(root_r_mm, 0.0, hub_z));
        parts.push(("blade", place(&laid, az, 0.0, Vec3::new(0.0, 0.0, 0.0))));
    }

    let boom = cylinder_z(boom_od * 0.5, boom_len, 12);
    let boom_aft = place(&boom, 0.0, PI / 2.0, Vec3::new(0.0, 0.0, hub_z * 0.4));
    parts.push(("tail boom", place(&boom_aft, PI, 0.0, Vec3::new(0.0, 0.0, 0.0))));
    parts
}

/// Build the full-aircraft assembly as one ASCII STL (mm).
pub fn aircraft_to_stl(c: &DesignCandidate, report: &DesignReport) -> String {
    let mut all: Vec<Tri> = Vec::new();
    for (_, tris) in aircraft_parts(c, report) {
        all.extend(tris);
    }
    tris_to_stl("aircraft", &all)
}

/// The whole-aircraft **B-rep assembly** as a full AP203 STEP file — every main
/// solid a `MANIFOLD_SOLID_BREP`, positioned, under one product/representation.
pub fn aircraft_to_step_ap203(c: &DesignCandidate, report: &DesignReport) -> String {
    crate::step_brep::assembly_to_step_ap203("aircraft", &aircraft_parts(c, report))
}

/// A valid ISO-10303-21 (STEP) **wireframe** of a blade's root and tip section
/// curves, as `POLYLINE`s through `CARTESIAN_POINT`s. Opens in CAD as curves; not
/// a solid B-rep (named limitation).
pub fn aircraft_to_step(c: &DesignCandidate) -> String {
    let blade = blade_from_design(c, 0.0);
    let base = naca00xx_contour(0.12, 40);

    let mut lines = Vec::new();
    let mut id = 1usize;
    let mut point_ids_root = Vec::new();
    let mut point_ids_tip = Vec::new();

    let chord_root = blade.chord_m * 1000.0;
    let chord_tip = blade.tip_chord_m * 1000.0;
    let z_tip = blade.span_m * 1000.0;

    for p in &base {
        lines.push(format!(
            "#{id}=CARTESIAN_POINT('',({:.4},{:.4},0.));",
            p.x * chord_root,
            p.y * chord_root
        ));
        point_ids_root.push(id);
        id += 1;
    }
    for p in &base {
        lines.push(format!(
            "#{id}=CARTESIAN_POINT('',({:.4},{:.4},{:.4}));",
            p.x * chord_tip,
            p.y * chord_tip,
            z_tip
        ));
        point_ids_tip.push(id);
        id += 1;
    }
    let refs = |ids: &[usize]| ids.iter().map(|i| format!("#{i}")).collect::<Vec<_>>().join(",");
    let root_poly = id;
    lines.push(format!("#{id}=POLYLINE('blade_root',({}));", refs(&point_ids_root)));
    id += 1;
    let tip_poly = id;
    lines.push(format!("#{id}=POLYLINE('blade_tip',({}));", refs(&point_ids_tip)));

    let body = lines.join("\n");
    format!(
        "ISO-10303-21;\n\
         HEADER;\n\
         FILE_DESCRIPTION(('helisim blade wireframe'),'2;1');\n\
         FILE_NAME('blade.step','',(''),(''),'helisim','','');\n\
         FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\n\
         ENDSEC;\n\
         DATA;\n\
         {body}\n\
         ENDSEC;\n\
         END-ISO-10303-21;\n\
         /* entities: root polyline #{root_poly}, tip polyline #{tip_poly} */\n"
    )
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
    fn assembly_stl_is_well_formed_and_nontrivial() {
        let (c, r) = cr();
        let stl = aircraft_to_stl(&c, &r);
        assert!(stl.starts_with("solid aircraft"));
        assert!(stl.trim_end().ends_with("endsolid aircraft"));
        // Many facets (fuselage + mast + blades + boom).
        assert!(stl.matches("facet normal").count() > 200);
    }

    #[test]
    fn step_is_valid_iso_10303_with_both_section_curves() {
        let (c, _) = cr();
        let step = aircraft_to_step(&c);
        assert!(step.starts_with("ISO-10303-21;"));
        assert!(step.contains("END-ISO-10303-21;"));
        assert!(step.contains("HEADER;") && step.contains("DATA;") && step.contains("ENDSEC;"));
        assert_eq!(step.matches("POLYLINE(").count(), 2);
        assert!(step.matches("CARTESIAN_POINT(").count() >= 2);
    }
}
