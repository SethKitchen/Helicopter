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
use crate::blade::blade_from_design;
use crate::fuselage::fuselage_for;
use crate::mesh::{
    Tri, Vec3, box_tris, cylinder_between, cylinder_z, ellipsoid, fuselage_shell,
    lofted_blade_tris, place, triangular_fin, tris_to_stl,
};
use crate::{
    boom_for, hub_from_blade, landing_gear_for, mast_for_torque, swashplate_for, tail_rotor_for,
};
use helisim_design::{DesignCandidate, DesignReport};
use std::f64::consts::PI;

/// The positioned closed-manifold meshes of every aircraft solid, each named —
/// shared by the STL (flattened), AP203 B-rep assembly (per-solid), UI export,
/// mass properties, and integration checks.
/// Positions: fuselage pod at origin, mast up +z to the hub, `n_blades` blades
/// radiating from the hub, the tail boom reaching aft (−x), with all internal
/// aircraft parts represented at their assembly locations.
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
    let mast = mast_for_torque(torque, hub_z / 1000.0);
    let mast_d = mast.diameter_m * 1000.0;
    let boom = boom_for(torque, c.radius_m, omega);
    let boom_len = boom.length_m * 1000.0;
    let boom_od = boom.tube_od_m * 1000.0;
    let fus = fuselage_for(c.gross_mass_kg, c.radius_m);
    let fl = fus.length_m * 1000.0;
    let fw = fus.width_m * 1000.0;
    let fh = fus.height_m * 1000.0;
    let hub = hub_from_blade(
        c.n_blades,
        blade.chord_m,
        blade.max_thickness_m,
        blade.root_radius_m,
        mast.diameter_m,
    );
    let hub_d = hub.hub_diameter_m * 1000.0;
    let swash = swashplate_for(c.radius_m, mast.diameter_m, c.n_blades);
    let swash_d = swash.outer_diameter_m * 1000.0;
    let gear = landing_gear_for(c.gross_mass_kg, fus.length_m, fus.width_m, 150.0e6);
    let tail_rotor = tail_rotor_for(torque, c.radius_m, c.tip_speed_ms);
    let tr_r = tail_rotor.radius_m * 1000.0;

    let mut parts: Vec<(&'static str, Vec<Tri>)> = Vec::new();
    parts.push(("fuselage", fuselage_shell(fl, fw, fh, 44, 52)));
    parts.push((
        "canopy",
        place(
            &ellipsoid(fl * 0.20, fw * 0.30, fh * 0.24, 14, 28),
            0.0,
            0.0,
            Vec3::new(fl * 0.10, 0.0, fh * 0.22),
        ),
    ));
    parts.push((
        "tail boom fairing",
        place(
            &ellipsoid(fl * 0.16, fw * 0.20, fh * 0.18, 12, 24),
            0.0,
            0.0,
            Vec3::new(-fl * 0.42, 0.0, boom_z_for(hub_z)),
        ),
    ));

    {
        let skid_r = (gear.strut_diameter_m * 1000.0 * 0.5).max(3.0);
        let skid_len = gear.skid_length_m * 1000.0;
        let track = gear.track_m * 1000.0;
        let gear_h = gear.height_m * 1000.0;
        let z_skid = -(fh * 0.5 + gear_h);
        let mut g: Vec<Tri> = Vec::new();
        for sy in [-track * 0.5, track * 0.5] {
            let side = if sy < 0.0 { -1.0 } else { 1.0 };
            g.extend(place(
                &cylinder_z(skid_r, skid_len, 12),
                0.0,
                PI / 2.0,
                Vec3::new(-skid_len * 0.5, sy, z_skid),
            ));
            for sx in [-skid_len * 0.28, skid_len * 0.28] {
                let foot = Vec3::new(sx, sy, z_skid + skid_r * 0.3);
                let hardpoint = Vec3::new(sx * 0.78, side * fw * 0.34, -fh * 0.43);
                g.extend(cylinder_between(foot, hardpoint, skid_r * 0.78, 12));
                g.extend(place(
                    &box_tris(skid_r * 5.0, skid_r * 3.5, skid_r * 1.2),
                    0.0,
                    0.0,
                    Vec3::new(hardpoint.x, hardpoint.y, hardpoint.z),
                ));
            }
            // Cross tube between left/right hardpoints at each station.
        }
        for sx in [-skid_len * 0.28, skid_len * 0.28] {
            g.extend(cylinder_between(
                Vec3::new(sx * 0.78, -fw * 0.34, -fh * 0.43),
                Vec3::new(sx * 0.78, fw * 0.34, -fh * 0.43),
                skid_r * 0.6,
                10,
            ));
        }
        parts.push(("landing_gear", g));
    }

    {
        let tray_z = -fh * 0.22;
        parts.push((
            "powertrain_tray",
            place(
                &box_tris(fl * 0.52, fw * 0.58, fh * 0.045),
                0.0,
                0.0,
                Vec3::new(fl * 0.02, 0.0, tray_z - fh * 0.20),
            ),
        ));
        parts.push((
            "battery",
            place(
                &box_tris(fl * 0.34, fw * 0.5, fh * 0.30),
                0.0,
                0.0,
                Vec3::new(fl * 0.04, 0.0, tray_z),
            ),
        ));
        parts.push((
            "motor",
            place(
                &cylinder_z(mast_d * 1.6, fh * 0.34, 18),
                0.0,
                0.0,
                Vec3::new(0.0, 0.0, tray_z),
            ),
        ));
        parts.push((
            "esc",
            place(
                &box_tris(fl * 0.14, fw * 0.30, fh * 0.10),
                0.0,
                0.0,
                Vec3::new(-fl * 0.18, fw * 0.18, tray_z + fh * 0.10),
            ),
        ));
        parts.push((
            "avionics",
            place(
                &box_tris(fl * 0.16, fw * 0.34, fh * 0.10),
                0.0,
                0.0,
                Vec3::new(fl * 0.22, 0.0, tray_z + fh * 0.14),
            ),
        ));
    }

    parts.push(("mast", cylinder_z(mast_d * 0.5, hub_z, 20)));
    parts.push((
        "swashplate",
        place(
            &cylinder_z(swash_d * 0.5, mast_d * 0.5, 28),
            0.0,
            0.0,
            Vec3::new(0.0, 0.0, hub_z - hub_d * 0.9),
        ),
    ));
    parts.push((
        "hub",
        place(
            &cylinder_z(hub_d * 0.5, blade.max_thickness_m * 1000.0 * 1.6, 24),
            0.0,
            0.0,
            Vec3::new(0.0, 0.0, hub_z),
        ),
    ));
    {
        let mut grips = Vec::new();
        let mut fittings = Vec::new();
        for k in 0..c.n_blades {
            let az = 2.0 * PI * k as f64 / c.n_blades as f64;
            let grip = place(
                &box_tris(hub_d * 0.85, blade.chord_m * 1000.0 * 0.42, hub_d * 0.18),
                az,
                0.0,
                Vec3::new(
                    (root_r_mm + hub_d * 0.22) * az.cos(),
                    (root_r_mm + hub_d * 0.22) * az.sin(),
                    hub_z,
                ),
            );
            grips.extend(grip);
            let root_x = root_r_mm + blade.chord_m * 1000.0 * 0.15;
            let root_pos = Vec3::new(root_x * az.cos(), root_x * az.sin(), hub_z);
            fittings.extend(place(
                &box_tris(
                    blade.chord_m * 1000.0 * 0.72,
                    blade.chord_m * 1000.0 * 0.20,
                    blade.max_thickness_m * 1000.0 * 1.9,
                ),
                az,
                0.0,
                root_pos,
            ));
            fittings.extend(place(
                &cylinder_z(
                    (blade.max_thickness_m * 1000.0 * 0.82).max(3.0),
                    blade.chord_m * 1000.0 * 0.24,
                    18,
                ),
                az + PI / 2.0,
                PI / 2.0,
                root_pos,
            ));
        }
        parts.push(("blade_grips", grips));
        parts.push(("blade_root_fittings", fittings));
    }

    let blade_tris = lofted_blade_tris(&blade, 34, 84);
    for k in 0..c.n_blades {
        let az = 2.0 * PI * k as f64 / c.n_blades as f64;
        let laid = place(
            &blade_tris,
            0.0,
            -PI / 2.0,
            Vec3::new(root_r_mm, 0.0, hub_z),
        );
        parts.push(("blade", place(&laid, az, 0.0, Vec3::new(0.0, 0.0, 0.0))));
    }

    let boom_mesh = cylinder_z(boom_od * 0.5, boom_len, 14);
    let boom_z = hub_z * 0.4;
    let boom_aft = place(&boom_mesh, 0.0, PI / 2.0, Vec3::new(0.0, 0.0, boom_z));
    parts.push((
        "tail boom",
        place(&boom_aft, PI, 0.0, Vec3::new(0.0, 0.0, 0.0)),
    ));

    {
        let x_tail = -boom_len * 0.88;
        parts.push((
            "tail_fin",
            place(
                &triangular_fin(0.22 * r_mm, 0.18 * r_mm, boom_od * 0.42),
                0.0,
                0.0,
                Vec3::new(x_tail, 0.0, boom_z + boom_od * 0.36),
            ),
        ));
        parts.push((
            "horizontal_stab",
            place(
                &box_tris(0.20 * r_mm, 0.38 * r_mm, boom_od * 0.16),
                0.0,
                0.0,
                Vec3::new(x_tail + 0.05 * r_mm, 0.0, boom_z),
            ),
        ));
    }

    {
        let tail_pos = Vec3::new(-boom_len, 0.0, boom_z);
        let hub_disk = cylinder_z(tr_r * 0.16, tr_r * 0.12, 16);
        let axis_x = place(&hub_disk, 0.0, PI / 2.0, Vec3::new(0.0, 0.0, 0.0));
        let mut tail: Vec<Tri> = place(&axis_x, PI / 2.0, 0.0, tail_pos);
        let tb = tail_rotor.blade();
        let tb_tris = lofted_blade_tris(&tb, 16, 52);
        let tb_root = tb.root_radius_m * 1000.0;
        for j in 0..2 {
            let laid = place(&tb_tris, 0.0, -PI / 2.0, Vec3::new(tb_root, 0.0, 0.0));
            let in_plane = place(&laid, 0.0, PI * j as f64, Vec3::new(0.0, 0.0, 0.0));
            tail.extend(place(&in_plane, 0.0, 0.0, tail_pos));
        }
        parts.push(("tail_rotor", tail));
    }
    parts
}

fn boom_z_for(hub_z: f64) -> f64 {
    hub_z * 0.4
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
    let refs = |ids: &[usize]| {
        ids.iter()
            .map(|i| format!("#{i}"))
            .collect::<Vec<_>>()
            .join(",")
    };
    let root_poly = id;
    lines.push(format!(
        "#{id}=POLYLINE('blade_root',({}));",
        refs(&point_ids_root)
    ));
    id += 1;
    let tip_poly = id;
    lines.push(format!(
        "#{id}=POLYLINE('blade_tip',({}));",
        refs(&point_ids_tip)
    ));

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
    fn whole_aircraft_ap203_brep_is_emitted() {
        let (c, r) = cr();
        let step = aircraft_to_step_ap203(&c, &r);
        assert!(step.starts_with("ISO-10303-21;") && step.contains("END-ISO-10303-21;"));
        // One solid per represented aircraft component.
        assert!(step.matches("MANIFOLD_SOLID_BREP").count() >= 14);
        assert!(step.contains("ADVANCED_BREP_SHAPE_REPRESENTATION"));
        // aircraft_parts returns the named positioned meshes.
        assert!(aircraft_parts(&c, &r).iter().any(|(n, _)| *n == "fuselage"));
        for name in [
            "landing_gear",
            "battery",
            "powertrain_tray",
            "motor",
            "esc",
            "avionics",
            "hub",
            "swashplate",
            "blade_root_fittings",
            "tail_fin",
            "horizontal_stab",
            "tail_rotor",
        ] {
            assert!(
                aircraft_parts(&c, &r).iter().any(|(n, _)| *n == name),
                "aircraft STEP/STL assembly should include {name}"
            );
        }
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
