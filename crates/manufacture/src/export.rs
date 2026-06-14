//! Geometry-file export — turn a part into a file you can print or cut.
//!
//! * **STL** (ASCII) of the blade as a constant-section extruded solid: ready for
//!   a slicer / 3D printer or a CAM package.
//! * **DXF** (ASCII) of the airfoil section as a closed polyline: ready for a
//!   laser/water-jet/CNC profile cut, or to loft in CAD.
//!
//! Both are written by hand (zero dependencies) and the tests check they are
//! well-formed (correct facet/vertex counts, valid headers/footers) — the same
//! "validate the structure, not a fabricated number" discipline.

use crate::airfoil_coords::{naca00xx_contour, Point};
use crate::blade::BladeSpec;

const THICKNESS_FRAC: f64 = 0.12;

/// 3D vertex, mm.
#[derive(Clone, Copy)]
struct V3 {
    x: f64,
    y: f64,
    z: f64,
}

fn facet(out: &mut String, a: V3, b: V3, c: V3) {
    // Per-facet normal from the triangle (unit; falls back to 0 if degenerate).
    let (ux, uy, uz) = (b.x - a.x, b.y - a.y, b.z - a.z);
    let (vx, vy, vz) = (c.x - a.x, c.y - a.y, c.z - a.z);
    let (mut nx, mut ny, mut nz) = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx);
    let len = (nx * nx + ny * ny + nz * nz).sqrt();
    if len > 0.0 {
        nx /= len;
        ny /= len;
        nz /= len;
    }
    out.push_str(&format!("  facet normal {nx:.6e} {ny:.6e} {nz:.6e}\n"));
    out.push_str("    outer loop\n");
    for v in [a, b, c] {
        out.push_str(&format!("      vertex {:.6e} {:.6e} {:.6e}\n", v.x, v.y, v.z));
    }
    out.push_str("    endloop\n  endfacet\n");
}

/// ASCII STL of a blade as a constant NACA section extruded over the span.
/// `n` points per airfoil surface. Coordinates in mm: x chordwise, y thickness,
/// z spanwise (0 = root .. span = tip).
pub fn blade_to_stl(blade: &BladeSpec, n: usize) -> String {
    let contour = blade.section_contour_mm(n); // mm, closed loop (TE→LE→TE)
    let m = contour.len();
    let z0 = 0.0;
    let z1 = blade.span_m * 1000.0;
    let at = |p: &Point, z: f64| V3 { x: p.x, y: p.y, z };

    let mut s = String::from("solid blade\n");

    // Side walls: each edge i→(i+1)%m becomes a quad (two facets).
    for i in 0..m {
        let a = &contour[i];
        let b = &contour[(i + 1) % m];
        facet(&mut s, at(a, z0), at(b, z0), at(b, z1));
        facet(&mut s, at(a, z0), at(b, z1), at(a, z1));
    }
    // End caps: fan-triangulate the (convex) section at each end.
    for i in 1..m - 1 {
        // Root cap (normal toward −z): order for outward normal.
        facet(&mut s, at(&contour[0], z0), at(&contour[i + 1], z0), at(&contour[i], z0));
        // Tip cap (normal toward +z).
        facet(&mut s, at(&contour[0], z1), at(&contour[i], z1), at(&contour[i + 1], z1));
    }

    s.push_str("endsolid blade\n");
    s
}

/// Number of facets `blade_to_stl` will write for `m` contour points:
/// `2m` wall + `2(m−2)` cap = `4m − 4`.
pub fn stl_facet_count(m: usize) -> usize {
    4 * m - 4
}

/// The 3D points of the blade section at span fraction `s ∈ [0,1]`, in mm —
/// scaled by the local chord, rotated by the local twist about the quarter-chord,
/// and placed at the spanwise height. Shared by the lofted STL and the assembly.
fn station_points(blade: &BladeSpec, base: &[Point], s: f64) -> Vec<V3> {
    let c = blade.local_chord_m(s) * 1000.0;
    let th = blade.local_twist_deg(s).to_radians();
    let xp = 0.25 * c; // pitch axis at quarter chord
    let z = s * blade.span_m * 1000.0;
    base.iter()
        .map(|p| {
            let (x, y) = (p.x * c, p.y * c);
            V3 {
                x: xp + (x - xp) * th.cos() - y * th.sin(),
                y: (x - xp) * th.sin() + y * th.cos(),
                z,
            }
        })
        .collect()
}

/// ASCII STL of a **lofted** blade: chord and twist interpolated over `n_span`
/// spanwise stations (a true tapered/twisted solid, not a constant extrusion).
/// `n_section` points per airfoil surface.
pub fn lofted_blade_to_stl(blade: &BladeSpec, n_span: usize, n_section: usize) -> String {
    let base = naca00xx_contour(THICKNESS_FRAC, n_section);
    let m = base.len();
    let mut s = String::from("solid blade_lofted\n");

    let mut prev = station_points(blade, &base, 0.0);
    for j in 1..n_span {
        let sf = j as f64 / (n_span - 1) as f64;
        let cur = station_points(blade, &base, sf);
        for i in 0..m {
            let (a, b) = (prev[i], prev[(i + 1) % m]);
            let (c, d) = (cur[(i + 1) % m], cur[i]);
            facet(&mut s, a, b, c);
            facet(&mut s, a, c, d);
        }
        prev = cur;
    }
    // End caps (fan-triangulate the convex section).
    let root = station_points(blade, &base, 0.0);
    let tip = station_points(blade, &base, 1.0);
    for i in 1..m - 1 {
        facet(&mut s, root[0], root[i + 1], root[i]);
        facet(&mut s, tip[0], tip[i], tip[i + 1]);
    }
    s.push_str("endsolid blade_lofted\n");
    s
}

/// Facet count of `lofted_blade_to_stl`: `2m(n_span−1)` walls + `2(m−2)` caps.
pub fn lofted_facet_count(n_span: usize, m: usize) -> usize {
    2 * m * (n_span - 1) + 2 * (m - 2)
}

/// ASCII DXF of an airfoil section as a closed `LWPOLYLINE`. Points in mm.
pub fn airfoil_to_dxf(contour: &[Point]) -> String {
    let mut s = String::new();
    s.push_str("0\nSECTION\n2\nENTITIES\n");
    s.push_str("0\nLWPOLYLINE\n8\n0\n");
    s.push_str(&format!("90\n{}\n70\n1\n", contour.len())); // vertex count, closed
    for p in contour {
        s.push_str(&format!("10\n{:.6}\n20\n{:.6}\n", p.x, p.y));
    }
    s.push_str("0\nENDSEC\n0\nEOF\n");
    s
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blade::blade_from_design;
    use helisim_design::DesignCandidate;

    fn blade() -> BladeSpec {
        blade_from_design(&DesignCandidate::model(), 0.0)
    }

    #[test]
    fn stl_is_well_formed_with_expected_facet_count() {
        let b = blade();
        let n = 60;
        let stl = blade_to_stl(&b, n);
        let m = b.section_contour_mm(n).len();
        assert!(stl.starts_with("solid blade"));
        assert!(stl.trim_end().ends_with("endsolid blade"));
        let facets = stl.matches("facet normal").count();
        assert_eq!(facets, stl_facet_count(m));
        // Each facet has exactly three vertices.
        assert_eq!(stl.matches("vertex ").count(), 3 * facets);
    }

    #[test]
    fn dxf_has_a_closed_polyline_of_the_section() {
        let b = blade();
        let contour = b.section_contour_mm(50);
        let dxf = airfoil_to_dxf(&contour);
        assert!(dxf.contains("LWPOLYLINE"));
        assert!(dxf.contains("\n70\n1\n")); // closed flag
        assert!(dxf.trim_end().ends_with("EOF"));
        // Vertex count code 90 matches the contour length.
        assert!(dxf.contains(&format!("90\n{}\n", contour.len())));
    }

    #[test]
    fn lofted_stl_is_well_formed_for_a_tapered_twisted_blade() {
        use crate::blade::blade_from_design_tapered;
        let b = blade_from_design_tapered(&DesignCandidate::model(), 8.0, 0.6);
        assert!(b.is_lofted());
        let (n_span, n_sec) = (12, 50);
        let stl = lofted_blade_to_stl(&b, n_span, n_sec);
        let m = naca00xx_contour(0.12, n_sec).len();
        assert!(stl.starts_with("solid blade_lofted"));
        assert!(stl.trim_end().ends_with("endsolid blade_lofted"));
        let facets = stl.matches("facet normal").count();
        assert_eq!(facets, lofted_facet_count(n_span, m));
        // The tip section is narrower than the root (taper) — max |x| shrinks.
        let root_pts = station_points(&b, &naca00xx_contour(0.12, n_sec), 0.0);
        let tip_pts = station_points(&b, &naca00xx_contour(0.12, n_sec), 1.0);
        let root_chord = root_pts.iter().map(|p| p.x).fold(f64::MIN, f64::max);
        let tip_chord = tip_pts.iter().map(|p| p.x).fold(f64::MIN, f64::max);
        assert!(tip_chord < root_chord);
    }

    #[test]
    fn stl_vertices_span_the_blade_length() {
        let b = blade();
        let stl = blade_to_stl(&b, 40);
        // The maximum z coordinate should reach the span (mm).
        let span_mm = b.span_m * 1000.0;
        assert!(stl.contains(&format!("{:.6e}", span_mm)));
    }
}
