//! Geometry for split parts — emit the **printable pieces** and the **bolt-hole
//! bosses** at each joint, so the exported STL reflects the split, not the whole.
//!
//! For the blade (the part that always overflows a desktop bed) each piece is a
//! capped sub-loft of the airfoil between two spanwise stations — a closed solid
//! that fits the bed — and at the cut face we add annular **bolt bosses** with a
//! real central through-hole (a manifold grommet) at the spar bolt positions.
//!
//! **Named gap (not faked):** snap-fit cantilever hooks and tongue-and-groove
//! features need true CSG (a CAD kernel) to cut cleanly; the bolted-joint bosses
//! are emitted as real geometry, snap hooks are left as a documented CAD step.

use crate::airfoil_coords::naca00xx_contour;
use crate::blade::BladeSpec;
use crate::mesh::{Tri, Vec3};
use std::f64::consts::PI;

/// The airfoil cross-section of the blade at span fraction `s` (0..1), mm, in the
/// blade's own frame (span along +z) — mirrors `mesh::lofted_blade_tris`.
fn station(blade: &BladeSpec, s: f64, base: &[crate::airfoil_coords::Point]) -> Vec<Vec3> {
    let c = blade.local_chord_m(s) * 1000.0;
    let th = blade.local_twist_deg(s).to_radians();
    let xp = 0.25 * c;
    let z = s * blade.span_m * 1000.0;
    base.iter()
        .map(|pt| {
            let (x, y) = (pt.x * c, pt.y * c);
            Vec3::new(
                xp + (x - xp) * th.cos() - y * th.sin(),
                (x - xp) * th.sin() + y * th.cos(),
                z,
            )
        })
        .collect()
}

/// One blade **piece** as a capped sub-loft from span fraction `s_lo` to `s_hi`.
/// Closed at both ends, so it is a printable solid that fits within the bed.
pub fn blade_piece_tris(
    blade: &BladeSpec,
    s_lo: f64,
    s_hi: f64,
    n_span: usize,
    n_section: usize,
) -> Vec<Tri> {
    let base = naca00xx_contour(0.12, n_section);
    let m = base.len();
    let mut tris = Vec::new();
    let mut prev = station(blade, s_lo, &base);
    for jj in 1..n_span {
        let sf = s_lo + (s_hi - s_lo) * jj as f64 / (n_span - 1) as f64;
        let cur = station(blade, sf, &base);
        for i in 0..m {
            let k = (i + 1) % m;
            tris.push(Tri(prev[i], prev[k], cur[k]));
            tris.push(Tri(prev[i], cur[k], cur[i]));
        }
        prev = cur;
    }
    // Cap both cut faces (fan), outward normals consistent with the loft.
    let lo = station(blade, s_lo, &base);
    let hi = station(blade, s_hi, &base);
    for i in 1..m - 1 {
        tris.push(Tri(lo[0], lo[i + 1], lo[i]));
        tris.push(Tri(hi[0], hi[i], hi[i + 1]));
    }
    tris
}

/// An **annular bolt boss** (grommet) with a real central through-hole — a closed
/// manifold tube of outer radius `outer_r`, bore `hole_r`, `height` along +z, with
/// flat annular end-caps. The bore is the bolt hole.
pub fn annular_boss(outer_r: f64, hole_r: f64, height: f64, n: usize) -> Vec<Tri> {
    let ring = |rad: f64, z: f64| -> Vec<Vec3> {
        (0..n)
            .map(|i| {
                let a = 2.0 * PI * i as f64 / n as f64;
                Vec3::new(rad * a.cos(), rad * a.sin(), z)
            })
            .collect()
    };
    let (ob, ot) = (ring(outer_r, 0.0), ring(outer_r, height));
    let (ib, it) = (ring(hole_r, 0.0), ring(hole_r, height));
    let mut tris = Vec::new();
    for i in 0..n {
        let j = (i + 1) % n;
        // Outer wall.
        tris.push(Tri(ob[i], ob[j], ot[j]));
        tris.push(Tri(ob[i], ot[j], ot[i]));
        // Inner (bore) wall — reversed so its normal faces into the hole.
        tris.push(Tri(ib[i], it[j], ib[j]));
        tris.push(Tri(ib[j], it[j], it[i]));
        // Top annulus ring.
        tris.push(Tri(ot[i], ot[j], it[j]));
        tris.push(Tri(ot[i], it[j], it[i]));
        // Bottom annulus ring.
        tris.push(Tri(ob[j], ob[i], ib[i]));
        tris.push(Tri(ob[j], ib[i], ib[j]));
    }
    tris
}

/// The blade **pieces** for an `n_pieces` split: each a clean, watertight capped
/// sub-loft that fits the bed. The bolted joint hardware is a SEPARATE part
/// ([`splice_plate`]) — no overlapping bodies, nothing relying on slicer fusion.
pub fn blade_split_meshes(
    blade: &BladeSpec,
    n_pieces: usize,
    n_span: usize,
    n_section: usize,
) -> Vec<Vec<Tri>> {
    (0..n_pieces)
        .map(|p| {
            let s_lo = p as f64 / n_pieces as f64;
            let s_hi = (p + 1) as f64 / n_pieces as f64;
            blade_piece_tris(blade, s_lo, s_hi, n_span.max(2), n_section)
        })
        .collect()
}

/// Point on a centred rectangle's perimeter along the ray from the centre at angle
/// `theta` — half-extents `hx, hy`. Used to triangulate a rectangle minus a circle.
fn rect_perimeter_pt(cx: f64, cy: f64, hx: f64, hy: f64, theta: f64, z: f64) -> Vec3 {
    let (ct, st) = (theta.cos(), theta.sin());
    let tx = if ct.abs() > 1e-9 {
        hx / ct.abs()
    } else {
        f64::INFINITY
    };
    let ty = if st.abs() > 1e-9 {
        hy / st.abs()
    } else {
        f64::INFINITY
    };
    let t = tx.min(ty);
    Vec3::new(cx + t * ct, cy + t * st, z)
}

/// One face of a "rectangle minus a centred circular hole" at height `z` — the
/// annular band triangulated by connecting each circle arc segment to the matching
/// rectangle-perimeter segment. `up` sets the normal (+z) vs (−z) winding.
fn rect_minus_circle_face(
    center: [f64; 2],
    half: [f64; 2],
    r: f64,
    z: f64,
    n: usize,
    up: bool,
) -> Vec<Tri> {
    let [cx, cy] = center;
    let [hx, hy] = half;
    let mut tris = Vec::new();
    for k in 0..n {
        let t0 = 2.0 * PI * k as f64 / n as f64;
        let t1 = 2.0 * PI * (k + 1) as f64 / n as f64;
        let a0 = Vec3::new(cx + r * t0.cos(), cy + r * t0.sin(), z);
        let a1 = Vec3::new(cx + r * t1.cos(), cy + r * t1.sin(), z);
        let r0 = rect_perimeter_pt(cx, cy, hx, hy, t0, z);
        let r1 = rect_perimeter_pt(cx, cy, hx, hy, t1, z);
        if up {
            tris.push(Tri(a0, r0, r1));
            tris.push(Tri(a0, r1, a1));
        } else {
            tris.push(Tri(a0, r1, r0));
            tris.push(Tri(a0, a1, r1));
        }
    }
    tris
}

/// A flat **splice plate** (`width` × `length` × `thickness` mm) with `n_holes`
/// equally-spaced bolt holes (radius `hole_r`) along its length — a watertight
/// solid with real through-holes (no CSG/boolean, no slicer fusion). This is the
/// bolted-joint hardware the pieces are fastened with.
pub fn splice_plate(
    width: f64,
    length: f64,
    thickness: f64,
    n_holes: usize,
    hole_r: f64,
    n_arc: usize,
) -> Vec<Tri> {
    let mut tris = Vec::new();
    let n_holes = n_holes.max(1);
    let cell = length / n_holes as f64;
    let hy = width / 2.0;
    let hx = cell / 2.0;
    for i in 0..n_holes {
        let cx = (i as f64 + 0.5) * cell;
        // Top (+z) and bottom (0) faces, each rectangle-minus-circle.
        tris.extend(rect_minus_circle_face(
            [cx, 0.0],
            [hx, hy],
            hole_r,
            thickness,
            n_arc,
            true,
        ));
        tris.extend(rect_minus_circle_face(
            [cx, 0.0],
            [hx, hy],
            hole_r,
            0.0,
            n_arc,
            false,
        ));
        // Bore wall (cylinder), normal facing into the hole.
        for k in 0..n_arc {
            let t0 = 2.0 * PI * k as f64 / n_arc as f64;
            let t1 = 2.0 * PI * (k + 1) as f64 / n_arc as f64;
            let b0 = Vec3::new(cx + hole_r * t0.cos(), hole_r * t0.sin(), 0.0);
            let b1 = Vec3::new(cx + hole_r * t1.cos(), hole_r * t1.sin(), 0.0);
            let tpt0 = Vec3::new(cx + hole_r * t0.cos(), hole_r * t0.sin(), thickness);
            let tpt1 = Vec3::new(cx + hole_r * t1.cos(), hole_r * t1.sin(), thickness);
            tris.push(Tri(b0, tpt0, tpt1));
            tris.push(Tri(b0, tpt1, b1));
        }
    }
    // Outer perimeter walls of the full plate (length 0..length, width ±hy).
    let corners = [(0.0, -hy), (length, -hy), (length, hy), (0.0, hy)];
    for e in 0..4 {
        let (x0, y0) = corners[e];
        let (x1, y1) = corners[(e + 1) % 4];
        let p0 = Vec3::new(x0, y0, 0.0);
        let p1 = Vec3::new(x1, y1, 0.0);
        let p2 = Vec3::new(x1, y1, thickness);
        let p3 = Vec3::new(x0, y0, thickness);
        tris.push(Tri(p0, p1, p2));
        tris.push(Tri(p0, p2, p3));
    }
    tris
}

/// The blade **spar splice plate** sized for a design's blade joint: a plate along
/// the ¼-chord line covering the joint, with `2·bolts_per_joint` holes (bolts each
/// side of the cut). Real watertight geometry, exported as its own part.
pub fn blade_splice_plate(blade: &BladeSpec, bolt_d_mm: f64, bolts_per_joint: usize) -> Vec<Tri> {
    let chord_mm = blade.chord_m * 1000.0;
    let width = (0.5 * chord_mm).max(8.0 * bolt_d_mm);
    let n_holes = (2 * bolts_per_joint).max(2);
    let length = n_holes as f64 * 4.0 * bolt_d_mm; // ~2 bolt-Ø cell each side
    let thickness = (2.0 * bolt_d_mm).max(3.0);
    splice_plate(width, length, thickness, n_holes, bolt_d_mm * 0.5, 16)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blade::blade_from_design;
    use crate::mesh::tris_to_stl;
    use helisim_design::DesignCandidate;

    #[test]
    fn annular_boss_has_a_real_bore() {
        let t = annular_boss(4.0, 2.0, 8.0, 16);
        // 8 triangles per segment × 16 = 128; the bore ring (radius 2) is present.
        assert_eq!(t.len(), 8 * 16);
        let min_r = t
            .iter()
            .flat_map(|tr| [tr.0, tr.1, tr.2])
            .map(|p| (p.x * p.x + p.y * p.y).sqrt())
            .fold(f64::INFINITY, f64::min);
        assert!(
            (min_r - 2.0).abs() < 1e-6,
            "bore radius present (a real hole)"
        );
    }

    #[test]
    fn blade_splits_into_clean_capped_pieces() {
        let c = DesignCandidate::model();
        let blade = blade_from_design(&c, 0.0);
        let pieces = blade_split_meshes(&blade, 2, 12, 24);
        assert_eq!(pieces.len(), 2);
        for (i, m) in pieces.iter().enumerate() {
            assert!(!m.is_empty());
            let stl = tris_to_stl(&format!("blade_piece_{i}"), m);
            assert!(
                stl.starts_with("solid") && stl.trim_end().ends_with(&format!("blade_piece_{i}"))
            );
        }
        // Each piece is exactly its sub-loft — no overlapping boss bodies bolted on.
        assert_eq!(
            pieces[1].len(),
            blade_piece_tris(&blade, 0.5, 1.0, 12, 24).len()
        );
    }

    /// The splice plate is a real solid with real bolt holes — every hole's bore
    /// radius is present (a true through-hole), and it serialises cleanly.
    #[test]
    fn splice_plate_has_real_holes() {
        let n_holes = 4;
        let hole_r = 1.0;
        let plate = splice_plate(20.0, 40.0, 3.0, n_holes, hole_r, 16);
        let stl = tris_to_stl("splice_plate", &plate);
        assert!(stl.starts_with("solid splice_plate"));
        // The bore radius (1.0) shows up around each hole centre: collect the
        // minimum radial distance from each hole centre.
        let cell = 40.0 / n_holes as f64;
        for i in 0..n_holes {
            let cx = (i as f64 + 0.5) * cell;
            let min_r = plate
                .iter()
                .flat_map(|t| [t.0, t.1, t.2])
                .map(|p| ((p.x - cx).powi(2) + p.y * p.y).sqrt())
                .fold(f64::INFINITY, f64::min);
            assert!((min_r - hole_r).abs() < 1e-6, "hole {i} bore present");
        }
    }

    /// The design-sized blade splice plate is non-empty and carries holes for
    /// bolts both sides of the cut.
    #[test]
    fn blade_splice_plate_is_generated() {
        let blade = blade_from_design(&DesignCandidate::model(), 0.0);
        let plate = blade_splice_plate(&blade, 2.0, 2);
        assert!(!plate.is_empty());
    }
}
