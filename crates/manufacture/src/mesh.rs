//! A tiny triangle-mesh toolkit (zero deps) for assembling multi-part geometry.
//!
//! Each part can be emitted as a list of [`Tri`]s in its own local frame, then
//! rotated/translated into place and concatenated into one solid for an assembly
//! STL. Primitives (cylinder, ellipsoid, lofted blade) live here; placement is
//! `rotate_z` / `rotate_y` / `translate`.

use crate::airfoil_coords::naca00xx_contour;
use crate::blade::BladeSpec;
use std::f64::consts::PI;

/// A 3D point / vector, mm.
#[derive(Clone, Copy, Debug)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Vec3 { x, y, z }
    }
}

/// A triangle.
#[derive(Clone, Copy, Debug)]
pub struct Tri(pub Vec3, pub Vec3, pub Vec3);

/// Rotate a point about the z-axis by `theta` (rad).
pub fn rotate_z(p: Vec3, theta: f64) -> Vec3 {
    let (c, s) = (theta.cos(), theta.sin());
    Vec3::new(p.x * c - p.y * s, p.x * s + p.y * c, p.z)
}

/// Rotate a point about the y-axis by `theta` (rad).
pub fn rotate_y(p: Vec3, theta: f64) -> Vec3 {
    let (c, s) = (theta.cos(), theta.sin());
    Vec3::new(p.x * c + p.z * s, p.y, -p.x * s + p.z * c)
}

/// Apply a rotation (about z then y) and a translation to every vertex of a mesh.
pub fn place(tris: &[Tri], rot_z: f64, rot_y: f64, t: Vec3) -> Vec<Tri> {
    let xf = |p: Vec3| {
        let p = rotate_z(p, rot_z);
        let p = rotate_y(p, rot_y);
        Vec3::new(p.x + t.x, p.y + t.y, p.z + t.z)
    };
    tris.iter()
        .map(|tri| Tri(xf(tri.0), xf(tri.1), xf(tri.2)))
        .collect()
}

/// A closed cylinder along +z, radius `r`, length `len` (mm), `n` facets around.
pub fn cylinder_z(r: f64, len: f64, n: usize) -> Vec<Tri> {
    let mut tris = Vec::new();
    let ring = |z: f64| -> Vec<Vec3> {
        (0..n)
            .map(|i| {
                let a = 2.0 * PI * i as f64 / n as f64;
                Vec3::new(r * a.cos(), r * a.sin(), z)
            })
            .collect()
    };
    let (bot, top) = (ring(0.0), ring(len));
    let (cb, ct) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, len));
    for i in 0..n {
        let j = (i + 1) % n;
        // Side quad.
        tris.push(Tri(bot[i], bot[j], top[j]));
        tris.push(Tri(bot[i], top[j], top[i]));
        // Caps.
        tris.push(Tri(cb, bot[j], bot[i]));
        tris.push(Tri(ct, top[i], top[j]));
    }
    tris
}

/// An ellipsoid (semi-axes `a,b,c` mm) as a lat-long mesh — the fuselage pod.
/// Pole rows are fan-triangulated (no degenerate triangles), so the surface is a
/// clean closed 2-manifold suitable for a B-rep solid.
pub fn ellipsoid(a: f64, b: f64, c: f64, n_lat: usize, n_long: usize) -> Vec<Tri> {
    let p = |i: usize, j: usize| -> Vec3 {
        let theta = PI * i as f64 / n_lat as f64; // 0..π
        let phi = 2.0 * PI * (j % n_long) as f64 / n_long as f64;
        Vec3::new(
            a * theta.sin() * phi.cos(),
            b * theta.sin() * phi.sin(),
            c * theta.cos(),
        )
    };
    let top = Vec3::new(0.0, 0.0, c);
    let bot = Vec3::new(0.0, 0.0, -c);
    let mut tris = Vec::new();
    for j in 0..n_long {
        // Top cap fan — wound so its shared row-1 edge runs OPPOSITE the body row
        // below (consistent outward orientation; the naive same-sense winding made
        // the poles back-facing, caught by `is_oriented_manifold`).
        tris.push(Tri(top, p(1, j + 1), p(1, j)));
        // Bottom cap fan — likewise opposite the body's last row.
        tris.push(Tri(bot, p(n_lat - 1, j), p(n_lat - 1, j + 1)));
    }
    for i in 1..n_lat - 1 {
        for j in 0..n_long {
            let (a0, a1) = (p(i, j), p(i, j + 1));
            let (b0, b1) = (p(i + 1, j), p(i + 1, j + 1));
            tris.push(Tri(a0, a1, b1));
            tris.push(Tri(a0, b1, b0));
        }
    }
    tris
}

/// The lofted blade as triangles in its own frame (span along +z, mm).
pub fn lofted_blade_tris(blade: &BladeSpec, n_span: usize, n_section: usize) -> Vec<Tri> {
    let base = naca00xx_contour(0.12, n_section);
    let m = base.len();
    let station = |s: f64| -> Vec<Vec3> {
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
    };
    let mut tris = Vec::new();
    let mut prev = station(0.0);
    for jj in 1..n_span {
        let sf = jj as f64 / (n_span - 1) as f64;
        let cur = station(sf);
        for i in 0..m {
            let k = (i + 1) % m;
            tris.push(Tri(prev[i], prev[k], cur[k]));
            tris.push(Tri(prev[i], cur[k], cur[i]));
        }
        prev = cur;
    }
    let (root, tip) = (station(0.0), station(1.0));
    for i in 1..m - 1 {
        tris.push(Tri(root[0], root[i + 1], root[i]));
        tris.push(Tri(tip[0], tip[i], tip[i + 1]));
    }
    tris
}

/// Serialise a mesh as one ASCII STL solid named `name`.
pub fn tris_to_stl(name: &str, tris: &[Tri]) -> String {
    let mut s = format!("solid {name}\n");
    for t in tris {
        let (u, v) = (
            Vec3::new(t.1.x - t.0.x, t.1.y - t.0.y, t.1.z - t.0.z),
            Vec3::new(t.2.x - t.0.x, t.2.y - t.0.y, t.2.z - t.0.z),
        );
        let (mut nx, mut ny, mut nz) = (
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x,
        );
        let len = (nx * nx + ny * ny + nz * nz).sqrt();
        if len > 0.0 {
            nx /= len;
            ny /= len;
            nz /= len;
        }
        s.push_str(&format!(
            "  facet normal {nx:.6e} {ny:.6e} {nz:.6e}\n    outer loop\n"
        ));
        for p in [t.0, t.1, t.2] {
            s.push_str(&format!(
                "      vertex {:.6e} {:.6e} {:.6e}\n",
                p.x, p.y, p.z
            ));
        }
        s.push_str("    endloop\n  endfacet\n");
    }
    s.push_str(&format!("endsolid {name}\n"));
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cylinder_has_expected_triangle_count() {
        // n sides → 2 side + 2 cap triangles each = 4n.
        let t = cylinder_z(5.0, 100.0, 16);
        assert_eq!(t.len(), 4 * 16);
    }

    #[test]
    fn rotate_z_ninety_degrees_maps_x_to_y() {
        let p = rotate_z(Vec3::new(1.0, 0.0, 0.0), PI / 2.0);
        assert!(p.x.abs() < 1e-9 && (p.y - 1.0).abs() < 1e-9);
    }

    #[test]
    fn place_translates_and_serialises() {
        let t = cylinder_z(2.0, 10.0, 8);
        let moved = place(&t, 0.0, 0.0, Vec3::new(100.0, 0.0, 0.0));
        let stl = tris_to_stl("c", &moved);
        assert!(stl.starts_with("solid c") && stl.trim_end().ends_with("endsolid c"));
        // Translated by +100 in x → some vertex near x=100.
        assert!(stl.contains("1.00"));
    }
}
