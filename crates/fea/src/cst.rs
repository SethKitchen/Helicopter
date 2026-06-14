//! 2-D plane-stress finite element — the constant-strain triangle (CST).
//!
//! A step beyond the 1-D beam: a true continuum element for the in-plane stress of
//! thin panels (a swashplate plate, a bracket, a rib web). Each triangle has three
//! nodes with two DOF each `(u, v)`; the strain is constant over the element, so
//! the stiffness is `K_e = t·A·Bᵀ D B` with the plane-stress constitutive matrix
//!
//! `D = E/(1−ν²)·[[1, ν, 0], [ν, 1, 0], [0, 0, (1−ν)/2]]`.
//!
//! Being a *constant-strain* element it reproduces any uniform stress field
//! exactly — which is the classic FE **patch test** and the validation here
//! (a uniaxial bar gives `σ = F/A` and `δ = FL/AE` to machine precision). True
//! plate-bending / curved-shell elements are the named next step; this is the
//! membrane (in-plane) shell behaviour.

use crate::linsolve::solve;

/// A plane-stress CST mesh.
#[derive(Clone, Debug)]
pub struct Cst {
    /// Node coordinates `(x, y)`, m.
    pub nodes: Vec<(f64, f64)>,
    /// Triangles as node-index triples.
    pub elements: Vec<[usize; 3]>,
    /// Panel thickness, m.
    pub thickness: f64,
    /// Young's modulus, Pa.
    pub e: f64,
    /// Poisson's ratio.
    pub nu: f64,
}

/// The solved CST response.
#[derive(Clone, Debug)]
pub struct CstSolution {
    /// Nodal displacements `(u, v)`, m.
    pub disp: Vec<(f64, f64)>,
    /// Per-element stress `[σxx, σyy, τxy]`, Pa.
    pub element_stress: Vec<[f64; 3]>,
}

impl Cst {
    /// `(B, area)`: the 3×6 strain-displacement matrix (row-major) and the element
    /// area, for the triangle `tri`.
    fn b_matrix(&self, tri: [usize; 3]) -> ([f64; 18], f64) {
        let [(x1, y1), (x2, y2), (x3, y3)] =
            [self.nodes[tri[0]], self.nodes[tri[1]], self.nodes[tri[2]]];
        let area = 0.5 * ((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1));
        let (b1, b2, b3) = (y2 - y3, y3 - y1, y1 - y2);
        let (c1, c2, c3) = (x3 - x2, x1 - x3, x2 - x1);
        let k = 1.0 / (2.0 * area);
        // rows: [εxx; εyy; γxy], cols: u1,v1,u2,v2,u3,v3
        let b = [
            b1 * k,
            0.0,
            b2 * k,
            0.0,
            b3 * k,
            0.0,
            0.0,
            c1 * k,
            0.0,
            c2 * k,
            0.0,
            c3 * k,
            c1 * k,
            b1 * k,
            c2 * k,
            b2 * k,
            c3 * k,
            b3 * k,
        ];
        (b, area)
    }

    fn d_matrix(&self) -> [f64; 9] {
        let f = self.e / (1.0 - self.nu * self.nu);
        [
            f,
            f * self.nu,
            0.0,
            f * self.nu,
            f,
            0.0,
            0.0,
            0.0,
            f * (1.0 - self.nu) / 2.0,
        ]
    }

    /// Solve for displacements and element stresses. `loads` are nodal forces
    /// `(node, fx, fy)`; `fixed_dofs` are constrained global DOF indices
    /// (`2*node` = u, `2*node+1` = v).
    pub fn solve(&self, loads: &[(usize, f64, f64)], fixed_dofs: &[usize]) -> Option<CstSolution> {
        let n = self.nodes.len();
        let dof = 2 * n;
        let mut k = vec![0.0; dof * dof];
        let d = self.d_matrix();

        for &tri in &self.elements {
            let (b, area) = self.b_matrix(tri);
            // DB = D (3x3) * B (3x6) → 3x6
            let mut db = [0.0; 18];
            for r in 0..3 {
                for c in 0..6 {
                    db[r * 6 + c] = (0..3).map(|m| d[r * 3 + m] * b[m * 6 + c]).sum();
                }
            }
            // Ke = t*A * Bᵀ DB  (6x6)
            let mut ke = [0.0; 36];
            for i in 0..6 {
                for j in 0..6 {
                    ke[i * 6 + j] = self.thickness
                        * area
                        * (0..3).map(|m| b[m * 6 + i] * db[m * 6 + j]).sum::<f64>();
                }
            }
            let map = [
                2 * tri[0],
                2 * tri[0] + 1,
                2 * tri[1],
                2 * tri[1] + 1,
                2 * tri[2],
                2 * tri[2] + 1,
            ];
            for i in 0..6 {
                for j in 0..6 {
                    k[map[i] * dof + map[j]] += ke[i * 6 + j];
                }
            }
        }

        let mut f = vec![0.0; dof];
        for &(node, fx, fy) in loads {
            f[2 * node] += fx;
            f[2 * node + 1] += fy;
        }
        for &g in fixed_dofs {
            for c in 0..dof {
                k[g * dof + c] = 0.0;
                k[c * dof + g] = 0.0;
            }
            k[g * dof + g] = 1.0;
            f[g] = 0.0;
        }
        let u = solve(&k, &f, dof)?;

        let disp = (0..n).map(|i| (u[2 * i], u[2 * i + 1])).collect();
        let mut element_stress = Vec::new();
        for &tri in &self.elements {
            let (b, _) = self.b_matrix(tri);
            let map = [
                2 * tri[0],
                2 * tri[0] + 1,
                2 * tri[1],
                2 * tri[1] + 1,
                2 * tri[2],
                2 * tri[2] + 1,
            ];
            let ue: Vec<f64> = map.iter().map(|&g| u[g]).collect();
            let strain: Vec<f64> = (0..3)
                .map(|r| (0..6).map(|c| b[r * 6 + c] * ue[c]).sum())
                .collect();
            let sigma: [f64; 3] =
                std::array::from_fn(|r| (0..3).map(|m| d[r * 3 + m] * strain[m]).sum());
            element_stress.push(sigma);
        }
        Some(CstSolution {
            disp,
            element_stress,
        })
    }
}

/// A rectangular `L×H` plate (m) meshed as two CSTs — a convenient validation
/// specimen and the unit cell of a panel mesh.
pub fn rectangle_two_tris(l: f64, h: f64, thickness: f64, e: f64, nu: f64) -> Cst {
    Cst {
        nodes: vec![(0.0, 0.0), (l, 0.0), (l, h), (0.0, h)],
        elements: vec![[0, 1, 2], [0, 2, 3]],
        thickness,
        e,
        nu,
    }
}
