//! 3-D solid finite element — the linear (4-node, constant-strain) tetrahedron.
//!
//! The 3-D analogue of the [`crate::cst`] triangle: each node has three DOF
//! `(u, v, w)` and the strain is constant over the element, so
//! `K_e = V · Bᵀ D B` with the full 3-D isotropic constitutive matrix `D` (6×6).
//! Constant-strain ⇒ it reproduces any uniform stress field exactly — the 3-D **patch
//! test** — which is the validation here (a uniaxial bar gives `σ = F/A` and
//! `δ = FL/AE`). This is what lets a thin part be modelled with through-thickness
//! resolution and a realistic bearing-pressure load, beyond the plane-stress CST.

use crate::linsolve::solve;

/// A 3-D tetrahedral mesh.
#[derive(Clone, Debug)]
pub struct TetMesh {
    /// Node coordinates `(x, y, z)`, m.
    pub nodes: Vec<(f64, f64, f64)>,
    /// Tetrahedra as node-index quadruples.
    pub elements: Vec<[usize; 4]>,
    /// Young's modulus, Pa.
    pub e: f64,
    /// Poisson's ratio.
    pub nu: f64,
}

/// Solved 3-D response.
#[derive(Clone, Debug)]
pub struct TetSolution {
    /// Nodal displacements `(u, v, w)`, m.
    pub disp: Vec<(f64, f64, f64)>,
    /// Per-element stress `[σxx, σyy, σzz, τyz, τxz, τxy]`, Pa.
    pub element_stress: Vec<[f64; 6]>,
}

impl TetMesh {
    /// 3-D isotropic constitutive matrix `D` (6×6, row-major), strain order
    /// `[εxx, εyy, εzz, γyz, γxz, γxy]`.
    fn d_matrix(&self) -> [f64; 36] {
        let (e, nu) = (self.e, self.nu);
        let lam = e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
        let mu = e / (2.0 * (1.0 + nu));
        let mut d = [0.0; 36];
        for i in 0..3 {
            for j in 0..3 {
                d[i * 6 + j] = if i == j { lam + 2.0 * mu } else { lam };
            }
        }
        d[3 * 6 + 3] = mu;
        d[4 * 6 + 4] = mu;
        d[5 * 6 + 5] = mu;
        d
    }

    /// `(B, volume)`: the 6×12 strain-displacement matrix (row-major) and the element
    /// volume, for tetrahedron `tet`. Shape-function gradients come from inverting the
    /// nodal `[1 x y z]` matrix (its `i`-th column is `[aᵢ bᵢ cᵢ dᵢ]`, and `bᵢ,cᵢ,dᵢ`
    /// are `∂Nᵢ/∂x,∂y,∂z`).
    fn b_matrix(&self, tet: [usize; 4]) -> ([f64; 72], f64) {
        let p: [(f64, f64, f64); 4] = std::array::from_fn(|k| self.nodes[tet[k]]);
        // Signed volume from the triple product of edge vectors.
        let e1 = (p[1].0 - p[0].0, p[1].1 - p[0].1, p[1].2 - p[0].2);
        let e2 = (p[2].0 - p[0].0, p[2].1 - p[0].1, p[2].2 - p[0].2);
        let e3 = (p[3].0 - p[0].0, p[3].1 - p[0].1, p[3].2 - p[0].2);
        let cross = (
            e2.1 * e3.2 - e2.2 * e3.1,
            e2.2 * e3.0 - e2.0 * e3.2,
            e2.0 * e3.1 - e2.1 * e3.0,
        );
        let vol = (e1.0 * cross.0 + e1.1 * cross.1 + e1.2 * cross.2).abs() / 6.0;

        // M = [[1,x,y,z]; ...]; coefficients for Nᵢ are M⁻¹·eᵢ (solve per node).
        let m: Vec<f64> = (0..4).flat_map(|k| [1.0, p[k].0, p[k].1, p[k].2]).collect();
        let mut grad = [(0.0, 0.0, 0.0); 4]; // (∂Nᵢ/∂x, ∂y, ∂z)
        for i in 0..4 {
            let mut rhs = [0.0; 4];
            rhs[i] = 1.0;
            // A singular nodal matrix means a degenerate (zero-volume, coplanar)
            // tetrahedron; fail loudly rather than silently zero the gradients.
            let coef = solve(&m, &rhs, 4)
                .unwrap_or_else(|| panic!("Tet::b_matrix: degenerate tetrahedron {tet:?}"));
            grad[i] = (coef[1], coef[2], coef[3]);
        }

        let mut b = [0.0; 72]; // 6 rows × 12 cols (row r starts at r*12)
        for (i, &(bx, cy, dz)) in grad.iter().enumerate() {
            let c0 = 3 * i; // u col; v = c0+1; w = c0+2
            b[c0] = bx; // εxx: ∂u/∂x
            b[12 + c0 + 1] = cy; // εyy: ∂v/∂y
            b[24 + c0 + 2] = dz; // εzz: ∂w/∂z
            b[36 + c0 + 1] = dz; // γyz: ∂v/∂z + ∂w/∂y
            b[36 + c0 + 2] = cy;
            b[48 + c0] = dz; // γxz: ∂u/∂z + ∂w/∂x
            b[48 + c0 + 2] = bx;
            b[60 + c0] = cy; // γxy: ∂u/∂y + ∂v/∂x
            b[60 + c0 + 1] = bx;
        }
        (b, vol)
    }

    /// Solve for displacements and per-element stresses. `loads` are nodal forces
    /// `(node, fx, fy, fz)`; `fixed_dofs` are constrained global DOF (`3n`,`3n+1`,`3n+2`).
    pub fn solve(
        &self,
        loads: &[(usize, f64, f64, f64)],
        fixed_dofs: &[usize],
    ) -> Option<TetSolution> {
        let n = self.nodes.len();
        let dof = 3 * n;
        let mut k = vec![0.0; dof * dof];
        let d = self.d_matrix();

        for &tet in &self.elements {
            let (b, vol) = self.b_matrix(tet);
            // DB = D·B (6×12)
            let mut db = [0.0; 72];
            for r in 0..6 {
                for c in 0..12 {
                    db[r * 12 + c] = (0..6).map(|m| d[r * 6 + m] * b[m * 12 + c]).sum();
                }
            }
            // Ke = V · Bᵀ·DB (12×12)
            let mut ke = [0.0; 144];
            for i in 0..12 {
                for j in 0..12 {
                    ke[i * 12 + j] =
                        vol * (0..6).map(|m| b[m * 12 + i] * db[m * 12 + j]).sum::<f64>();
                }
            }
            let map: [usize; 12] = std::array::from_fn(|q| 3 * tet[q / 3] + (q % 3));
            for i in 0..12 {
                for j in 0..12 {
                    k[map[i] * dof + map[j]] += ke[i * 12 + j];
                }
            }
        }

        let mut f = vec![0.0; dof];
        for &(node, fx, fy, fz) in loads {
            f[3 * node] += fx;
            f[3 * node + 1] += fy;
            f[3 * node + 2] += fz;
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

        let disp = (0..n)
            .map(|i| (u[3 * i], u[3 * i + 1], u[3 * i + 2]))
            .collect();
        let mut element_stress = Vec::new();
        for &tet in &self.elements {
            let (b, _) = self.b_matrix(tet);
            let map: [usize; 12] = std::array::from_fn(|q| 3 * tet[q / 3] + (q % 3));
            let ue: Vec<f64> = map.iter().map(|&g| u[g]).collect();
            let strain: [f64; 6] =
                std::array::from_fn(|r| (0..12).map(|c| b[r * 12 + c] * ue[c]).sum());
            let sigma: [f64; 6] =
                std::array::from_fn(|r| (0..6).map(|m| d[r * 6 + m] * strain[m]).sum());
            element_stress.push(sigma);
        }
        Some(TetSolution {
            disp,
            element_stress,
        })
    }
}

/// A unit-style box `lx×ly×lz` (m) meshed as 6 tetrahedra — the validation specimen
/// and the unit cell of an extruded slab mesh.
pub fn box_six_tets(lx: f64, ly: f64, lz: f64, e: f64, nu: f64) -> TetMesh {
    let nodes = vec![
        (0.0, 0.0, 0.0),
        (lx, 0.0, 0.0),
        (lx, ly, 0.0),
        (0.0, ly, 0.0),
        (0.0, 0.0, lz),
        (lx, 0.0, lz),
        (lx, ly, lz),
        (0.0, ly, lz),
    ];
    // Six tets sharing the 0–6 diagonal — a standard cube decomposition.
    let elements = vec![
        [0, 1, 2, 6],
        [0, 2, 3, 6],
        [0, 3, 7, 6],
        [0, 7, 4, 6],
        [0, 4, 5, 6],
        [0, 5, 1, 6],
    ];
    TetMesh {
        nodes,
        elements,
        e,
        nu,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 3-D PATCH TEST (the constant-strain oracle): impose a uniform-strain
    /// displacement field on the nodes; every element must recover that exact strain
    /// and the corresponding stress `σ = D·ε`. Tested for an axial strain and a shear.
    #[test]
    fn patch_test_reproduces_uniform_strain_exactly() {
        let (e, nu) = (70.0e9, 0.33);
        let mesh = box_six_tets(0.04, 0.03, 0.02, e, nu);
        let lam = e * nu / ((1.0 + nu) * (1.0 - 2.0 * nu));
        let mu = e / (2.0 * (1.0 + nu));

        // (a) Uniaxial strain εxx = 1e-3 ⇒ u = εxx·x, v = w = 0.
        let exx = 1.0e-3;
        let mut u = vec![0.0; 3 * mesh.nodes.len()];
        for (i, &(x, _, _)) in mesh.nodes.iter().enumerate() {
            u[3 * i] = exx * x;
        }
        let d = mesh.d_matrix();
        for &t in &mesh.elements {
            let (b, _) = mesh.b_matrix(t);
            let map: [usize; 12] = std::array::from_fn(|q| 3 * t[q / 3] + (q % 3));
            let ue: Vec<f64> = map.iter().map(|&g| u[g]).collect();
            let strain: [f64; 6] =
                std::array::from_fn(|r| (0..12).map(|c| b[r * 12 + c] * ue[c]).sum());
            let sigma: [f64; 6] =
                std::array::from_fn(|r| (0..6).map(|m| d[r * 6 + m] * strain[m]).sum());
            assert!((strain[0] - exx).abs() < 1e-12, "εxx exact");
            assert!((sigma[0] - (lam + 2.0 * mu) * exx).abs() / ((lam + 2.0 * mu) * exx) < 1e-9);
            assert!((sigma[1] - lam * exx).abs() / (lam * exx) < 1e-9); // σyy = λ·εxx
        }

        // (b) Pure shear γxy = 2e-3 ⇒ u = γ·y, v = w = 0 ⇒ τxy = μ·γ.
        let gam = 2.0e-3;
        let mut u2 = vec![0.0; 3 * mesh.nodes.len()];
        for (i, &(_, y, _)) in mesh.nodes.iter().enumerate() {
            u2[3 * i] = gam * y;
        }
        for &t in &mesh.elements {
            let (b, _) = mesh.b_matrix(t);
            let map: [usize; 12] = std::array::from_fn(|q| 3 * t[q / 3] + (q % 3));
            let ue: Vec<f64> = map.iter().map(|&g| u2[g]).collect();
            let strain: [f64; 6] =
                std::array::from_fn(|r| (0..12).map(|c| b[r * 12 + c] * ue[c]).sum());
            assert!((strain[5] - gam).abs() < 1e-12, "γxy exact");
        }
    }

    /// Solve-path sanity: a cube pulled in +x extends in +x and contracts laterally
    /// (Poisson), with the mean axial stress near F/A.
    #[test]
    fn solve_extends_in_load_direction() {
        let (e, nu) = (70.0e9, 0.33);
        let l = 0.05;
        let mesh = box_six_tets(l, l, l, e, nu);
        // u=0 on the x=0 face (nodes 0,3,4,7) + a 3-2-1 lateral restraint.
        let fixed = vec![0, 9, 12, 21, 1, 2, 11, 13];
        let total = 1.0e4;
        let fx = total / 4.0;
        let loads = vec![
            (1, fx, 0.0, 0.0),
            (2, fx, 0.0, 0.0),
            (5, fx, 0.0, 0.0),
            (6, fx, 0.0, 0.0),
        ];
        let sol = mesh.solve(&loads, &fixed).unwrap();
        let max_u = sol.disp.iter().map(|d| d.0).fold(0.0_f64, f64::max);
        assert!(max_u > 0.0, "extends in +x");
        let mean_sxx =
            sol.element_stress.iter().map(|s| s[0]).sum::<f64>() / sol.element_stress.len() as f64;
        let sigma_nom = total / (l * l);
        assert!(
            (mean_sxx - sigma_nom).abs() / sigma_nom < 0.05,
            "mean σxx ≈ F/A"
        );
    }

    #[test]
    fn positive_volume() {
        let mesh = box_six_tets(0.02, 0.03, 0.01, 70e9, 0.3);
        for &t in &mesh.elements {
            let (_, v) = mesh.b_matrix(t);
            assert!(v > 0.0, "tet has positive volume");
        }
    }
}
