//! Euler-Bernoulli beam finite-element model — deflection and stress of a loaded
//! beam (the rotor blade in flap, the tail boom as a cantilever, the mast).
//!
//! Two-node cubic beam elements, two DOF per node (transverse deflection `w`,
//! rotation `θ`). The element stiffness is the textbook
//!
//! `K_e = (EI/L³) · [[12, 6L, −12, 6L], [6L, 4L², −6L, 2L²],
//!                   [−12, −6L, 12, −6L], [6L, 2L², −6L, 4L²]]`.
//!
//! Cubic elements are *exact* for point loads (the FE solution equals beam theory
//! to machine precision), which is the validation anchor; distributed loads
//! converge with refinement. The model assembles `K`, applies nodal/consistent
//! loads and boundary conditions, solves `K u = F` ([`crate::linsolve`]), then
//! recovers element end moments and the peak bending stress `σ = M/Z`.

use crate::linsolve::solve;

/// Boundary condition at a node.
#[derive(Clone, Copy, Debug)]
pub enum Bc {
    /// Both `w` and `θ` fixed (built-in / cantilever support).
    Clamped(usize),
    /// `w` fixed, `θ` free (simple support / pin).
    Pinned(usize),
}

/// A beam model: node positions, per-element `EI` and section modulus `Z=I/c`,
/// plus an optional per-element axial **tension** for geometric (tension)
/// stiffening — the effect that lets a spinning rotor blade carry its lift far
/// stiffer than a static beam would.
#[derive(Clone, Debug)]
pub struct Beam {
    /// Node positions along the beam, m (ascending), length `n`.
    pub nodes_x: Vec<f64>,
    /// Bending stiffness `EI` per element, N·m², length `n−1`.
    pub ei: Vec<f64>,
    /// Section modulus `Z = I/c` per element, m³ (for stress), length `n−1`.
    pub section_modulus: Vec<f64>,
    /// Axial tension per element, N (≥0 stiffens), length `n−1`. Zero = pure beam.
    pub tension: Vec<f64>,
}

/// A transverse point load and/or moment applied at a node.
#[derive(Clone, Copy, Debug)]
pub struct NodalLoad {
    pub node: usize,
    /// Transverse force (+ in the `+w` direction), N.
    pub force: f64,
    /// Applied moment, N·m.
    pub moment: f64,
}

/// The solved response.
#[derive(Clone, Debug)]
pub struct BeamSolution {
    /// Transverse deflection at each node, m.
    pub deflection: Vec<f64>,
    /// Rotation at each node, rad.
    pub rotation: Vec<f64>,
    /// Peak |bending moment| over all element ends, N·m.
    pub max_moment_nm: f64,
    /// Peak bending stress `max|M|/Z`, Pa.
    pub max_stress_pa: f64,
    /// Peak |deflection|, m.
    pub max_deflection_m: f64,
}

impl Beam {
    fn n_nodes(&self) -> usize {
        self.nodes_x.len()
    }

    /// Consistent nodal load vector for a uniform distributed load `q` (N/m, + in
    /// `+w`) over the whole beam, added to any point loads passed to [`Self::solve`].
    pub fn uniform_load_vector(&self, q: f64) -> Vec<f64> {
        let n = self.n_nodes();
        let mut f = vec![0.0; 2 * n];
        for e in 0..n - 1 {
            let l = self.nodes_x[e + 1] - self.nodes_x[e];
            // [qL/2, qL²/12, qL/2, −qL²/12]
            f[2 * e] += q * l / 2.0;
            f[2 * e + 1] += q * l * l / 12.0;
            f[2 * e + 2] += q * l / 2.0;
            f[2 * e + 3] += -q * l * l / 12.0;
        }
        f
    }

    fn element_k(ei: f64, l: f64) -> [f64; 16] {
        let c = ei / (l * l * l);
        let l2 = l * l;
        [
            12.0 * c, 6.0 * l * c, -12.0 * c, 6.0 * l * c,
            6.0 * l * c, 4.0 * l2 * c, -6.0 * l * c, 2.0 * l2 * c,
            -12.0 * c, -6.0 * l * c, 12.0 * c, -6.0 * l * c,
            6.0 * l * c, 2.0 * l2 * c, -6.0 * l * c, 4.0 * l2 * c,
        ]
    }

    /// Geometric (tension) stiffness for axial tension `t`:
    /// `Kg = (T/30L)·[[36,3L,−36,3L],[3L,4L²,−3L,−L²],[−36,−3L,36,−3L],[3L,−L²,−3L,4L²]]`.
    fn element_kg(t: f64, l: f64) -> [f64; 16] {
        let c = t / (30.0 * l);
        let l2 = l * l;
        [
            36.0 * c, 3.0 * l * c, -36.0 * c, 3.0 * l * c,
            3.0 * l * c, 4.0 * l2 * c, -3.0 * l * c, -l2 * c,
            -36.0 * c, -3.0 * l * c, 36.0 * c, -3.0 * l * c,
            3.0 * l * c, -l2 * c, -3.0 * l * c, 4.0 * l2 * c,
        ]
    }

    /// Solve for the response under point loads, optional distributed-load vector
    /// (from [`Self::uniform_load_vector`]), and boundary conditions.
    pub fn solve(&self, loads: &[NodalLoad], dist: Option<&[f64]>, bcs: &[Bc]) -> Option<BeamSolution> {
        let n = self.n_nodes();
        let dof = 2 * n;
        let mut k = vec![0.0; dof * dof];
        // Assemble bending + geometric (tension) stiffness.
        for e in 0..n - 1 {
            let l = self.nodes_x[e + 1] - self.nodes_x[e];
            let ke = Self::element_k(self.ei[e], l);
            let t = self.tension.get(e).copied().unwrap_or(0.0);
            let kg = Self::element_kg(t, l);
            let map = [2 * e, 2 * e + 1, 2 * e + 2, 2 * e + 3];
            for i in 0..4 {
                for j in 0..4 {
                    k[map[i] * dof + map[j]] += ke[i * 4 + j] + kg[i * 4 + j];
                }
            }
        }
        // Load vector.
        let mut f = dist.map(|d| d.to_vec()).unwrap_or_else(|| vec![0.0; dof]);
        for ld in loads {
            f[2 * ld.node] += ld.force;
            f[2 * ld.node + 1] += ld.moment;
        }
        // Boundary conditions: zero the constrained DOF's row+col, unit diagonal.
        let constrain = |g: usize, k: &mut [f64], f: &mut [f64]| {
            for c in 0..dof {
                k[g * dof + c] = 0.0;
                k[c * dof + g] = 0.0;
            }
            k[g * dof + g] = 1.0;
            f[g] = 0.0;
        };
        for bc in bcs {
            match *bc {
                Bc::Clamped(node) => {
                    constrain(2 * node, &mut k, &mut f);
                    constrain(2 * node + 1, &mut k, &mut f);
                }
                Bc::Pinned(node) => constrain(2 * node, &mut k, &mut f),
            }
        }
        let u = solve(&k, &f, dof)?;

        let deflection: Vec<f64> = (0..n).map(|i| u[2 * i]).collect();
        let rotation: Vec<f64> = (0..n).map(|i| u[2 * i + 1]).collect();

        // Element end moments from f_e = K_e u_e (entries 1 and 3).
        let mut max_m = 0.0_f64;
        let mut max_stress = 0.0_f64;
        for e in 0..n - 1 {
            let l = self.nodes_x[e + 1] - self.nodes_x[e];
            let ke = Self::element_k(self.ei[e], l);
            let ue = [u[2 * e], u[2 * e + 1], u[2 * e + 2], u[2 * e + 3]];
            // Element end moments are rows 1 and 3 of K_e·u_e (4-wide rows).
            let m_i = (0..4).map(|j| ke[4 + j] * ue[j]).sum::<f64>();
            let m_j = (0..4).map(|j| ke[12 + j] * ue[j]).sum::<f64>();
            let m_max = m_i.abs().max(m_j.abs());
            if m_max > max_m {
                max_m = m_max;
            }
            let s = m_max / self.section_modulus[e];
            if s > max_stress {
                max_stress = s;
            }
        }
        let max_def = deflection.iter().cloned().fold(0.0_f64, |a, b| a.max(b.abs()));

        Some(BeamSolution {
            deflection,
            rotation,
            max_moment_nm: max_m,
            max_stress_pa: max_stress,
            max_deflection_m: max_def,
        })
    }
}

/// Build a uniform beam of length `len` (m) with `n_elements`, constant `ei` and
/// section modulus `z`.
pub fn uniform_beam(len: f64, n_elements: usize, ei: f64, z: f64) -> Beam {
    let n = n_elements + 1;
    let nodes_x = (0..n).map(|i| len * i as f64 / n_elements as f64).collect();
    Beam {
        nodes_x,
        ei: vec![ei; n_elements],
        section_modulus: vec![z; n_elements],
        tension: vec![0.0; n_elements],
    }
}
