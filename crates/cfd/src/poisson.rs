//! Iterative solve of the Poisson equation `∇²φ = rhs` on the interior of a
//! [`Grid`] with Dirichlet boundaries (the boundary values of `phi` are held
//! fixed), by **successive over-relaxation** (SOR) — a hand-rolled, std-only
//! linear solve, the workhorse inside the vorticity–streamfunction march.
//!
//! Five-point Laplacian: `(φ_{i±1,j} + φ_{i,j±1} − 4φ_{i,j})/h² = rhs_{i,j}`.
//! The Gauss–Seidel update is over-relaxed by `ω` (1 < ω < 2). For an `n×n` grid
//! the optimal factor is `ω* = 2/(1 + sin(π/(n−1)))`.

use crate::grid::Grid;

/// Optimal SOR relaxation factor for a square grid (the analytic value for the
/// 5-point Laplacian on an `n`-node side).
pub fn optimal_omega(n: usize) -> f64 {
    2.0 / (1.0 + (std::f64::consts::PI / (n as f64 - 1.0)).sin())
}

/// SOR-solve `∇²phi = rhs` in place, with `phi`'s boundary entries held as the
/// Dirichlet condition. Iterates until the max nodal update `< tol` or `max_iter`
/// sweeps; returns the number of sweeps taken.
pub fn sor_solve(
    phi: &mut [f64],
    rhs: &[f64],
    grid: &Grid,
    omega: f64,
    tol: f64,
    max_iter: usize,
) -> usize {
    let (nx, ny) = (grid.nx, grid.ny);
    let h2 = grid.h * grid.h;
    for sweep in 0..max_iter {
        let mut max_update = 0.0_f64;
        for j in 1..ny - 1 {
            for i in 1..nx - 1 {
                let k = j * nx + i;
                // Gauss–Seidel value, then over-relax.
                let gs = (phi[k - 1] + phi[k + 1] + phi[k - nx] + phi[k + nx] - h2 * rhs[k]) * 0.25;
                let new = (1.0 - omega) * phi[k] + omega * gs;
                max_update = max_update.max((new - phi[k]).abs());
                phi[k] = new;
            }
        }
        if max_update < tol {
            return sweep + 1;
        }
    }
    max_iter
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// MANUFACTURED-SOLUTION ORACLE — for `φ = sin(πx)·sin(πy)` (zero on the unit-
    /// square boundary), `∇²φ = −2π²·φ`. Solving `∇²φ = −2π²·sin(πx)sin(πy)` with
    /// that boundary recovers the exact field to the second-order discretisation
    /// error (which shrinks as `h²`).
    #[test]
    fn recovers_a_manufactured_solution() {
        let solve_err = |n: usize| -> f64 {
            let g = Grid::square(n);
            let exact: Vec<f64> = (0..g.len())
                .map(|k| {
                    let (i, j) = (k % n, k / n);
                    (PI * g.x(i)).sin() * (PI * g.y(j)).sin()
                })
                .collect();
            let rhs: Vec<f64> = exact.iter().map(|&e| -2.0 * PI * PI * e).collect();
            let mut phi = vec![0.0; g.len()]; // zero boundary = the exact BC here
            sor_solve(&mut phi, &rhs, &g, optimal_omega(n), 1e-10, 5000);
            phi.iter()
                .zip(&exact)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0, f64::max)
        };
        let e41 = solve_err(41);
        assert!(e41 < 5e-3, "max error {e41} should be small");
        // Second-order: halving h quarters the error (≈4×).
        let e81 = solve_err(81);
        assert!(e81 < e41 / 3.0, "error must drop ~4× on refinement ({e41} → {e81})");
    }
}
