//! Pressure recovery — the field the vorticity–streamfunction form drops, but the
//! one you need for **forces** (the path to airfoil `Cl/Cd`).
//!
//! Taking the divergence of the incompressible momentum equation and using
//! continuity (`u_x + v_y = 0`) gives the **pressure-Poisson** equation
//!
//! ```text
//! ∇²p = −ρ[(u_x)² + 2·u_y·v_x + (v_y)²]
//! ```
//!
//! (ρ = 1 here). On a closed domain the wall pressure is not known a priori, so
//! the natural boundary condition is **Neumann** (`∂p/∂n = 0` to leading order),
//! which leaves `p` defined up to a constant — pinned at one node. Solved by SOR.

use crate::grid::Grid;

/// The pressure-Poisson source `−ρ[(u_x)² + 2 u_y v_x + (v_y)²]` (ρ = 1) from a
/// velocity field, by central differences on the interior (zero on the boundary).
pub fn pressure_source(u: &[f64], v: &[f64], grid: &Grid) -> Vec<f64> {
    let (nx, ny, h) = (grid.nx, grid.ny, grid.h);
    let mut s = vec![0.0; grid.len()];
    for j in 1..ny - 1 {
        for i in 1..nx - 1 {
            let k = grid.idx(i, j);
            let ux = (u[k + 1] - u[k - 1]) / (2.0 * h);
            let uy = (u[k + nx] - u[k - nx]) / (2.0 * h);
            let vx = (v[k + 1] - v[k - 1]) / (2.0 * h);
            let vy = (v[k + nx] - v[k - nx]) / (2.0 * h);
            s[k] = -(ux * ux + 2.0 * uy * vx + vy * vy);
        }
    }
    s
}

/// Solve `∇²p = source` with homogeneous Neumann boundaries (`∂p/∂n = 0`), pinning
/// `p` at node `(0,0)` to `pin_value` (the Neumann problem is singular up to a
/// constant). SOR; returns the pressure field.
pub fn solve_pressure(source: &[f64], grid: &Grid, omega: f64, pin_value: f64) -> Vec<f64> {
    let (nx, ny) = (grid.nx, grid.ny);
    let h2 = grid.h * grid.h;
    let mut p = vec![0.0; grid.len()];
    // Second-order homogeneous Neumann via **mirror ghost nodes**: the value across
    // a wall reflects the inward node (`∂p/∂n = 0` to O(h²)). Every node — boundary
    // included — is an unknown solved with reflected neighbours.
    let mir = |a: usize, n: usize| if a == 0 { 1 } else if a == n - 1 { n - 2 } else { 0 };
    let len = grid.len();
    let mut prev = vec![0.0; len];
    for _ in 0..5_000 {
        for j in 0..ny {
            for i in 0..nx {
                let k = j * nx + i;
                let il = if i == 0 { mir(i, nx) } else { i - 1 };
                let ir = if i == nx - 1 { mir(i, nx) } else { i + 1 };
                let jd = if j == 0 { mir(j, ny) } else { j - 1 };
                let ju = if j == ny - 1 { mir(j, ny) } else { j + 1 };
                let sum = p[j * nx + il] + p[j * nx + ir] + p[jd * nx + i] + p[ju * nx + i];
                let gs = (sum - h2 * source[k]) * 0.25;
                p[k] = (1.0 - omega) * p[k] + omega * gs;
            }
        }
        // Project out the constant mode (Neumann null space) so the field can't drift
        // — otherwise the convergence measure never settles.
        let mean: f64 = p.iter().sum::<f64>() / len as f64;
        let mut max_change = 0.0_f64;
        for (pv, pp) in p.iter_mut().zip(prev.iter_mut()) {
            *pv -= mean;
            max_change = max_change.max((*pv - *pp).abs());
            *pp = *pv;
        }
        if max_change < 1e-7 {
            break;
        }
    }
    // Pin the gauge once at the end: fix p(0,0) = pin_value.
    let shift = pin_value - p[0];
    for pv in p.iter_mut() {
        *pv += shift;
    }
    p
}

/// Recover the pressure field from a velocity field (source + Neumann solve),
/// pinned to `pin_value` at the corner.
pub fn recover_pressure(u: &[f64], v: &[f64], grid: &Grid, pin_value: f64) -> Vec<f64> {
    let source = pressure_source(u, v, grid);
    solve_pressure(&source, grid, crate::poisson::optimal_omega(grid.nx), pin_value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// MANUFACTURED ORACLE — `p = cos(πx)·cos(πy)` satisfies `∇²p = −2π²p` and has
    /// **homogeneous Neumann** boundaries on the unit square (`∂p/∂x = −π sin(πx)…`
    /// vanishes at x = 0,1; likewise y). Solving `∇²p = −2π²cos(πx)cos(πy)` with the
    /// Neumann condition, pinned to the exact corner value, recovers it to the
    /// second-order discretisation error.
    #[test]
    fn recovers_a_manufactured_neumann_pressure() {
        let err = |n: usize| -> f64 {
            let g = Grid::square(n);
            let exact: Vec<f64> = (0..g.len())
                .map(|k| (PI * g.x(k % n)).cos() * (PI * g.y(k / n)).cos())
                .collect();
            let source: Vec<f64> = exact.iter().map(|&e| -2.0 * PI * PI * e).collect();
            let p = solve_pressure(&source, &g, crate::poisson::optimal_omega(n), exact[0]);
            p.iter().zip(&exact).map(|(a, b)| (a - b).abs()).fold(0.0, f64::max)
        };
        let e41 = err(41);
        assert!(e41 < 1e-2, "max pressure error {e41}");
        assert!(err(81) < e41 / 3.0, "second-order: refines ~4×");
    }
}
