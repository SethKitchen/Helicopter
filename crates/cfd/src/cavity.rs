//! Lid-driven cavity — the canonical viscous incompressible Navier–Stokes
//! benchmark — solved by the **vorticity–streamfunction** method.
//!
//! In 2D incompressible flow `u = ∂ψ/∂y`, `v = −∂ψ/∂x`, and the vorticity
//! `ω = ∂v/∂x − ∂u/∂y = −∇²ψ`. The N–S equations reduce to a Poisson link plus a
//! transport equation:
//!
//! ```text
//! ∇²ψ = −ω                         (kinematics — solved by SOR)
//! ∂ω/∂t + u·∂ω/∂x + v·∂ω/∂y = ν∇²ω  (vorticity transport — explicit march)
//! ```
//!
//! with `ν = U·L/Re` (here `U = L = 1`). Boundaries: no-slip on three walls, the
//! top **lid** sliding at `u = U`. Wall vorticity is set by **Thom's formula**
//! `ω_w = −2(ψ_{adj} − ψ_w)/h²` (plus `−2U/h` on the moving lid). We march the
//! pseudo-time to steady state. Convection uses first-order upwinding (robust at
//! any cell-Reynolds number); diffusion is central.

use crate::grid::Grid;
use crate::poisson::{optimal_omega, sor_solve};
use crate::solution::CavitySolution;

/// Cavity run settings.
#[derive(Clone, Copy, Debug)]
pub struct CavityConfig {
    /// Reynolds number `Re = U·L/ν`.
    pub re: f64,
    /// Nodes per side (odd ⇒ the centreline lies on a grid line).
    pub n: usize,
    /// Lid (top-wall) velocity `U`.
    pub lid_u: f64,
    /// Steady-state tolerance on the max per-step vorticity change.
    pub steady_tol: f64,
    /// Hard cap on pseudo-time steps.
    pub max_steps: usize,
    /// Time-step safety factor (≤ 1) against the explicit stability limit.
    pub cfl: f64,
}

impl CavityConfig {
    /// A run at Reynolds number `re` on an `n`-node grid (sensible defaults).
    pub fn new(re: f64, n: usize) -> Self {
        CavityConfig {
            re,
            n,
            lid_u: 1.0,
            steady_tol: 1e-6,
            max_steps: 200_000,
            cfl: 0.8,
        }
    }
}

/// Solve the lid-driven cavity to steady state.
pub fn solve_cavity(cfg: &CavityConfig) -> CavitySolution {
    let grid = Grid::square(cfg.n);
    let (nx, ny, h) = (grid.nx, grid.ny, grid.h);
    let nu = cfg.lid_u * 1.0 / cfg.re; // ν = U·L/Re
    let h2 = h * h;

    let n = grid.len();
    let mut psi = vec![0.0; n];
    let mut omega = vec![0.0; n];
    let mut omega_new = vec![0.0; n];
    let mut u = vec![0.0; n];
    let mut v = vec![0.0; n];
    let mut rhs = vec![0.0; n]; // −ω for the Poisson solve
    let omega_sor = optimal_omega(cfg.n);

    // Stable explicit step: bounded by both convection and diffusion.
    let u_ref = cfg.lid_u.max(1e-9);
    let dt = cfg.cfl / (2.0 * u_ref / h + 4.0 * nu / h2);

    let mut converged = false;
    let mut steps = 0;
    while steps < cfg.max_steps {
        // 1. Kinematics: ∇²ψ = −ω (ψ = 0 on all walls).
        for k in 0..n {
            rhs[k] = -omega[k];
        }
        sor_solve(&mut psi, &rhs, &grid, omega_sor, 1e-7, 60);

        // 2. Velocities from ψ (interior central differences).
        for j in 1..ny - 1 {
            for i in 1..nx - 1 {
                let k = grid.idx(i, j);
                u[k] = (psi[k + nx] - psi[k - nx]) / (2.0 * h);
                v[k] = -(psi[k + 1] - psi[k - 1]) / (2.0 * h);
            }
        }
        // Wall velocities: lid slides at U, others no-slip.
        for i in 0..nx {
            u[grid.idx(i, ny - 1)] = cfg.lid_u;
        }

        // 3. Wall vorticity (Thom's formula); the lid carries the −2U/h source.
        for i in 0..nx {
            omega[grid.idx(i, 0)] = -2.0 * psi[grid.idx(i, 1)] / h2; // bottom
            omega[grid.idx(i, ny - 1)] = -2.0 * psi[grid.idx(i, ny - 2)] / h2 - 2.0 * cfg.lid_u / h; // lid
        }
        for j in 0..ny {
            omega[grid.idx(0, j)] = -2.0 * psi[grid.idx(1, j)] / h2; // left
            omega[grid.idx(nx - 1, j)] = -2.0 * psi[grid.idx(nx - 2, j)] / h2; // right
        }

        // 4. Vorticity transport (explicit Euler; upwind convection, central diffusion).
        let mut max_change = 0.0_f64;
        for j in 1..ny - 1 {
            for i in 1..nx - 1 {
                let k = grid.idx(i, j);
                let dwdx = if u[k] > 0.0 {
                    (omega[k] - omega[k - 1]) / h
                } else {
                    (omega[k + 1] - omega[k]) / h
                };
                let dwdy = if v[k] > 0.0 {
                    (omega[k] - omega[k - nx]) / h
                } else {
                    (omega[k + nx] - omega[k]) / h
                };
                let lap = (omega[k - 1] + omega[k + 1] + omega[k - nx] + omega[k + nx]
                    - 4.0 * omega[k])
                    / h2;
                let rate = -u[k] * dwdx - v[k] * dwdy + nu * lap;
                omega_new[k] = omega[k] + dt * rate;
                max_change = max_change.max((omega_new[k] - omega[k]).abs());
            }
        }
        for j in 1..ny - 1 {
            for i in 1..nx - 1 {
                let k = grid.idx(i, j);
                omega[k] = omega_new[k];
            }
        }

        steps += 1;
        if max_change < cfg.steady_tol {
            converged = true;
            break;
        }
    }

    // Recover the pressure field from the converged velocity field (the quantity
    // the streamfunction form drops, but the path to forces).
    let pressure = crate::pressure::recover_pressure(&u, &v, &grid, 0.0);

    CavitySolution {
        grid,
        u,
        v,
        psi,
        omega,
        pressure,
        steps,
        converged,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Structural sanity (a fast, coarse run): the lid drives a single clockwise
    /// recirculation — ψ and the primary vortex are negative, the vortex sits in
    /// the upper-right quadrant, and the back-flow along the centreline is negative.
    #[test]
    fn lid_drives_a_clockwise_primary_vortex() {
        let sol = solve_cavity(&CavityConfig {
            max_steps: 20_000,
            ..CavityConfig::new(100.0, 41)
        });
        let (x, y, psi) = sol.primary_vortex();
        assert!(psi < 0.0, "recirculation streamfunction is negative");
        assert!(
            x > 0.5 && y > 0.5,
            "primary vortex in the upper-right quadrant"
        );
        let (u_min, _) = sol.min_centerline_u();
        assert!(u_min < 0.0, "centreline carries reverse flow");
    }
}
