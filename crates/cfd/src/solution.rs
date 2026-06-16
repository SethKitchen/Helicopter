//! The converged cavity flow field and the diagnostics used to compare against
//! the Ghia benchmark: centreline velocity profiles and the primary-vortex centre.

use crate::grid::Grid;

/// A solved lid-driven-cavity flow: velocity, streamfunction and vorticity on the
/// grid, plus convergence bookkeeping.
#[derive(Clone, Debug)]
pub struct CavitySolution {
    pub grid: Grid,
    /// x-velocity at each node.
    pub u: Vec<f64>,
    /// y-velocity at each node.
    pub v: Vec<f64>,
    /// Streamfunction ψ at each node.
    pub psi: Vec<f64>,
    /// Vorticity ω at each node.
    pub omega: Vec<f64>,
    /// Pressure `p` at each node, recovered from the velocity field via the
    /// pressure-Poisson equation (pinned to 0 at the bottom-left corner).
    pub pressure: Vec<f64>,
    /// Pseudo-time steps taken.
    pub steps: usize,
    /// Whether the steady-state tolerance was met.
    pub converged: bool,
}

impl CavitySolution {
    /// `u` along the **vertical centreline** `x = 0.5` as `(y, u)`. Requires an odd
    /// grid so the centreline lies on a column.
    pub fn centerline_u(&self) -> Vec<(f64, f64)> {
        let i = self.grid.nx / 2;
        (0..self.grid.ny)
            .map(|j| (self.grid.y(j), self.u[self.grid.idx(i, j)]))
            .collect()
    }

    /// `v` along the **horizontal centreline** `y = 0.5` as `(x, v)`.
    pub fn centerline_v(&self) -> Vec<(f64, f64)> {
        let j = self.grid.ny / 2;
        (0..self.grid.nx)
            .map(|i| (self.grid.x(i), self.v[self.grid.idx(i, j)]))
            .collect()
    }

    /// The minimum `u` on the vertical centreline and the `y` where it occurs.
    pub fn min_centerline_u(&self) -> (f64, f64) {
        self.centerline_u()
            .into_iter()
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .map(|(y, u)| (u, y))
            .unwrap()
    }

    /// The min and max `v` on the horizontal centreline (the two side jets).
    pub fn v_extrema(&self) -> (f64, f64) {
        let vs = self.centerline_v();
        let vmin = vs.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        let vmax = vs.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
        (vmin, vmax)
    }

    /// The **primary vortex** centre `(x, y, ψ)` — the most-negative streamfunction
    /// (for a lid moving in `+x`, the recirculation has `ψ < 0`).
    pub fn primary_vortex(&self) -> (f64, f64, f64) {
        let (mut k_min, mut p_min) = (0, f64::INFINITY);
        for (k, &p) in self.psi.iter().enumerate() {
            if p < p_min {
                p_min = p;
                k_min = k;
            }
        }
        let (i, j) = (k_min % self.grid.nx, k_min / self.grid.nx);
        (self.grid.x(i), self.grid.y(j), p_min)
    }

    /// The pressure range `(min, max)` over the field — a coarse diagnostic of the
    /// recovered pressure (stagnation high, vortex-core low).
    pub fn pressure_extrema(&self) -> (f64, f64) {
        let pmin = self.pressure.iter().copied().fold(f64::INFINITY, f64::min);
        let pmax = self
            .pressure
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        (pmin, pmax)
    }
}
