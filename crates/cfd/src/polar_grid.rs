//! A **body-fitted log-polar grid** for flow past a circular cylinder: the cylinder
//! surface is the coordinate line `r = 1`, so the no-slip wall is exact (no
//! staircase, unlike an immersed boundary on a Cartesian mesh).
//!
//! The map is `r = e^ξ`, `θ = η`, so the unbounded exterior `r ∈ [1, r_max]`,
//! `θ ∈ [0, 2π)` becomes a rectangle uniform in `(ξ, η)` — clustering nodes near
//! the body (where gradients are large) for free. The Laplacian transforms to
//! `∇² = e^{−2ξ}(∂_ξξ + ∂_ηη)`, the workhorse of the Dennis & Chang (1970)
//! cylinder solver.

use std::f64::consts::PI;

/// Log-polar grid: `n_r` radial nodes `ξ_i = i·dξ` (i=0 at the cylinder), `n_t`
/// azimuthal nodes `η_j = j·dη` (periodic, `dη = 2π/n_t`).
#[derive(Clone, Debug)]
pub struct PolarGrid {
    /// Radial node count (`ξ` direction), `i = 0..n_r`.
    pub n_r: usize,
    /// Azimuthal node count (`η` direction, periodic), `j = 0..n_t`.
    pub n_t: usize,
    /// Radial spacing `dξ`.
    pub dxi: f64,
    /// Azimuthal spacing `dη = 2π/n_t`.
    pub deta: f64,
    /// Outer-boundary `ξ_max = ln(r_max)`.
    pub xi_max: f64,
}

impl PolarGrid {
    /// A grid with `n_r × n_t` nodes out to radius `r_max` (cylinder radius = 1).
    pub fn new(n_r: usize, n_t: usize, r_max: f64) -> Self {
        let xi_max = r_max.ln();
        PolarGrid {
            n_r,
            n_t,
            dxi: xi_max / (n_r - 1) as f64,
            deta: 2.0 * PI / n_t as f64,
            xi_max,
        }
    }

    /// Flat index (radial-major), azimuth periodic.
    pub fn idx(&self, i: usize, j: usize) -> usize {
        i * self.n_t + j
    }

    /// Total node count.
    pub fn len(&self) -> usize {
        self.n_r * self.n_t
    }

    /// Whether the grid has no nodes.
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// `ξ` at radial index `i`.
    pub fn xi(&self, i: usize) -> f64 {
        i as f64 * self.dxi
    }

    /// Physical radius `r = e^ξ` at radial index `i`.
    pub fn r(&self, i: usize) -> f64 {
        self.xi(i).exp()
    }

    /// `η` (azimuth, radians) at index `j`.
    pub fn eta(&self, j: usize) -> f64 {
        j as f64 * self.deta
    }

    /// Periodic azimuthal neighbours `(j−1, j+1)`.
    pub fn eta_neighbors(&self, j: usize) -> (usize, usize) {
        ((j + self.n_t - 1) % self.n_t, (j + 1) % self.n_t)
    }
}
