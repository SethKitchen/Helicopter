//! The converged cylinder flow field and the three benchmark diagnostics compared
//! across the literature: **drag coefficient**, **recirculation (wake) length**, and
//! **separation angle**.

use crate::polar_grid::PolarGrid;
use std::f64::consts::PI;

/// A solved steady flow past a circular cylinder.
#[derive(Clone, Debug)]
pub struct CylinderSolution {
    pub grid: PolarGrid,
    /// Streamfunction ψ.
    pub psi: Vec<f64>,
    /// Vorticity ω.
    pub omega: Vec<f64>,
    /// Diameter-based Reynolds number.
    pub re_d: f64,
    /// Pseudo-time steps taken.
    pub steps: usize,
    /// Whether steady state was reached.
    pub converged: bool,
}

impl CylinderSolution {
    /// **Drag coefficient** via the dissipation identity. For steady flow past a
    /// stationary no-slip body the drag power equals the total viscous dissipation,
    /// `F_D·U = μ∫ω²dΩ` (the surface `∫u_θ²` term vanishes at no-slip), giving the
    /// sign-free `C_D = (2/Re_D)∫ω² dA` with the log-polar area element
    /// `dA = e^{2ξ}dξ dη`. (Trapezoidal in ξ, periodic sum in η.)
    pub fn drag_coefficient(&self) -> f64 {
        let g = &self.grid;
        let mut integral = 0.0;
        for i in 0..g.n_r {
            let w_xi = if i == 0 || i == g.n_r - 1 { 0.5 } else { 1.0 }; // trapezoidal
            let e2 = (2.0 * g.xi(i)).exp();
            for j in 0..g.n_t {
                let w = self.omega[g.idx(i, j)];
                integral += w_xi * w * w * e2;
            }
        }
        integral *= g.dxi * g.deta;
        (2.0 / self.re_d) * integral
    }

    /// **Drag coefficient** via the **surface integral** (friction + pressure) — a
    /// local measure, free of the far-wake truncation that biases the dissipation
    /// form low. Returns `(C_Df, C_Dp, C_D)`.
    ///
    /// Friction: the wall shear is `∝` the wall vorticity, giving
    /// `C_Df = (2/Re_D)∮ −ω_w sin η dη`. Pressure: the surface pressure follows from
    /// the wall tangential-momentum balance `∂p/∂η = (2/Re_D)(∂ω/∂ξ)|_w`, integrated
    /// around the cylinder, then `C_Dp = −∮ p cos η dη` (η from the +x/wake axis;
    /// `½ρU²D = 1`).
    pub fn drag_coefficient_surface(&self) -> (f64, f64, f64) {
        let g = &self.grid;
        let c = 2.0 / self.re_d;
        // Wall vorticity and its one-sided ξ-derivative (2nd order) at each azimuth.
        let w0 = |j: usize| self.omega[g.idx(0, j)];
        let dwdxi = |j: usize| {
            (-3.0 * self.omega[g.idx(0, j)] + 4.0 * self.omega[g.idx(1, j)]
                - self.omega[g.idx(2, j)])
                / (2.0 * g.dxi)
        };
        // Friction drag.
        let mut cdf = 0.0;
        for j in 0..g.n_t {
            cdf += -w0(j) * g.eta(j).sin();
        }
        cdf *= c * g.deta;
        // Surface pressure by cumulative integration of ∂p/∂η around the cylinder.
        let mut p = vec![0.0; g.n_t];
        let mut acc = 0.0;
        let mut prev = dwdxi(0);
        for (j, pj) in p.iter_mut().enumerate().skip(1) {
            let cur = dwdxi(j);
            acc += c * 0.5 * (cur + prev) * g.deta;
            *pj = acc;
            prev = cur;
        }
        let mut cdp = 0.0;
        for (j, &pj) in p.iter().enumerate() {
            cdp += -pj * g.eta(j).cos();
        }
        cdp *= g.deta;
        (cdf, cdp, cdf + cdp)
    }

    /// **Recirculation length** `L_wake/D`: the distance from the rear of the
    /// cylinder to the reattachment point on the wake axis (`η = 0`), where the
    /// radial velocity `u_r = e^{−ξ}ψ_η` returns from reversed to forward, divided
    /// by the diameter `D = 2`. Returns 0 if the flow is unseparated.
    pub fn wake_length_over_d(&self) -> f64 {
        let g = &self.grid;
        let u_r = |i: usize| -> f64 {
            // On η = 0: ψ_η = (ψ(i,1) − ψ(i,n_t−1))/(2dη); by symmetry ≈ ψ(i,1)/dη.
            let psi_eta = (self.psi[g.idx(i, 1)] - self.psi[g.idx(i, g.n_t - 1)]) / (2.0 * g.deta);
            (-g.xi(i)).exp() * psi_eta
        };
        // Scan outward; find the reversed→forward crossing (end of the bubble).
        let mut reversed = false;
        for i in 1..g.n_r - 1 {
            let ur = u_r(i);
            if ur < 0.0 {
                reversed = true;
            } else if reversed && ur >= 0.0 {
                // Linear-interpolate the crossing radius between i−1 and i.
                let u0 = u_r(i - 1);
                let frac = -u0 / (ur - u0);
                let xi_c = g.xi(i - 1) + frac * g.dxi;
                return (xi_c.exp() - 1.0) / 2.0;
            }
        }
        0.0
    }

    /// **Separation angle** `θ_sep` (degrees, measured from the rear stagnation point
    /// `η = 0`): the azimuth where the wall vorticity (∝ wall shear) crosses zero.
    /// Returns 0 if the boundary layer stays attached.
    pub fn separation_angle_deg(&self) -> f64 {
        let g = &self.grid;
        let w = |j: usize| self.omega[g.idx(0, j)];
        // Scan from the rear (η=0) toward the front (η=π) for the first sign change.
        for j in 1..g.n_t / 2 {
            let (w0, w1) = (w(j), w(j + 1));
            if w0 * w1 < 0.0 {
                let frac = -w0 / (w1 - w0);
                let eta = g.eta(j) + frac * g.deta;
                return eta * 180.0 / PI;
            }
        }
        0.0
    }

    /// Wall vorticity `ω_w(η)` around the cylinder (the surface-shear distribution).
    pub fn surface_vorticity(&self) -> Vec<(f64, f64)> {
        let g = &self.grid;
        (0..g.n_t)
            .map(|j| (g.eta(j), self.omega[g.idx(0, j)]))
            .collect()
    }

    /// Top–bottom asymmetry of the streamfunction (should be ≈ 0 — the steady flow
    /// is symmetric about the wake axis; a self-consistency check, not an oracle).
    pub fn top_bottom_asymmetry(&self) -> f64 {
        let g = &self.grid;
        let mut max_abs = 0.0_f64;
        let mut max_asym = 0.0_f64;
        for i in 0..g.n_r {
            for j in 0..g.n_t {
                // Reflection about the x-axis (η → −η): ψ is antisymmetric.
                let jr = (g.n_t - j) % g.n_t;
                let s = self.psi[g.idx(i, j)] + self.psi[g.idx(i, jr)];
                max_asym = max_asym.max(s.abs());
                max_abs = max_abs.max(self.psi[g.idx(i, j)].abs());
            }
        }
        max_asym / max_abs.max(1e-30)
    }
}
