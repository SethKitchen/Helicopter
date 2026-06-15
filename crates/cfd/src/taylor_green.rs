//! Taylor–Green vortex — an **exact, closed-form unsteady solution** of the full
//! incompressible Navier–Stokes equations, and the canonical verification of a
//! time-dependent solver (the Ghia cavity validates the *steady* solver; this
//! validates the *unsteady* one against an analytic answer, not a dataset).
//!
//! On the doubly-periodic square `[0, 2π]²`:
//!
//! ```text
//! u(x,y,t) =  cos x · sin y · e^{−2νt}
//! v(x,y,t) = −sin x · cos y · e^{−2νt}
//! ω(x,y,t) = −2 cos x · cos y · e^{−2νt}   (ω = v_x − u_y)
//! ```
//!
//! The nonlinear convection term `u·∇ω` is **identically zero** for this field, so
//! the vorticity simply diffuses: `∂ω/∂t = ν∇²ω = −2ν·ω`, giving the exact decay
//! `e^{−2νt}` (kinetic energy `∝ e^{−4νt}`). A correct solver must reproduce that
//! rate — it tests the diffusion operator, the periodic Poisson link, and that the
//! discrete convection genuinely cancels. (Source: G. I. Taylor & A. E. Green,
//! *Proc. R. Soc. Lond. A* **158** (1937); a standard CFD verification case.)
//!
//! This solver uses **central** convection (the field is smooth and well resolved,
//! so no upwinding is needed) to avoid masking the physical decay with scheme
//! diffusion.

use std::f64::consts::PI;

/// A periodic vorticity–streamfunction march of the Taylor–Green vortex.
pub struct TaylorGreen {
    /// Nodes per side (covers `[0, 2π)`; node `i` at `x = i·h`).
    pub n: usize,
    /// Kinematic viscosity `ν`.
    pub nu: f64,
    h: f64,
}

impl TaylorGreen {
    /// Set up an `n×n` periodic grid at viscosity `nu`.
    pub fn new(n: usize, nu: f64) -> Self {
        TaylorGreen { n, nu, h: 2.0 * PI / n as f64 }
    }

    fn idx(&self, i: usize, j: usize) -> usize {
        j * self.n + i
    }

    /// Exact vorticity field at time `t`.
    pub fn exact_omega(&self, t: f64) -> Vec<f64> {
        let decay = (-2.0 * self.nu * t).exp();
        let mut w = vec![0.0; self.n * self.n];
        for j in 0..self.n {
            for i in 0..self.n {
                let (x, y) = (i as f64 * self.h, j as f64 * self.h);
                w[self.idx(i, j)] = -2.0 * x.cos() * y.cos() * decay;
            }
        }
        w
    }

    /// Exact kinetic energy per unit area, `½⟨u²+v²⟩ = ¼·e^{−4νt}` (the spatial mean
    /// of `½(cos²x sin²y + sin²x cos²y)` is `1/8`, times the `e^{−4νt}` factor… the
    /// constant cancels in the ratio we validate).
    pub fn exact_energy_ratio(&self, t: f64) -> f64 {
        (-4.0 * self.nu * t).exp()
    }

    /// Solve the periodic Poisson `∇²ψ = −ω` by SOR with zero-mean projection (the
    /// periodic Laplacian is singular up to a constant).
    fn periodic_poisson(&self, omega: &[f64]) -> Vec<f64> {
        let n = self.n;
        let h2 = self.h * self.h;
        let omega_sor = 1.7;
        let mut psi = vec![0.0; n * n];
        for _ in 0..400 {
            let mut maxr = 0.0_f64;
            for j in 0..n {
                for i in 0..n {
                    let k = self.idx(i, j);
                    let l = self.idx((i + n - 1) % n, j);
                    let r = self.idx((i + 1) % n, j);
                    let d = self.idx(i, (j + n - 1) % n);
                    let up = self.idx(i, (j + 1) % n);
                    let gs = (psi[l] + psi[r] + psi[d] + psi[up] + h2 * omega[k]) * 0.25;
                    maxr = maxr.max((gs - psi[k]).abs());
                    psi[k] = (1.0 - omega_sor) * psi[k] + omega_sor * gs;
                }
            }
            let mean: f64 = psi.iter().sum::<f64>() / (n * n) as f64;
            for p in psi.iter_mut() {
                *p -= mean;
            }
            if maxr < 1e-9 {
                break;
            }
        }
        psi
    }

    /// March from the exact initial field to time `t_end` and return the simulated
    /// kinetic-energy ratio `E(t_end)/E(0)` (to compare against `e^{−4νt}`).
    pub fn march_energy_ratio(&self, t_end: f64, cfl: f64) -> f64 {
        let n = self.n;
        let h = self.h;
        let h2 = h * h;
        let mut omega = self.exact_omega(0.0);
        let e0 = self.energy(&self.periodic_poisson(&omega));
        // Stable step: diffusion-limited (smooth field, u≈1).
        let dt = cfl * (h2 / (4.0 * self.nu)).min(h / 1.0);
        let mut t = 0.0;
        let mut wn = vec![0.0; n * n];
        while t < t_end {
            let psi = self.periodic_poisson(&omega);
            // Velocities (central, periodic).
            let mut u = vec![0.0; n * n];
            let mut v = vec![0.0; n * n];
            for j in 0..n {
                for i in 0..n {
                    let k = self.idx(i, j);
                    u[k] = (psi[self.idx(i, (j + 1) % n)] - psi[self.idx(i, (j + n - 1) % n)]) / (2.0 * h);
                    v[k] = -(psi[self.idx((i + 1) % n, j)] - psi[self.idx((i + n - 1) % n, j)]) / (2.0 * h);
                }
            }
            // Vorticity transport (central convection + central diffusion).
            for j in 0..n {
                for i in 0..n {
                    let k = self.idx(i, j);
                    let l = self.idx((i + n - 1) % n, j);
                    let r = self.idx((i + 1) % n, j);
                    let d = self.idx(i, (j + n - 1) % n);
                    let up = self.idx(i, (j + 1) % n);
                    let dwdx = (omega[r] - omega[l]) / (2.0 * h);
                    let dwdy = (omega[up] - omega[d]) / (2.0 * h);
                    let lap = (omega[l] + omega[r] + omega[d] + omega[up] - 4.0 * omega[k]) / h2;
                    wn[k] = omega[k] + dt * (-u[k] * dwdx - v[k] * dwdy + self.nu * lap);
                }
            }
            omega.copy_from_slice(&wn);
            t += dt;
        }
        let e_end = self.energy(&self.periodic_poisson(&omega));
        e_end / e0
    }

    /// Kinetic energy `½Σ(u²+v²)` from a streamfunction field (central velocities).
    fn energy(&self, psi: &[f64]) -> f64 {
        let n = self.n;
        let h = self.h;
        let mut e = 0.0;
        for j in 0..n {
            for i in 0..n {
                let u = (psi[self.idx(i, (j + 1) % n)] - psi[self.idx(i, (j + n - 1) % n)]) / (2.0 * h);
                let v = -(psi[self.idx((i + 1) % n, j)] - psi[self.idx((i + n - 1) % n, j)]) / (2.0 * h);
                e += 0.5 * (u * u + v * v);
            }
        }
        e
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The headline exact-NS check: the marched kinetic energy decays at the
    /// analytic Taylor–Green rate `E(t)/E(0) = e^{−4νt}`.
    #[test]
    fn energy_decays_at_the_exact_taylor_green_rate() {
        let tg = TaylorGreen::new(48, 0.1);
        let t_end = 2.0;
        let got = tg.march_energy_ratio(t_end, 0.4);
        let want = tg.exact_energy_ratio(t_end);
        assert!((got - want).abs() / want < 0.03, "energy ratio {got} vs exact {want}");
    }

    /// The periodic Poisson recovers ψ = ω/2 for the Taylor–Green field (since
    /// `∇²(cos x cos y) = −2 cos x cos y`), confirming the periodic kinematic link.
    #[test]
    fn periodic_poisson_recovers_the_taylor_green_streamfunction() {
        let tg = TaylorGreen::new(32, 0.1);
        let omega = tg.exact_omega(0.0);
        let psi = tg.periodic_poisson(&omega);
        // exact ψ = −cos x cos y = ω/2.
        let err = psi
            .iter()
            .zip(&omega)
            .map(|(p, w)| (p - w / 2.0).abs())
            .fold(0.0, f64::max);
        assert!(err < 5e-3, "max ψ−ω/2 error {err}");
    }
}
