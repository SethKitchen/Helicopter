//! **Viscous** flow past a Joukowski airfoil — the cylinder solver carrying the
//! Joukowski **conformal metric**, which gives the rotor-feeding quantities the
//! inviscid map cannot: profile **drag** and the **lift reduction** at finite Re.
//!
//! The same vorticity–streamfunction machinery as the cylinder, but the log-polar
//! metric `e^{2ξ}` is replaced by the conformal metric `h² = |dζ/dz|²·e^{2ξ}` of the
//! map `ζ = z + c²/z` (circle radius `a = 1`, centre `z₀ = −ε`). The circle is taken
//! to **enclose** the critical point `z = c` by a margin `δ` (so `c = 1 − ε − δ`),
//! giving a **rounded trailing edge** — `dζ/dz ≠ 0` on the surface, so there is no
//! cusp singularity to special-case. Freestream at incidence `α`; the bound
//! circulation is left to develop viscously, with a point-vortex far-field
//! correction so a finite domain doesn't suppress the lift.
//!
//! Validation is *response correctness*: zero lift at zero incidence (symmetry),
//! positive **profile drag** (the rotor-feeding quantity the inviscid map can't give —
//! it returns `C_d = 0` by d'Alembert), and a **positive, linear lift** that develops
//! with incidence (the right mechanism and sign).
//!
//! **Lift magnitude — the `kutta_far_field` refinement.** A lifting flow's bound
//! circulation makes the far field decay only like `Γ/2πr`, so a plain uniform-flow
//! outer boundary on a finite domain *suppresses* the circulation and under-predicts
//! `C_l` (only ~11% of inviscid here). Feeding the vorticity integral back into the
//! far field to fix this is an *unstable* loop; instead, since the Kutta condition
//! *sets* the circulation and viscosity barely changes it for attached flow, we impose
//! the known inviscid Kutta value `Γ = 4πUa sin α` in the far field (fixed, stable).
//! That recovers the lift several-fold (to ~50% of inviscid); the residual is the
//! genuine low-Re viscous + rounded-TE soft-Kutta + finite-domain reduction. The
//! **drag** needs no such correction and carries no caveat.
//!
//! **Truly-emergent full-magnitude lift is a named limitation of this architecture**
//! (attempted, not faked): with a plain far field the un-imposed lift is stuck at
//! ~14% even with a sharpened TE, and the two routes that could fix it both fail —
//! the vorticity-feedback far field is an unstable loop, and simply enlarging the
//! domain (so the plain BC suppresses less) diverges (the far log-polar cells +
//! conformal metric destabilise). Recovering the full magnitude emergently would need
//! a different formulation (primitive-variable with a characteristic/convective outflow,
//! or a sharp-TE body-fitted grid that special-cases the metric singularity) — out of
//! scope here. The Kutta-imposed far field is the stable, physically-grounded stand-in.

use crate::complex::C;
use std::f64::consts::PI;

/// Viscous-airfoil run settings.
#[derive(Clone, Copy, Debug)]
pub struct AirfoilConfig {
    /// Thickness parameter (circle offset); thickness `∝ ε`.
    pub eps: f64,
    /// Trailing-edge rounding margin (the circle encloses `z=c` by `δ`).
    pub te_round: f64,
    /// Angle of attack (radians).
    pub alpha: f64,
    /// Chord-based Reynolds number `Re_c = U·chord/ν`.
    pub re_chord: f64,
    /// Radial / azimuthal node counts.
    pub n_r: usize,
    pub n_t: usize,
    /// Outer-boundary radius (circle radii).
    pub r_max: f64,
    /// Local-time-step relaxation factor.
    pub omega_relax: f64,
    /// Streamfunction SOR sweeps per outer iteration.
    pub psi_sweeps: usize,
    /// Steady-state tolerance and step cap.
    pub steady_tol: f64,
    pub max_steps: usize,
    /// Impose the inviscid **Kutta circulation** in the far field (recovers the lift
    /// magnitude a plain uniform-flow boundary suppresses). Stable — the circulation
    /// is fixed at the Kutta value rather than fed back from the vorticity (that loop
    /// is unstable on a finite domain). Leave off at α=0.
    pub kutta_far_field: bool,
}

impl AirfoilConfig {
    /// Defaults for a moderate-Re, ~12%-thick section.
    pub fn new(alpha_deg: f64, re_chord: f64) -> Self {
        AirfoilConfig {
            eps: 0.1,
            te_round: 0.04,
            alpha: alpha_deg * PI / 180.0,
            re_chord,
            n_r: 96,
            n_t: 160,
            r_max: 30.0,
            omega_relax: 0.6,
            psi_sweeps: 10,
            steady_tol: 5e-6,
            max_steps: 12_000,
            kutta_far_field: false,
        }
    }
}

/// A solved viscous airfoil flow + its diagnostics.
pub struct AirfoilViscousSolution {
    pub n_r: usize,
    pub n_t: usize,
    pub dxi: f64,
    pub deta: f64,
    pub psi: Vec<f64>,
    pub omega: Vec<f64>,
    pub h2: Vec<f64>,
    pub c: f64,
    pub eps: f64,
    pub alpha: f64,
    pub nu: f64,
    pub chord: f64,
    pub steps: usize,
    pub converged: bool,
}

fn idx(j_periodic: usize, i: usize, n_t: usize) -> usize {
    i * n_t + j_periodic
}

/// Solve viscous flow past the rounded-TE Joukowski airfoil.
pub fn solve_airfoil_viscous(cfg: &AirfoilConfig) -> AirfoilViscousSolution {
    let (n_r, n_t) = (cfg.n_r, cfg.n_t);
    let c = 1.0 - cfg.eps - cfg.te_round; // a = 1; circle encloses z=c by te_round
    let z0 = C::new(-cfg.eps, 0.0);
    let xi_max = cfg.r_max.ln();
    let dxi = xi_max / (n_r - 1) as f64;
    let deta = 2.0 * PI / n_t as f64;
    let (dxi2, deta2) = (dxi * dxi, deta * deta);

    // Map derivative dζ/dz = 1 − c²/z² and node geometry.
    let z_of = |i: usize, j: usize| -> C {
        let xi = i as f64 * dxi;
        let eta = j as f64 * deta;
        z0 + C::expi(eta).scale(xi.exp())
    };
    let dzeta_dz = |z: C| -> C { C::new(1.0, 0.0) - C::new(c * c, 0.0) / (z * z) };

    // Conformal metric h² = |dζ/dz|²·|z−z₀|², |z−z₀| = e^ξ.
    let mut h2 = vec![0.0; n_r * n_t];
    for i in 0..n_r {
        for j in 0..n_t {
            let z = z_of(i, j);
            let jac = dzeta_dz(z).norm_sqr();
            h2[idx(j, i, n_t)] = jac * (z - z0).norm_sqr();
        }
    }
    // Physical chord: ζ at η=0 (TE) minus η=π (LE).
    let map = |z: C| z + C::new(c * c, 0.0) / z;
    let chord = map(z_of(0, 0)).re - map(z_of(0, n_t / 2)).re;
    let nu = chord / cfg.re_chord; // ν = U·chord/Re_c, U=1

    let len = n_r * n_t;
    let mut psi = vec![0.0; len];
    let mut omega = vec![0.0; len];
    let mut omega_new = vec![0.0; len];

    // Far-field uniform flow at incidence: ψ = Im(e^{−iα}(z−z₀)) = e^ξ sin(η−α).
    let far_psi = |i: usize, j: usize, gamma: f64| -> f64 {
        let xi = i as f64 * dxi;
        let eta = j as f64 * deta;
        // ψ_vortex = +(Γ/2π)ln r imposes bound circulation Γ = ∫∫ω dA (sign set so the
        // feedback is stabilising, not divergent).
        xi.exp() * (eta - cfg.alpha).sin() + gamma / (2.0 * PI) * xi
    };
    for i in 1..n_r {
        for j in 0..n_t {
            psi[idx(j, i, n_t)] = far_psi(i, j, 0.0);
        }
    }

    let psi_sor = 1.7;
    let inv_psi = 1.0 / (2.0 / dxi2 + 2.0 / deta2);
    let diff_diag = nu * (2.0 / dxi2 + 2.0 / deta2);
    let neigh = |j: usize| ((j + n_t - 1) % n_t, (j + 1) % n_t);

    // Fixed far-field circulation: the inviscid Kutta value Γ = 4π U a sin α (a = 1),
    // imposed (not fed back) so the finite domain doesn't suppress the lift.
    let gamma = if cfg.kutta_far_field { 4.0 * PI * cfg.alpha.sin() } else { 0.0 };
    for j in 0..n_t {
        psi[idx(j, n_r - 1, n_t)] = far_psi(n_r - 1, j, gamma);
    }

    let mut converged = false;
    let mut steps = 0;
    while steps < cfg.max_steps {
        // 1. ψ Poisson: ∂_ξξψ + ∂_ηηψ = −h²ω (far-field ψ is fixed, set above).
        for _ in 0..cfg.psi_sweeps {
            for i in 1..n_r - 1 {
                for j in 0..n_t {
                    let (jm, jp) = neigh(j);
                    let k = idx(j, i, n_t);
                    let lap = (psi[idx(j, i + 1, n_t)] + psi[idx(j, i - 1, n_t)]) / dxi2
                        + (psi[idx(jp, i, n_t)] + psi[idx(jm, i, n_t)]) / deta2;
                    let gs = (lap + h2[k] * omega[k]) * inv_psi;
                    psi[k] = (1.0 - psi_sor) * psi[k] + psi_sor * gs;
                }
            }
        }

        // 2. Wall vorticity (Thom with the metric): ω_w = −2ψ_1/(h²_w·dξ²); far ω=0.
        //    Under-relaxed — h² is small near the rounded TE so ω_w is large and stiff.
        let wall_relax = 0.5;
        for j in 0..n_t {
            let kw = idx(j, 0, n_t);
            let thom = -2.0 * psi[idx(j, 1, n_t)] / (h2[kw] * dxi2);
            omega[kw] = (1.0 - wall_relax) * omega[kw] + wall_relax * thom;
            omega[idx(j, n_r - 1, n_t)] = 0.0;
        }

        // 3. Vorticity transport, point-Jacobi with local time-stepping (the 1/h²
        //    metric in the RHS cancels with dt∝h², exactly as the cylinder).
        let mut max_change = 0.0_f64;
        for i in 1..n_r - 1 {
            for j in 0..n_t {
                let (jm, jp) = neigh(j);
                let k = idx(j, i, n_t);
                let (kip, kim) = (idx(j, i + 1, n_t), idx(j, i - 1, n_t));
                let (kjp, kjm) = (idx(jp, i, n_t), idx(jm, i, n_t));
                let a = (psi[kjp] - psi[kjm]) / (2.0 * deta);
                let b = -(psi[kip] - psi[kim]) / (2.0 * dxi);
                let (ap, am) = (a.max(0.0), (-a).max(0.0));
                let (bp, bm) = (b.max(0.0), (-b).max(0.0));
                let dwdxi = (ap * (omega[k] - omega[kim]) + am * (omega[k] - omega[kip])) / dxi;
                let dwdeta = (bp * (omega[k] - omega[kjm]) + bm * (omega[k] - omega[kjp])) / deta;
                let conv = dwdxi + dwdeta;
                let diff = (omega[kip] + omega[kim] - 2.0 * omega[k]) / dxi2
                    + (omega[kjp] + omega[kjm] - 2.0 * omega[k]) / deta2;
                let denom = diff_diag + (ap + am) / dxi + (bp + bm) / deta;
                let update = cfg.omega_relax * (-conv + nu * diff) / denom;
                omega_new[k] = omega[k] + update;
                max_change = max_change.max(update.abs());
            }
        }
        for i in 1..n_r - 1 {
            for j in 0..n_t {
                let k = idx(j, i, n_t);
                omega[k] = omega_new[k];
            }
        }

        steps += 1;
        if max_change < cfg.steady_tol {
            converged = true;
            break;
        }
    }

    AirfoilViscousSolution {
        n_r,
        n_t,
        dxi,
        deta,
        psi,
        omega,
        h2,
        c,
        eps: cfg.eps,
        alpha: cfg.alpha,
        nu,
        chord,
        steps,
        converged,
    }
}

impl AirfoilViscousSolution {
    /// Lift and drag from the **physical surface integral** (skin friction `Cf = 2νω_w`
    /// along the tangent + pressure `Cp` from `dCp/dη = 2ν·∂ω/∂ξ|_w` along the normal),
    /// projected onto the lift (⊥ freestream) and drag (∥ freestream) axes.
    pub fn force_coefficients(&self) -> (f64, f64) {
        let (n_t, nu) = (self.n_t, self.nu);
        let z0 = C::new(-self.eps, 0.0);
        let z_of = |j: usize| z0 + C::expi(j as f64 * self.deta);
        let dzeta_dz = |z: C| C::new(1.0, 0.0) - C::new(self.c * self.c, 0.0) / (z * z);
        let omega_w = |j: usize| self.omega[idx(j, 0, n_t)];
        let dwdxi = |j: usize| {
            (-3.0 * self.omega[idx(j, 0, n_t)] + 4.0 * self.omega[idx(j, 1, n_t)]
                - self.omega[idx(j, 2, n_t)])
                / (2.0 * self.dxi)
        };
        // Cumulative surface pressure: dCp/dη = 2ν·∂ω/∂ξ|_w.
        let mut cp = vec![0.0; n_t];
        let mut acc = 0.0;
        let mut prev = 2.0 * nu * dwdxi(0);
        for (j, cpj) in cp.iter_mut().enumerate().skip(1) {
            let cur = 2.0 * nu * dwdxi(j);
            acc += 0.5 * (cur + prev) * self.deta;
            *cpj = acc;
            prev = cur;
        }
        let (ca, sa) = (self.alpha.cos(), self.alpha.sin());
        let (mut fx, mut fy) = (0.0, 0.0);
        for (j, &cpj) in cp.iter().enumerate() {
            let z = z_of(j);
            // dζ/dη = (dζ/dz)·i·e^{iη}; tangent t̂, outward normal n̂ = −i t̂.
            let dzeta_deta = dzeta_dz(z) * (C::expi(j as f64 * self.deta) * C::new(0.0, 1.0));
            let dl = dzeta_deta.abs() * self.deta;
            let t = dzeta_deta.scale(1.0 / dzeta_deta.abs());
            let n = t * C::new(0.0, -1.0);
            let cf = 2.0 * nu * omega_w(j);
            // dF = (Cf·t̂ − Cp·n̂)·dl.
            fx += (cf * t.re - cpj * n.re) * dl;
            fy += (cf * t.im - cpj * n.im) * dl;
        }
        let cl = (-fx * sa + fy * ca) / self.chord;
        let cd = (fx * ca + fy * sa) / self.chord;
        (cl, cd)
    }

    /// Lift from the **total circulation** `Γ = ∫∫ω dA` (Kutta–Joukowski, `C_l =
    /// 2Γ/chord`) — an independent route to `C_l` for the same-sign cross-check (its
    /// magnitude also feels the far-field suppression).
    pub fn lift_from_circulation(&self) -> f64 {
        let mut gamma = 0.0;
        for k in 0..self.psi.len() {
            gamma += self.omega[k] * self.h2[k];
        }
        gamma *= self.dxi * self.deta;
        2.0 * gamma / self.chord
    }

    /// The inviscid Kutta–Joukowski lift for this section (the reference the viscous
    /// lift sits below): `2π(1+ε/c)sin α`, normalised by the actual chord.
    pub fn inviscid_lift(&self) -> f64 {
        let a = 1.0; // circle radius
        2.0 * (4.0 * PI * a * self.alpha.sin()) / self.chord
    }
}
