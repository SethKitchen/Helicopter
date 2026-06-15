//! Steady viscous flow past a circular cylinder — the canonical "body in the flow"
//! validation, solved by the **vorticity–streamfunction** method on the body-fitted
//! log-polar grid ([`PolarGrid`]).
//!
//! In log-polar coordinates (`r = e^ξ`, `θ = η`) the steady incompressible
//! Navier–Stokes equations become (freestream `U = 1`, radius `a = 1`, so the
//! diameter `D = 2` and `ν = 2/Re_D`):
//!
//! ```text
//! ψ_ξξ + ψ_ηη = −e^{2ξ} ω                                  (kinematics)
//! ∂ω/∂t = e^{−2ξ}[ −(ψ_η ω_ξ − ψ_ξ ω_η) + ν(ω_ξξ + ω_ηη) ]  (transport)
//! ```
//!
//! Boundary conditions: no-slip on the cylinder (`ψ = 0`, `ψ_ξ = 0` ⇒ Thom wall
//! vorticity `ω_w = −2ψ_1/dξ²`); uniform flow `ψ → e^ξ sin η`, `ω → 0` at the outer
//! boundary; periodic in `η`. The front stagnation sits at `η = π` (upstream), the
//! wake along `η = 0` (downstream).
//!
//! Marched in pseudo-time to steady state with **local time-stepping**: the stable
//! explicit step scales as `e^{2ξ}/denom`, which cancels the `e^{−2ξ}` metric in the
//! transport RHS so every cell advances at its own stability limit (the far field no
//! longer crawls). This is point-Jacobi relaxation of the steady balance — robustly
//! stable for a relaxation factor `≤ 1`, with the convection upwinded.

use crate::cylinder_solution::CylinderSolution;
use crate::polar_grid::PolarGrid;

/// Cylinder run settings.
#[derive(Clone, Copy, Debug)]
pub struct CylinderConfig {
    /// Diameter-based Reynolds number `Re_D = U·D/ν`.
    pub re_d: f64,
    /// Radial node count.
    pub n_r: usize,
    /// Azimuthal node count.
    pub n_t: usize,
    /// Outer-boundary radius in cylinder radii (the far field).
    pub r_max: f64,
    /// Local-time-step relaxation factor (`0 < relax ≤ 1`; the explicit stability cap).
    /// The default suits `Re_D ≈ 40`; **reduce it (≈0.3) for `Re_D ≲ 30`**, where the
    /// ψ↔ω coupling is stiffer and a large factor diverges.
    pub omega_relax: f64,
    /// SOR sweeps of the streamfunction Poisson per outer iteration.
    pub psi_sweeps: usize,
    /// Steady-state tolerance on the max per-iteration vorticity change.
    pub steady_tol: f64,
    /// Hard cap on outer iterations.
    pub max_steps: usize,
}

impl CylinderConfig {
    /// Sensible defaults for a steady low-Re run at `re_d`.
    pub fn new(re_d: f64) -> Self {
        CylinderConfig {
            re_d,
            n_r: 84,
            n_t: 120,
            r_max: 30.0,
            omega_relax: 0.8,
            psi_sweeps: 8,
            steady_tol: 5e-6,
            max_steps: 8_000,
        }
    }
}

/// Solve steady flow past the cylinder to a pseudo-time steady state.
pub fn solve_cylinder(cfg: &CylinderConfig) -> CylinderSolution {
    let grid = PolarGrid::new(cfg.n_r, cfg.n_t, cfg.r_max);
    let (n_r, n_t) = (grid.n_r, grid.n_t);
    let (dxi, deta) = (grid.dxi, grid.deta);
    let (dxi2, deta2) = (dxi * dxi, deta * deta);
    let nu = 2.0 / cfg.re_d; // ν = 2/Re_D  (U=1, a=1, D=2)

    let len = grid.len();
    let mut psi = vec![0.0; len];
    let mut omega = vec![0.0; len];
    let mut omega_new = vec![0.0; len];

    // Outer-boundary streamfunction: uniform flow ψ = e^ξ sin η; seed the interior
    // with it (a good initial guess that already satisfies the far field).
    for i in 1..n_r {
        let r = grid.r(i);
        for j in 0..n_t {
            psi[grid.idx(i, j)] = r * grid.eta(j).sin();
        }
    }

    let psi_sor = 1.7;
    let inv_psi = 1.0 / (2.0 / dxi2 + 2.0 / deta2);
    let diff_diag = nu * (2.0 / dxi2 + 2.0 / deta2);

    let mut converged = false;
    let mut steps = 0;
    while steps < cfg.max_steps {
        // 1. Kinematics: solve ψ_ξξ + ψ_ηη = −e^{2ξ}ω (Dirichlet in ξ, periodic in η).
        for _ in 0..cfg.psi_sweeps {
            for i in 1..n_r - 1 {
                let e2 = (2.0 * grid.xi(i)).exp();
                for j in 0..n_t {
                    let (jm, jp) = grid.eta_neighbors(j);
                    let k = grid.idx(i, j);
                    let lap = (psi[grid.idx(i + 1, j)] + psi[grid.idx(i - 1, j)]) / dxi2
                        + (psi[grid.idx(i, jp)] + psi[grid.idx(i, jm)]) / deta2;
                    let gs = (lap + e2 * omega[k]) * inv_psi;
                    psi[k] = (1.0 - psi_sor) * psi[k] + psi_sor * gs;
                }
            }
        }

        // 2. Wall vorticity (Thom): ψ=0, ψ_ξ=0 ⇒ ω_w = −2ψ(ξ=dξ)/dξ²; far field ω=0.
        for j in 0..n_t {
            omega[grid.idx(0, j)] = -2.0 * psi[grid.idx(1, j)] / dxi2;
            omega[grid.idx(n_r - 1, j)] = 0.0;
        }

        // 3. Vorticity transport, point-Jacobi with LOCAL time-stepping. The stable
        //    explicit step dt = relax·e^{2ξ}/denom cancels the e^{−2ξ} metric in the
        //    RHS, so the update is just (relax/denom)·[−conv + ν·diff] everywhere —
        //    every cell relaxed at its own rate (upwind convection ⇒ stable).
        let mut max_change = 0.0_f64;
        for i in 1..n_r - 1 {
            for j in 0..n_t {
                let (jm, jp) = grid.eta_neighbors(j);
                let k = grid.idx(i, j);
                let (kip, kim) = (grid.idx(i + 1, j), grid.idx(i - 1, j));
                let (kjp, kjm) = (grid.idx(i, jp), grid.idx(i, jm));

                // Convective coefficients a·ω_ξ + b·ω_η, a = ψ_η, b = −ψ_ξ (upwind).
                let a = (psi[kjp] - psi[kjm]) / (2.0 * deta);
                let b = -(psi[kip] - psi[kim]) / (2.0 * dxi);
                let (ap, am) = (a.max(0.0), (-a).max(0.0));
                let (bp, bm) = (b.max(0.0), (-b).max(0.0));
                let dwdxi = (ap * (omega[k] - omega[kim]) + am * (omega[k] - omega[kip])) / dxi;
                let dwdeta = (bp * (omega[k] - omega[kjm]) + bm * (omega[k] - omega[kjp])) / deta;
                let conv = dwdxi + dwdeta;
                let diff = (omega[kip] + omega[kim] - 2.0 * omega[k]) / dxi2
                    + (omega[kjp] + omega[kjm] - 2.0 * omega[k]) / deta2;
                // Local diagonal (diffusion + upwind convection) sets the stable step.
                let denom = diff_diag + (ap + am) / dxi + (bp + bm) / deta;
                let update = cfg.omega_relax * (-conv + nu * diff) / denom;
                omega_new[k] = omega[k] + update;
                max_change = max_change.max(update.abs());
            }
        }
        for i in 1..n_r - 1 {
            for j in 0..n_t {
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

    CylinderSolution { grid, psi, omega, re_d: cfg.re_d, steps, converged }
}
