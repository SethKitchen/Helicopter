//! **CfdAirfoil** — the bridge that lets the rotor fly on CFD-derived sectional
//! aerodynamics. It runs the viscous Navier–Stokes airfoil ([`helisim_cfd`]) across a
//! sweep of angles *once* to build a `(α, Cl, Cd)` polar, then implements the BEMT
//! solver's [`Airfoil`] trait by interpolating that polar — the same offline-CFD →
//! table → solver pattern real rotor codes use (a 12 s NS solve can't live inside the
//! BEMT inner loop).
//!
//! What comes from where (and why):
//! * **Drag** `Cd(α)` is the **viscous NS** result — the genuinely viscous, strongly
//!   Reynolds-dependent quantity the inviscid map returns as zero and the analytic
//!   high-Re [`helisim_airfoil::LinearAirfoil`] under-predicts at model scale.
//! * **Lift** `Cl(α)` is the **exact inviscid** Joukowski value `2π(1+ε/c)·…·sin α`
//!   (the CFD confirms its sign and linearity; its *magnitude* is finite-domain-
//!   suppressed in the NS solve, so the validated closed form is used instead).
//!
//! **Honest regime cap:** this is a *laminar, low-Re* polar — right for small
//! model-scale blades (chord-Reynolds ~1e4–1e5, where the high-Re NACA0012 model is
//! actually wrong), NOT a substitute for high-Re tables. It also has no stall model
//! (lift stays linear), so keep the angle range to attached flow.

use helisim_airfoil::{Airfoil, TableAirfoil};
use helisim_cfd::{AirfoilConfig, solve_airfoil_viscous};

/// A CFD-generated low-Reynolds airfoil polar, usable anywhere a [`&dyn Airfoil`] is.
pub struct CfdAirfoil {
    table: TableAirfoil,
    polar: Vec<(f64, f64, f64)>, // (alpha_deg, Cl, Cd), symmetric
    re_chord: f64,
}

impl CfdAirfoil {
    /// Build directly from a precomputed `(alpha_deg, Cl, Cd)` polar (fast; for reuse
    /// of an already-computed CFD sweep).
    pub fn from_polar_deg(rows: &[(f64, f64, f64)], re_chord: f64) -> Self {
        let mut polar = rows.to_vec();
        polar.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        CfdAirfoil { table: TableAirfoil::from_deg(&polar), polar, re_chord }
    }

    /// Generate the polar by running the **viscous CFD** at each `α ≥ 0` in
    /// `alphas_deg` (drag from NS, lift from the inviscid Joukowski reference), then
    /// mirror to negative angles (the section is symmetric: `Cd(−α)=Cd(α)`,
    /// `Cl(−α)=−Cl(α)`). `n_r`/`n_t` trade accuracy for build time.
    pub fn from_cfd_sweep(re_chord: f64, alphas_deg: &[f64], n_r: usize, n_t: usize) -> Self {
        let mut polar: Vec<(f64, f64, f64)> = Vec::new();
        for &deg in alphas_deg {
            let cfg = AirfoilConfig {
                n_r,
                n_t,
                r_max: 30.0,
                omega_relax: 0.3,
                te_round: 0.1,
                psi_sweeps: 8,
                max_steps: 6000,
                ..AirfoilConfig::new(deg.abs(), re_chord)
            };
            let sol = solve_airfoil_viscous(&cfg);
            let (_, cd) = sol.force_coefficients();
            let cl = sol.inviscid_lift(); // validated exact lift; sign from the angle
            polar.push((deg.abs(), cl, cd));
            if deg.abs() > 1e-9 {
                polar.push((-deg.abs(), -cl, cd));
            }
        }
        polar.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        CfdAirfoil { table: TableAirfoil::from_deg(&polar), polar, re_chord }
    }

    /// The generated `(alpha_deg, Cl, Cd)` polar.
    pub fn polar(&self) -> &[(f64, f64, f64)] {
        &self.polar
    }

    /// The chord-based Reynolds number the polar was generated at.
    pub fn re_chord(&self) -> f64 {
        self.re_chord
    }
}

impl Airfoil for CfdAirfoil {
    fn cl_cd(&self, alpha: f64, mach: f64) -> (f64, f64) {
        self.table.cl_cd(alpha, mach)
    }
}
