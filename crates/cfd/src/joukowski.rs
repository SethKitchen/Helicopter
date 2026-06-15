//! **Joukowski airfoil** — the conformal map `ζ = z + c²/z` carries flow past a
//! circle into flow past an airfoil, turning the cylinder machinery into a *lifting*
//! section. This module computes the **inviscid** (potential-flow) lift with the
//! Kutta condition, which has an exact closed form to validate against — the natural
//! lift companion to the viscous cylinder (which gave drag).
//!
//! A circle of radius `a = c + ε` centred at `z₀ = −ε` passes through the critical
//! point `z = c` (the cusped trailing edge); the map sends it to a **symmetric**
//! airfoil of thickness `∝ ε`, chord `≈ 4c`. Uniform flow at incidence `α` past the
//! circle with the circulation `Γ = 4π U a sin α` (set by the Kutta condition —
//! rear stagnation pinned at the trailing edge) maps to the airfoil flow, and the
//! Kutta–Joukowski theorem gives
//!
//! ```text
//! C_l = 2Γ/(U·chord) = 2π(1 + ε/c) sin α          (→ 2π sin α as ε→0: thin-airfoil)
//! ```
//!
//! Integrating the surface pressure must reproduce this lift **and** return zero
//! drag (d'Alembert's paradox) — two independent checks the implementation has to
//! pass. (Camber would add an `α₀` offset; kept symmetric here.)

use crate::complex::C;
use std::f64::consts::PI;

/// A symmetric Joukowski airfoil defined by the map parameter `c` and the circle
/// offset `eps` (thickness grows with `eps/c`).
#[derive(Clone, Copy, Debug)]
pub struct JoukowskiAirfoil {
    pub c: f64,
    pub eps: f64,
}

/// Result of an inviscid solve: the integrated coefficients plus the surface
/// pressure distribution.
#[derive(Clone, Debug)]
pub struct AirfoilSolution {
    /// Lift coefficient from the surface-pressure integral.
    pub cl: f64,
    /// Drag coefficient from the surface-pressure integral (≈ 0 inviscid).
    pub cd: f64,
    /// `(x, y, Cp)` around the airfoil surface.
    pub surface: Vec<(f64, f64, f64)>,
}

impl JoukowskiAirfoil {
    /// A symmetric airfoil; `eps` is the circle offset (`≈ 0.77·t/c · c` for thin
    /// sections — `t/c ≈ 1.299·ε/c`).
    pub fn new(c: f64, eps: f64) -> Self {
        JoukowskiAirfoil { c, eps }
    }

    fn z0(&self) -> C {
        C::new(-self.eps, 0.0)
    }

    fn a(&self) -> f64 {
        self.c + self.eps
    }

    /// The conformal map `ζ = z + c²/z`.
    fn map(&self, z: C) -> C {
        z + C::new(self.c * self.c, 0.0) / z
    }

    /// A point on the circle at angle `eta`, then mapped to the airfoil surface.
    fn surface_z(&self, eta: f64) -> C {
        self.z0() + C::expi(eta).scale(self.a())
    }

    /// Airfoil chord (TE at `η=0` → LE at `η=π`, along x).
    pub fn chord(&self) -> f64 {
        self.map(self.surface_z(0.0)).re - self.map(self.surface_z(PI)).re
    }

    /// Thickness-to-chord ratio (max airfoil thickness / chord).
    pub fn thickness_ratio(&self) -> f64 {
        let n = 2000;
        let mut tmax = 0.0_f64;
        for k in 0..n {
            let eta = PI * k as f64 / n as f64;
            tmax = tmax.max(self.map(self.surface_z(eta)).im.abs());
        }
        2.0 * tmax / self.chord()
    }

    /// **Exact** inviscid lift from the Kutta–Joukowski circulation (closed form).
    pub fn lift_coefficient_exact(&self, alpha: f64) -> f64 {
        // Γ = 4π U a sin α, C_l = 2Γ/(U·chord); U = 1.
        let gamma = 4.0 * PI * self.a() * alpha.sin();
        2.0 * gamma / self.chord()
    }

    /// Solve the inviscid flow at incidence `alpha` and integrate the surface
    /// pressure to recover `(C_l, C_d)`. Uses `n` surface panels (offset off the
    /// trailing-edge cusp to avoid the `dζ/dz = 0` point).
    pub fn solve_inviscid(&self, alpha: f64, n: usize) -> AirfoilSolution {
        let (a, z0, c2) = (self.a(), self.z0(), self.c * self.c);
        let gamma = 4.0 * PI * a * alpha.sin();
        let u_inf = C::expi(-alpha); // freestream e^{-iα} (U = 1)

        // Complex velocity in the circle plane: dw/dz = U[e^{-iα} − a²e^{iα}/(z−z₀)²]
        //                                              + iΓ/(2π(z−z₀)).
        let dwdz = |z: C| -> C {
            let zr = z - z0;
            let t1 = u_inf;
            let t2 = C::expi(alpha).scale(a * a) / (zr * zr);
            let t3 = C::new(0.0, gamma / (2.0 * PI)) / zr;
            t1 - t2 + t3
        };
        let dzeta_dz = |z: C| -> C { C::new(1.0, 0.0) - C::new(c2, 0.0) / (z * z) };

        let dη = 2.0 * PI / n as f64;
        let mut surface = Vec::with_capacity(n);
        let (mut fx, mut fy) = (0.0, 0.0);
        for k in 0..n {
            let eta = (k as f64 + 0.5) * dη; // mid-panel: skips the TE cusp at η=0
            let z = self.surface_z(eta);
            let zeta = self.map(z);
            // Surface speed: |V| = |dw/dz| / |dζ/dz| (the map's stretching).
            let djdz = dzeta_dz(z);
            let speed = dwdz(z).abs() / djdz.abs();
            let cp = 1.0 - speed * speed; // U = 1
            // Surface element dζ/dη = (dζ/dz)·(i a e^{iη}); outward normal n = −i·t̂.
            let dz_deta = C::expi(eta).scale(a) * C::new(0.0, 1.0);
            let dzeta_deta = djdz * dz_deta;
            let dl = dzeta_deta.abs() * dη;
            let tangent = dzeta_deta.scale(1.0 / dzeta_deta.abs());
            let normal = tangent * C::new(0.0, -1.0); // rotate −90° → outward
            // dF = −Cp·n̂·dl (½ρU² folds into Cp; it cancels in the coefficient).
            fx += -cp * normal.re * dl;
            fy += -cp * normal.im * dl;
            surface.push((zeta.re, zeta.im, cp));
        }
        let chord = self.chord();
        // Project onto lift (⊥ freestream) and drag (∥ freestream).
        let cl = (-fx * alpha.sin() + fy * alpha.cos()) / chord;
        let cd = (fx * alpha.cos() + fy * alpha.sin()) / chord;
        AirfoilSolution { cl, cd, surface }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// d'Alembert + closed-form lift: the surface-pressure integral returns the exact
    /// Kutta–Joukowski lift and (essentially) zero drag — two independent checks of
    /// the conformal map, the Kutta circulation, and the force integration.
    #[test]
    fn surface_integral_matches_kutta_joukowski_and_dalembert() {
        let af = JoukowskiAirfoil::new(1.0, 0.1); // ~13% thick
        for &deg in &[3.0, 6.0, 10.0] {
            let alpha = deg * PI / 180.0;
            let s = af.solve_inviscid(alpha, 2000);
            let exact = af.lift_coefficient_exact(alpha);
            assert!((s.cl - exact).abs() / exact < 0.02, "Cl {} vs exact {}", s.cl, exact);
            assert!(s.cd.abs() < 0.02, "d'Alembert: Cd {} ≈ 0", s.cd);
        }
    }

    /// The lift slope is `≈ 2π(1+ε/c)` and collapses to thin-airfoil `2π` as the
    /// section thins — the result the rotor's analytic airfoil lift slope rests on.
    #[test]
    fn lift_slope_is_two_pi_with_thickness_correction() {
        let alpha = 4.0 * PI / 180.0;
        // Thin section → 2π.
        let thin = JoukowskiAirfoil::new(1.0, 0.001);
        let slope_thin = thin.lift_coefficient_exact(alpha) / alpha.sin();
        assert!((slope_thin - 2.0 * PI).abs() < 0.05, "thin slope {slope_thin} ≈ 2π");
        // Thicker → exceeds 2π by ≈ ε/c (here +10%).
        let thick = JoukowskiAirfoil::new(1.0, 0.1);
        let slope_thick = thick.lift_coefficient_exact(alpha) / alpha.sin();
        assert!(slope_thick > 2.0 * PI * 1.05, "thick slope {slope_thick} > 2π");
        // Symmetric ⇒ zero lift at zero incidence.
        assert!(thick.lift_coefficient_exact(0.0).abs() < 1e-12);
    }

    /// Geometry: the symmetric Joukowski thickness obeys the thin-section law
    /// `t/c → (3√3/4)·ε/c ≈ 1.299·ε/c` as `ε→0` (the leading-order coefficient), and
    /// stays a sensible `O(10%)` for a representative `ε/c = 0.1` section.
    #[test]
    fn thickness_ratio_follows_the_joukowski_geometry() {
        // Thin limit: the coefficient approaches 3√3/4.
        let thin = JoukowskiAirfoil::new(1.0, 0.01);
        let coeff = thin.thickness_ratio() / (thin.eps / thin.c);
        assert!((coeff - 1.299).abs() < 0.03, "thin t/c coefficient {coeff} ≈ 1.299");
        // Representative section: ~12% thick (exact geometry, below the thin estimate).
        let af = JoukowskiAirfoil::new(1.0, 0.1);
        let tc = af.thickness_ratio();
        assert!((0.10..0.13).contains(&tc), "t/c {tc} for ε/c=0.1");
    }
}
