//! Analytic airfoil: linear lift curve with a symmetric stall cutoff, a
//! quadratic drag polar, and optional Prandtl–Glauert compressibility.

use crate::airfoil::Airfoil;

/// Linear-lift airfoil: `Cl = a0 * (alpha - alpha0)` clamped at `±cl_max`, with a
/// quadratic drag polar `Cd = cd0 + cd1*|a| + cd2*a^2`. When `compressible` is
/// set, the lift-curve slope is scaled by the Prandtl–Glauert factor
/// `1/sqrt(1 - M^2)`.
#[derive(Clone, Debug)]
pub struct LinearAirfoil {
    /// Incompressible lift-curve slope, per radian.
    pub a0: f64,
    /// Zero-lift angle of attack, radians (0 for a symmetric section).
    pub alpha0: f64,
    /// Maximum |Cl| before a flat post-stall plateau.
    pub cl_max: f64,
    /// Minimum profile drag.
    pub cd0: f64,
    /// Linear drag coefficient (per radian).
    pub cd1: f64,
    /// Quadratic drag coefficient (per radian^2).
    pub cd2: f64,
    /// Apply Prandtl–Glauert compressibility to the lift-curve slope.
    pub compressible: bool,
}

impl LinearAirfoil {
    /// NACA 0012 — the symmetric section of the Caradonna & Tung rotor.
    ///
    /// `a0 = 5.73 /rad` is the classic measured slope for NACA 0012 (below the
    /// thin-airfoil `2*pi` due to viscosity); `cl_max ~ 1.4` near 14°.
    ///
    /// The drag polar is fitted to low-speed NACA 0012 data (Re ~ 1–3 M):
    /// `Cd ~ 0.0065` at zero lift, ~0.008 at 4°, ~0.011 at 8°, ~0.017 at 12°.
    /// (`Cd = 0.0065 + 0.28 * alpha^2`, alpha in radians.)
    pub fn naca0012() -> Self {
        LinearAirfoil {
            a0: 5.73,
            alpha0: 0.0,
            cl_max: 1.4,
            cd0: 0.0065,
            cd1: 0.0,
            cd2: 0.28,
            compressible: true,
        }
    }

    /// Same as [`Self::naca0012`] but with compressibility disabled — useful for
    /// incompressible cross-checks and unit tests.
    pub fn naca0012_incompressible() -> Self {
        LinearAirfoil {
            compressible: false,
            ..Self::naca0012()
        }
    }
}

impl Airfoil for LinearAirfoil {
    fn cl_cd(&self, alpha: f64, mach: f64) -> (f64, f64) {
        let a = alpha - self.alpha0;

        // Prandtl–Glauert: lift-curve slope grows toward the critical Mach.
        // Clamp the Mach term so the correction stays finite and physical.
        let slope = if self.compressible {
            let m = mach.clamp(0.0, 0.92);
            self.a0 / (1.0 - m * m).sqrt()
        } else {
            self.a0
        };

        // Linear lift up to stall, then a flat plateau at ±cl_max. The plateau
        // keeps the solver well-behaved if an inboard station momentarily
        // exceeds stall during iteration.
        let cl = (slope * a).clamp(-self.cl_max, self.cl_max);

        // Quadratic drag polar on the geometric angle.
        let cd = self.cd0 + self.cd1 * a.abs() + self.cd2 * a * a;

        (cl, cd)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn slope_and_zero_lift() {
        let af = LinearAirfoil::naca0012_incompressible();
        assert!(af.cl(0.0, 0.0).abs() < 1e-12);
        assert!((af.cl(0.1, 0.0) - 0.573).abs() < 1e-9);
    }

    #[test]
    fn compressibility_raises_slope() {
        let af = LinearAirfoil::naca0012();
        assert!(af.cl(0.05, 0.6) > af.cl(0.05, 0.0));
    }

    #[test]
    fn stall_plateau() {
        let af = LinearAirfoil::naca0012_incompressible();
        assert!((af.cl(1.0, 0.0) - af.cl_max).abs() < 1e-9);
    }
}
