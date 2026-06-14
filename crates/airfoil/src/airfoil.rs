//! The [`Airfoil`] trait — the polymorphism boundary for sectional aerodynamics.

/// Anything that can return sectional lift and drag coefficients.
///
/// The BEMT solver holds airfoils as `&dyn Airfoil` trait objects, so any model
/// implementing this trait (analytic, tabulated, Mach-corrected, ...) can be
/// plugged in interchangeably.
pub trait Airfoil {
    /// Lift and drag coefficients at angle of attack `alpha` (radians) and local
    /// Mach number `mach`. Models that ignore compressibility may disregard
    /// `mach`.
    fn cl_cd(&self, alpha: f64, mach: f64) -> (f64, f64);

    /// Lift coefficient only. Default implementation defers to [`Self::cl_cd`].
    fn cl(&self, alpha: f64, mach: f64) -> f64 {
        self.cl_cd(alpha, mach).0
    }

    /// Drag coefficient only. Default implementation defers to [`Self::cl_cd`].
    fn cd(&self, alpha: f64, mach: f64) -> f64 {
        self.cl_cd(alpha, mach).1
    }
}
