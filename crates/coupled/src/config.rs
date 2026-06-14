//! Coupled-solver configuration.

use helisim_flapping::FlapConfig;

/// Settings for the flap↔inflow fixed-point solve.
#[derive(Clone, Copy, Debug)]
pub struct CoupledConfig {
    /// Azimuthal stations for the load integral.
    pub n_azimuth: usize,
    /// Radial stations for the load integral.
    pub n_radial: usize,
    /// Maximum flap↔inflow iterations.
    pub max_iter: usize,
    /// Convergence tolerance on the inflow ratio.
    pub tol: f64,
    /// Relaxation factor on the inflow update (0,1].
    pub relax: f64,
    /// Flapping sub-solver resolution.
    pub flap: FlapConfig,
}

impl Default for CoupledConfig {
    fn default() -> Self {
        CoupledConfig {
            n_azimuth: 48,
            n_radial: 40,
            max_iter: 60,
            tol: 1e-7,
            relax: 0.6,
            flap: FlapConfig::default(),
        }
    }
}
