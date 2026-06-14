//! Solver configuration.

/// BEMT solver settings.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Number of radial stations (annuli).
    pub n_stations: usize,
    /// Apply the Prandtl tip-loss correction.
    pub tip_loss: bool,
    /// Bisection tolerance on the thrust-balance residual / bracket width.
    pub tol: f64,
    /// Maximum bisection iterations per station.
    pub max_iter: usize,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            n_stations: 200,
            tip_loss: true,
            tol: 1e-10,
            max_iter: 200,
        }
    }
}
