//! Forward-flight solver configuration.

/// Settings for the forward-flight BEMT solve.
#[derive(Clone, Copy, Debug)]
pub struct ForwardConfig {
    /// Number of radial stations in the inner integral.
    pub n_radial: usize,
    /// Number of azimuthal stations (around 0…2π) in the inner integral.
    pub n_azimuth: usize,
    /// Bisection tolerance on the thrust-balance residual / bracket width.
    pub tol: f64,
    /// Maximum outer bisection iterations.
    pub max_iter: usize,
    /// Null the lift (and drag) in the reverse-flow region (`U_T < 0`).
    pub null_reverse_flow: bool,
}

impl Default for ForwardConfig {
    fn default() -> Self {
        ForwardConfig {
            n_radial: 60,
            n_azimuth: 72, // 5° azimuthal steps
            tol: 1e-9,
            max_iter: 200,
            null_reverse_flow: true,
        }
    }
}
