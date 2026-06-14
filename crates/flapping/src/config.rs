//! Flapping solver configuration.

/// Resolution of the numerical harmonic-balance integration.
#[derive(Clone, Copy, Debug)]
pub struct FlapConfig {
    /// Azimuthal samples used to project the flap moment onto its harmonics.
    pub n_azimuth: usize,
    /// Radial samples for the inner flap-moment integral.
    pub n_radial: usize,
}

impl Default for FlapConfig {
    fn default() -> Self {
        FlapConfig {
            n_azimuth: 180,
            n_radial: 80,
        }
    }
}
