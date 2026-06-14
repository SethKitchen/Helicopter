//! Cyclic pitch control inputs.

/// First-harmonic cyclic pitch, radians. The blade pitch is
/// `θ(x,ψ) = θ_collective(x) + θ_1c cosψ + θ_1s sinψ`.
#[derive(Clone, Copy, Debug, Default)]
pub struct Controls {
    /// Lateral (cosine) cyclic pitch, rad.
    pub theta_1c: f64,
    /// Longitudinal (sine) cyclic pitch, rad.
    pub theta_1s: f64,
}

impl Controls {
    /// No cyclic input.
    pub fn none() -> Self {
        Controls::default()
    }

    /// From degrees.
    pub fn from_deg(theta_1c_deg: f64, theta_1s_deg: f64) -> Self {
        Controls {
            theta_1c: theta_1c_deg.to_radians(),
            theta_1s: theta_1s_deg.to_radians(),
        }
    }
}
