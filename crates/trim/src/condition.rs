//! The steady-flight condition to trim for.

/// A steady, level flight condition.
#[derive(Clone, Copy, Debug)]
pub struct TrimCondition {
    /// Forward speed, m/s (0 = hover).
    pub forward_speed: f64,
}

impl TrimCondition {
    /// Hover.
    pub fn hover() -> Self {
        TrimCondition { forward_speed: 0.0 }
    }

    /// Steady level forward flight at `v` m/s.
    pub fn forward(v: f64) -> Self {
        TrimCondition { forward_speed: v }
    }
}
