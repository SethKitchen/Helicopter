//! The assembled steady-autorotation result.

/// Outcome of a steady vertical-autorotation solve.
#[derive(Clone, Copy, Debug)]
pub struct AutorotationSolution {
    /// Equilibrium rate of descent `V_d`, m/s (positive downward).
    pub descent_rate_ms: f64,
    /// Descent rate normalised by hover induced velocity `V_d / v_h`. The measured
    /// ideal-autorotation band is ≈ [1.7, 2.0]; this is the validation target.
    pub descent_ratio: f64,
    /// Hover induced velocity `v_h = √(T/2ρA)`, m/s — the normalising scale.
    pub hover_induced_velocity_ms: f64,
    /// Induced velocity at the autorotative descent rate, m/s.
    pub induced_velocity_ms: f64,
    /// Profile power the descent is supplying, W.
    pub profile_power_w: f64,
    /// Thrust held (= weight), N.
    pub thrust_n: f64,
}

impl AutorotationSolution {
    /// Rate of descent in ft/min, the unit autorotation performance is usually
    /// quoted in.
    pub fn descent_rate_fpm(&self) -> f64 {
        self.descent_rate_ms * 196.850393
    }
}
