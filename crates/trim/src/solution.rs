//! Trim result.

/// The converged trim: control inputs, attitude, and the loads/power that result.
#[derive(Clone, Copy, Debug)]
pub struct TrimResult {
    /// Whether the Newton solve converged.
    pub converged: bool,
    /// Residual L2 norm at the solution.
    pub residual_norm: f64,
    /// Forward speed, m/s.
    pub forward_speed: f64,
    /// Advance ratio at the trim.
    pub mu: f64,

    /// Main-rotor collective, rad.
    pub collective: f64,
    /// Lateral cyclic θ₁c, rad.
    pub cyclic_lat: f64,
    /// Longitudinal cyclic θ₁s, rad.
    pub cyclic_lon: f64,
    /// Tail-rotor collective, rad.
    pub tail_collective: f64,
    /// Fuselage pitch attitude, rad.
    pub pitch: f64,
    /// Fuselage roll attitude, rad.
    pub roll: f64,

    /// Main-rotor thrust, N.
    pub thrust: f64,
    /// Main-rotor power, W.
    pub main_power: f64,
    /// Tail-rotor thrust, N.
    pub tail_thrust: f64,
    /// Tail-rotor power, W.
    pub tail_power: f64,
    /// Airframe parasite power (D·V), W.
    pub parasite_power: f64,
    /// Total power (main rotor + tail + parasite), W.
    pub total_power: f64,
    /// Longitudinal flap β₁c, rad.
    pub beta1c: f64,
    /// Lateral flap β₁s, rad.
    pub beta1s: f64,
}
