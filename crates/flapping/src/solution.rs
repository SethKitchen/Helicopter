//! Flapping solution.

/// First-harmonic flapping result and the hub reaction it implies.
#[derive(Clone, Copy, Debug)]
pub struct FlapSolution {
    /// Advance ratio.
    pub mu: f64,
    /// Inflow ratio used (from the forward-flight solve).
    pub lambda: f64,
    /// Rotating flap frequency `ν_β`.
    pub nu_beta: f64,
    /// Coning angle `β₀`, rad.
    pub beta0: f64,
    /// Longitudinal cyclic flapping `β₁c`, rad (≈ longitudinal TPP tilt).
    pub beta1c: f64,
    /// Lateral cyclic flapping `β₁s`, rad (≈ lateral TPP tilt).
    pub beta1s: f64,
    /// Hub pitching moment from the flap (∝ hinge offset), N·m.
    pub hub_pitch_moment: f64,
    /// Hub rolling moment from the flap (∝ hinge offset), N·m.
    pub hub_roll_moment: f64,
    /// Forcing-moment first harmonic `(cos, sin)` coefficients — for phase
    /// (90°-lag) analysis.
    pub forcing_1c: f64,
    /// Forcing-moment first harmonic sine coefficient.
    pub forcing_1s: f64,
}

impl FlapSolution {
    /// Tip-path-plane tilt magnitude, rad (`√(β₁c² + β₁s²)`).
    pub fn tpp_tilt(&self) -> f64 {
        (self.beta1c * self.beta1c + self.beta1s * self.beta1s).sqrt()
    }

    /// Phase (deg) of the aerodynamic forcing first harmonic.
    pub fn forcing_phase_deg(&self) -> f64 {
        self.forcing_1s.atan2(self.forcing_1c).to_degrees()
    }

    /// Phase (deg) of the flap response first harmonic. With
    /// `β = β₀ − β₁c cosψ − β₁s sinψ`, the response cos/sin coefficients are
    /// `(−β₁c, −β₁s)`.
    pub fn response_phase_deg(&self) -> f64 {
        (-self.beta1s).atan2(-self.beta1c).to_degrees()
    }

    /// Phase lag (deg, wrapped to [0,180]) of the flap response behind the
    /// aerodynamic forcing — expected near 90° for a resonant (`ν_β≈1`) rotor.
    pub fn phase_lag_deg(&self) -> f64 {
        let mut d = (self.forcing_phase_deg() - self.response_phase_deg()).abs() % 360.0;
        if d > 180.0 {
            d = 360.0 - d;
        }
        d
    }
}
