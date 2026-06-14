//! Coupled-solve result.

use helisim_rotor::{Operating, Rotor};

/// Jointly-converged flap + inflow forward-flight solution.
#[derive(Clone, Copy, Debug)]
pub struct CoupledSolution {
    /// Advance ratio.
    pub mu: f64,
    /// Converged inflow ratio.
    pub lambda: f64,
    /// Coning, rad.
    pub beta0: f64,
    /// Longitudinal flap, rad.
    pub beta1c: f64,
    /// Lateral flap, rad.
    pub beta1s: f64,
    /// Thrust coefficient (with flapping).
    pub ct: f64,
    /// Power/torque coefficient (full torque integral, with flapping).
    pub cp: f64,
    /// Profile-only power coefficient (always ≥ 0).
    pub cp_profile: f64,
    /// Rolling-moment coefficient.
    pub c_roll: f64,
    /// Pitching-moment coefficient.
    pub c_pitch: f64,
    /// Advancing-half mean C_T.
    pub advancing_ct: f64,
    /// Retreating-half mean C_T.
    pub retreating_ct: f64,
    /// Whether the flap↔inflow iteration converged.
    pub converged: bool,
    /// Iterations taken.
    pub iterations: usize,
}

impl CoupledSolution {
    /// Dimensional thrust, N.
    pub fn thrust_n(&self, op: &Operating, rotor: &Rotor) -> f64 {
        let vt = op.tip_speed(rotor.radius);
        self.ct * op.rho * rotor.disk_area() * vt * vt
    }

    /// Dimensional rotor power from the full torque integral, W. Reliable at
    /// moderate μ; can turn autorotative (negative) at high μ / low collective —
    /// prefer [`Self::rotor_power_w`] for trim.
    pub fn power_w(&self, op: &Operating, rotor: &Rotor) -> f64 {
        let vt = op.tip_speed(rotor.radius);
        self.cp * op.rho * rotor.disk_area() * vt * vt * vt
    }

    /// Physically-decomposed rotor power (induced + profile), W — always ≥ 0.
    /// Induced from momentum (`κ·C_T·λ`), profile from the drag-only integral.
    /// `kappa` is the induced power factor (~1.15). This is the trim-safe power
    /// that stays physical across the full speed range.
    pub fn rotor_power_w(&self, op: &Operating, rotor: &Rotor, kappa: f64) -> f64 {
        let vt = op.tip_speed(rotor.radius);
        let q = op.rho * rotor.disk_area() * vt * vt * vt;
        (kappa * self.ct.max(0.0) * self.lambda + self.cp_profile.max(0.0)) * q
    }

    /// Dimensional shaft torque, N·m.
    pub fn torque_nm(&self, op: &Operating, rotor: &Rotor) -> f64 {
        self.power_w(op, rotor) / op.omega
    }
}
