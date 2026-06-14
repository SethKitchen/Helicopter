//! Integrated forward-flight result.

use helisim_rotor::{Operating, Rotor};

/// Integrated forward-flight performance and the asymmetry it produces.
///
/// Moment-coefficient sign convention: coefficients are
/// `C = M / (ρ A (ΩR)² R)`. `c_roll` is the moment about the longitudinal
/// (fore–aft) axis from the lateral lift asymmetry — positive toward the
/// advancing side; `c_pitch` is about the lateral axis from any fore–aft
/// asymmetry (≈0 for uniform inflow).
#[derive(Clone, Copy, Debug)]
pub struct ForwardSolution {
    /// Advance ratio of the solve.
    pub mu: f64,
    /// Converged total inflow ratio `λ`.
    pub lambda: f64,
    /// Induced inflow ratio `λ_i = λ − μ tanα`.
    pub lambda_i: f64,
    /// Thrust coefficient `C_T` (azimuth-averaged).
    pub ct: f64,
    /// Power/torque coefficient `C_P = C_Q` (azimuth-averaged).
    pub cp: f64,
    /// Ideal induced power coefficient `λ_i · C_T`.
    pub cp_induced: f64,
    /// Profile power coefficient `C_P − C_P,induced`.
    pub cp_profile: f64,
    /// Rolling-moment coefficient (advancing-side excess lift).
    pub c_roll: f64,
    /// Pitching-moment coefficient.
    pub c_pitch: f64,
    /// Mean `C_T` from the advancing half (0 < ψ < π).
    pub advancing_ct: f64,
    /// Mean `C_T` from the retreating half (π < ψ < 2π).
    pub retreating_ct: f64,
    /// Fraction of the disk area in reverse flow (`U_T < 0`).
    pub reverse_flow_fraction: f64,
}

impl ForwardSolution {
    /// Dimensional shaft power, W, for the given operating point and rotor.
    pub fn power_w(&self, op: &Operating, rotor: &Rotor) -> f64 {
        let vt = op.tip_speed(rotor.radius);
        self.cp * op.rho * rotor.disk_area() * vt * vt * vt
    }

    /// Dimensional thrust, N.
    pub fn thrust_n(&self, op: &Operating, rotor: &Rotor) -> f64 {
        let vt = op.tip_speed(rotor.radius);
        self.ct * op.rho * rotor.disk_area() * vt * vt
    }

    /// Dimensional rolling moment, N·m.
    pub fn rolling_moment_nm(&self, op: &Operating, rotor: &Rotor) -> f64 {
        let vt = op.tip_speed(rotor.radius);
        self.c_roll * op.rho * rotor.disk_area() * vt * vt * rotor.radius
    }
}
