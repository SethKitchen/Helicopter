//! Integrated hover performance result.

use crate::station::Station;

/// Integrated hover performance plus the full spanwise distribution.
#[derive(Clone, Debug)]
pub struct HoverSolution {
    /// Thrust coefficient `C_T = T / (rho A (Omega R)^2)`.
    pub ct: f64,
    /// Power (== torque) coefficient `C_P = P / (rho A (Omega R)^3)`.
    pub cp: f64,
    /// Figure of merit `FM = C_T^{3/2} / (sqrt(2) C_P)`.
    pub figure_of_merit: f64,
    /// Dimensional thrust, N.
    pub thrust: f64,
    /// Dimensional torque, N·m.
    pub torque: f64,
    /// Dimensional shaft power, W.
    pub power: f64,
    /// Spanwise stations, root cutout → tip.
    pub stations: Vec<Station>,
}

impl HoverSolution {
    /// Blade loading coefficient `C_T / sigma`, a common normalisation for
    /// comparing rotors of different solidity. Caller supplies the solidity.
    pub fn ct_over_sigma(&self, solidity: f64) -> f64 {
        self.ct / solidity
    }
}
