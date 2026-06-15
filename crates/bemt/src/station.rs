//! Per-station converged BEMT state.

use helisim_rotor::Rotor;

/// Converged state at one radial station. Retained so callers can inspect or
/// plot the spanwise loading and inflow distribution (validation target #2).
#[derive(Clone, Debug)]
pub struct Station {
    /// Radial position `x = r/R`.
    pub x: f64,
    /// Converged induced inflow ratio `lambda = v_i / (Omega R)`.
    pub lambda: f64,
    /// Inflow angle, radians.
    pub phi: f64,
    /// Angle of attack, radians.
    pub alpha: f64,
    /// Sectional lift coefficient.
    pub cl: f64,
    /// Sectional drag coefficient.
    pub cd: f64,
    /// Prandtl tip-loss factor at this station.
    pub tip_loss: f64,
    /// Differential thrust coefficient `dC_T/dx`.
    pub dct_dx: f64,
    /// Differential power/torque coefficient `dC_P/dx`.
    pub dcp_dx: f64,
}

/// Converged section state (no differential coefficients yet) — the inputs to
/// [`Station::assemble`].
#[derive(Clone, Copy, Debug)]
pub(crate) struct SectionState {
    pub x: f64,
    pub lambda: f64,
    pub phi: f64,
    pub alpha: f64,
    pub cl: f64,
    pub cd: f64,
    pub tip_loss: f64,
}

impl Station {
    /// Assemble a station and its differential coefficients from converged
    /// section state. Centralises the `dCT/dx` and `dCP/dx` formulas so the
    /// solver and any future caller stay consistent.
    pub(crate) fn assemble(s: SectionState, rotor: &Rotor) -> Self {
        let SectionState {
            x,
            lambda,
            phi,
            alpha,
            cl,
            cd,
            tip_loss,
        } = s;
        let u2 = x * x + lambda * lambda;
        let sigma = rotor.local_solidity(x);
        let dct_dx = 0.5 * sigma * u2 * (cl * phi.cos() - cd * phi.sin());
        let dcp_dx = 0.5 * sigma * u2 * (cl * phi.sin() + cd * phi.cos()) * x;
        Station {
            x,
            lambda,
            phi,
            alpha,
            cl,
            cd,
            tip_loss,
            dct_dx,
            dcp_dx,
        }
    }
}
