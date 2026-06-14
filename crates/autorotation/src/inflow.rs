//! Axial-flight induced velocity across the descent regimes.
//!
//! Normalise the climb/descent rate by the hover induced velocity
//! `v_h = √(T / 2ρA)` and write `V̄_c = V_c/v_h` (positive = climb). Momentum
//! theory gives the induced-velocity ratio `v_i/v_h` as a function of `V̄_c`, but
//! only on the two ends:
//!
//! * **Climb / normal working state** (`V̄_c ≥ 0`):
//!   `v_i/v_h = -V̄_c/2 + √((V̄_c/2)² + 1)`.
//! * **Windmill-brake state** (`V̄_c ≤ -2`): the rotor extracts energy from the
//!   upflow; `v_i/v_h = -V̄_c/2 - √((V̄_c/2)² - 1)`.
//!
//! In between (`-2 < V̄_c < 0`) lies the **vortex-ring** and **turbulent-wake**
//! state: the flow recirculates, momentum theory's single-stream assumption fails
//! (the square root would go imaginary), and the induced velocity must come from
//! measurement. We use the standard quartic fit to the measured data:
//!
//! `v_i/v_h = k0 + k1 V̄_c + k2 V̄_c² + k3 V̄_c³ + k4 V̄_c⁴`,
//! `(k0..k4) = (1, -1.125, -1.372, -1.718, -0.655)`.
//!
//! Source: J. G. Leishman, *Principles of Helicopter Aerodynamics* (2nd ed.,
//! §2.13.4), the curve fit to the axial-descent induced-velocity measurements
//! (Castles & Gray; Washizu et al.). Valid for `-2 ≤ V̄_c ≤ 0`. This is an
//! *empirical* anchor, named as such: real steady autorotation falls inside this
//! band, so the fit — not a closed form — carries the descent-rate prediction.

/// Hover induced velocity `v_h = √(T / 2ρA)`, m/s. The normalising scale for all
/// axial-flight induced-velocity relations.
pub fn hover_induced_velocity(thrust_n: f64, rho: f64, disk_area_m2: f64) -> f64 {
    (thrust_n / (2.0 * rho * disk_area_m2)).sqrt()
}

/// Measured-data quartic fit for `v_i/v_h` in the vortex-ring / turbulent-wake
/// band, valid for `-2 ≤ V̄_c ≤ 0` (Leishman §2.13.4; see module docs).
pub fn vortex_ring_inflow_ratio(vc_over_vh: f64) -> f64 {
    let k = [1.0, -1.125, -1.372, -1.718, -0.655];
    let x = vc_over_vh;
    k[0] + k[1] * x + k[2] * x * x + k[3] * x.powi(3) + k[4] * x.powi(4)
}

/// Windmill-brake-state induced-velocity ratio (exact momentum theory), valid for
/// `V̄_c ≤ -2`: `v_i/v_h = -V̄_c/2 - √((V̄_c/2)² - 1)`.
pub fn windmill_brake_inflow_ratio(vc_over_vh: f64) -> f64 {
    let half = vc_over_vh / 2.0;
    -half - (half * half - 1.0).sqrt()
}

/// Climb / normal-working-state induced-velocity ratio (exact momentum theory),
/// valid for `V̄_c ≥ 0`: `v_i/v_h = -V̄_c/2 + √((V̄_c/2)² + 1)`.
pub fn climb_inflow_ratio(vc_over_vh: f64) -> f64 {
    let half = vc_over_vh / 2.0;
    -half + (half * half + 1.0).sqrt()
}

/// Induced-velocity ratio `v_i/v_h` for any normalised axial velocity `V̄_c`,
/// dispatching to the regime-appropriate model:
/// climb (`≥0`) and windmill brake (`≤-2`) use exact momentum theory; the
/// vortex-ring/turbulent-wake band uses the measured fit.
pub fn descent_inflow_ratio(vc_over_vh: f64) -> f64 {
    if vc_over_vh >= 0.0 {
        climb_inflow_ratio(vc_over_vh)
    } else if vc_over_vh <= -2.0 {
        windmill_brake_inflow_ratio(vc_over_vh)
    } else {
        vortex_ring_inflow_ratio(vc_over_vh)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The windmill-brake closed form must satisfy its momentum quadratic
    /// `(v_i/v_h)² + V̄_c (v_i/v_h) + 1 = 0` exactly — the non-circular anchor.
    #[test]
    fn windmill_satisfies_momentum_quadratic() {
        for &vc in &[-2.0, -3.0, -5.0, -10.0] {
            let vi = windmill_brake_inflow_ratio(vc);
            let resid = vi * vi + vc * vi + 1.0;
            assert!(resid.abs() < 1e-9, "V̄_c={vc}: residual {resid}");
        }
    }

    /// Climb closed form satisfies `(v_i/v_h)² + V̄_c (v_i/v_h) - 1 = 0`, and the
    /// hover limit `V̄_c → 0` gives `v_i = v_h`.
    #[test]
    fn climb_satisfies_momentum_quadratic_and_hover_limit() {
        assert!((climb_inflow_ratio(0.0) - 1.0).abs() < 1e-12);
        for &vc in &[0.0, 0.5, 2.0, 5.0] {
            let vi = climb_inflow_ratio(vc);
            let resid = vi * vi + vc * vi - 1.0;
            assert!(resid.abs() < 1e-9, "V̄_c={vc}: residual {resid}");
        }
    }

    /// The three regimes join up: the measured fit equals 1 at hover (`V̄_c=0`)
    /// and is within a few percent of the windmill closed form at the
    /// `V̄_c = -2` seam (the fit is not forced continuous, so a small jump is
    /// expected and bounded, not zero).
    #[test]
    fn regimes_are_continuous_within_tolerance() {
        assert!((vortex_ring_inflow_ratio(0.0) - 1.0).abs() < 1e-12);
        let fit = vortex_ring_inflow_ratio(-2.0);
        let mom = windmill_brake_inflow_ratio(-2.0); // = 1.0
        assert!(
            (fit - mom).abs() < 0.05,
            "seam jump {} too large",
            (fit - mom).abs()
        );
    }
}
