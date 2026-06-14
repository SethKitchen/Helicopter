//! Steady vertical autorotation: the equilibrium rate of descent.
//!
//! With the engine off the rotor shaft power is zero, so the rotor's power
//! balance in a descent (climb rate `V_c < 0`, induced velocity `v_i`, profile
//! power `P₀`) reads
//!
//! `0 = T (V_c + v_i) + P₀`.
//!
//! Writing the descent rate `V_d = -V_c > 0` and dividing by thrust,
//!
//! `V_d = v_i + P₀ / T`.
//!
//! The descent must supply both the induced power (`T v_i`) and the profile power
//! (`P₀`) the motor used to. The induced velocity itself depends on the descent
//! rate through the descent-regime curve `v_i = v_h · f(-V_d/v_h)` ([`inflow`]),
//! so the equation is implicit in `V_d`. The residual
//! `g(V_d) = V_d - v_h f(-V_d/v_h) - P₀/T` is monotone increasing across the
//! autorotative band, so we close it with the project's standard bisection.
//!
//! The equilibrium lands in the vortex-ring/turbulent-wake band (`V_d/v_h ≈ 1.8`),
//! exactly where momentum theory fails and the measured inflow fit is used — the
//! result is therefore validated against the *measured* ideal-autorotation
//! descent-rate band, not a closed form (see `tests/`).

use crate::inflow::{descent_inflow_ratio, hover_induced_velocity};
use crate::solution::AutorotationSolution;

/// Profile (drag) power `P₀ = C_{P0} ρ A (ΩR)³` with `C_{P0} = σ C_{d0} / 8`, the
/// standard mean-drag estimate. `tip_speed = ΩR` (m/s), `solidity = σ`,
/// `cd0` the blade mean profile-drag coefficient. Watts.
pub fn profile_power(rho: f64, disk_area_m2: f64, tip_speed: f64, solidity: f64, cd0: f64) -> f64 {
    let cp0 = solidity * cd0 / 8.0;
    cp0 * rho * disk_area_m2 * tip_speed.powi(3)
}

/// Solve the steady vertical-autorotation descent rate for a rotor carrying
/// `thrust_n` (= weight) at air density `rho`, disk area `disk_area_m2`, with
/// profile power `profile_power_w`. Returns the assembled [`AutorotationSolution`].
///
/// Bisection on the monotone residual `g(V_d)` over `V_d/v_h ∈ [1.0, 2.5]`, the
/// physical autorotative band (below 1 the rotor is still in powered descent,
/// above ~2 it is deep in the windmill state where `v_i → 0`).
pub fn steady_autorotation(
    thrust_n: f64,
    rho: f64,
    disk_area_m2: f64,
    profile_power_w: f64,
) -> AutorotationSolution {
    let v_h = hover_induced_velocity(thrust_n, rho, disk_area_m2);
    let p_over_t = profile_power_w / thrust_n; // velocity units, m/s

    // Residual in the *ratio* d = V_d / v_h: g(d) = d - f(-d) - (P₀/T)/v_h.
    let p_norm = p_over_t / v_h;
    let g = |d: f64| d - descent_inflow_ratio(-d) - p_norm;

    // Bracket V_d/v_h. g(1) = 1 - f(-1) - p_norm < 0 always (the lower bound is
    // safe), and g → ∞ as d → ∞ (deep windmill, f → 0), so we *grow* the upper
    // bracket until it brackets the root rather than fixing a ceiling — a draggy
    // rotor (small model, high C_d0) sits arbitrarily deep in the windmill-brake
    // state, and a fixed ceiling would silently clamp it.
    let mut lo = 1.0_f64;
    let mut hi = 2.5_f64;
    while g(hi) < 0.0 && hi < 1.0e6 {
        hi *= 1.5;
    }
    // g is increasing across the band; bracket then bisect.
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        let gm = g(mid);
        if gm.abs() < 1e-12 || (hi - lo) < 1e-12 {
            lo = mid;
            hi = mid;
            break;
        }
        if gm < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let d = 0.5 * (lo + hi);
    let descent_rate = d * v_h;
    let induced_velocity = v_h * descent_inflow_ratio(-d);

    AutorotationSolution {
        descent_rate_ms: descent_rate,
        descent_ratio: d,
        hover_induced_velocity_ms: v_h,
        induced_velocity_ms: induced_velocity,
        profile_power_w,
        thrust_n,
    }
}
