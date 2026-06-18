//! Flight-envelope limits — the boundaries a "best" design must respect, expressed
//! as closed-form conditions so they can serve as optimizer constraints.
//!
//! Four limits, each with an analytic oracle:
//! * **Advancing-tip Mach** (`Vne`, high-speed side): the advancing tip sees
//!   `V_tip + V`; compressibility caps it at `M_limit`, so `Vne = a·M_limit − V_tip`.
//! * **Retreating-blade stall** (`Vne`, the other side): the retreating tip flies at
//!   `(1−μ)V_tip`, so the section lift coefficient it must hold to carry the load
//!   grows like `1/(1−μ)²`. With the mean-lift relation `C̄_L = 6·C_T/σ`, stall onsets
//!   at `μ_stall = 1 − √(C̄_L/C_Lmax)` ⇒ `V = μ_stall·V_tip`. (A simplified boundary;
//!   the mean-lift-coefficient and retreating-tip-velocity assumptions are named, not
//!   a fitted number.)
//! * **Power-available max level speed**: the high-speed root of
//!   `P_required(V) = P_available` on the rising (parasite) branch of the bucket.
//! * **Hover ceiling**: the air density at which `P_hover(ρ) = P_available`
//!   (`P_hover ∝ ρ^{−1/2}`), with an ISA inversion to a density altitude.
//!
//! Validated in `tests/envelope_validation.rs` against each closed form and the
//! correct monotonic trends.

use crate::mission_profile::AircraftPower;

/// ISA sea-level density, kg/m³.
const RHO0: f64 = 1.225;
/// ISA sea-level temperature, K.
const T0: f64 = 288.15;
/// ISA tropospheric lapse rate, K/m.
const LAPSE: f64 = 0.0065;
/// `g/(L·R) − 1`, the ISA density-altitude exponent (R = 287.05 J/kg·K).
const ISA_EXP: f64 = 4.255_876;

/// Atmosphere + airframe limits that bound the envelope.
#[derive(Clone, Copy, Debug)]
pub struct EnvelopeLimits {
    /// Speed of sound, m/s.
    pub sound_speed_mps: f64,
    /// Advancing-tip Mach ceiling (drag-divergence / compressibility), e.g. 0.9.
    pub advancing_tip_mach_limit: f64,
    /// Section maximum lift coefficient for the retreating-stall boundary.
    pub cl_max: f64,
    /// Installed shaft power available, W.
    pub power_available_w: f64,
}

/// The assembled flight envelope.
#[derive(Clone, Copy, Debug)]
pub struct FlightEnvelope {
    /// High-speed Vne from the advancing-tip Mach limit, m/s.
    pub vne_advancing_mach_mps: f64,
    /// Vne from retreating-blade stall, m/s.
    pub vne_retreating_stall_mps: f64,
    /// Aerodynamic Vne = the lower of the two, m/s.
    pub vne_mps: f64,
    /// Power-limited maximum level speed, m/s.
    pub max_level_speed_mps: f64,
    /// Usable speed limit = min(aerodynamic Vne, power-limited max speed), m/s.
    pub speed_limit_mps: f64,
    /// Hover excess-power climb rate at the model's density, m/s.
    pub hover_climb_rate_mps: f64,
    /// Limiting air density for hover at the available power, kg/m³.
    pub hover_ceiling_density: f64,
    /// Hover ceiling as an ISA density altitude, m.
    pub hover_ceiling_altitude_m: f64,
}

/// Advancing-tip Vne: `a·M_limit − V_tip` (the airspeed at which the advancing tip
/// reaches the Mach limit). Clamped at 0 (a too-fast tip has no margin).
pub fn advancing_tip_vne(tip_speed_mps: f64, sound_speed_mps: f64, mach_limit: f64) -> f64 {
    (sound_speed_mps * mach_limit - tip_speed_mps).max(0.0)
}

/// Retreating-stall advance ratio `μ_stall = 1 − √(C̄_L/C_Lmax)`, `C̄_L = 6·C_T/σ`.
/// Returns 0 if the blade is already at/over stall in hover (`C̄_L ≥ C_Lmax`).
pub fn retreating_stall_mu(ct_over_sigma: f64, cl_max: f64) -> f64 {
    let cl_bar = 6.0 * ct_over_sigma;
    if cl_bar >= cl_max {
        return 0.0;
    }
    1.0 - (cl_bar / cl_max).sqrt()
}

/// ISA tropospheric density altitude for an air density, m (0 at sea level,
/// increasing as density falls). `h = (T0/L)·(1 − (ρ/ρ0)^{1/exp})`.
pub fn isa_density_altitude_m(rho: f64) -> f64 {
    (T0 / LAPSE) * (1.0 - (rho / RHO0).powf(1.0 / ISA_EXP))
}

/// Assemble the envelope for an aircraft turning a rotor of `tip_speed_mps` and
/// `solidity`, under `limits`.
pub fn analyze_envelope(
    power: &AircraftPower,
    tip_speed_mps: f64,
    solidity: f64,
    limits: &EnvelopeLimits,
) -> FlightEnvelope {
    let w = power.weight_n();

    // Thrust coefficient and blade loading at the operating density.
    let ct = w / (power.rho * power.disk_area_m2 * tip_speed_mps * tip_speed_mps);
    let ct_over_sigma = ct / solidity;

    let vne_adv = advancing_tip_vne(
        tip_speed_mps,
        limits.sound_speed_mps,
        limits.advancing_tip_mach_limit,
    );
    let mu_stall = retreating_stall_mu(ct_over_sigma, limits.cl_max);
    let vne_ret = mu_stall * tip_speed_mps;
    let vne = vne_adv.min(vne_ret);

    let max_level = max_level_speed(power, limits.power_available_w, vne.max(1.0));
    let speed_limit = vne.min(max_level);

    let p_hover = power.hover_shaft_power_w();
    let climb_rate = (limits.power_available_w - p_hover) / w;

    // Hover ceiling: ρ where P_hover(ρ) = P_avail. P_hover = W^{3/2}/(√(2A)·FM)·ρ^{-1/2}.
    let k = w.powf(1.5) / ((2.0 * power.disk_area_m2).sqrt() * power.figure_of_merit);
    let rho_ceiling = (k / limits.power_available_w).powi(2);

    FlightEnvelope {
        vne_advancing_mach_mps: vne_adv,
        vne_retreating_stall_mps: vne_ret,
        vne_mps: vne,
        max_level_speed_mps: max_level,
        speed_limit_mps: speed_limit,
        hover_climb_rate_mps: climb_rate,
        hover_ceiling_density: rho_ceiling,
        hover_ceiling_altitude_m: isa_density_altitude_m(rho_ceiling),
    }
}

/// Largest level airspeed in `(v_minpower, v_hi]` whose required power ≤ `p_avail`,
/// found by bisection on the rising (parasite) branch of the power bucket. If even
/// the minimum-power speed exceeds `p_avail` the rotor cannot sustain level flight →
/// returns 0. If `v_hi` is still within power, returns `v_hi` (power-unlimited there).
pub fn max_level_speed(power: &AircraftPower, p_avail: f64, v_hi: f64) -> f64 {
    let v_min = power.min_power_speed_mps(2.0, v_hi.max(4.0));
    if power.forward_shaft_power_w(v_min) > p_avail {
        return 0.0; // cannot fly level at any speed
    }
    if power.forward_shaft_power_w(v_hi) <= p_avail {
        return v_hi; // not power-limited within the search ceiling
    }
    // Root of P_req − p_avail on [v_min, v_hi] (P_req increasing there).
    let mut a = v_min;
    let mut b = v_hi;
    for _ in 0..100 {
        let mid = 0.5 * (a + b);
        if power.forward_shaft_power_w(mid) < p_avail {
            a = mid;
        } else {
            b = mid;
        }
        if (b - a) < 1e-6 {
            break;
        }
    }
    0.5 * (a + b)
}
