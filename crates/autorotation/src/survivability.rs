//! Can the rotor actually *flare* to a survivable landing? — the safety question
//! steady descent and stored energy only answer when combined.
//!
//! [`crate::descent`] gives the rate the aircraft arrives at the ground;
//! [`crate::index`] gives the energy stored in the rotor. Survival is the balance
//! of the two: in the final flare the pilot trades rotor rotational energy for a
//! thrust surge that arrests the descent. This module composes those two
//! *already-validated* quantities — it adds **no new dynamics** (deliberately: a
//! transient entry/flare integrator is exactly the "looks right, quietly wrong"
//! trap this project warns about, and is left as named future work). What it
//! computes is an honest **energy bound**.
//!
//! Usable flare energy is the rotor kinetic energy above the minimum controllable
//! rotor speed:
//!
//! `E_flare = ½ I (Ω₀² − Ω_min²) = ½ I Ω₀² (1 − f²)`,  `f = Ω_min/Ω₀`.
//!
//! To cushion a steady-autorotation arrival at descent rate `V_d` down to a safe
//! touchdown rate `V_safe`, the rotor must remove the descent kinetic energy
//! `½ m (V_d² − V_safe²)`. The dimensionless **flare margin**
//!
//! `M = E_flare / [½ m (V_d² − V_safe²)]`
//!
//! must exceed 1 — and comfortably so, since this bound ignores the profile drag
//! the rotor fights during the flare and any height lost to pilot reaction. It is
//! a *necessary* condition for a survivable autorotation, not a sufficient one,
//! and is named as such. The critical-hover-height estimate adds the
//! reaction-delay fall and a 1-g flare distance as transparent, documented bounds.
//!
//! Sources for the energy method: W. Johnson, *Helicopter Theory* (autorotation
//! energy balance); Prouty, *Helicopter Performance, Stability and Control*
//! (height-velocity / flare energy). The specific bound here is a transparent
//! simplification, stated as such, not a fit to a published H-V curve.

use crate::descent::steady_autorotation;
use crate::index::rotor_kinetic_energy;
use crate::G;

/// Flare-survivability assessment for a vertical autorotation (the bounding case).
#[derive(Clone, Copy, Debug)]
pub struct FlareAssessment {
    /// Steady vertical-autorotation descent rate, m/s.
    pub descent_rate_ms: f64,
    /// Usable rotor flare energy `½I(Ω₀²−Ω_min²)`, J.
    pub flare_energy_j: f64,
    /// Descent kinetic energy to remove `½m(V_d²−V_safe²)`, J.
    pub descent_ke_j: f64,
    /// Flare margin `E_flare / descent_KE` (must exceed 1; >1.5 desirable).
    pub flare_margin: f64,
    /// Whether the energy bound is met (`flare_margin ≥ 1`). Necessary, not
    /// sufficient — see module docs.
    pub can_flare: bool,
    /// Energy-bound minimum safe hover height, m: reaction-delay fall + 1-g flare.
    pub critical_hover_height_m: f64,
}

/// Assess vertical-autorotation flare survivability for a rotor.
///
/// * `weight_n` = gross weight (= hover thrust); `mass_kg` its mass.
/// * `inertia`, `omega0` = rotor polar inertia and nominal speed.
/// * `omega_min_frac` = minimum controllable rotor speed as a fraction of `omega0`
///   (e.g. 0.7 — below this the blades stall in the flare).
/// * `rho`, `disk_area_m2`, `profile_power_w` feed the steady descent rate.
/// * `reaction_delay_s` = pilot/sensor delay before the flare; `safe_touchdown_ms`
///   = acceptable touchdown descent rate.
#[allow(clippy::too_many_arguments)]
pub fn assess_vertical(
    weight_n: f64,
    mass_kg: f64,
    inertia: f64,
    omega0: f64,
    omega_min_frac: f64,
    rho: f64,
    disk_area_m2: f64,
    profile_power_w: f64,
    reaction_delay_s: f64,
    safe_touchdown_ms: f64,
) -> FlareAssessment {
    let descent = steady_autorotation(weight_n, rho, disk_area_m2, profile_power_w);
    let v_d = descent.descent_rate_ms;

    // Usable flare energy: KE at Ω₀ minus KE at the minimum controllable speed.
    let omega_min = omega_min_frac * omega0;
    let flare_energy = rotor_kinetic_energy(inertia, omega0) - rotor_kinetic_energy(inertia, omega_min);

    // Descent KE that must be removed to reach the safe touchdown rate.
    let descent_ke = 0.5 * mass_kg * (v_d * v_d - safe_touchdown_ms * safe_touchdown_ms).max(0.0);

    let flare_margin = if descent_ke > 0.0 {
        flare_energy / descent_ke
    } else {
        f64::INFINITY
    };

    // Energy-bound critical height: free-fall during the reaction delay (reaching
    // at most the steady descent rate) + a 1-g flare distance from V_d to V_safe.
    let w_delay = (G * reaction_delay_s).min(v_d);
    let h_delay = 0.5 * w_delay * reaction_delay_s; // ≈ distance accelerating to w_delay
    let h_flare = (v_d * v_d - safe_touchdown_ms * safe_touchdown_ms).max(0.0) / (2.0 * G);

    FlareAssessment {
        descent_rate_ms: v_d,
        flare_energy_j: flare_energy,
        descent_ke_j: descent_ke,
        flare_margin,
        can_flare: flare_margin >= 1.0,
        critical_hover_height_m: h_delay + h_flare,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::descent::profile_power;
    use std::f64::consts::PI;

    fn rep() -> (f64, f64, f64, f64, f64, f64, f64) {
        // Light helicopter: 1000 kg, R=4 m, V_tip=190, σ=0.07, C_d0=0.01,
        // I≈1500 kg·m². Returns (weight, mass, inertia, omega0, rho, area, p0).
        let (mass, r, rho, vt, sigma, cd0) = (1000.0, 4.0, 1.225, 190.0, 0.07, 0.010);
        let area = PI * r * r;
        let omega0 = vt / r;
        let p0 = profile_power(rho, area, vt, sigma, cd0);
        (mass * G, mass, 1500.0, omega0, rho, area, p0)
    }

    #[test]
    fn adequate_rotor_can_flare_with_margin() {
        let (w, m, i, om, rho, a, p0) = rep();
        let f = assess_vertical(w, m, i, om, 0.7, rho, a, p0, 1.0, 2.0);
        assert!(f.can_flare);
        assert!(f.flare_margin > 1.0);
        // The descent KE it removes is the steady-descent rate (composition).
        let v_d = steady_autorotation(w, rho, a, p0).descent_rate_ms;
        assert!((f.descent_rate_ms - v_d).abs() < 1e-12);
    }

    #[test]
    fn an_underspun_low_inertia_rotor_cannot_flare() {
        let (w, m, _i, om, rho, a, p0) = rep();
        // A tenth the inertia: not enough stored energy to arrest the descent.
        let f = assess_vertical(w, m, 150.0, om, 0.7, rho, a, p0, 1.0, 2.0);
        assert!(!f.can_flare);
        assert!(f.flare_margin < 1.0);
    }

    #[test]
    fn flare_margin_grows_with_inertia_and_rpm() {
        let (w, m, i, om, rho, a, p0) = rep();
        let base = assess_vertical(w, m, i, om, 0.7, rho, a, p0, 1.0, 2.0).flare_margin;
        let more_i = assess_vertical(w, m, 2.0 * i, om, 0.7, rho, a, p0, 1.0, 2.0).flare_margin;
        let more_rpm = assess_vertical(w, m, i, 1.2 * om, 0.7, rho, a, p0, 1.0, 2.0).flare_margin;
        assert!(more_i > base);
        assert!(more_rpm > base);
    }

    #[test]
    fn critical_height_grows_with_delay() {
        let (w, m, i, om, rho, a, p0) = rep();
        let quick = assess_vertical(w, m, i, om, 0.7, rho, a, p0, 0.5, 2.0).critical_hover_height_m;
        let slow = assess_vertical(w, m, i, om, 0.7, rho, a, p0, 2.0, 2.0).critical_hover_height_m;
        assert!(slow > quick);
    }
}
