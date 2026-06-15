//! Height-velocity ("dead man's curve") — the unsafe height/airspeed region for a
//! power failure, by the energy method.
//!
//! The H-V diagram marks combinations of height above ground and airspeed from
//! which a safe autorotative landing is not possible after a power loss. Its
//! classic **low-speed lobe** is the one that matters most for a hovering or
//! slow aircraft: from a hover you need a minimum height to establish autorotation
//! and flare, and below that critical height you cannot. Forward speed shrinks the
//! lobe, because the kinetic energy you already carry substitutes for height.
//!
//! This is built as an **energy-method boundary**, deliberately NOT a transient
//! entry/flare simulation: that simulation's height-loss is exactly the
//! "looks right, quietly wrong" integrator this project warns about, and it would
//! have no independent oracle for the transient. Instead the boundary is anchored
//! to the *already-validated* vertical critical height
//! ([`crate::survivability::assess_vertical`]) and uses the clean energy
//! equivalence that a forward speed `V` is worth a height `V²/2g`:
//!
//! `h_crit(V) = max(0, h_crit_hover − V²/2g)`.
//!
//! There is **no free parameter** — the curve passes through the validated vertical
//! critical height at `V=0` and closes (lobe vanishes) at the knee speed
//! `V_knee = √(2 g h_crit_hover)`. It is a first-order envelope, named as such.
//!
//! ## Deliberately deferred (named, not faked)
//! The **high-speed / low-altitude lobe** (too fast and low to flare before
//! ground strike) depends on the dynamic flare timing and is left to a future
//! transient flare model — it is *not* approximated here, because a credible value
//! needs the entry/flare dynamics this crate intentionally omits.

use crate::G;
use crate::survivability::{FlareParams, assess_vertical};

/// Low-speed critical height at airspeed `V`: the minimum height for a survivable
/// power-loss recovery, `max(0, h_crit_hover − V²/2g)`.
pub fn low_speed_critical_height(h_crit_hover: f64, airspeed: f64) -> f64 {
    (h_crit_hover - airspeed * airspeed / (2.0 * G)).max(0.0)
}

/// Knee speed at which the low-speed lobe closes: `√(2 g h_crit_hover)`.
pub fn knee_speed(h_crit_hover: f64) -> f64 {
    (2.0 * G * h_crit_hover).sqrt()
}

/// One sampled point of the low-speed H-V boundary.
#[derive(Clone, Copy, Debug)]
pub struct HvPoint {
    /// Airspeed, m/s.
    pub airspeed_ms: f64,
    /// Critical height at this airspeed, m (below = unsafe).
    pub critical_height_m: f64,
}

/// The low-speed height-velocity boundary plus its anchors.
#[derive(Clone, Debug)]
pub struct HeightVelocityDiagram {
    /// The vertical critical height (V=0 anchor), m.
    pub critical_hover_height_m: f64,
    /// Knee speed where the low-speed lobe closes, m/s.
    pub knee_speed_ms: f64,
    /// Sampled boundary points (airspeed, critical height).
    pub boundary: Vec<HvPoint>,
}

impl HeightVelocityDiagram {
    /// Whether a `(height, airspeed)` state is OUTSIDE the low-speed avoid lobe
    /// (i.e. survivable per this energy bound). A state is unsafe if it is below
    /// the critical height at its airspeed.
    pub fn is_safe_low_speed(&self, height_m: f64, airspeed_ms: f64) -> bool {
        height_m >= low_speed_critical_height(self.critical_hover_height_m, airspeed_ms)
    }
}

/// Build the low-speed H-V boundary for a rotor, anchored to its validated
/// vertical critical height. `airspeeds` samples the curve (m/s). The remaining
/// arguments mirror [`assess_vertical`].
pub fn build_low_speed_hv(p: &FlareParams, airspeeds: &[f64]) -> HeightVelocityDiagram {
    let h_hover = assess_vertical(p).critical_hover_height_m;

    let boundary = airspeeds
        .iter()
        .map(|&v| HvPoint {
            airspeed_ms: v,
            critical_height_m: low_speed_critical_height(h_hover, v),
        })
        .collect();

    HeightVelocityDiagram {
        critical_hover_height_m: h_hover,
        knee_speed_ms: knee_speed(h_hover),
        boundary,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::descent::profile_power;
    use std::f64::consts::PI;

    fn rep_hv() -> HeightVelocityDiagram {
        // Light helicopter (matches the survivability tests).
        let (mass, r, rho, vt, sigma, cd0) = (1000.0, 4.0, 1.225, 190.0, 0.07, 0.010);
        let area = PI * r * r;
        let omega0 = vt / r;
        let p0 = profile_power(rho, area, vt, sigma, cd0);
        let speeds: Vec<f64> = (0..=40).map(|i| i as f64).collect();
        build_low_speed_hv(
            &FlareParams {
                weight_n: mass * G,
                mass_kg: mass,
                inertia: 1500.0,
                omega0,
                omega_min_frac: 0.7,
                rho,
                disk_area_m2: area,
                profile_power_w: p0,
                reaction_delay_s: 1.0,
                safe_touchdown_ms: 2.0,
            },
            &speeds,
        )
    }

    #[test]
    fn hover_axis_equals_validated_critical_height() {
        // The curve passes exactly through the validated vertical critical height.
        let hv = rep_hv();
        let at_zero = low_speed_critical_height(hv.critical_hover_height_m, 0.0);
        assert!((at_zero - hv.critical_hover_height_m).abs() < 1e-12);
        assert_eq!(hv.boundary[0].airspeed_ms, 0.0);
        assert!((hv.boundary[0].critical_height_m - hv.critical_hover_height_m).abs() < 1e-12);
    }

    #[test]
    fn lobe_closes_at_the_knee_speed_with_no_free_parameter() {
        let hv = rep_hv();
        // At the knee speed the critical height is driven to zero.
        let at_knee = low_speed_critical_height(hv.critical_hover_height_m, hv.knee_speed_ms);
        assert!(at_knee < 1e-9, "critical height at knee = {at_knee}");
        // Knee = √(2 g h).
        assert!((hv.knee_speed_ms - (2.0 * G * hv.critical_hover_height_m).sqrt()).abs() < 1e-9);
    }

    #[test]
    fn critical_height_decreases_with_airspeed() {
        let hv = rep_hv();
        for w in hv.boundary.windows(2) {
            assert!(w[1].critical_height_m <= w[0].critical_height_m);
        }
    }

    #[test]
    fn safe_query_classifies_the_avoid_lobe() {
        let hv = rep_hv();
        let h = hv.critical_hover_height_m;
        // Low and slow, below the critical height → unsafe.
        assert!(!hv.is_safe_low_speed(0.3 * h, 1.0));
        // High above the curve → safe.
        assert!(hv.is_safe_low_speed(2.0 * h, 1.0));
        // Fast enough that the lobe has closed → safe even at low height.
        assert!(hv.is_safe_low_speed(0.1, hv.knee_speed_ms + 1.0));
    }
}
