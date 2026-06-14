//! Forward-flight autorotation: the glide polar and the speeds to fly.
//!
//! Vertical autorotation ([`crate::descent`]) is the worst case — all the descent
//! energy must be made by a rotor in its own draggy downwash, so the rate is high.
//! Flying *forward* in autorotation is far gentler and is the case a pilot
//! actually uses: at airspeed `V` the rotor needs the level-flight power
//!
//! `P_req(V) = T v_i(V) + P₀ + ½ ρ f V³`   (induced + profile + parasite),
//!
//! and with the engine off that power is supplied by losing height, so the rate
//! of descent is `RoD(V) = P_req(V) / W`. Plotting `RoD` against `V` is the
//! **glide polar**; it has the same bucket shape as the powered power curve,
//! because it *is* that curve divided by weight. Two operating speeds fall out:
//!
//! * **Minimum-sink speed** — the bottom of the bucket, `min_V RoD(V)`: stay up
//!   the longest, the speed to hold while sorting out a forced landing.
//! * **Best-glide speed** — `min_V RoD(V)/V`, the shallowest glide angle (the
//!   tangent from the origin to the polar): cover the most ground, the speed to
//!   reach a landing site.
//!
//! The forward induced velocity uses the dimensional Glauert relation
//! `v_i = v_h² / √(V² + v_i²)` — the same momentum inflow as
//! [`helisim_forward::inflow`], written dimensionally. The profile power `P₀` is
//! the fixed-RPM mean-drag estimate from [`crate::descent::profile_power`]; the
//! parasite term uses an equivalent flat-plate area `f`.
//!
//! ⚠ Power-derived: like every forward power figure in this project, `RoD` here
//! depends on the profile/parasite estimates and (through `P₀`) the same modelling
//! the powered power curve uses — it is the realistic *shape* and the right speed
//! *ordering*, not an independently-calibrated absolute. Force-balance quantities
//! (descent ratio, `v_h`) carry no such caveat.

use crate::inflow::hover_induced_velocity;

/// Forward induced velocity `v_i` (m/s) at airspeed `V`, the positive root of the
/// dimensional Glauert relation `v_i = v_h² / √(V² + v_i²)`. Bisection on the
/// monotone residual over `[0, v_h]` (the project's standard 1-D root shape).
pub fn forward_induced_velocity(v_h: f64, airspeed: f64) -> f64 {
    let vh2 = v_h * v_h;
    let g = |vi: f64| vi - vh2 / (airspeed * airspeed + vi * vi).sqrt();
    let (mut lo, mut hi) = (0.0_f64, v_h);
    for _ in 0..200 {
        let mid = 0.5 * (lo + hi);
        if g(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
        if (hi - lo) < 1e-12 {
            break;
        }
    }
    0.5 * (lo + hi)
}

/// Level-flight power required at airspeed `V` (W): induced + profile + parasite.
/// `flat_plate_area` is the equivalent parasite drag area `f` (m²).
pub fn power_required(
    weight_n: f64,
    rho: f64,
    disk_area_m2: f64,
    profile_power_w: f64,
    flat_plate_area: f64,
    airspeed: f64,
) -> f64 {
    let v_h = hover_induced_velocity(weight_n, rho, disk_area_m2);
    let v_i = forward_induced_velocity(v_h, airspeed);
    let induced = weight_n * v_i;
    let parasite = 0.5 * rho * flat_plate_area * airspeed.powi(3);
    induced + profile_power_w + parasite
}

/// Rate of descent in forward autorotation at airspeed `V` (m/s, positive down):
/// the level-flight power required, supplied by losing height.
pub fn forward_descent_rate(
    weight_n: f64,
    rho: f64,
    disk_area_m2: f64,
    profile_power_w: f64,
    flat_plate_area: f64,
    airspeed: f64,
) -> f64 {
    power_required(
        weight_n,
        rho,
        disk_area_m2,
        profile_power_w,
        flat_plate_area,
        airspeed,
    ) / weight_n
}

/// A point on the autorotation glide polar.
#[derive(Clone, Copy, Debug)]
pub struct GlidePoint {
    /// Airspeed, m/s.
    pub airspeed_ms: f64,
    /// Rate of descent, m/s (positive down).
    pub descent_rate_ms: f64,
    /// Glide angle below horizontal, degrees (`atan(RoD/V)`).
    pub glide_angle_deg: f64,
}

/// Minimum-sink and best-glide solutions extracted from a forward-autorotation
/// glide polar.
#[derive(Clone, Copy, Debug)]
pub struct GlidePolar {
    /// Speed of minimum rate of descent (stay-up-longest), m/s.
    pub min_sink: GlidePoint,
    /// Speed of shallowest glide angle (reach-furthest), m/s.
    pub best_glide: GlidePoint,
}

/// Build the glide polar over a list of airspeeds and return the minimum-sink and
/// best-glide operating points. `airspeeds` should span past the bucket minimum
/// (e.g. a few m/s up to ~2× the expected best-glide speed).
pub fn glide_polar(
    weight_n: f64,
    rho: f64,
    disk_area_m2: f64,
    profile_power_w: f64,
    flat_plate_area: f64,
    airspeeds: &[f64],
) -> GlidePolar {
    let mut min_sink = GlidePoint {
        airspeed_ms: 0.0,
        descent_rate_ms: f64::INFINITY,
        glide_angle_deg: 0.0,
    };
    let mut best_glide = GlidePoint {
        airspeed_ms: 0.0,
        descent_rate_ms: 0.0,
        glide_angle_deg: 90.0,
    };
    for &v in airspeeds {
        if v <= 0.0 {
            continue;
        }
        let rod = forward_descent_rate(
            weight_n,
            rho,
            disk_area_m2,
            profile_power_w,
            flat_plate_area,
            v,
        );
        let angle = (rod / v).atan().to_degrees();
        if rod < min_sink.descent_rate_ms {
            min_sink = GlidePoint {
                airspeed_ms: v,
                descent_rate_ms: rod,
                glide_angle_deg: angle,
            };
        }
        if angle < best_glide.glide_angle_deg {
            best_glide = GlidePoint {
                airspeed_ms: v,
                descent_rate_ms: rod,
                glide_angle_deg: angle,
            };
        }
    }
    GlidePolar {
        min_sink,
        best_glide,
    }
}
