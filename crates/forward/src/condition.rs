//! The forward-flight condition: advance ratio and disk angle of attack.

use helisim_rotor::Operating;

/// A forward-flight operating condition.
#[derive(Clone, Copy, Debug)]
pub struct ForwardCondition {
    /// Advance ratio `μ = V·cosα / (ΩR)` (in-plane freestream over tip speed).
    pub advance_ratio: f64,
    /// Disk (tip-path-plane) angle of attack, radians. Positive tilts the disk
    /// back; 0 is a level disk. The freestream component normal to the disk,
    /// `μ·tanα`, enters the momentum inflow.
    pub disk_aoa: f64,
}

impl ForwardCondition {
    /// From an explicit advance ratio and disk angle of attack.
    pub fn new(advance_ratio: f64, disk_aoa: f64) -> Self {
        ForwardCondition {
            advance_ratio,
            disk_aoa,
        }
    }

    /// Hover (`μ = 0`).
    pub fn hover() -> Self {
        ForwardCondition {
            advance_ratio: 0.0,
            disk_aoa: 0.0,
        }
    }

    /// From a forward speed (m/s), an operating point, rotor radius and disk
    /// tilt. `μ = V·cosα / (ΩR)`.
    pub fn from_speed(v_mps: f64, op: &Operating, radius: f64, disk_aoa: f64) -> Self {
        let mu = v_mps * disk_aoa.cos() / op.tip_speed(radius);
        ForwardCondition {
            advance_ratio: mu,
            disk_aoa,
        }
    }
}
