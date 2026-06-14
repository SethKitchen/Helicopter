//! The rotor operating point: rotational speed plus the fluid environment.

use crate::{RHO_SEA_LEVEL, SPEED_OF_SOUND};
use std::f64::consts::PI;

/// Rotational speed and the surrounding fluid. Kept separate from [`crate::Rotor`]
/// so the same geometry can be evaluated at many operating points.
#[derive(Clone, Copy, Debug)]
pub struct Operating {
    /// Rotational speed, rad/s.
    pub omega: f64,
    /// Air density, kg/m^3.
    pub rho: f64,
    /// Speed of sound, m/s.
    pub sound_speed: f64,
}

impl Operating {
    /// From RPM at sea-level standard conditions.
    pub fn from_rpm(rpm: f64) -> Self {
        Operating {
            omega: rpm * 2.0 * PI / 60.0,
            rho: RHO_SEA_LEVEL,
            sound_speed: SPEED_OF_SOUND,
        }
    }

    /// The operating point that yields a desired tip Mach number for a given
    /// radius (handy for matching wind-tunnel conditions).
    pub fn from_tip_mach(tip_mach: f64, radius: f64) -> Self {
        Operating {
            omega: tip_mach * SPEED_OF_SOUND / radius,
            rho: RHO_SEA_LEVEL,
            sound_speed: SPEED_OF_SOUND,
        }
    }

    /// Tip speed `Omega * R`, m/s.
    pub fn tip_speed(&self, radius: f64) -> f64 {
        self.omega * radius
    }

    /// Tip Mach number for a given radius.
    pub fn tip_mach(&self, radius: f64) -> f64 {
        self.tip_speed(radius) / self.sound_speed
    }

    /// Rotational speed in RPM.
    pub fn rpm(&self) -> f64 {
        self.omega * 60.0 / (2.0 * PI)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rpm_roundtrip() {
        let op = Operating::from_rpm(1250.0);
        assert!((op.rpm() - 1250.0).abs() < 1e-9);
    }

    #[test]
    fn tip_mach_matches_target() {
        let op = Operating::from_tip_mach(0.439, 1.143);
        assert!((op.tip_mach(1.143) - 0.439).abs() < 1e-9);
        // 0.439 * 340 / 1.143 ~ 130.6 rad/s ~ 1247 rpm
        assert!((op.rpm() - 1247.0).abs() < 2.0);
    }
}
