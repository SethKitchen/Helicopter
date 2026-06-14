//! Flap hinge and blade-inertia properties.

/// Flapping properties: the Lock number, the flap-hinge offset, and the
/// sectional lift-curve slope (the latter only needed to recover the blade flap
/// inertia from the Lock number for the dimensional hub moment).
#[derive(Clone, Copy, Debug)]
pub struct FlapProperties {
    /// Lock number `γ = ρ a c R⁴ / I_β` (typically 6–12 for articulated rotors).
    pub lock_number: f64,
    /// Flap-hinge offset as a fraction of radius (0 = central hinge).
    pub hinge_offset: f64,
    /// Sectional lift-curve slope, per radian (folds into `γ`; default 5.73).
    pub lift_slope: f64,
    /// Gyroscopic ("rotor-follows-shaft") hub-rate→flap coupling coefficient. The
    /// inertial precession term that supplies the in-phase flap response to body
    /// pitch/roll rate, hence the rotor's contribution to rate damping (Mq, Lp).
    /// **Default 0** (the pre-Milestone-6 behaviour, so every prior milestone's
    /// validated dynamics are unchanged). The physically-correct, externally-
    /// validated value is **−2** (see `validation/MILESTONE6_FLAP_FIX_PREREG.md`):
    /// magnitude 2 is textbook, sign mandated by gyroscopic damping. Set it on real
    /// aircraft; adopting −2 as the universal default requires revalidating the
    /// 5c–5m control stack (its own step), so it stays opt-in for now.
    pub gyro_rate: f64,
}

impl FlapProperties {
    /// A central-hinge articulated rotor (`ν_β = 1`).
    pub fn articulated(lock_number: f64) -> Self {
        FlapProperties {
            lock_number,
            hinge_offset: 0.0,
            lift_slope: 5.73,
            gyro_rate: 0.0,
        }
    }

    /// An articulated rotor with a flap-hinge offset (fraction of radius).
    pub fn with_offset(lock_number: f64, hinge_offset: f64) -> Self {
        FlapProperties {
            lock_number,
            hinge_offset,
            lift_slope: 5.73,
            gyro_rate: 0.0,
        }
    }

    /// Rotating flap frequency squared `ν_β²`. Central hinge → 1; offset stiffens
    /// it: `ν_β² = 1 + (3/2)·e/(1−e)` (Leishman).
    pub fn nu_beta_sq(&self) -> f64 {
        let e = self.hinge_offset;
        1.0 + 1.5 * e / (1.0 - e)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn central_hinge_is_resonant() {
        assert!((FlapProperties::articulated(8.0).nu_beta_sq() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn offset_stiffens() {
        assert!(FlapProperties::with_offset(8.0, 0.05).nu_beta_sq() > 1.0);
    }
}
