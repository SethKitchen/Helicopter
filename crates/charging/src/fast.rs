//! DC fast charging — a charger that supplies high-power DC directly, bypassing
//! the residential AC branch limit. This is the only source class that can
//! approach *flight power*, which is what a ~1:1 charge-to-flight ratio requires
//! (see [`crate::ratio`]).
//!
//! Rated by DC output power (the EV-charger convention: 50 / 150 / 350 kW classes).
//! Grid→DC conversion is ~94 % efficient (representative). Whether the *pack* can
//! absorb this is a separate question answered by the cell charge ceiling, not the
//! charger.

use crate::source::ChargeSource;

/// A DC fast charger rated by its DC output power.
#[derive(Clone, Copy, Debug)]
pub struct DcFastCharger {
    /// DC output power, kW.
    pub dc_power_kw: f64,
    /// Grid→DC conversion efficiency.
    pub efficiency: f64,
}

impl DcFastCharger {
    /// A charger of `dc_power_kw` DC output (≈94 % grid→DC).
    pub fn new(dc_power_kw: f64) -> Self {
        DcFastCharger {
            dc_power_kw,
            efficiency: 0.94,
        }
    }

    /// 50 kW DC (a common public fast-charge tier).
    pub fn dc_50kw() -> Self {
        Self::new(50.0)
    }
    /// 150 kW DC (mid-tier fast charge).
    pub fn dc_150kw() -> Self {
        Self::new(150.0)
    }
    /// 350 kW DC (high-power fast charge / "ultra-rapid").
    pub fn dc_350kw() -> Self {
        Self::new(350.0)
    }
}

impl ChargeSource for DcFastCharger {
    fn dc_power_w(&self) -> f64 {
        self.dc_power_kw * 1000.0
    }

    fn label(&self) -> String {
        format!("{:.0} kW DC fast charger", self.dc_power_kw)
    }

    fn input_energy_wh(&self, delivered_wh: f64) -> f64 {
        delivered_wh / self.efficiency
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dc_output_is_rated_power() {
        assert!((DcFastCharger::dc_150kw().dc_power_w() - 150_000.0).abs() < 1e-6);
    }

    #[test]
    fn grid_energy_exceeds_delivered() {
        let c = DcFastCharger::dc_50kw();
        assert!(c.input_energy_wh(1000.0) > 1000.0);
    }

    #[test]
    fn tiers_increase() {
        assert!(DcFastCharger::dc_50kw().dc_power_w() < DcFastCharger::dc_150kw().dc_power_w());
        assert!(DcFastCharger::dc_150kw().dc_power_w() < DcFastCharger::dc_350kw().dc_power_w());
    }
}
