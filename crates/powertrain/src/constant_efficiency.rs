//! Constant-efficiency powertrain: a single flat η for motor × ESC. The simplest
//! first cut; a torque/RPM efficiency map can replace it behind the trait later.

use crate::powertrain::Powertrain;

/// Flat combined motor + ESC efficiency.
#[derive(Clone, Copy, Debug)]
pub struct ConstantEfficiency {
    /// Combined efficiency in (0, 1].
    pub eta: f64,
}

impl ConstantEfficiency {
    /// Build with an explicit efficiency.
    pub fn new(eta: f64) -> Self {
        assert!(eta > 0.0 && eta <= 1.0, "efficiency must be in (0, 1]");
        ConstantEfficiency { eta }
    }

    /// Typical small electric-helicopter driveline: brushless motor (~0.85) ×
    /// ESC (~0.95) ≈ 0.80.
    pub fn typical_electric_heli() -> Self {
        ConstantEfficiency { eta: 0.80 }
    }
}

impl Powertrain for ConstantEfficiency {
    fn efficiency(&self, _mech_power: f64) -> f64 {
        self.eta
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scales_power_by_inverse_efficiency() {
        let pt = ConstantEfficiency::new(0.8);
        assert!((pt.electrical_power(800.0) - 1000.0).abs() < 1e-9);
    }

    /// DOCUMENTED EXAMPLE — the typical small-electric-heli driveline efficiency is
    /// the product of a brushless motor (~0.85 at its efficiency peak) and an ESC
    /// (~0.95, modern low-Rds(on) FETs): 0.85 × 0.95 = 0.8075 ≈ 0.80. A reader can
    /// check the power map: at η=0.80, 1000 W of shaft power needs 1250 W electrical
    /// (250 W lost as heat). Source: T-Motor U-series motor datasheets + ESC
    /// efficiency benchmarks (FET conduction loss ≈ 2–5 % of throughput).
    #[test]
    fn documented_typical_driveline_efficiency() {
        let pt = ConstantEfficiency::typical_electric_heli();
        assert!((pt.efficiency(0.0) - 0.80).abs() < 1e-9);
        // 0.85 motor × 0.95 ESC rounds to the 0.80 used.
        assert!((0.85_f64 * 0.95 - 0.8075).abs() < 1e-9);
        // 1000 W mechanical → 1250 W electrical.
        assert!((pt.electrical_power(1000.0) - 1250.0).abs() < 1e-6);
    }
}
