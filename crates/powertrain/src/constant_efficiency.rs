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
}
