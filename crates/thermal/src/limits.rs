//! Thermal safety band for a cell.

/// Where a temperature sits relative to the safe operating band.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ThermalStatus {
    /// At or below the recommended continuous limit.
    Safe,
    /// Between the recommended limit and the absolute maximum.
    Warning,
    /// Above the absolute maximum discharge temperature.
    OverTemp,
}

/// Discharge thermal limits, °C.
#[derive(Clone, Copy, Debug)]
pub struct ThermalLimits {
    /// Recommended continuous operating ceiling.
    pub warn_c: f64,
    /// Absolute maximum discharge surface temperature.
    pub max_c: f64,
}

impl Default for ThermalLimits {
    /// Samsung INR18650-25R discharge limits: recommended ≤ 60 °C, absolute
    /// maximum 75 °C (per the datasheet operating-temperature spec).
    fn default() -> Self {
        ThermalLimits {
            warn_c: 60.0,
            max_c: 75.0,
        }
    }
}

impl ThermalLimits {
    /// Classify a temperature against the band.
    pub fn classify(&self, temp_c: f64) -> ThermalStatus {
        if temp_c > self.max_c {
            ThermalStatus::OverTemp
        } else if temp_c > self.warn_c {
            ThermalStatus::Warning
        } else {
            ThermalStatus::Safe
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn classifies_bands() {
        let l = ThermalLimits::default();
        assert_eq!(l.classify(40.0), ThermalStatus::Safe);
        assert_eq!(l.classify(65.0), ThermalStatus::Warning);
        assert_eq!(l.classify(80.0), ThermalStatus::OverTemp);
    }
}
