//! Protection envelope — the BMS's primary safety job: keep every cell inside its
//! safe **voltage / current / temperature** window, and name the fault when it
//! leaves. This is the layer the bare [`Cell`]/[`Pack`] models don't have: they
//! describe what a cell *does*, not the limits a controller must enforce.
//!
//! Topology-agnostic by construction — the checks are per-cell, so the same
//! [`ProtectionLimits`] guards a 6S model pack or a 96S human-scale pack; only the
//! cell count differs.

use helisim_cell::Cell;

/// A protection trip, in priority order (temperature is the most dangerous, so it
/// is reported first when several conditions trip at once).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    /// Inside the safe envelope.
    None,
    /// Cell temperature above the limit.
    OverTemperature,
    /// Cell terminal voltage above the charge ceiling.
    OverVoltage,
    /// Cell terminal voltage below the discharge cutoff.
    UnderVoltage,
    /// Cell current magnitude above the continuous limit.
    OverCurrent,
}

/// Per-cell safe-operating limits. Built from a cell's own datasheet window plus
/// two integrator-set numbers: the **true** continuous current (the de-rated,
/// not the label, rating — see `helisim_cell::true_continuous_current`) and the
/// temperature ceiling.
#[derive(Clone, Copy, Debug)]
pub struct ProtectionLimits {
    /// Discharge cutoff voltage, V (lower bound).
    pub v_min: f64,
    /// Charge ceiling voltage, V (upper bound).
    pub v_max: f64,
    /// Continuous current limit (magnitude), A — set to the TRUE continuous
    /// rating, not the thermal-cutoff-limited datasheet label.
    pub i_continuous: f64,
    /// Cell temperature ceiling, °C.
    pub t_max: f64,
}

impl ProtectionLimits {
    /// Build from a cell: `v_min` = its discharge cutoff, `v_max` = its full-charge
    /// OCV (`ocv(1.0)`). The caller supplies the true continuous current limit and
    /// the temperature ceiling (both safety policy, not raw cell data).
    pub fn from_cell(cell: &dyn Cell, i_continuous: f64, t_max: f64) -> Self {
        ProtectionLimits {
            v_min: cell.cutoff_voltage(),
            v_max: cell.ocv(1.0),
            i_continuous,
            t_max,
        }
    }

    /// Classify a cell's state. `current` is discharge-positive; magnitude is
    /// checked so charge over-current trips too. Priority: temperature → over-volt
    /// → under-volt → over-current → none.
    pub fn check(&self, cell_voltage: f64, cell_current: f64, cell_temp_c: f64) -> Fault {
        if cell_temp_c > self.t_max {
            Fault::OverTemperature
        } else if cell_voltage > self.v_max {
            Fault::OverVoltage
        } else if cell_voltage < self.v_min {
            Fault::UnderVoltage
        } else if cell_current.abs() > self.i_continuous {
            Fault::OverCurrent
        } else {
            Fault::None
        }
    }

    /// Fraction of the continuous current limit a draw uses, `|I|/I_cont`
    /// (≥1.0 means a trip). The headroom the benchmark reports as "C-rate margin".
    pub fn current_utilisation(&self, cell_current: f64) -> f64 {
        cell_current.abs() / self.i_continuous
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_cell::{ampace_jp40, true_continuous_current};

    fn jp40_limits() -> ProtectionLimits {
        let cell = ampace_jp40();
        ProtectionLimits::from_cell(&cell, true_continuous_current("Ampace JP40").unwrap(), 60.0)
    }

    #[test]
    fn window_taken_from_cell() {
        let l = jp40_limits();
        assert!((l.v_min - 2.5).abs() < 1e-9);
        assert!((l.v_max - 4.2).abs() < 1e-9);
        assert!((l.i_continuous - 45.0).abs() < 1e-9);
    }

    #[test]
    fn each_fault_triggers() {
        let l = jp40_limits();
        // Nominal mid-pack point — safe.
        assert_eq!(l.check(3.6, 30.0, 25.0), Fault::None);
        // Below cutoff.
        assert_eq!(l.check(2.4, 10.0, 25.0), Fault::UnderVoltage);
        // Above ceiling.
        assert_eq!(l.check(4.25, 10.0, 25.0), Fault::OverVoltage);
        // Over the 45 A true-continuous limit (the label says 60 A — the BMS
        // enforces the honest number).
        assert_eq!(l.check(3.6, 50.0, 25.0), Fault::OverCurrent);
        // Charge over-current (negative current, magnitude checked).
        assert_eq!(l.check(3.6, -50.0, 25.0), Fault::OverCurrent);
        // Temperature dominates when several trip at once.
        assert_eq!(l.check(2.4, 50.0, 70.0), Fault::OverTemperature);
    }

    #[test]
    fn utilisation_is_unit_at_the_limit() {
        let l = jp40_limits();
        assert!((l.current_utilisation(45.0) - 1.0).abs() < 1e-9);
        assert!((l.current_utilisation(22.5) - 0.5).abs() < 1e-9);
    }
}
