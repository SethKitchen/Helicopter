//! State-of-charge estimation. The classic BMS estimator is **coulomb counting**
//! (integrate current to track charge moved) with periodic **OCV re-anchoring**
//! (when the cell rests, its terminal voltage *is* its OCV, which inverts to a
//! true SoC and corrects accumulated drift).
//!
//! Coulomb counting alone drifts (current-sensor bias integrates without bound);
//! OCV alone is unusable under load (the `I·R` sag corrupts it) and flat in the
//! mid-SoC plateau. Together they are the standard estimator — that complementary
//! pairing is the whole point of this module.

use helisim_cell::Cell;

/// Coulomb-counting SoC estimator with OCV re-anchoring. SoC is clamped to
/// `[0, 1]`.
#[derive(Clone, Debug)]
pub struct SocEstimator {
    soc: f64,
    capacity_ah: f64,
    /// Charge-acceptance (coulombic) efficiency, applied to charge current only.
    coulombic_efficiency: f64,
}

impl SocEstimator {
    /// Start at `soc0` for a cell of `capacity_ah`. Coulombic efficiency defaults
    /// to 0.99 (typical Li-ion); discharge counts at 100%.
    pub fn new(soc0: f64, capacity_ah: f64) -> Self {
        SocEstimator {
            soc: soc0.clamp(0.0, 1.0),
            capacity_ah,
            coulombic_efficiency: 0.99,
        }
    }

    /// Current best estimate of state of charge, `[0, 1]`.
    pub fn soc(&self) -> f64 {
        self.soc
    }

    /// Advance by `dt_s` seconds under `current_a` (discharge positive). Charge
    /// (negative current) is de-rated by the coulombic efficiency. ΔAh =
    /// `I·dt/3600`; ΔSoC = `ΔAh / capacity`.
    pub fn step(&mut self, current_a: f64, dt_s: f64) {
        let d_ah = current_a * dt_s / 3600.0;
        let d_soc = if current_a >= 0.0 {
            d_ah / self.capacity_ah // discharge: full coulombs leave
        } else {
            (d_ah * self.coulombic_efficiency) / self.capacity_ah // charge: some lost
        };
        self.soc = (self.soc - d_soc).clamp(0.0, 1.0);
    }

    /// Re-anchor the estimate from a **rested** terminal voltage (no load, so the
    /// terminal voltage equals OCV). Inverts the cell's monotone OCV-SoC curve by
    /// bisection. Call this opportunistically when the pack is idle; it wipes out
    /// the coulomb-counter's accumulated drift.
    pub fn reanchor_from_ocv(&mut self, v_rest: f64, cell: &dyn Cell) {
        // OCV is monotone increasing in SoC; clamp to the endpoints.
        if v_rest <= cell.ocv(0.0) {
            self.soc = 0.0;
            return;
        }
        if v_rest >= cell.ocv(1.0) {
            self.soc = 1.0;
            return;
        }
        let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if cell.ocv(mid) < v_rest {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        self.soc = 0.5 * (lo + hi);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_cell::{ampace_jp40, Cell};

    #[test]
    fn coulomb_counting_tracks_a_full_discharge() {
        let cell = ampace_jp40(); // 4.0 Ah
        let mut est = SocEstimator::new(1.0, cell.capacity_ah());
        // Discharge at 1C (4.0 A) for one hour → should land near empty.
        for _ in 0..3600 {
            est.step(4.0, 1.0);
        }
        assert!(est.soc() < 1e-6, "soc {}", est.soc());
    }

    #[test]
    fn charge_uses_coulombic_efficiency() {
        let mut est = SocEstimator::new(0.0, 4.0);
        // Charge 4.0 Ah worth of coulombs into a 4.0 Ah cell; with 99% efficiency
        // the SoC lands just below 1.0 (not exactly full).
        for _ in 0..3600 {
            est.step(-4.0, 1.0);
        }
        assert!(est.soc() > 0.985 && est.soc() < 1.0, "soc {}", est.soc());
    }

    #[test]
    fn ocv_reanchor_corrects_drift() {
        let cell = ampace_jp40();
        // Truth is 0.6; seed the estimator wrong at 0.4 (simulated sensor drift).
        let mut est = SocEstimator::new(0.4, cell.capacity_ah());
        let v_true = cell.ocv(0.6);
        est.reanchor_from_ocv(v_true, &cell);
        assert!((est.soc() - 0.6).abs() < 1e-3, "soc {}", est.soc());
    }

    #[test]
    fn reanchor_clamps_at_endpoints() {
        let cell = ampace_jp40();
        let mut est = SocEstimator::new(0.5, cell.capacity_ah());
        est.reanchor_from_ocv(2.0, &cell); // below empty
        assert_eq!(est.soc(), 0.0);
        est.reanchor_from_ocv(5.0, &cell); // above full
        assert_eq!(est.soc(), 1.0);
    }
}
