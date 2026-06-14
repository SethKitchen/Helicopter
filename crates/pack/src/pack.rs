//! A battery pack: `series` cells in series, `parallel` strings in parallel.
//!
//! Pack-level scaling of an identical cell model:
//! * voltage   = `S × cell`            (series adds voltage)
//! * capacity  = `P × cell`            (parallel adds capacity)
//! * resistance= `(S/P) × cell`        (series adds, parallel divides)
//! * mass / energy / current rating scale with the cell count accordingly.
//!
//! State of charge is shared across cells (a balanced-pack assumption).

use helisim_cell::Cell;

/// A series/parallel pack wrapping one cell model. Holds the cell as a boxed
/// trait object so any chemistry can be packed.
pub struct Pack {
    cell: Box<dyn Cell>,
    /// Cells in series (sets voltage).
    pub series: usize,
    /// Strings in parallel (sets capacity).
    pub parallel: usize,
}

impl Pack {
    /// Build an `series`S `parallel`P pack from a cell model.
    pub fn new(cell: Box<dyn Cell>, series: usize, parallel: usize) -> Self {
        assert!(series >= 1 && parallel >= 1, "S and P must be >= 1");
        Pack {
            cell,
            series,
            parallel,
        }
    }

    fn s(&self) -> f64 {
        self.series as f64
    }
    fn p(&self) -> f64 {
        self.parallel as f64
    }

    /// Total number of cells.
    pub fn cell_count(&self) -> usize {
        self.series * self.parallel
    }

    /// Pack open-circuit voltage at `soc`, volts.
    pub fn ocv(&self, soc: f64) -> f64 {
        self.s() * self.cell.ocv(soc)
    }

    /// Pack internal resistance at `soc`, ohms.
    pub fn internal_resistance(&self, soc: f64) -> f64 {
        (self.s() / self.p()) * self.cell.internal_resistance(soc)
    }

    /// Pack terminal voltage under a `pack_current` (A) at `soc`.
    pub fn terminal_voltage(&self, soc: f64, pack_current: f64) -> f64 {
        self.ocv(soc) - pack_current * self.internal_resistance(soc)
    }

    /// Pack capacity, amp-hours.
    pub fn capacity_ah(&self) -> f64 {
        self.p() * self.cell.capacity_ah()
    }

    /// Pack nominal voltage, volts.
    pub fn nominal_voltage(&self) -> f64 {
        self.s() * self.cell.nominal_voltage()
    }

    /// Pack discharge cut-off voltage, volts.
    pub fn cutoff_voltage(&self) -> f64 {
        self.s() * self.cell.cutoff_voltage()
    }

    /// Nominal stored energy, watt-hours.
    pub fn energy_wh(&self) -> f64 {
        self.nominal_voltage() * self.capacity_ah()
    }

    /// Pack mass, kilograms.
    pub fn mass_kg(&self) -> f64 {
        self.cell_count() as f64 * self.cell.mass_kg()
    }

    /// Maximum continuous pack current, amps.
    pub fn max_continuous_current(&self) -> f64 {
        self.p() * self.cell.max_continuous_current()
    }

    /// Per-cell current for a given pack current, amps.
    pub fn cell_current(&self, pack_current: f64) -> f64 {
        pack_current / self.p()
    }

    /// Per-cell C-rate (1/h) for a given pack current.
    pub fn cell_c_rate(&self, pack_current: f64) -> f64 {
        self.cell_current(pack_current) / self.cell.capacity_ah()
    }

    /// Cell continuous C-rate rating (1/h).
    pub fn continuous_c_rating(&self) -> f64 {
        self.cell.max_continuous_c_rate()
    }

    /// Per-cell internal resistance at `soc`, ohms (for per-cell heat).
    pub fn cell_resistance(&self, soc: f64) -> f64 {
        self.cell.internal_resistance(soc)
    }

    /// Per-cell lumped heat capacity `m·c_p`, J/K.
    pub fn cell_heat_capacity(&self) -> f64 {
        self.cell.heat_capacity()
    }

    /// Per-cell cooling surface area, m².
    pub fn cell_surface_area(&self) -> f64 {
        self.cell.surface_area()
    }

    /// Maximum power the pack can deliver into a matched load at `soc`, watts.
    pub fn max_power(&self, soc: f64) -> f64 {
        let v = self.ocv(soc);
        v * v / (4.0 * self.internal_resistance(soc))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_cell::{Cell, TheveninCell};

    fn pack_6s2p() -> Pack {
        Pack::new(Box::new(TheveninCell::samsung_25r()), 6, 2)
    }

    #[test]
    fn topology_scaling() {
        let p = pack_6s2p();
        assert_eq!(p.cell_count(), 12);
        assert!((p.nominal_voltage() - 6.0 * 3.6).abs() < 1e-9);
        assert!((p.capacity_ah() - 2.0 * 2.5).abs() < 1e-9);
        assert!((p.mass_kg() - 12.0 * 0.045).abs() < 1e-9);
        // resistance = (S/P) * r_cell = 3 * r_cell
        let r_cell = TheveninCell::samsung_25r().internal_resistance(0.5);
        assert!((p.internal_resistance(0.5) - 3.0 * r_cell).abs() < 1e-9);
    }

    #[test]
    fn cell_current_is_pack_current_over_parallel() {
        let p = pack_6s2p();
        assert!((p.cell_current(40.0) - 20.0).abs() < 1e-9);
        // 20 A per cell / 2.5 Ah = 8 C
        assert!((p.cell_c_rate(40.0) - 8.0).abs() < 1e-9);
    }

    /// DOCUMENTED EXAMPLE — a 6S2P Samsung INR18650-25R pack. The cell numbers are
    /// the published datasheet values (3.6 V nominal, 2.5 Ah, 20 A continuous,
    /// 45 g, ~21 mΩ); the pack numbers follow from the S/P scaling and a reader can
    /// check every one by hand:
    ///   V_nom = 6 × 3.6 = 21.6 V   |   Ah = 2 × 2.5 = 5.0 Ah
    ///   Wh = 21.6 × 5.0 = 108 Wh   |   R = (6/2) × 21 mΩ = 63 mΩ
    ///   mass = 12 × 45 g = 540 g   |   I_cont = 2 × 20 A = 40 A (8C)
    /// Source: Samsung SDI INR18650-25R datasheet.
    #[test]
    fn documented_6s2p_samsung_25r_pack() {
        let p = pack_6s2p();
        assert_eq!(p.cell_count(), 12);
        assert!((p.nominal_voltage() - 21.6).abs() < 1e-6, "V {}", p.nominal_voltage());
        assert!((p.capacity_ah() - 5.0).abs() < 1e-6);
        assert!((p.energy_wh() - 108.0).abs() < 1e-3, "Wh {}", p.energy_wh());
        assert!((p.internal_resistance(0.5) - 0.063).abs() < 1e-6, "R {}", p.internal_resistance(0.5));
        assert!((p.mass_kg() - 0.540).abs() < 1e-6);
        assert!((p.max_continuous_current() - 40.0).abs() < 1e-6);
    }
}
