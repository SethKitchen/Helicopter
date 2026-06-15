//! Charging from a **solar PV array** through an MPPT charge controller.
//!
//! A panel's nameplate watts are at STC (1000 W/m², 25 °C). Real delivered DC
//! power is `count × rated × (irradiance/1000) × derate × mppt_eff`, where the
//! **derate** (≈0.80, the system performance ratio) folds in soiling, wiring,
//! temperature, mismatch and aging, and the **MPPT** controller converts at
//! ≈95–98 %. Daily energy uses **peak-sun-hours** (the day's irradiation expressed
//! as equivalent hours at 1000 W/m²; US average ≈ 4.5, range ~3.5–6).
//!
//! Sources: residential panels ≈400 W (2025 norm); MPPT 95–98 %; PR/derate
//! 0.75–0.85; peak-sun-hours 3.5–6 (US). All overridable; irradiance/derate are
//! stated assumptions (use NREL PVWatts for a site-specific figure).

use crate::source::ChargeSource;

/// STC reference irradiance, W/m².
pub const STC_IRRADIANCE: f64 = 1000.0;

/// A photovoltaic array + MPPT charge controller.
#[derive(Clone, Copy, Debug)]
pub struct SolarArray {
    /// Number of panels.
    pub panel_count: usize,
    /// Per-panel nameplate (STC) power, W.
    pub panel_rated_w: f64,
    /// System derate / performance ratio (soiling, wiring, temp, mismatch, aging).
    pub derate: f64,
    /// MPPT charge-controller conversion efficiency.
    pub mppt_eff: f64,
    /// Peak sun hours per day (equivalent hours at 1000 W/m²).
    pub peak_sun_hours: f64,
}

impl SolarArray {
    /// `panel_count` modern 400 W residential panels at US-average conditions
    /// (derate 0.80, MPPT 0.97, 4.5 peak-sun-hours).
    pub fn typical(panel_count: usize) -> Self {
        SolarArray {
            panel_count,
            panel_rated_w: 400.0,
            derate: 0.80,
            mppt_eff: 0.97,
            peak_sun_hours: 4.5,
        }
    }

    /// Array nameplate (STC) power, W.
    pub fn nameplate_w(&self) -> f64 {
        self.panel_count as f64 * self.panel_rated_w
    }

    /// Delivered DC power at an arbitrary irradiance fraction (1.0 = full sun/STC).
    pub fn power_at_irradiance(&self, irradiance_fraction: f64) -> f64 {
        self.nameplate_w() * irradiance_fraction * self.derate * self.mppt_eff
    }
}

impl ChargeSource for SolarArray {
    /// Delivered DC power at full sun (STC irradiance).
    fn dc_power_w(&self) -> f64 {
        self.power_at_irradiance(1.0)
    }

    fn label(&self) -> String {
        format!(
            "{}× {:.0} W solar ({:.1} kW nameplate, MPPT {:.0}%, derate {:.0}%, {:.1} sun-h/day)",
            self.panel_count,
            self.panel_rated_w,
            self.nameplate_w() / 1000.0,
            self.mppt_eff * 100.0,
            self.derate * 100.0,
            self.peak_sun_hours
        )
    }

    /// Solar energy per day, Wh = nameplate × peak-sun-hours × derate × MPPT.
    fn daily_energy_wh(&self) -> Option<f64> {
        Some(self.nameplate_w() * self.peak_sun_hours * self.derate * self.mppt_eff)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SOURCED formula — 4× 400 W panels: 1.6 kW nameplate; at full sun
    /// 1600 × 0.80 × 0.97 = 1241.6 W DC; daily 1600 × 4.5 × 0.80 × 0.97 ≈ 5587 Wh.
    #[test]
    fn array_power_and_daily_energy() {
        let a = SolarArray::typical(4);
        assert!((a.nameplate_w() - 1600.0).abs() < 1e-9);
        assert!(
            (a.dc_power_w() - 1241.6).abs() < 1e-3,
            "dc {}",
            a.dc_power_w()
        );
        assert!((a.daily_energy_wh().unwrap() - 5587.2).abs() < 1.0);
    }

    #[test]
    fn power_scales_with_irradiance() {
        let a = SolarArray::typical(4);
        assert!((a.power_at_irradiance(0.5) - 0.5 * a.dc_power_w()).abs() < 1e-9);
        assert!((a.power_at_irradiance(0.0)).abs() < 1e-12); // night = no power
    }

    #[test]
    fn more_panels_more_power() {
        assert!(SolarArray::typical(8).dc_power_w() > SolarArray::typical(4).dc_power_w());
    }
}
