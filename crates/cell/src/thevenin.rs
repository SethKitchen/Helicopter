//! First-order Thévenin cell: an OCV-SoC lookup curve plus a constant series
//! resistance. The two parameters (the OCV curve and `r_internal`) are exactly
//! what get fitted to a datasheet; everything else is predicted.

use crate::cell::Cell;

/// Thévenin (Rint) cell model. `ocv_curve` is `(soc, ocv)` sorted by ascending
/// `soc`; OCV is linearly interpolated between points and held at the ends.
#[derive(Clone, Debug)]
pub struct TheveninCell {
    ocv_curve: Vec<(f64, f64)>,
    r_internal: f64,
    capacity_ah: f64,
    nominal_voltage: f64,
    cutoff_voltage: f64,
    max_continuous_current: f64,
    mass_kg: f64,
}

impl TheveninCell {
    /// Construct from explicit parameters. `ocv_curve` rows are `(soc, ocv)` and
    /// are sorted internally.
    pub fn new(
        ocv_curve: &[(f64, f64)],
        r_internal: f64,
        capacity_ah: f64,
        nominal_voltage: f64,
        cutoff_voltage: f64,
        max_continuous_current: f64,
        mass_kg: f64,
    ) -> Self {
        let mut ocv_curve = ocv_curve.to_vec();
        ocv_curve.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
        TheveninCell {
            ocv_curve,
            r_internal,
            capacity_ah,
            nominal_voltage,
            cutoff_voltage,
            max_continuous_current,
            mass_kg,
        }
    }

    /// Samsung INR18650-25R — the validated oracle cell.
    ///
    /// Source: Samsung SDI INR18650-25R datasheet + independent capacity tests.
    /// Nominal 2500 mAh (0.2C), 3.6 V nominal, 4.20 V full, 2.5 V cut-off, 20 A
    /// (8C) continuous, 45 g, measured DC internal resistance ≈ 14.8 mΩ.
    ///
    /// The OCV-SoC curve is a representative 18650 NMC/LiCoO2 profile anchored to
    /// the 25R's measured endpoints (4.20 V full, 2.50 V empty) and 3.6 V nominal
    /// — i.e. fitted to the (near-OCV) low-rate behaviour, with a ~3.6 V mean.
    /// `r_internal` is fitted so the predicted 20 A discharge energy matches the
    /// datasheet's 7.83 Wh; it lands at ~21 mΩ — above the instantaneous DC IR
    /// (14.8 mΩ) and 1 kHz AC impedance (≤18 mΩ), the excess being the
    /// sustained-load polarisation the single-R model lumps in. Capacity at 5 A
    /// and 10 A is then *predicted*, not fitted (see the cell discharge tests).
    pub fn samsung_25r() -> Self {
        let ocv = [
            (0.00, 2.50),
            (0.05, 3.10),
            (0.10, 3.30),
            (0.20, 3.45),
            (0.30, 3.52),
            (0.40, 3.58),
            (0.50, 3.64),
            (0.60, 3.72),
            (0.70, 3.82),
            (0.80, 3.93),
            (0.90, 4.05),
            (1.00, 4.20),
        ];
        TheveninCell::new(&ocv, 0.021, 2.5, 3.6, 2.5, 20.0, 0.045)
    }
}

impl Cell for TheveninCell {
    fn ocv(&self, soc: f64) -> f64 {
        let c = &self.ocv_curve;
        if soc <= c[0].0 {
            return c[0].1;
        }
        let last = c[c.len() - 1];
        if soc >= last.0 {
            return last.1;
        }
        let i = c.partition_point(|p| p.0 < soc);
        let (s0, v0) = c[i - 1];
        let (s1, v1) = c[i];
        v0 + (v1 - v0) * (soc - s0) / (s1 - s0)
    }

    fn internal_resistance(&self, _soc: f64) -> f64 {
        self.r_internal
    }

    fn capacity_ah(&self) -> f64 {
        self.capacity_ah
    }
    fn nominal_voltage(&self) -> f64 {
        self.nominal_voltage
    }
    fn cutoff_voltage(&self) -> f64 {
        self.cutoff_voltage
    }
    fn max_continuous_current(&self) -> f64 {
        self.max_continuous_current
    }
    fn mass_kg(&self) -> f64 {
        self.mass_kg
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ocv_endpoints_and_monotonic() {
        let c = TheveninCell::samsung_25r();
        assert!((c.ocv(1.0) - 4.20).abs() < 1e-9);
        assert!((c.ocv(0.0) - 2.50).abs() < 1e-9);
        // Monotonic increasing in SoC.
        let mut prev = -1.0;
        for k in 0..=100 {
            let v = c.ocv(k as f64 / 100.0);
            assert!(v >= prev - 1e-9);
            prev = v;
        }
    }

    #[test]
    fn terminal_voltage_sags_under_load() {
        let c = TheveninCell::samsung_25r();
        assert!(c.terminal_voltage(0.5, 20.0) < c.ocv(0.5));
        let expected_sag = 20.0 * c.internal_resistance(0.5);
        assert!((c.ocv(0.5) - c.terminal_voltage(0.5, 20.0) - expected_sag).abs() < 1e-9);
    }
}
