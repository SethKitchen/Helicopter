//! Life-aware pack sizing — fold a target service life into the pack capacity.
//!
//! The minimum (energy-only) pack flies the mission but, cycled deeply every day,
//! wears out in ~1–2 years. To last `life_years` at `flights_per_year` you size the
//! pack BIGGER so each flight uses a shallower depth-of-discharge (and a bigger
//! pack also lowers the flight C-rate) — both reduce cycle fade. This finds the
//! smallest pack (largest DoD) that still meets the life target, using the cell
//! [`DegradationModel`].
//!
//! Elegant coupling worth noting: the same oversize that buys 10-year life also
//! drops the flight C-rate, so a **1:1 charge (= flight power) becomes a gentle
//! low-C charge** — the longevity and fast-turnaround goals reinforce each other.

use helisim_cell::DegradationModel;

/// A life-sized pack: the depth-of-discharge it runs at, how much bigger than the
/// energy-minimum pack it is, and the resulting capacity / mass / C-rate / fade.
#[derive(Clone, Copy, Debug)]
pub struct LifeSizing {
    /// Depth of discharge used each flight (≤1.0).
    pub dod: f64,
    /// Oversize vs the energy-minimum (full-DoD) pack, `1/dod`.
    pub oversize: f64,
    /// Flight (discharge) C-rate of the sized pack, `dod / flight_time_h`.
    pub flight_c_rate: f64,
    /// Required pack capacity, Wh.
    pub capacity_wh: f64,
    /// Pack mass, kg (from `pack_specific_energy_wh_per_kg`).
    pub pack_mass_kg: f64,
    /// Predicted capacity fade over the life, fraction.
    pub fade_over_life: f64,
    /// True if the life target is met (false only if even a tiny DoD can't — i.e.
    /// calendar fade alone exceeds EOL).
    pub feasible: bool,
}

/// Size a pack so `flights_per_year` over `life_years` keeps fade ≤ EOL.
/// `flight_energy_wh` is the energy used per flight; `flight_time_h` its duration.
pub fn size_for_life(
    model: &DegradationModel,
    flight_energy_wh: f64,
    flight_time_h: f64,
    flights_per_year: f64,
    life_years: f64,
    storage_temp_c: f64,
    pack_specific_energy_wh_per_kg: f64,
) -> LifeSizing {
    let n = flights_per_year * life_years;
    // Lower DoD ⇒ lower C-rate and lower effective throughput ⇒ less fade (monotone),
    // so scan DoD from full down and take the FIRST (largest) depth that meets EOL.
    let mut chosen: Option<f64> = None;
    let mut d = 1.0;
    while d >= 0.05 {
        let c_rate = d / flight_time_h;
        let fade = model.fade_over_life(n, d, c_rate, 25.0, life_years, storage_temp_c, 1.0);
        if fade <= model.eol_fade {
            chosen = Some(d);
            break;
        }
        d -= 0.01;
    }
    let feasible = chosen.is_some();
    let dod = chosen.unwrap_or(0.05);
    let c_rate = dod / flight_time_h;
    let capacity_wh = flight_energy_wh / dod;
    let fade = model.fade_over_life(n, dod, c_rate, 25.0, life_years, storage_temp_c, 1.0);
    LifeSizing {
        dod,
        oversize: 1.0 / dod,
        flight_c_rate: c_rate,
        capacity_wh,
        pack_mass_kg: capacity_wh / pack_specific_energy_wh_per_kg,
        fade_over_life: fade,
        feasible,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> DegradationModel {
        DegradationModel::default()
    }

    /// Daily flying (365/yr) of a 20-min mission forces an oversized pack
    /// (DoD < 1) to reach 10 years; the predicted fade clears EOL.
    #[test]
    fn daily_flying_needs_oversize() {
        let s = size_for_life(&model(), 1000.0, 1.0 / 3.0, 365.0, 10.0, 25.0, 180.0);
        assert!(
            s.oversize > 1.5,
            "daily should oversize, got {}",
            s.oversize
        );
        assert!(s.dod < 1.0 && s.feasible);
        assert!(s.fade_over_life <= 0.20 + 1e-6);
        // Bigger pack ⇒ lower flight C-rate than the full-DoD 3C.
        assert!(s.flight_c_rate < 3.0);
        // Capacity is the energy-minimum (1000 Wh) scaled by the oversize.
        assert!((s.capacity_wh - 1000.0 * s.oversize).abs() < 1.0);
    }

    /// Light use (10 flights/yr) needs no oversize — full DoD already lasts 10 yr.
    #[test]
    fn light_use_no_oversize() {
        let s = size_for_life(&model(), 1000.0, 1.0 / 3.0, 10.0, 10.0, 25.0, 180.0);
        assert!((s.oversize - 1.0).abs() < 1e-9, "oversize {}", s.oversize);
        assert!((s.dod - 1.0).abs() < 1e-9);
    }

    /// More flights per year ⇒ a bigger pack is required.
    #[test]
    fn oversize_grows_with_usage() {
        let a = size_for_life(&model(), 1000.0, 1.0 / 3.0, 100.0, 10.0, 25.0, 180.0);
        let b = size_for_life(&model(), 1000.0, 1.0 / 3.0, 365.0, 10.0, 25.0, 180.0);
        assert!(b.oversize >= a.oversize);
    }

    /// Cooler storage (less calendar fade) needs a smaller pack for the same life.
    #[test]
    fn cooler_storage_needs_less_pack() {
        let warm = size_for_life(&model(), 1000.0, 1.0 / 3.0, 365.0, 10.0, 30.0, 180.0);
        let cool = size_for_life(&model(), 1000.0, 1.0 / 3.0, 365.0, 10.0, 15.0, 180.0);
        assert!(cool.oversize <= warm.oversize);
    }
}
