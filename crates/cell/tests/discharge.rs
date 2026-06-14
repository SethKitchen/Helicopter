//! Cell discharge validation against the Samsung INR18650-25R datasheet.
//!
//! Discipline (mirrors the Caradonna-Tung "validate the quantity you didn't fit"
//! approach): the OCV-SoC curve is fitted to the low-rate (≈OCV) behaviour and
//! `r_internal` is fitted to the single 20 A energy point (7.83 Wh). The model
//! must then *predict* the delivered capacity at the other rates (0.5 A, 5 A,
//! 10 A) — which it was not fitted to — and reproduce the cell's signature flat
//! capacity and monotonic voltage sag.
//!
//! Datasheet / measured oracle (Samsung SDI + independent tests):
//!   * 2500 mAh at 0.2C (0.5 A); ~2450 mAh and never below 2400 mAh from 5–20 A
//!   * energy at 20 A = 7.83 Wh  (=> average voltage ≈ 3.20 V)
//!   * cut-off 2.5 V, full 4.20 V, nominal 3.6 V, DC IR ≈ 14.8 mΩ

use helisim_cell::{Cell, TheveninCell};

/// Simulate a constant-current discharge to the cut-off voltage. Returns
/// (delivered capacity in mAh, energy in Wh, average terminal voltage).
fn discharge_cc(cell: &TheveninCell, current: f64, dt_s: f64) -> (f64, f64, f64) {
    let cap = cell.capacity_ah();
    let dt_h = dt_s / 3600.0;
    let mut soc = 1.0;
    let mut ah = 0.0;
    let mut wh = 0.0;
    loop {
        let v = cell.terminal_voltage(soc, current);
        if v <= cell.cutoff_voltage() || soc <= 0.0 {
            break;
        }
        let dq = current * dt_h;
        ah += dq;
        wh += v * dq;
        soc -= dq / cap;
    }
    let avg_v = if ah > 0.0 { wh / ah } else { 0.0 };
    (ah * 1000.0, wh, avg_v)
}

#[test]
fn fit_point_20a_energy_matches_datasheet() {
    let cell = TheveninCell::samsung_25r();
    let (mah, wh, avg_v) = discharge_cc(&cell, 20.0, 0.5);
    // The R fit targets 7.83 Wh at 20 A.
    assert!(
        (wh - 7.83).abs() / 7.83 < 0.04,
        "20A energy {wh:.3} Wh vs 7.83 Wh (avg V {avg_v:.3}, {mah:.0} mAh)"
    );
    // Average voltage at 20 A should be ~3.20 V.
    assert!((avg_v - 3.20).abs() < 0.12, "20A avg voltage {avg_v:.3} V");
}

#[test]
fn predicted_capacity_is_flat_across_c_rates() {
    // 0.5 A and 5/10 A capacities are NOT fitted — they are predictions.
    let cell = TheveninCell::samsung_25r();
    let (m05, _, v05) = discharge_cc(&cell, 0.5, 1.0);
    let (m5, _, v5) = discharge_cc(&cell, 5.0, 0.5);
    let (m10, _, v10) = discharge_cc(&cell, 10.0, 0.5);
    let (m20, _, v20) = discharge_cc(&cell, 20.0, 0.5);

    // 0.2C delivers ~full 2500 mAh.
    assert!((m05 - 2500.0).abs() < 60.0, "0.5A capacity {m05:.0} mAh");
    // 5–20 A: ~2450 mAh and never below 2400 mAh (the 25R's signature flatness).
    for (label, m) in [("5A", m5), ("10A", m10), ("20A", m20)] {
        assert!(
            (2400.0..=2520.0).contains(&m),
            "{label} predicted capacity {m:.0} mAh outside [2400, 2520]"
        );
    }

    // Voltage sag must be monotone with C-rate.
    assert!(
        v05 > v5 && v5 > v10 && v10 > v20,
        "avg V should fall with current"
    );
}

#[test]
fn internal_resistance_is_physical() {
    // Fitted R should sit just above the measured DC IR (14.8 mΩ), reflecting
    // sustained-load polarisation lumped into one resistance.
    let cell = TheveninCell::samsung_25r();
    let r = cell.internal_resistance(0.5);
    assert!(
        (0.0148..=0.030).contains(&r),
        "fitted R {r:.4} Ω not physical"
    );
}

#[test]
fn continuous_rating_is_8c() {
    let cell = TheveninCell::samsung_25r();
    assert!((cell.max_continuous_c_rate() - 8.0).abs() < 1e-9);
}

/// DOCUMENTED defaults — the trait's lumped-thermal and electrical default methods
/// for an 18650: specific heat 900 J/(kg·K), surface area ≈4.09e-3 m² (18×65 mm
/// cylinder), heat capacity = m·c_p, and the V=OCV−I·R / max-power identities.
#[test]
fn trait_default_thermal_and_electrical_methods() {
    let cell = TheveninCell::samsung_25r();
    assert!((cell.specific_heat() - 900.0).abs() < 1e-9);
    assert!((cell.surface_area() - 4.09e-3).abs() < 1e-9);
    assert!((cell.heat_capacity() - cell.mass_kg() * 900.0).abs() < 1e-9);
    // V = OCV − I·R at SoC 0.5.
    let (soc, i) = (0.5, 10.0);
    assert!(
        (cell.terminal_voltage(soc, i) - (cell.ocv(soc) - i * cell.internal_resistance(soc))).abs()
            < 1e-12
    );
    // max power = OCV²/4R.
    assert!(
        (cell.max_power(soc) - cell.ocv(soc).powi(2) / (4.0 * cell.internal_resistance(soc))).abs()
            < 1e-9
    );
}
