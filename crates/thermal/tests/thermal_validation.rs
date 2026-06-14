//! Thermal validation against published 18650 thermal characterization.
//!
//! Oracle (Samsung INR18650-25R + 18650 literature):
//!   * specific heat capacity 800–1100 J/(kg·K) (measured values cluster ~900)
//!   * Batemo free-convection thermal test terminates each constant-current
//!     discharge at *either* 2.5 V *or* a 75 °C surface temperature — i.e. at
//!     high current the cell hits the 75 °C protection limit before it empties,
//!     while at moderate current it empties first and stays cooler.
//!   * datasheet discharge surface-temperature limit: 75 °C.
//!
//! Non-circular: the heat capacity is from independent calorimetry, the surface
//! area from geometry, the heat is I²R using the cell's own resistance (which
//! also produces the validated voltage sag), and the cooling is the textbook
//! natural-convection coefficient — none of it tuned to the temperature data.
//! The model must then reproduce the *which-limit-terminates-discharge* behaviour.

use helisim_cell::{Cell, TheveninCell};
use helisim_thermal::{Convective, LumpedThermalCell};

const PROTECT_C: f64 = 75.0;

/// Constant-current discharge with lumped thermal, under natural convection,
/// stopping at the voltage cut-off OR the 75 °C protection limit (as the Batemo
/// rig does). Returns (peak temperature °C, terminated_on_temperature).
fn cc_discharge_thermal(cell: &TheveninCell, current: f64, ambient_c: f64) -> (f64, bool) {
    let lump = LumpedThermalCell::new(cell.heat_capacity(), cell.surface_area(), ambient_c);
    let cooling = Convective::natural_air();
    let dt = 0.5;
    let dt_h = dt / 3600.0;
    let cap = cell.capacity_ah();

    let mut soc = 1.0;
    let mut temp = ambient_c;
    let mut peak = ambient_c;
    loop {
        if cell.terminal_voltage(soc, current) <= cell.cutoff_voltage() || soc <= 0.0 {
            return (peak, false); // emptied before overheating
        }
        let q = current * current * cell.internal_resistance(soc);
        temp = lump.step(temp, q, &cooling, dt);
        peak = peak.max(temp);
        if temp >= PROTECT_C {
            return (peak, true); // hit the protection limit first
        }
        soc -= current * dt_h / cap;
    }
}

#[test]
fn specific_heat_in_literature_range() {
    let c = TheveninCell::samsung_25r();
    assert!(
        (800.0..=1100.0).contains(&c.specific_heat()),
        "specific heat {} outside measured 18650 range",
        c.specific_heat()
    );
}

#[test]
fn high_rate_20a_hits_protection_limit_first() {
    // Matches the Batemo free-convection test terminating on 75 °C, not voltage.
    let c = TheveninCell::samsung_25r();
    let (peak, on_temp) = cc_discharge_thermal(&c, 20.0, 25.0);
    assert!(
        on_temp,
        "20 A should reach 75 °C before emptying (peak {peak:.0} °C)"
    );
}

#[test]
fn moderate_rate_10a_stays_within_limit() {
    // At 10 A the cell should empty on voltage and stay under 75 °C.
    let c = TheveninCell::samsung_25r();
    let (peak, on_temp) = cc_discharge_thermal(&c, 10.0, 25.0);
    assert!(!on_temp, "10 A should empty before overheating");
    assert!(peak < PROTECT_C, "10 A peak {peak:.0} °C should be < 75 °C");
    assert!(
        peak > 40.0,
        "10 A should still warm noticeably (peak {peak:.0} °C)"
    );
}

#[test]
fn temperature_rise_is_monotonic_with_current() {
    let c = TheveninCell::samsung_25r();
    let p5 = cc_discharge_thermal(&c, 5.0, 25.0).0;
    let p10 = cc_discharge_thermal(&c, 10.0, 25.0).0;
    let p20 = cc_discharge_thermal(&c, 20.0, 25.0).0;
    assert!(
        p5 < p10 && p10 <= p20,
        "peaks: 5A={p5:.0} 10A={p10:.0} 20A={p20:.0}"
    );
}

#[test]
fn adiabatic_rise_matches_heat_capacity() {
    // Independent sanity on C = m·c_p: a published thermal-runaway study reports
    // 6 A producing ~1.6 kJ and heating the cell ~34 °C with no loss. Our heat
    // capacity (~40 J/K) gives ΔT = Q/C in the same ballpark for ~1.2–1.6 kJ.
    let c = TheveninCell::samsung_25r();
    let mc = c.heat_capacity();
    let dt_for_1600j = 1600.0 / mc;
    assert!(
        (25.0..=45.0).contains(&dt_for_1600j),
        "ΔT {dt_for_1600j:.0} °C off"
    );
}
