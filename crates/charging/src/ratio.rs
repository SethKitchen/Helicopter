//! Charge-to-flight time ratio — the figure of merit behind "34 hours to charge
//! for 20 minutes of flight".
//!
//! ## The load-bearing identity
//! `ratio = charge_time / flight_time = P_flight / P_charge` — the stored energy
//! cancels. Two consequences a designer must internalise:
//! * **Pack size does not change the ratio.** A bigger battery flies longer AND
//!   takes proportionally longer to charge; the ratio is unmoved. (Worse: on a
//!   fixed source it can't go faster, so a bigger pack only stretches both.)
//! * **The only levers are charge power (up) and flight power (down).** 1:1 means
//!   charging at flight power.
//!
//! Flight is a *hard* discharge — a 20-minute hover is a ~3C draw — so a 1:1 charge
//! is a ~3C charge. Whether that is allowed is set by the **cell charge ceiling**
//! ([`cell_charge_power_ceiling_w`]); high-power cells (P50B ~5C) clear it, a wall
//! socket's *power* does not.

/// Flight (hover) time from usable energy and hover power, hours.
pub fn flight_time_h(usable_energy_wh: f64, hover_power_w: f64) -> f64 {
    if hover_power_w > 0.0 {
        usable_energy_wh / hover_power_w
    } else {
        f64::INFINITY
    }
}

/// Charge:flight ratio (charge_time ÷ flight_time). 1.0 is parity; 34.0 is the
/// wall-socket human-pack case.
pub fn charge_flight_ratio(charge_time_h: f64, flight_time_h: f64) -> f64 {
    if flight_time_h > 0.0 {
        charge_time_h / flight_time_h
    } else {
        f64::INFINITY
    }
}

/// Charge power needed to hit a target ratio: `P_charge = P_flight / ratio`
/// (so ratio 1 ⇒ charge at flight power).
pub fn charge_power_for_ratio(hover_power_w: f64, target_ratio: f64) -> f64 {
    hover_power_w / target_ratio
}

/// Maximum charge power the **cells** will accept, W = pack charge-current ceiling
/// × pack nominal voltage. If this is below flight power, even an infinite source
/// cannot reach 1:1 — the cells are the limit; if above, a strong enough source can.
pub fn cell_charge_power_ceiling_w(pack_charge_current_a: f64, pack_nominal_v: f64) -> f64 {
    pack_charge_current_a * pack_nominal_v
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The identity: ratio = P_flight / P_charge, independent of energy.
    #[test]
    fn ratio_equals_power_ratio_and_is_size_independent() {
        let p_flight = 150_000.0;
        let p_charge = 50_000.0;
        for energy in [10_000.0, 52_650.0, 1_000_000.0] {
            let t_f = flight_time_h(energy, p_flight);
            let t_c = flight_time_h(energy, p_charge); // charge "time" = E / P_charge
            let r = charge_flight_ratio(t_c, t_f);
            assert!((r - p_flight / p_charge).abs() < 1e-9, "energy {energy}");
            assert!((r - 3.0).abs() < 1e-9); // 150k/50k = 3, for every pack size
        }
    }

    /// 1:1 requires charging at flight power.
    #[test]
    fn unity_ratio_needs_flight_power() {
        assert!((charge_power_for_ratio(120_000.0, 1.0) - 120_000.0).abs() < 1e-9);
        // halving the ratio target doubles the charge power needed.
        assert!((charge_power_for_ratio(120_000.0, 0.5) - 240_000.0).abs() < 1e-9);
    }

    /// The cell ceiling decides whether 1:1 is even physically allowed.
    #[test]
    fn cell_ceiling_gates_feasibility() {
        // Human pack: 195S15P P50B, 25 A/cell charge → 375 A at 702 V ≈ 263 kW.
        let ceiling = cell_charge_power_ceiling_w(15.0 * 25.0, 702.0);
        assert!(ceiling > 250_000.0);
        // A ~126 kW flight power sits UNDER the ceiling → 1:1 is cell-feasible.
        assert!(126_000.0 < ceiling);
    }
}
