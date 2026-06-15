//! Electrical **power budget** the battery pack must supply — the bridge from the
//! actuation hardware to the battery/BMS sizing.
//!
//! The pack feeds two loads through two converters:
//! * the **motor**, through the ESC (the big load — continuous shaft power);
//! * the **control-surface actuators** (swashplate cyclic + tail servos), through
//!   an **HV BEC** (a small but real load that the pack must also carry).
//!
//! Plus a small **avionics** rail (flight controller / receiver / gyro).
//!
//! This module turns an [`ActuationPlan`] into the single number the battery
//! sizing needs — total pack power and peak pack current — so the pack is sized to
//! run the motor AND the actuators, not the motor alone. It is pure arithmetic over
//! the plan (no battery dependency); the CLI feeds the result into `bms` sizing.

use crate::loads::{CELL_NOMINAL_V, pack_voltage_v};
use crate::plan::ActuationPlan;

/// Standard budgeting figure for a digital servo's peak current draw, A. Matches
/// the figure used in [`ActuationPlan::power_and_connections`] (~1–2 A under load;
/// 2 A is the conservative peak).
pub const SERVO_PEAK_A_EACH: f64 = 2.0;
/// Switching HV-BEC conversion efficiency (pack voltage → servo voltage).
/// Representative for a good switching BEC; a stated assumption.
pub const BEC_EFFICIENCY: f64 = 0.85;
/// Avionics rail (flight controller + receiver + gyro), W — representative.
pub const AVIONICS_POWER_W: f64 = 5.0;

/// The electrical demand the pack must supply, broken out by load.
#[derive(Clone, Copy, Debug)]
pub struct PowerBudget {
    /// Pack nominal voltage, V (`cells × 3.7`).
    pub pack_voltage_v: f64,
    /// Motor continuous electrical power (through the ESC), W.
    pub motor_power_w: f64,
    /// Motor continuous current at the pack voltage, A.
    pub motor_current_a: f64,
    /// Number of control-surface servos (cyclic ×N + tail).
    pub servo_count: usize,
    /// Servo rail voltage, V (HV BEC output).
    pub servo_voltage_v: f64,
    /// Combined servo peak current at the servo voltage, A.
    pub servo_peak_current_a: f64,
    /// Servo electrical power at the servo rail, W.
    pub servo_power_w: f64,
    /// Power the HV BEC draws FROM the pack to supply the servos, W
    /// (`servo_power / BEC_EFFICIENCY`).
    pub bec_input_power_w: f64,
    /// Avionics rail power, W.
    pub avionics_power_w: f64,
    /// Total power the pack must supply, W (motor + BEC input + avionics).
    pub total_pack_power_w: f64,
    /// Peak current the pack must supply at its nominal voltage, A.
    pub pack_peak_current_a: f64,
}

/// Compute the pack power budget for a selected [`ActuationPlan`].
pub fn power_budget(plan: &ActuationPlan) -> PowerBudget {
    // Pack voltage from the motor's cell count (fall back to a 6S model rail if the
    // motor was beyond-catalogue and no cell count was set).
    let cells = if plan.cells > 0 { plan.cells } else { 6 };
    let pack_voltage_v = pack_voltage_v(cells);

    // Motor electrical power = the continuous demand the pack delivers through the
    // ESC. (`motor_current_a` is that demand ÷ pack voltage; recover power = I·V,
    // falling back to the demand figure if the current was NaN/beyond-catalogue.)
    let motor_current_a = if plan.motor_current_a.is_finite() {
        plan.motor_current_a
    } else {
        plan.motor_power_demand_w / pack_voltage_v
    };
    let motor_power_w = motor_current_a * pack_voltage_v;

    let servo_count = plan.n_cyclic_servos + 1; // cyclic ×N + tail
    let servo_voltage_v = plan
        .cyclic_servo
        .part
        .as_ref()
        .map(|s| s.voltage_v)
        .unwrap_or(8.4);
    let servo_peak_current_a = servo_count as f64 * SERVO_PEAK_A_EACH;
    let servo_power_w = servo_peak_current_a * servo_voltage_v;
    let bec_input_power_w = servo_power_w / BEC_EFFICIENCY;

    let total_pack_power_w = motor_power_w + bec_input_power_w + AVIONICS_POWER_W;
    let pack_peak_current_a = total_pack_power_w / pack_voltage_v;

    PowerBudget {
        pack_voltage_v,
        motor_power_w,
        motor_current_a,
        servo_count,
        servo_voltage_v,
        servo_peak_current_a,
        servo_power_w,
        bec_input_power_w,
        avionics_power_w: AVIONICS_POWER_W,
        total_pack_power_w,
        pack_peak_current_a,
    }
}

impl PowerBudget {
    /// Human-readable derivation, line by line — the calculation that "feeds power
    /// to the motor and the control-surface actuators".
    pub fn explain(&self) -> Vec<String> {
        vec![
            format!(
                "Pack rail: {:.1} V nominal ({} cells × {:.1} V).",
                self.pack_voltage_v,
                (self.pack_voltage_v / CELL_NOMINAL_V).round() as u32,
                CELL_NOMINAL_V
            ),
            format!(
                "Motor (via ESC): {:.0} W = {:.0} A × {:.1} V — the dominant load.",
                self.motor_power_w, self.motor_current_a, self.pack_voltage_v
            ),
            format!(
                "Servos (via HV BEC): {} servos × {:.0} A peak × {:.1} V = {:.0} W at the rail; \
                 BEC draws {:.0} W from the pack ({:.0} % efficient).",
                self.servo_count,
                SERVO_PEAK_A_EACH,
                self.servo_voltage_v,
                self.servo_power_w,
                self.bec_input_power_w,
                BEC_EFFICIENCY * 100.0
            ),
            format!("Avionics rail: {:.0} W.", self.avionics_power_w),
            format!(
                "→ Pack must supply {:.0} W = {:.0} A peak at {:.1} V (motor + servos + avionics).",
                self.total_pack_power_w, self.pack_peak_current_a, self.pack_voltage_v
            ),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan::select_actuation;
    use helisim_airfoil::LinearAirfoil;
    use helisim_bemt::Config;
    use helisim_design::{DesignCandidate, evaluate};

    fn model_budget() -> PowerBudget {
        let c = DesignCandidate::model();
        let report = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
        let plan = select_actuation(&c, &report);
        power_budget(&plan)
    }

    #[test]
    fn budget_sums_its_parts() {
        let b = model_budget();
        assert!(
            (b.total_pack_power_w - (b.motor_power_w + b.bec_input_power_w + b.avionics_power_w))
                .abs()
                < 1e-9
        );
        assert!((b.pack_peak_current_a - b.total_pack_power_w / b.pack_voltage_v).abs() < 1e-9);
    }

    #[test]
    fn servo_power_is_deterministic_from_count_and_voltage() {
        let b = model_budget();
        assert_eq!(b.servo_count, 4); // 3 cyclic + 1 tail
        assert!((b.servo_power_w - 4.0 * SERVO_PEAK_A_EACH * b.servo_voltage_v).abs() < 1e-9);
        // The BEC draws MORE than the rail power (conversion loss).
        assert!(b.bec_input_power_w > b.servo_power_w);
    }

    #[test]
    fn motor_dominates_but_actuators_add_real_load() {
        let b = model_budget();
        // Actuators + avionics raise the pack demand above the motor alone.
        assert!(b.total_pack_power_w > b.motor_power_w);
        // ...but the motor is the dominant load (servos are a minority share).
        assert!(b.motor_power_w > 0.5 * b.total_pack_power_w);
    }

    #[test]
    fn explain_lists_motor_servo_and_total() {
        let b = model_budget();
        let lines = b.explain();
        assert!(lines.iter().any(|l| l.contains("Motor")));
        assert!(lines.iter().any(|l| l.contains("Servos")));
        assert!(lines.iter().any(|l| l.contains("Pack must supply")));
    }
}
