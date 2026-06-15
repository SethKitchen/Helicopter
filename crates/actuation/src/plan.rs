//! [`ActuationPlan`] — the selected motor + swashplate/tail servos for a design.
//!
//! Composes the demands ([`crate::loads`]) and the catalogues
//! ([`crate::motor`]/[`crate::servo`]) through the smallest-adequate rule
//! ([`crate::scaling`]) into one buyable hardware set, with masses (which feed the
//! cost BOM) and honest notes (Kv/current checks, beyond-catalogue flags).

use crate::loads::{
    head_speed_rpm, motor_current_a, motor_kv_ok, motor_power_demand, servo_torque_demand,
};
use crate::motor::{BldcMotor, scorpion_hk_catalogue};
use crate::scaling::{Sized, size_or_extrapolate};
use crate::selectable::Selectable;
use crate::servo::{Servo, ServoRole, align_hv_catalogue};
use helisim_design::{DesignCandidate, DesignReport};

/// Tail-pitch control load as a fraction of the per-blade cyclic moment.
/// Representative, from the typical 700-class servo pairing (Align DS820 cyclic
/// 23 kg·cm / DS825 tail 12.5 kg·cm ≈ 0.54). A dedicated tail-rotor feathering
/// model would refine it; named here as an assumption, not derived physics.
pub const TAIL_LOAD_FRACTION: f64 = 0.54;

/// Knobs for the selection (defaults suit a model CCPM head).
#[derive(Clone, Copy, Debug)]
pub struct ActuationConfig {
    /// Main-rotor reduction (motor RPM = head RPM × gear_ratio).
    pub gear_ratio: f64,
    /// Number of swashplate (CCPM cyclic/collective) servos.
    pub n_cyclic_servos: usize,
    /// Safety factor on the motor power demand (the hover→continuous margin is
    /// already in [`motor_power_demand`], so this is extra headroom; default 1.0).
    pub motor_sf: f64,
    /// Safety factor on the servo torque demand (control loads are peaky).
    pub servo_sf: f64,
}

impl Default for ActuationConfig {
    fn default() -> Self {
        ActuationConfig {
            gear_ratio: 9.0,
            n_cyclic_servos: 3,
            motor_sf: 1.0,
            servo_sf: 1.5,
        }
    }
}

/// The chosen actuation hardware for a design.
#[derive(Clone, Debug)]
pub struct ActuationPlan {
    /// The motor (real part, or extrapolated + flagged).
    pub motor: Sized<BldcMotor>,
    /// One swashplate servo (×`n_cyclic_servos`).
    pub cyclic_servo: Sized<Servo>,
    /// Number of cyclic servos.
    pub n_cyclic_servos: usize,
    /// The tail-pitch servo.
    pub tail_servo: Sized<Servo>,
    /// Pack cells the motor is run on (its rated max).
    pub cells: u32,
    /// Gear ratio used.
    pub gear_ratio: f64,
    /// Motor continuous-power demand, W.
    pub motor_power_demand_w: f64,
    /// Motor continuous current at the demand on `cells`, A.
    pub motor_current_a: f64,
    /// Per-blade swashplate control-moment demand, N·m.
    pub servo_demand_nm: f64,
    /// Tail-pitch control-moment demand, N·m.
    pub tail_demand_nm: f64,
    /// Total installed actuation mass (motor + all servos), kg.
    pub total_mass_kg: f64,
    /// Total actuation hardware price, USD (0 for any beyond-catalogue part).
    pub total_price_usd: f64,
    /// Selection notes: Kv/current checks and beyond-catalogue flags.
    pub notes: Vec<String>,
}

/// Servos of a given role, or the whole set if none match (so a custom catalogue
/// without that role still selects something rather than failing).
fn pool_by_role(servos: &[Servo], role: ServoRole) -> Vec<Servo> {
    let matching: Vec<Servo> = servos.iter().cloned().filter(|s| s.role == role).collect();
    if matching.is_empty() {
        servos.to_vec()
    } else {
        matching
    }
}

/// Select actuation with the default Scorpion/Align catalogues and config.
pub fn select_actuation(c: &DesignCandidate, report: &DesignReport) -> ActuationPlan {
    select_actuation_with(
        c,
        report,
        &scorpion_hk_catalogue(),
        &align_hv_catalogue(),
        ActuationConfig::default(),
    )
}

/// Select actuation against supplied catalogues (the override hook, like
/// `cost::UnitCosts`).
pub fn select_actuation_with(
    c: &DesignCandidate,
    report: &DesignReport,
    motors: &[BldcMotor],
    servos: &[Servo],
    cfg: ActuationConfig,
) -> ActuationPlan {
    let mut notes = Vec::new();

    // --- motor: size on continuous power, then check Kv + current ---
    let motor_power_demand_w = motor_power_demand(report.hover_shaft_power_w);
    let motor = size_or_extrapolate(motors, motor_power_demand_w, cfg.motor_sf, "motor");

    let (cells, motor_current_a_val) = match &motor.part {
        Some(m) => {
            let cells = m.max_cells;
            let rpm = head_speed_rpm(c);
            if !motor_kv_ok(m.kv, cells, rpm, cfg.gear_ratio) {
                notes.push(format!(
                    "⚠ {} ({} Kv) on {}S cannot reach head speed at {:.0}:1 — \
                     raise cells or lower the gear ratio.",
                    m.name, m.kv, cells, cfg.gear_ratio
                ));
            }
            let amps = motor_current_a(motor_power_demand_w, cells);
            if amps > m.max_cont_current_a {
                notes.push(format!(
                    "⚠ {} draws {:.0} A at the power demand on {}S, over its {:.0} A \
                     continuous rating — use more cells.",
                    m.name, amps, cells, m.max_cont_current_a
                ));
            }
            (cells, amps)
        }
        None => (0, f64::NAN),
    };
    if let Some(n) = &motor.note {
        notes.push(n.clone());
    }

    // --- servos: swashplate (per-blade moment) + tail (fraction of it) ---
    // Select within the matching role pool so a cyclic position gets a cyclic
    // servo and the tail gets a tail servo (fall back to the full set if a custom
    // catalogue has no servo of that role).
    let servo_demand_nm = servo_torque_demand(c);
    let tail_demand_nm = servo_demand_nm * TAIL_LOAD_FRACTION;
    let cyclic_pool = pool_by_role(servos, ServoRole::Cyclic);
    let tail_pool = pool_by_role(servos, ServoRole::Tail);
    let cyclic_servo =
        size_or_extrapolate(&cyclic_pool, servo_demand_nm, cfg.servo_sf, "cyclic servo");
    let tail_servo = size_or_extrapolate(&tail_pool, tail_demand_nm, cfg.servo_sf, "tail servo");
    for s in [&cyclic_servo, &tail_servo] {
        if let Some(n) = &s.note {
            notes.push(n.clone());
        }
    }

    let total_mass_kg =
        (motor.mass_g + cfg.n_cyclic_servos as f64 * cyclic_servo.mass_g + tail_servo.mass_g)
            / 1000.0;

    // Price (0 for any beyond-catalogue part — no real price to quote).
    let motor_price = motor.part.map(|m| m.price_usd).unwrap_or(0.0);
    let cyclic_price = cyclic_servo.part.map(|s| s.price_usd).unwrap_or(0.0);
    let tail_price = tail_servo.part.map(|s| s.price_usd).unwrap_or(0.0);
    let total_price_usd = motor_price + cfg.n_cyclic_servos as f64 * cyclic_price + tail_price;

    ActuationPlan {
        motor,
        cyclic_servo,
        n_cyclic_servos: cfg.n_cyclic_servos,
        tail_servo,
        cells,
        gear_ratio: cfg.gear_ratio,
        motor_power_demand_w,
        motor_current_a: motor_current_a_val,
        servo_demand_nm,
        tail_demand_nm,
        total_mass_kg,
        total_price_usd,
        notes,
    }
}

impl ActuationPlan {
    /// Motor mass, kg (0 with NaN guard if unselected/extrapolated to NaN).
    pub fn motor_mass_kg(&self) -> f64 {
        if self.motor.mass_g.is_finite() {
            self.motor.mass_g / 1000.0
        } else {
            0.0
        }
    }

    /// Total servo mass (cyclic ×N + tail), kg.
    pub fn servo_mass_kg(&self) -> f64 {
        (self.n_cyclic_servos as f64 * self.cyclic_servo.mass_g + self.tail_servo.mass_g) / 1000.0
    }

    /// The purchasable bill of materials for actuation: one line per part with
    /// quantity, name, **price** and **direct purchase link** (or a beyond-
    /// catalogue note where no real part exists).
    pub fn purchase_lines(&self) -> Vec<String> {
        let mut lines = Vec::new();
        match &self.motor.part {
            Some(m) => lines.push(format!(
                "motor   1× {:<24} ${:>7.2}   {}\n          ({})",
                m.name, m.price_usd, m.purchase_url, m.price_note
            )),
            None => lines.push(format!(
                "motor   1× (beyond catalogue — demand {:.0} W; no buyable part)",
                self.motor_power_demand_w
            )),
        }
        if let Some(s) = &self.cyclic_servo.part {
            lines.push(format!(
                "cyclic  {}× {:<24} ${:>7.2} ea (${:.2})   {}\n          ({})",
                self.n_cyclic_servos,
                s.name,
                s.price_usd,
                self.n_cyclic_servos as f64 * s.price_usd,
                s.purchase_url,
                s.price_note
            ));
        }
        if let Some(s) = &self.tail_servo.part {
            lines.push(format!(
                "tail    1× {:<24} ${:>7.2}   {}\n          ({})",
                s.name, s.price_usd, s.purchase_url, s.price_note
            ));
        }
        lines.push(format!(
            "total actuation hardware ≈ ${:.2} (prices sourced, as-of 2026-06 — confirm at the links)",
            self.total_price_usd
        ));
        lines
    }

    /// What power and connections the whole actuation set needs — a system
    /// budget (pack → ESC / HV BEC) plus each part's electrical detail.
    pub fn power_and_connections(&self) -> Vec<String> {
        let mut out = Vec::new();
        if let Some(m) = &self.motor.part {
            // ~2 A peak per digital servo is a standard budgeting figure.
            let servo_peak = (self.n_cyclic_servos + 1) as f64 * 2.0;
            out.push(format!(
                "System: one {}S LiPo feeds the ESC (motor) and an HV BEC (servos). Current budget ≈ \
                 {:.0} A motor continuous + ≈ {:.0} A servo peak.",
                self.cells, m.max_cont_current_a, servo_peak
            ));
            out.push(format!(
                "HV BEC: rated ≥ {:.0} A at the servo voltage to power {} cyclic + 1 tail digital servos.",
                servo_peak, self.n_cyclic_servos
            ));
            out.push(format!("— {} (motor) —", m.name));
            out.extend(m.power_and_connections(self.cells));
        }
        if let Some(s) = &self.cyclic_servo.part {
            out.push(format!("— {}× {} (cyclic) —", self.n_cyclic_servos, s.name));
            out.extend(s.power_and_connections(ServoRole::Cyclic));
        }
        if let Some(s) = &self.tail_servo.part {
            out.push(format!("— {} (tail) —", s.name));
            out.extend(s.power_and_connections(ServoRole::Tail));
        }
        out
    }

    /// How each selected part connects to the helicopter structure (which
    /// structure it mounts to and how), for the build output.
    pub fn structural_connections(&self) -> Vec<String> {
        let mut out = Vec::new();
        if let Some(m) = &self.motor.part {
            out.push(format!("— {} → powertrain tray + mast drive —", m.name));
            out.extend(m.structural_connections(self.gear_ratio));
        }
        if let Some(s) = &self.cyclic_servo.part {
            out.push(format!(
                "— {}× {} → frame deck + swashplate —",
                self.n_cyclic_servos, s.name
            ));
            out.extend(s.structural_connections(ServoRole::Cyclic));
        }
        if let Some(s) = &self.tail_servo.part {
            out.push(format!("— {} → tail boom + pitch slider —", s.name));
            out.extend(s.structural_connections(ServoRole::Tail));
        }
        out
    }

    /// Construct-and-use instructions for the selected hardware (motor install +
    /// each servo install), for the build output.
    pub fn build_instructions(&self) -> Vec<String> {
        let mut out = Vec::new();
        if let Some(m) = &self.motor.part {
            out.push(format!("— {} (install & use) —", m.name));
            out.extend(m.install_steps(self.cells, self.gear_ratio));
        }
        if let Some(s) = &self.cyclic_servo.part {
            out.push(format!(
                "— {}× {} cyclic / swashplate (install & use) —",
                self.n_cyclic_servos, s.name
            ));
            out.extend(s.install_steps(ServoRole::Cyclic));
        }
        if let Some(s) = &self.tail_servo.part {
            out.push(format!("— {} tail (install & use) —", s.name));
            out.extend(s.install_steps(ServoRole::Tail));
        }
        out
    }

    /// One-line summary for the design report.
    pub fn summary(&self) -> String {
        let motor = self
            .motor
            .part
            .as_ref()
            .map(|m| {
                format!(
                    "{} ({:.0} W, {} g)",
                    m.name(),
                    m.max_cont_power_w,
                    m.mass_g as i64
                )
            })
            .unwrap_or_else(|| "beyond catalogue".to_string());
        let cyc = self
            .cyclic_servo
            .part
            .as_ref()
            .map(|s| s.name().to_string())
            .unwrap_or_else(|| "beyond catalogue".to_string());
        let tail = self
            .tail_servo
            .part
            .as_ref()
            .map(|s| s.name().to_string())
            .unwrap_or_else(|| "beyond catalogue".to_string());
        format!(
            "motor {motor} on {}S; {}× {cyc} cyclic + {tail} tail; {:.2} kg actuation",
            self.cells, self.n_cyclic_servos, self.total_mass_kg
        )
    }
}
