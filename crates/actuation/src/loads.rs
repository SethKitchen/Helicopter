//! Demands the actuators must meet — derived from the design, not guessed.
//!
//! The selection rule is only as good as the load fed to it. Two demands:
//! * **Motor** — the continuous shaft power it must sustain (hover power with a
//!   climb/manoeuvre margin), plus a Kv/voltage feasibility gate (it must actually
//!   reach head speed on the chosen cells) and a current ceiling.
//! * **Servo** — the per-blade swashplate control moment, dominated by the
//!   centrifugal **propeller ("tennis-racket") moment** that tries to flatten a
//!   pitched, spinning blade, with the aerodynamic hinge moment as a small second
//!   term. Reference: Prouty, *Helicopter Performance, Stability, and Control*,
//!   feathering/propeller moment.

use helisim_design::DesignCandidate;
use std::f64::consts::PI;

/// Sea-level air density, kg/m³ (consistent with the BEMT core).
const RHO: f64 = 1.225;

/// Continuous-power margin over hover for the motor: climb, manoeuvre and
/// headroom. Hover is the *minimum* steady demand; a heli motor is sized above it.
pub const MOTOR_POWER_MARGIN: f64 = 1.6;

/// Representative max blade pitch (collective + cyclic), rad — the worst-case
/// feathering angle the servo holds against. ~14°.
pub const THETA_MAX_RAD: f64 = 0.244;

/// Representative section pitching-moment coefficient magnitude for the hinge
/// (aerodynamic feathering) term. Near zero for the symmetric NACA 0012 about its
/// aerodynamic centre; a small non-zero value keeps the term present (control
/// linkages, AC/feather-axis offset) rather than silently dropped.
pub const CM_HINGE: f64 = 0.01;

/// Load-factor derate of no-load Kv·V when reaching head speed under aerodynamic
/// load (a loaded outrunner turns ~80–85 % of its no-load RPM).
pub const KV_LOAD_FACTOR: f64 = 0.85;

/// Nominal LiPo cell voltage, V (mid-discharge).
pub const CELL_NOMINAL_V: f64 = 3.7;

/// Continuous shaft-power the motor must sustain, W = hover power × margin.
pub fn motor_power_demand(hover_shaft_power_w: f64) -> f64 {
    hover_shaft_power_w * MOTOR_POWER_MARGIN
}

/// Main-rotor head speed, RPM (`Ω·60/2π`).
pub fn head_speed_rpm(c: &DesignCandidate) -> f64 {
    c.omega() * 60.0 / (2.0 * PI)
}

/// One blade's mass, kg — areal density × planform (cutout → tip), as in
/// `manufacture::fasteners`.
pub fn blade_mass_kg(c: &DesignCandidate) -> f64 {
    let span = c.radius_m - c.root_cutout * c.radius_m;
    c.blade_areal_density_kg_m2 * c.chord_m * span
}

/// Centrifugal **propeller (feathering) moment** per blade, N·m.
///
/// A blade element of mass `dm` at chordwise offset `y` feels centrifugal force
/// `Ω²r dm`; when the blade is feathered by `θ` the out-of-plane displacement
/// `y sinθ` gives a restoring feathering moment. Integrated:
/// `M = Ω² · I_f · sinθ cosθ`, with chordwise feathering inertia
/// `I_f = ∫ y² dm = m_blade · c²/12` for a rectangular blade about its mid-chord
/// feathering axis. This is the dominant swashplate control load.
pub fn propeller_moment_nm(c: &DesignCandidate, theta_rad: f64) -> f64 {
    let omega = c.omega();
    let i_feather = blade_mass_kg(c) * c.chord_m * c.chord_m / 12.0;
    omega * omega * i_feather * theta_rad.sin() * theta_rad.cos()
}

/// Aerodynamic hinge (feathering) moment per blade, N·m — the secondary term.
/// Section moment `Cm·½ρU²c²` integrated over the span (`∫₀^R (Ωr)² dr = Ω²R³/3`).
pub fn aero_hinge_moment_nm(c: &DesignCandidate) -> f64 {
    let omega = c.omega();
    CM_HINGE * 0.5 * RHO * c.chord_m * c.chord_m * omega * omega * c.radius_m.powi(3) / 3.0
}

/// Total per-blade control moment the swashplate servo reacts, N·m =
/// propeller moment (primary) + aero hinge moment (secondary), at `THETA_MAX_RAD`.
pub fn servo_torque_demand(c: &DesignCandidate) -> f64 {
    propeller_moment_nm(c, THETA_MAX_RAD).abs() + aero_hinge_moment_nm(c)
}

/// Pack nominal voltage for `cells` in series, V.
pub fn pack_voltage_v(cells: u32) -> f64 {
    cells as f64 * CELL_NOMINAL_V
}

/// Can this motor reach head speed at the given gear ratio on `cells`?
/// `Kv · V_pack · load_factor ≥ head_rpm · gear_ratio`.
pub fn motor_kv_ok(kv: f64, cells: u32, head_rpm: f64, gear_ratio: f64) -> bool {
    kv * pack_voltage_v(cells) * KV_LOAD_FACTOR >= head_rpm * gear_ratio
}

/// Continuous current the motor would draw delivering `power_w` on `cells`, A.
pub fn motor_current_a(power_w: f64, cells: u32) -> f64 {
    power_w / pack_voltage_v(cells)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DOCUMENTED — the propeller moment for `DesignCandidate::model()` at 14°.
    /// Hand-check: Ω = 125/0.6 = 208.3 rad/s; m_blade = 2.5·0.05·(0.6·0.85) = 0.0638 kg;
    /// I_f = 0.0638·0.05²/12 = 1.328e-5 kg·m²; M = 208.3²·1.328e-5·sin14°cos14°
    /// ≈ 0.576·0.235 ≈ 0.135 N·m. A mini cyclic servo (DS450 = 0.392 N·m) covers it.
    #[test]
    fn propeller_moment_documented_value() {
        let c = DesignCandidate::model();
        let m = propeller_moment_nm(&c, THETA_MAX_RAD);
        assert!((m - 0.135).abs() < 0.02, "got {m}");
    }

    /// The propeller moment scales as Ω², m_blade and c² — the parametric levers.
    #[test]
    fn propeller_moment_scaling_laws() {
        let c = DesignCandidate::model();
        let base = propeller_moment_nm(&c, THETA_MAX_RAD);

        // Ω² : double tip speed (Ω) at fixed radius → ×4.
        let faster = DesignCandidate {
            tip_speed_ms: 2.0 * c.tip_speed_ms,
            ..c
        };
        assert!((propeller_moment_nm(&faster, THETA_MAX_RAD) / base - 4.0).abs() < 1e-6);

        // c² (and m_blade ∝ c): doubling chord → I_f ∝ c³ → ×8.
        let wider = DesignCandidate {
            chord_m: 2.0 * c.chord_m,
            ..c
        };
        assert!((propeller_moment_nm(&wider, THETA_MAX_RAD) / base - 8.0).abs() < 1e-6);
    }

    #[test]
    fn motor_power_demand_applies_margin() {
        assert!((motor_power_demand(600.0) - 960.0).abs() < 1e-9);
    }

    /// A high-Kv small motor reaches head speed on a small pack; a low-Kv 700-class
    /// motor cannot on too few cells.
    #[test]
    fn kv_gate_distinguishes_feasible_from_not() {
        let c = DesignCandidate::model();
        let rpm = head_speed_rpm(&c); // ~1989
        // 1600 Kv on 6S (22.2 V), 9:1 gear: 1600·22.2·0.85 = 30192 ≥ 1989·9 = 17901. OK.
        assert!(motor_kv_ok(1600.0, 6, rpm, 9.0));
        // 520 Kv on 2S (7.4 V): 520·7.4·0.85 = 3271 < 17901. Not OK.
        assert!(!motor_kv_ok(520.0, 2, rpm, 9.0));
    }
}
