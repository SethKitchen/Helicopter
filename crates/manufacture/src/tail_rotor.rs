//! Tail rotor — the anti-torque sub-assembly, sized as a small rotor in its own
//! right.
//!
//! The tail rotor must produce the side thrust that balances main-rotor torque
//! about the boom: `T_tr = Q / L_boom`. From there it is a miniature of the main
//! rotor — a radius set as a fraction of the main rotor, a tail solidity, its own
//! blades and hub. It reuses the blade [`BladeSpec`] for its (small) blades, so
//! the same shaping/export path applies.

use crate::blade::BladeSpec;
use crate::part::{BuildPart, Source};
use std::f64::consts::PI;

/// Tail-rotor radius as a fraction of the main-rotor radius (typical).
const TAIL_RADIUS_FRACTION: f64 = 0.18;
/// Tail-rotor solidity (higher than the main rotor — small, heavily loaded).
const TAIL_SOLIDITY: f64 = 0.12;
/// Tail-blade root cutout fraction.
const TAIL_CUTOUT: f64 = 0.20;

/// A tail-rotor specification.
#[derive(Clone, Debug)]
pub struct TailRotorSpec {
    /// Tail-rotor radius, m.
    pub radius_m: f64,
    /// Number of tail blades.
    pub n_blades: usize,
    /// Tail-blade chord, m.
    pub chord_m: f64,
    /// Required anti-torque thrust, N.
    pub thrust_n: f64,
    /// Tail-rotor tip speed, m/s.
    pub tip_speed_ms: f64,
    /// Tail-rotor speed, rev/min.
    pub rpm: f64,
    /// Estimated tail-rotor induced power, W.
    pub power_w: f64,
}

/// Size a tail rotor to react `main_torque_nm`, given the main rotor radius and
/// tip speed (the tail tip speed is matched to the main for a first cut).
pub fn tail_rotor_for(
    main_torque_nm: f64,
    main_radius_m: f64,
    main_tip_speed_ms: f64,
) -> TailRotorSpec {
    let boom_len = 1.15 * main_radius_m;
    let thrust = main_torque_nm / boom_len;
    let radius = TAIL_RADIUS_FRACTION * main_radius_m;
    let n_blades = 2;
    let chord = TAIL_SOLIDITY * PI * radius / n_blades as f64;
    let tip_speed = main_tip_speed_ms; // match for a first cut
    let omega = tip_speed / radius;
    let rpm = omega * 60.0 / (2.0 * PI);
    // Induced power estimate: P = T · v_i, v_i = √(T / 2ρA).
    let area = PI * radius * radius;
    let v_i = (thrust / (2.0 * 1.225 * area)).sqrt();
    let power = thrust * v_i;
    TailRotorSpec {
        radius_m: radius,
        n_blades,
        chord_m: chord,
        thrust_n: thrust,
        tip_speed_ms: tip_speed,
        rpm,
        power_w: power,
    }
}

impl TailRotorSpec {
    /// The (small) tail blade as a [`BladeSpec`], so the same shaping/export path
    /// applies as for the main blades.
    pub fn blade(&self) -> BladeSpec {
        let root_radius = TAIL_CUTOUT * self.radius_m;
        let span = self.radius_m - root_radius;
        let max_thickness = 0.12 * self.chord_m;
        BladeSpec {
            airfoil: "NACA 0012",
            n_blades: self.n_blades,
            root_radius_m: root_radius,
            tip_radius_m: self.radius_m,
            span_m: span,
            chord_m: self.chord_m,
            tip_chord_m: self.chord_m,
            twist_deg: 0.0,
            max_thickness_m: max_thickness,
            stock_block_mm: (
                span * 1000.0 * 1.10,
                self.chord_m * 1000.0 * 1.20,
                max_thickness * 1000.0 * 1.20,
            ),
            method: "3D-printed WHOLE in Markforged Onyx + continuous Fiberglass (fiber spanwise)",
            printed: true,
            service_print: false, // small tail blades fit a desktop bed
        }
    }
}

impl BuildPart for TailRotorSpec {
    fn name(&self) -> &str {
        "tail rotor (anti-torque)"
    }
    fn material(&self) -> &str {
        "small blades + grip hub; purchased pitch bearings; driven off the boom"
    }
    fn source(&self) -> Source {
        Source::Assembled
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("radius", self.radius_m * 1000.0),
            ("chord", self.chord_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        let b = self.blade();
        vec![
            format!(
                "1. Make {} tail blade(s): span {:.0} mm, chord {:.0} mm, NACA 0012 (same method \
                 as the main blade).",
                self.n_blades,
                b.span_m * 1000.0,
                self.chord_m * 1000.0
            ),
            format!(
                "2. Build a Ø{:.0} mm tail hub with pitch grips; it must make {:.1} N thrust to \
                 balance the main torque.",
                2.0 * b.root_radius_m * 1000.0,
                self.thrust_n
            ),
            format!(
                "3. Spin it at ~{:.0} rpm ({:.0} W); link its pitch to the pedal/yaw control.",
                self.rpm, self.power_w
            ),
            "4. Set the thrust direction to oppose main-rotor torque; check pedal authority."
                .to_string(),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thrust_balances_torque_over_the_boom() {
        let t = tail_rotor_for(2.0, 0.7, 90.0);
        let boom_len = 1.15 * 0.7;
        assert!((t.thrust_n - 2.0 / boom_len).abs() < 1e-9);
        // Tail radius is the expected fraction of the main rotor.
        assert!((t.radius_m - 0.18 * 0.7).abs() < 1e-12);
    }

    #[test]
    fn tail_blade_is_a_small_naca0012_blade() {
        let t = tail_rotor_for(2.0, 0.7, 90.0);
        let b = t.blade();
        assert_eq!(b.airfoil, "NACA 0012");
        assert!(b.span_m > 0.0 && b.span_m < t.radius_m);
        assert!((b.max_thickness_m - 0.12 * t.chord_m).abs() < 1e-12);
    }

    #[test]
    fn bigger_torque_needs_more_tail_thrust() {
        let a = tail_rotor_for(2.0, 0.7, 90.0);
        let b = tail_rotor_for(8.0, 0.7, 90.0);
        assert!(b.thrust_n > a.thrust_n);
    }
}
