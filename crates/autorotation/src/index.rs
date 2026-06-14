//! Flare energy: how much the spinning rotor can do to arrest the descent.
//!
//! Steady autorotation gets the aircraft down at a constant (high) rate; the
//! landing is survived by the **flare** — trading the rotor's stored rotational
//! kinetic energy for a momentary thrust surge that arrests the descent just
//! above the ground. The available energy is `E = ½ I Ω²` (`I` = polar moment of
//! inertia of the rotor about the shaft, `Ω` = rotational speed).
//!
//! Two derived metrics:
//!
//! * **Flare-height equivalent** `E / W` (metres) — the height through which the
//!   rotor's stored energy could lift the aircraft's weight. A clean, unit-system-
//!   independent feel for the margin: it must comfortably exceed the height lost
//!   while the pilot reacts and the collective comes up.
//! * **Autorotation index** `AI = E / (W · DL)` — stored energy per unit weight
//!   per unit disk loading `DL = W/A`. Higher disk loading (smaller disk for the
//!   weight) makes the flare harder, so it divides. This is the standard
//!   rotor-sizing autorotation figure of merit (e.g. Fradenburgh; Prouty,
//!   *Helicopter Performance, Stability and Control*). Its absolute threshold is
//!   convention- and unit-dependent, so we report it and validate its defining
//!   algebra and scaling rather than asserting a fabricated cutoff.
//!
//! The headline design consequence — surfaced here, paid for in the sizing study —
//! is that stored energy scales as `I Ω²` while weight scales with size cubed:
//! small model rotors carry proportionally little flare energy, so autorotation
//! margin is a *sizing constraint*, not an afterthought.

/// Rotor stored rotational kinetic energy `E = ½ I Ω²`, joules.
/// `inertia` is the rotor polar moment of inertia about the shaft (kg·m²),
/// `omega` the rotational speed (rad/s).
pub fn rotor_kinetic_energy(inertia: f64, omega: f64) -> f64 {
    0.5 * inertia * omega * omega
}

/// Flare-height equivalent `E / W`, metres: the height the stored rotor energy
/// could lift the gross weight. `weight_n = m g`.
pub fn flare_height_equivalent(inertia: f64, omega: f64, weight_n: f64) -> f64 {
    rotor_kinetic_energy(inertia, omega) / weight_n
}

/// Autorotation index `AI = ½IΩ² / (W · DL)`, where the disk loading
/// `DL = W / A`. Units: m³/N in SI (J·m²/N²). Reported as a comparative sizing
/// metric; higher is more autorotation-capable. `disk_area_m2 = A`.
pub fn autorotation_index(inertia: f64, omega: f64, weight_n: f64, disk_area_m2: f64) -> f64 {
    let energy = rotor_kinetic_energy(inertia, omega);
    let disk_loading = weight_n / disk_area_m2;
    energy / (weight_n * disk_loading)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Kinetic energy is exactly `½IΩ²` and scales quadratically with RPM —
    /// doubling Ω quadruples the available flare energy.
    #[test]
    fn kinetic_energy_scales_quadratically() {
        let e1 = rotor_kinetic_energy(0.5, 100.0);
        let e2 = rotor_kinetic_energy(0.5, 200.0);
        assert!((e1 - 0.5 * 0.5 * 100.0 * 100.0).abs() < 1e-9);
        assert!((e2 / e1 - 4.0).abs() < 1e-9);
    }

    /// Flare-height equivalent is energy per unit weight, in metres.
    #[test]
    fn flare_height_is_energy_over_weight() {
        let e = rotor_kinetic_energy(2.0, 50.0); // 2500 J
        let h = flare_height_equivalent(2.0, 50.0, 1000.0);
        assert!((h - e / 1000.0).abs() < 1e-9);
        assert!((h - 2.5).abs() < 1e-9);
    }

    /// The index penalises disk loading: for fixed stored energy and weight, a
    /// smaller disk (higher DL) gives a lower autorotation index.
    #[test]
    fn index_penalises_disk_loading() {
        let big_disk = autorotation_index(1.0, 100.0, 1000.0, 10.0);
        let small_disk = autorotation_index(1.0, 100.0, 1000.0, 5.0);
        assert!(small_disk < big_disk);
        // Halving area halves AI (DL doubles).
        assert!((big_disk / small_disk - 2.0).abs() < 1e-9);
    }
}
