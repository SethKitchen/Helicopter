//! Rotor blade geometry, expressed as functions of the radial station `x = r/R`.

use std::f64::consts::PI;

/// Geometry of a single rotor with rigid, rectangular (or linearly tapered)
/// blades. Radial quantities are functions of the nondimensional station
/// `x = r/R`, so the solver never special-cases constant vs. varying chord/twist.
#[derive(Clone, Debug)]
pub struct Rotor {
    /// Number of blades.
    pub n_blades: usize,
    /// Rotor radius, m.
    pub radius: f64,
    /// Chord at the root station, m.
    pub root_chord: f64,
    /// Chord at the tip, m. Equal to `root_chord` for a rectangular blade.
    pub tip_chord: f64,
    /// Collective (root) pitch, radians.
    pub collective: f64,
    /// Linear twist from root to tip, radians (negative = washout). Local pitch
    /// is `collective + twist_rate * (x - root_cutout)`.
    pub twist_rate: f64,
    /// Inboard start of the lifting blade, as a fraction of radius.
    pub root_cutout: f64,
}

impl Rotor {
    /// A rectangular, untwisted rotor (the Caradonna & Tung geometry).
    pub fn rectangular(
        n_blades: usize,
        radius: f64,
        chord: f64,
        collective: f64,
        root_cutout: f64,
    ) -> Self {
        Rotor {
            n_blades,
            radius,
            root_chord: chord,
            tip_chord: chord,
            collective,
            twist_rate: 0.0,
            root_cutout,
        }
    }

    /// Return a copy with a different collective pitch (radians). Convenient for
    /// sweeping pitch in a validation loop without mutating the original.
    pub fn with_collective(&self, collective: f64) -> Self {
        Rotor {
            collective,
            ..self.clone()
        }
    }

    /// Chord at station `x`, m (linear between root and tip chords).
    pub fn chord(&self, x: f64) -> f64 {
        self.root_chord + (self.tip_chord - self.root_chord) * x
    }

    /// Local geometric pitch at station `x`, radians.
    pub fn pitch(&self, x: f64) -> f64 {
        self.collective + self.twist_rate * (x - self.root_cutout)
    }

    /// Local solidity `sigma(x) = n_blades * chord(x) / (pi * R)`.
    pub fn local_solidity(&self, x: f64) -> f64 {
        self.n_blades as f64 * self.chord(x) / (PI * self.radius)
    }

    /// Disk area, m^2.
    pub fn disk_area(&self) -> f64 {
        PI * self.radius * self.radius
    }

    /// Nominal solidity using the tip chord; a single number for reporting.
    pub fn solidity(&self) -> f64 {
        self.n_blades as f64 * self.tip_chord / (PI * self.radius)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn caradonna_tung_solidity() {
        let r = Rotor::rectangular(2, 1.143, 0.191, 0.0, 0.2);
        // sigma = 2 * 0.191 / (pi * 1.143) ~ 0.1064
        assert!((r.solidity() - 0.10638).abs() < 1e-4);
    }

    #[test]
    fn twist_applies_from_cutout() {
        let mut r = Rotor::rectangular(2, 1.0, 0.1, 0.1, 0.2);
        r.twist_rate = -0.2;
        assert!((r.pitch(0.2) - 0.1).abs() < 1e-12);
        assert!((r.pitch(1.0) - (0.1 - 0.2 * 0.8)).abs() < 1e-12);
    }
}
