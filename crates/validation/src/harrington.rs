//! Secondary check: the Harrington (1951) Rotor 1, the traditional
//! figure-of-merit hover benchmark.
//!
//! Source: R. D. Harrington, "Full-Scale-Tunnel Investigation of the
//! Static-Thrust Performance of a Coaxial Helicopter Rotor," NACA TN-2318 /
//! related NACA TN-2474, 1951.
//!
//! Geometry (single rotor): 2 blades, radius 12.5 ft (3.81 m), thrust-weighted
//! solidity ≈ 0.027, symmetric NACA sections with thickness taper (very thick
//! inboard, ~12% at the tip). BEMT cannot capture the thickness taper exactly,
//! so this rotor is modelled with an equivalent rectangular blade matching the
//! solidity. It is a *figure-of-merit* benchmark, not a C_T-vs-collective table —
//! the accepted result is a hover peak figure of merit of roughly 0.70 for
//! Rotor 1. We therefore expose the geometry and the expected FM band rather
//! than fabricate a collective→C_T oracle.

use crate::oracle::{OraclePoint, ValidationCase};
use helisim_airfoil::{Airfoil, LinearAirfoil};
use helisim_rotor::Rotor;
use std::f64::consts::PI;

/// Harrington Rotor 1, modelled as an equivalent rectangular rotor.
#[derive(Clone, Copy, Debug)]
pub struct HarringtonRotor1 {
    /// Rotor radius, m.
    pub radius: f64,
    /// Thrust-weighted solidity to match.
    pub solidity: f64,
    /// Number of blades.
    pub n_blades: usize,
    /// Root cutout, fraction of radius.
    pub root_cutout: f64,
    /// Tip Mach number representative of the static-thrust tests.
    pub tip_mach: f64,
}

impl Default for HarringtonRotor1 {
    fn default() -> Self {
        HarringtonRotor1 {
            radius: 3.81,
            solidity: 0.027,
            n_blades: 2,
            root_cutout: 0.20,
            tip_mach: 0.45,
        }
    }
}

impl HarringtonRotor1 {
    /// Equivalent rectangular chord that reproduces the target solidity:
    /// `c = sigma * pi * R / n_blades`.
    pub fn equivalent_chord(&self) -> f64 {
        self.solidity * PI * self.radius / self.n_blades as f64
    }

    /// Published hover peak figure-of-merit band for Rotor 1 (inclusive).
    pub fn expected_peak_fm(&self) -> (f64, f64) {
        (0.62, 0.75)
    }
}

impl ValidationCase for HarringtonRotor1 {
    fn name(&self) -> &str {
        "Harrington (1951) Rotor 1"
    }

    fn description(&self) -> &str {
        "2-blade rotor, R=3.81 m, sigma=0.027, figure-of-merit hover benchmark"
    }

    fn build_rotor(&self, collective_rad: f64) -> Rotor {
        Rotor::rectangular(
            self.n_blades,
            self.radius,
            self.equivalent_chord(),
            collective_rad,
            self.root_cutout,
        )
    }

    fn airfoil(&self) -> Box<dyn Airfoil> {
        Box::new(LinearAirfoil::naca0012())
    }

    fn oracle_points(&self) -> Vec<OraclePoint> {
        // Harrington's published data is FM vs C_T/sigma, not C_T vs collective.
        // We do not fabricate a collective→C_T table; the FM band is checked via
        // a collective sweep in the CLI instead. See `expected_peak_fm`.
        Vec::new()
    }

    fn notes(&self) -> Option<String> {
        let (lo, hi) = self.expected_peak_fm();
        Some(format!(
            "Figure-of-merit benchmark: expected hover peak FM in [{lo:.2}, {hi:.2}]. \
             Modelled as an equivalent rectangular blade (chord {:.4} m) matching \
             solidity {:.3}; thickness taper not represented.",
            self.equivalent_chord(),
            self.solidity
        ))
    }
}
