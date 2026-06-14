//! Aircraft definition for trim: main rotor, tail rotor, mass and geometry.

use helisim_airfoil::{Airfoil, LinearAirfoil};
use helisim_flapping::FlapProperties;
use helisim_rotor::{Operating, Rotor};

/// Tail rotor: a small rotor whose thrust balances main-rotor torque in yaw.
pub struct TailRotor {
    /// Tail rotor geometry.
    pub rotor: Rotor,
    /// Tail rotor operating point (own RPM).
    pub op: Operating,
    /// Sectional aero.
    pub airfoil: Box<dyn Airfoil>,
    /// Longitudinal arm from CG to the tail-rotor hub, m (positive aft).
    pub arm: f64,
    /// Height of the tail-rotor hub above the CG, m.
    pub height: f64,
}

/// A conventional single-main-rotor helicopter.
pub struct Aircraft {
    /// Main rotor geometry (collective is overwritten during trim).
    pub main: Rotor,
    /// Main rotor operating point.
    pub main_op: Operating,
    /// Main rotor sectional aero.
    pub main_airfoil: Box<dyn Airfoil>,
    /// Main rotor flap properties (Lock number, hinge offset).
    pub flap: FlapProperties,
    /// Tail rotor.
    pub tail: TailRotor,
    /// Gross mass, kg.
    pub mass: f64,
    /// Air density, kg/m³.
    pub rho: f64,
    /// Main-rotor hub height above the CG, m.
    pub hub_height: f64,
    /// Longitudinal CG offset aft of the shaft, m (positive aft).
    pub cg_offset: f64,
    /// Equivalent flat-plate parasite area, m² (simple; refined in 5b).
    pub parasite_area: f64,
}

impl Aircraft {
    /// A representative small electric helicopter, used as the default test
    /// aircraft. Main rotor R=0.8 m, 2 blades, 1500 RPM; 8 kg gross.
    pub fn demo() -> Self {
        let main = Rotor::rectangular(2, 0.8, 0.06, 8f64.to_radians(), 0.15);
        let main_op = Operating::from_rpm(1500.0);
        let tail = TailRotor {
            rotor: Rotor::rectangular(2, 0.15, 0.025, 6f64.to_radians(), 0.15),
            op: Operating::from_rpm(4000.0),
            airfoil: Box::new(LinearAirfoil::naca0012()),
            arm: 0.95,
            height: 0.10,
        };
        Aircraft {
            main,
            main_op,
            main_airfoil: Box::new(LinearAirfoil::naca0012()),
            flap: FlapProperties::with_offset(8.0, 0.04),
            tail,
            mass: 8.0,
            rho: 1.225,
            hub_height: 0.25,
            cg_offset: 0.0,
            parasite_area: 0.05,
        }
    }

    /// The UH-60A Black Hawk, built **strictly** from the Milestone-6 locked
    /// parameter mapping (`crates/validation/MILESTONE6_PARAMETER_MAPPING.md`),
    /// sourced from NASA TM 85890 (GENHEL). No value here may be tuned toward the
    /// comparison oracle — every number is the report's, converted to SI, or a
    /// documented physics-based mapping decision.
    ///
    /// Direct from Table 1: R=8.178 m, c=0.527 m, 4 blades, Ω=27.0 rad/s,
    /// γ=8.1936, hinge e=0.04659, twist −0.3142 rad, a=5.73 /rad; mass 7439 kg.
    /// Judgment calls (locked): SC1095→NACA-0012-class at a=5.73; root cutout 0.15
    /// (not in Table 1; representative); TR chord from σ_TR=0.1875, 4 blades;
    /// canted TR / stabilator / Ixz omitted here (named error sources — they do not
    /// affect the hover *longitudinal* derivatives, which are main-rotor-dominated);
    /// parasite 3.25 m² (~35 ft², only matters in forward flight). Geometry
    /// (hub height 1.722 m, tail arm 9.44 m, tail height 1.969 m) from station/
    /// waterlines.
    pub fn uh60() -> Self {
        let mut main = Rotor::rectangular(4, 8.178, 0.527, 0.0, 0.15);
        main.twist_rate = -0.3142;
        let mut tail_blade = Rotor::rectangular(4, 1.676, 0.247, 0.0, 0.15);
        tail_blade.twist_rate = -0.3142;
        let tail = TailRotor {
            rotor: tail_blade,
            op: Operating::from_rpm(1190.0), // Ω = 124.62 rad/s
            airfoil: Box::new(LinearAirfoil::naca0012()),
            arm: 9.44,
            height: 1.969,
        };
        // Flap with the validated gyroscopic rate-damping term (Milestone 6): the
        // "rotor-follows-shaft" coupling, coefficient −2 (derived, not fitted).
        let mut flap = FlapProperties::with_offset(8.1936, 0.04659);
        flap.gyro_rate = -2.0;
        Aircraft {
            main,
            main_op: Operating::from_rpm(257.83), // Ω = 27.0 rad/s
            main_airfoil: Box::new(LinearAirfoil::naca0012()),
            flap,
            tail,
            mass: 7439.0,
            rho: 1.225,
            hub_height: 1.722,
            cg_offset: 0.0,
            parasite_area: 3.25,
        }
    }
}
