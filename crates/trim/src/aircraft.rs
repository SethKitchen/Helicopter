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
    /// Forward longitudinal shaft tilt, rad (positive = mast leans forward). The rotor
    /// thrust is resolved into body axes through this tilt, so in hover the fuselage must
    /// pitch ~`shaft_tilt` nose-up to keep thrust vertical. **Default 0** (every prior
    /// milestone & the demo aircraft unchanged); set to the sourced 3° on the real
    /// aircraft (Milestone 6). Added to TEST the pre-registered backward-reaching claim
    /// that the UH-60 `cg_offset` was over-attributing the missing shaft-tilt nose-up
    /// (`validation/MILESTONE6_SHAFT_TILT_PREREG.md`).
    pub shaft_tilt: f64,
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
            shaft_tilt: 0.0,
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
            // CG 0.488 m aft of the hub (Table 1 stationlines: CG STA 360.4 − hub
            // STA 341.2 = 19.2 in). Sourced, not fitted; corrects an initial under-use
            // of the data (locked 0). Drives the nose-up hover attitude; does not
            // enter the derivative computation (trim-only), so 5/6 derivs unaffected.
            cg_offset: 0.488,
            parasite_area: 3.25,
            // Sourced 3° fwd shaft tilt (TM 85890 Table 1, 0.05236 rad). Set to TEST the
            // pre-registered claim (MILESTONE6_SHAFT_TILT_PREREG.md) that cg_offset=0.488
            // was over-attributing this missing nose-up term. NOT re-tuned to hit +5.05°.
            shaft_tilt: 0.05236,
        }
    }

    /// The Boeing-Vertol BO-105C, the **second** external-validation airframe, built
    /// **strictly** from the locked mapping (`MILESTONE6_BO105_PREREG.md`), sourced
    /// from NASA CR-3144 (Heffley). The point: a **hingeless** rotor — an adversarial
    /// test of whether the gyroscopic flap-damping term (`gyro_rate=−2`, derived on the
    /// *articulated* UH-60) generalizes to completely different hub mechanics with
    /// parameter changes alone. No value here is tuned toward the oracle.
    ///
    /// Direct from Table III-1: R=4.91 m, c=0.27 m, 4 blades, hingeless, twist −8°
    /// (−0.1396 rad), shaft tilt 3° fwd, 424 rpm (Ω=44.40 rad/s) for the tabulated data,
    /// blade flap inertia I_β=219.50 kg·m². Tail: 2 blades, R=0.95 m, c=0.18 m, gear
    /// ratio 5.24 (Ω_tr=232.7 rad/s). Mass/inertia (Fig III-3b, CASE-29 nominal): 2096
    /// kg, Ixx 1803, Iyy 4892, Izz 4428 kg·m², Ixz 0 (matches our diagonal model).
    /// Computed: γ = ρ·a·c·R⁴/I_β = 5.02 (a=5.73, NACA 23012→NACA-class per mapping #1).
    /// Judgment calls (locked): ν_β=1.12 (hinge offset e=0.145) — the hingeless flap
    /// frequency, set to the bracket MIDPOINT [1.08,1.15] since CR-3144 omits it; the
    /// gyro conclusion is verified ACROSS the whole bracket in the test, not at this one
    /// value. `gyro_rate=−2` is the UNCHANGED UH-60 value (this is the test). hub_height
    /// 0.95 m is NOT cleanly sourced (CG waterline absent from CR-3144); the headline
    /// Mq/Lp is hub-spring-dominated (~95%) so insensitive to it — demonstrated in the
    /// test. Geometry: tail arm 5.99 m (FS 335 − CG FS 99.15), tail height 1.14 m,
    /// cg_offset 0.018 m (CG ~under the hub; trim-only). NO canted TR, NO pitch-bias
    /// actuator, NO stabilator modeled (hover) — all UH-60 confounds absent here.
    pub fn bo105() -> Self {
        let mut main = Rotor::rectangular(4, 4.91, 0.27, 0.0, 0.15);
        main.twist_rate = -8f64.to_radians(); // −0.1396 rad
        let mut tail_blade = Rotor::rectangular(2, 0.95, 0.18, 0.0, 0.15);
        tail_blade.twist_rate = 0.0;
        let tail = TailRotor {
            rotor: tail_blade,
            op: Operating::from_rpm(2221.8), // Ω = 424·5.24·(2π/60) = 232.7 rad/s
            airfoil: Box::new(LinearAirfoil::naca0012()),
            arm: 5.99,
            height: 1.14,
        };
        // Hingeless flap: ν_β=1.12 (e=0.145), the UH-60-derived gyro term UNCHANGED (−2).
        let mut flap = FlapProperties::with_offset(5.02, 0.145);
        flap.gyro_rate = -2.0;
        Aircraft {
            main,
            main_op: Operating::from_rpm(424.0), // Ω = 44.40 rad/s
            main_airfoil: Box::new(LinearAirfoil::naca0012()),
            flap,
            tail,
            mass: 2096.0,
            rho: 1.225,
            hub_height: 0.95,
            cg_offset: 0.018,
            parasite_area: 1.4, // hover-irrelevant (∝½ρV³→0); representative
            // Sourced 3° fwd shaft tilt (CR-3144 Table III-1). With cg_offset≈0 here, this
            // is the BO-105's nose-up driver — predicted to recover Θ→~+2.6° (oracle +2.64).
            shaft_tilt: 3f64.to_radians(),
        }
    }

    /// The Hughes OH-6A — the **third** external airframe, built as the GAIN DISCRIMINATOR
    /// (MILESTONE6_OH6A_GAIN_PREREG.md) for the UH-60 attitude over-response. CR-3144 gives
    /// it at hover at three cg positions (FWD/MID/AFT), so dΘ/d(cg_offset) directly measures
    /// the cg→attitude gain (shaft tilt cancels in the slope). Strictly from CR-3144, no
    /// tuning. `cg_offset` is the SWEEP variable — set per case by the discriminator test.
    ///
    /// Table II-1: R=4.013 m, c=0.171 m, 4 blades, NACA 0015 (→NACA-class a=5.73), articulated,
    /// twist −8°, shaft tilt 3° fwd, hub FS 100 / WL 83, I_β=63.49 kg·m². Fig II-3 (nominal):
    /// mass 1157 kg, Iyy 1219, cg FS 97–104 (mid=100=hub), CG WL 49.6. ⇒ hub_height = (83−49.6)
    /// in = 0.848 m (SOURCED — both WLs given, unlike the BO-105). Tail: 2 blades R=0.648 m,
    /// gear 6.447, hub FS 282/WL 54.3 ⇒ arm 4.62 m, height 0.119 m. γ=ρacR⁴/I_β=4.9.
    /// Articulated hinge offset NOT in CR-3144 ⇒ e=0.03 nominal (ν_β≈1.05); the discriminator
    /// brackets it. `gyro_rate=−2` (validated general; zero effect on the trim attitude since
    /// it forces only on body rate). shaft_tilt 3° sourced.
    pub fn oh6a() -> Self {
        let mut main = Rotor::rectangular(4, 4.013, 0.171, 0.0, 0.15);
        main.twist_rate = -8f64.to_radians();
        let mut tail_blade = Rotor::rectangular(2, 0.648, 0.12, 0.0, 0.15);
        tail_blade.twist_rate = -8f64.to_radians();
        let tail = TailRotor {
            rotor: tail_blade,
            op: Operating::from_rpm(3114.0), // Ω = 483·6.447·(2π/60) = 326 rad/s
            airfoil: Box::new(LinearAirfoil::naca0012()),
            arm: 4.62,
            height: 0.119,
        };
        let mut flap = FlapProperties::with_offset(4.9, 0.03);
        flap.gyro_rate = -2.0;
        Aircraft {
            main,
            main_op: Operating::from_rpm(483.0), // Ω = 50.58 rad/s
            main_airfoil: Box::new(LinearAirfoil::naca0012()),
            flap,
            tail,
            mass: 1157.0,
            rho: 1.225,
            hub_height: 0.848,
            cg_offset: 0.0, // SWEEP variable — set per case (FWD −0.0762, MID 0, AFT +0.1016)
            parasite_area: 1.0, // hover-irrelevant
            shaft_tilt: 3f64.to_radians(),
        }
    }
}
