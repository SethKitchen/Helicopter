//! End-to-end coupling tests: rotor aero → powertrain → pack → endurance.
//!
//! These assert the chain produces a coherent answer and surfaces the real
//! design tension (more weight → more power → higher C-rate → less endurance).

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_cell::TheveninCell;
use helisim_mission::{MissionConfig, MissionScenario, analyze_climb, analyze_hover};
use helisim_pack::Pack;
use helisim_powertrain::ConstantEfficiency;
use helisim_rotor::{Operating, Rotor};
use helisim_thermal::{Convective, ThermalLimits};

/// A representative small electric helicopter: ~1 m diameter, 2 blades, 2200 RPM,
/// driven by a 6S3P pack of Samsung 25R cells through an 80%-efficient driveline.
fn rotor_op_air() -> (Rotor, Operating, LinearAirfoil) {
    let rotor = Rotor::rectangular(2, 0.5, 0.045, 0.0, 0.15);
    let op = Operating::from_rpm(2200.0);
    (rotor, op, LinearAirfoil::naca0012())
}

fn pack_6s3p() -> Pack {
    Pack::new(Box::new(TheveninCell::samsung_25r()), 6, 3)
}

#[test]
fn hover_is_feasible_and_within_rating() {
    let (rotor, op, af) = rotor_op_air();
    let pack = pack_6s3p();
    let pt = ConstantEfficiency::typical_electric_heli();
    let scen = MissionScenario {
        rotor: &rotor,
        op: &op,
        airfoil: &af,
        pack: &pack,
        powertrain: &pt,
        cooling: &Convective::natural_air(),
        limits: ThermalLimits::default(),
        bemt_cfg: &Config::default(),
        mission_cfg: &MissionConfig::default(),
    };
    let rep = analyze_hover(&scen, 3.0);

    assert!(rep.hover_feasible, "should trim to hover");
    assert!(rep.mech_power_w > 0.0 && rep.elec_power_w > rep.mech_power_w);
    assert!(rep.hover_pack_current > 0.0);
    assert!(
        rep.within_continuous_rating,
        "hover should be within the 8C rating"
    );
    assert!(rep.hover_cell_c_rate < pack.continuous_c_rating());
    assert!(rep.endurance.feasible && rep.endurance.endurance_min > 1.0);
}

#[test]
fn design_tension_heavier_costs_power_and_endurance() {
    let (rotor, op, af) = rotor_op_air();
    let pt = ConstantEfficiency::typical_electric_heli();
    let cool = Convective::natural_air();
    let pack = pack_6s3p();
    let scen = MissionScenario {
        rotor: &rotor,
        op: &op,
        airfoil: &af,
        pack: &pack,
        powertrain: &pt,
        cooling: &cool,
        limits: ThermalLimits::default(),
        bemt_cfg: &Config::default(),
        mission_cfg: &MissionConfig::default(),
    };

    let light = analyze_hover(&scen, 2.5);
    let heavy = analyze_hover(&scen, 4.0);

    assert!(light.hover_feasible && heavy.hover_feasible);
    // More weight → more collective, power, current, C-rate; less endurance.
    assert!(heavy.collective_deg > light.collective_deg);
    assert!(heavy.mech_power_w > light.mech_power_w);
    assert!(heavy.hover_cell_c_rate > light.hover_cell_c_rate);
    assert!(heavy.endurance.endurance_min < light.endurance.endurance_min);
}

#[test]
fn hover_stays_cool_but_sustained_climb_overheats() {
    // The safety insight: on a hot day with a realistically-sized (6S2P) pack,
    // hover is thermally fine, but a sustained climb drives the cell over its
    // temperature limit — and can do so while the current is still within the
    // C-rate rating, which the old C-rate-only check would have called safe.
    let (rotor, op, af) = rotor_op_air();
    let pt = ConstantEfficiency::typical_electric_heli();
    let cool = Convective::natural_air();
    let lim = ThermalLimits::default();
    let cfg = Config::default();
    let mcfg = MissionConfig {
        ambient_c: 30.0,
        ..MissionConfig::default()
    };
    let pack = Pack::new(Box::new(TheveninCell::samsung_25r()), 6, 2);
    let mass = 5.0;
    let scen = MissionScenario {
        rotor: &rotor,
        op: &op,
        airfoil: &af,
        pack: &pack,
        powertrain: &pt,
        cooling: &cool,
        limits: lim,
        bemt_cfg: &cfg,
        mission_cfg: &mcfg,
    };

    let h = analyze_hover(&scen, mass);
    assert!(
        h.hover_feasible && h.hover_peak_temp_c < lim.max_c,
        "hover should stay under limit"
    );

    let c = analyze_climb(&scen, mass, 6.0, 360.0);
    assert!(c.feasible);
    assert!(
        c.peak_temp_c > lim.max_c,
        "sustained climb should exceed thermal limit"
    );
    // ...and the climb is still within the C-rate rating: thermal bites first.
    assert!(
        c.within_c_rating,
        "climb C-rate {:.1} should be within rating",
        c.cell_c_rate
    );
    assert!(c.time_to_over_temp_s.is_some());
}

#[test]
fn forced_cooling_helps() {
    let (rotor, op, af) = rotor_op_air();
    let pt = ConstantEfficiency::typical_electric_heli();
    let lim = ThermalLimits::default();
    let cfg = Config::default();
    let mcfg = MissionConfig {
        ambient_c: 30.0,
        ..MissionConfig::default()
    };
    let pack = Pack::new(Box::new(TheveninCell::samsung_25r()), 6, 2);
    let base = MissionScenario {
        rotor: &rotor,
        op: &op,
        airfoil: &af,
        pack: &pack,
        powertrain: &pt,
        cooling: &Convective::natural_air(),
        limits: lim,
        bemt_cfg: &cfg,
        mission_cfg: &mcfg,
    };
    let natural = analyze_climb(&base, 5.0, 6.0, 360.0);
    let forced = analyze_climb(
        &MissionScenario {
            cooling: &Convective::forced_air(),
            ..base
        },
        5.0,
        6.0,
        360.0,
    );
    assert!(
        forced.peak_temp_c < natural.peak_temp_c,
        "forced air should run cooler"
    );
}

#[test]
fn overload_is_reported_not_panicked() {
    let (rotor, op, af) = rotor_op_air();
    let pack = pack_6s3p();
    let pt = ConstantEfficiency::typical_electric_heli();
    // Far beyond what a 0.5 m rotor at 2200 RPM can lift.
    let rep = analyze_hover(
        &MissionScenario {
            rotor: &rotor,
            op: &op,
            airfoil: &af,
            pack: &pack,
            powertrain: &pt,
            cooling: &Convective::natural_air(),
            limits: ThermalLimits::default(),
            bemt_cfg: &Config::default(),
            mission_cfg: &MissionConfig::default(),
        },
        50.0,
    );
    assert!(!rep.hover_feasible);
    assert!(!rep.endurance.feasible);
}

#[test]
fn infeasible_mass_reports_not_feasible_without_panicking() {
    // A mass far beyond what the small rotor can lift exercises the infeasible
    // early-return paths of analyze_hover and analyze_climb (no trim solution).
    let (rotor, op, af) = rotor_op_air();
    let pt = ConstantEfficiency::typical_electric_heli();
    let huge = 1000.0; // kg — impossible for a 0.5 m model rotor
    let pack = pack_6s3p();
    let scen = MissionScenario {
        rotor: &rotor,
        op: &op,
        airfoil: &af,
        pack: &pack,
        powertrain: &pt,
        cooling: &Convective::natural_air(),
        limits: ThermalLimits::default(),
        bemt_cfg: &Config::default(),
        mission_cfg: &MissionConfig::default(),
    };
    let h = analyze_hover(&scen, huge);
    assert!(!h.hover_feasible);
    assert!(h.collective_deg.is_nan());

    let c = analyze_climb(&scen, huge, 5.0, 60.0);
    assert!(!c.feasible);
}
