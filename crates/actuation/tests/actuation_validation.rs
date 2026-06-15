//! Validation for actuation selection.
//!
//! Character (like `manufacture::fasteners`): the catalogues match published
//! datasheets (external oracle), the selection is **falsifiable** (chosen passes,
//! next-down fails), the control-load is hand-checkable + scales correctly, and
//! the scale behaviour is honest (model → real mini parts; human-scale → flagged
//! beyond-catalogue, not a faked match).

use helisim_actuation::loads::{THETA_MAX_RAD, propeller_moment_nm, servo_torque_demand};
use helisim_actuation::motor::{BldcMotor, scorpion_hk_catalogue};
use helisim_actuation::plan::{ActuationConfig, select_actuation, select_actuation_with};
use helisim_actuation::scaling::size_or_extrapolate;
use helisim_actuation::selectable::{Selectable, select_smallest_adequate};
use helisim_actuation::servo::{align_hv_catalogue, kgcm_to_nm};
use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_design::{DesignCandidate, evaluate};

/// DATASHEET ORACLE — Scorpion HKII-4525-520 and Align DS820, hand-checkable
/// against the published spec sheets.
#[test]
fn catalogues_match_datasheets() {
    let m = scorpion_hk_catalogue()
        .into_iter()
        .find(|m| m.name == "Scorpion HKII-4525-520")
        .unwrap();
    assert_eq!(
        (m.kv, m.mass_g, m.max_cont_power_w, m.max_cont_current_a),
        (520.0, 503.0, 4450.0, 100.0)
    );

    let s = align_hv_catalogue()
        .into_iter()
        .find(|s| s.name == "Align DS820")
        .unwrap();
    // 23 kg·cm @8.4 V → 2.256 N·m, 70 g.
    assert!((s.stall_torque_nm - kgcm_to_nm(23.0)).abs() < 1e-9);
    assert_eq!(s.mass_g, 70.0);
}

/// FALSIFIABLE — motor selection is the smallest (lightest) adequate, and the
/// next size down really fails. Demand between the 475 W and 1400 W members.
#[test]
fn motor_selection_is_smallest_adequate() {
    let cat = scorpion_hk_catalogue();
    let demand = 900.0; // W
    let chosen = select_smallest_adequate(&cat, demand, 1.0).unwrap();
    assert_eq!(chosen.name(), "Scorpion HKII-4525-520"); // only buyable member ≥ 900 W
    // The 475 W HKII-2221-8 cannot carry 900 W.
    let small = cat
        .iter()
        .find(|m| m.name == "Scorpion HKII-2221-8 V2")
        .unwrap();
    assert!(small.max_cont_power_w < demand);
    assert!(chosen.max_cont_power_w >= demand);
}

/// FALSIFIABLE — servo selection smallest-adequate. Demand between DS450 (0.392)
/// and DS825 (1.226 N·m).
#[test]
fn servo_selection_is_smallest_adequate() {
    let cat = align_hv_catalogue();
    let demand = 0.6; // N·m
    let chosen = select_smallest_adequate(&cat, demand, 1.0).unwrap();
    assert_eq!(chosen.name(), "Align DS825"); // lightest ≥ 0.6
    let small = cat.iter().find(|s| s.name == "Align DS450").unwrap();
    assert!(small.stall_torque_nm < demand);
}

/// The control-load demand for the model selects a *mini* cyclic servo and a
/// small Scorpion — physically reasonable for a ~3.5 kg heli.
#[test]
fn model_scale_selects_small_real_parts() {
    let c = DesignCandidate::model();
    let report = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
    let plan = select_actuation(&c, &report);

    let motor = plan.motor.part.expect("a real motor at model scale");
    assert!(!plan.motor.beyond_catalogue);
    // ~3.5 kg heli: hover ~0.6 kW × 1.6 margin ~ 1 kW → a 30-class Scorpion (<300 g).
    assert!(motor.mass_g < 300.0, "got {} g", motor.mass_g);

    let cyc = plan.cyclic_servo.part.expect("a real cyclic servo");
    assert!(!plan.cyclic_servo.beyond_catalogue);
    // Mini swashplate servo: the propeller moment is small at model scale.
    assert!(cyc.mass_g < 25.0, "got {} g", cyc.mass_g);

    // Total actuation mass is a small fraction of gross — a real heli proportion.
    assert!(plan.total_mass_kg < 0.5);
    assert!(plan.servo_mass_kg() > 0.0 && plan.motor_mass_kg() > 0.0);
}

/// Propeller-moment hand-check + scaling laws (mirrors the unit test, exercised
/// through the public API so the demand wiring is covered too).
#[test]
fn servo_demand_dominated_by_propeller_moment() {
    let c = DesignCandidate::model();
    let pm = propeller_moment_nm(&c, THETA_MAX_RAD);
    let total = servo_torque_demand(&c);
    // Propeller moment is the dominant term (aero hinge is the small remainder).
    assert!(pm / total > 0.6, "propeller fraction {}", pm / total);
    assert!((pm - 0.135).abs() < 0.02);
}

/// HONEST SCALING — the human-scale design outgrows the hobby catalogue: the
/// motor is flagged beyond-catalogue (an extrapolated mass, regime-change note),
/// not silently matched to a 700-class hobby motor.
#[test]
fn human_scale_flags_beyond_catalogue() {
    let c = DesignCandidate::human_scale_2pax();
    let report = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
    let plan = select_actuation(&c, &report);

    // ~700 kg aircraft needs tens of kW — far past the 4450 W ceiling.
    assert!(
        plan.motor.beyond_catalogue,
        "expected beyond-catalogue motor"
    );
    assert!(plan.motor.part.is_none());
    assert!(
        plan.notes
            .iter()
            .any(|n| n.contains("regime") || n.contains("exceeds"))
    );
    // An extrapolated mass is still returned (estimate, not a fake part).
    assert!(plan.motor.mass_g.is_finite() && plan.motor.mass_g > 503.0);
}

/// The override hook works: a custom (tiny) catalogue forces extrapolation even at
/// model scale, proving the defaults are swappable like `UnitCosts`.
#[test]
fn custom_catalogue_override() {
    let c = DesignCandidate::model();
    let report = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
    // A custom catalogue with only a too-weak toy motor → forced beyond-catalogue,
    // proving the defaults are swappable like `UnitCosts`.
    let tiny = vec![BldcMotor {
        name: "ToyMotor-50W",
        kv: 3000.0,
        mass_g: 20.0,
        max_cont_power_w: 50.0,
        max_cont_current_a: 10.0,
        max_cells: 2,
        stator_d_mm: 10.0,
        price_usd: 9.99,
        purchase_url: "https://example.com/toy",
        price_note: "test fixture",
    }];
    let plan = select_actuation_with(
        &c,
        &report,
        &tiny,
        &align_hv_catalogue(),
        ActuationConfig::default(),
    );
    assert!(plan.motor.beyond_catalogue);
}

/// Sanity that `size_or_extrapolate` returns a real part within range (guards the
/// scaling seam used by the plan).
#[test]
fn size_within_range_is_real() {
    let s = size_or_extrapolate(&align_hv_catalogue(), 0.3, 1.0, "cyclic servo");
    assert!(!s.beyond_catalogue && s.part.is_some());
}
