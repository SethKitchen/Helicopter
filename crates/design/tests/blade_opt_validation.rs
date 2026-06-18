//! Blade twist/taper optimization validated against the minimum-induced-loss
//! oracle: induced power is least when the inflow is uniform across the disk, which
//! ideal twist (θ∝1/x) achieves. The buildable linear blade can only approach it —
//! so the test checks the *ordering* (ideal beats optimized-linear beats untwisted)
//! and the honest residual gap, not a single fabricated target.

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_design::BladeProblem;
use helisim_rotor::Operating;
use std::f64::consts::PI;

/// A representative model-scale rotor problem. Tip loss is OFF so the comparison
/// matches the clean combined-BEMT theory the ideal-twist result is derived in.
fn problem(af: &LinearAirfoil) -> BladeProblem<'_> {
    let radius = 0.7;
    let n_blades = 2;
    let solidity = 0.07;
    let mean_chord = solidity * PI * radius / n_blades as f64;
    BladeProblem {
        n_blades,
        radius_m: radius,
        mean_chord_m: mean_chord,
        root_cutout: 0.15,
        op: Operating::from_tip_mach(100.0 / 340.0, radius), // ~100 m/s tip
        airfoil: af,
        target_thrust_n: 30.0, // ~3 kg-class, comfortably liftable
        cfg: Config {
            tip_loss: false,
            ..Config::default()
        },
    }
}

/// ANCHOR: ideal twist drives the inflow coefficient of variation toward zero
/// (uniform inflow — the exact minimum-induced-power condition), far below an
/// untwisted blade at the same thrust.
#[test]
fn ideal_twist_makes_inflow_uniform() {
    let af = LinearAirfoil::naca0012();
    let p = problem(&af);
    let ideal = p.ideal_twist_anchor().expect("ideal anchor trims");
    let flat = p.evaluate(0.0, 1.0).expect("untwisted trims");

    println!(
        "inflow CV: ideal {:.4} vs untwisted {:.4}",
        ideal.inflow_cv, flat.inflow_cv
    );
    assert!(
        ideal.inflow_cv < 0.05,
        "ideal twist nearly uniform (CV {:.4})",
        ideal.inflow_cv
    );
    assert!(
        flat.inflow_cv > 3.0 * ideal.inflow_cv,
        "untwisted blade markedly less uniform than ideal"
    );
}

/// OPTIMIZER: minimizing power over linear twist + taper at fixed thrust reduces
/// BOTH the power and the inflow non-uniformity versus an untwisted blade, and the
/// optimum is a washout (negative twist) — the physically expected result.
#[test]
fn optimized_linear_blade_beats_untwisted() {
    let af = LinearAirfoil::naca0012();
    let p = problem(&af);
    let flat = p.evaluate(0.0, 1.0).expect("untwisted trims");
    let opt = p.optimize();

    println!(
        "power W: untwisted {:.2} → optimized {:.2} (twist {:.1}°, taper {:.2}); CV {:.4} → {:.4}",
        flat.power_w,
        opt.power_w,
        opt.twist_rate.to_degrees(),
        opt.taper_ratio,
        flat.inflow_cv,
        opt.inflow_cv
    );
    assert!(
        opt.power_w <= flat.power_w + 1e-9,
        "optimization must not increase power"
    );
    assert!(
        opt.power_w < flat.power_w,
        "optimization should reduce power vs untwisted"
    );
    assert!(
        opt.inflow_cv < flat.inflow_cv,
        "optimization should flatten the inflow"
    );
    assert!(
        opt.twist_rate < 0.0,
        "optimum is a washout (tip pitched down)"
    );
}

/// HONEST GAP + induced-power ordering: the linear optimum cannot reach the
/// hyperbolic ideal's uniformity (residual CV is the cost of a buildable blade), and
/// the ideal's power is the lowest of the three — the minimum-induced-loss bound.
#[test]
fn linear_optimum_approaches_but_cannot_reach_ideal() {
    let af = LinearAirfoil::naca0012();
    let p = problem(&af);
    let ideal = p.ideal_twist_anchor().expect("ideal anchor trims");
    let opt = p.optimize();

    // Linear is more uniform than untwisted (checked above) but still short of ideal.
    assert!(
        opt.inflow_cv > ideal.inflow_cv,
        "a linear blade cannot reach hyperbolic uniformity (opt CV {:.4} vs ideal {:.4})",
        opt.inflow_cv,
        ideal.inflow_cv
    );
    // Ideal twist is the minimum-INDUCED-power bound: its induced C_P is no greater
    // than the linear optimum's (total power is NOT bounded — ideal twist's singular
    // root pitch inflates profile drag; that is the point of splitting the two).
    println!(
        "induced C_P: ideal {:.6} ≤ linear-opt {:.6}; total power ideal {:.1} W vs opt {:.1} W",
        ideal.induced_cp, opt.induced_cp, ideal.power_w, opt.power_w
    );
    assert!(
        ideal.induced_cp <= opt.induced_cp + 1e-6,
        "ideal twist induced C_P {:.6} should bound the linear optimum {:.6}",
        ideal.induced_cp,
        opt.induced_cp
    );
}
