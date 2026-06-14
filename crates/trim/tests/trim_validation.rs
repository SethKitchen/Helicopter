//! Trim validation.
//!
//! Primary (cross-check) oracle: hover trim — reached through the *full
//! force/moment Newton solve* — must land on the same main-rotor collective and
//! power that the independent hover-BEMT path (milestone 1) gives by inverting
//! thrust = weight directly. Two independent routes agreeing is the strongest
//! validation. System checks: tail rotor balances main torque; forward flight
//! shows the classic trends (collective down, longitudinal stick forward,
//! nose-down attitude with speed).

use helisim_bemt::{Config, solve_hover};
use helisim_trim::{Aircraft, NewtonConfig, TrimCondition, trim};

const G: f64 = 9.80665;

/// Independent hover BEMT trim: the collective that makes thrust = weight, and
/// the resulting main-rotor power.
fn hover_bemt(ac: &Aircraft) -> (f64, f64) {
    let w = ac.mass * G;
    let thrust = |th: f64| {
        solve_hover(
            &ac.main.with_collective(th),
            &ac.main_op,
            ac.main_airfoil.as_ref(),
            &Config::default(),
        )
        .thrust
    };
    let (mut lo, mut hi) = (0.0_f64, 20f64.to_radians());
    for _ in 0..80 {
        let m = 0.5 * (lo + hi);
        if thrust(m) < w {
            lo = m;
        } else {
            hi = m;
        }
    }
    let th = 0.5 * (lo + hi);
    let s = solve_hover(
        &ac.main.with_collective(th),
        &ac.main_op,
        ac.main_airfoil.as_ref(),
        &Config::default(),
    );
    (th, s.power)
}

#[test]
fn hover_trim_matches_standalone_bemt() {
    let ac = Aircraft::demo();
    let (th_bemt, p_bemt) = hover_bemt(&ac);
    let r = trim(&ac, &TrimCondition::hover(), &NewtonConfig::default());

    assert!(r.converged);
    // Same collective (small difference from the trim's slight roll attitude).
    assert!(
        (r.collective - th_bemt).abs() < 1.5f64.to_radians() * 0.1 + 0.01,
        "collective {:.4} vs BEMT {:.4}",
        r.collective,
        th_bemt
    );
    // Same main-rotor power to within 2% — the capstone cross-check.
    assert!(
        (r.main_power - p_bemt).abs() / p_bemt < 0.02,
        "main power {:.1} vs BEMT {:.1}",
        r.main_power,
        p_bemt
    );
}

#[test]
fn hover_tail_rotor_balances_main_torque() {
    let ac = Aircraft::demo();
    let r = trim(&ac, &TrimCondition::hover(), &NewtonConfig::default());
    let main_torque = r.main_power / ac.main_op.omega;
    // Yaw balance: T_tr · arm ≈ Q_main.
    assert!((r.tail_thrust * ac.tail.arm - main_torque).abs() < 0.2);
    assert!(r.tail_thrust > 0.0 && r.tail_power > 0.0);
}

#[test]
fn hover_thrust_supports_weight() {
    let ac = Aircraft::demo();
    let r = trim(&ac, &TrimCondition::hover(), &NewtonConfig::default());
    let w = ac.mass * G;
    assert!((r.thrust - w).abs() / w < 0.02);
}

#[test]
fn forward_flight_classic_trends() {
    // Moderate speeds (μ ≲ 0.12), where the one-way inflow coupling is sound.
    let ac = Aircraft::demo();
    let cfg = NewtonConfig::default();
    let hover = trim(&ac, &TrimCondition::hover(), &cfg);
    let v10 = trim(&ac, &TrimCondition::forward(10.0), &cfg);
    let v15 = trim(&ac, &TrimCondition::forward(15.0), &cfg);

    assert!(v10.converged && v15.converged);

    // Collective drops with forward speed (translational lift / lower induced power).
    assert!(v10.collective < hover.collective);
    assert!(v15.collective < v10.collective);

    // Longitudinal stick moves forward (more negative θ1s) with speed.
    assert!(v15.cyclic_lon < v10.cyclic_lon);

    // Fuselage pitches nose-down (more negative) with speed.
    assert!(v15.pitch < v10.pitch && v10.pitch < 0.0);

    // Thrust still supports the weight.
    let w = ac.mass * G;
    assert!((v10.thrust - w).abs() / w < 0.05);
}

#[test]
fn power_is_positive_well_past_old_breakdown() {
    // The exact regime that went negative with one-way coupling (μ ≈ 0.16–0.25)
    // must now be physical: both rotor and total power strictly positive, and the
    // collective stays sensible (not driven negative). This is the 5b target.
    let ac = Aircraft::demo();
    let cfg = NewtonConfig::default();
    for &v in &[20.0, 25.0, 30.0] {
        let r = trim(&ac, &TrimCondition::forward(v), &cfg);
        assert!(r.converged, "v={v} should converge");
        assert!(r.mu > 0.15, "v={v} should be past μ=0.13");
        assert!(
            r.main_power > 0.0,
            "main power {} must be positive at v={v}",
            r.main_power
        );
        assert!(r.total_power > 0.0);
        assert!(
            r.collective > 0.0,
            "collective {} should stay positive",
            r.collective
        );
    }
}

#[test]
fn complete_power_bucket() {
    // With two-way coupling (physical rotor power) + airframe parasite, the
    // trimmed power vs speed shows the full bucket: hover-high → interior
    // minimum → high-speed rise.
    let ac = Aircraft::demo();
    let cfg = NewtonConfig::default();
    let speeds: Vec<f64> = (0..=8).map(|k| k as f64 * 5.0).collect(); // 0..40
    let power: Vec<f64> = speeds
        .iter()
        .map(|&v| trim(&ac, &TrimCondition::forward(v), &cfg).total_power)
        .collect();

    let (min_i, &min_p) = power
        .iter()
        .enumerate()
        .min_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .unwrap();
    assert!(
        min_i > 0 && min_i < power.len() - 1,
        "bucket minimum should be interior"
    );
    assert!(min_p < power[0], "minimum power below hover");
    assert!(
        *power.last().unwrap() > min_p * 1.5,
        "power should rise well above the minimum"
    );
}
