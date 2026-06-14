//! Forward-flight validation.
//!
//! Primary (analytic, non-circular) oracle: the Glauert induced-inflow closed
//! form anchors the inflow solve, exactly as the Leishman closed form anchored
//! hover. System-level qualitative check: the power-required-vs-speed "bucket"
//! (induced power falls with forward speed, so power at moderate speed dips below
//! hover, then rises). Headline result: a rigid (un-flapped) blade in forward
//! flight produces a large uncommanded rolling moment — the physical reason
//! flapping exists.

use helisim_airfoil::LinearAirfoil;
use helisim_forward::{
    ForwardCondition, ForwardConfig, ForwardSolution, glauert_inflow, glauert_inflow_closed_form,
    solve_forward,
};
use helisim_rotor::{Operating, Rotor};

fn rotor_op() -> (Rotor, Operating, LinearAirfoil) {
    // Caradonna & Tung geometry, reused from the hover oracle for continuity.
    let rotor = Rotor::rectangular(2, 1.143, 0.191, 8f64.to_radians(), 0.15);
    let op = Operating::from_tip_mach(0.439, 1.143);
    (rotor, op, LinearAirfoil::naca0012())
}

#[test]
fn dimensional_accessors_match_their_coefficients() {
    // The dimensional outputs are the coefficients × ρ·A·(ΩR)^k, so a reader can
    // check them by hand from the converged coefficients.
    let (rotor, op, af) = rotor_op();
    let cond = ForwardCondition::from_speed(30.0, &op, rotor.radius, 0.0);
    // from_speed: μ = V cosα / (ΩR), α = 0 here.
    assert!((cond.advance_ratio - 30.0 / op.tip_speed(rotor.radius)).abs() < 1e-12);

    let sol = solve_forward(&rotor, &op, &af, &cond, &ForwardConfig::default());
    let vt = op.tip_speed(rotor.radius);
    let a = rotor.disk_area();
    assert!((sol.thrust_n(&op, &rotor) - sol.ct * op.rho * a * vt * vt).abs() < 1e-6);
    assert!((sol.power_w(&op, &rotor) - sol.cp * op.rho * a * vt * vt * vt).abs() < 1e-3);
    assert!(
        (sol.rolling_moment_nm(&op, &rotor) - sol.c_roll * op.rho * a * vt * vt * rotor.radius)
            .abs()
            < 1e-6
    );
    // Glauert momentum thrust is consistent at the converged inflow.
    let g = glauert_inflow(sol.ct, cond.advance_ratio, 0.0, 1e-12, 200);
    assert!((g - glauert_inflow_closed_form(sol.ct, cond.advance_ratio)).abs() < 1e-6);
}

/// Trim collective so the forward-flight C_T equals `target_ct` (C_T rises
/// monotonically with collective). Returns the solved solution.
fn trim_to_ct(
    rotor: &Rotor,
    op: &Operating,
    af: &LinearAirfoil,
    cond: &ForwardCondition,
    target_ct: f64,
) -> ForwardSolution {
    let cfg = ForwardConfig::default();
    let ct_at = |theta: f64| solve_forward(&rotor.with_collective(theta), op, af, cond, &cfg).ct;
    let mut lo = 0.0_f64;
    let mut hi = 18f64.to_radians();
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if ct_at(mid) < target_ct {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let theta = 0.5 * (lo + hi);
    solve_forward(&rotor.with_collective(theta), op, af, cond, &cfg)
}

#[test]
fn inflow_solver_matches_glauert_closed_form() {
    for &ct in &[0.004, 0.008, 0.012] {
        for &mu in &[0.0, 0.1, 0.25, 0.4] {
            let solved = glauert_inflow(ct, mu, 0.0, 1e-12, 200);
            let exact = glauert_inflow_closed_form(ct, mu);
            assert!((solved - exact).abs() < 1e-6, "ct={ct} mu={mu}");
        }
    }
}

#[test]
fn rotor_power_dips_below_hover() {
    // The rotor-alone power falls below hover as forward speed cuts induced
    // power. (The high-speed *rise* of the full bucket comes from airframe
    // parasite drag, which a rotor-only model does not have — see the next test
    // and the airframe/rigid-body milestone.)
    let (rotor, op, af) = rotor_op();
    let cp_at = |mu: f64| trim_to_ct(&rotor, &op, &af, &ForwardCondition::new(mu, 0.0), 0.006).cp;
    assert!(
        cp_at(0.15) < cp_at(0.0),
        "rotor power should dip below hover"
    );
    assert!(
        cp_at(0.3) < cp_at(0.15),
        "rotor-alone power keeps falling without parasite"
    );
}

#[test]
fn full_power_curve_has_a_bucket_with_parasite() {
    // The classic aircraft power bucket (dip then rise) emerges once a
    // representative airframe parasite term C_P,para = 0.5·(f/A)·μ³ is overlaid
    // on the rotor power. f/A is the equivalent flat-plate area over disk area.
    let (rotor, op, af) = rotor_op();
    let f_over_a = 0.01;
    let total_cp = |mu: f64| {
        let rotor_cp = trim_to_ct(&rotor, &op, &af, &ForwardCondition::new(mu, 0.0), 0.006).cp;
        rotor_cp + 0.5 * f_over_a * mu * mu * mu
    };
    let grid: Vec<f64> = (0..=10).map(|k| k as f64 * 0.04).collect();
    let powers: Vec<f64> = grid.iter().map(|&m| total_cp(m)).collect();
    let (min_i, _) = powers
        .iter()
        .enumerate()
        .min_by(|a, b| a.1.partial_cmp(b.1).unwrap())
        .unwrap();
    // Minimum is at moderate speed, below hover, with both ends higher.
    assert!(
        min_i > 0 && min_i < grid.len() - 1,
        "bucket minimum should be interior"
    );
    assert!(powers[min_i] < powers[0], "bucket should dip below hover");
    assert!(
        *powers.last().unwrap() > powers[min_i],
        "power should rise at high speed"
    );
}

#[test]
fn induced_power_falls_with_speed_at_constant_thrust() {
    let (rotor, op, af) = rotor_op();
    let target_ct = 0.006;
    let ind = |mu: f64| {
        let cond = ForwardCondition::new(mu, 0.0);
        trim_to_ct(&rotor, &op, &af, &cond, target_ct).cp_induced
    };
    assert!(ind(0.3) < ind(0.15) && ind(0.15) < ind(0.0));
}

#[test]
fn rigid_blade_produces_large_uncommanded_rolling_moment() {
    // The headline result. At a representative cruise μ, the advancing side
    // carries far more thrust than the retreating side, giving a big rolling
    // moment that dwarfs the pitching moment — there is no flapping to relieve it.
    let (rotor, op, af) = rotor_op();
    let cond = ForwardCondition::new(0.25, 0.0);
    let sol = solve_forward(&rotor, &op, &af, &cond, &ForwardConfig::default());

    assert!(
        sol.advancing_ct > 1.5 * sol.retreating_ct,
        "advancing should dominate"
    );
    assert!(
        sol.c_roll.abs() > 1e-4,
        "rolling moment should be substantial"
    );
    assert!(
        sol.c_roll.abs() > 8.0 * sol.c_pitch.abs(),
        "roll ≫ pitch for uniform inflow"
    );

    // Dimensionally non-trivial: tens of N·m on this ~1 m rotor.
    let m_roll = sol.rolling_moment_nm(&op, &rotor).abs();
    assert!(
        m_roll > 5.0,
        "rolling moment {m_roll:.1} N·m should be sizeable"
    );
}

#[test]
fn reverse_flow_region_grows_with_speed_and_is_small_at_low_mu() {
    let (rotor, op, af) = rotor_op();
    let cfg = ForwardConfig::default();
    let frac = |mu: f64| {
        solve_forward(&rotor, &op, &af, &ForwardCondition::new(mu, 0.0), &cfg).reverse_flow_fraction
    };
    let f1 = frac(0.1);
    let f3 = frac(0.3);
    let f5 = frac(0.5);
    assert!(
        f1 < 0.02,
        "reverse-flow disk should be tiny at μ=0.1 (got {f1:.4})"
    );
    assert!(f1 < f3 && f3 < f5, "reverse-flow area should grow with μ");
}
