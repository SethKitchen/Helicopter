//! Nonlinear time-march validation against the pre-computed 5c eigenvalues.
//!
//! The linear milestone handed this one its oracle: hover is predicted to have an
//! unstable oscillation (≈0.50 ± 0.90i → period ~7 s, doubling ~1.4 s for the
//! demo aircraft). The nonlinear longitudinal time-march must reproduce that,
//! and — the strongest check — must coincide with the linear model `ẋ = Ax` for
//! small perturbations, diverging only as amplitude grows into nonlinearity.

use helisim_dynamics::{Inertia, analyze_coupled_hover, analyze_hover_longitudinal};
use helisim_sim::{
    fit_growing_oscillation, simulate_hover_longitudinal, simulate_linear, simulate_linear_nd,
};
use helisim_trim::Aircraft;

const I_YY: f64 = 0.8;

/// Max relative difference between two θ histories over `[0, t_window]`.
fn max_rel_theta(
    nl: &helisim_sim::Trajectory,
    ll: &helisim_sim::Trajectory,
    dt: f64,
    t_window: f64,
) -> f64 {
    let n = (t_window / dt) as usize;
    let mut m = 0.0_f64;
    for k in 0..n.min(nl.states.len()).min(ll.states.len()) {
        let a = nl.states[k][3];
        let b = ll.states[k][3];
        if b.abs() > 1e-6 {
            m = m.max((a - b).abs() / b.abs());
        }
    }
    m
}

#[test]
fn trim_is_an_exact_fixed_point() {
    // Starting exactly at the equilibrium, the integrator must not drift.
    let ac = Aircraft::demo();
    let traj = simulate_hover_longitudinal(&ac, I_YY, [0.0, 0.0, 0.0, 0.0], 0.02, 12.0);
    assert!(
        traj.max_abs() < 1e-8,
        "fixed point drifted: {:.2e}",
        traj.max_abs()
    );
}

#[test]
fn nonlinear_tracks_linear_in_the_small() {
    // THE GATE: for a moderate perturbation, the nonlinear trajectory coincides
    // with the linear ẋ=Ax prediction through the linear regime (first ~4 s),
    // i.e. the time-march reproduces the eigenvalue dynamics.
    let ac = Aircraft::demo();
    let lin = analyze_hover_longitudinal(&ac, I_YY);
    let dt = 0.01;
    let x0 = [0.5, 0.0, 0.0, 0.0];
    let nl = simulate_hover_longitudinal(&ac, I_YY, x0, dt, 5.0);
    let ll = simulate_linear(&lin.a_matrix, x0, dt, 5.0);
    let rel = max_rel_theta(&nl, &ll, dt, 4.0);
    assert!(
        rel < 0.05,
        "nonlinear vs linear θ differ by {:.1}% in the linear regime",
        rel * 100.0
    );
}

#[test]
fn match_is_not_a_step_size_artifact() {
    // The linear-regime agreement must hold at two step sizes.
    let ac = Aircraft::demo();
    let lin = analyze_hover_longitudinal(&ac, I_YY);
    let x0 = [0.5, 0.0, 0.0, 0.0];
    for &dt in &[0.02, 0.005] {
        let nl = simulate_hover_longitudinal(&ac, I_YY, x0, dt, 5.0);
        let ll = simulate_linear(&lin.a_matrix, x0, dt, 5.0);
        assert!(
            max_rel_theta(&nl, &ll, dt, 4.0) < 0.05,
            "step dt={dt} failed"
        );
    }
}

#[test]
fn reproduces_eigenvalue_period_and_growth() {
    // Extract period and growth from the nonlinear trajectory peaks and compare
    // to the 5c eigenvalue. (Period drifts a little as the second peak enters
    // mild nonlinearity; the growth rate is robust.)
    let ac = Aircraft::demo();
    let lin = analyze_hover_longitudinal(&ac, I_YY);
    let osc = lin
        .modes
        .iter()
        .find(|m| m.oscillatory && m.eigenvalue.im > 0.0)
        .unwrap();

    let traj = simulate_hover_longitudinal(&ac, I_YY, [0.1, 0.0, 0.0, 0.0], 0.01, 12.0);
    let theta = traj.column(3);
    // Two peaks in a moderate-amplitude window (≤ 1.5 rad), still mostly linear.
    let fit = fit_growing_oscillation(&traj.times, &theta, 1.5).expect("should fit ≥2 peaks");

    assert!(
        (fit.period - osc.period).abs() / osc.period < 0.25,
        "period {:.2}s vs 5c {:.2}s",
        fit.period,
        osc.period
    );
    assert!(
        (fit.growth_rate - osc.eigenvalue.re).abs() / osc.eigenvalue.re < 0.20,
        "growth {:.3} vs 5c σ {:.3}",
        fit.growth_rate,
        osc.eigenvalue.re
    );
    assert!(fit.growth_rate > 0.0, "must be growing (unstable)");
}

#[test]
fn perturbation_grows_then_departs_from_linear() {
    // Physical result the linear model cannot give: the small perturbation grows
    // (unstable), and the nonlinear trajectory eventually departs from the linear
    // prediction as amplitude builds.
    let ac = Aircraft::demo();
    let lin = analyze_hover_longitudinal(&ac, I_YY);
    let dt = 0.01;
    let x0 = [0.5, 0.0, 0.0, 0.0];
    let nl = simulate_hover_longitudinal(&ac, I_YY, x0, dt, 10.0);
    let ll = simulate_linear(&lin.a_matrix, x0, dt, 10.0);

    // Grows: late amplitude far exceeds the initial perturbation.
    assert!(nl.max_abs() > 5.0 * 0.5, "perturbation should grow");
    // Departs: late-time difference is large even though early matched.
    assert!(
        max_rel_theta(&nl, &ll, dt, 10.0) > 0.05,
        "should depart from linear at large amplitude"
    );
}

#[test]
fn coupling_transfers_longitudinal_perturbation_into_lateral_motion() {
    // 5e-ii time-domain coupling gate: a purely longitudinal perturbation (Δu)
    // excites lateral states (v, p, φ) only when the cross-coupling is on.
    // State order [u, w, q, θ, v, p, r, φ]; lateral states are indices 4,5,6,7.
    let ac = Aircraft::demo();
    let j = Inertia {
        mass: 8.0,
        i_xx: 0.4,
        i_yy: 0.8,
        i_zz: 1.0,
    };
    let x0 = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // only Δu

    let dec = analyze_coupled_hover(&ac, j, false);
    let cpl = analyze_coupled_hover(&ac, j, true);
    let lat_max = |states: &[Vec<f64>]| {
        states
            .iter()
            .flat_map(|s| [s[4], s[5], s[6], s[7]])
            .fold(0.0_f64, |m, v| m.max(v.abs()))
    };

    let dec_traj = simulate_linear_nd(&dec.a_matrix, &x0, 0.01, 4.0);
    let cpl_traj = simulate_linear_nd(&cpl.a_matrix, &x0, 0.01, 4.0);

    // Decoupled: lateral states stay exactly zero.
    assert!(lat_max(&dec_traj) < 1e-9, "decoupled: no lateral motion");
    // Coupled: the longitudinal perturbation drives real lateral motion.
    assert!(
        lat_max(&cpl_traj) > 1e-3,
        "coupled: longitudinal Δu should excite lateral"
    );
}
