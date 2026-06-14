//! 5g — nonlinear coupled 8-state march validation.
//!
//! Gated by the pre-computed linear models (5c longitudinal, corrected 5e-i
//! lateral, coupled 8×8) and the now-validated general-state aero. Checks:
//!  * trim is an exact 8-D fixed point (drift at numerical-noise level), even
//!    though the equilibrium is unstable;
//!  * the nonlinear EOM's numerical Jacobian matches the independently-built
//!    coupled 8×8 linear model (two independent routes);
//!  * a perturbed march tracks the 8-D linear prediction in the small-amplitude
//!    window, then departs as the (two) oscillatory instabilities compound.

use helisim_dynamics::{Complex, Inertia, analyze_coupled_hover, eigenvalues};
use helisim_sim::{equilibrium_state8, linearize8, simulate_linear_nd, simulate8};
use helisim_trim::Aircraft;

fn inertia() -> Inertia {
    Inertia {
        mass: 8.0,
        i_xx: 0.4,
        i_yy: 0.8,
        i_zz: 1.0,
    }
}

#[test]
fn trim_is_an_exact_8d_fixed_point() {
    // Start exactly at the (unstable) equilibrium; drift must stay at numerical-
    // noise level over a 6 s horizon — the 8-D analog of 5d's 1.3e-12. (Over
    // longer horizons the unstable modes amplify machine noise, as they should.)
    let ac = Aircraft::demo();
    let eq = equilibrium_state8(&ac);
    let traj = simulate8(&ac, inertia(), [0.0; 8], 0.02, 6.0);
    let drift = traj
        .iter()
        .map(|s| (0..8).map(|i| (s[i] - eq[i]).abs()).fold(0.0_f64, f64::max))
        .fold(0.0_f64, f64::max);
    assert!(drift < 1e-6, "8-D fixed-point drift {drift:.2e} too large");
}

#[test]
fn nonlinear_jacobian_matches_coupled_linear_model() {
    // The nonlinear EOM linearized numerically must reproduce the independently-
    // assembled coupled 8×8 model — two genuinely different routes to the modes.
    let ac = Aircraft::demo();
    let j = inertia();
    let nl = eigenvalues(&linearize8(&ac, j));
    let lin = analyze_coupled_hover(&ac, j, true).eigenvalues;

    let contains = |set: &[Complex], e: Complex| {
        set.iter()
            .any(|c| (c.re - e.re).abs() < 0.06 && (c.im - e.im).abs() < 0.06)
    };
    for e in &nl {
        assert!(
            contains(&lin, *e),
            "nonlinear-Jacobian eig {:?} not in coupled model",
            e
        );
    }
    // Both unstable oscillatory instabilities present in the nonlinear Jacobian.
    let unstable_osc = nl
        .iter()
        .filter(|e| e.re > 0.05 && e.im.abs() > 0.1)
        .count();
    assert!(
        unstable_osc >= 2,
        "expected the two oscillatory instabilities, got {unstable_osc}"
    );
}

#[test]
fn nonlinear_tracks_linear_then_departs() {
    // Explicit amplitude and window (named, not incidental): a Δu = 0.1 m/s
    // perturbation tracks the 8-D linear prediction to within 5% through a 4 s
    // window, then departs (>20%) by ~8 s as the two unstable oscillations grow.
    const AMP: f64 = 0.1; // m/s
    const DT: f64 = 0.01;
    const LINEAR_WINDOW_S: f64 = 4.0;
    const DEPART_S: f64 = 8.0;

    let ac = Aircraft::demo();
    let j = inertia();
    let eq = equilibrium_state8(&ac);
    let a8 = linearize8(&ac, j);
    let pert = [AMP, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

    let nl = simulate8(&ac, j, pert, DT, DEPART_S);
    let ll = simulate_linear_nd(&a8, &pert, DT, DEPART_S);

    let rel = |k: usize| {
        let pl: f64 = (0..8).map(|i| ll[k][i] * ll[k][i]).sum::<f64>().sqrt();
        (0..8)
            .map(|i| ((nl[k][i] - eq[i]) - ll[k][i]).abs())
            .fold(0.0_f64, f64::max)
            / pl.max(1e-9)
    };
    let kw = (LINEAR_WINDOW_S / DT) as usize;
    let kd = (DEPART_S / DT) as usize;

    assert!(
        rel(kw) < 0.05,
        "should track linear within 5% at {LINEAR_WINDOW_S}s (got {:.1}%)",
        rel(kw) * 100.0
    );
    assert!(
        rel(kd) > 0.20,
        "should depart from linear by {DEPART_S}s (got {:.1}%)",
        rel(kd) * 100.0
    );
}

#[test]
fn longitudinal_perturbation_excites_lateral_via_coupling() {
    // A purely longitudinal Δu drives lateral motion (v,p,φ) in the nonlinear
    // coupled march — the cross-coupling that neither 4-state model shows.
    let ac = Aircraft::demo();
    let eq = equilibrium_state8(&ac);
    let nl = simulate8(
        &ac,
        inertia(),
        [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        0.01,
        5.0,
    );
    let lat = nl
        .iter()
        .map(|s| {
            [s[4] - eq[4], s[5] - eq[5], s[7] - eq[7]]
                .iter()
                .fold(0.0_f64, |m, &v| m.max(v.abs()))
        })
        .fold(0.0_f64, f64::max);
    assert!(
        lat > 1e-3,
        "longitudinal perturbation should excite lateral via coupling"
    );
}
