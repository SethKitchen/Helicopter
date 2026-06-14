//! Design-study validation.
//!
//! The sizing crate adds no physics, so it is not validated against a new oracle.
//! Its job is to *compose the already-validated cores correctly* and to surface
//! the design trades, so the checks are:
//!
//! 1. **Composition consistency** — the report's autorotation numbers equal the
//!    autorotation crate called directly on the same inputs (it really delegates
//!    to the trusted model, rather than a drifting parallel calculation).
//! 2. **Feasibility** — the starter model point hovers, with a physical FM,
//!    positive power and positive airtime.
//! 3. **Trade directions** — growing the disk at fixed tip speed moves each
//!    priority metric the physically correct way.

use helisim_airfoil::LinearAirfoil;
use helisim_autorotation::{profile_power, steady_autorotation, G};
use helisim_bemt::Config;
use helisim_design::{evaluate, recommend, sweep_radius, DesignCandidate, DesignSpace};

#[test]
fn delegates_autorotation_to_the_validated_core() {
    let c = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let report = evaluate(&c, &af, &Config::default());

    // Reproduce the autorotation path independently from the same inputs.
    let op = c.operating();
    let weight = c.gross_mass_kg * G;
    let p0 = profile_power(op.rho, c.disk_area(), c.tip_speed_ms, c.solidity(), c.blade_cd0);
    let auto = steady_autorotation(weight, op.rho, c.disk_area(), p0);

    assert!((report.autorotation_ratio - auto.descent_ratio).abs() < 1e-12);
    assert!((report.autorotation_descent_fpm - auto.descent_rate_fpm()).abs() < 1e-9);
}

#[test]
fn starter_model_point_is_feasible_and_physical() {
    let c = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let r = evaluate(&c, &af, &Config::default());

    assert!(r.hover_feasible);
    assert!(r.collective_deg > 0.0 && r.collective_deg < 25.0);
    assert!(r.figure_of_merit > 0.0 && r.figure_of_merit < 1.0);
    assert!(r.hover_shaft_power_w > 0.0);
    assert!(r.endurance_min > 0.0);
    assert!(r.tip_mach > 0.0 && r.tip_mach < 1.0);
    assert!(r.oaspl_db.is_finite());
    // The model descends at least as fast as the induced-ideal autorotation; a
    // small low-disk-loading rotor is profile-heavy, so it sits *above* the
    // full-scale [1.7, 2.0] band (a real model-scale penalty, surfaced not hidden).
    assert!(r.autorotation_ratio > 1.7);
}

#[test]
fn bigger_disk_is_more_efficient_quieter_and_gentler_in_autorotation() {
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    let pts = sweep_radius(&base, &[0.5, 0.7], &af, &cfg);
    let (small, big) = (pts[0].report, pts[1].report);

    assert!(small.hover_feasible && big.hover_feasible);
    // Lower disk loading at the bigger radius.
    assert!(big.disk_loading < small.disk_loading);
    // Efficiency & airtime: less hover power, longer endurance.
    assert!(big.hover_shaft_power_w < small.hover_shaft_power_w);
    assert!(big.endurance_min > small.endurance_min);
    // Safety: a gentler (slower) autorotation descent.
    assert!(big.autorotation_descent_fpm < small.autorotation_descent_fpm);
    // Noise: quieter at the observer.
    assert!(big.oaspl_db < small.oaspl_db);
}

#[test]
fn recommender_returns_a_safe_feasible_winner() {
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let space = DesignSpace::model_default();
    let rec = recommend(&space, &base, &af, &Config::default()).expect("should find a design");

    // The winner satisfies every hard constraint.
    let r = &rec.best.report;
    assert!(r.hover_feasible);
    assert!(r.flare_margin >= space.min_flare_margin);
    assert!(r.endurance_min >= space.min_endurance_min);
    assert!(r.tip_mach <= space.max_tip_mach + 1e-9);
    assert!(rec.n_feasible >= 1 && rec.n_feasible <= rec.n_evaluated);
}

#[test]
fn recommender_ranks_by_score_and_winner_is_the_max() {
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let rec = recommend(&DesignSpace::model_default(), &base, &af, &Config::default()).unwrap();

    // Ranked list is sorted descending; the winner tops it.
    for w in rec.ranked.windows(2) {
        assert!(w[0].score >= w[1].score);
    }
    assert!((rec.best.score - rec.ranked[0].score).abs() < 1e-12);
}

#[test]
fn tightening_the_safety_floor_shrinks_the_feasible_set() {
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();

    let mut lax = DesignSpace::model_default();
    lax.min_flare_margin = 1.0;
    let mut strict = DesignSpace::model_default();
    strict.min_flare_margin = 2.0;

    let n_lax = recommend(&lax, &base, &af, &cfg).unwrap().n_feasible;
    let n_strict = recommend(&strict, &base, &af, &cfg).map(|r| r.n_feasible).unwrap_or(0);
    // A stricter safety floor can only reduce (or keep) the feasible count.
    assert!(n_strict <= n_lax);
}
