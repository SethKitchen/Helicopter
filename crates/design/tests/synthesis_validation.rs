//! The design-synthesis upgrades to the recommender: the Pareto front it now
//! returns, and the optional forward-flight envelope constraint. Validates the
//! invariants (the scalarised winner is Pareto-optimal; the front is mutually
//! non-dominated) and that the envelope floor really prunes the search.

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_design::{
    DesignCandidate, DesignSpace, EnvelopeConstraint, EnvelopeLimits, candidate_envelope, recommend,
};
use helisim_optimize::dominates;

/// Objective vector (minimisation form) for a report — mirrors what `recommend`
/// feeds the Pareto front, so the test can re-derive dominance independently.
fn obj(r: &helisim_design::DesignReport) -> Vec<f64> {
    vec![
        -r.vertical_integration_index,
        r.total_cost,
        -r.endurance_min,
        -r.figure_of_merit,
        r.oaspl_db,
    ]
}

/// The recommended winner is on the Pareto front, the front is mutually
/// non-dominated, and it is no larger than the full ranked list.
#[test]
fn winner_is_pareto_optimal_and_front_is_clean() {
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let rec = recommend(
        &DesignSpace::model_default(),
        &base,
        &af,
        &Config::default(),
    )
    .unwrap();

    assert!(!rec.pareto.is_empty(), "a non-empty Pareto front");
    assert!(rec.pareto.len() <= rec.ranked.len());

    // The scalarised winner must itself be non-dominated (a weighted-sum optimum of
    // these objectives is always Pareto-optimal) — i.e. it appears on the front.
    let best_obj = obj(&rec.best.report);
    let on_front = rec.pareto.iter().any(|s| obj(&s.report) == best_obj);
    assert!(on_front, "the recommended design is on the Pareto front");

    // No ranked candidate dominates the winner.
    for s in &rec.ranked {
        assert!(
            !dominates(&obj(&s.report), &best_obj),
            "nothing should dominate the recommended winner"
        );
    }

    // The front members are mutually non-dominated.
    for (i, a) in rec.pareto.iter().enumerate() {
        for (j, b) in rec.pareto.iter().enumerate() {
            if i != j {
                assert!(
                    !dominates(&obj(&a.report), &obj(&b.report)),
                    "front members must not dominate each other"
                );
            }
        }
    }
}

/// The forward-flight envelope constraint prunes candidates whose usable speed limit
/// is below the floor — and the surviving winner clears it.
#[test]
fn envelope_floor_prunes_the_feasible_set() {
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();

    let open = recommend(&DesignSpace::model_default(), &base, &af, &cfg).unwrap();

    let limits = EnvelopeLimits {
        sound_speed_mps: 340.0,
        advancing_tip_mach_limit: 0.85,
        cl_max: 1.2,
        power_available_w: 1500.0,
    };
    // A demanding speed floor that not every rotor can meet.
    let mut constrained = DesignSpace::model_default();
    constrained.envelope = Some(EnvelopeConstraint {
        limits,
        min_speed_limit_mps: 25.0,
    });
    let rec = recommend(&constrained, &base, &af, &cfg).expect("some design clears the floor");

    assert!(
        rec.n_feasible <= open.n_feasible,
        "the envelope floor cannot grow the feasible set"
    );
    // The surviving winner actually meets the floor.
    let env = candidate_envelope(&rec.best.candidate, &rec.best.report, &limits);
    assert!(
        env.speed_limit_mps >= 25.0 - 1e-6,
        "winner clears the envelope floor (got {:.1} m/s)",
        env.speed_limit_mps
    );
}
