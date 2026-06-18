//! Mission- AND life-driven weight closure (gaps 3+4 + the 10-yr-life / 1:1-charge
//! constraint folded into the recommender): a geometry is sized to fly a mission
//! with the service-life pack oversize, the spiral closed. Validates the fixed point
//! is self-consistent, heavier missions close heavier, daily flying oversizes the
//! pack (so the design is battery-heavy), non-hovering geometries are rejected, and
//! the recommender actually re-sizes its candidates — all in one integrated solve.

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_design::{
    DesignCandidate, DesignSpace, LifeRequirement, Mission, Segment, SizingPolicy, recommend,
};

fn policy(mission: Mission) -> SizingPolicy {
    SizingPolicy {
        payload_kg: 0.5,
        empty_fraction: 0.45,
        fixed_mass_kg: 0.25,
        specific_energy_wh_kg: 220.0,
        mission,
        life: LifeRequirement::daily(10.0), // 365/yr for 10 years
        max_gross_kg: 200.0,
    }
}

/// A light-use policy (no oversize needed) for comparison.
fn light_use_policy(mission: Mission) -> SizingPolicy {
    let mut p = policy(mission);
    p.life = LifeRequirement {
        flights_per_year: 10.0,
        ..LifeRequirement::daily(10.0)
    };
    p
}

fn short_mission() -> Mission {
    Mission {
        segments: vec![Segment::Hover { duration_s: 300.0 }],
    }
}

fn long_mission() -> Mission {
    Mission {
        segments: vec![
            Segment::Climb {
                rate_mps: 2.0,
                height_m: 150.0,
            },
            Segment::Cruise {
                speed_mps: 15.0,
                distance_m: 8000.0,
            },
            Segment::Hover { duration_s: 600.0 },
        ],
    }
}

/// SELF-CONSISTENT FIXED POINT: the closed gross mass equals empty + payload +
/// battery; the battery is the LIFE-sized pack (mission energy / DoD / specific
/// energy); and the pack capacity / oversize are consistent.
#[test]
fn closure_is_a_self_consistent_fixed_point() {
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    let geom = DesignCandidate::model();
    let pol = policy(short_mission());

    let s = pol
        .close(&geom, &af, &cfg)
        .expect("a model rotor closes for a short mission");
    // Mass balance closes.
    assert!(
        (s.empty_kg + pol.payload_kg + s.battery_kg - s.gross_kg).abs() < 1e-3,
        "empty + payload + battery == gross"
    );
    // Battery is the life-sized pack: capacity = energy/DoD, mass = capacity/specific.
    assert!(
        (s.pack_capacity_wh - s.mission_energy_wh / s.dod).abs() < 1e-2,
        "capacity == energy/DoD"
    );
    let implied = s.pack_capacity_wh / pol.specific_energy_wh_kg;
    assert!(
        (implied - s.battery_kg).abs() < 1e-3,
        "battery == capacity / specific energy"
    );
    assert!((s.oversize - 1.0 / s.dod).abs() < 1e-9, "oversize == 1/DoD");
    // Empty = affine structure fraction + fixed + the geometry-based rotor group.
    assert!(
        (s.empty_kg - (pol.empty_fraction * s.gross_kg + pol.fixed_mass_kg + s.rotor_group_kg))
            .abs()
            < 1e-9
    );
    assert!(
        s.rotor_group_kg > 0.0,
        "rotor group has mass that grows with the disk"
    );
}

/// DAILY-LIFE OVERSIZE (the 10-yr finding): daily flying forces a shallow DoD, so the
/// pack is oversized (>1×) and the design is heavier than the same mission flown only
/// occasionally. This is what pushes a daily-life model past a naive one-flight size.
#[test]
fn daily_flying_oversizes_the_pack_and_design() {
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    let geom = DesignCandidate::model();

    let daily = policy(short_mission()).close(&geom, &af, &cfg).unwrap();
    let light = light_use_policy(short_mission())
        .close(&geom, &af, &cfg)
        .unwrap();
    println!(
        "daily: DoD {:.0}% oversize {:.1}× gross {:.2} kg, fade {:.0}% | light: DoD {:.0}% gross {:.2} kg",
        daily.dod * 100.0,
        daily.oversize,
        daily.gross_kg,
        daily.fade_over_life * 100.0,
        light.dod * 100.0,
        light.gross_kg
    );
    assert!(
        daily.oversize > 1.5,
        "daily flying oversizes the pack (got {:.1}×)",
        daily.oversize
    );
    assert!(
        daily.dod < light.dod,
        "daily flying runs a shallower DoD than light use"
    );
    assert!(
        daily.gross_kg > light.gross_kg,
        "the life pack makes the daily design heavier"
    );
    assert!(
        daily.fade_over_life <= 0.20 + 1e-6,
        "daily design meets end-of-life"
    );
}

/// MISSION-DEMAND MONOTONICITY: a longer/harder mission needs more energy → a bigger
/// battery → a heavier closed gross mass.
#[test]
fn harder_mission_closes_heavier() {
    use std::f64::consts::PI;
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    // An ample rotor that carries BOTH life-packs (the small model() rotor correctly
    // fails to close a big life-pack — a smaller-rotor limitation, not monotonicity).
    let r = 1.1;
    let geom = DesignCandidate::model().with_geometry(3, r, 0.06 * PI * r / 3.0, 100.0);
    let hover = |s| Mission {
        segments: vec![Segment::Hover { duration_s: s }],
    };

    let s_short = policy(hover(120.0)).close(&geom, &af, &cfg).unwrap();
    let s_long = policy(hover(240.0)).close(&geom, &af, &cfg).unwrap();
    println!(
        "short mission gross {:.2} kg ({:.1} Wh) vs long {:.2} kg ({:.1} Wh)",
        s_short.gross_kg, s_short.mission_energy_wh, s_long.gross_kg, s_long.mission_energy_wh
    );
    assert!(s_long.mission_energy_wh > s_short.mission_energy_wh);
    assert!(
        s_long.gross_kg > s_short.gross_kg,
        "harder mission ⇒ heavier closure"
    );
}

/// REJECTION: a geometry that cannot hover at any sized mass returns `None`. A
/// vanishingly small rotor cannot lift the dead mass, so it never closes.
#[test]
fn non_hovering_geometry_is_rejected() {
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    // A tiny rotor: radius 0.1 m, thin chord — cannot lift ~1+ kg of dead mass.
    let geom = DesignCandidate::model().with_geometry(2, 0.1, 0.01, 90.0);
    assert!(
        policy(long_mission()).close(&geom, &af, &cfg).is_none(),
        "a tiny rotor cannot close a real mission"
    );
}

/// RECOMMENDER INTEGRATION: with a sizing policy set, the recommended candidate's
/// gross mass is the SOLVED closure (not the base candidate's fixed mass), and it is
/// feasible. Demonstrates the mission objective + weight closure folded into search.
#[test]
fn recommend_resizes_candidates_to_the_mission() {
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    let base = DesignCandidate::model();
    let base_gross = base.gross_mass_kg;

    let mut space = DesignSpace::model_default();
    space.min_endurance_min = 0.0; // the mission (not a hover-endurance floor) governs sizing
    space.sizing = Some(policy(long_mission()));

    let rec = recommend(&space, &base, &af, &cfg).expect("a mission-sized design exists");
    // The winner was re-sized: its gross mass came from the closure, generally ≠ base.
    let won = rec.best.candidate.gross_mass_kg;
    println!("base gross {base_gross:.2} kg → mission-sized winner gross {won:.2} kg");
    assert!(rec.best.report.hover_feasible, "the sized winner hovers");
    assert!(
        (won - base_gross).abs() > 1e-6,
        "gross mass was re-solved, not inherited"
    );
    // And the sized winner carries a pack big enough for the mission (closure ⇒ feasible).
    assert!(rec.best.report.endurance_min > 0.0);
}
