//! Mission-profile energy validated by closed forms: exact segment-sum energy, the
//! Breguet-style range identity, and an interior power-bucket minimum the optimizer
//! recovers. A mission integrator that quietly mis-sums would mis-size every pack.

use helisim_design::{AircraftPower, Mission, Segment};

fn aircraft() -> AircraftPower {
    AircraftPower {
        gross_mass_kg: 3.5,
        rho: 1.225,
        disk_area_m2: 1.54, // R≈0.7 m
        figure_of_merit: 0.7,
        flat_plate_area_m2: 0.05,
        profile_power_w: 60.0,
        powertrain_eta: 0.8,
    }
}

/// SEGMENT-SUM ORACLE: mission energy is exactly `Σ Pᵢ·tᵢ`. A single hover leg
/// equals `P_hover·t/3600`; a multi-leg mission equals the sum of its legs; and the
/// electrical energy is the shaft energy over efficiency.
#[test]
fn segment_energy_is_exact_power_times_time() {
    let p = aircraft();

    // One 120 s hover.
    let hover = Mission {
        segments: vec![Segment::Hover { duration_s: 120.0 }],
    };
    let expected = p.hover_shaft_power_w() * 120.0 / 3600.0;
    assert!((hover.shaft_energy_wh(&p) - expected).abs() < 1e-9);
    assert!((hover.elec_energy_wh(&p) - expected / p.powertrain_eta).abs() < 1e-9);

    // Three legs sum to their parts.
    let s0 = Segment::Hover { duration_s: 30.0 };
    let s1 = Segment::Climb {
        rate_mps: 2.0,
        height_m: 50.0,
    };
    let s2 = Segment::Cruise {
        speed_mps: 15.0,
        distance_m: 1500.0,
    };
    let mission = Mission {
        segments: vec![s0, s1, s2],
    };
    let parts = s0.shaft_energy_wh(&p) + s1.shaft_energy_wh(&p) + s2.shaft_energy_wh(&p);
    assert!((mission.shaft_energy_wh(&p) - parts).abs() < 1e-9);
    // Total time: 30 + 50/2 + 1500/15 = 30 + 25 + 100 = 155 s.
    assert!((mission.total_time_s(&p) - 155.0).abs() < 1e-9);
}

/// BREGUET RANGE IDENTITY: cruise energy = equivalent-drag × distance, so the range
/// achievable on a usable energy `E` is `E / D_equiv`. The cleanest forward-flight
/// energy oracle (independent of how power splits into induced/parasite/profile).
#[test]
fn cruise_range_equals_energy_over_equivalent_drag() {
    let p = aircraft();
    let (v, dist) = (18.0, 3600.0);
    let seg = Segment::Cruise {
        speed_mps: v,
        distance_m: dist,
    };
    let energy_j = seg.shaft_energy_wh(&p) * 3600.0; // Wh → J
    let drag = p.forward_equiv_drag_n(v);
    assert!((energy_j - drag * dist).abs() < 1e-6, "E = D·d");

    // Range for a given usable shaft energy = E / D.
    let usable_j = 200.0 * 3600.0; // 200 Wh
    let range = usable_j / drag;
    let flown = Mission {
        segments: vec![Segment::Cruise {
            speed_mps: v,
            distance_m: range,
        }],
    };
    assert!((flown.shaft_energy_wh(&p) * 3600.0 - usable_j).abs() < 1e-3);
}

/// POWER BUCKET: forward power has an interior minimum (induced ∝ 1/V falling,
/// parasite ∝ V³ rising). The optimizer recovers the min-power speed, and best-range
/// (min P/V) is strictly faster than best-loiter (min P) — the textbook ordering.
#[test]
fn forward_power_bucket_has_interior_minimum() {
    let p = aircraft();
    let v_loiter = p.min_power_speed_mps(3.0, 60.0);
    let v_range = p.best_range_speed_mps(3.0, 60.0);

    println!("min-power speed {v_loiter:.1} m/s, best-range speed {v_range:.1} m/s");
    // Interior: power at the optimum is below power at both ends of the range.
    let p_min = p.forward_shaft_power_w(v_loiter);
    assert!(p_min < p.forward_shaft_power_w(3.0), "below the slow end");
    assert!(p_min < p.forward_shaft_power_w(60.0), "below the fast end");
    assert!(v_loiter > 3.0 && v_loiter < 60.0, "minimum is interior");
    // Best-range speed is faster than best-loiter speed.
    assert!(
        v_range > v_loiter,
        "max-range speed exceeds min-power speed"
    );
}

/// MISSION FEASIBILITY OBJECTIVE: a pack with enough usable energy flies the mission;
/// one just short does not. The objective the optimizer/closure consume.
#[test]
fn mission_feasibility_against_usable_energy() {
    let p = aircraft();
    let mission = Mission {
        segments: vec![
            Segment::Climb {
                rate_mps: 2.0,
                height_m: 100.0,
            },
            Segment::Cruise {
                speed_mps: 15.0,
                distance_m: 5000.0,
            },
            Segment::Hover { duration_s: 60.0 },
        ],
    };
    let need = mission.elec_energy_wh(&p);
    assert!(
        mission.feasible(&p, need + 1e-6),
        "feasible with exactly enough"
    );
    assert!(!mission.feasible(&p, need - 1.0), "infeasible 1 Wh short");

    // Required pack mass scales with usable specific energy.
    let m_pack = mission.required_pack_mass_kg(&p, 150.0);
    assert!((m_pack - need / 150.0).abs() < 1e-9);
}
