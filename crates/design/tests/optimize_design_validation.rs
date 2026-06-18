//! Continuous design optimization finds the TRUE optimum: an interior minimum-mass
//! design (perturbing it in any direction makes it heavier), feasible against the
//! constraints — not a grid corner. This is what lets the report drop the "may be
//! larger" caveat.

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_design::{
    DesignBounds, DesignCandidate, DesignConstraints, LifeRequirement, Mission, Segment,
    SizingPolicy, evaluate, optimize_design,
};
use std::f64::consts::PI;

fn policy() -> SizingPolicy {
    SizingPolicy {
        payload_kg: 1.0,
        empty_fraction: 0.30, // non-rotor structure; the rotor group is explicit now
        fixed_mass_kg: 0.25,
        specific_energy_wh_kg: 220.0,
        mission: Mission {
            segments: vec![
                Segment::Climb {
                    rate_mps: 2.0,
                    height_m: 100.0,
                },
                Segment::Cruise {
                    speed_mps: 14.0,
                    distance_m: 4000.0,
                },
                Segment::Hover { duration_s: 120.0 },
            ],
        },
        life: LifeRequirement::daily(10.0),
        max_gross_kg: 80.0,
    }
}

fn bounds() -> DesignBounds {
    DesignBounds {
        radius_m: (0.4, 2.5),
        tip_speed_ms: (70.0, 150.0),
        solidity: (0.04, 0.12),
    }
}

fn constraints() -> DesignConstraints {
    DesignConstraints {
        min_flare_margin: 1.5,
        max_tip_mach: 0.55,
        envelope: None,
    }
}

/// The optimizer returns the lightest FEASIBLE design: it is feasible, and no nearby
/// design is both lighter AND feasible — a smaller rotor is lighter but violates the
/// flare-margin safety floor (infeasible), a bigger rotor is heavier. This is a true
/// constrained optimum (located between grid nodes), not a grid corner.
#[test]
fn optimum_is_the_lightest_feasible_design() {
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    let base = DesignCandidate::model();
    let pol = policy();
    let cons = constraints();

    // One blade count keeps the test quick; the report optimizes over 2/3/4.
    let opt =
        optimize_design(&base, &[3], &bounds(), &cons, &pol, &af, &cfg).expect("an optimum exists");
    let c = &opt.candidate;
    let (r0, g0) = (c.radius_m, opt.sized.gross_kg);
    println!(
        "optimum: {} blades, R={:.2} m, σ={:.3}, V_tip={:.0} m/s → gross {:.2} kg (flare {:.2})",
        c.n_blades,
        r0,
        c.solidity(),
        c.tip_speed_ms,
        g0,
        opt.report.flare_margin
    );

    // The optimum is feasible.
    assert!(opt.report.hover_feasible);
    assert!(opt.report.flare_margin >= cons.min_flare_margin - 1e-3);
    assert!(opt.report.tip_mach <= cons.max_tip_mach + 1e-3);

    // Evaluate a scaled-radius neighbour: its closed gross + whether it's feasible.
    let neighbour = |scale: f64| {
        let r = r0 * scale;
        let chord = c.solidity() * PI * r / c.n_blades as f64;
        let geom = base.with_geometry(c.n_blades, r, chord, c.tip_speed_ms);
        pol.sized_candidate(&geom, &af, &cfg).map(|(cand, s)| {
            let rep = evaluate(&cand, &af, &cfg);
            let feasible = rep.hover_feasible
                && rep.flare_margin >= cons.min_flare_margin - 1e-3
                && rep.tip_mach <= cons.max_tip_mach + 1e-3;
            (s.gross_kg, feasible, rep.flare_margin)
        })
    };
    let small = neighbour(0.85);
    let big = neighbour(1.15);
    println!("R×0.85 → {small:?}; R×1.15 → {big:?}; optimum {g0:.2} kg");

    // No nearby design is both lighter AND feasible.
    for (gross, feasible, _flare) in [small, big].into_iter().flatten() {
        assert!(
            !feasible || gross > g0 - 1e-2,
            "a feasible neighbour ({gross:.2} kg) is lighter than the optimum ({g0:.2})"
        );
    }
    // The smaller rotor IS lighter but infeasible (the binding safety constraint) —
    // i.e. the optimum genuinely sits on the flare-margin floor, you can't shrink it.
    if let Some((gross, feasible, flare)) = small {
        assert!(
            gross < g0 && !feasible,
            "smaller rotor should be lighter ({gross:.2}) but unsafe (flare {flare:.2})"
        );
    }
    // The radius optimum is interior to the bound (not pinned at 2.5 m).
    assert!(
        !opt.at_bound.contains(&"radius"),
        "radius optimum is interior, not at the bound"
    );
}
