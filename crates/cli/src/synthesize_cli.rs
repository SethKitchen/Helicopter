//! `synthesize` subcommand: the design-synthesis layer — the inverse of the analysis
//! stack. Demonstrates all five solvers that turn "evaluate this design" into "find
//! the best design":
//!  1. blade twist/taper optimization vs the ideal-twist minimum-induced-loss anchor;
//!  2. the Nelder–Mead + Pareto optimizer (the new solver shape) inside the recommender;
//!  3. gross-weight (sizing) convergence;
//!  4. a mission-profile energy budget (best-loiter / best-range speeds);
//!  5. the flight envelope (Vne, power-limited max speed, hover ceiling).

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::Config;
use helisim_design::{
    AircraftPower, BladeProblem, DesignCandidate, DesignSpace, EnvelopeConstraint, EnvelopeLimits,
    FixedDiskLoading, Mission, Segment, WeightClosure, analyze_envelope, candidate_envelope,
    recommend,
};
use helisim_rotor::Operating;
use std::f64::consts::PI;

pub fn run() {
    println!("helisim — design synthesis (find the best design, not just evaluate one)\n");
    blade_shape_section();
    weight_closure_section();
    mission_section();
    envelope_section();
    recommend_section();
}

/// (1) Optimize the spanwise blade shape and show it against the ideal-twist bound.
fn blade_shape_section() {
    let af = LinearAirfoil::naca0012();
    let radius = 0.7;
    let solidity = 0.07;
    let mean_chord = solidity * PI * radius / 2.0;
    let p = BladeProblem {
        n_blades: 2,
        radius_m: radius,
        mean_chord_m: mean_chord,
        root_cutout: 0.15,
        op: Operating::from_tip_mach(100.0 / 340.0, radius),
        airfoil: &af,
        target_thrust_n: 30.0,
        cfg: Config {
            tip_loss: false,
            ..Config::default()
        },
    };
    let flat = p.evaluate(0.0, 1.0).unwrap();
    let opt = p.optimize();
    let ideal = p.ideal_twist_anchor().unwrap();

    println!("=== (1) BLADE-SHAPE OPTIMIZATION (min hover power at fixed thrust) ===");
    println!(
        "  untwisted rectangular : {:>6.1} W   inflow CV {:.3}",
        flat.power_w, flat.inflow_cv
    );
    println!(
        "  optimized linear blade: {:>6.1} W   inflow CV {:.3}  (twist {:.1}°, taper {:.2})",
        opt.power_w,
        opt.inflow_cv,
        opt.twist_rate.to_degrees(),
        opt.taper_ratio
    );
    println!(
        "  ideal-twist anchor    : {:>6.1} W   inflow CV {:.3}  (min induced C_P, not buildable)",
        ideal.power_w, ideal.inflow_cv
    );
    println!(
        "  → optimization cut hover power {:.0}%, flattening the inflow toward the ideal.\n",
        100.0 * (flat.power_w - opt.power_w) / flat.power_w
    );
}

/// (3) Close the gross-weight spiral for a representative model.
fn weight_closure_section() {
    let battery = FixedDiskLoading {
        disk_loading_n_m2: 60.0,
        rho: 1.225,
        figure_of_merit: 0.7,
        powertrain_eta: 0.8,
        flight_time_h: 1.0 / 3.0,
        specific_energy_wh_kg: 220.0,
        usable_fraction: 0.8,
    };
    let f = battery.battery_fraction();
    let prob = WeightClosure {
        payload_kg: 0.5,
        empty_fraction: 0.45,
        fixed_mass_kg: 0.25,
        battery: &battery,
    };
    println!("=== (3) GROSS-WEIGHT CLOSURE  W = W_empty(W) + payload + W_batt(W) ===");
    match prob.solve(50.0) {
        Some(r) => println!(
            "  closes at {:.2} kg  (empty {:.2} + payload {:.2} + battery {:.2}); battery fraction {:.1}%, {} iters",
            r.gross_kg,
            r.empty_kg,
            r.payload_kg,
            r.battery_kg,
            f * 100.0,
            r.iters
        ),
        None => {
            println!("  mass spiral DIVERGES (empty + battery fraction ≥ 1) — no closed design")
        }
    }
    println!();
}

/// (4) Budget a multi-leg mission and find the best-loiter / best-range speeds.
fn mission_section() {
    let power = AircraftPower {
        gross_mass_kg: 3.5,
        rho: 1.225,
        disk_area_m2: 1.54,
        figure_of_merit: 0.7,
        flat_plate_area_m2: 0.05,
        profile_power_w: 60.0,
        powertrain_eta: 0.8,
    };
    let mission = Mission {
        segments: vec![
            Segment::Climb {
                rate_mps: 2.0,
                height_m: 100.0,
            },
            Segment::Cruise {
                speed_mps: 15.0,
                distance_m: 4000.0,
            },
            Segment::Loiter {
                speed_mps: 12.0,
                duration_s: 120.0,
            },
            Segment::Hover { duration_s: 60.0 },
        ],
    };
    println!("=== (4) MISSION-PROFILE ENERGY (size for a mission, not just hover) ===");
    println!(
        "  climb→cruise→loiter→hover: {:.1} Wh shaft / {:.1} Wh electrical over {:.0} s",
        mission.shaft_energy_wh(&power),
        mission.elec_energy_wh(&power),
        mission.total_time_s(&power)
    );
    println!(
        "  best-loiter speed {:.1} m/s (min power), best-range speed {:.1} m/s (min drag)\n",
        power.min_power_speed_mps(3.0, 50.0),
        power.best_range_speed_mps(3.0, 50.0)
    );
}

/// (5) Assemble the flight envelope for a representative rotor.
fn envelope_section() {
    let power = AircraftPower {
        gross_mass_kg: 3.5,
        rho: 1.225,
        disk_area_m2: 1.54,
        figure_of_merit: 0.7,
        flat_plate_area_m2: 0.05,
        profile_power_w: 60.0,
        powertrain_eta: 0.8,
    };
    // Available shaft power ≈ 2.3× the ~340 W hover need — a realistic installed margin.
    let limits = EnvelopeLimits {
        sound_speed_mps: 340.0,
        advancing_tip_mach_limit: 0.85,
        cl_max: 1.2,
        power_available_w: 800.0,
    };
    let env = analyze_envelope(&power, 110.0, 0.07, &limits);
    println!("=== (5) FLIGHT ENVELOPE (limits as constraints) ===");
    println!(
        "  Vne: advancing-Mach {:.0} m/s, retreating-stall {:.0} m/s → Vne {:.0} m/s",
        env.vne_advancing_mach_mps, env.vne_retreating_stall_mps, env.vne_mps
    );
    println!(
        "  power-limited max level speed {:.0} m/s → usable limit {:.0} m/s",
        env.max_level_speed_mps, env.speed_limit_mps
    );
    println!(
        "  hover climb rate {:.1} m/s; hover ceiling ρ={:.3} kg/m³ (≈{:.0} m ISA)\n",
        env.hover_climb_rate_mps, env.hover_ceiling_density, env.hover_ceiling_altitude_m
    );
}

/// (2) The recommender, now returning a Pareto front and able to be envelope-constrained.
fn recommend_section() {
    let base = DesignCandidate::model();
    let af = LinearAirfoil::naca0012();
    let cfg = Config::default();
    let mut space = DesignSpace::model_default();
    space.envelope = Some(EnvelopeConstraint {
        limits: EnvelopeLimits {
            sound_speed_mps: 340.0,
            advancing_tip_mach_limit: 0.85,
            cl_max: 1.2,
            power_available_w: 1500.0,
        },
        min_speed_limit_mps: 15.0,
    });

    println!("=== (2) RECOMMEND with Nelder–Mead/Pareto + envelope constraint ===");
    match recommend(&space, &base, &af, &cfg) {
        Some(rec) => {
            for line in &rec.rationale {
                println!("  {line}");
            }
            println!("  Pareto front (non-dominated trades):");
            for sc in rec.pareto.iter().take(6) {
                let c = &sc.candidate;
                let r = &sc.report;
                let env = candidate_envelope(c, r, &space.envelope.unwrap().limits);
                println!(
                    "    {}b R={:.2} σ={:.3} V={:>3.0} | endur {:>4.1}m  FM {:.2}  ${:>4.0}  {:>4.1}dB  Vlim {:>3.0} m/s",
                    c.n_blades,
                    c.radius_m,
                    c.solidity(),
                    c.tip_speed_ms,
                    r.endurance_min,
                    r.figure_of_merit,
                    r.total_cost,
                    r.oaspl_db,
                    env.speed_limit_mps
                );
            }
        }
        None => {
            println!("  no design satisfies the constraints — relax the floor or widen the grid")
        }
    }
}
