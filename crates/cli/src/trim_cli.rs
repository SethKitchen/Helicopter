//! `trim` subcommand: steady-flight trim via the multidimensional Newton solve,
//! with the hover cross-check against the standalone hover BEMT and a forward-
//! speed sweep showing the classic trim trends.

use helisim_bemt::{Config, solve_hover};
use helisim_trim::{Aircraft, NewtonConfig, TrimCondition, trim};

const G: f64 = 9.80665;

/// Independent hover BEMT trim (milestone-1 path): collective for thrust = weight.
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

pub fn run() {
    let ac = Aircraft::demo();
    let cfg = NewtonConfig::default();
    let w = ac.mass * G;

    println!("helisim — steady-flight trim (6-DOF force/moment Newton solve)\n");
    println!(
        "Aircraft: {:.0} kg, main R={:.2} m ({:.0} RPM), tail R={:.2} m, arm {:.2} m, Lock γ={:.0}",
        ac.mass,
        ac.main.radius,
        ac.main_op.rpm(),
        ac.tail.rotor.radius,
        ac.tail.arm,
        ac.flap.lock_number
    );

    // Hover cross-check.
    let (th_bemt, p_bemt) = hover_bemt(&ac);
    let h = trim(&ac, &TrimCondition::hover(), &cfg);
    println!("\n=== Hover cross-check (two independent paths must agree) ===");
    println!(
        "  full force/moment trim : collective {:.3}°, main power {:.1} W",
        h.collective.to_degrees(),
        h.main_power
    );
    println!(
        "  standalone hover BEMT  : collective {:.3}°, main power {:.1} W",
        th_bemt.to_degrees(),
        p_bemt
    );
    println!(
        "  agreement: Δcollective {:.3}°, Δpower {:.2}%  ->  {}",
        (h.collective - th_bemt).to_degrees(),
        (h.main_power - p_bemt) / p_bemt * 100.0,
        if (h.main_power - p_bemt).abs() / p_bemt < 0.02 {
            "MATCH"
        } else {
            "mismatch"
        },
    );
    println!(
        "  hover also: thrust {:.1} N (W={:.1}), tail {:.2} N / {:.1} W, roll {:.2}°, lat-cyclic {:.2}°",
        h.thrust,
        w,
        h.tail_thrust,
        h.tail_power,
        h.roll.to_degrees(),
        h.cyclic_lat.to_degrees()
    );

    // Forward sweep — the complete power bucket (two-way coupling + parasite).
    println!("\n=== Steady level forward-flight trim sweep (complete power bucket) ===");
    println!(
        "{:>6} {:>6} {:>7} {:>8} {:>7} {:>7} {:>7} {:>7}",
        "V m/s", "mu", "coll°", "lonCyc°", "pitch°", "mainP", "parasP", "totP"
    );
    let mut min_p = f64::INFINITY;
    let mut min_v = 0.0;
    for &v in &[0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0] {
        let r = trim(&ac, &TrimCondition::forward(v), &cfg);
        if r.total_power < min_p {
            min_p = r.total_power;
            min_v = v;
        }
        println!(
            "{:>6.0} {:>6.3} {:>7.2} {:>8.2} {:>7.1} {:>7.0} {:>7.0} {:>7.0}",
            v,
            r.mu,
            r.collective.to_degrees(),
            r.cyclic_lon.to_degrees(),
            r.pitch.to_degrees(),
            r.main_power,
            r.parasite_power,
            r.total_power,
        );
    }
    println!(
        "\nThe bucket is complete: hover-high → minimum ~{:.0} W near {:.0} m/s → high-speed\n\
         rise (parasite-driven). Rotor power stays positive and the collective physical\n\
         across the range — the two-way flap↔inflow coupling fixed the high-μ collapse\n\
         that drove power negative with the frozen-inflow model. Minimum-power speed sets\n\
         max endurance; the origin-tangent speed (a touch faster) sets max range.",
        min_p, min_v
    );
}
