//! `forward` subcommand: forward-flight BEMT sweep over advance ratio —
//! the power bucket, and the uncommanded rolling moment of a rigid blade.

use helisim_airfoil::LinearAirfoil;
use helisim_forward::{ForwardCondition, ForwardConfig, ForwardSolution, solve_forward};
use helisim_rotor::{Operating, Rotor};

/// Trim collective so forward-flight C_T equals `target` (C_T rises with pitch).
fn trim_to_ct(
    rotor: &Rotor,
    op: &Operating,
    af: &LinearAirfoil,
    cond: &ForwardCondition,
    target: f64,
    cfg: &ForwardConfig,
) -> ForwardSolution {
    let ct_at = |th: f64| solve_forward(&rotor.with_collective(th), op, af, cond, cfg).ct;
    let (mut lo, mut hi) = (0.0_f64, 18f64.to_radians());
    for _ in 0..80 {
        let m = 0.5 * (lo + hi);
        if ct_at(m) < target {
            lo = m;
        } else {
            hi = m;
        }
    }
    solve_forward(&rotor.with_collective(0.5 * (lo + hi)), op, af, cond, cfg)
}

pub fn run() {
    let rotor = Rotor::rectangular(2, 1.143, 0.191, 8f64.to_radians(), 0.15);
    let op = Operating::from_tip_mach(0.439, 1.143);
    let af = LinearAirfoil::naca0012();
    let cfg = ForwardConfig::default();
    let target_ct = 0.006;
    let f_over_a = 0.01; // representative airframe flat-plate / disk-area ratio
    let vt = op.tip_speed(rotor.radius);

    println!("helisim — forward-flight BEMT (Glauert inflow + azimuthal integration)\n");
    println!(
        "Rotor: C&T geometry, R={:.3} m, {:.0} RPM, M_tip={:.2}; trimmed to C_T={:.4}",
        rotor.radius,
        op.rpm(),
        op.tip_mach(rotor.radius),
        target_ct
    );
    println!("Disk level (α=0), uniform inflow, reverse-flow lift nulled.\n");

    println!(
        "{:>5} {:>6} {:>7} {:>8} {:>9} {:>10} {:>9} {:>8} {:>7}",
        "mu", "V m/s", "coll°", "lambda_i", "Pind/Ph", "rotorCp", "totCp*", "Mroll", "rev%"
    );
    for k in 0..=9 {
        let mu = k as f64 * 0.05;
        let cond = ForwardCondition::new(mu, 0.0);
        let s = trim_to_ct(&rotor, &op, &af, &cond, target_ct, &cfg);
        let v = mu * vt;
        let coll = trimmed_collective(&rotor, &op, &af, &cond, target_ct, &cfg).to_degrees();
        let pind_ratio = s.cp_induced / hover_induced(&rotor, &op, &af, target_ct, &cfg);
        let tot_cp = s.cp + 0.5 * f_over_a * mu * mu * mu;
        let m_roll = s.rolling_moment_nm(&op, &rotor);
        println!(
            "{:>5.2} {:>6.1} {:>7.2} {:>8.4} {:>9.2} {:>10.6} {:>9.6} {:>7.0}N·m {:>6.1}%",
            mu,
            v,
            coll,
            s.lambda_i,
            pind_ratio,
            s.cp,
            tot_cp,
            m_roll,
            s.reverse_flow_fraction * 100.0,
        );
    }
    println!(
        "\n* totCp = rotor C_P + representative airframe parasite 0.5·(f/A)·μ³ (f/A={f_over_a})."
    );
    println!("  The rotor C_P falls monotonically (induced power collapses); the bucket's");
    println!("  high-speed rise is the parasite term — the airframe model arrives with trim.");

    // Headline: the rolling moment of a rigid blade at a cruise advance ratio.
    let cond = ForwardCondition::new(0.30, 0.0);
    let s = trim_to_ct(&rotor, &op, &af, &cond, target_ct, &cfg);
    println!("\n=== Headline: rigid (un-flapped) blade at μ=0.30 ===");
    println!(
        "  advancing-half C_T {:.5}  vs  retreating-half C_T {:.5}  ({:.1}× more)",
        s.advancing_ct,
        s.retreating_ct,
        s.advancing_ct / s.retreating_ct
    );
    println!(
        "  uncommanded rolling moment {:.0} N·m  (C_roll {:.5}); pitching {:.0} N·m",
        s.rolling_moment_nm(&op, &rotor),
        s.c_roll,
        s.c_pitch * op.rho * rotor.disk_area() * vt * vt * rotor.radius,
    );
    println!(
        "  This large lateral moment is why blade flapping exists — the next milestone\n  \
         lets the blade flap to trade the moment for a tilt of the tip-path plane."
    );
}

/// Re-run the trim to recover just the trimmed collective (radians).
fn trimmed_collective(
    rotor: &Rotor,
    op: &Operating,
    af: &LinearAirfoil,
    cond: &ForwardCondition,
    target: f64,
    cfg: &ForwardConfig,
) -> f64 {
    let ct_at = |th: f64| solve_forward(&rotor.with_collective(th), op, af, cond, cfg).ct;
    let (mut lo, mut hi) = (0.0_f64, 18f64.to_radians());
    for _ in 0..80 {
        let m = 0.5 * (lo + hi);
        if ct_at(m) < target {
            lo = m;
        } else {
            hi = m;
        }
    }
    0.5 * (lo + hi)
}

/// Induced power coefficient at hover for the same trimmed thrust (denominator
/// for the induced-power ratio column).
fn hover_induced(
    rotor: &Rotor,
    op: &Operating,
    af: &LinearAirfoil,
    target: f64,
    cfg: &ForwardConfig,
) -> f64 {
    trim_to_ct(rotor, op, af, &ForwardCondition::hover(), target, cfg).cp_induced
}
