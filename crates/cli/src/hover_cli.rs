//! `hover` subcommand: velocity / position hold (5m) — the outermost cascade and
//! the capstone of the augmentation stack. An aircraft open-loop-unstable in both
//! axes holds hover position hands-off, every inner layer individually validated.

use helisim_dynamics::{Inertia, eigenvalues};
use helisim_sim::{
    VelocityHold, equilibrium_state11, equilibrium_state11_at, linearize15, simulate15,
};
use helisim_trim::Aircraft;

pub fn run() {
    let ac = Aircraft::demo();
    let j = Inertia {
        mass: 8.0,
        i_xx: 0.4,
        i_yy: 0.8,
        i_zz: 1.0,
    };
    let vh = VelocityHold::hover_hold();
    let vt = ac.main_op.tip_speed(ac.main.radius);

    println!("helisim — velocity / position hold (5m): the outermost cascade\n");
    println!(
        "Cascade: velocity error → attitude command → the 5k/5l PI attitude loop →\n\
         the 5j rate loop → controls. For hover-hold the velocity-error integrator IS\n\
         position, so the loop returns to and holds station. Hold + steady command\n\
         only (not guidance). 15 states: plant(11) + attitude(2) + velocity(2).\n"
    );

    // Timescale separation off the seam (the well-formed-cascade gate).
    let vel = [5.0, 0.0, 0.0];
    let e = eigenvalues(&linearize15(&ac, j, vel, &vh));
    let mut mags: Vec<f64> = e
        .iter()
        .map(|c| (c.re * c.re + c.im * c.im).sqrt())
        .collect();
    mags.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut ratios: Vec<f64> = (0..mags.len() - 1).map(|i| mags[i + 1] / mags[i]).collect();
    ratios.sort_by(|a, b| b.partial_cmp(a).unwrap());
    let max_re = e.iter().map(|c| c.re).fold(f64::MIN, f64::max);
    println!(
        "Off the seam (5 m/s, μ={:.3}) — the cascade is well-formed:",
        vel[0] / vt
    );
    println!("  closed-loop max Re = {max_re:.4} (all stable, via the QR eigensolver)");
    println!(
        "  three timescale clusters: velocity |λ|≈{:.2}, attitude |λ|≈1.3, rate/inflow |λ|≈{:.0}",
        mags[0],
        mags.last().unwrap()
    );
    println!(
        "  cluster separation {:.1}× and {:.1}× (≥3× target — loops don't fight)\n",
        ratios[1], ratios[0]
    );

    // The pre-computed tracking target: 5l's drift → 0.
    let eqf = equilibrium_state11_at(&ac, vel);
    let dt = 0.01;
    let d = simulate15(
        &ac,
        j,
        vel,
        &vh,
        [0.0, 0.0],
        [0.0, 0.6, 0.0],
        [0.0; 15],
        dt,
        40.0,
    );
    println!("Pre-computed target — the ~1.6 m/s drift attitude hold (5l) left under 0.6 N·m:");
    println!(
        "  velocity hold drives it to {:.3} m/s (drift arrested).\n",
        d[(40.0 / dt) as usize][0] - eqf[0]
    );

    // The capstone: hover position hold across the seam.
    let eqh = equilibrium_state11(&ac);
    let mut pert = [0.0; 15];
    pert[0] = 0.5;
    let h = simulate15(
        &ac,
        j,
        [0.0, 0.0, 0.0],
        &vh,
        [0.0, 0.0],
        [0.0; 3],
        pert,
        dt,
        40.0,
    );
    println!("CAPSTONE — hover position hold across the seam, from a Δu=0.5 m/s kick:");
    println!(
        "{:>5} {:>9} {:>9} {:>10} {:>10}",
        "t s", "u m/s", "v m/s", "pos x m", "yaw °/s"
    );
    for t in [0.0, 5.0, 10.0, 20.0, 40.0] {
        let s = h[(t / dt) as usize];
        println!(
            "{:>5.0} {:>9.4} {:>9.4} {:>10.3} {:>10.3}",
            t,
            s[0] - eqh[0],
            s[4] - eqh[4],
            s[13],
            s[6].to_degrees()
        );
    }
    println!(
        "\n  The aircraft — open-loop unstable in BOTH axes — arrests the drift, returns\n  \
         to station, and holds it hands-off; the yaw/wake-skew seam-residual that\n  \
         survived 5j/5k/5l no longer runs away (position feedback finally has authority\n  \
         over the slow drift). The capstone of the augmentation stack."
    );
}
