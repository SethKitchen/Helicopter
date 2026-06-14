//! `attitude` subcommand: attitude hold (5k) — the outer attitude loop wrapping
//! the 5j rate damper. Shows the pre-computed target (the hover phugoid residual
//! → LHP), clean regulation OFF the seam, and the honest hover seam-residual.

use helisim_dynamics::{Inertia, eigenvalues};
use helisim_sim::{
    PiAttitudeHold, RateSas, Trim, attitude_hold, augmented_matrix, closed_loop_matrix,
    control_matrix11, control_matrix11_at, equilibrium_state11, equilibrium_state11_at,
    linearize11, linearize11_at, simulate11_sas, simulate13,
};
use helisim_trim::Aircraft;

fn max_re(a: &[Vec<f64>]) -> f64 {
    eigenvalues(a).iter().map(|e| e.re).fold(f64::MIN, f64::max)
}

pub fn run() {
    let ac = Aircraft::demo();
    let j = Inertia {
        mass: 8.0,
        i_xx: 0.4,
        i_yy: 0.8,
        i_zz: 1.0,
    };
    let rate = RateSas::rate_damper(0.2, 0.2, 0.4);
    let hold = attitude_hold(RateSas::rate_damper(0.2, 0.2, 0.4), 0.1, 0.1);
    let vt = ac.main_op.tip_speed(ac.main.radius);

    println!("helisim — attitude hold (5k): outer attitude loop on the rate damper\n");
    println!(
        "Cascade: θ→lon-cyclic, φ→lat-cyclic (gains 0.1) wrapping the validated 5j\n\
         rate damper. A HOLD, not command-tracking or guidance — it regulates to the\n\
         trim attitude. Designed off the wake-skew seam, confirmed across it.\n"
    );

    // The pre-computed target: the hover phugoid residual the rate damper can't reach.
    let a_h = linearize11(&ac, j);
    let b_h = control_matrix11(&ac, j);
    println!("Pre-computed target — the hover phugoid residual the rate damper left at +0.024:");
    println!(
        "  rate damper only : max Re {:+.4}  (slow mode rate feedback can't reach)",
        max_re(&closed_loop_matrix(&a_h, &b_h, &rate))
    );
    println!(
        "  + attitude hold  : max Re {:+.4}  (the attitude loop drives it into the LHP)\n",
        max_re(&closed_loop_matrix(&a_h, &b_h, &hold))
    );

    // Off-seam trustworthy oracle.
    let vel = [5.0, 0.0, 0.0];
    let a_f = linearize11_at(&ac, j, vel);
    let b_f = control_matrix11_at(&ac, j, vel);
    println!(
        "Off the seam (5 m/s, μ={:.3}) — the trustworthy oracle, fully stable:",
        vel[0] / vt
    );
    println!(
        "  + attitude hold  : max Re {:+.4}\n",
        max_re(&closed_loop_matrix(&a_f, &b_f, &hold))
    );

    // Regulation: off-seam returns to trim; hover bounded but seam-residual.
    let dt = 0.01;
    let mut pert = [0.0; 11];
    pert[3] = 5f64.to_radians();
    let eqf = equilibrium_state11_at(&ac, vel);
    let eqh = equilibrium_state11(&ac);
    let off = simulate11_sas(&ac, j, vel, &Trim, &hold, pert, dt, 16.0);
    let hov = simulate11_sas(&ac, j, [0.0, 0.0, 0.0], &Trim, &hold, pert, dt, 16.0);
    let hov_d = simulate11_sas(&ac, j, [0.0, 0.0, 0.0], &Trim, &rate, pert, dt, 16.0);

    println!("Regulation from a θ=5° disturbance, attitude hold active:");
    println!(
        "{:>5} {:>14} {:>14} {:>16}",
        "t s", "OFF-seam θ", "hover θ", "hover θ (damper)"
    );
    for t in [0.0, 4.0, 8.0, 12.0, 16.0] {
        let k = (t / dt) as usize;
        let fmt = |s: &[f64; 11], e: &[f64; 11]| {
            if s[3].is_finite() {
                format!("{:.2}°", (s[3] - e[3]).to_degrees())
            } else {
                "NaN".into()
            }
        };
        println!(
            "{:>5.0} {:>14} {:>14} {:>16}",
            t,
            fmt(&off[k], &eqf),
            fmt(&hov[k], &eqh),
            fmt(&hov_d[k], &eqh)
        );
    }
    println!(
        "\n  OFF the seam attitude hold returns to trim and holds (the clean oracle).\n  \
         ACROSS the seam at hover it beats the damper (which diverges to NaN) and keeps\n  \
         pitch/roll bounded, but a slow residual drift remains — the wake-skew coupling\n  \
         the hover Jacobian can't see, the SAME 5i/5j seam limitation. Fully killing it\n  \
         needs the off-seam-trustworthy outer (velocity/position) loop — milestone 5m.\n"
    );

    // 5l — integral action: zero steady-state attitude error.
    let pi = PiAttitudeHold::new(hold, 0.3, 0.3);
    let p_only = PiAttitudeHold::new(hold, 0.0, 0.0);
    println!("Integral action (5l) — zero steady-state error to a sustained disturbance:");
    println!(
        "  off-seam augmented (13-state) closed-loop max Re = {:.4} (stable, but MARGINAL —",
        max_re(&augmented_matrix(&a_f, &b_f, &pi))
    );
    println!(
        "    the integrator's own near-origin pole; firm margin is the velocity loop's job, 5m)"
    );
    let dist = [0.0, 0.6, 0.0];
    let tw = 14.0;
    let dp = simulate13(&ac, j, vel, &Trim, &p_only, dist, [0.0; 11], dt, tw);
    let dpi = simulate13(&ac, j, vel, &Trim, &pi, dist, [0.0; 11], dt, tw);
    let k = (tw / dt) as usize;
    println!("  under a sustained 0.6 N·m pitch moment, at t={tw}s:");
    println!(
        "    proportional  : θ offset {:.2}°",
        (dp[k][3] - eqf[3]).to_degrees()
    );
    println!(
        "    PI            : θ offset {:.3}°  (driven to ≈0 — the integral property)",
        (dpi[k][3] - eqf[3]).to_degrees()
    );
    println!(
        "  But the forward speed has drifted {:.2} m/s: attitude hold zeroes ATTITUDE\n  \
         error, not velocity — that residual drift is exactly what velocity/position\n  \
         hold (5m) closes. (Anti-windup not yet needed at these amplitudes — named\n  \
         for when it is.)",
        (dpi[k][0] - eqf[0]).abs()
    );
}
