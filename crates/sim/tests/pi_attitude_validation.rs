//! Milestone 5l validation — PI attitude hold (integral action closing 5k's
//! residual steady-state error). A correctness fix to the attitude loop, done
//! before the velocity loop (5m) so the outer loop isn't built on an inner loop
//! with a known standing error.
//!
//!  * Gate A — the integrator doesn't destabilize: off the seam the 13-state
//!    augmented closed loop stays stable. With the accurate QR eigensolver the
//!    integrator adds a slow near-origin pole (its own mode), so the system is only
//!    *marginally* stable on its own — firm damping margin arrives with the outer
//!    velocity loop (5m). (An earlier "kI lower-bound" reading was a
//!    characteristic-polynomial artifact, exposed when the eigensolver moved to QR.)
//!  * Gate B — the falsifiable oracle: under a sustained disturbance the bounded
//!    proportional offset (5k) goes to ≈0 with integral action — zero steady-state
//!    attitude error, the textbook integral property.
//!  * Gate C — the scope boundary, made concrete: attitude error → 0, but VELOCITY
//!    drifts (attitude hold ≠ velocity hold). That residual drift is exactly what
//!    the 5m outer loop closes — it is the boundary, not a failure.

use helisim_dynamics::{Inertia, eigenvalues};
use helisim_sim::{
    PiAttitudeHold, RateSas, Sim11Setup, Trim, attitude_hold, augmented_matrix,
    control_matrix11_at, equilibrium_state11_at, linearize11_at, simulate13,
};
use helisim_trim::Aircraft;

fn inertia() -> Inertia {
    Inertia {
        mass: 8.0,
        i_xx: 0.4,
        i_yy: 0.8,
        i_zz: 1.0,
    }
}
fn proportional() -> RateSas {
    attitude_hold(RateSas::rate_damper(0.2, 0.2, 0.4), 0.1, 0.1)
}
fn pi() -> PiAttitudeHold {
    PiAttitudeHold::new(proportional(), 0.3, 0.3)
}
fn max_re(a: &[Vec<f64>]) -> f64 {
    eigenvalues(a).iter().map(|e| e.re).fold(f64::MIN, f64::max)
}

#[test]
fn integrator_does_not_destabilize_off_seam() {
    // Off the seam (trustworthy, QR eigensolver), the 13-state closed loop stays
    // stable. The integrator contributes its own slow near-origin pole, so the
    // system is only marginally stable on its own — the firm damping margin is the
    // velocity loop's job (5m). The integral gain barely moves this dominant pole
    // (it is NOT an attitude mode the integral can reposition).
    let ac = Aircraft::demo();
    let j = inertia();
    let vel = [5.0, 0.0, 0.0];
    let a = linearize11_at(&ac, j, vel);
    let b = control_matrix11_at(&ac, j, vel);
    let mr = max_re(&augmented_matrix(&a, &b, &pi()));
    let lo = max_re(&augmented_matrix(
        &a,
        &b,
        &PiAttitudeHold::new(proportional(), 0.1, 0.1),
    ));
    println!(
        "off-seam 13-state max Re (QR): kI=0.3 → {mr:.4}, kI=0.1 → {lo:.4} (gain-independent slow pole)"
    );
    assert!(
        mr < 1e-3,
        "PI integrator does not destabilize (stable, if marginally — 5m firms it)"
    );
    assert!(
        (mr - lo).abs() < 1e-3,
        "the near-origin pole is gain-independent — not the integral's to move"
    );
}

#[test]
fn integral_action_zeroes_the_steady_state_attitude_error() {
    // The falsifiable oracle: under a sustained pitch-moment disturbance, the
    // proportional law leaves a standing offset; integral action drives it to ≈0.
    let ac = Aircraft::demo();
    let j = inertia();
    let vel = [5.0, 0.0, 0.0];
    let eq = equilibrium_state11_at(&ac, vel);
    let dt = 0.01;
    let dist = [0.0, 0.6, 0.0]; // sustained pitch moment (N·m)
    let t = 14.0; // window: attitude has settled, before the (separate) velocity drift dominates

    let p_only = PiAttitudeHold::new(proportional(), 0.0, 0.0);
    let setup = Sim11Setup { ac: &ac, j, vel };
    let d_p = simulate13(&setup, &Trim, &p_only, dist, [0.0; 11], [dt, t]);
    let d_pi = simulate13(&setup, &Trim, &pi(), dist, [0.0; 11], [dt, t]);
    let k = (t / dt) as usize;
    let off_p = (d_p[k][3] - eq[3]).to_degrees().abs();
    let off_pi = (d_pi[k][3] - eq[3]).to_degrees().abs();
    println!(
        "sustained {} N·m at t={t}s: proportional θ offset {off_p:.2}°, PI θ offset {off_pi:.3}°",
        dist[1]
    );
    assert!(
        off_p > 1.0,
        "proportional control leaves a standing attitude offset"
    );
    assert!(
        off_pi < 0.3,
        "integral action zeroes the steady-state attitude error"
    );
    assert!(
        off_pi < 0.2 * off_p,
        "PI offset is a small fraction of the proportional offset"
    );
}

#[test]
fn attitude_hold_zeroes_attitude_but_leaves_a_velocity_drift() {
    // The scope boundary, concrete: with the attitude regulated to ≈0 by integral
    // action, the forward speed u DRIFTS (the disturbance-countering thrust tilt
    // accelerates the aircraft). Attitude hold ≠ velocity hold — this residual is
    // what the 5m outer loop closes. Asserted as present, to mark the boundary.
    let ac = Aircraft::demo();
    let j = inertia();
    let vel = [5.0, 0.0, 0.0];
    let eq = equilibrium_state11_at(&ac, vel);
    let dt = 0.01;
    let dist = [0.0, 0.6, 0.0];
    let t = 14.0;
    let d = simulate13(
        &Sim11Setup { ac: &ac, j, vel },
        &Trim,
        &pi(),
        dist,
        [0.0; 11],
        [dt, t],
    );
    let k = (t / dt) as usize;
    let theta = (d[k][3] - eq[3]).to_degrees().abs();
    let u_drift = (d[k][0] - eq[0]).abs();
    println!(
        "at t={t}s: attitude θ held at {theta:.3}°, but forward speed drifted {u_drift:.2} m/s"
    );
    assert!(theta < 0.3, "attitude is regulated");
    assert!(
        u_drift > 0.5,
        "velocity drifts — attitude hold does not regulate speed (→ 5m)"
    );
}
