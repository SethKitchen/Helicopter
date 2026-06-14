//! Milestone 5m validation — velocity / position hold (the outermost cascade).
//! The capstone of the augmentation stack: an aircraft open-loop-unstable in both
//! axes holding hover position hands-off, every layer individually validated.
//!
//!  * Gate A — the cascade is WELL-FORMED off the seam: the 15-state closed loop
//!    is stable and its eigenvalues form three clusters (fast rate/inflow, mid
//!    attitude, slow velocity) separated by ≥3× (the named timescale-separation
//!    ratio, chosen before tuning). If the loops fought, an outer mode would crowd
//!    the attitude cluster.
//!  * Gate B — the pre-computed tracking target: the ~1.6 m/s velocity drift that
//!    attitude hold (5l) left under a sustained disturbance is driven to ≈0.
//!  * Gate C — the capstone, ACROSS the seam: at hover the same gains arrest a
//!    velocity disturbance, return position to ≈0 and hold it, AND the yaw/wake-skew
//!    seam-residual the inner loops left no longer drifts — position feedback has
//!    authority over the slow drift the inner loops are blind to.
//!  * Gate D — anti-windup is still not needed (commanded attitudes stay modest);
//!    named so the boundary is explicit.

use helisim_dynamics::{Inertia, eigenvalues};
use helisim_sim::{
    PiAttitudeHold, RateSas, Trim, VelocityHold, attitude_hold, equilibrium_state11,
    equilibrium_state11_at, linearize15, simulate13, simulate15,
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

#[test]
fn cascade_is_well_formed_off_seam() {
    let ac = Aircraft::demo();
    let j = inertia();
    let vel = [5.0, 0.0, 0.0];
    let e = eigenvalues(&linearize15(&ac, j, vel, &VelocityHold::hover_hold()));
    assert!(
        e.iter().all(|c| c.re < 0.0),
        "the full cascade is stable off the seam"
    );

    // Three clusters by |λ|: the two largest consecutive ratio-gaps are the cluster
    // boundaries and must each clear the named ≥3× timescale separation.
    let mut mags: Vec<f64> = e
        .iter()
        .map(|c| (c.re * c.re + c.im * c.im).sqrt())
        .collect();
    mags.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let mut ratios: Vec<f64> = (0..mags.len() - 1).map(|i| mags[i + 1] / mags[i]).collect();
    ratios.sort_by(|a, b| b.partial_cmp(a).unwrap());
    println!(
        "two largest |λ| gaps (cluster boundaries): {:.1}×, {:.1}×",
        ratios[0], ratios[1]
    );
    println!(
        "|λ| range: slow {:.3}, fast {:.1}",
        mags.first().unwrap(),
        mags.last().unwrap()
    );
    const SEP: f64 = 3.0; // named separation target (textbook 3–5× cascade rule)
    assert!(
        ratios[1] >= SEP,
        "rate/attitude/velocity form 3 clusters each ≥{SEP}× separated"
    );
}

/// DOCUMENTED handling-qualities check (closes the "tuned gains, no published
/// basis" gap from validation/ORACLE_COVERAGE.md). ADS-33E / MIL-F-9490D specify
/// a **Level-1 minimum damping ratio ζ ≥ 0.35** for the closed-loop oscillatory
/// modes. Computing ζ = −Re/|λ| from the validated off-seam 15-state closed loop:
/// the **velocity/position-hold modes (|λ|<0.3) MEET Level 1** (ζ ≈ 0.45, 0.76),
/// while the faster **body/attitude modes (|λ|≈1.3) sit at ζ ≈ 0.10**, below it.
///
/// Honest finding: the cascade is stable and its outer (slow) modes are
/// well-damped to the published criterion, but the rate-damper gains were tuned
/// for timescale separation + stability, NOT inner-loop damping — raising them to
/// bring the body modes to ζ≥0.35 is the named next step for full HQ compliance.
#[test]
fn closed_loop_damping_vs_ads33_level1() {
    let ac = Aircraft::demo();
    let j = inertia();
    let vel = [5.0, 0.0, 0.0];
    let e = eigenvalues(&linearize15(&ac, j, vel, &VelocityHold::hover_hold()));
    const LEVEL1: f64 = 0.35; // ADS-33E / MIL-F-9490D Level-1 minimum damping ratio

    let mut slow_ok = false;
    for c in &e {
        if c.im <= 1e-6 {
            continue; // not oscillatory
        }
        let mag = (c.re * c.re + c.im * c.im).sqrt();
        let zeta = -c.re / mag;
        assert!(c.re < 0.0, "every oscillatory mode is stable");
        if mag < 0.3 {
            // velocity/position-hold cluster — must meet the published Level-1 bound.
            assert!(zeta >= LEVEL1, "slow mode |λ|={mag:.2} ζ={zeta:.2} below Level-1");
            slow_ok = true;
        }
    }
    assert!(slow_ok, "found the slow velocity/position oscillatory modes");
}

#[test]
fn velocity_hold_zeroes_the_drift_attitude_hold_left() {
    // The pre-computed target: 5l left ~1.6 m/s drift under a sustained 0.6 N·m
    // moment; the velocity loop drives it to ≈0.
    let ac = Aircraft::demo();
    let j = inertia();
    let vel = [5.0, 0.0, 0.0];
    let eq = equilibrium_state11_at(&ac, vel);
    let dt = 0.01;
    let dist = [0.0, 0.6, 0.0];

    // 5l attitude hold alone: a drift that keeps growing (eventually out of range).
    // Measured in its valid window (t=14 s), where it is already ~1.6 m/s.
    let pi = PiAttitudeHold::new(
        attitude_hold(RateSas::rate_damper(0.2, 0.2, 0.4), 0.1, 0.1),
        0.3,
        0.3,
    );
    let att = simulate13(&ac, j, vel, &Trim, &pi, dist, [0.0; 11], dt, 14.0);
    let drift_5l = (att[(14.0 / dt) as usize][0] - eq[0]).abs();

    // 5m velocity hold: the same drift, now driven to ≈0 (settled by t=40 s).
    let vh = simulate15(
        &ac,
        j,
        vel,
        &VelocityHold::hover_hold(),
        [0.0, 0.0],
        dist,
        [0.0; 15],
        dt,
        40.0,
    );
    let drift_5m = (vh[(40.0 / dt) as usize][0] - eq[0]).abs();

    println!(
        "sustained 0.6 N·m: attitude-hold drift {drift_5l:.2} m/s (still growing) → velocity-hold {drift_5m:.3} m/s (arrested)"
    );
    assert!(
        drift_5l > 0.5,
        "attitude hold alone leaves a real velocity drift"
    );
    assert!(drift_5m < 0.1, "velocity hold drives the drift to ≈0");
    assert!(
        drift_5m < 0.1 * drift_5l,
        "the outer loop closes the drift the inner loop left"
    );
}

#[test]
fn hover_position_hold_is_the_capstone() {
    // ACROSS the seam: the open-loop-unstable-both-axes aircraft holds hover
    // position hands-off. From a velocity disturbance, drift is arrested, position
    // returns to ≈0, and the yaw/wake-skew seam-residual no longer runs away.
    let ac = Aircraft::demo();
    let j = inertia();
    let eq = equilibrium_state11(&ac);
    let dt = 0.01;
    let t = 40.0;
    let mut pert = [0.0; 15];
    pert[0] = 0.5; // Δu = 0.5 m/s

    let h = simulate15(
        &ac,
        j,
        [0.0, 0.0, 0.0],
        &VelocityHold::hover_hold(),
        [0.0, 0.0],
        [0.0; 3],
        pert,
        dt,
        t,
    );
    let s = h[(t / dt) as usize];
    assert!(
        s.iter().all(|v| v.is_finite()),
        "hover hold stays finite (does not run away)"
    );
    let (u, v) = (s[0] - eq[0], s[4] - eq[4]);
    let (pos_x, pos_y) = (s[13], s[14]);
    let yaw_rate = s[6].to_degrees();
    println!(
        "hover hold from Δu=0.5 m/s at t={t}s: u={u:.3} v={v:.3}, position ({pos_x:.3},{pos_y:.3}) m, yaw rate {yaw_rate:.2}°/s"
    );
    assert!(
        u.abs() < 0.05 && v.abs() < 0.05,
        "velocity drift arrested at hover"
    );
    assert!(
        pos_x.abs() < 0.2 && pos_y.abs() < 0.2,
        "position returned to ≈0 and held"
    );
    assert!(
        yaw_rate.abs() < 1.0,
        "the yaw/wake-skew seam-residual is regulated — no runaway drift"
    );
}

#[test]
fn commanded_attitudes_stay_modest_no_windup() {
    // Anti-windup not yet needed: even under the sustained disturbance the outer
    // loop commands only a small attitude, so the controls never saturate.
    let ac = Aircraft::demo();
    let j = inertia();
    let vh = VelocityHold::hover_hold();
    let dt = 0.01;
    let d = simulate15(
        &ac,
        j,
        [5.0, 0.0, 0.0],
        &vh,
        [0.0, 0.0],
        [0.0, 0.6, 0.0],
        [0.0; 15],
        dt,
        40.0,
    );
    // θ_cmd = k_u·u_err + ki_u·ζ_u; track its peak over the run.
    let eq = equilibrium_state11_at(&ac, [5.0, 0.0, 0.0]);
    let theta_cmd_peak = d
        .iter()
        .map(|s| (vh.k_u * (s[0] - eq[0]) + vh.ki_u * s[13]).abs())
        .fold(0.0_f64, f64::max);
    println!(
        "peak commanded attitude θ_cmd = {:.2}° (modest → anti-windup not needed)",
        theta_cmd_peak.to_degrees()
    );
    assert!(
        theta_cmd_peak.to_degrees() < 5.0,
        "commanded attitude stays small — no integrator windup"
    );
}
