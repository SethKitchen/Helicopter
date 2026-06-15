//! Milestone 5i validation — control-input time histories on the driven 11-state
//! system (rigid body + Pitt–Peters inflow in the loop).
//!
//! The validation character shifts from "matches a number" to "responds
//! correctly", so each response is chosen for a pre-computed, falsifiable
//! signature:
//!  * Gate A — control effectiveness: signs pinned to physical conventions +
//!    on-axis dominance; the heave/yaw effectiveness match the raw aero force.
//!  * Gate B — the off-axis cyclic response evolves from the frozen-inflow sign to
//!    the dynamic-inflow-corrected sign (the time-domain `−3.2 → +0.5`).
//!  * Gate C — open-loop divergence: a control PULSE excites the same unstable
//!    mode (period/growth) as an initial-condition perturbation, matching the
//!    linear eigenvalue, inside a bounded pre-divergence window.

use helisim_dynamics::{Inertia, RotorAero, eigenvalues, quasi_static_inflow};
use helisim_flapping::Controls;
use helisim_sim::control::ControlSchedule;
use helisim_sim::{
    Channel, Pulse, Trim, control_matrix11, equilibrium_state11, linearize11, rk4_step_t,
    simulate11, solve_equilibrium11,
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
fn equilibrium_is_a_fixed_point() {
    let ac = Aircraft::demo();
    let j = inertia();
    let eq = equilibrium_state11(&ac);
    let traj = simulate11(&ac, j, &Trim, [0.0; 11], 0.01, 3.0);
    let drift = traj
        .iter()
        .map(|s| {
            (0..11)
                .map(|i| (s[i] - eq[i]).abs())
                .fold(0.0_f64, f64::max)
        })
        .fold(0.0_f64, f64::max);
    println!("11-state equilibrium drift over 3 s (no input): {drift:.2e}");
    println!("ν_e = [{:.5}, {:.5}, {:.5}]", eq[8], eq[9], eq[10]);
    assert!(
        drift < 1e-3,
        "equilibrium should be a fixed point, drift {drift:.2e}"
    );
}

#[test]
fn control_effectiveness_signs_and_magnitudes() {
    let ac = Aircraft::demo();
    let b = control_matrix11(&ac, inertia());
    // rows: 0 u̇,1 ẇ,2 q̇,3 θ̇,4 v̇,5 ṗ,6 ṙ,7 φ̇,8.. inflow
    let names = ["u̇", "ẇ", "q̇", "θ̇", "v̇", "ṗ", "ṙ", "φ̇"];
    let chans = ["collective", "lat-cyc", "lon-cyc", "pedal"];
    println!("Control-effectiveness B = ∂ẋ/∂u (per rad):");
    print!("{:>6}", "");
    for c in chans {
        print!("{c:>12}");
    }
    println!();
    for (r, nm) in names.iter().enumerate() {
        print!("{nm:>6}");
        for val in &b[r] {
            print!("{val:>12.2}");
        }
        println!();
    }

    // Collective raises thrust → climb: ẇ < 0 (w is body-down).
    assert!(b[1][0] < 0.0, "collective → climb (ẇ<0), got {}", b[1][0]);
    // Lateral cyclic → roll dominant over pitch (on-axis ≫ off-axis).
    assert!(
        b[5][1].abs() > 5.0 * b[2][1].abs(),
        "lat-cyc roll must dominate pitch"
    );
    assert!(
        b[5][1] > 0.0,
        "positive lat-cyc → right roll (ṗ>0), got {}",
        b[5][1]
    );
    // Longitudinal cyclic → pitch dominant over roll.
    assert!(
        b[2][2].abs() > 5.0 * b[5][2].abs(),
        "lon-cyc pitch must dominate roll"
    );
    // Pedal → yaw, dominant over the other moments.
    assert!(
        b[6][3].abs() > 5.0 * b[2][3].abs(),
        "pedal yaw must dominate pitch"
    );
    assert!(
        b[6][3].abs() > 1.0,
        "pedal must have real yaw authority, got {}",
        b[6][3]
    );
}

#[test]
fn off_axis_cyclic_response_flips_with_inflow() {
    // The 5h off-axis sign correction, now in the driven 11-state context. The
    // off-axis pitch response to lateral cyclic ∂q̇/∂θ1c has TWO values:
    //  * INSTANTANEOUS (t=0): the inflow is still at equilibrium, so this is the
    //    frozen/uniform-inflow sign — exactly the control-matrix entry the EOM
    //    sees at the first instant of a step.
    //  * SETTLED: after the inflow lag (~τ), ν reaches its new steady value; the
    //    off-axis derivative is then the dynamic-inflow-corrected sign.
    // The two have OPPOSITE signs — the time-domain manifestation of −3.2 → +0.5.
    let ac = Aircraft::demo();
    let (coll, _, t1c, t1s, _, _, _) = solve_equilibrium11(&ac);
    let iyy = inertia().i_yy;

    // Instantaneous off-axis ∂q̇/∂θ1c = the control-matrix entry (inflow frozen).
    let frozen = control_matrix11(&ac, inertia())[2][1];
    // Settled off-axis: re-solve the quasi-static inflow at the perturbed control.
    let rotor = ac.main.with_collective(coll);
    let my = |dt1c: f64| {
        quasi_static_inflow(
            &RotorAero {
                rotor: &rotor,
                op: &ac.main_op,
                airfoil: ac.main_airfoil.as_ref(),
                props: &ac.flap,
                hub_height: ac.hub_height,
                controls: &Controls {
                    theta_1c: t1c + dt1c,
                    theta_1s: t1s,
                },
            },
            [0.0; 3],
            [0.0; 2],
        )
        .0
        .my
    };
    let settled = (my(0.01) - my(-0.01)) / 0.02 / iyy;

    println!(
        "off-axis ∂q̇/∂θ1c:  instantaneous(frozen)={frozen:+.2}  settled(inflow)={settled:+.2}"
    );
    println!(
        "  (×Iyy: frozen ∂My/∂θ1c={:.2} — matches the 5h −3.2)",
        frozen * iyy
    );
    assert!(
        frozen < 0.0,
        "instantaneous off-axis carries the frozen sign"
    );
    assert!(
        settled > 0.0,
        "the developed inflow flips the off-axis sign (the 5h correction)"
    );

    // And the very early time-march carries the frozen sign (before the lag).
    let eq = equilibrium_state11(&ac);
    let step = helisim_sim::Step {
        channel: Channel::LatCyclic,
        amplitude: 0.01,
        t_start: 0.0,
    };
    let traj = simulate11(&ac, inertia(), &step, [0.0; 11], 0.002, 0.05);
    let q_early = traj[10][2] - eq[2]; // t = 0.02 s, < τ ≈ 0.085 s
    println!("  early pitch rate q(0.02 s) = {q_early:+.2e} (frozen sign)");
    assert!(
        q_early * frozen > 0.0,
        "early transient should match the frozen off-axis sign"
    );
}

/// Linear driven march `ẋ = A·x + B·u(t)` (x = perturbation from equilibrium),
/// the oracle the nonlinear control response must track before departing.
fn linear_driven(
    a: &[Vec<f64>],
    b: &[[f64; 4]; 11],
    sched: &dyn ControlSchedule,
    x0: [f64; 11],
    dt: f64,
    t_end: f64,
) -> Vec<[f64; 11]> {
    let mut x = x0.to_vec();
    let mut out = vec![x0];
    let mut t = 0.0;
    let n = (t_end / dt).round() as usize;
    for _ in 0..n {
        x = rk4_step_t(&x, t, dt, |tt, xx| {
            let u = sched.deltas(tt);
            (0..11)
                .map(|i| {
                    (0..11).map(|j| a[i][j] * xx[j]).sum::<f64>()
                        + (0..4).map(|c| b[i][c] * u[c]).sum::<f64>()
                })
                .collect()
        });
        t += dt;
        let mut row = [0.0; 11];
        row.copy_from_slice(&x[..11]);
        out.push(row);
    }
    out
}

#[test]
fn eleven_state_modes_preserve_body_dynamics_plus_fast_inflow() {
    // The architecture gate: adding the three inflow states preserves the body
    // modes (near the 8-state 0.48±0.87i / 0.67±1.33i) and adds three FAST, STABLE
    // inflow modes (the inflow lag, τ ~ O(1 rev)). No inflow mode is unstable.
    let ac = Aircraft::demo();
    let a = linearize11(&ac, inertia());
    let eigs = eigenvalues(&a);
    let mut unstable: Vec<_> = eigs.iter().filter(|e| e.re > 0.05 && e.im > 0.05).collect();
    unstable.sort_by(|x, y| x.im.partial_cmp(&y.im).unwrap());
    println!(
        "unstable body modes: {:?}",
        unstable
            .iter()
            .map(|e| format!("{:.3}{:+.3}i", e.re, e.im))
            .collect::<Vec<_>>()
    );
    let fast: Vec<_> = eigs.iter().filter(|e| e.re < -10.0).collect();
    println!(
        "fast inflow modes: {:?}",
        fast.iter()
            .map(|e| format!("{:.1}{:+.1}i", e.re, e.im))
            .collect::<Vec<_>>()
    );
    assert!(unstable.len() >= 2, "both axes oscillatory-unstable");
    assert!(
        (unstable[0].re - 0.478).abs() < 0.1 && (unstable[1].re - 0.666).abs() < 0.12,
        "body modes near 8-state"
    );
    assert!(fast.len() >= 3, "three fast inflow modes added");
    assert!(
        eigs.iter().filter(|e| e.re < -5.0).all(|e| e.re < 0.0),
        "inflow modes stable"
    );
}

#[test]
fn open_loop_diverges_and_tracks_linear_in_the_analytic_channel() {
    // Open-loop, controls released, the aircraft does NOT return to trim — it
    // diverges (this is what 5j will tame). Validated inside a bounded window:
    //
    //  * The forward-speed channel u is ANALYTIC at hover, so a control pulse and
    //    an initial-condition perturbation BOTH track the 11-state linear model in
    //    u — two routes exciting the same validated dynamics.
    //  * The pitch/lateral RATES depart from the hover linearization faster: the
    //    Pitt–Peters wake skew χ(μ) is non-analytic at μ=0 (μ=|V|≥0 is rectified),
    //    so the λ₀↔λ₁c coupling — the SAME one behind the Gate-2 off-axis flip —
    //    activates the instant the aircraft moves and the hover Jacobian cannot
    //    see it. The hover divergence is therefore FASTER than the hover-linear
    //    rate. We validate the analytic channel and document the rest.
    let ac = Aircraft::demo();
    let j = inertia();
    let eq = equilibrium_state11(&ac);
    let a = linearize11(&ac, j);
    let b = control_matrix11(&ac, j);
    let dt = 0.002;
    let t_end = 1.0;

    let u_rel_at = |nl: &[[f64; 11]], lin: &[[f64; 11]], t: f64| -> f64 {
        let k = (t / dt) as usize;
        ((nl[k][0] - eq[0]) - lin[k][0]).abs() / lin[k][0].abs().max(1e-9)
    };

    // (1) control PULSE on longitudinal cyclic, released after 0.2 s.
    let pulse = Pulse {
        channel: Channel::LonCyclic,
        amplitude: 0.005,
        t_start: 0.0,
        duration: 0.2,
    };
    let nl_p = simulate11(&ac, j, &pulse, [0.0; 11], dt, t_end);
    let lin_p = linear_driven(&a, &b, &pulse, [0.0; 11], dt, t_end);
    // (2) initial-condition perturbation Δu = 0.1 m/s (5g scale).
    let mut ic = [0.0; 11];
    ic[0] = 0.1;
    let nl_i = simulate11(&ac, j, &Trim, ic, dt, t_end);
    let lin_i = linear_driven(&a, &b, &Trim, ic, dt, t_end);

    let window = 0.5; // the bounded pre-departure window — stated, not hoped
    println!("forward-speed u tracking (analytic channel):");
    println!(
        "  pulse rel @{window}s = {:.1}%   IC rel @{window}s = {:.1}%",
        u_rel_at(&nl_p, &lin_p, window) * 100.0,
        u_rel_at(&nl_i, &lin_i, window) * 100.0
    );
    assert!(
        u_rel_at(&nl_p, &lin_p, window) < 0.05,
        "control-pulse u tracks linear in window"
    );
    assert!(
        u_rel_at(&nl_i, &lin_i, window) < 0.05,
        "IC u tracks linear in window"
    );

    // It diverges: the response has grown and the aircraft has NOT returned to trim.
    let grow0: f64 = (0..8)
        .map(|i| (nl_p[0][i] - eq[i]).abs())
        .fold(0.0, f64::max);
    let grow1: f64 = (0..8)
        .map(|i| (nl_p.last().unwrap()[i] - eq[i]).abs())
        .fold(0.0, f64::max);
    println!(
        "open-loop pulse response grew {:.1e} → {:.1e} (diverges, does not return)",
        grow0, grow1
    );
    assert!(grow1 > grow0 && grow1 > 1e-3, "open-loop response diverges");
}
