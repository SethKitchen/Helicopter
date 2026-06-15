//! Milestone 5j validation — stability augmentation (rate-feedback SCAS).
//!
//! The 5i lesson projected forward: the hover Jacobian is blind to the wake-skew
//! λ₀↔λ₁c coupling (non-analytic at μ=0), so it is the wrong DESIGN oracle for the
//! pitch/lateral-rate channels, for the same reason it was the wrong validation
//! gate. So the SAS is designed and validated in three layers:
//!
//!  * Gate A — OFF the seam (small forward speed) the linear model SEES the
//!    coupling and linear↔nonlinear agree, so closed-loop eigenvalues in the LHP
//!    are a TRUSTWORTHY gate. This is the oracle hover cannot provide.
//!  * Gate B — the rate damper collapses the hover instability, but a small
//!    positive residual remains: rate feedback is necessary, not sufficient (it
//!    cannot reach the slow speed/phugoid mode). Stated in the ledger.
//!  * Gate C — confirm ACROSS the seam: the same gains turn the open-loop hover
//!    divergence (blows up) into a bounded nonlinear response — including the
//!    pitch/lateral-rate channel the hover Jacobian could not see.

use helisim_dynamics::{Inertia, eigenvalues};
use helisim_sim::{
    Channel, Pulse, RateSas, Trim, closed_loop_matrix, control_matrix11, control_matrix11_at,
    equilibrium_state11, equilibrium_state11_at, linearize11, linearize11_at, rk4_step_t,
    simulate11, simulate11_sas, Sim11Setup,
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
fn sas() -> RateSas {
    RateSas::rate_damper(0.2, 0.2, 0.4)
}
fn max_re(a: &[Vec<f64>]) -> f64 {
    eigenvalues(a).iter().map(|e| e.re).fold(f64::MIN, f64::max)
}

/// Linear closed-loop march `ẋ = A_cl·x`.
fn lin_cl(acl: &[Vec<f64>], x0: [f64; 11], dt: f64, t_end: f64) -> Vec<[f64; 11]> {
    let mut x = x0.to_vec();
    let mut out = vec![x0];
    let n = (t_end / dt).round() as usize;
    for _ in 0..n {
        x = rk4_step_t(&x, 0.0, dt, |_, xx| {
            (0..11)
                .map(|i| (0..11).map(|j| acl[i][j] * xx[j]).sum())
                .collect()
        });
        let mut r = [0.0; 11];
        r.copy_from_slice(&x[..11]);
        out.push(r);
    }
    out
}

#[test]
fn off_seam_design_is_trustworthy() {
    // Small forward speed: μ ≈ 0.04, off the wake-skew seam. Here the Jacobian is
    // differentiable and the closed-loop eigenvalue gate can be trusted.
    let ac = Aircraft::demo();
    let j = inertia();
    let vel = [5.0, 0.0, 0.0];
    let a = linearize11_at(&ac, j, vel);
    let b = control_matrix11_at(&ac, j, vel);
    let acl = closed_loop_matrix(&a, &b, &sas());
    let (ol, cl) = (max_re(&a), max_re(&acl));
    println!("off-seam (5 m/s): open-loop max Re {ol:.3} → closed-loop max Re {cl:.4}");
    assert!(ol > 0.05, "open-loop is unstable off-seam too");
    assert!(
        cl < 0.0,
        "SAS drives closed-loop into the LHP off the seam (trustworthy)"
    );

    // And linear↔nonlinear AGREE off the seam — in the pitch-rate channel q that
    // the hover Jacobian was blind to — so the eigenvalue gate is meaningful here.
    let eq = equilibrium_state11_at(&ac, vel);
    let dt = 0.005;
    let mut ic = [0.0; 11];
    ic[2] = 0.05; // pitch-rate kick
    let nl = simulate11_sas(&Sim11Setup { ac: &ac, j, vel }, &Trim, &sas(), ic, [dt, 2.0]);
    let li = lin_cl(&acl, ic, dt, 2.0);
    let k = (1.0 / dt) as usize;
    let rel = ((nl[k][2] - eq[2]) - li[k][2]).abs() / li[k][2].abs().max(1e-9);
    println!(
        "  pitch-rate q: nonlinear vs linear closed-loop rel err @1s = {:.1}%",
        rel * 100.0
    );
    assert!(
        rel < 0.05,
        "off the seam, nonlinear tracks the linear closed-loop in q"
    );
}

#[test]
fn sas_damps_hover_but_leaves_a_residual() {
    // Rate feedback collapses the violent hover instability but cannot fully
    // stabilize it: a small positive residual (the slow speed/phugoid mode rate
    // feedback can't reach) remains. Necessary, not sufficient.
    let ac = Aircraft::demo();
    let j = inertia();
    let a = linearize11(&ac, j);
    let b = control_matrix11(&ac, j);
    let acl = closed_loop_matrix(&a, &b, &sas());
    let (ol, cl) = (max_re(&a), max_re(&acl));
    println!("hover: open-loop max Re {ol:.3} → closed-loop max Re {cl:.4}");
    assert!(ol > 0.6, "hover open-loop violently unstable (~0.70)");
    assert!(
        cl < 0.1 && cl < 0.1 * ol,
        "rate damper collapses the instability >10×"
    );
    assert!(
        cl > 0.0,
        "but a small positive residual remains — damper, not hold"
    );
}

#[test]
fn nonlinear_hover_holds_attitude_across_the_seam() {
    // The same gains, confirmed ACROSS the seam on the nonlinear hover march: the
    // control pulse that diverged open-loop (blows up) is held bounded by the SAS,
    // including the pitch/lateral-rate channel the hover Jacobian could not see.
    let ac = Aircraft::demo();
    let j = inertia();
    let eq = equilibrium_state11(&ac);
    let dt = 0.01;
    let t_end = 8.0;
    let pulse = Pulse {
        channel: Channel::LonCyclic,
        amplitude: 0.01,
        t_start: 0.0,
        duration: 0.2,
    };

    let open = simulate11(&ac, j, &pulse, [0.0; 11], dt, t_end);
    let aug = simulate11_sas(
        &Sim11Setup { ac: &ac, j, vel: [0.0, 0.0, 0.0] },
        &pulse,
        &sas(),
        [0.0; 11],
        [dt, t_end],
    );

    let attitude = |tr: &[[f64; 11]]| -> f64 {
        tr.iter()
            .map(|s| (s[3] - eq[3]).abs().max((s[7] - eq[7]).abs()))
            .filter(|v| v.is_finite())
            .fold(0.0_f64, f64::max)
    };
    let open_blows_up = open
        .iter()
        .any(|s| !s[3].is_finite() || (s[3] - eq[3]).abs() > 1.0);
    let aug_max = attitude(&aug);
    println!("open-loop pulse: diverges/blows up = {open_blows_up}");
    println!(
        "SAS pulse: max attitude excursion over {t_end}s = {:.1}°",
        aug_max.to_degrees()
    );
    assert!(
        open_blows_up,
        "open-loop diverges to the pulse (the 5i result)"
    );
    assert!(
        aug.iter().all(|s| s[3].is_finite()),
        "SAS response stays finite"
    );
    assert!(
        aug_max.to_degrees() < 10.0,
        "SAS holds attitude bounded across the seam"
    );
}
