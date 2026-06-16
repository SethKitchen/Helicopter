//! Validation of the **viscous** Joukowski airfoil (the cylinder solver carrying the
//! conformal metric). The oracle is *response correctness* plus the inviscid Joukowski
//! reference — the rotor-relevant facts the inviscid map could not give:
//!
//!  * **symmetry** — zero lift at zero incidence;
//!  * **profile drag** — positive viscous `C_d` (the inviscid map gives `C_d = 0` by
//!    d'Alembert, so this is genuinely the viscous contribution);
//!  * **lift response** — `C_l` is positive for `α > 0` and grows ~linearly with `α`;
//!  * **lift-magnitude recovery** — imposing the inviscid **Kutta circulation** in the
//!    far field lifts `C_l` several-fold back toward the inviscid value (a plain
//!    uniform-flow boundary on a finite domain suppresses the slowly-decaying bound
//!    circulation). The residual below inviscid is the genuine low-Re viscous + rounded-
//!    TE soft-Kutta + finite-domain reduction.

use helisim_cfd::{AirfoilConfig, solve_airfoil_viscous};

fn run(deg: f64, kutta: bool) -> helisim_cfd::AirfoilViscousSolution {
    let cfg = AirfoilConfig {
        n_r: 72,
        n_t: 112,
        re_chord: 200.0,
        omega_relax: 0.3,
        te_round: 0.1,
        psi_sweeps: 10,
        r_max: 30.0,
        max_steps: 6000,
        kutta_far_field: kutta,
        ..AirfoilConfig::new(deg, 200.0)
    };
    solve_airfoil_viscous(&cfg)
}

#[test]
fn viscous_airfoil_drag_lift_and_kutta_recovery() {
    let (s0, s3, s6) = (run(0.0, false), run(3.0, false), run(6.0, false));
    let s6k = run(6.0, true);
    assert!(
        s0.converged && s3.converged && s6.converged,
        "plain solves converge"
    );

    let (cl0, cd0) = s0.force_coefficients();
    let (cl3, _) = s3.force_coefficients();
    let (cl6, cd6) = s6.force_coefficients();
    let (cl6k, _) = s6k.force_coefficients();

    // Symmetry: zero lift at zero incidence.
    assert!(cl0.abs() < 1e-3, "Cl(0) = {cl0} should be ≈ 0");

    // Profile drag: positive and substantial — the viscous contribution the inviscid
    // map cannot produce (it gives Cd = 0 by d'Alembert).
    assert!(
        cd0 > 0.05 && cd6 > 0.0,
        "positive profile drag (Cd0={cd0}, Cd6={cd6})"
    );

    // Lift develops with the correct sign and ~linearly in α (plain far field).
    assert!(
        cl3 > 0.0 && cl6 > 0.0,
        "lift positive for α > 0 (Cl3={cl3}, Cl6={cl6})"
    );
    assert!(
        (cl6 / cl3 - 2.0).abs() < 0.3,
        "lift ~linear: Cl(6)/Cl(3) = {}",
        cl6 / cl3
    );

    // Refinement: the Kutta far field recovers the lift several-fold toward inviscid,
    // while staying below it (the documented viscous/soft-Kutta/finite-domain reduction).
    let inv = s6k.inviscid_lift();
    assert!(
        cl6k > 2.5 * cl6,
        "Kutta far field recovers lift: {cl6k} vs plain {cl6}"
    );
    assert!(
        cl6k > 0.3 * inv && cl6k < inv,
        "recovered Cl {cl6k} in (0.3, 1)·inviscid {inv}"
    );
}
