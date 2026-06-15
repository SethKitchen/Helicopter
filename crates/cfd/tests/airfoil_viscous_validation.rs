//! Validation of the **viscous** Joukowski airfoil (the cylinder solver carrying the
//! conformal metric). The oracle here is *response correctness* plus the inviscid
//! Joukowski reference, not a single published number — the rotor-relevant facts the
//! inviscid map could not give:
//!
//!  * **symmetry** — zero lift at zero incidence;
//!  * **profile drag** — positive viscous `C_d` (the inviscid map gives `C_d = 0` by
//!    d'Alembert, so this is genuinely the viscous contribution);
//!  * **lift response** — `C_l` is positive for `α > 0`, grows ~linearly with `α`,
//!    and (as a same-sign cross-check) the total-circulation route agrees in sign.
//!
//! The viscous lift sits well *below* the inviscid `2π(1+ε/c)sin α`: a uniform-flow
//! outer boundary on a finite domain suppresses the slowly-decaying bound circulation
//! (documented in the module — the magnitude, not the mechanism, is the casualty).

use helisim_cfd::{AirfoilConfig, solve_airfoil_viscous};

fn run(deg: f64) -> helisim_cfd::AirfoilViscousSolution {
    let cfg = AirfoilConfig {
        n_r: 72,
        n_t: 112,
        re_chord: 200.0,
        omega_relax: 0.3,
        te_round: 0.1,
        psi_sweeps: 10,
        r_max: 30.0,
        max_steps: 6000,
        ..AirfoilConfig::new(deg, 200.0)
    };
    solve_airfoil_viscous(&cfg)
}

#[test]
fn viscous_airfoil_drag_and_lift_response() {
    let (s0, s3, s6) = (run(0.0), run(3.0), run(6.0));
    assert!(s0.converged && s3.converged && s6.converged, "all should reach steady state");

    let (cl0, cd0) = s0.force_coefficients();
    let (cl3, _) = s3.force_coefficients();
    let (cl6, cd6) = s6.force_coefficients();

    // Symmetry: zero lift at zero incidence.
    assert!(cl0.abs() < 1e-3, "Cl(0) = {cl0} should be ≈ 0");

    // Profile drag: positive and substantial — the viscous contribution the inviscid
    // map cannot produce (it gives Cd = 0 by d'Alembert).
    assert!(cd0 > 0.05, "Cd(0) = {cd0} should be a positive profile drag");
    assert!(cd6 > 0.0, "Cd at incidence positive too");

    // Lift develops with the correct sign and ~linearly in α.
    assert!(cl3 > 0.0 && cl6 > 0.0, "lift positive for α > 0 (Cl3={cl3}, Cl6={cl6})");
    assert!((cl6 / cl3 - 2.0).abs() < 0.25, "lift ~linear: Cl(6)/Cl(3) = {}", cl6 / cl3);

    // Same-sign cross-check: the independent total-circulation lift route agrees.
    assert!(s6.lift_from_circulation() > 0.0, "circulation-route lift has the same sign");

    // The viscous lift sits below the inviscid Kutta–Joukowski value (finite-domain
    // suppression — documented, not a failure).
    assert!(cl6 < s6.inviscid_lift(), "viscous Cl {cl6} below inviscid {}", s6.inviscid_lift());
}
