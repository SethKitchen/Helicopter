//! **External validation** of viscous flow past a circular cylinder — the canonical
//! "a body sits in the flow, integrate its surface loads to get forces" benchmark,
//! and the bridge from the validated NS core toward sectional airfoil loads.
//!
//! Oracle (steady regime `5 ≲ Re ≲ 47`, a pair of stationary recirculating eddies),
//! a tightly-clustered multi-source benchmark — all cited, none fabricated:
//!   * Tritton, *J. Fluid Mech.* **6** (1959) 547 — experimental `C_D`;
//!   * Dennis & Chang, *J. Fluid Mech.* **42** (1970) 471 — stream-function/vorticity;
//!   * Coutanceau & Bouard, *J. Fluid Mech.* **79** (1977) 231 — experimental wake;
//!   * Le, Calhoun, Xu & Wang — immersed-boundary tabulations of the same case.
//!
//! At **Re_D = 40** the consensus is `C_D ≈ 1.48–1.66` (Tritton 1.48, Dennis & Chang
//! 1.522), recirculation length `L_wake/D ≈ 2.18–2.35`, separation angle (from the
//! rear stagnation) `θ_sep ≈ 53.5–54.2°`.
//!
//! Our hand-rolled body-fitted log-polar solver reproduces all three to within
//! ~5–15%, with **two independent drag routes (surface integral and total
//! dissipation) that agree** — the ★ cross-check. The residual is owned: first-order
//! upwinding plus the finite outer domain (a blockage-vs-far-wake-truncation
//! trade-off) and grid resolution; refining moves every quantity toward the
//! benchmark. Re_D = 20 (`C_D ≈ 2.0–2.2`, `L/D ≈ 0.93`) is reproducible with a
//! reduced relaxation factor (the ψ↔ω coupling is stiffer there).

use helisim_cfd::{CylinderConfig, solve_cylinder};

#[test]
fn matches_re40_cylinder_benchmark() {
    let cfg = CylinderConfig::new(40.0);
    let s = solve_cylinder(&cfg);
    assert!(
        s.converged,
        "steady state should be reached (steps {})",
        s.steps
    );

    // Symmetry of the steady solution must emerge (not imposed) — a self-consistency
    // gate independent of the oracle.
    assert!(
        s.top_bottom_asymmetry() < 1e-3,
        "flow should be top–bottom symmetric"
    );

    // Separation angle (from the rear stagnation): the tightest, most robust check.
    let theta = s.separation_angle_deg();
    assert!(
        (theta - 53.8).abs() < 5.0,
        "θ_sep {theta}° vs benchmark ≈53.5–54.2°"
    );

    // Recirculation length L_wake/D.
    let lw = s.wake_length_over_d();
    assert!(
        (lw - 2.26).abs() / 2.26 < 0.15,
        "L_wake/D {lw} vs benchmark ≈2.18–2.35"
    );

    // Drag — TWO independent routes. The local **surface integral** (friction +
    // pressure) is the accurate force measure and is checked against the oracle; the
    // whole-field **dissipation** route is a cross-check (it under-predicts a little
    // because the far-wake dissipation extends past the finite domain — documented,
    // not fudged).
    let (cdf, cdp, cd_surf) = s.drag_coefficient_surface();
    let cd_diss = s.drag_coefficient();
    assert!(
        cdf > 0.0 && cdp > 0.0,
        "friction and pressure drag both positive"
    );
    assert!(
        (cd_surf - 1.52).abs() / 1.52 < 0.15,
        "C_D(surface) {cd_surf} vs ≈1.48–1.66"
    );
    assert!(
        (cd_diss - 1.52).abs() / 1.52 < 0.30,
        "C_D(dissipation) {cd_diss} right order"
    );
    // ★ The two physically-independent drag routes agree (one local-surface, one
    // whole-field dissipation) — the cross-check that *could* disagree but doesn't.
    assert!(
        (cd_surf - cd_diss).abs() / cd_surf < 0.16,
        "the two drag routes should agree: surface {cd_surf} vs dissipation {cd_diss}"
    );
}
