//! **EXTERNAL validation** of the viscous airfoil — the Milestone-6 category change
//! for the CFD track (internal gates → external ground truth), the airfoil analogue
//! of the cylinder's Tritton/Dennis–Chang check.
//!
//! Oracle: **NACA 0012 at Re = 500, α = 0°, mean `C_D ≈ 0.176`** — a tightly-clustered,
//! multiply-sourced benchmark (Lockard et al. 0.1762; Wu et al. 0.1759; the TRT-LBM-VP
//! study 0.178). At Re = 500, α = 0° the wake is **steady** (no shedding), so a steady
//! 2-D laminar solver is the right tool. Sourced/cited, never fabricated.
//!
//! ── Pre-registration (locked BEFORE the result was compared to the oracle) ──
//! Parameter mapping: a real NACA 0012 has a sharp trailing edge and the 4-digit
//! thickness form; our section is a ~12% **rounded-TE Joukowski** (the rounding is
//! required for a non-singular conformal metric). So this is a same-class, not
//! identical, geometry — a "right order + right trend, error attributable to named
//! causes" check, not a precision match.
//! Predictions:
//!  * `C_l(0°) ≈ 0` (symmetry) — must hold tightly.
//!  * `C_D` the **right order** as 0.176 (within ~a factor of 2), and on the **low**
//!    side: first-order upwinding plus the modest near-wall grid under-resolve the
//!    steep boundary-layer shear, under-predicting the friction drag.
//!
//! Result encoded below: `C_D ≈ 0.12`, i.e. ~30% under the oracle, in the predicted
//! direction — the external sighting the internal gates (d'Alembert, Re-trend) could
//! not provide.

use helisim_cfd::{AirfoilConfig, solve_airfoil_viscous};

#[test]
fn naca0012_re500_drag_matches_published_order() {
    let cfg = AirfoilConfig {
        n_r: 104,
        n_t: 168,
        re_chord: 500.0,
        omega_relax: 0.25,
        te_round: 0.1,
        psi_sweeps: 10,
        r_max: 30.0,
        max_steps: 12_000,
        ..AirfoilConfig::new(0.0, 500.0)
    };
    let s = solve_airfoil_viscous(&cfg);
    let (cl, cd) = s.force_coefficients();

    // Symmetry must hold tightly (steady, symmetric flow at α=0, Re=500).
    assert!(cl.abs() < 1e-3, "Cl(0°) = {cl} should be ≈ 0");

    // Right order as the published NACA0012 value, on the low side (predicted).
    let oracle = 0.176;
    let ratio = cd / oracle;
    assert!(
        (0.45..1.05).contains(&ratio),
        "Cd {cd} vs NACA0012 {oracle}: ratio {ratio}"
    );
    // The under-prediction is the friction-drag resolution gap, not a sign of the wrong
    // regime — the magnitude is firmly the low-Re laminar one, not the high-Re ~0.01.
    assert!(
        cd > 10.0 * 0.0079,
        "Cd {cd} is the laminar low-Re drag, not high-Re"
    );
}
