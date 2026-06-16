//! Reynolds-number response of the viscous airfoil (refinement: higher-Re polars).
//! The single hardest thing for a laminar solver to get right is the *trend*: profile
//! drag must fall as `Re` rises (a thinning boundary layer), roughly like a laminar
//! `~Re^{-1/2}`. There is no single published number to hit here (a specific rounded
//! section at low Re), so the oracle is the monotone trend and its rough rate.

use helisim_cfd::{AirfoilConfig, solve_airfoil_viscous};

fn cd_at(re: f64) -> f64 {
    let cfg = AirfoilConfig {
        n_r: 80,
        n_t: 128,
        re_chord: re,
        omega_relax: 0.25,
        te_round: 0.1,
        psi_sweeps: 10,
        r_max: 30.0,
        max_steps: 12_000,
        ..AirfoilConfig::new(0.0, re)
    };
    let s = solve_airfoil_viscous(&cfg);
    // The drag is read from a steady field. The α=0 lift stays near zero (the blunt
    // rounded TE begins a faint wake asymmetry at higher Re, so it is small rather than
    // exactly zero) and the drag is bounded — the guard against a runaway.
    let (cl, cd) = s.force_coefficients();
    assert!(
        cl.abs() < 0.05,
        "α=0 lift near zero (Cl={cl}, not diverged)"
    );
    assert!((0.0..0.5).contains(&cd), "Cd {cd} bounded and positive");
    cd
}

#[test]
fn profile_drag_falls_with_reynolds_number() {
    let cd_low = cd_at(200.0);
    let cd_high = cd_at(500.0);
    // Drag drops substantially with Re — the model blade's true regime is far less
    // draggy than the very-low-Re illustration suggests.
    assert!(
        cd_high < 0.8 * cd_low,
        "Cd should fall with Re: {cd_low} → {cd_high}"
    );
    // ...and roughly like a laminar boundary layer (~Re^{-1/2}); 2.5× Re ⇒ ~0.6× Cd,
    // a generous band around it (the rounded TE adds a pressure-drag floor).
    let ratio = cd_high / cd_low;
    assert!(
        (0.30..0.75).contains(&ratio),
        "Cd ratio {ratio} in the laminar band"
    );
}
