//! Wiring the CFD airfoil into the rotor: the `CfdAirfoil` plugs into the BEMT solver
//! through the `Airfoil` trait, and carries a *finding* — at model-scale low Reynolds
//! number the viscous drag is far higher than the analytic high-Re NACA0012 model
//! assumes, so the rotor's figure of merit drops.

use helisim_airfoil::{Airfoil, LinearAirfoil};
use helisim_bemt::{Config, solve_hover};
use helisim_cfd_airfoil::CfdAirfoil;
use helisim_rotor::{Operating, Rotor};

#[test]
fn cfd_polar_interpolates_through_the_airfoil_trait() {
    // A hand polar (the fast constructor) used through the &dyn Airfoil interface.
    let af = CfdAirfoil::from_polar_deg(&[(-8.0, -0.9, 0.13), (0.0, 0.0, 0.10), (8.0, 0.9, 0.13)], 200.0);
    let (cl, cd) = af.cl_cd(4.0_f64.to_radians(), 0.0);
    assert!((cl - 0.45).abs() < 1e-6 && (cd - 0.115).abs() < 1e-6, "midpoint interp");
    // Symmetric section: lift flips sign with angle, drag does not.
    assert!(af.cl(-4.0_f64.to_radians(), 0.0) < 0.0);
    assert!(af.cd(-4.0_f64.to_radians(), 0.0) > 0.0);
}

#[test]
fn viterna_completes_the_polar_to_deep_stall() {
    // An attached polar (±14°, lift slope ~4/rad, low-Re drag), completed to ±90°.
    let attached =
        CfdAirfoil::from_polar_deg(&[(-14.0, -1.0, 0.06), (0.0, 0.0, 0.04), (14.0, 1.0, 0.06)], 500.0);
    let full = attached.with_viterna_stall(14.0, 2.0);

    // The attached region is preserved.
    assert!((full.cd(0.0, 0.0) - 0.04).abs() < 1e-6, "attached drag kept at α=0");
    // Deep stall reaches the flat-plate limits: drag climbs toward Cd_max, lift → 0.
    assert!(full.cd(45f64.to_radians(), 0.0) > full.cd(14f64.to_radians(), 0.0), "drag rises past stall");
    assert!(full.cd(85f64.to_radians(), 0.0) > 1.5, "Cd near 90° approaches flat-plate ~2.0");
    assert!(full.cl(89f64.to_radians(), 0.0).abs() < 0.15, "lift → 0 at 90°");
    // Still a symmetric section across the whole range.
    assert!(full.cl(-45f64.to_radians(), 0.0) < 0.0 && full.cd(-45f64.to_radians(), 0.0) > 0.0);
}

#[test]
fn cfd_low_re_drag_penalises_the_rotor_vs_analytic_high_re() {
    // Generate the polar from the viscous NS solve (a coarse, few-angle sweep).
    let cfd = CfdAirfoil::from_cfd_sweep(200.0, &[0.0, 4.0, 8.0], 64, 100);
    let linear = LinearAirfoil::naca0012();

    // The finding, at the airfoil level: the low-Re viscous drag is many times the
    // analytic high-Re value (which is ~0.01). Cl is the same order (both lifting).
    let a = 4.0_f64.to_radians();
    let (cl_cfd, cd_cfd) = cfd.cl_cd(a, 0.0);
    let (_, cd_lin) = linear.cl_cd(a, 0.0);
    assert!(cd_cfd > 5.0 * cd_lin, "low-Re Cd {cd_cfd} ≫ high-Re Cd {cd_lin}");
    assert!(cl_cfd > 0.0, "CFD airfoil still lifts");

    // The finding, at the rotor level: the same rotor flown on the CFD low-Re polar
    // has a much lower figure of merit than on the analytic high-Re airfoil — both
    // physical, the difference is the model-scale profile-drag penalty.
    let rotor = Rotor::rectangular(2, 1.143, 0.191, 8f64.to_radians(), 0.2);
    let op = Operating::from_tip_mach(0.439, 1.143);
    let cfg = Config::default();
    let h_cfd = solve_hover(&rotor, &op, &cfd, &cfg);
    let h_lin = solve_hover(&rotor, &op, &linear, &cfg);

    assert!(h_cfd.ct > 0.0 && h_cfd.cp > 0.0, "CFD-airfoil hover is physical");
    assert!(h_cfd.figure_of_merit > 0.0 && h_cfd.figure_of_merit < 1.0, "FM in range");
    assert!(h_cfd.cp > h_lin.cp, "CFD low-Re drag raises profile power");
    assert!(
        h_cfd.figure_of_merit < h_lin.figure_of_merit,
        "low-Re penalty: FM {} < {}",
        h_cfd.figure_of_merit,
        h_lin.figure_of_merit
    );
}
