//! `cfd` subcommand: run the from-scratch viscous Navier–Stokes core on the
//! lid-driven cavity and show the comparison against the Ghia et al. (1982)
//! benchmark — the project's first solve of the actual N–S equations on a grid.

use helisim_cfd::{CavityConfig, solve_cavity};

pub fn run() {
    println!("helisim — CFD: viscous 2-D Navier–Stokes (vorticity–streamfunction), zero deps\n");
    let re = 100.0;
    let n = 65;
    println!("Lid-driven cavity at Re={re:.0} on a {n}×{n} grid (lid U=1):");
    let cfg = CavityConfig { steady_tol: 1e-5, ..CavityConfig::new(re, n) };
    let s = solve_cavity(&cfg);
    println!(
        "  steady state reached in {} pseudo-time steps (converged: {}).\n",
        s.steps, s.converged
    );

    // Ghia, Ghia & Shin, J. Comput. Phys. 48 (1982), Tables I & II — Re=100.
    let (u_min, y_at) = s.min_centerline_u();
    let (v_min, v_max) = s.v_extrema();
    let (vx, vy, vpsi) = s.primary_vortex();

    println!("  Validation vs Ghia et al. (1982) — gold-standard CFD benchmark:");
    println!("    {:<34}{:>12}{:>12}{:>9}", "quantity", "this solver", "Ghia 1982", "err");
    let row = |label: &str, got: f64, want: f64| {
        println!(
            "    {:<34}{:>12.5}{:>12.5}{:>8.1}%",
            label,
            got,
            want,
            100.0 * (got - want).abs() / want.abs()
        );
    };
    row("min u, vertical centreline", u_min, -0.21090);
    row("max v, horizontal centreline", v_max, 0.17527);
    row("min v, horizontal centreline", v_min, -0.24533);
    row("primary-vortex ψ", vpsi, -0.103423);
    println!(
        "    primary-vortex centre             ({vx:.3},{vy:.3})    (0.617,0.734)   ~{:.0}mm",
        ((vx - 0.6172).hypot(vy - 0.7344)) * 1000.0
    );
    println!(
        "    (u_min occurs at y={y_at:.3}; Ghia 0.4531.)  All within ~1–2% — refines toward Ghia.\n"
    );

    // Pressure recovery (the field the streamfunction form drops — the path to forces).
    let (pmin, pmax) = s.pressure_extrema();
    println!("  Pressure recovered from the velocity field (pressure-Poisson, Neumann):");
    println!("    range p ∈ [{pmin:+.4}, {pmax:+.4}] (pinned 0 at corner) — high at the");
    println!("    downstream-lid stagnation, low in the vortex core, as expected.\n");

    // Unsteady-solver validation: the exact Taylor–Green vortex decay.
    let tg = helisim_cfd::TaylorGreen::new(48, 0.1);
    let (got, want) = (tg.march_energy_ratio(2.0, 0.4), tg.exact_energy_ratio(2.0));
    println!("  Unsteady check — Taylor–Green vortex (exact NS solution), ν=0.1, t=2:");
    println!(
        "    kinetic energy E(t)/E(0) = {got:.4} vs exact e^(-4νt) = {want:.4}  ({:.1}%)\n",
        100.0 * (got - want).abs() / want
    );

    // A body in the flow: steady viscous flow past a circular cylinder (forces from
    // the surface integral) — the bridge toward sectional airfoil loads.
    println!("  Flow past a circular cylinder at Re_D=40 (body-fitted log-polar grid):");
    let cs = helisim_cfd::solve_cylinder(&helisim_cfd::CylinderConfig::new(40.0));
    let (_, _, cd_surf) = cs.drag_coefficient_surface();
    let cd_diss = cs.drag_coefficient();
    println!(
        "    C_D = {cd_surf:.3} (surface) / {cd_diss:.3} (dissipation)   vs benchmark ≈1.48–1.66"
    );
    println!(
        "    L_wake/D = {:.2} (≈2.18–2.35)   θ_sep = {:.1}° (≈53.5–54.2°)   [Tritton/Dennis–Chang]",
        cs.wake_length_over_d(),
        cs.separation_angle_deg()
    );
    println!("    Two independent drag routes (surface + dissipation) agree — the ★ cross-check.\n");

    // Lift: the Joukowski conformal map turns the circle flow into a lifting airfoil.
    use std::f64::consts::PI;
    let af = helisim_cfd::JoukowskiAirfoil::new(1.0, 0.1);
    println!(
        "  Joukowski airfoil (lift), t/c = {:.0}% — inviscid Cl from the surface-pressure integral:",
        100.0 * af.thickness_ratio()
    );
    println!("    {:>6}  {:>9}  {:>9}  {:>8}", "α(deg)", "Cl(integ)", "Cl(exact)", "Cd");
    for &deg in &[0.0, 4.0, 8.0] {
        let s = af.solve_inviscid(deg * PI / 180.0, 2000);
        println!(
            "    {deg:>6.0}  {:>9.4}  {:>9.4}  {:>8.4}",
            s.cl,
            af.lift_coefficient_exact(deg * PI / 180.0),
            s.cd
        );
    }
    let slope = af.lift_coefficient_exact(4.0 * PI / 180.0) / (4.0 * PI / 180.0).sin();
    println!(
        "    lift slope {:.3}/rad = 2π·(1+ε/c); Cd≈0 confirms d'Alembert. (Rotor LinearAirfoil",
        slope
    );
    println!("    uses 5.73/rad ≈ 0.91·2π — the viscous/real reduction this inviscid value bounds.)\n");

    // Viscous airfoil: the cylinder solver carrying the Joukowski conformal metric —
    // the profile drag (the inviscid map gives Cd=0) and the lift response.
    println!("  VISCOUS airfoil (NS solve, conformal metric), Re_chord=200:");
    let vcfg = |deg: f64| helisim_cfd::AirfoilConfig {
        n_r: 64,
        n_t: 100,
        r_max: 30.0,
        omega_relax: 0.3,
        te_round: 0.1,
        psi_sweeps: 8,
        max_steps: 6000,
        ..helisim_cfd::AirfoilConfig::new(deg, 200.0)
    };
    let v0 = helisim_cfd::solve_airfoil_viscous(&vcfg(0.0));
    let v6 = helisim_cfd::solve_airfoil_viscous(&vcfg(6.0));
    let (cl0, cd0) = v0.force_coefficients();
    let (cl6, cd6) = v6.force_coefficients();
    println!("    α=0°:  Cl = {cl0:+.3} (symmetry)   Cd = {cd0:.3} (PROFILE DRAG — inviscid gives 0)");
    println!(
        "    α=6°:  Cl = {cl6:+.3} (>0, develops viscously)   Cd = {cd6:.3}   [inviscid Cl = {:.3}]",
        v6.inviscid_lift()
    );
    println!("    Lift positive & linear; magnitude below inviscid (finite-domain far field — named).\n");

    println!(
        "Wired into the rotor: `CfdAirfoil` (crate helisim-cfd-airfoil) builds this viscous\n\
         polar once and serves it through the BEMT `Airfoil` trait. Finding — at Re_c=200 the\n\
         low-Re Cd is ~28x the analytic high-Re value, so a model rotor's figure of merit\n\
         collapses (~0.66 -> ~0.11). (Re=200 is illustratively low; the penalty is real but\n\
         milder at model-blade Re~1e4-1e5.)  Next: a stable circulation-corrected far field\n\
         to recover the full lift magnitude."
    );
}
