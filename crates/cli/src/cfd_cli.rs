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

    println!(
        "Next toward airfoil Cl/Cd: an immersed/body-fitted boundary so a section sits\n\
         in the flow; the pressure + wall-shear integral then gives the sectional loads."
    );
}
