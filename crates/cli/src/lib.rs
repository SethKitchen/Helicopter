//! helisim CLI as a library — every subcommand is a `pub mod` and [`dispatch`]
//! routes a mode string to it. The binary ([`main`](../main.rs)) is a thin shell
//! over this, which also lets the commands be smoke-tested (see `tests/`).

pub mod attitude_cli;
pub mod battery_build_cli;
pub mod bms_cli;
pub mod build_cli;
pub mod charge_build_cli;
pub mod charging_cli;
pub mod design_cli;
pub mod dynamics_cli;
pub mod flapping_cli;
pub mod fly_cli;
pub mod forward_cli;
pub mod hover_cli;
pub mod inflow_cli;
pub mod lateral_cli;
pub mod mission_cli;
pub mod report;
pub mod sas_cli;
pub mod sim_cli;
pub mod study;
pub mod trim_cli;

use helisim_bemt::{Config, solve_hover};
use helisim_rotor::Operating;
use helisim_validation::{CaradonnaTung, HarringtonRotor1, ValidationCase, run_case};

/// Route a mode string to its subcommand (the binary's `main` calls this).
pub fn dispatch(mode: &str) {
    let cfg = Config::default();
    match mode {
        "study" => study::run(),
        "forward" => forward_cli::run(),
        "flapping" => flapping_cli::run(),
        "trim" => trim_cli::run(),
        "dynamics" => dynamics_cli::run(),
        "lateral" => lateral_cli::run(),
        "sim" => sim_cli::run(),
        "coupled" => sim_cli::run_coupled(),
        "inflow" => inflow_cli::run(),
        "fly" => fly_cli::run(),
        "sas" => sas_cli::run(),
        "attitude" => attitude_cli::run(),
        "hover" => hover_cli::run(),
        "mission" => mission_cli::run(),
        "bms" | "battery" => bms_cli::run(),
        "battery-build" => battery_build_cli::run(),
        "charging" => charging_cli::run(),
        "charge-build" => charge_build_cli::run(),
        "design" => design_cli::run(),
        "build" => build_cli::run(),
        "harrington" => harrington_sweep(&cfg),
        "spanwise" => {
            validation_report(&cfg);
            spanwise_dump(&cfg);
        }
        _ => validation_report(&cfg),
    }
}

/// Run every validation case and print the comparison tables.
pub fn validation_report(cfg: &Config) {
    println!("helisim — hover BEMT validation (zero dependencies, std only)");

    let ct = CaradonnaTung::default();
    let results = run_case(&ct, cfg);
    report::print_validation(ct.name(), ct.description(), &results);
    let pass = results.iter().filter(|r| r.pass).count();
    println!(
        "  -> {pass}/{} oracle points within tolerance",
        results.len()
    );
    if let Some(notes) = ct.notes() {
        println!("  note: {notes}");
    }

    let harr = HarringtonRotor1::default();
    report::print_validation(harr.name(), harr.description(), &run_case(&harr, cfg));
    if let Some(notes) = harr.notes() {
        println!("  note: {notes}");
    }
    harrington_sweep(cfg);
}

/// Dump the Caradonna & Tung θ=8° spanwise loading/inflow distribution.
pub fn spanwise_dump(cfg: &Config) {
    let ct = CaradonnaTung::default();
    let rotor = ct.build_rotor(8f64.to_radians());
    let op = Operating::from_tip_mach(0.439, rotor.radius);
    let sol = solve_hover(&rotor, &op, ct.airfoil().as_ref(), cfg);
    println!("\n=== Caradonna & Tung θ=8°, M_tip=0.439 — spanwise distribution ===");
    report::print_solution_summary("integrated", &sol);
    report::print_spanwise(&sol, 20);
}

/// Sweep collective pitch on the Harrington rotor and report the peak figure of
/// merit against the published band.
pub fn harrington_sweep(cfg: &Config) {
    let harr = HarringtonRotor1::default();
    let airfoil = harr.airfoil();
    println!(
        "\n=== Harrington Rotor 1 — figure-of-merit sweep (M_tip={:.2}) ===",
        harr.tip_mach
    );
    println!(
        "{:>7} {:>10} {:>10} {:>7}",
        "theta", "C_T", "C_T/sigma", "FM"
    );

    let mut peak_fm = 0.0_f64;
    let mut peak_theta = 0.0_f64;
    for step in 0..=16 {
        let theta_deg = 2.0 + step as f64 * 0.75; // 2° … 14°
        let rotor = harr.build_rotor(theta_deg.to_radians());
        let op = Operating::from_tip_mach(harr.tip_mach, rotor.radius);
        let sol = solve_hover(&rotor, &op, airfoil.as_ref(), cfg);
        let ct_sigma = sol.ct_over_sigma(rotor.solidity());
        println!(
            "{:>6.2}° {:>10.5} {:>10.4} {:>7.3}",
            theta_deg, sol.ct, ct_sigma, sol.figure_of_merit
        );
        if sol.figure_of_merit > peak_fm {
            peak_fm = sol.figure_of_merit;
            peak_theta = theta_deg;
        }
    }

    let (lo, hi) = harr.expected_peak_fm();
    let verdict = if (lo..=hi).contains(&peak_fm) {
        "OK"
    } else {
        "outside band"
    };
    println!(
        "  peak FM = {peak_fm:.3} at θ={peak_theta:.2}° (expected [{lo:.2}, {hi:.2}]) -> {verdict}"
    );
}
