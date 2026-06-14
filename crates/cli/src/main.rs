//! helisim — command-line driver.
//!
//! Milestone 1: solve the Caradonna & Tung (1981) hover rotor with the BEMT core
//! and validate the integrated thrust coefficient against published data, then
//! run the Harrington (1951) Rotor 1 figure-of-merit sanity check.
//!
//! Usage:
//!   helisim                 run the full validation report (default)
//!   helisim spanwise        also print the C&T θ=8° spanwise distribution
//!   helisim harrington      run the Harrington FM sweep
//!   helisim study           C_T sensitivity diagnostic
//!   helisim forward         forward-flight sweep: power bucket + rolling moment
//!   helisim flapping        blade flapping: moment→TPP-tilt + 90° phase lag
//!   helisim trim            steady-flight trim (Newton) + hover cross-check
//!   helisim dynamics        hover stability derivatives + modes (instability)
//!   helisim sim             nonlinear time-march vs the linear eigenvalue gate
//!   helisim lateral         lateral-directional oracle + coupled 8-state gate
//!   helisim coupled         nonlinear 8-state march vs the coupled linear gate
//!   helisim inflow          Pitt-Peters dynamic inflow: τ→0 gate + off-axis sign flip
//!   helisim fly             control-input time histories: effectiveness + open-loop divergence
//!   helisim sas             stability augmentation: off-seam design, hover damping, nonlinear hold
//!   helisim attitude        attitude hold: phugoid→LHP, off-seam regulation, hover seam-residual
//!   helisim hover           velocity/position hold: timescale separation + hands-off hover capstone
//!   helisim mission         end-to-end electric hover: power → C-rate → endurance

mod attitude_cli;
mod dynamics_cli;
mod flapping_cli;
mod fly_cli;
mod forward_cli;
mod hover_cli;
mod inflow_cli;
mod lateral_cli;
mod mission_cli;
mod report;
mod sas_cli;
mod sim_cli;
mod study;
mod trim_cli;

use helisim_bemt::{Config, solve_hover};
use helisim_rotor::Operating;
use helisim_validation::{CaradonnaTung, HarringtonRotor1, ValidationCase, run_case};

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_default();
    let cfg = Config::default();

    match mode.as_str() {
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
        "harrington" => harrington_sweep(&cfg),
        "spanwise" => {
            validation_report(&cfg);
            spanwise_dump(&cfg);
        }
        _ => validation_report(&cfg),
    }
}

/// Run every validation case and print the comparison tables.
fn validation_report(cfg: &Config) {
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
fn spanwise_dump(cfg: &Config) {
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
fn harrington_sweep(cfg: &Config) {
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
