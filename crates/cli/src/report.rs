//! Console reporting helpers for the CLI. Pure formatting — no solving here.

use helisim_bemt::HoverSolution;
use helisim_validation::PointResult;

/// Print the validation table for a single case against its C_T oracle.
pub fn print_validation(name: &str, description: &str, results: &[PointResult]) {
    println!("\n=== {name} ===");
    println!("{description}");
    if results.is_empty() {
        println!("(no C_T-vs-collective oracle points for this case)");
        return;
    }
    println!(
        "{:>7} {:>7} {:>12} {:>12} {:>9} {:>7} {:>6}",
        "theta", "M_tip", "C_T (exp)", "C_T (BEMT)", "err %", "FM", "pass"
    );
    for r in results {
        println!(
            "{:>6.1}° {:>7.3} {:>12.5} {:>12.5} {:>8.1}% {:>7.3} {:>6}",
            r.point.collective_deg,
            r.point.tip_mach,
            r.point.ct_expected,
            r.ct_pred,
            r.rel_err * 100.0,
            r.fm_pred,
            if r.pass { "OK" } else { "FAIL" },
        );
    }
}

/// Print a one-line summary of a hover solution.
pub fn print_solution_summary(label: &str, sol: &HoverSolution) {
    println!(
        "{label}: C_T={:.5}  C_P={:.6}  FM={:.3}  T={:.1} N  P={:.0} W",
        sol.ct, sol.cp, sol.figure_of_merit, sol.thrust, sol.power
    );
}

/// Print the spanwise loading and inflow distribution (validation target #2).
pub fn print_spanwise(sol: &HoverSolution, every: usize) {
    println!(
        "\n{:>6} {:>9} {:>9} {:>9} {:>8} {:>8} {:>10}",
        "r/R", "lambda", "alpha°", "Cl", "Cd", "F", "dCT/dx"
    );
    for (i, s) in sol.stations.iter().enumerate() {
        if i % every != 0 && i != sol.stations.len() - 1 {
            continue;
        }
        println!(
            "{:>6.3} {:>9.5} {:>9.3} {:>9.4} {:>8.4} {:>8.4} {:>10.5}",
            s.x,
            s.lambda,
            s.alpha.to_degrees(),
            s.cl,
            s.cd,
            s.tip_loss,
            s.dct_dx,
        );
    }
}
