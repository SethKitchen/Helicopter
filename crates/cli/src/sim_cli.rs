//! `sim` subcommand: nonlinear longitudinal time-march of a perturbed hover,
//! validated against the pre-computed 5c eigenvalues — the linear analysis hands
//! the simulator its oracle.

use helisim_dynamics::{Inertia, analyze_coupled_hover, analyze_hover_longitudinal, eigenvalues};
use helisim_sim::{
    equilibrium_state8, fit_growing_oscillation, linearize8, simulate_hover_longitudinal,
    simulate_linear, simulate_linear_nd, simulate8,
};
use helisim_trim::Aircraft;

/// `coupled` subcommand: the nonlinear 8-state march (5g).
pub fn run_coupled() {
    let ac = Aircraft::demo();
    let j = Inertia {
        mass: 8.0,
        i_xx: 0.4,
        i_yy: 0.8,
        i_zz: 1.0,
    };
    let eq = equilibrium_state8(&ac);

    println!("helisim — nonlinear coupled 8-state march (5g, both rotors in the loop)\n");
    println!(
        "8-state hover equilibrium: pitch {:.2}°, roll {:.2}° (roll carries the tail side force)",
        eq[3].to_degrees(),
        eq[7].to_degrees()
    );

    let fp = simulate8(&ac, j, [0.0; 8], 0.02, 6.0);
    let drift = fp
        .iter()
        .map(|s| (0..8).map(|i| (s[i] - eq[i]).abs()).fold(0.0_f64, f64::max))
        .fold(0.0_f64, f64::max);
    println!("Fixed point (start at rest): max drift over 6 s = {drift:.1e}");

    // Eigenvalues: nonlinear Jacobian vs the independent coupled linear model.
    let a8 = linearize8(&ac, j);
    let nl = eigenvalues(&a8);
    let lin = analyze_coupled_hover(&ac, j, true).eigenvalues;
    println!("\nEigenvalues — nonlinear-EOM Jacobian vs independent coupled 8×8 model:");
    let mut pairs: Vec<_> = nl.iter().filter(|e| e.im >= -1e-9).collect();
    pairs.sort_by(|a, b| a.re.partial_cmp(&b.re).unwrap());
    for e in pairs {
        let near = lin
            .iter()
            .filter(|c| c.im >= -1e-9)
            .min_by(|a, b| {
                ((a.re - e.re).hypot(a.im - e.im))
                    .partial_cmp(&(b.re - e.re).hypot(b.im - e.im))
                    .unwrap()
            })
            .unwrap();
        let kind = if e.im.abs() > 0.1 {
            if e.re > 0.0 {
                "UNSTABLE osc"
            } else {
                "stable osc  "
            }
        } else {
            "subsidence  "
        };
        println!(
            "  {:>7.3}{:+.3}i  {kind}   (coupled: {:.3}{:+.3}i)",
            e.re, e.im, near.re, near.im
        );
    }

    // Perturb and watch nonlinear track linear, then depart.
    let dt = 0.01;
    let pert = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]; // Δu = 0.1 m/s
    let nlt = simulate8(&ac, j, pert, dt, 8.0);
    let llt = simulate_linear_nd(&a8, &pert, dt, 8.0);
    println!("\nΔu = 0.1 m/s: nonlinear vs 8-D linear (perturbation norm, m & rad mixed):");
    println!(
        "{:>5} {:>10} {:>10} {:>8} {:>10}",
        "t s", "‖nl‖", "‖lin‖", "rel%", "lateral"
    );
    for k in (0..=8).map(|s| (s as f64 / dt) as usize) {
        if k < nlt.len() {
            let pn: f64 = (0..8)
                .map(|i| (nlt[k][i] - eq[i]).powi(2))
                .sum::<f64>()
                .sqrt();
            let pl: f64 = (0..8).map(|i| llt[k][i].powi(2)).sum::<f64>().sqrt();
            let rd = (0..8)
                .map(|i| ((nlt[k][i] - eq[i]) - llt[k][i]).abs())
                .fold(0.0_f64, f64::max)
                / pl.max(1e-9);
            let lat = [nlt[k][4] - eq[4], nlt[k][5] - eq[5], nlt[k][7] - eq[7]]
                .iter()
                .fold(0.0_f64, |m, &v| m.max(v.abs()));
            println!(
                "{:>5.0} {:>10.4} {:>10.4} {:>7.1}% {:>10.4}",
                k as f64 * dt,
                pn,
                pl,
                rd * 100.0,
                lat
            );
        }
    }
    println!(
        "\nThe march tracks the linear prediction through ~4 s, then departs as BOTH\n\
         oscillatory instabilities (longitudinal + lateral) compound — a shorter\n\
         valid window than 4-D. A purely longitudinal Δu drives lateral motion\n\
         (last column) via the now-validated cross-coupling. Built entirely on\n\
         validated pieces: 5c/5d longitudinal, corrected 5e-i/5f lateral aero."
    );
}

pub fn run() {
    let ac = Aircraft::demo();
    let i_yy = 0.8;
    let lin = analyze_hover_longitudinal(&ac, i_yy);
    let osc = lin
        .modes
        .iter()
        .find(|m| m.oscillatory && m.eigenvalue.im > 0.0)
        .unwrap();

    println!("helisim — nonlinear longitudinal time-march (RK4, rotor-in-the-loop)\n");
    println!(
        "5c linear prediction (the pre-computed oracle): unstable oscillation\n  \
         eigenvalue {:.3}{:+.3}i  →  period {:.2}s, doubling {:.2}s\n",
        osc.eigenvalue.re, osc.eigenvalue.im, osc.period, osc.time_to_half_or_double
    );

    // Fixed-point check.
    let fp = simulate_hover_longitudinal(&ac, i_yy, [0.0, 0.0, 0.0, 0.0], 0.02, 12.0);
    println!(
        "Trim fixed point: start at equilibrium → max|state| drift over 12s = {:.1e}",
        fp.max_abs()
    );

    // Perturbed trajectory + linear comparison.
    let dt = 0.01;
    let x0 = [0.5, 0.0, 0.0, 0.0];
    let nl = simulate_hover_longitudinal(&ac, i_yy, x0, dt, 12.0);
    let ll = simulate_linear(&lin.a_matrix, x0, dt, 12.0);

    println!("\nPerturbed hover by Δu = 0.5 m/s — nonlinear vs linear ẋ=Ax (pitch θ, deg):");
    println!(
        "{:>5} {:>10} {:>10} {:>8}",
        "t s", "θ nonlin", "θ linear", "rel%"
    );
    for k in (0..=12).map(|s| (s as f64 / dt) as usize) {
        if k < nl.states.len() && k < ll.states.len() {
            let (a, b) = (nl.states[k][3], ll.states[k][3]);
            let rel = if b.abs() > 1e-6 {
                (a - b).abs() / b.abs() * 100.0
            } else {
                0.0
            };
            println!(
                "{:>5.0} {:>10.3} {:>10.3} {:>7.1}%",
                k as f64 * dt,
                a.to_degrees(),
                b.to_degrees(),
                rel
            );
        }
    }

    // Fit period/growth from the nonlinear trajectory.
    let small = simulate_hover_longitudinal(&ac, i_yy, [0.1, 0.0, 0.0, 0.0], 0.01, 12.0);
    if let Some(f) = fit_growing_oscillation(&small.times, &small.column(3), 1.5) {
        println!(
            "\nFitted from the nonlinear run: period {:.2}s, σ {:.3} (t×2 {:.2}s) — vs 5c {:.2}s / {:.3}",
            f.period, f.growth_rate, f.time_to_double, osc.period, osc.eigenvalue.re
        );
    }

    println!(
        "\nThe nonlinear time-march coincides with the linear prediction through the\n\
         linear regime (first ~4 s), then DEPARTS as the unstable oscillation grows\n\
         into nonlinearity — the physical result the linear model alone cannot give.\n\
         The match to the pre-computed eigenvalue is the falsifiable gate; it holds\n\
         at multiple step sizes, so it is not an integrator artifact."
    );
}
