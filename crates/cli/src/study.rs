//! Sensitivity study: isolate how each modeling choice moves the predicted
//! Caradonna & Tung thrust coefficient relative to the published oracle. This is
//! a diagnostic tool, not part of the validated path.

use helisim_airfoil::LinearAirfoil;
use helisim_bemt::{Config, solve_hover};
use helisim_rotor::{Operating, Rotor};

const ORACLE: [(f64, f64); 3] = [(5.0, 0.00213), (8.0, 0.00460), (12.0, 0.00796)];

fn ct_at(theta_deg: f64, a0: f64, compressible: bool, tip_loss: bool) -> f64 {
    let rotor = Rotor::rectangular(2, 1.143, 0.191, theta_deg.to_radians(), 0.20);
    let op = Operating::from_tip_mach(0.439, rotor.radius);
    let af = LinearAirfoil {
        a0,
        compressible,
        ..LinearAirfoil::naca0012()
    };
    let cfg = Config {
        tip_loss,
        ..Config::default()
    };
    solve_hover(&rotor, &op, &af, &cfg).ct
}

fn row(label: &str, a0: f64, compressible: bool, tip_loss: bool) {
    print!("{label:<32}");
    let mut max_err: f64 = 0.0;
    for (theta, exp) in ORACLE {
        let ct = ct_at(theta, a0, compressible, tip_loss);
        let err = (ct - exp) / exp * 100.0;
        max_err = max_err.max(err.abs());
        print!("  {ct:.5} ({err:+5.1}%)");
    }
    println!("   maxerr {max_err:4.1}%");
}

pub fn run() {
    println!("Caradonna & Tung C_T sensitivity (oracle: 0.00213 / 0.00460 / 0.00796)\n");
    println!(
        "{:<32}  {:>14}  {:>14}  {:>14}",
        "config", "θ=5°", "θ=8°", "θ=12°"
    );

    row("a0=5.73 comp+tiploss (current)", 5.73, true, true);
    row("a0=5.73 incomp+tiploss", 5.73, false, true);
    row("a0=5.73 comp, no tiploss", 5.73, true, false);
    row("a0=5.73 incomp, no tiploss", 5.73, false, false);
    println!();
    for a0 in [5.73, 5.5, 5.25, 5.0, 4.75] {
        row(&format!("a0={a0:.2} incomp+tiploss"), a0, false, true);
    }
    println!();
    for a0 in [5.73, 5.5, 5.25, 5.0] {
        row(&format!("a0={a0:.2} comp+tiploss"), a0, true, true);
    }
}
