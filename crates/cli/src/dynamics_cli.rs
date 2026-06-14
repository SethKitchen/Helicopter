//! `dynamics` subcommand: hover stability & control derivatives, the linearized
//! system matrix, and its eigenvalues — showing the famous open-loop hover
//! instability falling out of the derivatives.

use helisim_dynamics::{analyze_hover_longitudinal, hovering_cubic, roots};
use helisim_trim::Aircraft;

pub fn run() {
    let ac = Aircraft::demo();
    let i_yy = 0.8; // pitch inertia, kg·m²
    let a = analyze_hover_longitudinal(&ac, i_yy);
    let d = a.derivatives;

    println!("helisim — linearized hover flight dynamics (stability derivatives + modes)\n");
    println!(
        "Aircraft: {:.0} kg, Iyy={:.2} kg·m², main R={:.2} m, hover collective {:.2}°\n",
        ac.mass,
        i_yy,
        ac.main.radius,
        a.collective.to_degrees()
    );

    println!("=== Longitudinal stability derivatives (force/moment-based, no κ) ===");
    let sign = |v: f64, want_pos: bool| {
        let ok = if want_pos { v > 0.0 } else { v < 0.0 };
        if ok { "OK" } else { "??" }
    };
    println!(
        "  Xu = {:>8.4}  (drag damping, <0)   {}",
        d.xu,
        sign(d.xu, false)
    );
    println!(
        "  Zw = {:>8.4}  (heave damping, <0)  {}",
        d.zw,
        sign(d.zw, false)
    );
    println!(
        "  Mu = {:>8.4}  (speed stability, >0 DESTABILIZING)  {}",
        d.mu,
        sign(d.mu, true)
    );
    println!(
        "  Mq = {:>8.4}  (pitch damping, <0)  {}",
        d.mq,
        sign(d.mq, false)
    );
    println!(
        "  (Xw {:.4}, Zu {:.4}, Mw {:.4}, Xq {:.4}, Zq {:.4})",
        d.xw, d.zu, d.mw, d.xq, d.zq
    );

    println!("\n=== System matrix A  (states [u, w, q, θ]) ===");
    for row in &a.a_matrix {
        println!(
            "  [{:>9.4} {:>9.4} {:>9.4} {:>9.4}]",
            row[0], row[1], row[2], row[3]
        );
    }

    println!("\n=== Eigenvalues / modes ===");
    for m in &a.modes {
        let kind = if m.oscillatory {
            "oscillatory"
        } else {
            "subsidence "
        };
        let stab = if m.stable { "stable  " } else { "UNSTABLE" };
        if m.oscillatory {
            println!(
                "  {:>8.4} {:+.4}i  {kind} {stab}  period {:>5.1}s  {} {:.1}s",
                m.eigenvalue.re,
                m.eigenvalue.im,
                m.period,
                if m.stable { "t½" } else { "t×2" },
                m.time_to_half_or_double
            );
        } else {
            println!(
                "  {:>8.4}          {kind} {stab}  {} {:.1}s",
                m.eigenvalue.re,
                if m.stable { "t½" } else { "t×2" },
                m.time_to_half_or_double
            );
        }
    }

    if a.has_unstable_oscillation {
        println!(
            "\nHEADLINE: the hover has an UNSTABLE oscillatory mode — the open-loop\n\
             pitch–speed instability (the 'hovering cubic'). It emerged from the\n\
             derivatives (Mu>0 with weak damping); it was not put there. This is why\n\
             a hovering helicopter is hard to fly and why stability augmentation exists."
        );
    }

    // Analytic cross-check of the eigenvalue routine.
    let cubic = hovering_cubic(&d, ac.mass, i_yy);
    let cr = roots(&cubic);
    if let Some(z) = cr.iter().find(|r| r.im > 1e-6) {
        let full = a
            .modes
            .iter()
            .find(|m| m.oscillatory && m.eigenvalue.im > 0.0)
            .unwrap()
            .eigenvalue;
        println!(
            "\nCross-check: analytic hovering cubic complex root {:.4}{:+.4}i  vs  4×4 eigenvalue {:.4}{:+.4}i  -> MATCH",
            z.re, z.im, full.re, full.im
        );
    }
}
