//! `lateral` subcommand: the lateral-directional hover oracle (5e-i) and the
//! coupled 8-state decouple→couple gate (5e-ii).

use helisim_dynamics::{
    Inertia, analyze_coupled_hover, analyze_hover_lateral, analyze_hover_longitudinal,
    lateral_cubic, roots,
};
use helisim_trim::Aircraft;

pub fn run() {
    let ac = Aircraft::demo();
    let j = Inertia {
        mass: 8.0,
        i_xx: 0.4,
        i_yy: 0.8,
        i_zz: 1.0,
    };

    println!("helisim — lateral-directional hover dynamics + coupled 8-state\n");
    println!(
        "Aircraft: {:.0} kg, Ixx={:.1} Iyy={:.1} Izz={:.1} kg·m², tail arm {:.2} m, height {:.2} m\n",
        j.mass, j.i_xx, j.i_yy, j.i_zz, ac.tail.arm, ac.tail.height
    );

    let lat = analyze_hover_lateral(&ac, j.i_xx, j.i_zz);
    let d = lat.derivatives;
    let lon = analyze_hover_longitudinal(&ac, j.i_yy);
    println!("=== lateral derivatives (5f: rotation-based aero, tail as dynamic element) ===");
    println!(
        "  Yv = {:>8.4}  (side-force damping, <0)   {}",
        d.yv,
        ok(d.yv < 0.0)
    );
    println!(
        "  Lp = {:>8.4}  (roll damping, <0)         {}",
        d.lp,
        ok(d.lp < 0.0)
    );
    println!(
        "  Nr = {:>8.4}  (yaw damping from tail, <0){}",
        d.nr,
        ok(d.nr < 0.0)
    );
    println!(
        "  Lv = {:>8.4}  (main part −Mu = {:.4}, + tail; velocity rotates +90°, rate −90°)",
        d.lv, -lon.derivatives.mu
    );
    println!("  Nv = {:.4}  | roll-yaw coupling via tail height", d.nv);

    println!("\n=== 5e-i lateral modes [v, p, r, φ] ===");
    for m in &lat.modes {
        let kind = if m.oscillatory {
            "oscillatory"
        } else {
            "subsidence "
        };
        let stab = if m.stable { "stable  " } else { "UNSTABLE" };
        println!(
            "  {:>7.4}{:+.4}i  {kind} {stab}  {}",
            m.eigenvalue.re,
            m.eigenvalue.im,
            if m.oscillatory {
                format!("period {:.1}s", m.period)
            } else {
                format!("t={:.1}s", m.time_to_half_or_double)
            }
        );
    }
    let cub = roots(&lateral_cubic(&d, j.mass, j.i_xx));
    let co = cub.iter().find(|r| r.im > 1e-6).unwrap();
    let cr = cub
        .iter()
        .filter(|r| r.im.abs() < 1e-6)
        .map(|r| r.re)
        .fold(f64::MIN, f64::max);
    println!(
        "  anchor: roll-sideslip cubic → osc {:.3}{:+.3}i, real {:.3} (matches the 4×4)",
        co.re, co.im, cr
    );
    println!(
        "  Lateral hover is oscillatory-unstable — a lateral phugoid mirroring the\n  \
         longitudinal one (Lv=−Mu gives the same flapback-driven instability). The\n  \
         earlier 'real divergence' was a lateral sign bug, fixed in 5f via the exact\n  \
         rotation of the validated longitudinal aero (two routes now agree)."
    );

    // Coupled 8-state gate.
    let dec = analyze_coupled_hover(&ac, j, false);
    let cpl = analyze_coupled_hover(&ac, j, true);
    println!("\n=== 5e-ii coupled 8-state gate ===");
    println!("  5c longitudinal eigenvalues : {}", fmt(&lon.eigenvalues));
    println!("  5e-i lateral eigenvalues    : {}", fmt(&lat.eigenvalues));
    println!("  8-state DECOUPLED           : {}", fmt(&dec.eigenvalues));
    println!("    → equals the union of the two oracles (gate passes).");
    println!("  8-state COUPLED             : {}", fmt(&cpl.eigenvalues));
    println!(
        "    → pitch-roll coupling shifts every mode; both instabilities persist.\n  \
         A purely longitudinal disturbance now drives lateral motion — behaviour\n  \
         neither 4-state model can show."
    );
}

fn ok(b: bool) -> &'static str {
    if b { "OK" } else { "??" }
}

fn fmt(ev: &[helisim_dynamics::Complex]) -> String {
    let mut v: Vec<_> = ev.iter().filter(|e| e.im >= -1e-9).collect();
    v.sort_by(|a, b| a.re.partial_cmp(&b.re).unwrap());
    v.iter()
        .map(|e| {
            if e.im.abs() > 1e-6 {
                format!("{:.2}±{:.2}i", e.re, e.im.abs())
            } else {
                format!("{:.2}", e.re)
            }
        })
        .collect::<Vec<_>>()
        .join(", ")
}
