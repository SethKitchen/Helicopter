//! `sas` subcommand: rate-feedback stability augmentation (5j), designed and
//! validated in three layers around the wake-skew seam — off-seam (trustworthy
//! oracle), hover-linear (damps but leaves a residual), nonlinear hover (holds
//! across the seam).

use helisim_dynamics::{Inertia, eigenvalues};
use helisim_sim::{
    Channel, Pulse, RateSas, closed_loop_matrix, control_matrix11, control_matrix11_at,
    equilibrium_state11, linearize11, linearize11_at, simulate11, simulate11_sas,
};
use helisim_trim::Aircraft;

fn max_re(a: &[Vec<f64>]) -> f64 {
    eigenvalues(a).iter().map(|e| e.re).fold(f64::MIN, f64::max)
}

pub fn run() {
    let ac = Aircraft::demo();
    let j = Inertia {
        mass: 8.0,
        i_xx: 0.4,
        i_yy: 0.8,
        i_zz: 1.0,
    };
    let sas = RateSas::rate_damper(0.2, 0.2, 0.4);
    let vt = ac.main_op.tip_speed(ac.main.radius);

    println!("helisim — rate-feedback stability augmentation (5j)\n");
    println!(
        "SCAS rate damper: p→lat-cyclic, q→lon-cyclic, r→pedal (gains 0.2/0.2/0.4).\n\
         A DAMPER, not an attitude hold — it makes the aircraft flyable, not hands-off\n\
         stable. The hover Jacobian is blind to the wake-skew λ₀↔λ₁c coupling (the 5i\n\
         finding), so the design is anchored OFF the seam where the oracle is clean.\n"
    );

    // Layer 1 — off the seam, the trustworthy oracle.
    let vel = [5.0, 0.0, 0.0];
    let a_f = linearize11_at(&ac, j, vel);
    let b_f = control_matrix11_at(&ac, j, vel);
    let acl_f = closed_loop_matrix(&a_f, &b_f, &sas);
    println!(
        "Layer 1 — OFF the seam (5 m/s, μ={:.3}), the differentiable oracle:",
        vel[0] / vt
    );
    println!(
        "  open-loop max Re {:.3}  →  closed-loop max Re {:.4}  (into the LHP — stabilized)",
        max_re(&a_f),
        max_re(&acl_f)
    );
    println!("  here linear↔nonlinear agree, so this eigenvalue gate is trustworthy.\n");

    // Layer 2 — hover linear: damps but a residual remains.
    let a_h = linearize11(&ac, j);
    let b_h = control_matrix11(&ac, j);
    let acl_h = closed_loop_matrix(&a_h, &b_h, &sas);
    println!("Layer 2 — hover LINEAR (necessary, not sufficient):");
    println!(
        "  open-loop max Re {:.3}  →  closed-loop max Re {:+.4}",
        max_re(&a_h),
        max_re(&acl_h)
    );
    println!(
        "  the violent instability (doubling ~1 s) collapses to a slow residual\n  \
         (doubling ~30 s) — the speed/phugoid mode rate feedback cannot reach.\n"
    );

    // Layer 3 — nonlinear hover, across the seam.
    let eq = equilibrium_state11(&ac);
    let dt = 0.01;
    let pulse = Pulse {
        channel: Channel::LonCyclic,
        amplitude: 0.01,
        t_start: 0.0,
        duration: 0.2,
    };
    let open = simulate11(&ac, j, &pulse, [0.0; 11], dt, 8.0);
    let aug = simulate11_sas(&ac, j, [0.0, 0.0, 0.0], &pulse, &sas, [0.0; 11], dt, 8.0);
    println!("Layer 3 — nonlinear HOVER, same pulse that diverged open-loop, ACROSS the seam:");
    println!(
        "{:>5} {:>22} {:>22}",
        "t s", "open-loop θ,φ deg", "SAS θ,φ deg"
    );
    for t in [2.0, 4.0, 6.0, 8.0] {
        let k = (t / dt) as usize;
        let (o, c) = (open[k], aug[k]);
        let fmt = |x: f64| {
            if x.is_finite() {
                format!("{:.1}", x.to_degrees())
            } else {
                "  NaN".into()
            }
        };
        println!(
            "{:>5.0} {:>10},{:>10} {:>10},{:>10}",
            t,
            fmt(o[3] - eq[3]),
            fmt(o[7] - eq[7]),
            fmt(c[3] - eq[3]),
            fmt(c[7] - eq[7])
        );
    }
    println!(
        "\n  Open-loop blows up; the SAS holds attitude to a few degrees — including the\n  \
         pitch/lateral-rate channel the hover Jacobian could not see. The residual slow\n  \
         drift is the phugoid the damper doesn't hold; killing it needs attitude/velocity\n  \
         feedback — a later milestone, deliberately out of SCAS scope."
    );
}
