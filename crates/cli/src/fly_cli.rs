//! `fly` subcommand: control-input time histories on the driven 11-state system
//! (5i). Shows the control conventions/effectiveness, the off-axis sign flip in
//! the driven control derivatives (the 5h result, now in the loop), and the
//! open-loop divergence to a control pulse.

use helisim_dynamics::{Inertia, eigenvalues, quasi_static_inflow};
use helisim_flapping::Controls;
use helisim_sim::{
    Channel, Pulse, control_matrix11, equilibrium_state11, linearize11, simulate11,
    solve_equilibrium11,
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
    let eq = equilibrium_state11(&ac);

    println!("helisim — control-input time histories on the driven 11-state system (5i)\n");
    println!(
        "State [u,w,q,θ,v,p,r,φ,λ₀,λ₁s,λ₁c] — the rigid body PLUS Pitt–Peters inflow,\n\
         integrated together with time-varying controls. The inflow is in the loop, so\n\
         the rotor response carries the correct timing (5h was the precondition).\n"
    );
    println!(
        "11-state equilibrium: pitch {:.2}°, roll {:.2}°, ν_e=[{:.4},{:.4},{:.4}]\n",
        eq[3].to_degrees(),
        eq[7].to_degrees(),
        eq[8],
        eq[9],
        eq[10]
    );

    // Control conventions + effectiveness.
    let b = control_matrix11(&ac, j);
    println!("Control effectiveness B = ∂ẋ/∂u (per rad) — signs pinned to physical effect:");
    let names = ["u̇", "ẇ", "q̇", "θ̇", "v̇", "ṗ", "ṙ", "φ̇"];
    print!("{:>6}", "");
    for c in [
        Channel::Collective,
        Channel::LatCyclic,
        Channel::LonCyclic,
        Channel::Pedal,
    ] {
        print!("{:>10}", c.name().split(' ').next_back().unwrap());
    }
    println!();
    for (r, nm) in names.iter().enumerate() {
        print!("{nm:>6}");
        for val in &b[r] {
            print!("{val:>10.2}");
        }
        println!();
    }
    println!(
        "  collective→climb (ẇ<0), +lat-cyc→right roll (ṗ>0), +lon-cyc→pitch, pedal→yaw;\n  \
         on-axis dominates off-axis ~20:1.\n"
    );

    // Off-axis sign flip in the driven control derivative (the 5h result, in the loop).
    let (coll, _, t1c, t1s, _, _, _) = solve_equilibrium11(&ac);
    let rotor = ac.main.with_collective(coll);
    let my = |dt1c: f64| {
        quasi_static_inflow(
            &rotor,
            &ac.main_op,
            ac.main_airfoil.as_ref(),
            &ac.flap,
            ac.hub_height,
            &Controls {
                theta_1c: t1c + dt1c,
                theta_1s: t1s,
            },
            [0.0; 3],
            [0.0; 2],
        )
        .0
        .my
    };
    let settled = (my(0.01) - my(-0.01)) / 0.02;
    println!("Off-axis pitch response to lateral cyclic, ∂My/∂θ1c:");
    println!(
        "  instantaneous (inflow frozen at t=0): {:.2}",
        b[2][1] * j.i_yy
    );
    println!("  settled       (inflow developed)    : {settled:+.2}");
    println!(
        "  The sign flips as the inflow develops — the 5h −3.2→+0.5 correction, now\n  \
         a TIME-DOMAIN effect: a lateral-cyclic step pitches one way at first, the\n  \
         other way once the inflow lag plays out.\n"
    );

    // Open-loop divergence to a control pulse.
    let pulse = Pulse {
        channel: Channel::LonCyclic,
        amplitude: 0.005,
        t_start: 0.0,
        duration: 0.2,
    };
    let dt = 0.01;
    let traj = simulate11(&ac, j, &pulse, [0.0; 11], dt, 3.0);
    let eigs = eigenvalues(&linearize11(&ac, j));
    let mut uns: Vec<_> = eigs.iter().filter(|e| e.re > 0.05 && e.im > 0.05).collect();
    uns.sort_by(|x, y| x.im.partial_cmp(&y.im).unwrap());
    println!("Open-loop response to a 0.005-rad longitudinal-cyclic pulse (0.2 s), then released:");
    println!(
        "  unstable modes: {:?}",
        uns.iter()
            .map(|e| format!("{:.2}{:+.2}i", e.re, e.im))
            .collect::<Vec<_>>()
    );
    println!(
        "{:>6} {:>10} {:>10} {:>10}",
        "t s", "u m/s", "θ deg", "φ deg"
    );
    for t in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0] {
        let s = traj[(t / dt) as usize];
        println!(
            "{:>6.1} {:>10.4} {:>10.2} {:>10.2}",
            t,
            s[0] - eq[0],
            (s[3] - eq[3]).to_degrees(),
            (s[7] - eq[7]).to_degrees()
        );
    }
    println!(
        "\n  The controls are back at trim by 0.2 s, yet the aircraft keeps diverging —\n  \
         open-loop unstable, as the modes demand. Taming this (rate feedback → modes\n  \
         into the LHP) is milestone 5j.\n  \
         NOTE: the hover divergence runs FASTER than the hover-linearized rate — the\n  \
         dynamic-inflow wake skew χ(μ) is non-analytic at μ=0, so the same λ₀↔λ₁c\n  \
         coupling that flips the off-axis sign also escapes the hover Jacobian."
    );
}
