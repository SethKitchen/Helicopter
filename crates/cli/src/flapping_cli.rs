//! `flapping` subcommand: how blade flapping resolves the rigid-blade rolling
//! moment — the moment is reacted by a tip-path-plane tilt, not transmitted to
//! the hub — plus the gyroscopic 90° phase lag.

use helisim_airfoil::LinearAirfoil;
use helisim_flapping::{
    Controls, FlapConfig, FlapProperties, solve_flapping, solve_flapping_with_inflow,
};
use helisim_forward::{ForwardCondition, ForwardConfig, solve_forward};
use helisim_rotor::{Operating, Rotor};

pub fn run() {
    let rotor = Rotor::rectangular(2, 1.143, 0.191, 8f64.to_radians(), 0.15);
    let op = Operating::from_tip_mach(0.439, 1.143);
    let af = LinearAirfoil::naca0012();
    let props = FlapProperties::articulated(8.0); // γ=8, central hinge (ν=1)
    let cfg = FlapConfig::default();
    let fwd = ForwardConfig::default();

    println!("helisim — rigid-blade first-harmonic flapping (harmonic-balance solve)\n");
    println!(
        "Rotor: C&T geometry, R={:.3} m, {:.0} RPM, collective 8°, Lock γ={:.0}, central hinge\n",
        rotor.radius,
        op.rpm(),
        props.lock_number
    );

    println!(
        "{:>5} {:>9} {:>8} {:>8} {:>8} {:>11} {:>10}",
        "mu", "coning°", "b1c°", "b1s°", "TPP°", "rigidRoll", "flapHub"
    );
    for k in 1..=8 {
        let mu = k as f64 * 0.05;
        let cond = ForwardCondition::new(mu, 0.0);
        let rigid = solve_forward(&rotor, &op, &af, &cond, &fwd);
        let s = solve_flapping_with_inflow(
            &rotor,
            &op,
            rigid.lambda,
            &cond,
            &props,
            &Controls::none(),
            &cfg,
        );
        println!(
            "{:>5.2} {:>9.3} {:>8.3} {:>8.3} {:>8.3} {:>9.0}N·m {:>8.0}N·m",
            mu,
            s.beta0.to_degrees(),
            s.beta1c.to_degrees(),
            s.beta1s.to_degrees(),
            s.tpp_tilt().to_degrees(),
            rigid.rolling_moment_nm(&op, &rotor).abs(),
            s.hub_roll_moment.abs(),
        );
    }

    // Headline conversion at a cruise advance ratio.
    let cond = ForwardCondition::new(0.30, 0.0);
    let rigid = solve_forward(&rotor, &op, &af, &cond, &fwd);
    let s = solve_flapping_with_inflow(
        &rotor,
        &op,
        rigid.lambda,
        &cond,
        &props,
        &Controls::none(),
        &cfg,
    );
    println!("\n=== Headline: the rolling moment is resolved by flapping (μ=0.30) ===");
    println!(
        "  rigid blade  : {:.0} N·m uncommanded ROLLING moment at the hub",
        rigid.rolling_moment_nm(&op, &rotor).abs()
    );
    println!(
        "  hinged blade : hub moment {:.1} N·m (central hinge transmits none) — instead the",
        s.hub_roll_moment.abs()
    );
    println!(
        "                 tip-path plane tilts {:.2}° back (β1c) and {:.2}° laterally (β1s).",
        s.beta1c.to_degrees(),
        s.beta1s.to_degrees()
    );

    // Hinge offset leaves a residual hub moment.
    let off = solve_flapping(
        &rotor,
        &op,
        &af,
        &cond,
        &FlapProperties::with_offset(8.0, 0.05),
        &Controls::none(),
        &cfg,
    );
    println!(
        "  5% hinge offset: ν_β={:.3}, residual hub pitch moment {:.0} N·m (∝ offset).",
        off.nu_beta,
        off.hub_pitch_moment.abs()
    );

    // The 90° phase lag, cleanly: cosine cyclic pitch in hover → sine flap.
    let lam_h = solve_forward(&rotor, &op, &af, &ForwardCondition::hover(), &fwd).lambda;
    let s = solve_flapping_with_inflow(
        &rotor,
        &op,
        lam_h,
        &ForwardCondition::hover(),
        &props,
        &Controls::from_deg(2.0, 0.0),
        &cfg,
    );
    println!("\n=== Gyroscopic 90° phase lag (hover, 2° cosine cyclic θ1c) ===");
    println!(
        "  pitch input is pure cosψ; flap response is pure sinψ: β1c={:.2}°, β1s={:.2}°.",
        s.beta1c.to_degrees(),
        s.beta1s.to_degrees()
    );
    println!(
        "  forcing phase {:.0}°, response phase {:.0}° → lag {:.0}° (not imposed — it emerges).",
        s.forcing_phase_deg(),
        s.response_phase_deg(),
        s.phase_lag_deg(),
    );
}
