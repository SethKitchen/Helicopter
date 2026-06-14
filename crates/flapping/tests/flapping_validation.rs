//! Flapping validation.
//!
//! Primary (analytic) oracle: the numerically harmonic-balanced flap coefficients
//! match the textbook closed form `(β₀, β₁c, β₁s)(γ, μ, θ, λ)` — the Lock-number
//! anchor, directly analogous to the Leishman (hover) and Glauert (forward)
//! anchors. System checks: the gyroscopic 90° phase lag falls out on its own, the
//! rigid-blade hub rolling moment is reacted by flapping (→0 for a central hinge,
//! residual ∝ hinge offset), and forward flight produces the rearward blow-back.

use helisim_airfoil::LinearAirfoil;
use helisim_flapping::{
    Controls, FlapConfig, FlapProperties, closed_form_coefficients, solve_flapping,
    solve_flapping_with_inflow,
};
use helisim_forward::{ForwardCondition, ForwardConfig, solve_forward};
use helisim_rotor::{Operating, Rotor};

fn rotor_op() -> (Rotor, Operating, LinearAirfoil) {
    let rotor = Rotor::rectangular(2, 1.143, 0.191, 8f64.to_radians(), 0.15);
    let op = Operating::from_tip_mach(0.439, 1.143);
    (rotor, op, LinearAirfoil::naca0012())
}

fn inflow(rotor: &Rotor, op: &Operating, af: &LinearAirfoil, mu: f64) -> f64 {
    solve_forward(
        rotor,
        op,
        af,
        &ForwardCondition::new(mu, 0.0),
        &ForwardConfig::default(),
    )
    .lambda
}

#[test]
fn coefficients_match_closed_form() {
    let (rotor, op, af) = rotor_op();
    let props = FlapProperties::articulated(8.0); // ν=1, central hinge
    let cfg = FlapConfig::default();
    let theta = rotor.collective;

    for &mu in &[0.1, 0.2, 0.3, 0.4] {
        let lam = inflow(&rotor, &op, &af, mu);
        let s = solve_flapping_with_inflow(
            &rotor,
            &op,
            lam,
            &ForwardCondition::new(mu, 0.0),
            &props,
            &Controls::none(),
            &cfg,
        );
        let (c0, c1c, c1s) = closed_form_coefficients(theta, lam, mu, 8.0);
        assert!(
            (s.beta0 - c0).abs() < 5e-4,
            "β0 μ={mu}: {} vs {}",
            s.beta0,
            c0
        );
        assert!(
            (s.beta1c - c1c).abs() < 1e-3,
            "β1c μ={mu}: {} vs {}",
            s.beta1c,
            c1c
        );
        assert!(
            (s.beta1s - c1s).abs() < 1e-3,
            "β1s μ={mu}: {} vs {}",
            s.beta1s,
            c1s
        );
    }
}

#[test]
fn ninety_degree_phase_lag_falls_out() {
    // Hover + a pure cosine (longitudinal) cyclic pitch input. The flap response
    // must come out a quarter-revolution later: purely sine (β1s), with β1c ≈ 0,
    // and β1s ≈ −θ1c. This 90° gyroscopic lag is not imposed — it emerges.
    let (rotor, op, af) = rotor_op();
    let props = FlapProperties::articulated(8.0);
    let cfg = FlapConfig::default();
    let lam = inflow(&rotor, &op, &af, 0.0);
    let theta_1c = 2f64.to_radians();
    let s = solve_flapping_with_inflow(
        &rotor,
        &op,
        lam,
        &ForwardCondition::hover(),
        &props,
        &Controls {
            theta_1c,
            theta_1s: 0.0,
        },
        &cfg,
    );
    assert!(
        s.beta1c.abs() < 1e-4,
        "β1c should be ~0 (response is 90° shifted)"
    );
    assert!((s.beta1s + theta_1c).abs() < 2e-3, "β1s should be ≈ −θ1c");
    assert!(
        (s.phase_lag_deg() - 90.0).abs() < 5.0,
        "phase lag {} should be ~90°",
        s.phase_lag_deg()
    );
}

#[test]
fn forward_flight_blows_back() {
    // Rearward tip-path-plane tilt (β1c > 0) dominates lateral tilt — the
    // advancing-side lift excess, acting through the 90° lag, tilts the disk back.
    let (rotor, op, af) = rotor_op();
    let props = FlapProperties::articulated(8.0);
    let s = solve_flapping(
        &rotor,
        &op,
        &af,
        &ForwardCondition::new(0.3, 0.0),
        &props,
        &Controls::none(),
        &FlapConfig::default(),
    );
    assert!(s.beta1c > 0.0, "should flap back");
    assert!(
        s.beta1c.abs() > s.beta1s.abs(),
        "longitudinal flap should dominate lateral"
    );
    // The aerodynamic forcing's 1/rev is essentially pure sine (lateral asymmetry)
    // while the response is dominated by β1c (longitudinal) — the 90° conversion.
    assert!(
        s.forcing_1c.abs() < 0.1 * s.forcing_1s.abs(),
        "forcing is ~pure sine"
    );
}

#[test]
fn central_hinge_transmits_no_hub_moment_offset_does() {
    let (rotor, op, af) = rotor_op();
    let cond = ForwardCondition::new(0.3, 0.0);
    let cfg = FlapConfig::default();

    // Central hinge: the large forward-flight moment is reacted by flapping, not
    // transmitted to the hub.
    let central = solve_flapping(
        &rotor,
        &op,
        &af,
        &cond,
        &FlapProperties::articulated(8.0),
        &Controls::none(),
        &cfg,
    );
    assert!(central.hub_roll_moment.abs() < 1e-9);
    assert!(central.hub_pitch_moment.abs() < 1e-9);

    // Hinge offset leaves a residual hub moment that grows with the offset.
    let small = solve_flapping(
        &rotor,
        &op,
        &af,
        &cond,
        &FlapProperties::with_offset(8.0, 0.03),
        &Controls::none(),
        &cfg,
    );
    let large = solve_flapping(
        &rotor,
        &op,
        &af,
        &cond,
        &FlapProperties::with_offset(8.0, 0.08),
        &Controls::none(),
        &cfg,
    );
    assert!(
        small.hub_pitch_moment.abs() > 1.0,
        "offset should transmit a hub moment"
    );
    assert!(
        large.hub_pitch_moment.abs() > small.hub_pitch_moment.abs(),
        "moment grows with offset"
    );
}

#[test]
fn higher_lock_number_gives_more_coning() {
    // Coning scales with the Lock number (aerodynamic/inertial ratio).
    let (rotor, op, af) = rotor_op();
    let cfg = FlapConfig::default();
    let lam = inflow(&rotor, &op, &af, 0.0);
    let cone = |g: f64| {
        solve_flapping_with_inflow(
            &rotor,
            &op,
            lam,
            &ForwardCondition::hover(),
            &FlapProperties::articulated(g),
            &Controls::none(),
            &cfg,
        )
        .beta0
    };
    assert!(cone(12.0) > cone(6.0));
}
