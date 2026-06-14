//! Validation of the two-way flap↔inflow coupling.

use helisim_airfoil::LinearAirfoil;
use helisim_coupled::{CoupledConfig, solve_coupled};
use helisim_flapping::{Controls, FlapProperties};
use helisim_forward::{ForwardCondition, ForwardConfig, solve_forward};
use helisim_rotor::{Operating, Rotor};

fn setup() -> (Rotor, Operating, LinearAirfoil) {
    let rotor = Rotor::rectangular(2, 1.143, 0.191, 8f64.to_radians(), 0.15);
    let op = Operating::from_tip_mach(0.439, 1.143);
    (rotor, op, LinearAirfoil::naca0012())
}

#[test]
fn fixed_point_converges_across_speed() {
    let (rotor, op, af) = setup();
    let flap = FlapProperties::articulated(8.0);
    let cfg = CoupledConfig::default();
    for &mu in &[0.1, 0.2, 0.3, 0.4] {
        let s = solve_coupled(
            &rotor,
            &op,
            &af,
            &ForwardCondition::new(mu, 0.0),
            &flap,
            &Controls::none(),
            &cfg,
        );
        assert!(
            s.converged,
            "μ={mu} should converge (iters {})",
            s.iterations
        );
        assert!(s.iterations < cfg.max_iter, "μ={mu} converged early");
        assert!(s.lambda > 0.0 && s.lambda < 0.5, "inflow bounded");
    }
}

#[test]
fn coupling_redistributes_loading_off_the_advancing_side() {
    // The whole point: flapping folded into the loads sheds the advancing-side
    // excess the rigid solve over-predicts, so the advancing/retreating loading
    // ratio drops toward unity.
    let (rotor, op, af) = setup();
    let cond = ForwardCondition::new(0.3, 0.0);
    let flap = FlapProperties::articulated(8.0);
    let s = solve_coupled(
        &rotor,
        &op,
        &af,
        &cond,
        &flap,
        &Controls::none(),
        &CoupledConfig::default(),
    );
    let rigid = solve_forward(&rotor, &op, &af, &cond, &ForwardConfig::default());

    let coupled_ratio = s.advancing_ct / s.retreating_ct;
    let rigid_ratio = rigid.advancing_ct / rigid.retreating_ct;
    assert!(
        coupled_ratio < rigid_ratio,
        "coupling should equalise loading"
    );
    assert!(
        coupled_ratio > 1.0,
        "advancing side still carries more (just less so)"
    );
}

#[test]
fn profile_power_is_nonnegative() {
    // The reliable high-μ power component.
    let (rotor, op, af) = setup();
    let flap = FlapProperties::articulated(8.0);
    for &mu in &[0.1, 0.3, 0.5] {
        let s = solve_coupled(
            &rotor,
            &op,
            &af,
            &ForwardCondition::new(mu, 0.0),
            &flap,
            &Controls::none(),
            &CoupledConfig::default(),
        );
        assert!(s.cp_profile >= 0.0, "profile power must be ≥0 at μ={mu}");
    }
}
