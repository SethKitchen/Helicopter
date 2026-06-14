//! The flapping solve: harmonic-balance assembly → 3×3 linear solve → hub moments.

use crate::config::FlapConfig;
use crate::controls::Controls;
use crate::harmonics::build_system;
use crate::linalg::solve3;
use crate::properties::FlapProperties;
use crate::solution::FlapSolution;
use helisim_airfoil::Airfoil;
use helisim_forward::{ForwardCondition, ForwardConfig, solve_forward};
use helisim_rotor::{Operating, Rotor};

/// Solve first-harmonic flapping for a given inflow `lambda` (use this when you
/// already have the forward-flight inflow and want to control it explicitly).
pub fn solve_flapping_with_inflow(
    rotor: &Rotor,
    op: &Operating,
    lambda: f64,
    cond: &ForwardCondition,
    props: &FlapProperties,
    controls: &Controls,
    cfg: &FlapConfig,
) -> FlapSolution {
    let mu = cond.advance_ratio;
    let sys = build_system(rotor, controls, mu, lambda, cfg);
    let nu2 = props.nu_beta_sq();
    let gamma = props.lock_number;

    // [diag(ν², 1−ν², 1−ν²) − (γ/2) G] b = (γ/2) F
    let diag = [nu2, 1.0 - nu2, 1.0 - nu2];
    let mut a = [[0.0_f64; 3]; 3];
    for (r, (a_row, g_row)) in a.iter_mut().zip(sys.response.iter()).enumerate() {
        for (a_rc, g_rc) in a_row.iter_mut().zip(g_row.iter()) {
            *a_rc = -0.5 * gamma * g_rc;
        }
        a_row[r] += diag[r];
    }
    let rhs = [
        0.5 * gamma * sys.forcing[0],
        0.5 * gamma * sys.forcing[1],
        0.5 * gamma * sys.forcing[2],
    ];
    let b = solve3(a, rhs);
    let (beta0, beta1c, beta1s) = (b[0], b[1], b[2]);

    // Dimensional hub moment. Recover flap inertia from the Lock number:
    // I_β = ρ a c R⁴ / γ. The effective flap stiffness from hinge offset is
    // K_β = I_β Ω² (ν²−1); the steady hub moment from N_b blades is (N_b/2) K_β β₁.
    let i_beta = op.rho * props.lift_slope * rotor.tip_chord * rotor.radius.powi(4) / gamma;
    let k_beta = i_beta * op.omega * op.omega * (nu2 - 1.0);
    let half_nb = 0.5 * rotor.n_blades as f64;
    let hub_pitch_moment = half_nb * k_beta * beta1c;
    let hub_roll_moment = half_nb * k_beta * beta1s;

    FlapSolution {
        mu,
        lambda,
        nu_beta: nu2.sqrt(),
        beta0,
        beta1c,
        beta1s,
        hub_pitch_moment,
        hub_roll_moment,
        forcing_1c: sys.forcing[1],
        forcing_1s: sys.forcing[2],
    }
}

/// Solve flapping, taking the inflow from a forward-flight BEMT solve (the
/// documented one-way coupling: flapping responds to the rigid-blade inflow).
pub fn solve_flapping(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    cond: &ForwardCondition,
    props: &FlapProperties,
    controls: &Controls,
    cfg: &FlapConfig,
) -> FlapSolution {
    let lambda = solve_forward(rotor, op, airfoil, cond, &ForwardConfig::default()).lambda;
    solve_flapping_with_inflow(rotor, op, lambda, cond, props, controls, cfg)
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_airfoil::LinearAirfoil;

    fn rotor_op() -> (Rotor, Operating, LinearAirfoil) {
        let rotor = Rotor::rectangular(2, 1.143, 0.191, 8f64.to_radians(), 0.15);
        let op = Operating::from_tip_mach(0.439, 1.143);
        (rotor, op, LinearAirfoil::naca0012())
    }

    #[test]
    fn coning_is_positive_and_cyclic_grows_with_speed() {
        let (rotor, op, af) = rotor_op();
        let props = FlapProperties::articulated(8.0);
        let hover = solve_flapping(
            &rotor,
            &op,
            &af,
            &ForwardCondition::hover(),
            &props,
            &Controls::none(),
            &FlapConfig::default(),
        );
        let fwd = solve_flapping(
            &rotor,
            &op,
            &af,
            &ForwardCondition::new(0.3, 0.0),
            &props,
            &Controls::none(),
            &FlapConfig::default(),
        );
        assert!(hover.beta0 > 0.0, "coning should be positive");
        assert!(
            hover.beta1c.abs() < 1e-6 && hover.beta1s.abs() < 1e-6,
            "no cyclic flap in hover"
        );
        assert!(
            fwd.tpp_tilt() > hover.tpp_tilt(),
            "forward flight should tilt the TPP"
        );
    }
}
