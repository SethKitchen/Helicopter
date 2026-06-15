//! The flap↔inflow fixed-point solve.

use crate::config::CoupledConfig;
use crate::loads::{Flow, integrate_with_flap};
use crate::solution::CoupledSolution;
use helisim_airfoil::Airfoil;
use helisim_flapping::{Controls, FlapProperties, solve_flapping_with_inflow};
use helisim_forward::{ForwardCondition, ForwardConfig, glauert_inflow, solve_forward};
use helisim_rotor::{Operating, Rotor};

/// Solve forward flight with flapping and inflow converged together.
///
/// Each iteration: (1) solve the flapping at the current inflow, (2) integrate
/// the blade loads with that flapping folded into `u_P`, (3) update the inflow
/// from momentum (Glauert) for the resulting thrust. Relaxed fixed point.
pub fn solve_coupled(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    cond: &ForwardCondition,
    flap: &FlapProperties,
    controls: &Controls,
    cfg: &CoupledConfig,
) -> CoupledSolution {
    let mu = cond.advance_ratio;
    let tip_mach = op.tip_mach(rotor.radius);

    // Warm start: the rigid (frozen-inflow) forward inflow.
    let mut lambda = solve_forward(rotor, op, airfoil, cond, &ForwardConfig::default()).lambda;
    let mut converged = false;
    let mut iterations = 0;

    for i in 0..cfg.max_iter {
        iterations = i + 1;

        // (1) Flapping responds to the current inflow; (2) loads with that
        // flapping folded into u_P; (3) inflow from momentum for that thrust.
        let fl = solve_flapping_with_inflow(rotor, op, lambda, cond, flap, controls, &cfg.flap);
        let loads = integrate_with_flap(
            rotor,
            airfoil,
            Flow {
                tip_mach,
                mu,
                lambda,
            },
            controls,
            [fl.beta0, fl.beta1c, fl.beta1s],
            [cfg.n_azimuth, cfg.n_radial],
        );
        // Clamp to a physical band so the fixed point stays bounded if the
        // high-μ loads misbehave.
        let lambda_new = glauert_inflow(loads.ct.max(0.0), mu, cond.disk_aoa, 1e-12, 100);
        let delta = (lambda_new - lambda).abs();
        lambda = (lambda + cfg.relax * (lambda_new - lambda)).clamp(1e-4, 0.5);

        if delta < cfg.tol {
            converged = true;
            break;
        }
    }

    // Final flapping + loads at the converged inflow.
    let fl = solve_flapping_with_inflow(rotor, op, lambda, cond, flap, controls, &cfg.flap);
    let loads = integrate_with_flap(
        rotor,
        airfoil,
        Flow {
            tip_mach,
            mu,
            lambda,
        },
        controls,
        [fl.beta0, fl.beta1c, fl.beta1s],
        [cfg.n_azimuth, cfg.n_radial],
    );

    CoupledSolution {
        mu,
        lambda,
        beta0: fl.beta0,
        beta1c: fl.beta1c,
        beta1s: fl.beta1s,
        ct: loads.ct,
        cp: loads.cp,
        cp_profile: loads.cp_profile,
        c_roll: loads.c_roll,
        c_pitch: loads.c_pitch,
        advancing_ct: loads.advancing_ct,
        retreating_ct: loads.retreating_ct,
        converged,
        iterations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_airfoil::LinearAirfoil;

    fn setup() -> (Rotor, Operating, LinearAirfoil) {
        let rotor = Rotor::rectangular(2, 1.143, 0.191, 8f64.to_radians(), 0.15);
        let op = Operating::from_tip_mach(0.439, 1.143);
        (rotor, op, LinearAirfoil::naca0012())
    }

    #[test]
    fn converges_and_equalises_loading() {
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
        assert!(
            s.converged,
            "flap↔inflow should converge (iters {})",
            s.iterations
        );

        // Rigid forward solve over-predicts advancing-side thrust; the coupled
        // result must be more equalised (smaller advancing/retreating ratio).
        let rigid = solve_forward(&rotor, &op, &af, &cond, &ForwardConfig::default());
        let coupled_ratio = s.advancing_ct / s.retreating_ct;
        let rigid_ratio = rigid.advancing_ct / rigid.retreating_ct;
        assert!(
            coupled_ratio < rigid_ratio,
            "coupling should equalise loading"
        );
    }
}
