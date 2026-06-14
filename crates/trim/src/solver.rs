//! The trim entry point: Newton solve (with speed continuation for forward
//! flight) over the six force/moment residuals.

use crate::aircraft::Aircraft;
use crate::condition::TrimCondition;
use crate::newton::{NewtonConfig, solve_newton};
use crate::residual::{calibrate_kappa, evaluate};
use crate::solution::TrimResult;

/// Initial guess: a plausible hover setting (collective 8°, small tail collective,
/// level attitude, no cyclic).
fn initial_guess() -> Vec<f64> {
    vec![
        8f64.to_radians(), // θ0
        0.0,               // θ1c
        0.0,               // θ1s
        5f64.to_radians(), // θ0_tr
        0.0,               // pitch
        0.0,               // roll
    ]
}

/// Trim the aircraft for the given steady flight condition.
///
/// Hover is solved directly. Forward flight uses **speed continuation**: trim is
/// first found in hover, then marched up to the target speed in ~5 m/s steps,
/// each Newton solve warm-started from the previous — robust against the larger
/// nonlinearity at speed.
pub fn trim(ac: &Aircraft, cond: &TrimCondition, cfg: &NewtonConfig) -> TrimResult {
    // Hover solve first (also the warm start for forward flight).
    let hover = TrimCondition::hover();
    let (mut x, mut norm, mut ok) = solve_newton(
        |x| evaluate(ac, &hover, x, 1.15).residuals.to_vec(),
        &initial_guess(),
        cfg,
    );

    // Calibrate the induced power factor at the hover collective.
    let kappa = calibrate_kappa(ac, x[0]);

    if cond.forward_speed > 1e-6 {
        let target = cond.forward_speed;
        let steps = (target / 5.0).ceil().max(1.0) as usize;
        for s in 1..=steps {
            let v = target * s as f64 / steps as f64;
            let c = TrimCondition::forward(v);
            let r = solve_newton(|x| evaluate(ac, &c, x, kappa).residuals.to_vec(), &x, cfg);
            x = r.0;
            norm = r.1;
            ok = r.2;
        }
    }

    build_result(ac, cond, &x, norm, ok, kappa)
}

fn build_result(
    ac: &Aircraft,
    cond: &TrimCondition,
    x: &[f64],
    norm: f64,
    ok: bool,
    kappa: f64,
) -> TrimResult {
    let e = evaluate(ac, cond, x, kappa);
    TrimResult {
        converged: ok,
        residual_norm: norm,
        forward_speed: cond.forward_speed,
        mu: e.mu,
        collective: x[0],
        cyclic_lat: x[1],
        cyclic_lon: x[2],
        tail_collective: x[3],
        pitch: x[4],
        roll: x[5],
        thrust: e.main.thrust,
        main_power: e.main.power,
        tail_thrust: e.tail_thrust,
        tail_power: e.tail_power,
        parasite_power: e.parasite_power,
        total_power: e.main.power + e.tail_power + e.parasite_power,
        beta1c: e.main.beta1c,
        beta1s: e.main.beta1s,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hover_trims_thrust_to_weight() {
        let ac = Aircraft::demo();
        let r = trim(&ac, &TrimCondition::hover(), &NewtonConfig::default());
        assert!(
            r.converged,
            "hover trim should converge (norm {})",
            r.residual_norm
        );
        let w = ac.mass * crate::G;
        // Thrust supports the weight (small attitude → ~equal).
        assert!(
            (r.thrust - w).abs() / w < 0.02,
            "thrust {} vs weight {}",
            r.thrust,
            w
        );
        // Tail rotor balances main torque: T_tr · arm ≈ Q.
        assert!((r.tail_thrust * ac.tail.arm - r.main_power / ac.main_op.omega).abs() < 0.5);
    }
}
