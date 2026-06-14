//! Forward-flight BEMT solve: outer Glauert-inflow bisection wrapping an inner
//! azimuthal + radial blade-element double integral.

use crate::condition::ForwardCondition;
use crate::config::ForwardConfig;
use crate::inflow::momentum_ct;
use crate::solution::ForwardSolution;
use helisim_airfoil::Airfoil;
use helisim_rotor::{Operating, Rotor};
use std::f64::consts::PI;

/// Accumulated integrals from one inner double integral at a trial inflow.
#[derive(Clone, Copy, Default)]
struct Integrals {
    ct: f64,
    cp: f64,
    c_roll: f64,
    c_pitch: f64,
    advancing_ct: f64,
    retreating_ct: f64,
    reverse_fraction: f64,
}

/// Inner double integral: blade-element loads over azimuth and radius at a given
/// total inflow `lambda`. All coefficients are azimuth-averaged.
fn integrate(
    lambda: f64,
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    cond: &ForwardCondition,
    cfg: &ForwardConfig,
) -> Integrals {
    let mu = cond.advance_ratio;
    let tip_mach = op.tip_mach(rotor.radius);
    let x0 = rotor.root_cutout;
    let dx = (1.0 - x0) / cfg.n_radial as f64;
    let dpsi = 2.0 * PI / cfg.n_azimuth as f64;

    let mut acc = Integrals::default();
    let mut disk_area_int = 0.0; // ∫∫ x dx dψ over the whole disk (= π)
    let mut reverse_area_int = 0.0;

    for j in 0..cfg.n_azimuth {
        let psi = (j as f64 + 0.5) * dpsi;
        let (sin_psi, cos_psi) = psi.sin_cos();
        let mut ct_psi = 0.0;

        for i in 0..cfg.n_radial {
            let x = x0 + (i as f64 + 0.5) * dx;
            disk_area_int += x * dx * dpsi;

            let u_t = x + mu * sin_psi;
            if u_t <= 0.0 {
                // Reverse-flow region: null the element (first-cut), record area.
                reverse_area_int += x * dx * dpsi;
                if cfg.null_reverse_flow {
                    continue;
                }
            }
            let u_p = lambda;
            let u2 = u_t * u_t + u_p * u_p;
            let phi = u_p.atan2(u_t);
            let alpha = rotor.pitch(x) - phi;
            let mach = tip_mach * u2.sqrt();
            let (cl, cd) = airfoil.cl_cd(alpha, mach);

            let sigma = rotor.local_solidity(x);
            let (s, c) = (phi.sin(), phi.cos());
            let f_thrust = cl * c - cd * s; // out-of-plane (thrust) direction
            let f_inplane = cl * s + cd * c; // in-plane (torque) direction
            let w = 0.5 * sigma * u2;

            let dct = w * f_thrust * dx;
            ct_psi += dct;
            acc.cp += w * f_inplane * x * dx;
            acc.c_roll += dct * x * sin_psi;
            acc.c_pitch += dct * x * cos_psi;
        }

        acc.ct += ct_psi;
        if sin_psi > 0.0 {
            acc.advancing_ct += ct_psi;
        } else {
            acc.retreating_ct += ct_psi;
        }
    }

    // Azimuth average: divide by the number of stations.
    let n = cfg.n_azimuth as f64;
    acc.ct /= n;
    acc.cp /= n;
    acc.c_roll /= n;
    acc.c_pitch /= n;
    // Advancing/retreating halves each cover n/2 stations.
    acc.advancing_ct /= n / 2.0;
    acc.retreating_ct /= n / 2.0;
    acc.reverse_fraction = if disk_area_int > 0.0 {
        reverse_area_int / disk_area_int
    } else {
        0.0
    };
    acc
}

/// Solve forward-flight BEMT for `rotor` at operating point `op`, sectional aero
/// `airfoil`, and forward-flight condition `cond`.
pub fn solve_forward(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    cond: &ForwardCondition,
    cfg: &ForwardConfig,
) -> ForwardSolution {
    let mu = cond.advance_ratio;
    let offset = mu * cond.disk_aoa.tan(); // λ floor for λ_i ≥ 0

    // Residual: blade-element C_T minus momentum C_T. Decreasing in λ.
    let residual = |lam: f64| {
        integrate(lam, rotor, op, airfoil, cond, cfg).ct - momentum_ct(lam, mu, cond.disk_aoa)
    };

    // Bracket the root. At the floor, momentum thrust is ~0, so residual > 0
    // when the rotor produces lift.
    let lo0 = offset + 1e-9;
    let mut lo = lo0;
    let mut hi = offset + 0.02;
    if residual(lo) <= 0.0 {
        // No net thrust at this collective/condition.
        return assemble(lo, rotor, op, airfoil, cond, cfg);
    }
    let mut steps = 0;
    while residual(hi) > 0.0 && hi < offset + 1.0 {
        hi += (hi - offset) * 0.5 + 0.02;
        steps += 1;
        if steps > 80 {
            break;
        }
    }

    let mut lambda = 0.5 * (lo + hi);
    for _ in 0..cfg.max_iter {
        lambda = 0.5 * (lo + hi);
        let f = residual(lambda);
        if f.abs() < cfg.tol || (hi - lo) < cfg.tol {
            break;
        }
        if f > 0.0 {
            lo = lambda;
        } else {
            hi = lambda;
        }
    }

    assemble(lambda, rotor, op, airfoil, cond, cfg)
}

/// Run the final integral at the converged inflow and package the solution.
fn assemble(
    lambda: f64,
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    cond: &ForwardCondition,
    cfg: &ForwardConfig,
) -> ForwardSolution {
    let r = integrate(lambda, rotor, op, airfoil, cond, cfg);
    let lambda_i = lambda - cond.advance_ratio * cond.disk_aoa.tan();
    let cp_induced = lambda_i * r.ct;
    ForwardSolution {
        mu: cond.advance_ratio,
        lambda,
        lambda_i,
        ct: r.ct,
        cp: r.cp,
        cp_induced,
        cp_profile: r.cp - cp_induced,
        c_roll: r.c_roll,
        c_pitch: r.c_pitch,
        advancing_ct: r.advancing_ct,
        retreating_ct: r.retreating_ct,
        reverse_flow_fraction: r.reverse_fraction,
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
    fn hover_limit_is_axisymmetric() {
        let (rotor, op, af) = setup();
        let sol = solve_forward(
            &rotor,
            &op,
            &af,
            &ForwardCondition::hover(),
            &ForwardConfig::default(),
        );
        assert!(sol.ct > 0.0);
        // Uniform-inflow hover: λ ≈ √(C_T/2).
        assert!((sol.lambda - (sol.ct / 2.0).sqrt()).abs() < 1e-3);
        // No asymmetry in hover.
        assert!(sol.c_roll.abs() < 1e-6);
        assert!(sol.c_pitch.abs() < 1e-6);
        assert!(sol.reverse_flow_fraction < 1e-9);
    }

    #[test]
    fn forward_flight_produces_rolling_moment() {
        let (rotor, op, af) = setup();
        let cond = ForwardCondition::new(0.3, 0.0);
        let sol = solve_forward(&rotor, &op, &af, &cond, &ForwardConfig::default());
        // Advancing side carries more thrust → large rolling moment, |roll|≫|pitch|.
        assert!(sol.advancing_ct > sol.retreating_ct);
        assert!(sol.c_roll.abs() > 1e-4);
        assert!(sol.c_roll.abs() > 5.0 * sol.c_pitch.abs());
    }
}
