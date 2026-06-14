//! The hover BEMT solve: per-station inflow root-find + spanwise integration.

use crate::config::Config;
use crate::solution::HoverSolution;
use crate::station::Station;
use crate::tip_loss::prandtl_tip_loss;
use helisim_airfoil::Airfoil;
use helisim_rotor::{Operating, Rotor};

/// Section state evaluated at a trial inflow, returned alongside the
/// thrust-balance residual so the converged call avoids recomputation.
struct SectionEval {
    residual: f64,
    phi: f64,
    alpha: f64,
    cl: f64,
    cd: f64,
    tip_loss: f64,
}

/// Evaluate the thrust-balance residual `blade_element_dCT - momentum_dCT` and
/// the section state at station `x` for a trial inflow `lambda`.
fn evaluate(
    x: f64,
    lambda: f64,
    rotor: &Rotor,
    airfoil: &dyn Airfoil,
    tip_mach: f64,
    use_tip_loss: bool,
) -> SectionEval {
    let phi = lambda.atan2(x);
    let alpha = rotor.pitch(x) - phi;
    let u2 = x * x + lambda * lambda; // (U / Omega R)^2
    let mach = tip_mach * u2.sqrt();
    let (cl, cd) = airfoil.cl_cd(alpha, mach);

    let sigma = rotor.local_solidity(x);
    let be = 0.5 * sigma * u2 * (cl * phi.cos() - cd * phi.sin());

    let tip_loss = if use_tip_loss {
        prandtl_tip_loss(x, phi, rotor.n_blades)
    } else {
        1.0
    };
    let mom = 4.0 * tip_loss * lambda * lambda * x;

    SectionEval {
        residual: be - mom,
        phi,
        alpha,
        cl,
        cd,
        tip_loss,
    }
}

/// Solve for the converged inflow at one station by bisection on `lambda`.
fn solve_station(
    x: f64,
    rotor: &Rotor,
    airfoil: &dyn Airfoil,
    tip_mach: f64,
    cfg: &Config,
) -> Station {
    let eval = |lam: f64| evaluate(x, lam, rotor, airfoil, tip_mach, cfg.tip_loss);

    // At lambda -> 0 the momentum term vanishes; the residual is positive
    // whenever the section produces lift. If it is already non-positive the
    // section makes no net thrust here, so inflow is effectively zero.
    let lo = 1e-9;
    let lo_eval = eval(lo);
    if lo_eval.residual <= 0.0 {
        return Station::assemble(
            x,
            lo,
            lo_eval.phi,
            lo_eval.alpha,
            lo_eval.cl,
            lo_eval.cd,
            lo_eval.tip_loss,
            rotor,
        );
    }

    // Grow the upper bracket until the residual turns non-positive.
    let mut hi = 0.02_f64;
    let mut steps = 0;
    while eval(hi).residual > 0.0 && hi < 1.0 {
        hi *= 1.5;
        steps += 1;
        if steps > 60 {
            break;
        }
    }

    // Bisection on the monotone (decreasing-in-lambda) residual.
    let mut a = lo;
    let mut b = hi;
    let mut lambda = 0.5 * (a + b);
    for _ in 0..cfg.max_iter {
        lambda = 0.5 * (a + b);
        let e = eval(lambda);
        if e.residual.abs() < cfg.tol || (b - a) < cfg.tol {
            break;
        }
        if e.residual > 0.0 {
            a = lambda;
        } else {
            b = lambda;
        }
    }

    let e = eval(lambda);
    Station::assemble(x, lambda, e.phi, e.alpha, e.cl, e.cd, e.tip_loss, rotor)
}

/// Run the hover BEMT solve for `rotor` at operating point `op` with sectional
/// aerodynamics `airfoil`.
pub fn solve_hover(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    cfg: &Config,
) -> HoverSolution {
    assert!(cfg.n_stations >= 2, "need at least 2 stations");
    let tip_mach = op.tip_mach(rotor.radius);

    // Stations from the root cutout to the tip. The exact tip is nudged inward
    // to avoid the identically-zero tip-loss factor there.
    let x0 = rotor.root_cutout;
    let x1 = 1.0;
    let n = cfg.n_stations;
    let mut stations = Vec::with_capacity(n);
    for i in 0..n {
        let frac = i as f64 / (n - 1) as f64;
        let x = (x0 + (x1 - x0) * frac).clamp(x0, 1.0 - 1e-9);
        stations.push(solve_station(x, rotor, airfoil, tip_mach, cfg));
    }

    // Trapezoidal integration of the differential coefficients over the span.
    let mut ct = 0.0;
    let mut cp = 0.0;
    for w in stations.windows(2) {
        let dx = w[1].x - w[0].x;
        ct += 0.5 * (w[0].dct_dx + w[1].dct_dx) * dx;
        cp += 0.5 * (w[0].dcp_dx + w[1].dcp_dx) * dx;
    }

    let figure_of_merit = if ct > 0.0 && cp > 0.0 {
        ct.powf(1.5) / (2.0_f64.sqrt() * cp)
    } else {
        0.0
    };

    // Dimensionalise: q = rho A (Omega R)^2.
    let vt = op.tip_speed(rotor.radius);
    let q = op.rho * rotor.disk_area() * vt * vt;
    let thrust = ct * q;
    let power = cp * q * vt;
    let torque = power / op.omega;

    HoverSolution {
        ct,
        cp,
        figure_of_merit,
        thrust,
        torque,
        power,
        stations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_airfoil::LinearAirfoil;

    #[test]
    fn positive_thrust_and_sane_fm() {
        let rotor = Rotor::rectangular(2, 1.143, 0.191, 8f64.to_radians(), 0.2);
        let op = Operating::from_tip_mach(0.439, 1.143);
        let af = LinearAirfoil::naca0012();
        let sol = solve_hover(&rotor, &op, &af, &Config::default());
        assert!(sol.ct > 0.0);
        assert!(sol.cp > 0.0);
        assert!(sol.figure_of_merit > 0.0 && sol.figure_of_merit < 1.0);
    }

    #[test]
    fn thrust_increases_with_collective() {
        let op = Operating::from_tip_mach(0.439, 1.143);
        let af = LinearAirfoil::naca0012();
        let cfg = Config::default();
        let lo = solve_hover(
            &Rotor::rectangular(2, 1.143, 0.191, 5f64.to_radians(), 0.2),
            &op,
            &af,
            &cfg,
        );
        let hi = solve_hover(
            &Rotor::rectangular(2, 1.143, 0.191, 12f64.to_radians(), 0.2),
            &op,
            &af,
            &cfg,
        );
        assert!(hi.ct > lo.ct);
    }
}
