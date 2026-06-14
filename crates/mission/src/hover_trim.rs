//! Hover trim: find the collective pitch that makes rotor thrust equal weight.
//!
//! Thrust rises monotonically with collective (verified in the BEMT validation
//! suite), so the trim is a clean bisection on collective.

use helisim_airfoil::Airfoil;
use helisim_bemt::{Config, HoverSolution, solve_hover};
use helisim_rotor::{Operating, Rotor};

/// Find the collective (radians) at which the rotor produces `target_thrust`
/// newtons, and return it with the converged hover solution. Searches
/// collective in `[0, 25°]`. Returns `None` if even 25° cannot reach the target
/// (rotor/​RPM too small for the weight).
pub fn trim_hover_collective(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    target_thrust: f64,
    cfg: &Config,
) -> Option<(f64, HoverSolution)> {
    let thrust_at = |theta: f64| {
        let r = rotor.with_collective(theta);
        solve_hover(&r, op, airfoil, cfg).thrust
    };

    let lo = 0.0_f64;
    let hi = 25f64.to_radians();
    if thrust_at(hi) < target_thrust {
        return None; // cannot lift the weight even at max collective
    }

    let mut a = lo;
    let mut b = hi;
    for _ in 0..100 {
        let mid = 0.5 * (a + b);
        if thrust_at(mid) < target_thrust {
            a = mid;
        } else {
            b = mid;
        }
        if (b - a) < 1e-6 {
            break;
        }
    }
    let theta = 0.5 * (a + b);
    let sol = solve_hover(&rotor.with_collective(theta), op, airfoil, cfg);
    Some((theta, sol))
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_airfoil::LinearAirfoil;

    #[test]
    fn trim_hits_target_thrust() {
        let rotor = Rotor::rectangular(2, 0.6, 0.05, 0.0, 0.15);
        let op = Operating::from_rpm(2200.0);
        let af = LinearAirfoil::naca0012();
        let target = 35.0; // N
        let (theta, sol) = trim_hover_collective(&rotor, &op, &af, target, &Config::default())
            .expect("should trim");
        assert!((sol.thrust - target).abs() / target < 0.01);
        assert!(theta > 0.0 && theta < 25f64.to_radians());
    }

    #[test]
    fn impossible_target_returns_none() {
        let rotor = Rotor::rectangular(2, 0.6, 0.05, 0.0, 0.15);
        let op = Operating::from_rpm(2200.0);
        let af = LinearAirfoil::naca0012();
        assert!(trim_hover_collective(&rotor, &op, &af, 1.0e6, &Config::default()).is_none());
    }
}
