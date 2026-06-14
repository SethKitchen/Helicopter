//! Prandtl tip-loss factor.

use std::f64::consts::PI;

/// Prandtl tip-loss factor `F` at station `x` for an `n_blades` rotor with local
/// inflow angle `phi`.
///
/// `F = (2/pi) arccos(exp(-f))` with `f = (B/2)(1 - x)/(x sin phi)`. As `x -> 1`,
/// `f -> 0` and `F -> 0`, driving the loading to zero at the tip. Guarded against
/// the `phi -> 0` singularity by flooring `sin phi`.
pub fn prandtl_tip_loss(x: f64, phi: f64, n_blades: usize) -> f64 {
    let sin_phi = phi.sin().abs().max(1e-6);
    let f = 0.5 * n_blades as f64 * (1.0 - x) / (x * sin_phi);
    let e = (-f).exp().clamp(0.0, 1.0);
    (2.0 / PI) * e.acos()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn drops_toward_tip() {
        let f_mid = prandtl_tip_loss(0.5, 0.05, 2);
        let f_tip = prandtl_tip_loss(0.999, 0.05, 2);
        assert!(f_mid > 0.8);
        assert!(f_tip < 0.2);
        assert!(f_mid > f_tip);
    }

    #[test]
    fn bounded_unit_interval() {
        for &x in &[0.2, 0.5, 0.9, 0.99] {
            let f = prandtl_tip_loss(x, 0.06, 2);
            assert!((0.0..=1.0).contains(&f));
        }
    }
}
