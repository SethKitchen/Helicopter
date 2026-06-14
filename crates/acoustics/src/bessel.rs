//! Integer-order Bessel function of the first kind `J_n(x)` — std-only.
//!
//! Gutin's rotational-noise formula ([`crate::rotational`]) evaluates `J_{mB}` at
//! arguments up to roughly `m B M_tip` (here ≲ 10) for orders up to a few tens.
//! The project forbids external crates, so we compute it from the ascending power
//! series
//!
//! `J_n(x) = Σ_{k≥0} (-1)^k / (k! (n+k)!) · (x/2)^{2k+n}`,
//!
//! evaluated by the stable term ratio `t_k / t_{k-1} = -(x/2)² / (k (n+k))` so no
//! factorial or power is ever formed explicitly. For the orders and arguments the
//! acoustics needs this converges in a few dozen terms with negligible
//! cancellation; the test module checks it against tabulated zeros and extrema.

/// Bessel function of the first kind, integer order `n ≥ 0`, real argument `x`.
pub fn bessel_j(n: usize, x: f64) -> f64 {
    if x == 0.0 {
        return if n == 0 { 1.0 } else { 0.0 };
    }
    let half = x / 2.0;
    let half_sq = half * half;

    // Leading term t_0 = (x/2)^n / n!, built by ratio to avoid overflow.
    let mut term = 1.0;
    for k in 1..=n {
        term *= half / k as f64;
    }
    let mut sum = term;
    for k in 1..=100 {
        term *= -half_sq / ((k as f64) * ((n + k) as f64));
        sum += term;
        if term.abs() <= 1e-18 * sum.abs().max(1e-300) {
            break;
        }
    }
    sum
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `J_n(0) = δ_{n0}`.
    #[test]
    fn value_at_zero() {
        assert!((bessel_j(0, 0.0) - 1.0).abs() < 1e-15);
        assert!(bessel_j(1, 0.0).abs() < 1e-15);
        assert!(bessel_j(5, 0.0).abs() < 1e-15);
    }

    /// First few zeros of `J_0` and `J_1` (Abramowitz & Stegun Table 9.5).
    #[test]
    fn known_zeros() {
        assert!(bessel_j(0, 2.404_825_557_695_773).abs() < 1e-9);
        assert!(bessel_j(0, 5.520_078_110_286_311).abs() < 1e-9);
        assert!(bessel_j(1, 3.831_705_970_207_512).abs() < 1e-9);
    }

    /// Tabulated values (A&S Table 9.1): `J_0(1)=0.7651976866`,
    /// `J_1(1)=0.4400505857`, and the `J_1` maximum `0.5818652` at `x=1.8411838`.
    #[test]
    fn known_values() {
        assert!((bessel_j(0, 1.0) - 0.765_197_686_6).abs() < 1e-9);
        assert!((bessel_j(1, 1.0) - 0.440_050_585_7).abs() < 1e-9);
        assert!((bessel_j(1, 1.841_183_8) - 0.581_865_2).abs() < 1e-6);
        // A higher order, J_5(5) = 0.2611405461 (A&S Table 9.1).
        assert!((bessel_j(5, 5.0) - 0.261_140_546_1).abs() < 1e-8);
    }

    /// Recurrence `J_{n-1}(x) + J_{n+1}(x) = (2n/x) J_n(x)` — independent check.
    #[test]
    fn satisfies_recurrence() {
        let x = 4.3;
        for n in 1..10 {
            let lhs = bessel_j(n - 1, x) + bessel_j(n + 1, x);
            let rhs = (2.0 * n as f64 / x) * bessel_j(n, x);
            assert!((lhs - rhs).abs() < 1e-10, "n={n}: {lhs} vs {rhs}");
        }
    }
}
