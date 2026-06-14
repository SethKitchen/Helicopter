//! Glauert forward-flight momentum inflow.
//!
//! The momentum thrust of the tilted actuator disk is
//! `C_T = 2 λ_i √(μ² + λ²)`, where `λ` is the total inflow ratio normal to the
//! disk and `λ_i = λ − μ tanα` is its induced part. Inverting for `λ_i` at a
//! given `C_T` is the classic Glauert high-speed inflow relation.

/// Momentum-theory thrust coefficient for a trial total inflow `lambda` at
/// advance ratio `mu` and disk angle of attack `disk_aoa` (radians).
/// `C_T = 2 (λ − μ tanα) √(μ² + λ²)`.
pub fn momentum_ct(lambda: f64, mu: f64, disk_aoa: f64) -> f64 {
    let lambda_i = lambda - mu * disk_aoa.tan();
    2.0 * lambda_i * (mu * mu + lambda * lambda).sqrt()
}

/// Solve the Glauert relation for the total inflow `λ` that produces thrust `ct`
/// at advance ratio `mu` and disk tilt `disk_aoa`, by bisection on the monotone
/// residual `momentum_ct(λ) − ct`.
pub fn glauert_inflow(ct: f64, mu: f64, disk_aoa: f64, tol: f64, max_iter: usize) -> f64 {
    let offset = mu * disk_aoa.tan(); // λ must exceed this for λ_i > 0
    let mut lo = offset + 1e-12;
    let mut hi = offset + 1.0;
    // Grow the upper bracket until it overshoots ct.
    while momentum_ct(hi, mu, disk_aoa) < ct && hi < offset + 100.0 {
        hi *= 2.0;
    }
    for _ in 0..max_iter {
        let mid = 0.5 * (lo + hi);
        let f = momentum_ct(mid, mu, disk_aoa) - ct;
        if f.abs() < tol || (hi - lo) < tol {
            return mid;
        }
        if f < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// Analytic closed form for the induced inflow at a *level* disk (`α = 0`),
/// where `λ = λ_i` solves `λ_i⁴ + μ² λ_i² − (C_T/2)² = 0`:
/// `λ_i = √( (−μ² + √(μ⁴ + C_T²)) / 2 )`.
///
/// This is the non-circular anchor for the inflow solver, the forward-flight
/// analogue of the Leishman hover closed form. Hover limit (`μ→0`):
/// `λ_i → √(C_T/2)`; high-speed limit (`μ≫λ_i`): `λ_i → C_T/(2μ)`.
pub fn glauert_inflow_closed_form(ct: f64, mu: f64) -> f64 {
    let li2 = (-(mu * mu) + (mu.powi(4) + ct * ct).sqrt()) / 2.0;
    li2.max(0.0).sqrt()
}

/// Induced-power ratio `P_i(μ)/P_i(hover) = λ_i(μ)/λ_i(0)` at level disk — the
/// induced part of the forward-flight "power bucket". Decreases with speed.
pub fn induced_power_ratio(ct: f64, mu: f64) -> f64 {
    let hover = glauert_inflow_closed_form(ct, 0.0);
    if hover <= 0.0 {
        return 0.0;
    }
    glauert_inflow_closed_form(ct, mu) / hover
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solver_matches_closed_form_level_disk() {
        for &ct in &[0.004, 0.008, 0.012] {
            for &mu in &[0.0, 0.05, 0.15, 0.3, 0.5] {
                let solved = glauert_inflow(ct, mu, 0.0, 1e-12, 200);
                let exact = glauert_inflow_closed_form(ct, mu);
                assert!(
                    (solved - exact).abs() < 1e-6,
                    "ct={ct} mu={mu}: solved {solved} vs exact {exact}"
                );
            }
        }
    }

    #[test]
    fn hover_and_high_speed_limits() {
        let ct = 0.008;
        // Hover: λ_i = √(C_T/2).
        assert!((glauert_inflow_closed_form(ct, 0.0) - (ct / 2.0).sqrt()).abs() < 1e-9);
        // High speed: λ_i → C_T/(2μ).
        let mu = 2.0;
        assert!((glauert_inflow_closed_form(ct, mu) - ct / (2.0 * mu)).abs() < 1e-4);
    }

    #[test]
    fn induced_power_decreases_with_speed() {
        let ct = 0.008;
        let r0 = induced_power_ratio(ct, 0.0);
        let r1 = induced_power_ratio(ct, 0.1);
        let r2 = induced_power_ratio(ct, 0.3);
        assert!((r0 - 1.0).abs() < 1e-9);
        assert!(r1 < r0 && r2 < r1);
    }

    #[test]
    fn high_thrust_grows_the_bracket_and_zero_thrust_has_no_induced_power() {
        // A large C_T at low μ forces the bracket-growth loop (initial hi < ct).
        let solved = glauert_inflow(2.0, 0.05, 0.0, 1e-10, 200);
        assert!((momentum_ct(solved, 0.05, 0.0) - 2.0).abs() < 1e-4);
        // Zero thrust → zero hover induced velocity → induced-power ratio is 0.
        assert_eq!(induced_power_ratio(0.0, 0.1), 0.0);
    }
}
