//! Pitt–Peters three-state finite-state dynamic inflow.
//!
//! The rotor inflow `λ(x,ψ) = λ₀ + λ₁s·x·sinψ + λ₁c·x·cosψ` is given its own
//! dynamics: the three states `ν = [λ₀, λ₁s, λ₁c]` evolve via
//!
//! ```text
//! [M] dν/dτ + [L]⁻¹ ν = C ,   τ = Ωt ,   C = [C_T, C_roll, C_pitch]
//! ```
//!
//! with the Pitt–Peters apparent-mass matrix `[M]` and gain matrix `[L]`. As the
//! apparent mass → 0 the inflow snaps to its steady value `ν = [L]·C` — the
//! quasi-static inflow used through 5g (the τ→0 gate). The lag of the cyclic
//! states `λ₁s, λ₁c` is what shifts the off-axis (pitch–roll) coupling — the
//! documented dynamic-inflow signature.
//!
//! **Architecture shift:** the inflow leaves the inner fixed-point and becomes
//! integrated state — three extra states per rotor in the EOM.

use helisim_flapping::solve3;
use std::f64::consts::PI;

/// Apparent-mass diagonal `[M₀, M₁s, M₁c]` (Pitt–Peters). The negative cyclic
/// entries pair with the negative cyclic gains so the states decay to steady.
pub fn apparent_mass() -> [f64; 3] {
    [8.0 / (3.0 * PI), -16.0 / (45.0 * PI), -16.0 / (45.0 * PI)]
}

/// Pitt–Peters steady gain matrix `[L]` (the `1/V` folded in), so steady
/// `ν = [L]·C`. `mu` is advance ratio, `lambda` the mean inflow magnitude.
/// Hover (`χ=0`): `diag(0.5, -2, -2)/V`, recovering the Glauert mean (λ₀=C_T/2V).
pub fn l_matrix(mu: f64, lambda: f64) -> [[f64; 3]; 3] {
    let v = (mu * mu + lambda * lambda).sqrt().max(1e-4);
    let chi = mu.atan2(lambda.abs().max(1e-6)); // wake skew angle (0 hover → π/2 fast)
    let t = (0.5 * chi).tan();
    let cc = chi.cos();
    let k = 15.0 * PI / 64.0;
    let iv = 1.0 / v;
    [
        [0.5 * iv, 0.0, -k * t * iv],
        [0.0, -4.0 / (1.0 + cc) * iv, 0.0],
        [k * t * iv, 0.0, -4.0 * cc / (1.0 + cc) * iv],
    ]
}

fn matvec(a: [[f64; 3]; 3], x: [f64; 3]) -> [f64; 3] {
    [
        a[0][0] * x[0] + a[0][1] * x[1] + a[0][2] * x[2],
        a[1][0] * x[0] + a[1][1] * x[1] + a[1][2] * x[2],
        a[2][0] * x[0] + a[2][1] * x[1] + a[2][2] * x[2],
    ]
}

/// Steady inflow for a given aerodynamic forcing `C = [C_T, C_roll, C_pitch]`:
/// `ν = [L]·C`. (The quasi-static inflow; the τ→0 limit of the dynamics.)
pub fn steady_inflow_for(c: [f64; 3], mu: f64, lambda: f64) -> [f64; 3] {
    matvec(l_matrix(mu, lambda), c)
}

/// Time derivative `dν/dt` of the Pitt–Peters inflow states (real time), with an
/// `lag` scale on the apparent mass (1 = Pitt–Peters; →0 reproduces quasi-static
/// by making the states snap to steady — the falsifiable τ→0 gate).
pub fn inflow_derivative(
    nu: [f64; 3],
    c: [f64; 3],
    mu: f64,
    lambda: f64,
    omega: f64,
    lag: f64,
) -> [f64; 3] {
    // g = [L]⁻¹ ν  (solve L g = ν).
    let g = solve3(l_matrix(mu, lambda), nu);
    let m = apparent_mass();
    let lag = lag.max(1e-9);
    [
        omega * (c[0] - g[0]) / (lag * m[0]),
        omega * (c[1] - g[1]) / (lag * m[1]),
        omega * (c[2] - g[2]) / (lag * m[2]),
    ]
}

/// Gravest-mode inflow time constant, seconds: `τ = M₀·(L⁻¹)₀₀⁻¹ / Ω` at hover,
/// i.e. `(8/3π)/(2·λ·Ω)` — an O(1-rev) lag, checkable against the literature.
pub fn gravest_time_constant(lambda: f64, omega: f64) -> f64 {
    let v = lambda.abs().max(1e-4);
    apparent_mass()[0] / (2.0 * v * omega)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hover_steady_matches_glauert() {
        // At hover, C_roll=C_pitch=0 → λ₁s=λ₁c=0; and λ₀ = C_T/(2λ₀) ⇒ √(C_T/2).
        let lambda = 0.05;
        let ct = 2.0 * lambda * lambda; // momentum hover
        let nu = steady_inflow_for([ct, 0.0, 0.0], 0.0, lambda);
        assert!(
            (nu[0] - lambda).abs() < 1e-9,
            "λ₀ {} should equal {}",
            nu[0],
            lambda
        );
        assert!(
            nu[1].abs() < 1e-12 && nu[2].abs() < 1e-12,
            "no cyclic inflow at hover"
        );
    }

    /// Numeric oracle for the **off-axis wake-skew coupling** the hover test can't
    /// see (at μ=0 it vanishes). In forward flight the wake skews (χ>0) and the
    /// Pitt–Peters `[L]` couples `λ₀ ↔ λ₁c` through `±(15π/64)·tan(χ/2)/V`. We check
    /// (a) the matrix entries match that closed form, and (b) the *behaviour* it
    /// produces: a pure pitch-moment forcing drives a non-zero coning inflow `λ₀`
    /// at μ>0 but exactly zero at μ=0.
    #[test]
    fn off_axis_wake_skew_coupling_matches_pitt_peters() {
        let (mu, lambda) = (0.2_f64, 0.05_f64);
        let v = (mu * mu + lambda * lambda).sqrt();
        let chi = mu.atan2(lambda);
        let coupling = (15.0 * PI / 64.0) * (0.5 * chi).tan() / v; // the analytic term
        assert!(coupling > 0.1, "coupling is substantial at μ=0.2");

        let l = l_matrix(mu, lambda);
        assert!((l[2][0] - coupling).abs() < 1e-9, "λ₀→λ₁c coupling L[2][0]");
        assert!(
            (l[0][2] + coupling).abs() < 1e-9,
            "λ₁c→λ₀ coupling (opposite sign)"
        );

        // Behaviour: a pure pitch moment couples into the coning inflow λ₀ at μ>0…
        let nu_fwd = steady_inflow_for([0.0, 0.0, 1e-3], mu, lambda);
        assert!((nu_fwd[0] - (-coupling * 1e-3)).abs() < 1e-12 && nu_fwd[0] != 0.0);
        // …but not at hover, where the wake is unskewed.
        let nu_hover = steady_inflow_for([0.0, 0.0, 1e-3], 0.0, lambda);
        assert!(nu_hover[0].abs() < 1e-15);
    }

    #[test]
    fn steady_is_a_fixed_point_of_the_dynamics() {
        let (mu, lambda, omega) = (0.1, 0.05, 150.0);
        let c = [0.006, 0.0005, -0.0003];
        let nu = steady_inflow_for(c, mu, lambda);
        let d = inflow_derivative(nu, c, mu, lambda, omega, 1.0);
        for v in d {
            assert!(
                v.abs() < 1e-6,
                "steady inflow should have ~zero derivative, got {v}"
            );
        }
    }

    #[test]
    fn time_constant_is_order_one_rev() {
        // τ ≈ (8/3π)/(2λΩ); for λ≈0.06, Ω≈157 → ~0.045 s ≈ ~1 rev — physical.
        let tau = gravest_time_constant(0.06, 157.0);
        let rev = 2.0 * PI / 157.0;
        assert!(
            tau > 0.2 * rev && tau < 3.0 * rev,
            "τ {tau:.4}s vs rev {rev:.4}s"
        );
    }
}
