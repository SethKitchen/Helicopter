//! Analytic first-harmonic flapping coefficients — the validation oracle.
//!
//! Classic closed form for a central-hinge (`ν_β = 1`), untwisted, constant-chord
//! articulated rotor with uniform inflow and no cyclic pitch (Leishman / Johnson
//! / Prouty). `θ` is the collective (rad), `λ` the inflow ratio, `μ` the advance
//! ratio, `γ` the Lock number.
//!
//! Convention: `β = β₀ − β₁c cosψ − β₁s sinψ`, with `ψ = 0` downstream (over the
//! tail) increasing toward the advancing side. In this convention `β₁c > 0` is
//! rearward tip-path-plane tilt — the forward-flight "blow-back" — so the
//! longitudinal/lateral coefficients are positive (some texts that place `ψ = 0`
//! upstream carry the opposite sign).

/// `(β₀, β₁c, β₁s)` from the algebraic closed form.
pub fn closed_form_coefficients(theta: f64, lambda: f64, mu: f64, lock: f64) -> (f64, f64, f64) {
    let mu2 = mu * mu;
    // Coning.
    let beta0 = lock * (theta * (1.0 + mu2) / 8.0 - lambda / 6.0);
    // Longitudinal flapping (the dominant rearward "blow-back" response).
    let beta1c = 2.0 * mu * (4.0 * theta / 3.0 - lambda) / (1.0 - 0.5 * mu2);
    // Lateral flapping (driven by coning).
    let beta1s = (4.0 / 3.0) * mu * beta0 / (1.0 + 0.5 * mu2);
    (beta0, beta1c, beta1s)
}
