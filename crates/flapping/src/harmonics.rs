//! Harmonic-balance assembly of the flap system.
//!
//! Projecting `β'' + ν²β = (γ/2) M(ψ)` onto {1, cosψ, sinψ}, with
//! `β = β₀ − β₁c cosψ − β₁s sinψ`, gives (since `β''+ν²β = ν²β₀ +
//! (1−ν²)β₁c cosψ + (1−ν²)β₁s sinψ`):
//!
//! ```text
//! ν²β₀        = (γ/2) M₀(β)
//! (1−ν²)β₁c   = (γ/2) M_c(β)
//! (1−ν²)β₁s   = (γ/2) M_s(β)
//! ```
//!
//! The flap moment `M = ∫₀¹ x (u_T² θ − u_T u_P) dx` is linear in the flap
//! coefficients through `u_P = λ + x β' + μ β cosψ`, so its harmonics split into
//! a **forcing** part `F` (from θ and λ, with β = 0) and a **response** matrix
//! `G` (the β-dependence — the aerodynamic damping/stiffness). The system is then
//! `[diag(ν², 1−ν², 1−ν²) − (γ/2) G] b = (γ/2) F`.
//!
//! `F` and `G` are computed by numerically projecting the moment onto the three
//! harmonics — first-principles, reusing the same azimuth×radius integration as
//! the rest of the project.

use crate::config::FlapConfig;
use crate::controls::Controls;
use helisim_rotor::Rotor;
use std::f64::consts::PI;

/// The harmonic-balance pieces: forcing harmonics `F = [F₀, F_c, F_s]` and the
/// 3×3 response matrix `G` (rows = harmonics {0, c, s}, cols = {β₀, β₁c, β₁s}).
pub struct FlapSystem {
    /// Forcing harmonics (from θ and λ, β = 0).
    pub forcing: [f64; 3],
    /// Response matrix (β-dependence of the moment harmonics).
    pub response: [[f64; 3]; 3],
}

/// Assemble the forcing vector and response matrix for the flap moment at advance
/// ratio `mu`, inflow `lambda`, with cyclic `controls`. The radial integral runs
/// 0→1 (root cutout neglected) and the reverse-flow region is not nulled — both
/// to match the analytic closed-form oracle.
pub fn build_system(
    rotor: &Rotor,
    controls: &Controls,
    mu: f64,
    lambda: f64,
    cfg: &FlapConfig,
) -> FlapSystem {
    let na = cfg.n_azimuth;
    let nr = cfg.n_radial;
    let dx = 1.0 / nr as f64;

    let mut f = [0.0_f64; 3];
    let mut g = [[0.0_f64; 3]; 3];

    for j in 0..na {
        let psi = 2.0 * PI * (j as f64 + 0.5) / na as f64;
        let (sp, cp) = psi.sin_cos();

        // Forcing moment (β = 0): M = ∫ x (u_T² θ − u_T λ) dx.
        let mut mf = 0.0;
        for i in 0..nr {
            let x = (i as f64 + 0.5) * dx;
            let u_t = x + mu * sp;
            let theta = rotor.pitch(x) + controls.theta_1c * cp + controls.theta_1s * sp;
            mf += x * (u_t * u_t * theta - u_t * lambda) * dx;
        }
        f[0] += mf;
        f[1] += mf * cp;
        f[2] += mf * sp;

        // Response basis: (β, β') as the flap coefficients sweep unit values.
        //   β₀ basis: β=1,        β'=0
        //   β₁c basis: β=−cosψ,   β'=+sinψ
        //   β₁s basis: β=−sinψ,   β'=−cosψ
        let basis = [(1.0, 0.0), (-cp, sp), (-sp, -cp)];
        for (k, &(b, bd)) in basis.iter().enumerate() {
            let mut mk = 0.0;
            for i in 0..nr {
                let x = (i as f64 + 0.5) * dx;
                let u_t = x + mu * sp;
                let u_pb = x * bd + mu * b * cp; // β-induced through-disk velocity
                mk += x * (-u_t * u_pb) * dx;
            }
            g[0][k] += mk;
            g[1][k] += mk * cp;
            g[2][k] += mk * sp;
        }
    }

    // Harmonic projections: mean for the constant, 2/N·Σ for cos/sin.
    let n = na as f64;
    f[0] /= n;
    f[1] *= 2.0 / n;
    f[2] *= 2.0 / n;
    for (row, &scale) in g.iter_mut().zip([1.0 / n, 2.0 / n, 2.0 / n].iter()) {
        for v in row.iter_mut() {
            *v *= scale;
        }
    }

    FlapSystem {
        forcing: f,
        response: g,
    }
}
