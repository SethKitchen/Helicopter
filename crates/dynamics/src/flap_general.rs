//! First-harmonic rigid-blade flapping under general flow — the harmonic-balance
//! 3×3 solve, generalized to arbitrary in-plane velocity `(μ_u, μ_v)`, body rates
//! `(p̄, q̄)`, and a **linear inflow** `[λ₀, λ₁s, λ₁c]`. Uniform inflow `[λ₀,0,0]`
//! recovers the pre-5h behaviour exactly. Split out of `full_aero` so each file
//! holds one concept; consumed by `full_aero::assemble_forces`.

use helisim_flapping::solve3;
use std::f64::consts::PI;

use crate::context::RotorAero;
use crate::full_aero::{N_AZ, N_R};

/// First-harmonic flap coefficients `(β₀, β₁c, β₁s)` for the given flow
/// `mu = [μ_u, μ_v]`, rates `rates = [p̄, q̄]`, and linear inflow
/// `inflow = [λ₀, λ₁s, λ₁c]` → `λ(x,ψ)=λ₀+λ₁c x cosψ+λ₁s x sinψ`.
pub(crate) fn flap_coeffs(
    aero: &RotorAero,
    mu: [f64; 2],
    inflow: [f64; 3],
    rates: [f64; 2],
) -> (f64, f64, f64) {
    let (rotor, controls, props) = (aero.rotor, aero.controls, aero.props);
    let [mu_u, mu_v] = mu;
    let [p_bar, q_bar] = rates;
    let dx = 1.0 / N_R as f64;
    let dpsi = 2.0 * PI / N_AZ as f64;
    let mut f = [0.0_f64; 3];
    let mut g = [[0.0_f64; 3]; 3];

    for j in 0..N_AZ {
        let psi = (j as f64 + 0.5) * dpsi;
        let (sp, cp) = psi.sin_cos();
        let u_t = |x: f64| x + mu_u * sp - mu_v * cp;
        let mut mf = 0.0;
        for i in 0..N_R {
            let x = (i as f64 + 0.5) * dx;
            let ut = u_t(x);
            let theta = rotor.pitch(x) + controls.theta_1c * cp + controls.theta_1s * sp;
            let lam = inflow[0] + inflow[2] * x * cp + inflow[1] * x * sp;
            let u_pf = lam + p_bar * x * sp - q_bar * x * cp; // β-independent forcing
            mf += x * (ut * ut * theta - ut * u_pf) * dx;
        }
        f[0] += mf;
        f[1] += mf * cp;
        f[2] += mf * sp;

        // Basis (β-dependent part of u_P): flap rate + coning×in-plane-flow.
        let basis = [(1.0, 0.0), (-cp, sp), (-sp, -cp)];
        for (k, &(b, bd)) in basis.iter().enumerate() {
            let mut mk = 0.0;
            for i in 0..N_R {
                let x = (i as f64 + 0.5) * dx;
                let ut = u_t(x);
                let u_pb = x * bd + b * (mu_u * cp + mu_v * sp);
                mk += x * (-ut * u_pb) * dx;
            }
            g[0][k] += mk;
            g[1][k] += mk * cp;
            g[2][k] += mk * sp;
        }
    }
    let n = N_AZ as f64;
    f[0] /= n;
    f[1] *= 2.0 / n;
    f[2] *= 2.0 / n;
    for (row, &sc) in g.iter_mut().zip([1.0 / n, 2.0 / n, 2.0 / n].iter()) {
        for v in row.iter_mut() {
            *v *= sc;
        }
    }

    let nu2 = props.nu_beta_sq();
    let gamma = props.lock_number;
    let diag = [nu2, 1.0 - nu2, 1.0 - nu2];
    let mut a = [[0.0_f64; 3]; 3];
    for (r, (a_row, g_row)) in a.iter_mut().zip(g.iter()).enumerate() {
        for (a_rc, g_rc) in a_row.iter_mut().zip(g_row.iter()) {
            *a_rc = -0.5 * gamma * g_rc;
        }
        a_row[r] += diag[r];
    }
    // Gyroscopic ("rotor-follows-shaft") hub-rate→flap coupling (see aero.rs):
    // q̄·sinψ → rhs[2] (→β1c, pitch damping),
    // p̄·cosψ → rhs[1] (→β1s, roll damping). Same coefficient/sign as the
    // longitudinal path, so the 5f rotation symmetry (Lp = Mq) is preserved.
    let mut rhs = [0.5 * gamma * f[0], 0.5 * gamma * f[1], 0.5 * gamma * f[2]];
    rhs[2] += props.gyro_rate * q_bar;
    rhs[1] += props.gyro_rate * p_bar;
    let b = solve3(a, rhs);
    (b[0], b[1], b[2])
}
