//! Perturbable main-rotor longitudinal forces and moments.
//!
//! Computes the body-axis X (longitudinal), Z (vertical) forces and M (pitch)
//! moment of the main rotor as a function of the body state (u, w, q), with the
//! controls held at trim. The body state enters the rotor through:
//!
//! * `u`  → advance ratio `μ = u/(ΩR)` (edgewise flow).
//! * `w`  → normal inflow `λ_c = −w/(ΩR)` (heave: descent reduces inflow).
//! * `q`  → a 1/rev velocity term `−q̄·x·cosψ` in `u_P` (pitch-rate flap response).
//!
//! Inflow and flapping are converged together (the 5b coupling), and the flap
//! harmonic balance carries the cyclic and the pitch-rate forcing. This is the
//! function the stability derivatives differentiate.

use helisim_airfoil::Airfoil;
use helisim_flapping::{Controls, FlapProperties, solve3};
use helisim_rotor::{Operating, Rotor};
use std::f64::consts::PI;

const N_AZ: usize = 36;
const N_R: usize = 30;

/// Main-rotor longitudinal loads in body axes (x fwd, z down).
#[derive(Clone, Copy, Debug)]
pub struct LongAero {
    /// Thrust magnitude (⟂ tip-path plane), N.
    pub thrust: f64,
    /// Body-x force, N.
    pub x_force: f64,
    /// Body-z force, N (negative = up).
    pub z_force: f64,
    /// Pitch moment about the CG, N·m (positive nose-up).
    pub pitch_moment: f64,
    /// Longitudinal flap β₁c, rad.
    pub beta1c: f64,
    /// Lateral flap β₁s, rad.
    pub beta1s: f64,
}

/// Induced inflow with climb: solve `λ_i = C_T / (2√(μ² + (λ_c+λ_i)²))`.
fn induced(ct: f64, mu: f64, lambda_c: f64) -> f64 {
    let mut li = 0.05_f64;
    for _ in 0..40 {
        let total = lambda_c + li;
        let denom = 2.0 * (mu * mu + total * total).sqrt();
        let new = if denom > 1e-9 { ct / denom } else { 0.0 };
        if (new - li).abs() < 1e-9 {
            li = new;
            break;
        }
        li = 0.5 * (li + new);
    }
    li.clamp(-0.5, 0.5)
}

/// First-harmonic flap coefficients (β0, β1c, β1s) by harmonic balance, with the
/// pitch-rate forcing `−q̄·x·cosψ` folded into the through-disk velocity.
fn flap_coeffs(
    rotor: &Rotor,
    controls: &Controls,
    mu: f64,
    lambda: f64,
    q_bar: f64,
    props: &FlapProperties,
) -> (f64, f64, f64) {
    let dx = 1.0 / N_R as f64;
    let dpsi = 2.0 * PI / N_AZ as f64;
    let mut f = [0.0_f64; 3];
    let mut g = [[0.0_f64; 3]; 3];

    for j in 0..N_AZ {
        let psi = (j as f64 + 0.5) * dpsi;
        let (sp, cp) = psi.sin_cos();
        let mut mf = 0.0;
        for i in 0..N_R {
            let x = (i as f64 + 0.5) * dx;
            let u_t = x + mu * sp;
            let theta = rotor.pitch(x) + controls.theta_1c * cp + controls.theta_1s * sp;
            let u_pf = lambda - q_bar * x * cp;
            mf += x * (u_t * u_t * theta - u_t * u_pf) * dx;
        }
        f[0] += mf;
        f[1] += mf * cp;
        f[2] += mf * sp;

        let basis = [(1.0, 0.0), (-cp, sp), (-sp, -cp)];
        for (k, &(b, bd)) in basis.iter().enumerate() {
            let mut mk = 0.0;
            for i in 0..N_R {
                let x = (i as f64 + 0.5) * dx;
                let u_t = x + mu * sp;
                let u_pb = x * bd + mu * b * cp;
                mk += x * (-u_t * u_pb) * dx;
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
    // Gyroscopic ("rotor-follows-shaft") coupling of hub pitch rate into flap: the
    // q̄·sinψ inertial forcing (from Ω_f×(Ω_f×r), the spin–hub-rate cross term) that
    // feeds the IN-PHASE β1c — the pitch-damping flap the aero-only forcing leaves
    // ~14× short (Milestone-6 finding; derivation in MILESTONE6_FLAP_FIX_PREREG.md).
    // Standard coefficient 2; sign set so the induced β1c opposes q (adds damping),
    // not fitted. (sinψ harmonic → rhs index 2.)
    let mut rhs = [0.5 * gamma * f[0], 0.5 * gamma * f[1], 0.5 * gamma * f[2]];
    rhs[2] += props.gyro_rate * q_bar;
    let b = solve3(a, rhs);
    (b[0], b[1], b[2])
}

/// Thrust coefficient and rearward in-plane force coefficient (H), with flapping
/// and pitch rate in `u_P`.
#[allow(clippy::too_many_arguments)]
fn loads(
    rotor: &Rotor,
    airfoil: &dyn Airfoil,
    tip_mach: f64,
    mu: f64,
    lambda: f64,
    controls: &Controls,
    b0: f64,
    b1c: f64,
    b1s: f64,
    q_bar: f64,
) -> (f64, f64) {
    let x0 = rotor.root_cutout;
    let dx = (1.0 - x0) / N_R as f64;
    let dpsi = 2.0 * PI / N_AZ as f64;
    let mut ct = 0.0;
    let mut ch = 0.0;
    for j in 0..N_AZ {
        let psi = (j as f64 + 0.5) * dpsi;
        let (sp, cp) = psi.sin_cos();
        let beta = b0 - b1c * cp - b1s * sp;
        let beta_dot = b1c * sp - b1s * cp;
        let thc = controls.theta_1c * cp + controls.theta_1s * sp;
        for i in 0..N_R {
            let x = x0 + (i as f64 + 0.5) * dx;
            let u_t = x + mu * sp;
            if u_t <= 0.0 {
                continue;
            }
            let u_p = lambda + x * beta_dot + mu * beta * cp - q_bar * x * cp;
            let u2 = u_t * u_t + u_p * u_p;
            let phi = u_p.atan2(u_t);
            let alpha = rotor.pitch(x) + thc - phi;
            let mach = tip_mach * u2.sqrt();
            let (cl, cd) = airfoil.cl_cd(alpha, mach);
            let (s, c) = phi.sin_cos();
            let w = 0.5 * rotor.local_solidity(x) * u2;
            ct += w * (cl * c - cd * s) * dx;
            ch += w * (cl * s + cd * c) * sp * dx;
        }
    }
    let n = N_AZ as f64;
    (ct / n, ch / n)
}

/// Main-rotor longitudinal forces/moments at body state (u, w, q) with controls
/// held at trim. `hub_height` is the hub height above the CG.
#[allow(clippy::too_many_arguments)]
pub fn longitudinal_main_aero(
    rotor: &Rotor,
    op: &Operating,
    airfoil: &dyn Airfoil,
    props: &FlapProperties,
    hub_height: f64,
    controls: &Controls,
    u: f64,
    w: f64,
    q: f64,
) -> LongAero {
    let omega = op.omega;
    let vt = op.tip_speed(rotor.radius);
    let tip_mach = op.tip_mach(rotor.radius);
    let mu = u / vt;
    let lambda_c = -w / vt; // descent (w>0 down) reduces inflow
    let q_bar = q / omega;

    // Converged flap↔inflow with the perturbation terms.
    let mut li = 0.05_f64;
    let (mut b1c, mut b1s) = (0.0, 0.0);
    let mut ct = 0.0;
    let mut ch = 0.0;
    for _ in 0..60 {
        let lam = lambda_c + li;
        let (b0, b1c_k, b1s_k) = flap_coeffs(rotor, controls, mu, lam, q_bar, props);
        b1c = b1c_k;
        b1s = b1s_k;
        let l = loads(
            rotor, airfoil, tip_mach, mu, lam, controls, b0, b1c, b1s, q_bar,
        );
        ct = l.0;
        ch = l.1;
        let li_new = induced(ct.max(0.0), mu, lambda_c);
        let d = (li_new - li).abs();
        li = (li + 0.6 * (li_new - li)).clamp(1e-4, 0.5);
        if d < 1e-8 {
            break;
        }
    }

    let q_dyn = op.rho * rotor.disk_area() * vt * vt;
    let thrust = ct * q_dyn;
    let h_force = ch * q_dyn; // rearward in-plane force
    let x_force = -thrust * b1c.sin() - h_force;
    let z_force = -thrust * b1c.cos();

    // Hub pitch moment from flap-hinge offset.
    let nu2 = props.nu_beta_sq();
    let i_beta =
        op.rho * props.lift_slope * rotor.tip_chord * rotor.radius.powi(4) / props.lock_number;
    let k_beta = i_beta * omega * omega * (nu2 - 1.0);
    let hub_pitch = 0.5 * rotor.n_blades as f64 * k_beta * b1c;
    let pitch_moment = -hub_height * x_force + hub_pitch;

    LongAero {
        thrust,
        x_force,
        z_force,
        pitch_moment,
        beta1c: b1c,
        beta1s: b1s,
    }
}
