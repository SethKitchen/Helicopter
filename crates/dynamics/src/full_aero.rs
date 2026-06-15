//! Generalized main-rotor aero: full body-axis forces and moments as a function
//! of the in-plane velocity `(u, v)`, normal velocity `w`, and body rates
//! `(p, q)`. The longitudinal projection (`v=p=0`) is identical to
//! [`crate::aero::longitudinal_main_aero`], so 5c/5d stay consistent and the
//! coupled 8-state model reproduces both 4-state oracles when decoupled.
//!
//! Generalization of the through-disk velocity and tangential velocity:
//! ```text
//! u_T = x + μ_u sinψ − μ_v cosψ
//! u_P = λ + x β' + β(μ_u cosψ + μ_v sinψ) + p̄ x sinψ − q̄ x cosψ
//! ```
//! Lateral velocity `v` rotates the advancing side 90°, so the flap response and
//! its moment become lateral (roll) — the rotor is axisymmetric.

use crate::context::RotorAero;
use crate::flap_general::flap_coeffs;
use std::f64::consts::PI;

pub(crate) const N_AZ: usize = 36;
pub(crate) const N_R: usize = 30;

/// Body-axis forces and moments (x fwd, y right, z down; roll/pitch/yaw).
#[derive(Clone, Copy, Debug, Default)]
pub struct Forces6 {
    pub fx: f64,
    pub fy: f64,
    pub fz: f64,
    pub mx: f64,
    pub my: f64,
    pub mz: f64,
}

/// Rotate the horizontal force `(fx,fy)` and horizontal moment `(mx,my)` of a
/// `Forces6` by `psi` about the (vertical) shaft axis; `fz, mz` are invariant.
/// Used to build the lateral/coupled response by exact rotation of the validated
/// longitudinal response (the rotor is axisymmetric about the shaft).
pub fn rotate6(f: Forces6, psi: f64) -> Forces6 {
    let (s, c) = psi.sin_cos();
    Forces6 {
        fx: f.fx * c - f.fy * s,
        fy: f.fx * s + f.fy * c,
        fz: f.fz,
        mx: f.mx * c - f.my * s,
        my: f.mx * s + f.my * c,
        mz: f.mz,
    }
}

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

/// Loads with a linear inflow. Returns `(C_T, C_P, Hx, Hy, C_Laero, C_Maero)`:
/// thrust, power, in-plane H-forces, and the *aerodynamic* roll/pitch moment
/// coefficients (∫dC_T·x·sinψ and ∫dC_T·x·cosψ) — the latter two are the forcing
/// the Pitt–Peters inflow states respond to.
fn loads(
    aero: &RotorAero,
    mu: [f64; 2],
    inflow: [f64; 3],
    flap: [f64; 3],
    rates: [f64; 2],
) -> (f64, f64, f64, f64, f64, f64) {
    let (rotor, airfoil, controls) = (aero.rotor, aero.airfoil, aero.controls);
    let tip_mach = aero.op.tip_mach(rotor.radius);
    let [mu_u, mu_v] = mu;
    let [b0, b1c, b1s] = flap;
    let [p_bar, q_bar] = rates;
    let x0 = rotor.root_cutout;
    let dx = (1.0 - x0) / N_R as f64;
    let dpsi = 2.0 * PI / N_AZ as f64;
    let (mut ct, mut cp, mut hx, mut hy, mut cl_a, mut cm_a) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for j in 0..N_AZ {
        let psi = (j as f64 + 0.5) * dpsi;
        let (sp, cp_) = psi.sin_cos();
        let beta = b0 - b1c * cp_ - b1s * sp;
        let beta_dot = b1c * sp - b1s * cp_;
        let thc = controls.theta_1c * cp_ + controls.theta_1s * sp;
        for i in 0..N_R {
            let x = x0 + (i as f64 + 0.5) * dx;
            let u_t = x + mu_u * sp - mu_v * cp_;
            if u_t <= 0.0 {
                continue;
            }
            let lam = inflow[0] + inflow[2] * x * cp_ + inflow[1] * x * sp;
            let u_p = lam + x * beta_dot + beta * (mu_u * cp_ + mu_v * sp) + p_bar * x * sp
                - q_bar * x * cp_;
            let u2 = u_t * u_t + u_p * u_p;
            let phi = u_p.atan2(u_t);
            let alpha = rotor.pitch(x) + thc - phi;
            let mach = tip_mach * u2.sqrt();
            let (cl, cd) = airfoil.cl_cd(alpha, mach);
            let (s, c) = phi.sin_cos();
            let w = 0.5 * rotor.local_solidity(x) * u2;
            let f_q = cl * s + cd * c; // in-plane (tangential) force coeff
            let dct = w * (cl * c - cd * s) * dx;
            ct += dct;
            cp += w * f_q * x * dx;
            hx += w * f_q * sp * dx; // fore-aft in-plane force
            hy += -w * f_q * cp_ * dx; // lateral in-plane force
            cl_a += dct * x * sp; // aero roll moment (inflow forcing)
            cm_a += dct * x * cp_; // aero pitch moment (inflow forcing)
        }
    }
    let n = N_AZ as f64;
    (ct / n, cp / n, hx / n, hy / n, cl_a / n, cm_a / n)
}

/// Full main-rotor body forces and moments at velocity `(u,v,w)` and rates
/// `(p,q)`, with controls at trim. `hub_height` is the hub height above the CG.
/// Yaw moment is the steady main-rotor torque reaction.
pub fn main_rotor_full(aero: &RotorAero, vel: [f64; 3], rates: [f64; 2]) -> Forces6 {
    let op = aero.op;
    let li = uniform_inflow(aero, vel, rates);
    let vt = op.tip_speed(aero.rotor.radius);
    let mu = [vel[0] / vt, vel[1] / vt];
    let rates_bar = [rates[0] / op.omega, rates[1] / op.omega];
    // Cyclic states zero, heave folded into the mean inflow: identical to
    // `main_rotor_with_inflow(.., [λ₀,0,0])` — the 5h τ→0 baseline gate.
    assemble_forces(aero, mu, rates_bar, [li - vel[2] / vt, 0.0, 0.0]).0
}

/// The converged **uniform** (Glauert) induced inflow `λ₀` — the quasi-static
/// baseline through 5g, with cyclic inflow forced to zero. Exposed so the
/// Pitt–Peters layer can recover the validated baseline exactly by passing
/// `[λ₀, 0, 0]` to `main_rotor_with_inflow`.
pub fn uniform_inflow(aero: &RotorAero, vel: [f64; 3], rates: [f64; 2]) -> f64 {
    let op = aero.op;
    let vt = op.tip_speed(aero.rotor.radius);
    let mu = [vel[0] / vt, vel[1] / vt];
    let lambda_c = -vel[2] / vt;
    let rates_bar = [rates[0] / op.omega, rates[1] / op.omega];

    let mut li = 0.05_f64;
    for _ in 0..60 {
        let lam = lambda_c + li;
        let b = flap_coeffs(aero, mu, [lam, 0.0, 0.0], rates_bar);
        let ct = loads(aero, mu, [lam, 0.0, 0.0], [b.0, b.1, b.2], rates_bar).0;
        let li_new = induced(ct.max(0.0), (mu[0] * mu[0] + mu[1] * mu[1]).sqrt(), lambda_c);
        let d = (li_new - li).abs();
        li = (li + 0.6 * (li_new - li)).clamp(1e-4, 0.5);
        if d < 1e-8 {
            break;
        }
    }
    li
}

/// Aerodynamic forcing for the Pitt–Peters inflow states: thrust and the
/// aerodynamic roll/pitch moment coefficients.
#[derive(Clone, Copy, Debug, Default)]
pub struct InflowAero {
    pub c_t: f64,
    pub c_roll: f64,
    pub c_pitch: f64,
}

/// Flap + loads + force/moment assembly for a GIVEN linear inflow `[λ₀,λ₁s,λ₁c]`.
/// Returns the body forces/moments and the inflow forcing. This is the shared
/// core used both by the uniform-inflow [`main_rotor_full`] and the inflow-input
/// `main_rotor_with_inflow` (in `inflow_coupling`).
pub(crate) fn assemble_forces(
    aero: &RotorAero,
    mu: [f64; 2],
    rates: [f64; 2],
    inflow: [f64; 3],
) -> (Forces6, InflowAero) {
    let (rotor, op, props, hub_height) = (aero.rotor, aero.op, aero.props, aero.hub_height);
    let omega = op.omega;
    let vt = op.tip_speed(rotor.radius);

    let (b0, b1c, b1s) = flap_coeffs(aero, mu, inflow, rates);
    let (ct, cp, hx, hy, cl_a, cm_a) = loads(aero, mu, inflow, [b0, b1c, b1s], rates);

    let qd = op.rho * rotor.disk_area() * vt * vt;
    let thrust = ct * qd;
    let (h_x, h_y) = (hx * qd, hy * qd);
    let fx = -thrust * b1c.sin() - h_x;
    let fy = -thrust * b1s.sin() - h_y;
    let fz = -thrust * b1c.cos() * b1s.cos();

    let nu2 = props.nu_beta_sq();
    let i_beta =
        op.rho * props.lift_slope * rotor.tip_chord * rotor.radius.powi(4) / props.lock_number;
    let k_beta = i_beta * omega * omega * (nu2 - 1.0);
    let half_nb = 0.5 * rotor.n_blades as f64;
    let hub_pitch = half_nb * k_beta * b1c;
    let hub_roll = -half_nb * k_beta * b1s;
    let torque = cp * qd * rotor.radius;

    (
        Forces6 {
            fx,
            fy,
            fz,
            mx: hub_height * fy + hub_roll,
            my: -hub_height * fx + hub_pitch,
            mz: torque,
        },
        InflowAero {
            c_t: ct,
            c_roll: cl_a,
            c_pitch: cm_a,
        },
    )
}
