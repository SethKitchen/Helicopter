//! Stability augmentation (5j): rate feedback (a SCAS damper) on the driven
//! 11-state system. Rate states feed back to the controls that damp them —
//! `p→lat-cyclic`, `q→lon-cyclic`, `r→pedal` — with the gain signs chosen (from
//! the control-effectiveness signs) so each closes a negative-damping loop.
//!
//! **Scope (named, to keep 5j from sprawling into autopilot):** this is a *damper*,
//! not an attitude/velocity hold. It makes the aircraft flyable-by-a-pilot by
//! pushing the unstable modes into the LHP; it does NOT hold attitude hands-off.
//! Attitude/velocity hold and any outer autopilot loop is a later milestone.
//!
//! **Design vs validation (the 5i lesson projected forward):** the hover Jacobian
//! is blind to the wake-skew λ₀↔λ₁c coupling (non-analytic at μ=0), so it is a
//! valid design oracle only for the modes that survive the linearization. Gains
//! are seeded from the hover linear model (which sees the body modes), then
//! *designed and validated OFF the seam* at a small forward speed where the
//! Jacobian is differentiable and linear↔nonlinear agree, then *confirmed across
//! the seam* on the nonlinear hover march. Linear closed-loop eigenvalues in the
//! LHP are necessary but — at hover — explicitly not sufficient.

use crate::control::{Channel, ControlSchedule};
use crate::driven_equilibrium::{equilibrium_state11_at, model11_at};
use crate::driven_march::{State11, deriv11};
use crate::rk4::rk4_step_t;
use helisim_dynamics::Inertia;
use helisim_trim::Aircraft;

/// A proportional rate damper: control deltas `Δu = K·(x − x_eq)`, with `K`
/// nonzero only on the (rate → damping-control) entries.
#[derive(Clone, Copy, Debug, Default)]
pub struct RateSas {
    /// Feedback gain `K` (4 controls × 11 states).
    pub gain: [[f64; 11]; 4],
}

impl RateSas {
    /// Rate damper with roll/pitch/yaw-rate gains. Positive gains add damping:
    /// `Δθ1c=−gp·p`, `Δθ1s=−gq·q` (∂ṗ/∂θ1c, ∂q̇/∂θ1s > 0), `Δθ_tail=+gr·r`
    /// (∂ṙ/∂pedal < 0) — each sign closes a negative-feedback loop on the rate.
    pub fn rate_damper(gp: f64, gq: f64, gr: f64) -> Self {
        let mut gain = [[0.0; 11]; 4];
        gain[Channel::LatCyclic as usize][5] = -gp; // p (index 5)
        gain[Channel::LonCyclic as usize][2] = -gq; // q (index 2)
        gain[Channel::Pedal as usize][6] = gr; // r (index 6)
        Self { gain }
    }

    /// Feedback control deltas for state `x` about equilibrium `eq`.
    pub fn deltas(&self, x: &[f64], eq: &[State11; 1]) -> [f64; 4] {
        let eq = &eq[0];
        let mut d = [0.0; 4];
        for (c, dc) in d.iter_mut().enumerate() {
            for i in 0..11 {
                *dc += self.gain[c][i] * (x[i] - eq[i]);
            }
        }
        d
    }
}

/// Closed-loop system matrix `A_cl = A + B·K` — the linear design/eigenvalue model.
pub fn closed_loop_matrix(a: &[Vec<f64>], b: &[[f64; 4]; 11], sas: &RateSas) -> Vec<Vec<f64>> {
    let mut acl = a.to_vec();
    for (i, row) in acl.iter_mut().enumerate() {
        for (jcol, e) in row.iter_mut().enumerate() {
            let bk: f64 = b[i]
                .iter()
                .zip(sas.gain.iter())
                .map(|(&bic, kc)| bic * kc[jcol])
                .sum();
            *e += bk;
        }
    }
    acl
}

/// Integrate the SAS-augmented 11-state EOM about the equilibrium at body velocity
/// `vel`, with pilot feedforward `pilot` plus the state-feedback `sas`.
#[allow(clippy::too_many_arguments)]
pub fn simulate11_sas(
    ac: &Aircraft,
    j: Inertia,
    vel: [f64; 3],
    pilot: &dyn ControlSchedule,
    sas: &RateSas,
    perturbation: State11,
    dt: f64,
    t_end: f64,
) -> Vec<State11> {
    simulate11_sas_dist(ac, j, vel, pilot, sas, [0.0; 3], perturbation, dt, t_end)
}

/// As [`simulate11_sas`], with a **sustained external body-moment disturbance**
/// `disturb = [L, M, N]` (N·m) — a steady gust / c.g.-offset moment the controller
/// must counter. The feedback never sees the disturbance directly (it feeds back
/// state), so the standing attitude error this leaves is the regulation gate: a
/// rate damper allows a standing offset (or diverges), attitude hold drives it small.
#[allow(clippy::too_many_arguments)]
pub fn simulate11_sas_dist(
    ac: &Aircraft,
    j: Inertia,
    vel: [f64; 3],
    pilot: &dyn ControlSchedule,
    sas: &RateSas,
    disturb: [f64; 3],
    perturbation: State11,
    dt: f64,
    t_end: f64,
) -> Vec<State11> {
    let m = model11_at(ac, j, vel);
    let eq = [equilibrium_state11_at(ac, vel)];
    let (ixx, iyy, izz) = (j.i_xx, j.i_yy, j.i_zz);
    let mut s: Vec<f64> = (0..11).map(|i| eq[0][i] + perturbation[i]).collect();
    let mut out = vec![to11(&s)];
    let mut t = 0.0;
    let n = (t_end / dt).round() as usize;
    for _ in 0..n {
        s = rk4_step_t(&s, t, dt, |tt, x| {
            let mut d = pilot.deltas(tt);
            let fb = sas.deltas(x, &eq);
            for c in 0..4 {
                d[c] += fb[c];
            }
            let mut ds = deriv11(&m, d, x);
            // External moment disturbance adds directly to the angular accelerations.
            ds[5] += disturb[0] / ixx; // ṗ
            ds[2] += disturb[1] / iyy; // q̇
            ds[6] += disturb[2] / izz; // ṙ
            ds
        });
        t += dt;
        out.push(to11(&s));
    }
    out
}

fn to11(s: &[f64]) -> State11 {
    let mut a = [0.0; 11];
    a.copy_from_slice(&s[..11]);
    a
}
