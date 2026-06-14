//! PI attitude hold (5l): add **integral action** to the 5k proportional attitude
//! loop, closing its residual steady-state error.
//!
//! 5k held attitude but, being proportional, left a *bounded standing offset*
//! under a sustained disturbance (~4° for 0.8 N·m). That residual is a property of
//! the attitude loop, so it is fixed HERE — before the velocity loop (5m) is built
//! on top — otherwise the outer loop would command an inner loop with a known
//! steady-state error and inherit it. (Controls form of "don't build the tuned
//! layer on unverified/flawed physics".)
//!
//! Integral action adds two controller states `z = [∫(θ−θ_e), ∫(φ−φ_e)]` to the
//! marched vector (now 13). At steady state `ż = 0 ⇒ θ=θ_e, φ=φ_e` — **zero
//! steady-state error to a constant disturbance**, the textbook integral property,
//! and the falsifiable oracle for this milestone.
//!
//! Anti-windup is NOT yet implemented (named): at these disturbance amplitudes the
//! controls stay well inside their range, so the integrator doesn't saturate; it
//! becomes necessary once a disturbance drives the cyclic to its limits.

use crate::control::{Channel, ControlSchedule};
use crate::driven_equilibrium::{equilibrium_state11_at, model11_at};
use crate::driven_march::{State11, deriv11};
use crate::rk4::rk4_step_t;
use crate::sas::{RateSas, closed_loop_matrix};
use helisim_dynamics::Inertia;
use helisim_trim::Aircraft;

/// PI attitude hold: the proportional law `p` (rate damper + attitude P) plus
/// integral gains on the pitch- and roll-attitude errors.
#[derive(Clone, Copy, Debug)]
pub struct PiAttitudeHold {
    pub p: RateSas,
    pub ki_theta: f64,
    pub ki_phi: f64,
}

impl PiAttitudeHold {
    pub fn new(p: RateSas, ki_theta: f64, ki_phi: f64) -> Self {
        Self {
            p,
            ki_theta,
            ki_phi,
        }
    }

    /// Control deltas for plant state `x`, integrator state `z`, equilibrium `eq`.
    pub fn deltas(&self, x: &[f64], z: [f64; 2], eq: &[State11; 1]) -> [f64; 4] {
        let mut d = self.p.deltas(x, eq);
        d[Channel::LonCyclic as usize] += -self.ki_theta * z[0];
        d[Channel::LatCyclic as usize] += -self.ki_phi * z[1];
        d
    }
}

/// The 13-state augmented closed-loop matrix `[[A_cl, −B·Ki], [C, 0]]` where
/// `A_cl = A + B·Kp`, `C` picks the attitude errors (`ż = [θ, φ]`). Its eigenvalues
/// are the closed-loop poles WITH integral action — the off-seam trustworthy gate
/// (the integrator must not destabilize).
pub fn augmented_matrix(a: &[Vec<f64>], b: &[[f64; 4]; 11], pi: &PiAttitudeHold) -> Vec<Vec<f64>> {
    let acl = closed_loop_matrix(a, b, &pi.p);
    let mut aug = vec![vec![0.0; 13]; 13];
    for (i, arow) in acl.iter().enumerate() {
        aug[i][..11].copy_from_slice(&arow[..11]);
        aug[i][11] = -b[i][Channel::LonCyclic as usize] * pi.ki_theta;
        aug[i][12] = -b[i][Channel::LatCyclic as usize] * pi.ki_phi;
    }
    aug[11][3] = 1.0; // ż_θ = θ − θ_e
    aug[12][7] = 1.0; // ż_φ = φ − φ_e
    aug
}

/// State `[u,w,q,θ,v,p,r,φ,λ₀,λ₁s,λ₁c, z_θ, z_φ]` (plant 11 + 2 integrators).
pub type State13 = [f64; 13];

/// Integrate the PI-attitude-hold 13-state EOM about the equilibrium at body
/// velocity `vel`, with pilot feedforward, the PI controller, and an optional
/// sustained external moment disturbance `[L,M,N]`. Integrators start at zero.
pub fn simulate13(
    ac: &Aircraft,
    j: Inertia,
    vel: [f64; 3],
    pilot: &dyn ControlSchedule,
    pi: &PiAttitudeHold,
    disturb: [f64; 3],
    perturbation: State11,
    dt: f64,
    t_end: f64,
) -> Vec<State13> {
    let m = model11_at(ac, j, vel);
    let eq = [equilibrium_state11_at(ac, vel)];
    let (ixx, iyy, izz) = (j.i_xx, j.i_yy, j.i_zz);
    let mut s: Vec<f64> = (0..11).map(|i| eq[0][i] + perturbation[i]).collect();
    s.push(0.0);
    s.push(0.0);
    let mut out = vec![to13(&s)];
    let mut t = 0.0;
    let n = (t_end / dt).round() as usize;
    for _ in 0..n {
        s = rk4_step_t(&s, t, dt, |tt, x| {
            let z = [x[11], x[12]];
            let mut d = pilot.deltas(tt);
            let fb = pi.deltas(&x[..11], z, &eq);
            for c in 0..4 {
                d[c] += fb[c];
            }
            let mut ds = deriv11(&m, d, &x[..11]);
            ds[5] += disturb[0] / ixx;
            ds[2] += disturb[1] / iyy;
            ds[6] += disturb[2] / izz;
            ds.push(x[3] - eq[0][3]); // ż_θ
            ds.push(x[7] - eq[0][7]); // ż_φ
            ds
        });
        t += dt;
        out.push(to13(&s));
    }
    out
}

fn to13(s: &[f64]) -> State13 {
    let mut a = [0.0; 13];
    a.copy_from_slice(&s[..13]);
    a
}
