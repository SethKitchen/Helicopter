//! Run a nonlinear longitudinal trajectory from a trimmed hover.

use crate::eom::{LongModel, state_derivative};
use crate::rk4::rk4_step;
use helisim_dynamics::hover_collective_for_weight;
use helisim_flapping::Controls;
use helisim_trim::Aircraft;

/// A recorded trajectory: times and the `[u, w, q, θ]` states.
#[derive(Clone, Debug)]
pub struct Trajectory {
    /// Sample times, s.
    pub times: Vec<f64>,
    /// States at each time, `[u, w, q, θ]`.
    pub states: Vec<[f64; 4]>,
}

impl Trajectory {
    /// Column `i` of the state across time (0=u,1=w,2=q,3=θ).
    pub fn column(&self, i: usize) -> Vec<f64> {
        self.states.iter().map(|s| s[i]).collect()
    }
    /// Max absolute value of any state component over the trajectory.
    pub fn max_abs(&self) -> f64 {
        self.states
            .iter()
            .flat_map(|s| s.iter())
            .fold(0.0_f64, |m, &v| m.max(v.abs()))
    }
}

/// Trim `ac` at hover, perturb the longitudinal state by `perturbation`
/// `[Δu, Δw, Δq, Δθ]`, and integrate the nonlinear longitudinal EOM with fixed
/// step `dt` to `t_end`. `i_yy` is the pitch inertia.
pub fn simulate_hover_longitudinal(
    ac: &Aircraft,
    i_yy: f64,
    perturbation: [f64; 4],
    dt: f64,
    t_end: f64,
) -> Trajectory {
    // Same self-consistent hover equilibrium the 5c eigenvalues are taken about,
    // so [0,0,0,0] is an exact fixed point and the gate compares like with like.
    let collective = hover_collective_for_weight(ac);
    let model = LongModel {
        rotor: ac.main.with_collective(collective),
        op: ac.main_op,
        airfoil: ac.main_airfoil.as_ref(),
        flap: ac.flap,
        controls: Controls::none(),
        hub_height: ac.hub_height,
        mass: ac.mass,
        i_yy,
    };

    let mut s = perturbation; // equilibrium is [0,0,0,0] in these perturbation states
    let mut time = 0.0;
    let mut times = vec![0.0];
    let mut states = vec![s];

    let n = (t_end / dt).round() as usize;
    for _ in 0..n {
        let next = rk4_step(&s, dt, |x| state_derivative(&model, x));
        s = [next[0], next[1], next[2], next[3]];
        time += dt;
        times.push(time);
        states.push(s);
    }
    Trajectory { times, states }
}

/// Integrate a general linear system `ẋ = A·x` (any size) from `x0` to `t_end`.
/// Returns the state at each step. Used to time-march the coupled 8-state model.
pub fn simulate_linear_nd(a: &[Vec<f64>], x0: &[f64], dt: f64, t_end: f64) -> Vec<Vec<f64>> {
    let n = x0.len();
    let mut s = x0.to_vec();
    let mut out = vec![s.clone()];
    let steps = (t_end / dt).round() as usize;
    for _ in 0..steps {
        s = rk4_step(&s, dt, |x| {
            (0..n)
                .map(|i| (0..n).map(|jj| a[i][jj] * x[jj]).sum())
                .collect()
        });
        out.push(s.clone());
    }
    out
}

/// Integrate the *linear* model `ẋ = A·x` from `x0` (for the linear-vs-nonlinear
/// consistency gate). `a` is the 4×4 system matrix from the dynamics crate.
pub fn simulate_linear(a: &[Vec<f64>], x0: [f64; 4], dt: f64, t_end: f64) -> Trajectory {
    let mut s = x0.to_vec();
    let mut time = 0.0;
    let mut times = vec![0.0];
    let mut states = vec![x0];
    let n = (t_end / dt).round() as usize;
    for _ in 0..n {
        let next = rk4_step(&s, dt, |x| {
            (0..4)
                .map(|i| (0..4).map(|j| a[i][j] * x[j]).sum())
                .collect()
        });
        s = next;
        time += dt;
        times.push(time);
        states.push([s[0], s[1], s[2], s[3]]);
    }
    Trajectory { times, states }
}
