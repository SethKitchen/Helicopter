//! Fixed-step classical Runge–Kutta (RK4) integrator.

/// Advance `state` by `dt` using one RK4 step, with derivative function `f`.
pub fn rk4_step(state: &[f64], dt: f64, f: impl Fn(&[f64]) -> Vec<f64>) -> Vec<f64> {
    rk4_step_t(state, 0.0, dt, |_, x| f(x))
}

/// Time-aware RK4 step: `f(t, x)` so a time-varying forcing (e.g. a control-input
/// schedule) is sampled at the correct substep times `t, t+dt/2, t+dt`.
pub fn rk4_step_t(state: &[f64], t: f64, dt: f64, f: impl Fn(f64, &[f64]) -> Vec<f64>) -> Vec<f64> {
    let n = state.len();
    let add =
        |a: &[f64], b: &[f64], s: f64| -> Vec<f64> { (0..n).map(|i| a[i] + s * b[i]).collect() };

    let k1 = f(t, state);
    let k2 = f(t + dt / 2.0, &add(state, &k1, dt / 2.0));
    let k3 = f(t + dt / 2.0, &add(state, &k2, dt / 2.0));
    let k4 = f(t + dt, &add(state, &k3, dt));

    (0..n)
        .map(|i| state[i] + dt / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn integrates_harmonic_oscillator() {
        // ẋ = v, v̇ = -x → circle; RK4 conserves energy x²+v² ≈ 1 (phase-robust).
        let dt = 0.01;
        let mut s = vec![1.0, 0.0];
        for _ in 0..1000 {
            s = rk4_step(&s, dt, |x| vec![x[1], -x[0]]);
        }
        let energy = s[0] * s[0] + s[1] * s[1];
        assert!((energy - 1.0).abs() < 1e-6, "energy {energy} drifted");
    }

    #[test]
    fn integrates_exponential() {
        // ẋ = x → e^t.
        let dt = 0.001;
        let mut s = vec![1.0];
        for _ in 0..1000 {
            s = rk4_step(&s, dt, |x| vec![x[0]]);
        }
        assert!((s[0] - std::f64::consts::E).abs() < 1e-6);
    }
}
