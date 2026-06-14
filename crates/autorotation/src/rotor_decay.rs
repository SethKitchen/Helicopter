//! Autorotation-entry rotor-speed decay — "how many seconds do you have?"
//!
//! The instant the motor quits in a hover, the rotor must keep turning on its own
//! stored kinetic energy while it supplies the power the motor no longer does.
//! Rotor speed decays, and below a minimum controllable speed `Ω_min` the blades
//! stall and recovery is impossible. The time available to react and establish
//! autorotation is the physical content of the low-speed height-velocity lobe.
//!
//! The rotor angular equation of motion with the shaft torque removed is
//!
//! `I Ω dΩ/dt = −P_drain(t)`,
//!
//! where `P_drain` is the aerodynamic power the rotor must source. **Worst case**,
//! immediately after power loss and before any descent develops, the rotor must
//! supply the *full hover power* `P_h`, with `P_drain` constant. Then the ODE
//! integrates in closed form:
//!
//! `½ I (Ω₀² − Ω(t)²) = P_h t`  ⟹  `t_decay = ½ I (Ω₀² − Ω_min²) / P_h = E_flare / P_h`.
//!
//! That is the conservative reaction-time bound: the rotor's usable energy divided
//! by the power draining it. As the aircraft starts to descend, the upflow returns
//! energy to the rotor and `P_drain` falls toward zero (steady autorotation needs
//! no shaft power), so the real decay is *slower* — more time than the bound.
//!
//! This module gives both: the analytic bound, and an RK4 march of the ODE with a
//! decaying `P_drain` — **gated against the analytic bound in the constant-power
//! limit** (the project's rule: never trust a time-integrator without a
//! pre-computed oracle; here the oracle is exact). The integrator is honest where
//! the full vortex-ring-coupled entry aerodynamics — deliberately not modelled —
//! would not have a clean oracle.

/// Analytic worst-case time (s) for rotor speed to fall from `omega0` to
/// `omega_min` while supplying constant hover power `hover_power_w`:
/// `t = ½ I (Ω₀² − Ω_min²) / P_h`.
pub fn decay_time_constant_power(
    inertia: f64,
    omega0: f64,
    omega_min: f64,
    hover_power_w: f64,
) -> f64 {
    0.5 * inertia * (omega0 * omega0 - omega_min * omega_min) / hover_power_w
}

/// RK4 march of `I Ω dΩ/dt = −P_h·relief(t)`, returning `(time, omega)` samples
/// until `omega` reaches `omega_min` or `t_max`. The power drain relaxes as the
/// descent establishes: `relief(t) = exp(−t/establish_tau)` (large `establish_tau`
/// → constant hover power = the worst case, recovering the analytic bound).
pub fn simulate_decay(
    inertia: f64,
    omega0: f64,
    omega_min: f64,
    hover_power_w: f64,
    establish_tau: f64,
    dt: f64,
    t_max: f64,
) -> Vec<(f64, f64)> {
    // dΩ/dt = −P_h·relief(t) / (I·Ω)
    let deriv = |t: f64, omega: f64| {
        let relief = (-t / establish_tau).exp();
        -hover_power_w * relief / (inertia * omega)
    };
    let mut out = vec![(0.0, omega0)];
    let mut t = 0.0;
    let mut omega = omega0;
    while t < t_max && omega > omega_min {
        let k1 = deriv(t, omega);
        let k2 = deriv(t + 0.5 * dt, omega + 0.5 * dt * k1);
        let k3 = deriv(t + 0.5 * dt, omega + 0.5 * dt * k2);
        let k4 = deriv(t + dt, omega + dt * k3);
        omega += dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
        t += dt;
        out.push((t, omega));
    }
    out
}

/// Time (s) for `simulate_decay` to reach `omega_min`, or `None` if it never does
/// within `t_max` (the descent relieved the drain before the rotor decayed —
/// recoverable). Linearly interpolates the crossing.
pub fn time_to_min_rpm(
    inertia: f64,
    omega0: f64,
    omega_min: f64,
    hover_power_w: f64,
    establish_tau: f64,
    dt: f64,
    t_max: f64,
) -> Option<f64> {
    let hist = simulate_decay(inertia, omega0, omega_min, hover_power_w, establish_tau, dt, t_max);
    let last = *hist.last().unwrap();
    if last.1 > omega_min {
        return None; // never decayed to the limit — recoverable
    }
    // Interpolate between the last two samples for the crossing time.
    let (t0, w0) = hist[hist.len() - 2];
    let (t1, w1) = last;
    Some(t0 + (w0 - omega_min) / (w0 - w1) * (t1 - t0))
}

#[cfg(test)]
mod tests {
    use super::*;

    const I: f64 = 1500.0;
    const OM0: f64 = 47.5; // rad/s (190/4)
    const PH: f64 = 200_000.0; // W hover power
    const OM_MIN: f64 = 0.7 * OM0;

    #[test]
    fn analytic_decay_time_value() {
        let t = decay_time_constant_power(I, OM0, OM_MIN, PH);
        let expected = 0.5 * I * (OM0 * OM0 - OM_MIN * OM_MIN) / PH;
        assert!((t - expected).abs() < 1e-12);
        assert!(t > 0.0);
    }

    /// ORACLE GATE: with the descent relief switched off (huge τ → constant hover
    /// power), the RK4 march must reproduce the analytic decay time.
    #[test]
    fn rk4_matches_analytic_in_constant_power_limit() {
        let analytic = decay_time_constant_power(I, OM0, OM_MIN, PH);
        let marched = time_to_min_rpm(I, OM0, OM_MIN, PH, 1.0e9, 0.001, 100.0).unwrap();
        assert!((marched - analytic).abs() / analytic < 1e-3, "{marched} vs {analytic}");
    }

    /// With a realistic descent relief, the rotor decays SLOWER (more time) than
    /// the worst-case bound — or never reaches Ω_min at all.
    #[test]
    fn descent_relief_buys_time() {
        let analytic = decay_time_constant_power(I, OM0, OM_MIN, PH);
        // If it reaches Ω_min at all, the relieved decay must take longer than the
        // worst-case bound; if it never decays (None), that is even safer.
        if let Some(t) = time_to_min_rpm(I, OM0, OM_MIN, PH, 3.0, 0.001, 100.0) {
            assert!(t > analytic, "relieved decay {t} should exceed bound {analytic}");
        }
    }

    /// Step-size check (the project's RK4 caution): halving dt barely moves it.
    #[test]
    fn step_size_convergence() {
        let coarse = time_to_min_rpm(I, OM0, OM_MIN, PH, 1.0e9, 0.01, 100.0).unwrap();
        let fine = time_to_min_rpm(I, OM0, OM_MIN, PH, 1.0e9, 0.0025, 100.0).unwrap();
        assert!((coarse - fine).abs() / fine < 1e-3);
    }
}
