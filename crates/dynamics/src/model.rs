//! Assemble the linearized longitudinal system matrix and extract its modes.

use crate::aero::longitudinal_main_aero;
use crate::complex::Complex;
use crate::context::RotorAero;
use crate::derivatives::{LongitudinalDerivatives, longitudinal_derivatives};
use crate::eigen::eigenvalues;
use helisim_flapping::Controls;
use helisim_trim::Aircraft;

const G: f64 = 9.80665;

/// The hover-equilibrium main-rotor collective for `ac`: the collective at which
/// the *dynamics* aero (uniform inflow) produces thrust = weight at zero cyclic.
///
/// Defining the equilibrium with the same aero the dynamics/sim use makes
/// `[u,w,q,θ] = 0` an exact fixed point of the EOM (so the time-march starts at
/// rest and stays there), and makes the 5c eigenvalues and the 5d time-march
/// describe the *same* equilibrium. (This collective differs slightly from the
/// bemt tip-loss trim of milestone 1 — a separate, earlier validation.)
pub fn hover_collective_for_weight(ac: &Aircraft) -> f64 {
    let w = ac.mass * G;
    let thrust = |coll: f64| {
        longitudinal_main_aero(
            &RotorAero {
                rotor: &ac.main.with_collective(coll),
                op: &ac.main_op,
                airfoil: ac.main_airfoil.as_ref(),
                props: &ac.flap,
                hub_height: ac.hub_height,
                controls: &Controls::none(),
            },
            0.0,
            0.0,
            0.0,
        )
        .thrust
    };
    let (mut lo, mut hi) = (0.0_f64, 20f64.to_radians());
    for _ in 0..80 {
        let mid = 0.5 * (lo + hi);
        if thrust(mid) < w {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    0.5 * (lo + hi)
}

/// A dynamic mode classified from an eigenvalue.
#[derive(Clone, Copy, Debug)]
pub struct Mode {
    /// The eigenvalue (1/s).
    pub eigenvalue: Complex,
    /// Oscillatory (non-zero imaginary part)?
    pub oscillatory: bool,
    /// Stable (negative real part)?
    pub stable: bool,
    /// Undamped natural frequency, rad/s (oscillatory modes).
    pub frequency: f64,
    /// Period, s (oscillatory modes; ∞ otherwise).
    pub period: f64,
    /// Time to half (stable) or double (unstable) amplitude, s.
    pub time_to_half_or_double: f64,
}

impl Mode {
    /// Classify an eigenvalue into a mode (public for the lateral analysis).
    pub fn from_eigenvalue_pub(e: Complex) -> Self {
        Self::from_eigenvalue(e)
    }

    fn from_eigenvalue(e: Complex) -> Self {
        let oscillatory = e.im.abs() > 1e-6;
        let stable = e.re < 0.0;
        let frequency = if oscillatory {
            (e.re * e.re + e.im * e.im).sqrt()
        } else {
            0.0
        };
        let period = if oscillatory && e.im.abs() > 1e-9 {
            2.0 * std::f64::consts::PI / e.im.abs()
        } else {
            f64::INFINITY
        };
        let time = if e.re.abs() > 1e-9 {
            (2.0_f64).ln() / e.re.abs()
        } else {
            f64::INFINITY
        };
        Mode {
            eigenvalue: e,
            oscillatory,
            stable,
            frequency,
            period,
            time_to_half_or_double: time,
        }
    }
}

/// Result of the hover longitudinal modal analysis.
#[derive(Clone, Debug)]
pub struct ModalAnalysis {
    /// Hover-equilibrium collective used, rad.
    pub collective: f64,
    /// The stability derivatives.
    pub derivatives: LongitudinalDerivatives,
    /// The 4×4 system matrix A (states [u, w, q, θ]).
    pub a_matrix: Vec<Vec<f64>>,
    /// Eigenvalues of A.
    pub eigenvalues: Vec<Complex>,
    /// Classified modes.
    pub modes: Vec<Mode>,
    /// Whether any mode is unstable (positive real part).
    pub unstable: bool,
    /// Whether there is an *oscillatory* unstable mode (the hover signature).
    pub has_unstable_oscillation: bool,
}

/// Run the hover longitudinal modal analysis for `ac` with pitch inertia `i_yy`.
///
/// Trims hover, measures the longitudinal stability derivatives, assembles the
/// 4×4 system matrix for states `[u, w, q, θ]`, and extracts the modes.
pub fn analyze_hover_longitudinal(ac: &Aircraft, i_yy: f64) -> ModalAnalysis {
    let collective = hover_collective_for_weight(ac);
    let d = longitudinal_derivatives(ac, collective, Controls::none());

    let m = ac.mass;
    // States [u, w, q, θ]; hover (u0 = 0, θ0 ≈ 0).
    let a = vec![
        vec![d.xu / m, d.xw / m, d.xq / m, -G],
        vec![d.zu / m, d.zw / m, d.zq / m, 0.0],
        vec![d.mu / i_yy, d.mw / i_yy, d.mq / i_yy, 0.0],
        vec![0.0, 0.0, 1.0, 0.0],
    ];

    let ev = eigenvalues(&a);
    let modes: Vec<Mode> = ev.iter().map(|&e| Mode::from_eigenvalue(e)).collect();
    let unstable = modes.iter().any(|m| !m.stable && m.eigenvalue.re > 1e-6);
    let has_unstable_oscillation = modes
        .iter()
        .any(|m| m.oscillatory && m.eigenvalue.re > 1e-6);

    ModalAnalysis {
        collective,
        derivatives: d,
        a_matrix: a,
        eigenvalues: ev,
        modes,
        unstable,
        has_unstable_oscillation,
    }
}

/// The classic hover longitudinal characteristic *cubic* in [u, q, θ] (heave
/// decoupled), for the analytic cross-check of the eigenvalue routine:
/// `s³ + a s² + b s + c` with
/// `a = −(Xu/m + Mq/Iyy)`, `b = (Xu·Mq − Xq·Mu)/(m·Iyy)`, `c = g·Mu/Iyy`.
pub fn hovering_cubic(d: &LongitudinalDerivatives, mass: f64, i_yy: f64) -> [f64; 4] {
    let a = -(d.xu / mass + d.mq / i_yy);
    let b = (d.xu * d.mq - d.xq * d.mu) / (mass * i_yy);
    let c = G * d.mu / i_yy;
    [1.0, a, b, c]
}
