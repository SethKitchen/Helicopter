//! Longitudinal stability derivatives by central-difference perturbation of the
//! main-rotor force/moment response about a trimmed hover.

use crate::aero::longitudinal_main_aero;
use crate::context::RotorAero;
use helisim_flapping::Controls;
use helisim_trim::Aircraft;

/// The longitudinal stability derivatives (dimensional): force/moment per unit
/// body velocity or pitch rate. Signs follow body axes (x fwd, z down, pitch up).
#[derive(Clone, Copy, Debug)]
pub struct LongitudinalDerivatives {
    /// ∂X/∂u — surge (drag) damping (< 0).
    pub xu: f64,
    /// ∂X/∂w.
    pub xw: f64,
    /// ∂X/∂q.
    pub xq: f64,
    /// ∂Z/∂u.
    pub zu: f64,
    /// ∂Z/∂w — heave damping (< 0).
    pub zw: f64,
    /// ∂Z/∂q.
    pub zq: f64,
    /// ∂M/∂u — speed stability; > 0 here (destabilizing, the famous one).
    pub mu: f64,
    /// ∂M/∂w.
    pub mw: f64,
    /// ∂M/∂q — pitch damping (< 0).
    pub mq: f64,
}

/// Compute the longitudinal derivatives about a hover equilibrium for `ac`, with
/// the main rotor at `collective` and `controls` held fixed. Central differences
/// in u, w (m/s) and q (rad/s).
pub fn longitudinal_derivatives(
    ac: &Aircraft,
    collective: f64,
    controls: Controls,
) -> LongitudinalDerivatives {
    let rotor = ac.main.with_collective(collective);
    let eval = |u: f64, w: f64, q: f64| {
        longitudinal_main_aero(
            &RotorAero {
                rotor: &rotor,
                op: &ac.main_op,
                airfoil: ac.main_airfoil.as_ref(),
                props: &ac.flap,
                hub_height: ac.hub_height,
                controls: &controls,
            },
            u,
            w,
            q,
        )
    };

    let (du, dw, dq) = (0.5, 0.5, 0.05); // perturbation steps

    let (up, um) = (eval(du, 0.0, 0.0), eval(-du, 0.0, 0.0));
    let (wp, wm) = (eval(0.0, dw, 0.0), eval(0.0, -dw, 0.0));
    let (qp, qm) = (eval(0.0, 0.0, dq), eval(0.0, 0.0, -dq));

    LongitudinalDerivatives {
        xu: (up.x_force - um.x_force) / (2.0 * du),
        zu: (up.z_force - um.z_force) / (2.0 * du),
        mu: (up.pitch_moment - um.pitch_moment) / (2.0 * du),
        xw: (wp.x_force - wm.x_force) / (2.0 * dw),
        zw: (wp.z_force - wm.z_force) / (2.0 * dw),
        mw: (wp.pitch_moment - wm.pitch_moment) / (2.0 * dw),
        xq: (qp.x_force - qm.x_force) / (2.0 * dq),
        zq: (qp.z_force - qm.z_force) / (2.0 * dq),
        mq: (qp.pitch_moment - qm.pitch_moment) / (2.0 * dq),
    }
}
