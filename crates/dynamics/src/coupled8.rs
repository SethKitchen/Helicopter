//! Coupled 8-state lateral-longitudinal linear model (5e-ii).
//!
//! State `[u, w, q, θ, v, p, r, φ]`. The block structure:
//! * longitudinal 4×4 block — from the full-aero u/w/q perturbations (identical
//!   to 5c).
//! * lateral 4×4 block — from [`crate::lateral`] (identical to 5e-i).
//! * cross-coupling — the pitch-roll coupling. The longitudinal→lateral terms
//!   (Yu, Lu, Nu, Yq, Lq, Nq) come from the *validated* longitudinal-input
//!   perturbations read on the off-axis; the lateral→longitudinal terms follow
//!   by the rotor's axisymmetry (`Xv=−Yu`, `Mv=−Lu`, `Mp=−Lq`, …), so the whole
//!   coupling rests on validated machinery, not the lateral azimuthal convention.
//!
//! **The falsifiable gate:** with the cross-blocks zeroed the 8 eigenvalues must
//! equal the union of the 5c (longitudinal) and 5e-i (lateral) predictions;
//! turning coupling on shifts them into genuinely coupled modes.

use crate::complex::Complex;
use crate::eigen::eigenvalues;
use crate::full_aero::{main_rotor_full, rotate6};
use crate::lateral::lateral_derivatives;
use crate::model::{Mode, hover_collective_for_weight};
use helisim_flapping::Controls;
use helisim_trim::Aircraft;
use std::f64::consts::FRAC_PI_2;

const G: f64 = 9.80665;

/// Mass and inertia properties for the 8-state model.
#[derive(Clone, Copy, Debug)]
pub struct Inertia {
    pub mass: f64,
    pub i_xx: f64,
    pub i_yy: f64,
    pub i_zz: f64,
}

/// Result of the coupled 8-state modal analysis.
#[derive(Clone, Debug)]
pub struct CoupledModal {
    /// 8×8 system matrix.
    pub a_matrix: Vec<Vec<f64>>,
    /// Eigenvalues.
    pub eigenvalues: Vec<Complex>,
    /// Classified modes.
    pub modes: Vec<Mode>,
    /// Whether coupling was enabled.
    pub coupled: bool,
}

/// Six-component force/moment derivatives wrt one input.
#[derive(Clone, Copy, Default)]
struct D6 {
    fx: f64,
    fy: f64,
    fz: f64,
    mx: f64,
    my: f64,
    mz: f64,
}

/// Assemble and solve the coupled 8-state hover model. With `coupled = false`
/// the cross blocks are zeroed (the decouple gate).
pub fn analyze_coupled_hover(ac: &Aircraft, j: Inertia, coupled: bool) -> CoupledModal {
    let coll = hover_collective_for_weight(ac);
    let rotor = ac.main.with_collective(coll);
    let f = |vel: [f64; 3], rates: [f64; 2]| {
        main_rotor_full(
            &rotor,
            &ac.main_op,
            ac.main_airfoil.as_ref(),
            &ac.flap,
            ac.hub_height,
            &Controls::none(),
            vel,
            rates,
        )
    };
    let deriv = |plus: crate::full_aero::Forces6, minus: crate::full_aero::Forces6, h: f64| D6 {
        fx: (plus.fx - minus.fx) / (2.0 * h),
        fy: (plus.fy - minus.fy) / (2.0 * h),
        fz: (plus.fz - minus.fz) / (2.0 * h),
        mx: (plus.mx - minus.mx) / (2.0 * h),
        my: (plus.my - minus.my) / (2.0 * h),
        mz: (plus.mz - minus.mz) / (2.0 * h),
    };

    // Longitudinal-input perturbations (validated machinery → on-axis AND off-axis).
    let (du, dw, dq) = (0.5, 0.5, 0.05);
    let su = deriv(
        f([du, 0.0, 0.0], [0.0, 0.0]),
        f([-du, 0.0, 0.0], [0.0, 0.0]),
        du,
    );
    let sw = deriv(
        f([0.0, 0.0, dw], [0.0, 0.0]),
        f([0.0, 0.0, -dw], [0.0, 0.0]),
        dw,
    );
    let sq = deriv(
        f([0.0, 0.0, 0.0], [0.0, dq]),
        f([0.0, 0.0, 0.0], [0.0, -dq]),
        dq,
    );

    // Longitudinal on-axis (matches 5c).
    let (xu, xw, xq) = (su.fx, sw.fx, sq.fx);
    let (zu, zw, zq) = (su.fz, sw.fz, sq.fz);
    let (mu, mw, mq) = (su.my, sw.my, sq.my);
    // Longitudinal→lateral cross (off-axis of u/w/q).
    let (yu, yw, yq) = (su.fy, sw.fy, sq.fy);
    let (lu, lw, lq) = (su.mx, sw.mx, sq.mx);
    let (nu, nw, nq) = (su.mz, sw.mz, sq.mz);

    // Lateral on-axis (5e-i, rotation-based + tail).
    let ld = lateral_derivatives(ac);

    // Lateral→longitudinal cross by the EXACT rotation of the validated
    // longitudinal response: pure-v = R₊₉₀·F(u=v), pure-p = R₋₉₀·F(q=p). Read the
    // longitudinal components (fx, fz, my) of those rotated responses.
    let rv = |s: f64| rotate6(f([s, 0.0, 0.0], [0.0, 0.0]), FRAC_PI_2);
    let rp = |s: f64| rotate6(f([0.0, 0.0, 0.0], [0.0, s]), -FRAC_PI_2);
    let (vp, vm) = (rv(du), rv(-du));
    let (pp, pm) = (rp(dq), rp(-dq));
    let (xv, zv, mv) = (
        (vp.fx - vm.fx) / (2.0 * du),
        (vp.fz - vm.fz) / (2.0 * du),
        (vp.my - vm.my) / (2.0 * du),
    );
    let (xp, zp, mp) = (
        (pp.fx - pm.fx) / (2.0 * dq),
        (pp.fz - pm.fz) / (2.0 * dq),
        (pp.my - pm.my) / (2.0 * dq),
    );

    let (m, ixx, iyy, izz) = (j.mass, j.i_xx, j.i_yy, j.i_zz);
    let k = if coupled { 1.0 } else { 0.0 }; // gate switch on cross blocks

    // Rows: u̇, ẇ, q̇, θ̇, v̇, ṗ, ṙ, φ̇ ; Cols: u, w, q, θ, v, p, r, φ.
    let a = vec![
        vec![xu / m, xw / m, xq / m, -G, k * xv / m, k * xp / m, 0.0, 0.0],
        vec![
            zu / m,
            zw / m,
            zq / m,
            0.0,
            k * zv / m,
            k * zp / m,
            0.0,
            0.0,
        ],
        vec![
            mu / iyy,
            mw / iyy,
            mq / iyy,
            0.0,
            k * mv / iyy,
            k * mp / iyy,
            0.0,
            0.0,
        ],
        vec![0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        vec![
            k * yu / m,
            k * yw / m,
            k * yq / m,
            0.0,
            ld.yv / m,
            ld.yp / m,
            ld.yr / m,
            G,
        ],
        vec![
            k * lu / ixx,
            k * lw / ixx,
            k * lq / ixx,
            0.0,
            ld.lv / ixx,
            ld.lp / ixx,
            ld.lr / ixx,
            0.0,
        ],
        vec![
            k * nu / izz,
            k * nw / izz,
            k * nq / izz,
            0.0,
            ld.nv / izz,
            ld.np / izz,
            ld.nr / izz,
            0.0,
        ],
        vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    ];

    let ev = eigenvalues(&a);
    let modes = ev.iter().map(|&e| Mode::from_eigenvalue_pub(e)).collect();
    CoupledModal {
        a_matrix: a,
        eigenvalues: ev,
        modes,
        coupled,
    }
}
