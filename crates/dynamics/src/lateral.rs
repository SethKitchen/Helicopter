//! Lateral-directional linear analysis about hover (the 5e-i oracle).
//!
//! States `[v, p, r, φ]` (side velocity, roll rate, yaw rate, roll attitude).
//! Derivatives come from the main rotor (Lv, Lp — the dihedral and roll damping,
//! by the rotor's axisymmetry the lateral twins of Mu, Mq) and the **tail rotor**,
//! which enters here for the first time as a *dynamic* element: it sets Yv, Nv,
//! Nr (yaw damping) through its thrust response to the lateral velocity it sees.
//!
//! **Tail-rotor height is included** (named decision): the tail rotor sits
//! `h_tr` above the CG, so its thrust makes a rolling moment `h_tr·T_tr` as well
//! as the yaw `−l_tr·T_tr`. The lateral velocity at the tail along its thrust
//! axis is `v_axial = v + p·h_tr − r·l_tr`; its thrust falls as that velocity
//! rises (the tail "climbing" into its own thrust), which gives Nr<0, Yv<0, and
//! the roll-yaw coupling.

use crate::aero::longitudinal_main_aero;
use crate::complex::Complex;
use crate::eigen::eigenvalues;
use crate::full_aero::{Forces6, main_rotor_full, rotate6};
use crate::model::{Mode, hover_collective_for_weight};
use helisim_airfoil::Airfoil;
use helisim_flapping::Controls;
use helisim_flapping::FlapProperties;
use helisim_rotor::{Operating, Rotor};
use helisim_trim::{Aircraft, NewtonConfig, TrimCondition, trim};
use std::f64::consts::FRAC_PI_2;

const G: f64 = 9.80665;

/// Main-rotor response to a pure lateral velocity `v`, by the exact rotation of
/// the validated longitudinal response: a velocity perturbation rotates by +90°
/// (`u → v`), so the response is `R₊₉₀·F(u=v)`.
fn main_pure_v(
    rotor: &Rotor,
    op: &Operating,
    af: &dyn Airfoil,
    flap: &FlapProperties,
    h: f64,
    v: f64,
) -> Forces6 {
    rotate6(
        main_rotor_full(
            rotor,
            op,
            af,
            flap,
            h,
            &Controls::none(),
            [v, 0.0, 0.0],
            [0.0, 0.0],
        ),
        FRAC_PI_2,
    )
}

/// Main-rotor response to a pure roll rate `p`, by rotation of the validated
/// longitudinal pitch-rate response: an angular-rate perturbation rotates by −90°
/// (`q → p`), so the response is `R₋₉₀·F(q=p)`.
fn main_pure_p(
    rotor: &Rotor,
    op: &Operating,
    af: &dyn Airfoil,
    flap: &FlapProperties,
    h: f64,
    p: f64,
) -> Forces6 {
    rotate6(
        main_rotor_full(
            rotor,
            op,
            af,
            flap,
            h,
            &Controls::none(),
            [0.0, 0.0, 0.0],
            [0.0, p],
        ),
        -FRAC_PI_2,
    )
}

/// Lateral-directional stability derivatives (dimensional).
#[derive(Clone, Copy, Debug)]
pub struct LateralDerivatives {
    /// ∂Y/∂v — side-force damping (< 0).
    pub yv: f64,
    pub yp: f64,
    pub yr: f64,
    /// ∂L/∂v — dihedral effect.
    pub lv: f64,
    /// ∂L/∂p — roll damping (< 0).
    pub lp: f64,
    pub lr: f64,
    pub nv: f64,
    pub np: f64,
    /// ∂N/∂r — yaw damping from the tail rotor (< 0).
    pub nr: f64,
}

/// Tail-rotor thrust as a function of the lateral velocity along its thrust axis.
/// Modelled as a rotor in axial flow: positive `v_axial` (with the thrust) is a
/// climb that reduces thrust, so `dT/dv_axial < 0`. Public for the 8-state march.
pub fn tail_thrust(ac: &Aircraft, tail_collective: f64, v_axial: f64) -> f64 {
    let rotor = ac.tail.rotor.with_collective(tail_collective);
    // Map axial velocity (along +thrust) to the rotor's "climb" = heave w<0.
    longitudinal_main_aero(
        &rotor,
        &ac.tail.op,
        ac.tail.airfoil.as_ref(),
        &ac.flap,
        0.0,
        &Controls::none(),
        0.0,
        -v_axial,
        0.0,
    )
    .thrust
}

/// Main-rotor body forces/moments for a pure lateral velocity `v` (rotation of
/// the validated longitudinal response). Public for the 5f cross-check.
pub fn main_velocity_response(ac: &Aircraft, v: f64) -> Forces6 {
    let rotor = ac.main.with_collective(hover_collective_for_weight(ac));
    main_pure_v(
        &rotor,
        &ac.main_op,
        ac.main_airfoil.as_ref(),
        &ac.flap,
        ac.hub_height,
        v,
    )
}

/// Main-rotor body forces/moments for a pure roll rate `p`. Public for the 5f
/// cross-check.
pub fn main_rollrate_response(ac: &Aircraft, p: f64) -> Forces6 {
    let rotor = ac.main.with_collective(hover_collective_for_weight(ac));
    main_pure_p(
        &rotor,
        &ac.main_op,
        ac.main_airfoil.as_ref(),
        &ac.flap,
        ac.hub_height,
        p,
    )
}

/// Compute the lateral-directional derivatives about hover.
///
/// Main rotor: computed by **exact rotation of the validated longitudinal aero**
/// — `Yv=Xu, Lv=−Mu` (velocity rotates +90°), `Yp=−Xq, Lp=Mq` (rate rotates −90°).
/// The −90° vs +90° (rate vs velocity) is what makes `Lv` *negative* (the earlier
/// axisymmetry-by-assertion used `+Mu` and was wrong, which had made the lateral
/// mode look like a divergence instead of the true oscillation).
///
/// Tail rotor: thrust `T_tr(v_axial)`, `v_axial = v + p·h_tr − r·l_tr`,
/// contributing side force, roll `h_tr·T_tr`, and yaw `−l_tr·T_tr`. The slope
/// `G_t = dT_tr/dv_axial < 0` sets all tail terms, including the roll-yaw coupling
/// from the tail height.
pub fn lateral_derivatives(ac: &Aircraft) -> LateralDerivatives {
    let main_c = hover_collective_for_weight(ac);
    let tail_c = trim(ac, &TrimCondition::hover(), &NewtonConfig::default()).tail_collective;
    let rotor = ac.main.with_collective(main_c);
    let af = ac.main_airfoil.as_ref();

    // Main rotor via rotation of the validated longitudinal response.
    let (dv, dp) = (0.5, 0.05);
    let vp = main_pure_v(&rotor, &ac.main_op, af, &ac.flap, ac.hub_height, dv);
    let vm = main_pure_v(&rotor, &ac.main_op, af, &ac.flap, ac.hub_height, -dv);
    let pp = main_pure_p(&rotor, &ac.main_op, af, &ac.flap, ac.hub_height, dp);
    let pm = main_pure_p(&rotor, &ac.main_op, af, &ac.flap, ac.hub_height, -dp);
    let (yv_m, lv_m, nv_m) = (
        (vp.fy - vm.fy) / (2.0 * dv),
        (vp.mx - vm.mx) / (2.0 * dv),
        (vp.mz - vm.mz) / (2.0 * dv),
    );
    let (yp_m, lp_m, np_m) = (
        (pp.fy - pm.fy) / (2.0 * dp),
        (pp.mx - pm.mx) / (2.0 * dp),
        (pp.mz - pm.mz) / (2.0 * dp),
    );

    // Tail-rotor thrust slope vs axial velocity.
    let dva = 0.5;
    let g_t = (tail_thrust(ac, tail_c, dva) - tail_thrust(ac, tail_c, -dva)) / (2.0 * dva);
    let (l, h) = (ac.tail.arm, ac.tail.height);

    LateralDerivatives {
        yv: yv_m + g_t,
        lv: lv_m + h * g_t,
        nv: nv_m - l * g_t,
        yp: yp_m + h * g_t,
        lp: lp_m + h * h * g_t,
        np: np_m - l * h * g_t,
        yr: -l * g_t,
        lr: -l * h * g_t,
        nr: l * l * g_t,
    }
}

/// Lateral modal analysis result.
#[derive(Clone, Debug)]
pub struct LateralModal {
    pub derivatives: LateralDerivatives,
    /// 4×4 system matrix, states `[v, p, r, φ]`.
    pub a_matrix: Vec<Vec<f64>>,
    pub eigenvalues: Vec<Complex>,
    pub modes: Vec<Mode>,
    pub has_unstable_oscillation: bool,
    pub unstable: bool,
}

/// Assemble and solve the lateral-directional system. `i_xx` is roll inertia,
/// `i_zz` yaw inertia.
pub fn analyze_hover_lateral(ac: &Aircraft, i_xx: f64, i_zz: f64) -> LateralModal {
    let d = lateral_derivatives(ac);
    let m = ac.mass;
    // States [v, p, r, φ]; v̇ has +g·φ (roll tips gravity sideways), φ̇ = p.
    let a = vec![
        vec![d.yv / m, d.yp / m, d.yr / m, G],
        vec![d.lv / i_xx, d.lp / i_xx, d.lr / i_xx, 0.0],
        vec![d.nv / i_zz, d.np / i_zz, d.nr / i_zz, 0.0],
        vec![0.0, 1.0, 0.0, 0.0],
    ];
    let ev = eigenvalues(&a);
    let modes: Vec<Mode> = ev.iter().map(|&e| Mode::from_eigenvalue_pub(e)).collect();
    let unstable = modes.iter().any(|m| m.eigenvalue.re > 1e-6);
    let has_unstable_oscillation = modes
        .iter()
        .any(|m| m.oscillatory && m.eigenvalue.re > 1e-6);
    LateralModal {
        derivatives: d,
        a_matrix: a,
        eigenvalues: ev,
        modes,
        has_unstable_oscillation,
        unstable,
    }
}

/// Reduced lateral roll-sideslip characteristic cubic (dropping yaw r), the
/// analytic anchor for the eigen-routine — the lateral twin of the hovering
/// cubic: `s³ + a s² + b s + c` with
/// `a = −(Yv/m + Lp/Ixx)`, `b = (Yv·Lp − Yp·Lv)/(m·Ixx)`, `c = −g·Lv/Ixx`.
pub fn lateral_cubic(d: &LateralDerivatives, mass: f64, i_xx: f64) -> [f64; 4] {
    let a = -(d.yv / mass + d.lp / i_xx);
    let b = (d.yv * d.lp - d.yp * d.lv) / (mass * i_xx);
    let c = -G * d.lv / i_xx;
    [1.0, a, b, c]
}
