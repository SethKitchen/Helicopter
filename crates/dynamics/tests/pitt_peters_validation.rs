//! Milestone 5h validation — Pitt–Peters dynamic inflow.
//!
//! The inflow states are INTERNAL (no standalone cross-check oracle), so the two
//! clean gates carry the validation:
//!
//! * **Gate 1 (τ→0, exact & falsifiable):** zeroing the new cyclic states recovers
//!   the validated 5g baseline bit-for-bit; and marching the dynamic inflow with
//!   `lag → 0` collapses onto the quasi-static fixed point.
//! * **Gate 2 (documented signature):** the cyclic inflow shifts the OFF-axis
//!   (pitch↔roll) response to cyclic — the classic dynamic-inflow result.

use helisim_dynamics::{
    RotorAero, main_rotor_full, main_rotor_with_inflow, march_inflow, quasi_static_inflow,
    uniform_inflow,
};
use helisim_flapping::Controls;
use helisim_rotor::Rotor;
use helisim_trim::Aircraft;

/// Bundle the main-rotor context for the calls below.
fn aero<'a>(ac: &'a Aircraft, rotor: &'a Rotor, controls: &'a Controls) -> RotorAero<'a> {
    RotorAero {
        rotor,
        op: &ac.main_op,
        airfoil: ac.main_airfoil.as_ref(),
        props: &ac.flap,
        hub_height: ac.hub_height,
        controls,
    }
}

fn hover_setup() -> (Aircraft, f64) {
    let ac = Aircraft::demo();
    let coll = helisim_dynamics::hover_collective_for_weight(&ac);
    (ac, coll)
}

/// Gate 1a — zeroing the cyclic inflow states recovers the 5g uniform-inflow
/// baseline EXACTLY (the "turn the new physics off via its own parameter").
#[test]
fn zeroing_cyclic_inflow_recovers_baseline_exactly() {
    let (ac, coll) = hover_setup();
    let rotor = ac.main.with_collective(coll);
    let c = Controls {
        theta_1c: 0.01,
        theta_1s: -0.008,
    };
    // A non-trivial flight condition so every force/moment component is exercised.
    let vel = [3.0, 1.5, -0.5];
    let rates = [0.05, -0.03];

    let baseline = main_rotor_full(&aero(&ac, &rotor, &c), vel, rates);
    let li = uniform_inflow(&aero(&ac, &rotor, &c), vel, rates);
    let (recovered, _) =
        main_rotor_with_inflow(&aero(&ac, &rotor, &c), vel, rates, [li, 0.0, 0.0]);

    for (a, b) in [
        (baseline.fx, recovered.fx),
        (baseline.fy, recovered.fy),
        (baseline.fz, recovered.fz),
        (baseline.mx, recovered.mx),
        (baseline.my, recovered.my),
        (baseline.mz, recovered.mz),
    ] {
        assert!(
            (a - b).abs() < 1e-12,
            "baseline {a} vs zero-cyclic {b} must be bit-exact"
        );
    }
}

/// Gate 1b — marching the dynamic inflow with `lag → 0` collapses onto the
/// quasi-static fixed point (the τ→0 reduction, falsifiable).
#[test]
fn dynamic_inflow_reduces_to_quasi_static_as_lag_goes_to_zero() {
    let (ac, coll) = hover_setup();
    let rotor = ac.main.with_collective(coll);
    let c = Controls {
        theta_1c: 0.012,
        theta_1s: 0.0,
    };
    let vel = [4.0, 0.0, 0.0];
    let rates = [0.0, 0.0];

    let (f_qs, nu_qs, _) = quasi_static_inflow(&aero(&ac, &rotor, &c), vel, rates);

    // March from a deliberately wrong inflow; tiny lag ⇒ snaps to quasi-static.
    let (nu_dyn, f_dyn) = march_inflow(
        &aero(&ac, &rotor, &c),
        vel,
        rates,
        [0.02, 0.02, -0.02],
        1e-4,
        1e-6,
        4000,
    );

    let dnu: f64 = (0..3)
        .map(|i| (nu_dyn[i] - nu_qs[i]).abs())
        .fold(0.0, f64::max);
    let dmom = (f_dyn.mx - f_qs.mx).abs().max((f_dyn.my - f_qs.my).abs());
    println!("Gate1b: ν_qs={nu_qs:?} ν_dyn(lag→0)={nu_dyn:?} Δν={dnu:.2e} Δmoment={dmom:.2e} N·m");
    assert!(
        dnu < 1e-4,
        "lag→0 inflow must match quasi-static, Δν={dnu:.2e}"
    );
    assert!(
        dmom < 1e-3,
        "lag→0 moments must match quasi-static, Δ={dmom:.2e}"
    );
}

/// Gate 2 — the cyclic inflow shifts the OFF-axis cyclic response. At hover a
/// lateral cyclic θ1c primarily commands roll (on-axis); the off-axis pitch
/// response is what dynamic inflow famously corrects. We measure the off-axis
/// hub-moment derivative with the cyclic inflow FROZEN (pre-5h) vs SOLVED
/// (Pitt–Peters quasi-static) and confirm it shifts in the documented direction.
#[test]
fn cyclic_inflow_shifts_off_axis_response() {
    let (ac, coll) = hover_setup();
    let rotor = ac.main.with_collective(coll);
    let vel = [0.0, 0.0, 0.0];
    let rates = [0.0, 0.0];
    let d = 0.01_f64; // lateral cyclic step, rad

    // λ₀ baseline (cyclic inflow zero) for the frozen-inflow comparison.
    let base = Controls {
        theta_1c: 0.0,
        theta_1s: 0.0,
    };
    let li = uniform_inflow(&aero(&ac, &rotor, &base), vel, rates);

    let off_axis = |c: &Controls, frozen: bool| -> f64 {
        if frozen {
            let (f, _) =
                main_rotor_with_inflow(&aero(&ac, &rotor, c), vel, rates, [li, 0.0, 0.0]);
            f.my // off-axis (pitch) moment for a lateral-cyclic (roll) input
        } else {
            let (f, _, _) = quasi_static_inflow(&aero(&ac, &rotor, c), vel, rates);
            f.my
        }
    };

    let cp = Controls {
        theta_1c: d,
        theta_1s: 0.0,
    };
    let cm = Controls {
        theta_1c: -d,
        theta_1s: 0.0,
    };

    let off_frozen = (off_axis(&cp, true) - off_axis(&cm, true)) / (2.0 * d);
    let off_inflow = (off_axis(&cp, false) - off_axis(&cm, false)) / (2.0 * d);
    // On-axis for context.
    let on = |c: &Controls| {
        let (f, _, _) = quasi_static_inflow(&aero(&ac, &rotor, c), vel, rates);
        f.mx
    };
    let on_inflow = (on(&cp) - on(&cm)) / (2.0 * d);

    println!(
        "Gate2: off-axis ∂My/∂θ1c  frozen={off_frozen:.1}  with-inflow={off_inflow:.1}  \
         (on-axis ∂Mx/∂θ1c={on_inflow:.1})"
    );
    // The on-axis response is large (the primary commanded moment); the off-axis
    // is the small coupling dynamic inflow famously corrects.
    assert!(
        on_inflow.abs() > 10.0 * off_inflow.abs(),
        "on-axis must dominate off-axis"
    );
    // The documented signature: the cyclic inflow FLIPS THE SIGN of the off-axis
    // response (the classic "wrong sign of off-axis response to cyclic"). This is
    // the headline emerging on its own — not a tuned target.
    assert!(
        off_frozen < 0.0,
        "frozen-inflow off-axis is negative (the pre-5h sign)"
    );
    assert!(
        off_inflow > 0.0,
        "cyclic inflow flips the off-axis sign — the documented result"
    );
}
