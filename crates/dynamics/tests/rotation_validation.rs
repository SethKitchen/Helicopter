//! 5f — exact rotation-based lateral aero, validated against the longitudinal
//! oracle. The lateral main-rotor response is built by rotating the *validated*
//! longitudinal response (velocity perturbations rotate +90°, rate perturbations
//! −90°). The gate: the rotation path must reproduce the longitudinal oracle
//!   Lv = −Mu,  Yv = Xu,  Lp = Mq,
//! and it must do so across a range of perturbation amplitudes (the rotation
//! construction is exact at hover, not merely tangent at the linearization point).

use helisim_dynamics::derivatives::longitudinal_derivatives;
use helisim_dynamics::model::hover_collective_for_weight;
use helisim_dynamics::{main_rollrate_response, main_rotor_full, main_velocity_response, rotate6};
use helisim_flapping::Controls;
use helisim_trim::Aircraft;

#[test]
fn rotation_reproduces_longitudinal_oracle_signs() {
    let ac = Aircraft::demo();
    let coll = hover_collective_for_weight(&ac);
    let ld = longitudinal_derivatives(&ac, coll, Controls::none());

    let v = 0.5;
    let fv = main_velocity_response(&ac, v);
    let fvm = main_velocity_response(&ac, -v);
    let lv = (fv.mx - fvm.mx) / (2.0 * v);
    let yv = (fv.fy - fvm.fy) / (2.0 * v);
    // Lateral velocity rolls the rotor opposite to how forward velocity pitches it.
    assert!(
        (lv - (-ld.mu)).abs() < 1e-3,
        "Lv {lv:.4} should equal −Mu {:.4}",
        -ld.mu
    );
    assert!(
        (yv - ld.xu).abs() < 1e-3,
        "Yv {yv:.4} should equal Xu {:.4}",
        ld.xu
    );

    let p = 0.05;
    let fp = main_rollrate_response(&ac, p);
    let fpm = main_rollrate_response(&ac, -p);
    let lp = (fp.mx - fpm.mx) / (2.0 * p);
    assert!(
        (lp - ld.mq).abs() < 1e-3,
        "Lp {lp:.4} should equal Mq {:.4}",
        ld.mq
    );
}

#[test]
fn rotation_construction_is_exact_across_amplitudes() {
    // Lv from the rotation path is the same at small and large lateral velocity
    // (linear at hover), and equals −Mu — i.e. the construction is exact, not just
    // locally tangent. (In forward flight the disk is no longer azimuthally
    // symmetric and this equality would break — that is the more-general regime
    // the rotation path, unlike the axisymmetry shortcut, is built to handle.)
    let ac = Aircraft::demo();
    let coll = hover_collective_for_weight(&ac);
    let neg_mu = -longitudinal_derivatives(&ac, coll, Controls::none()).mu;

    for &v in &[0.25_f64, 0.5, 1.0, 2.0] {
        let lv =
            (main_velocity_response(&ac, v).mx - main_velocity_response(&ac, -v).mx) / (2.0 * v);
        assert!(
            (lv - neg_mu).abs() < 0.01,
            "Lv at v=±{v}: {lv:.4} should match −Mu {neg_mu:.4}"
        );
    }
}

#[test]
fn lateral_response_is_equivariant_with_longitudinal() {
    // The deepest check: the full body-axis response to a lateral velocity is the
    // +90°-rotated response to the equal forward velocity (rotor axisymmetry).
    let ac = Aircraft::demo();
    let fv = main_velocity_response(&ac, 0.8); // already R₊₉₀·F(u=0.8) by construction
    // Side force from v equals drag-direction force from u (both the "along-flow"
    // in-plane force), and roll from v equals −(pitch from u): the rotation holds.
    let coll = hover_collective_for_weight(&ac);
    let ld = longitudinal_derivatives(&ac, coll, Controls::none());
    assert!((fv.fy / 0.8 - ld.xu).abs() < 0.05);
    assert!((fv.mx / 0.8 - (-ld.mu)).abs() < 0.05);
}

#[test]
fn full_aero_is_equivariant_for_combined_states() {
    // 5g precondition / combined-axis gate: with simultaneous v, w, p, q active,
    // rotating the whole input state about the shaft must rotate the whole
    // force/moment response by the same angle. This is the analog of 5f's
    // single-axis sweep for the combined states the nonlinear march visits —
    // where naive rotation-composition could hide a nonlinear-inflow error.
    let ac = Aircraft::demo();
    let rotor = ac.main.with_collective(hover_collective_for_weight(&ac));
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
    // A genuinely off-axis combined state.
    let (u, v, w, p, q) = (0.3_f64, 0.4, -0.2, 0.04, 0.03);
    let base = f([u, v, w], [p, q]);

    for &psi in &[0.3_f64, 0.8, 1.5, 2.5, 4.0] {
        let (s, c) = psi.sin_cos();
        let (u2, v2) = (u * c - v * s, u * s + v * c); // rotate velocity
        let (p2, q2) = (p * c - q * s, p * s + q * c); // rotate rate vector
        let rotated_state = f([u2, v2, w], [p2, q2]);
        let rotated_response = rotate6(base, psi);
        for (a, b, name) in [
            (rotated_state.fx, rotated_response.fx, "fx"),
            (rotated_state.fy, rotated_response.fy, "fy"),
            (rotated_state.fz, rotated_response.fz, "fz"),
            (rotated_state.mx, rotated_response.mx, "mx"),
            (rotated_state.my, rotated_response.my, "my"),
            (rotated_state.mz, rotated_response.mz, "mz"),
        ] {
            assert!(
                (a - b).abs() < 1e-6,
                "ψ={psi} {name}: {a} vs {b} (non-equivariant)"
            );
        }
    }
}
