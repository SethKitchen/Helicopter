//! Milestone 6 — external validation of the hover longitudinal stability
//! derivatives against the UH-60A, the first time the model meets ground truth.
//!
//! Oracle: NASA TM 85890 (GENHEL), Table 13 (M-moment) and Table 12 (X/Z-force)
//! "BOEING-VERTOL UH-60 MATH MODEL", hover (0.5-knot) column unless noted; values
//! converted to SI here (the conversion done ONCE — see asserts). The aircraft is
//! `Aircraft::uh60()`, built strictly from the locked parameter mapping
//! (`crates/validation/MILESTONE6_PARAMETER_MAPPING.md`); no parameter was tuned to
//! this comparison.
//!
//! Predictions locked before the run (`MILESTONE6_PREDICTIONS.md`): the force/moment
//! derivatives match in sign + order (~20–35% from uniform inflow + rigid blade).
//! What the comparison actually found is encoded as the test's expectations, per the
//! "be honest about model error" rule — including the one predicted-clean derivative
//! that came out NOT clean (Mq).

use helisim_dynamics::{
    hover_collective_for_weight, lateral_derivatives, longitudinal_derivatives,
};
use helisim_flapping::Controls;
use helisim_trim::Aircraft;

const IYY: f64 = 40000.0 * 1.355_82; // UH-60 pitch inertia, slug-ft² → kg·m² = 54 233
const FT_PER_M: f64 = 3.280_84; // moment-from-velocity derivatives: 1/(ft·s) → 1/(m·s)

#[test]
fn uh60_hover_longitudinal_derivatives_vs_genhel() {
    let ac = Aircraft::uh60();
    let coll = hover_collective_for_weight(&ac); // trim converged (≈19.1° root, washout)
    let d = longitudinal_derivatives(&ac, coll, Controls::none());
    let (m, iyy) = (ac.mass, IYY);

    // Normalize raw force/moment derivatives to GENHEL's inertia-normalized form.
    let xu = d.xu / m; // 1/s (unit-invariant)
    let zw = d.zw / m; // 1/s
    let mu = d.mu / iyy; // rad/(m·s)
    let mq = d.mq / iyy; // 1/s

    // Oracle (SI). Hover (0.5 kt) where trustworthy; 20 kt where the hover column is
    // degenerate (Mq=+0.77 positive, Mu anomalously tiny — Boeing-Vertol near-zero-V).
    let (xu_o, zw_o): (f64, f64) = (-0.0150, -0.2748); // 1/s, hover
    let mu_o_hover = 0.0005 * FT_PER_M; // +0.00164 rad/(m·s)  (hover, anomalously small)
    let mu_o_20kt = 0.0091 * FT_PER_M; // +0.0299 rad/(m·s)

    // --- SIGNS: the robust external check (these emerged unprompted in 5c and must
    // match the real aircraft). Mu>0 is the destabilizing speed stability the whole
    // hover instability rests on. ---
    assert!(xu < 0.0, "Xu<0 drag damping");
    assert!(zw < 0.0, "Zw<0 heave damping");
    assert!(
        mu > 0.0,
        "Mu>0 — destabilizing speed stability, matches the real UH-60"
    );
    assert!(mq < 0.0, "Mq<0 pitch damping");

    // --- Zw: the headline MATCH. Heave damping is main-rotor-dominated and clean;
    // it lands within the predicted band. ---
    let zw_err = (zw - zw_o).abs() / zw_o.abs();
    assert!(
        zw_err < 0.20,
        "Zw within 20% of GENHEL (got {:.0}%)",
        zw_err * 100.0
    );

    // --- Xu: small, noisy drag-damping term (the oracle itself wanders −0.015→+0.018
    // across speed); ours is the right sign and order. ---
    assert!(
        xu.abs() < 2.0 * xu_o.abs().max(0.03),
        "Xu same order as GENHEL"
    );

    // --- Mu: right sign and order. The oracle hover point is anomalously small, so a
    // clean % is ill-defined; ours (0.010) sits inside the oracle's low-speed range
    // [0.0016 hover .. 0.030 at 20 kt]. The destabilizing physics is externally present. ---
    assert!(
        mu > 0.5 * mu_o_hover && mu < 1.5 * mu_o_20kt,
        "Mu in the oracle low-speed range"
    );

    // --- Mq: the Milestone-6 finding, now FIXED. The gyroscopic hub-rate→flap term
    // (aero.rs::GYRO_RATE, derived not fitted) restored the in-phase β1c response, so
    // pitch damping is no longer 1–2 orders short — Mq ≈ −0.45, right sign, substantial.
    // The clean magnitude check is the ROLL axis (Lp, below): hover Mq itself can't be
    // cleanly matched here (oracle hover Mq is degenerate +0.77, and the 20-kt −1.03
    // carries the horizontal stabilator + forward-flight effects this hover-main-rotor
    // model omits — the residual is that, not the flap). ---
    assert!(
        mq < -0.25,
        "Mq now substantial (gyro fix); was ~−0.016 (15–65× short)"
    );
    println!(
        "UH-60 hover (SI): Xu {xu:+.4} (o -0.0150) | Zw {zw:+.4} (o -0.2748, {:.0}% ✓) | \
         Mu {mu:+.4} (o +0.0016..+0.030) | Mq {mq:+.4} (was -0.016; gyro fix → -0.45; \
         residual vs -1.03(20kt) is the stabilator)",
        zw_err * 100.0
    );
}

const IXX: f64 = 5629.0 * 1.355_82; // 7 632 kg·m²
const IZZ: f64 = 37200.0 * 1.355_82; // 50 437 kg·m²

#[test]
fn uh60_lateral_directional_discriminator_vs_genhel() {
    // The discriminating experiment (pre-registered in MILESTONE6_RESULTS.md): does the
    // hover damping deficit travel with the main-rotor flap across axes, or is it
    // pitch-specific? Roll is the CLEAN test — oracle Lp is non-degenerate and roll has
    // no horizontal-stabilator confound. Oracle: NASA TM 85890 Tables 12/14/15, 0.5 kt.
    let ac = Aircraft::uh60();
    let d = lateral_derivatives(&ac);
    let (m, ixx, izz) = (ac.mass, IXX, IZZ);
    let (lp, nr, yv) = (d.lp / ixx, d.nr / izz, d.yv / m);
    let nv = d.nv / izz;

    let (lp_o, nr_o, yv_o): (f64, f64, f64) = (-3.3484, -0.2879, -0.0465); // 1/s, hover
    let nv_o = 0.0081 * FT_PER_M; // +0.0266 rad/(m·s)

    // Signs first.
    assert!(
        lp < 0.0 && nr < 0.0 && yv < 0.0,
        "roll/yaw damping and side-force damping all <0"
    );
    assert!(nv > 0.0, "Nv>0 weathercock yaw stiffness");

    // Nr — tail-rotor-based (NO main-rotor flap): lands on the oracle. This proves the
    // moment-assembly / normalization / units machinery is correct, definitively ruling
    // out the "boring bug" for the whole comparison (a units bug couldn't let Nr land).
    let nr_err = (nr - nr_o).abs() / nr_o.abs();
    assert!(
        nr_err < 0.10,
        "Nr (tail-based) matches GENHEL within 10% (got {:.0}%)",
        nr_err * 100.0
    );

    // Yv, Nv (not main-rotor-flap-damping): land within band.
    assert!((yv - yv_o).abs() / yv_o.abs() < 0.20, "Yv within 20%");
    assert!((nv - nv_o).abs() / nv_o.abs() < 0.30, "Nv within 30%");

    // Lp — main-rotor-flap rate damping, the CLEAN test of the gyroscopic fix (oracle
    // Lp is non-degenerate; roll has no stabilator confound). It was ~17× too small
    // (−0.19); the derived gyro term (coefficient 2, physics-mandated sign, NOT fitted)
    // brings it to −3.25 vs the oracle −3.35 — ~3%, well within the model's accuracy
    // band. This is the headline confirmation: a term derived from first principles,
    // not tuned, closed a deficit external validation revealed, and (verified
    // separately) left every velocity derivative and Nr bit-for-bit unchanged.
    let lp_err = (lp - lp_o).abs() / lp_o.abs();
    assert!(
        lp_err < 0.15,
        "Lp matches GENHEL within 15% after the gyro fix (got {:.0}%)",
        lp_err * 100.0
    );

    println!(
        "UH-60 hover lateral (SI): Nr {nr:+.4} (o -0.288, {:.1}% ✓ tail) | \
         Yv {yv:+.4} (o -0.0465) | Nv {nv:+.4} (o +0.0266) | \
         Lp {lp:+.4} (o -3.35, {:.0}% ✓ — gyro fix, was ~17× short)",
        nr_err * 100.0,
        lp_err * 100.0
    );
}

#[test]
fn cg_offset_does_not_affect_the_derivatives() {
    // Load-bearing claim, MEASURED not asserted (the measurement corrected the
    // assertion). cg_offset, set to the sourced 0.488 m for the trim-attitude
    // comparison, must not move the validated hover derivatives. Compare 0.488 vs 0.
    let a_set = Aircraft::uh60(); // cg_offset = 0.488
    let mut a_zero = Aircraft::uh60();
    a_zero.cg_offset = 0.0;
    let ds = longitudinal_derivatives(
        &a_set,
        hover_collective_for_weight(&a_set),
        Controls::none(),
    );
    let dz = longitudinal_derivatives(
        &a_zero,
        hover_collective_for_weight(&a_zero),
        Controls::none(),
    );
    let ls = lateral_derivatives(&a_set);
    let lz = lateral_derivatives(&a_zero);

    // Longitudinal: BIT-FOR-BIT identical — longitudinal_main_aero/hover_collective use
    // neither cg_offset nor trim, so there is exactly nothing for cg_offset to touch.
    assert_eq!(
        (ds.xu, ds.zw, ds.mu, ds.mq),
        (dz.xu, dz.zw, dz.mu, dz.mq),
        "longitudinal exact"
    );

    // Lateral: NOT bit-for-bit — `lateral_derivatives` re-trims for the tail collective,
    // which has a vanishing dependence on the longitudinal CG offset (2nd-order coupling
    // in the 6-var trim). The change is ~1e-5 relative (6th significant figure), far below
    // the validated comparison (Lp 3%, Nr 1.5%), but REAL — so a tolerance, not equality.
    // (The honest, measured version of "cg_offset is trim-only".)
    let rel = |a: f64, b: f64| (a - b).abs() / b.abs().max(1e-12);
    let lat_max = rel(ls.lp, lz.lp)
        .max(rel(ls.nr, lz.nr))
        .max(rel(ls.yv, lz.yv));
    assert!(
        lat_max < 1e-4,
        "lateral changes only at ~1e-5 via the tail-trim coupling"
    );
    println!(
        "cg_offset 0.488 vs 0: longitudinal bit-for-bit identical; lateral Δ ≤ {lat_max:.0e} rel \
         (tail-trim coupling, negligible vs the 3% / 1.5% comparison)"
    );
}
