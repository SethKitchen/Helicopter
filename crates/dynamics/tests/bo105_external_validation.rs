//! Milestone 6 — SECOND airframe: BO-105C hover derivatives vs NASA CR-3144 (Heffley).
//!
//! The cross-aircraft test. Oracle: CR-3144 Vol 1, Table III-3, CASE 29 (0 KT, sea level,
//! 2096 kg, mid CG), body-fixed FRL axis, SI units, inertia/mass-normalized. Aircraft:
//! `Aircraft::bo105()`, built strictly from the locked mapping (MILESTONE6_BO105_PREREG.md);
//! no parameter tuned to this comparison. Predictions locked before the run in that file.
//!
//! HEADLINE (P1): does the gyroscopic flap-damping term `gyro_rate=−2`, derived & validated
//! on the ARTICULATED UH-60, generalize to this HINGELESS rotor with parameter changes
//! alone? Tested as an A/B (gyro 0 vs −2) — ungameable by the single-source glimpse, since
//! params and the −2 coefficient are both locked.

use helisim_dynamics::{
    hover_collective_for_weight, lateral_derivatives, longitudinal_derivatives,
};
use helisim_flapping::Controls;
use helisim_trim::Aircraft;

// CR-3144 Fig III-3b, CASE-29 nominal loading — SI directly (no English conversion).
const MASS: f64 = 2096.0;
const IXX: f64 = 1803.0;
const IYY: f64 = 4892.0;
const IZZ: f64 = 4428.0;

// Oracle hover derivatives (CASE 29), SI, normalized as CR-3144 reports them.
const MQ_O: f64 = -3.3972;
const LP_O: f64 = -9.2439;
const ZW_O: f64 = -0.3317;
const XU_O: f64 = -0.0166;
const MU_O: f64 = 0.0663; // rad/(m·s) — SI, no ft→m conversion
const YV_O: f64 = -0.0320;
const NV_O: f64 = 0.0325;
const NR_O: f64 = -0.3270;

fn mq_lp(ac: &Aircraft) -> (f64, f64) {
    let coll = hover_collective_for_weight(ac);
    let dl = longitudinal_derivatives(ac, coll, Controls::none());
    let dt = lateral_derivatives(ac);
    (dl.mq / IYY, dt.lp / IXX)
}

#[test]
fn bo105_hover_gyro_generalization_ab() {
    // ARM A: gyro OFF — predict Mq/Lp far too small (the UH-60 pre-fix deficit REAPPEARS
    // on a 2nd, hingeless airframe ⇒ model-general gap, not UH-60-specific).
    let mut a = Aircraft::bo105();
    a.flap.gyro_rate = 0.0;
    let (mq_a, lp_a) = mq_lp(&a);

    // ARM B: gyro = −2 (the UH-60 value, UNCHANGED) — predict Mq/Lp move into the
    // order-consistent band (~2×, Heffley tolerance).
    let b = Aircraft::bo105(); // gyro = −2
    let (mq_b, lp_b) = mq_lp(&b);

    println!("BO-105 hover GYRO A/B (SI, normalized):");
    println!(
        "  Lp: gyro0 {lp_a:+.3} | gyro-2 {lp_b:+.3} | oracle {LP_O:+.3}  (ratios {:.2}× / {:.2}×)",
        lp_a / LP_O,
        lp_b / LP_O
    );
    println!(
        "  Mq: gyro0 {mq_a:+.3} | gyro-2 {mq_b:+.3} | oracle {MQ_O:+.3}  (ratios {:.2}× / {:.2}×)",
        mq_a / MQ_O,
        mq_b / MQ_O
    );

    // TWO HONESTY NOTES baked in:
    // (1) TOO-GOOD FLAG (locked hygiene): 0.99× on an analysis-program oracle, with a
    //     pre-registered ~2× tolerance, is suspiciously perfect (the UH-60 Lp was already
    //     flagged at 3%). The robust claim is NOT "1% accurate" — it is the A→B
    //     generalization + order-consistency ACROSS the ν_β bracket (other test). The gate
    //     here is therefore the WIDE band, deliberately not tightened to celebrate 0.99×.
    // (2) Mq and Lp both ≈0.99× is ONE match, not two: by the 5f rotation symmetry Lp=Mq
    //     (raw moment); oracle Lp/Mq = 2.72 ≈ Iyy/Ixx = 2.71, so it is the same raw flap
    //     rate-damping moment seen through two inertias.
    let lp_mq_oracle = LP_O / MQ_O; // 2.72
    let lp_mq_inertia = IYY / IXX; // 2.71
    assert!(
        (lp_mq_oracle - lp_mq_inertia).abs() < 0.1,
        "Lp/Mq tracks Iyy/Ixx — one rate-damping moment"
    );

    // Arm A: the deficit reappears (far short on the clean roll axis).
    assert!(
        lp_a.abs() < 0.5 * LP_O.abs(),
        "gyro-off Lp far short (deficit reappears on hingeless)"
    );
    // Arm B: gyro term lifts damping substantially toward the oracle, and on the CLEAN
    // roll axis lands order-consistent (WIDE band — not tightened to the 0.99× too-good).
    assert!(
        lp_b.abs() > 1.5 * lp_a.abs(),
        "gyro term substantially increases roll damping"
    );
    let lp_ratio = lp_b / LP_O;
    assert!(
        (0.5..2.0).contains(&lp_ratio),
        "Lp order-consistent w/ oracle after gyro (got {lp_ratio:.2}×)"
    );
    assert!(
        mq_b.abs() > mq_a.abs(),
        "gyro term increases pitch damping too"
    );
}

#[test]
fn bo105_hover_collective_bemt_third_sighting() {
    // P4: BEMT C_T over-prediction — THIRD independent sighting (after C&T hover C_T and
    // the UH-60 trim collective). Oracle CASE 29 trim main collective θMR = 14.32°. Predict
    // mine LOWER (BEMT over-predicts thrust ⇒ thrust=weight met at less collective).
    // DIRECTION is the commit; magnitude is path/config-dependent (tip-loss caveat), NOT a
    // clean scalar — exactly as characterized for the UH-60.
    let ac = Aircraft::bo105();
    let coll = hover_collective_for_weight(&ac).to_degrees();
    let oracle = 14.32; // θMR, CASE 29
    let lower = coll < oracle;
    println!(
        "BO-105 hover collective: mine {coll:.2}° vs oracle θMR {oracle:.2}° ({:.0}% {})",
        ((oracle - coll) / oracle * 100.0).abs(),
        if lower {
            "lower → BEMT over-predicts thrust (3rd sighting)"
        } else {
            "HIGHER — breaks the pattern"
        }
    );
    assert!(
        lower,
        "collective lower than oracle (BEMT over-prediction direction, 3rd sighting)"
    );
}

#[test]
fn bo105_gyro_conclusion_holds_across_the_nu_beta_bracket_and_hub_height() {
    // CR-3144 omits ν_β; the gyro conclusion must hold across the hingeless bracket
    // [1.08,1.15] (e∈[0.0999,0.177]) — robust to the sourcing gap, not a single tuned ν_β.
    // Also confirm the headline is insensitive to the un-sourced hub_height (hub-spring
    // dominated): vary it widely and check Lp barely moves.
    println!("BO-105 Lp(gyro=−2) across the ν_β bracket (oracle {LP_O:+.3}):");
    for &e in &[0.0999, 0.145, 0.177] {
        let mut ac = Aircraft::bo105();
        ac.flap.hinge_offset = e;
        let nu = ac.flap.nu_beta_sq().sqrt();
        let (_, lp) = mq_lp(&ac);
        println!("  e={e:.4} (ν_β={nu:.3}): Lp {lp:+.3}  ({:.2}×)", lp / LP_O);
        let r = lp / LP_O;
        assert!(
            (0.4..2.5).contains(&r),
            "Lp order-consistent across the bracket (e={e}, got {r:.2}×)"
        );
    }
    println!("BO-105 Lp(gyro=−2) across hub_height (oracle {LP_O:+.3}):");
    let mut lps = vec![];
    for &h in &[0.5, 0.95, 1.5] {
        let mut ac = Aircraft::bo105();
        ac.hub_height = h;
        let (_, lp) = mq_lp(&ac);
        println!("  hub_height={h:.2} m: Lp {lp:+.3}",);
        lps.push(lp);
    }
    let spread = (lps.iter().cloned().fold(f64::MIN, f64::max)
        - lps.iter().cloned().fold(f64::MAX, f64::min))
    .abs()
        / lps[1].abs();
    assert!(
        spread < 0.15,
        "Lp insensitive to the un-sourced hub_height (spread {spread:.0e})"
    );
}

#[test]
fn bo105_hover_velocity_derivatives_vs_cr3144() {
    // P3: the clean velocity derivatives (not flap-rate-damping) — sign + order, like the
    // UH-60. Heffley tolerances are WIDER than GENHEL (analysis-program oracle).
    let ac = Aircraft::bo105();
    let coll = hover_collective_for_weight(&ac);
    let dl = longitudinal_derivatives(&ac, coll, Controls::none());
    let dt = lateral_derivatives(&ac);
    let (zw, xu, mu) = (dl.zw / MASS, dl.xu / MASS, dl.mu / IYY);
    let (yv, nv, nr) = (dt.yv / MASS, dt.nv / IZZ, dt.nr / IZZ);

    println!(
        "BO-105 hover velocity derivs (SI): Zw {zw:+.4} (o {ZW_O}) | Xu {xu:+.4} (o {XU_O}) | \
        Mu {mu:+.4} (o {MU_O}) | Yv {yv:+.4} (o {YV_O}) | Nv {nv:+.4} (o {NV_O}) | Nr {nr:+.4} (o {NR_O})"
    );

    // Signs (the robust external check).
    assert!(zw < 0.0 && xu < 0.0, "Zw, Xu < 0 (heave / drag damping)");
    assert!(mu > 0.0, "Mu > 0 (destabilizing speed stability)");
    assert!(
        yv < 0.0 && nr < 0.0,
        "Yv, Nr < 0 (side-force / yaw damping)"
    );
    assert!(nv > 0.0, "Nv > 0 (weathercock)");

    // Order-consistency (within ~2×) on the clean ones, like the UH-60.
    let order_ok = |mine: f64, o: f64| (mine / o) > 0.4 && (mine / o) < 2.5;
    assert!(order_ok(zw, ZW_O), "Zw order-consistent");
    assert!(order_ok(nr, NR_O), "Nr order-consistent (tail-based)");
}
