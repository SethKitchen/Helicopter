//! Milestone 6 — OH-6A cg-sweep: the GAIN DISCRIMINATOR for the UH-60 attitude over-response.
//! Pre-registered in this test before the run. CR-3144 gives the OH-6A at hover (0 kt,
//! 1157 kg) at THREE cg positions (FWD CASE 13 / MID CASE 4 / AFT CASE 16), so the SLOPE
//! dΘ/d(cg_offset) directly measures the cg→attitude gain — and the slope cancels shaft tilt
//! and every other constant offset. H1 (gain too strong) predicts my slope is STEEPER than
//! the oracle's; H1 falsified if the slopes match (~within 30%). Frozen params: cg_offset is
//! the sweep variable from sourced station lines, not a fit knob.

use helisim_trim::{Aircraft, NewtonConfig, TrimCondition, trim};

// Oracle (CR-3144 Fig II-3 + CASE 13/4/16): cg FS 97/100/104, hub FS 100, Θ 0.20/1.66/3.60°.
const CG_FWD: f64 = -0.0762; // (97-100) in → m
const CG_MID: f64 = 0.0; //     (100-100) = hub
const CG_AFT: f64 = 0.1016; //  (104-100) in → m
const TH_FWD: f64 = 0.20;
const TH_MID: f64 = 1.66;
const TH_AFT: f64 = 3.60;

fn oracle_slope() -> f64 {
    (TH_AFT - TH_FWD) / (CG_AFT - CG_FWD) // °/m
}

fn my_theta(e: f64, cg: f64) -> f64 {
    let mut ac = Aircraft::oh6a();
    ac.flap.hinge_offset = e;
    ac.cg_offset = cg;
    trim(&ac, &TrimCondition::hover(), &NewtonConfig::default())
        .pitch
        .to_degrees()
}

fn my_slope(e: f64) -> f64 {
    (my_theta(e, CG_AFT) - my_theta(e, CG_FWD)) / (CG_AFT - CG_FWD)
}

#[test]
fn oh6a_cg_sweep_gain_discriminator() {
    let os = oracle_slope();
    println!("OH-6A cg→attitude SLOPE discriminator (oracle dΘ/d(cg) = {os:.1} °/m):");
    println!(
        "  oracle Θ: FWD {TH_FWD:.2}° (cg {CG_FWD:+.3}m) | MID {TH_MID:.2}° (0) | AFT {TH_AFT:.2}° (cg {CG_AFT:+.3}m)"
    );
    // My sweep at nominal articulated hinge e=0.03.
    let (tf, tm, ta) = (
        my_theta(0.03, CG_FWD),
        my_theta(0.03, CG_MID),
        my_theta(0.03, CG_AFT),
    );
    let ms = (ta - tf) / (CG_AFT - CG_FWD);
    println!(
        "  mine   Θ: FWD {tf:.2}° | MID {tm:.2}° | AFT {ta:.2}°  → slope {ms:.1} °/m  ({:.2}× oracle)",
        ms / os
    );

    // Robustness across the unsourced articulated hinge-offset bracket [0.02, 0.08].
    println!("  slope across hinge-offset bracket (oracle {os:.1}):");
    let mut ratios = vec![];
    for &e in &[0.02, 0.03, 0.05, 0.08] {
        let s = my_slope(e);
        println!(
            "    e={e:.2} (ν_β={:.3}): slope {s:.1} °/m  ({:.2}× oracle)",
            {
                let mut a = Aircraft::oh6a();
                a.flap.hinge_offset = e;
                a.flap.nu_beta_sq().sqrt()
            },
            s / os
        );
        ratios.push(s / os);
    }

    // OUTCOME — H1 (as pre-registered) is FALSIFIED; believe the disagreement.
    // The pre-registration predicted the slope would be STEEPER by ~2–2.5× ROBUSTLY across
    // the hinge-offset bracket. It is NOT: the slope is strongly hinge-offset-dependent
    // (33→12.5 °/m over e=0.02→0.08) and REPRODUCES the oracle (19.1) at a perfectly physical
    // articulated offset e≈0.05 (0.96×). So:
    //  (1) the oracle slope is BRACKETED by physical hinge offsets — my slope spans below AND
    //      above it ⇒ the cg→attitude gain is SOUND at a reasonable hub stiffness, NOT
    //      intrinsically too strong. H1 (intrinsic structural gain error) is refuted.
    //  (2) the slope is hub-stiffness(hinge-offset)-DOMINATED ⇒ this discriminator is
    //      CONFOUNDED by the unsourced hinge offset: "gain too strong" and "hub stiffness too
    //      low" are entangled and not separable here.
    //  ⇒ the UH-60's 55% over-response is therefore NOT a model-general cg→attitude gain
    //     error; it re-localizes to a UH-60-specific factor (candidate: effective hub
    //     stiffness under-modeled by its sourced geometric hinge offset — the OH-6A itself
    //     wanted e≈0.05, above a nominal 0.03). Documented in this test's pre-registration.
    let min_ratio = ratios.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_ratio = ratios.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    assert!(
        min_ratio < 1.0 && max_ratio > 1.3,
        "oracle slope is bracketed by physical hinge offsets (got {min_ratio:.2}–{max_ratio:.2}×) — cg gain sound, NOT intrinsically too strong (H1 refuted)"
    );
    assert!(
        max_ratio / min_ratio > 2.0,
        "slope strongly hinge-offset-dependent (gain & hub-stiffness entangled; the discriminator is confounded)"
    );
    // Sign sanity (holds regardless): aft cg → more nose-up, same sign as the oracle.
    assert!(ta > tf, "aft cg → more nose-up (same sign as oracle)");
}
