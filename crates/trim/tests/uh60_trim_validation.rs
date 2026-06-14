//! Milestone 6 — UH-60 hover TRIM validation (rigging-free attitude check).
//! Oracle: NASA TM 85890 Table 4 (level-flight trim), 1.0-kt column: Euler pitch
//! Θ=+5.05°, roll Φ=−2.34°. The fuselage attitudes are physical (no control rigging
//! needed); the stick positions δ need the swashplate rigging and are deferred.
//! Predictions pre-registered in validation/MILESTONE6_TRIM_PREREG.md, all confirmed.

use helisim_trim::{Aircraft, NewtonConfig, TrimCondition, trim};

#[test]
fn uh60_hover_trim_attitude_vs_genhel() {
    // SHAFT-TILT VERIFICATION + the backward-reaching falsifier into the cg_offset
    // attribution (pre-registered: MILESTONE6_SHAFT_TILT_PREREG.md). Both longitudinal
    // params INDEPENDENTLY SOURCED — cg_offset 0.488 m (station lines) AND shaft_tilt 3°
    // (Table 1) — and the attitude is a PREDICTION, NOT a target (neither re-tuned to hit
    // +5.05°; the discipline lock that keeps two-params/one-target from becoming a fit).
    let ac = Aircraft::uh60();
    let r = trim(&ac, &TrimCondition::hover(), &NewtonConfig::default());
    let (pitch, roll) = (r.pitch.to_degrees(), r.roll.to_degrees());
    // Decomposition: cg-only (shaft tilt off) to expose the seam.
    let mut cg_only = Aircraft::uh60();
    cg_only.shaft_tilt = 0.0;
    let pitch_cg = trim(&cg_only, &TrimCondition::hover(), &NewtonConfig::default()).pitch.to_degrees();
    let (pitch_o, roll_o) = (5.05, -2.34);
    println!(
        "UH-60 hover trim: Θ(cg+shaft)={pitch:+.2}° | Θ(cg-only)={pitch_cg:+.2}° (o +5.05°) | Φ={roll:+.2}° (o -2.34°)"
    );

    // Roll: unchanged by the (longitudinal) shaft tilt — the tail-side-force bank (5g φ_e).
    assert!(roll < 0.0, "left bank to balance tail side force");
    assert!((roll - roll_o).abs() / roll_o.abs() < 0.20, "roll Φ within 20% of GENHEL");

    // THE FINDING (Outcome A confirmed). cg-only gives Θ=+5.94° (≈+5.05°, the seam); adding
    // the SOURCED 3° shaft tilt → Θ=+7.82°, which OVERSHOOTS +5.05° by ~55%. So cg_offset
    // =0.488 was OVER-ATTRIBUTING the omitted shaft-tilt nose-up — the +5.94≈+5.05 closeness
    // was parameter-sum fortuity (the seam), invisible on the UH-60 alone and only separable
    // once the BO-105 (cg≈0) validated the shaft-tilt mechanism in isolation. The combined
    // over-prediction localizes a real trim-geometry issue (the longitudinal attitude over-
    // responds to cg+shaft forcing by ~55%) — the next investigation, NOT fixed by re-tuning.
    assert!(pitch > pitch_cg, "shaft tilt adds nose-up on top of the cg-only attitude");
    assert!(pitch > pitch_o * 1.2, "Θ OVERSHOOTS oracle with both sourced terms (cg over-attribution)");
    assert!((pitch_cg - pitch_o).abs() / pitch_o < 0.25, "cg-only ≈ oracle was the SEAM (the fortuitous match)");
}

#[test]
fn uh60_hover_collective_shows_the_bemt_overprediction() {
    // Stick-position comparison via the TM 85890 control rigging (Table 1):
    // root collective θ0 = C5 + C6·δc, C5=0.2286 rad, C6=0.02792 rad/in.
    // Pre-registered (MILESTONE6_TRIM_PREREG.md): my hover collective comes out LOWER
    // than the UH-60's because the BEMT over-predicts thrust, so the rotor makes
    // thrust=weight at less collective. The DIRECTION is a second independent external
    // sighting of the milestone-1 BEMT over-prediction (the first was C&T hover C_T) —
    // triangulation. The MAGNITUDE is NOT a clean matched scalar (see
    // MILESTONE6_RESULTS.md "BEMT bias"): collective-reduction ≠ C_T-over-prediction,
    // and this trim aero (`longitudinal_main_aero`) omits the Prandtl tip loss the C&T
    // hover BEMT included, inflating its over-prediction (~56% at fixed collective vs
    // C&T's ~20–27%). So this test asserts only the robust part: direction + a sane
    // margin. Account for the bias; do not correct with a scalar.
    let ac = Aircraft::uh60();
    let r = trim(&ac, &TrimCondition::hover(), &NewtonConfig::default());
    let (c5, c6, c7, c8): (f64, f64, f64, f64) = (0.2286, 0.02792, 0.1743, -0.07734);
    let mine_deg = r.collective.to_degrees();
    let oracle_deg = (c5 + c6 * 5.719).to_degrees(); // δc=5.719 in → 22.25°
    let frac_lower = (oracle_deg - mine_deg) / oracle_deg;
    println!(
        "UH-60 hover root collective: mine {mine_deg:.2}° vs oracle {oracle_deg:.2}° \
         ({:.0}% lower → BEMT over-predicts thrust; DIRECTION triangulated w/ C&T, \
         magnitude confounded by the trim aero's tip-loss omission). \
         Pedal: mine {:.2}° → δp {:.2} in vs -1.279 (right sign; torque/κ-derived)",
        frac_lower * 100.0,
        r.tail_collective.to_degrees(),
        (r.tail_collective - c7) / c8
    );
    // Assert only the robust claim: collective is lower (over-prediction direction) by a
    // sane margin. NOT a magnitude match to C&T (the paths differ — tip loss).
    assert!(mine_deg < oracle_deg, "collective lower (BEMT over-predicts thrust)");
    assert!((0.03..0.30).contains(&frac_lower), "lower by a sane margin (got {:.0}%)", frac_lower * 100.0);
}

#[test]
fn uh60_hover_cyclic_vs_genhel_units_and_lateral() {
    // CYCLIC comparison via the full TM 85890 swashplate rigging (Table 1 sensitivities +
    // Table 2 crossfeed mixing), pre-registered in MILESTONE6_CYCLIC_PREREG.md, rigging
    // LOCKED in MILESTONE6_PARAMETER_MAPPING.md (#10).
    //
    // What the run surfaced (see MILESTONE6_RESULTS.md "Cyclic"):
    //  - UNITS are clean: the SAME rigging inverts collective and pedal into the oracle's
    //    range (δc low only by the already-separate BEMT bias; δp within ~20%). The cyclic
    //    conversion is the identical rad/(rad/in) arithmetic, so it is unit-clean too.
    //  - LATERAL cyclic (the clean axis, no actuator augmentation): blade-pitch magnitude
    //    ~1.7× the oracle — order-consistent. This is the defensible cyclic result.
    //  - LONGITUDINAL cyclic is CONFOUNDED by the UH-60's pitch-bias actuator (PBA): the
    //    PBA adds to the *total* longitudinal cyclic as a function of pitch attitude
    //    (active at hover), so the oracle's *pilot* δe is only the residual; my model has
    //    no PBA so my θ1s does the whole job. The PBA gain lives in ref 2, NOT sourceable
    //    from TM 85890 → the longitudinal axis is not apples-to-apples. Named confound,
    //    like the stabilator/canted-TR — documented, NOT asserted as a match.
    //  - SIGN/axis correspondence stays deferred to an explicit convention reconciliation;
    //    only convention-free MAGNITUDES are asserted here.
    let ac = Aircraft::uh60();
    let r = trim(&ac, &TrimCondition::hover(), &NewtonConfig::default());

    // Rigging (NASA TM 85890 Table 1 + Table 2), no value tuned. CAIS=CBIS=0 (blank in
    // the UH-60 column → no built-in cyclic bias); SKV feedback = 0 (body rates 0 at trim).
    let (c5, c6, c7, c8) = (0.2286, 0.02792, 0.1743, -0.07734_f64); // rad, rad/in
    let (ck1, ck2) = (0.04939, 0.02792_f64); // rad/in: long, lat cyclic sensitivity
    let (sk4, skm2, sk8, sk11) = (-0.1640, -0.5746, -0.16, -0.2889_f64); // crossfeeds (SK1=SK5=SK10=1)

    // --- UNITS gate: invert collective and pedal through the SAME machinery (triangular:
    // δc independent; δp depends on δc). If these land in the oracle's range, the rad↔in
    // arithmetic is sound and the (identical-arithmetic) cyclic inversion is unit-clean.
    let dc = (r.collective - c5) / c6;
    let dp = (r.tail_collective - c7) / c8 - sk11 * dc;
    // δc low by the BEMT thrust over-prediction (its own test); still the right ORDER.
    assert!((2.0..7.0).contains(&dc), "δc={dc:.2} in not in oracle order (o 5.72) — units bug");
    assert!((-2.5..-0.5).contains(&dp), "δp={dp:.2} in not in oracle order (o -1.28) — units bug");

    // --- LATERAL cyclic (clean axis): compare blade-pitch MAGNITUDE to the oracle's.
    // Oracle lateral blade cyclic A = CK2·(δa + SK8·δc) from Table 4 (no PBA on this axis).
    let (de_o, da_o, dc_o) = (0.1266, 0.2321, 5.719); // Table 4, 1.0-kt column (in.)
    let a_oracle = ck2 * (da_o + sk8 * dc_o); // rad, lateral cyclic blade pitch
    let b_oracle = ck1 * (de_o + sk4 * dc_o + skm2 * (-1.279)); // rad, lon (PILOT only, PBA excluded)
    let lat_ratio = r.cyclic_lat.abs() / a_oracle.abs();

    println!(
        "UH-60 hover cyclic (blade pitch, convention-free magnitudes):\n  \
         LATERAL  mine |θ1c|={:.2}° vs oracle |A|={:.2}°  → {:.2}× (clean axis, order-consistent)\n  \
         LON.     mine |θ1s|={:.2}° vs oracle pilot |B|={:.2}°  → {:.1}× (CONFOUNDED by PBA — not comparable)\n  \
         units x-check: δc={:.2} in (o 5.72, low by BEMT bias) | δp={:+.2} in (o -1.28)",
        r.cyclic_lat.to_degrees(), a_oracle.abs().to_degrees(), lat_ratio,
        r.cyclic_lon.to_degrees(), b_oracle.abs().to_degrees(),
        r.cyclic_lon.abs() / b_oracle.abs(),
        dc, dp
    );

    // Lateral cyclic blade-pitch magnitude order-consistent with the oracle (within ~3×).
    // This is the pre-registered cyclic result on the unconfounded axis. NOT a tight match
    // (uniform inflow, tail model, sign deferred) — an order-of-magnitude / factor check.
    assert!(
        (1.0 / 3.0..3.0).contains(&lat_ratio),
        "lateral cyclic blade pitch {:.2}× oracle — outside the order-consistent band",
        lat_ratio
    );
    // Longitudinal cyclic deliberately NOT asserted: PBA confound (named above).
}
