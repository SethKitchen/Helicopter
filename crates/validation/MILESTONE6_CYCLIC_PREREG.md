# Cyclic stick-position comparison — pre-registered (BEFORE wiring Table 2)

The cyclic comparison is the one remaining hover-trim oracle that exercises **untested
machinery** (the swashplate crossfeed mixing) and can therefore find a **real error**.
The trap it carries — flagged on the locked hygiene checklist (units, sign conventions)
— is a **deg↔in scale error hiding in the rigging arithmetic**. This file commits the
expectations BEFORE the rigging is wired, so the wiring self-tests against a prediction
instead of being a free parameter that can be nudged until it matches Table 4.

## What is and isn't sourced yet (honest status)

- **Sourced (Table 1, in UH60_GENHEL_TM85890.md):** collective `θ0=C5+C6·δc`,
  pedal `θ0_tr=C7+C8·δp`. Cyclic gains noted from the report: CK1=0.04939 rad/in,
  CK2=0.02792 rad/in.
- **NOT yet sourced:** the **Table 2 cyclic crossfeed mixing** (the 2×2 that maps
  longitudinal/lateral stick inches → blade θ1c/θ1s, with any collective/pedal
  crossfeed). Sourcing it from NASA TM 85890 and **locking it in
  MILESTONE6_PARAMETER_MAPPING.md** is the first half of the next focused step — NOT
  done on a session tail. No inversion is performed in this turn.
- **Oracle (Table 4, 1.0-kt / hover column):** δe = 0.127 in (longitudinal stick),
  δa = 0.232 in (lateral stick). [Already recorded in MILESTONE6_TRIM_PREREG.md.]

## My model's cyclic (what trim solves)

`evaluate()` (trim/src/residual.rs): unknowns `[θ0, θ1c, θ1s, θ0_tr, pitch, roll]`.
`cyclic_lon = θ1c`, `cyclic_lat = θ1s`, both in **rad of blade pitch**. In hover the flap
solve runs so cyclic tilts the thrust (β1c>0 blow-back → thrust aft). The longitudinal
moment the cyclic must trim is the **aft-CG hang** `cg_offset·wz` (cg_offset=0.488 m);
the lateral side comes from the tail-side-force / φ_e roll.

## Locked predictions (a priori — NOT read off my trimmed θ first)

1. **Magnitude: hover cyclic is SMALL.** A hover needs only small cyclic — enough to
   trim the aft-CG pitch hang and the lateral tail-side-force asymmetry, not large
   maneuvering tilt. So |θ1c|, |θ1s| should be **O(1°), at most a few degrees**, and the
   inverted stick positions should be **O(0.1–1 in)** — the same order as the oracle's
   δe=0.127, δa=0.232 in. A trimmed cyclic of tens of degrees, or inverted sticks of
   many inches, is non-physical for hover and would itself flag an error.

2. **The scale-trap commit (the whole point of pre-registering).** The gains set the
   deg↔in scale: CK1=0.04939 rad/in = **2.83 °/in**, CK2=0.02792 rad/in = **1.60 °/in**.
   So **~1–3° of blade cyclic ↔ ~0.35–1.9 in of stick** (modulo the Table 2 mixing,
   which redistributes between axes but is **O(1)** — it cannot change the order of
   magnitude). Therefore:
   - An inversion that turns my O(1°) cyclic into **O(0.1–1 in)** sticks ⇒ scale sane.
   - An inversion off by **≳5×** from that (e.g. tens of inches, or hundredths) ⇒ a
     **deg/rad or missing-gain SCALE BUG, logged as a units artifact, NOT a model
     finding** (hygiene rule: a clean-factor miss is a conversion bug). This is the
     error the pre-registration exists to catch.

3. **Direction (held loosely, convention-gated).** Longitudinal: aft CG → rotor must
   carry a nose-down trim moment → forward TPP tilt → small **forward** δe. Lateral:
   left bank / tail-side-force balance → small δa of the oracle's sign. BUT GENHEL's
   stick-sign and body-axis conventions must be **confirmed and transformed ONCE,
   explicitly** before any sign comparison (this convention class bit the project twice
   — flapping cyclic, lateral ±90°). Sign is therefore **informative, not a pass/fail**
   until the convention is reconciled. Do not flip a sign per-axis "to match."

## UH-60-specific confound named up front
The **canted tail rotor** (20°) couples TR thrust into pitch via `T_tr·sinK·arm`
(parameter-mapping item 2). My current `uh60()` tail enters the longitudinal moment only
through the (small) terms it models; the cant's pitch coupling, if not resolved, biases
the **longitudinal** cyclic δe specifically. Named as an error source for δe before the
number is seen — separate from the rigging-scale trap.

## Pass / fail (for the eventual run)
- PASS-SCALE: inverted sticks land O(0.1–1 in) — the rigging scale is sane (the trap
  did not fire).
- PASS-PHYSICS: δe, δa are small and order-consistent with oracle (0.127, 0.232 in).
- INFORMATIVE (not pass/fail): the exact magnitudes/signs vs oracle — measures the
  residual hover-cyclic physics (aft-CG + tail side force) and the canted-TR omission,
  AFTER the convention is reconciled.
- A `≳5×` scale miss is recorded as a **units bug to fix**, never as a model finding.

## Next focused step (its own session, not a tail)
1. Source Table 2 crossfeed mixing from NASA TM 85890; lock it in the parameter mapping.
2. Wire the inversion (blade θ1c/θ1s → δe, δa) using the locked mixing.
3. Run uh60 hover trim; record my θ1c/θ1s (model output) AND the inverted δe/δa.
4. Check against THIS file's predictions (scale first, then physics, then — after
   convention reconciliation — direction); write the outcome into MILESTONE6_RESULTS.md.

## OUTCOME (run done — test: uh60_hover_cyclic_vs_genhel_units_and_lateral)

Table 2 sourced from TM 85890 (feedforward SK1/5/9/10=1; crossfeed SK4=−0.164,
SKM2=−0.5746, SK8=−0.16, SK11=−0.2889) and locked in parameter mapping #10. Ran the
inversion. Three findings, in the pre-registered order:

1. **UNITS clean (scale gate PASSES — but NOT the way the prereg framed it).** The same
   rigging inverts collective δc=3.87 in (o 5.72, low only by the already-separate BEMT
   bias) and pedal δp=−1.02 in (o −1.28, ~20%) — both the right ORDER. The cyclic
   conversion is the identical rad/(rad/in) arithmetic ⇒ unit-clean. **Correction to my
   own prereg:** my first cut asserted a loose `[0.01, 3.0] in` band on the inverted
   cyclic *sticks*, and δe=−2.0 in (16× the oracle 0.127) slid under 3.0 and "passed" —
   the exact loose-threshold laundering I'd just been warned about. The real units
   discriminator is NOT a band on the suspect quantity; it is that the **same machinery
   gives sane collective/pedal** (different gains, so it tests the arithmetic, not the
   cyclic value). Rewrote the gate to that.

2. **LATERAL cyclic — the clean axis — order-consistent.** Blade-pitch magnitude
   |θ1c|=1.89° vs oracle |A|=1.09° = **1.73×**. In-band (within ~3×), comparable to the
   other in-band derivative/trim results. This is the defensible cyclic result. (Sign
   +1.89 vs −1.09 stays deferred to the convention reconciliation.)

3. **LONGITUDINAL cyclic — CONFOUNDED by the pitch-bias actuator (PBA), not comparable.**
   |θ1s|=5.84° vs oracle pilot-only |B|=0.22° = 27×. The UH-60 has a PBA (TM 85890 p.6):
   a variable-length rod that **adds to the *total* longitudinal cyclic** as a function of
   pitch attitude/rate/airspeed, with **pitch-attitude feedback active throughout the
   speed range** (so live at hover, Θ=+5.05°), authority 15% of full throw. The oracle's
   *pilot* δe is therefore only the residual after the PBA acts; my model has no PBA, so
   my θ1s carries the whole longitudinal trim. The **PBA gain is in ref 2, NOT sourceable
   from TM 85890** ⇒ I cannot reconstruct the oracle's *total* longitudinal cyclic, so the
   axis is not apples-to-apples. Named confound (added to parameter mapping #11), like the
   stabilator/canted-TR — documented, NOT asserted, NOT fudged.

**Bonus (weak corroboration of the axis labeling):** the *no-axis-swap* reading is the
only one that makes both findings coherent — a clean lateral axis (1.7×) AND the
confounded axis being exactly the one with the PBA. A swap would scatter both 5–9× with no
clean axis and no mechanism. Weak evidence my θ1c=lat/θ1s=lon labeling is right; held
loosely, the formal sign reconciliation still owed before any direction claim.

**Lesson banked:** the discriminator for "units bug vs real discrepancy" is whether a
*parallel quantity through the same machinery* (collective/pedal) comes out sane — not a
magnitude band on the suspect quantity itself, which can launder a real discrepancy (or a
confound) as a pass.
