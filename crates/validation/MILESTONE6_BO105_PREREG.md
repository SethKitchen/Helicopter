# BO-105 (2nd airframe) — pre-registration, BEFORE building Aircraft::bo105()

Second external airframe. Purpose: convert the UH-60's *held-loosely* claims into
**cross-aircraft tests** — above all whether the gyroscopic flap-damping term
(`gyro_rate=−2`), derived & validated on the ARTICULATED UH-60, **generalizes to a
HINGELESS rotor with parameter changes alone (correct physics) or was UH-60-specific
(boundary found).** Source: NASA CR-3144 (Heffley), captured in BO105_HEFFLEY_CR3144.md.

## Honesty note — the single-source leak, and why the test survives it
CR-3144 holds parameters AND oracle in one document (like TM 85890). Confirming the hover
derivative table was *complete* put it on screen, so I **glimpsed the two headline
magnitudes**: hover Mq and Lp are LARGE (hingeless-consistent). Therefore the qualitative
claim "the BO-105 has much stronger flap damping than the UH-60" is **NOT a blind
prediction** — I state it knowing the target.
**What the glimpse CANNOT game (the falsifiable core):** with the airframe parameters
LOCKED from CR-3144 (geometry/inertia/controls) and the gyro coefficient LOCKED at −2 (the
UH-60-derived value, NOT re-tuned), whether *my model's output* lands in-band is determined
by physics I cannot adjust after seeing the target. The test is structured as an **A/B**
(below) precisely so that seeing the oracle cannot fake either arm.

## Parameter-mapping decisions — locked on physics (no tuning toward the oracle)
1. **NACA 23012 → our `LinearAirfoil` at lift slope a=5.73 /rad** (same choice as the UH-60
   SC1095 mapping; camber/zero-lift-angle and drag polar unmodeled — neutral for the
   force/moment derivatives, a collective offset absorbed in trim / a power effect owned by κ).
2. **Flap frequency ν_β — BRACKETED, not a single injected number.** CR-3144 omits ν_β (the
   hingeless stiffness). Per the PBA precedent (don't inject a match-forcing parameter from a
   cited background ref), ν_β is swept across the **hingeless physical range [1.08, 1.15]**
   (e_eff ≈ [0.10, 0.18] via ν_β²=1+1.5e/(1−e)). The gyro conclusion must hold **across the
   whole bracket** to count — robust to the sourcing gap.
3. **Lock number γ computed from CR-3144**: γ = ρ·a·c·R⁴/I_β = 1.225·5.73·0.27·4.91⁴/219.50
   ≈ 5.0 (vs UH-60 γ=8.19) — a genuinely different flap param set (feeds the Lp-discriminator).
4. **`gyro_rate = −2` FIXED** (the UH-60 value, NOT re-derived/re-tuned) — this IS the test.
5. **Tail rotor: conventional 2-blade, NO cant** (BO-105 has no canted TR — removes the
   UH-60 canted-TR mapping judgment entirely; a cleaner Nr/Yv path).
6. **Horizontal stabilizer omitted** (hover: negligible dynamic pressure, ∝½ρV²→0) — same
   as UH-60 #3; named as a forward-flight error source.
7. **Ixz = 0** — no omission needed; the oracle and our model agree (cleaner than UH-60).
8. **Mass/inertia/CG from the Nominal-Weight (CASE 29) row**: 2096 kg, Ix 1803, Iy 4892,
   Iz 4428 kg·m²; mid CG (cg_offset from the FS data, trim-only as on the UH-60).
9. **Density** ρ=1.225 (sea level — the oracle's stated condition).

## Predictions (headline = gyro generalization) — the falsifiable A/B
**P1 — GYRO GENERALIZES (the headline). A/B, ungameable by the glimpse:**
- **Arm A (gyro_rate=0):** Mq and Lp come out FAR too small (the same in-phase-β1c deficit
  the UH-60 showed pre-fix, ~10–20×) — i.e. **the deficit reappears on a 2nd, hingeless
  airframe**, confirming it is a model-general gap, not UH-60-specific.
- **Arm B (gyro_rate=−2, the UH-60 value, unchanged):** Mq and Lp move into the
  order-consistent band (within ~2×, the Heffley tolerance) **across the whole ν_β bracket.**
- **Verdict:** A-small AND B-in-band ⇒ the gyro term is **correct physics that generalizes**
  (the strongest possible evidence). **Falsifier:** if B is still far off (or now far too
  large) across the whole bracket, the −2 form does **not** cleanly generalize to hingeless —
  a **boundary of the approximation**, a clean localized result either way (the signature of
  a good oracle). NOT to be fixed by tuning −2 to a new value.

**P2 — Lp second-airframe discriminator (the dormant UH-60 thread, now active).** The
UH-60 Lp landed at 3% with the textbook coefficient-2 — flagged as possibly param-sum
fortuity. The BO-105 has a *different* γ (5.0 vs 8.19) and ν_β (hingeless). Prediction: if
the gyro term is real physics, Lp lands in-band here too (arm B); if it drifts on this
airframe, the 3% was partly the UH-60's parameter sums (the benign-vs-coincidence question
the UH-60 results left open).

**P3 — velocity derivatives (the clean ones) match in sign + order**, like the UH-60: Zw,
Xu, Mu, Yv, Nv, and the tail-based Nr land within ~sign+order (Heffley tolerance), since
they don't depend on the flap rate-damping term. Nr is cleaner here (no canted TR).

**P4 — BEMT over-prediction, THIRD independent sighting.** On hover trim, my collective /
thrust shows the documented BEMT C_T over-prediction DIRECTION on a *third* rotor (after
C&T and the UH-60). A third sighting on a different rotor moves the bias from "exists +
triangulated" toward "characterized" — or, if it breaks, that's informative. Direction is
the commit; magnitude stays path/config-dependent (tip-loss caveat), not a clean scalar.

## Tolerances — WIDER than the UH-60 GENHEL comparison (pre-committed)
Heffley's derivatives are Boeing-Vertol Y-92 analysis-program outputs (Ref 4), late-1970s —
their own modeling/identification uncertainty. **Expect looser agreement than GENHEL gave.**
Pre-register **order-consistent / within ~2×** as the norm (a 1.73×-style result), NOT the
Lp-3% tightness. A suspiciously tight (<5%) match is a too-good FLAG (compensating error /
coincidence), symmetric with the predicted-clean-but-off headline — same locked hygiene.

## Execution hygiene (locked, applied when numbers come in)
- **Units:** oracle is SI already (no English conversion), but the per-row NORMALIZATION
  (M/L′/N′ inertia-normalized vs raw) must be pinned from the report's format-definition
  section ONCE before comparing; cross-check a dimensionless form.
- **Signs/axes:** Heffley body-fixed FRL axis + its own control-sign conventions — confirm
  vs ours and transform ONCE explicitly, never per-derivative (the class that bit us twice).
- **Convergence ≠ accuracy:** the BO-105 (different σ, tip speed, hingeless) may not trim
  from the default guess — a solver-robustness issue (continuation), NOT a physics mismatch.
- **Too-good is a flag**, as above.

## Scope & sequencing
- **Hover derivatives first** (least-approximated, like the UH-60), then trim positions.
- **The build + run is its OWN session** (the irreversible-comparison lesson) — this file +
  the dataset are the lock; nothing is wired here.
- Build `bo105()` STRICTLY from CR-3144 + this mapping. If a needed parameter is missing,
  name it (as ν_β is) — never inject an unsourced number to force a match.

## OUTCOME (run done — `dynamics/tests/bo105_external_validation.rs`, 4 tests)

Built `Aircraft::bo105()` from CR-3144 (geometry Table III-1, inertia Fig III-3b, γ=5.02
computed) with `gyro_rate=−2` UNCHANGED from the UH-60. Oracle = CASE 29 (0 kt, 2096 kg).

**P1 — GYRO GENERALIZES. Headline confirmed, the A/B held exactly as pre-registered:**
| | gyro=0 (arm A) | gyro=−2 (arm B) | oracle |
|---|---|---|---|
| Lp | −1.06 (**0.11×**) | −9.18 (0.99×) | −9.24 |
| Mq | −0.38 (**0.11×**) | −3.37 (0.99×) | −3.40 |
- **Arm A:** the rate-damping deficit **REAPPEARS on the hingeless rotor** (~0.11×, ~9×
  short) — proving it is a **model-general gap, not UH-60-specific.**
- **Arm B:** the UNCHANGED −2 lifts it into the band. The gyro term derived on an
  *articulated* rotor generalizes to a *hingeless* one with parameter changes alone —
  **the strongest possible evidence it is correct physics.**

**Two honesty controls applied (NOT celebrating the 0.99×):**
1. **TOO-GOOD flag (locked hygiene).** 0.99× on an analysis-program oracle, with a
   pre-registered ~2× tolerance and the UH-60 Lp already flagged at 3%, is *suspiciously
   perfect*. The **robust claim is NOT "1% accurate"** — it is the A→B generalization PLUS
   order-consistency **across the whole ν_β bracket** [1.08,1.15]: Lp = 0.71× / 0.99× /
   1.18× at ν_β = 1.08 / 1.12 / 1.15. The conclusion does **not** hinge on ν_β=1.12 (which
   I set as the bracket *midpoint*; that it coincides with the canonical BO-105 value is
   genuine, not tuned). hub_height (un-sourced) confirmed irrelevant: Lp spread 8% across
   [0.5,1.5] m (hub-spring-dominated, as predicted).
2. **One match, not two.** Mq and Lp both ≈0.99× is the SAME raw flap rate-damping moment
   seen through two inertias (5f rotation: Lp=Mq raw; oracle Lp/Mq=2.72 ≈ Iyy/Ixx=2.71) —
   not two independent confirmations.

**P2 — Lp second-airframe discriminator RESOLVED.** The UH-60 Lp 3% was flagged as possibly
parameter-sum fortuity. The BO-105 has a *different* γ (5.02 vs 8.19), a *hingeless* hub,
and ν_β 1.12 vs the UH-60's ~1.03 — yet the SAME untuned −2 lands it in-band across the
bracket. So the UH-60 result was **not pure fortuity; the gyro term genuinely generalizes.**
(The individual near-perfect %s on each airframe stay too-good-flagged; the cross-aircraft
generalization is the robust, ungameable finding.)

**P3 — velocity derivatives (the clean ones): all sign + order, in the wider Heffley band.**
Zw 2%, **Nr 1.5%** (tail-based — the SAME 1.5% as the UH-60, again ruling out any
assembly/units bug since Nr uses no main flap), Xu 8%, Mu 17% (low), Yv 17%, Nv 70% (high —
the noisy weathercock-stiffness outlier, right sign/order, within ~2×). No conversion trap
(oracle already SI). All signs match the real aircraft.

**P4 — BEMT over-prediction, THIRD independent sighting CONFIRMED.** My hover collective
12.33° vs oracle θMR 14.32° = **14% lower** — the documented BEMT thrust over-prediction
direction, now on a *third* rotor (C&T, UH-60, BO-105), and at ~the SAME 14% as the UH-60.
Direction is robust/triangulated across three rotors; magnitude stays path/config-dependent
(tip-loss caveat) — account, don't correct.

**Net:** the gyroscopic flap-damping fix is now externally validated on **two independent
airframes** (articulated UH-60 / hingeless BO-105) against **two independent oracles**
(GENHEL blade-element / Boeing-Vertol Y-92), with the coefficient −2 derived from first
principles and never re-tuned. The dormant Lp discriminator is closed in favor of "correct
generalizing physics." Tolerances honored (wider for Heffley), too-good flagged not
celebrated, the single-source ν_β gap handled by bracketing. 158 tests pass.

## Cyclic / attitude trim comparison — the axis the UH-60 PBA confounded (pre-reg + outcome)

**The wrinkle that changes the discipline:** CR-3144 CASE 29 reports the trim **control
angles directly in degrees** (θMR=14.32°, B1S=−0.42°, A1S=−0.33°, θTR=10.17°; Θ=2.64°,
Φ=−2.97°) — NOT pilot-stick inches behind a swashplate rigging. So the cyclic comparison is
**rigging-free at the blade-pitch level**: the UH-60 deg↔in SCALE trap is **moot here**.
The only remaining trap is the **sign/axis convention** (reconcile before any sign claim).
And the BO-105 has **no pitch-bias actuator**, so B1S is the *full* longitudinal cyclic —
this is the longitudinal-cyclic axis the UH-60 PBA blocked, now cleanly comparable.

**Honesty note:** B1S/A1S/Θ/Φ were on screen during the CASE-29 read, so their *smallness*
is not blind; the falsifiable core (does my locked-param trim produce comparable values) is
not gameable by the glimpse.

**Predictions (physics-grounded, before comparing):** (1) hover cyclic is SMALL, |θ1s|,
|θ1c| ~ O(0.1–1°), order-consistent with |B1S|=0.42°, |A1S|=0.33°. (2) Roll Φ negative ~−2
to −3° (tail-side-force bank — the mechanism the model has; oracle −2.97°). (3) Pitch Θ
nose-up, small (cg_offset≈0.018 m here, so mostly rotor hub-moment-driven; oracle +2.64°).
(4) Tolerances Heffley-grade (~order-consistent), not GENHEL-tight. Sign comparison gated on
the convention reconciliation; magnitudes are convention-free.

**OUTCOME (run done — `trim/tests/bo105_trim_validation.rs`).** The two axes split cleanly,
and the split IS the finding:
- **LATERAL axis — clean, the headline (the comparison the UH-60 PBA blocked, now delivered):**
  lateral cyclic |θ1c|=0.34° vs |A1S|=0.33° = **1.03×** (sign deferred to convention);
  roll Φ=−2.78° vs −2.97° = **6%** (the tail-side-force bank). The lateral/roll trim is
  well-captured. (1.03× is too-good-flagged for a single hover-small number, but the roll 6%
  corroborates the axis — the lateral trim physics is right.)
- **LONGITUDINAL axis — a NAMED MISS, localized:** θ1s=+0.03° vs B1S=−0.42°, and Θ=+0.09°
  vs +2.64° — both ≈0. **Cause: bo105() omits the 3° forward SHAFT TILT** (CR-3144 Table
  III-1; the trim residual has no shaft-tilt term). In hover a 3° fwd shaft vertical-izes via
  a ~3° nose-up fuselage ≈ the oracle +2.64°; cg_offset (the UH-60's nose-up lever) is ≈0
  here, so with shaft tilt omitted nothing pitches the BO-105 nose-up. Localization ARGUED
  from physics (Θ≈shaft tilt), **not yet MEASURED** — verifying it needs a shaft-tilt term in
  the residual.
- **NEW THREAD (shaft tilt), with a UH-60 cross-implication.** Adding shaft tilt is a trim-
  residual change (sign-sensitive) that should default to 0 (spare UH-60/demo/5a–5m), like
  `gyro_rate`. Cross-implication to handle together: the **UH-60 also omits its 3° shaft
  tilt**, currently absorbed into the cg_offset=0.488 attribution (which gave Θ=+5.94° vs
  +5.05°, 18% over) — so adding shaft tilt globally would re-open that attribution (likely
  over-shoot, meaning cg_offset over-compensated for the omission). Deferred as its own
  focused step with both airframes revalidated; NOT rushed here. The BO-105 longitudinal-
  cyclic axis is thus delivered but its longitudinal *attitude* is the omitted-shaft-tilt
  measurement, pending.
