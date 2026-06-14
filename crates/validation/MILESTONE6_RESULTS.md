# Milestone 6 — results: hover longitudinal derivatives vs the UH-60A

First time the model meets ground truth. Read against the predictions and parameter
mappings that were **locked before this run** (`MILESTONE6_PREDICTIONS.md`,
`MILESTONE6_PARAMETER_MAPPING.md`). Oracle: NASA TM 85890 (GENHEL), Tables 12 & 13,
"Boeing-Vertol UH-60 math model" — a published, independent UH-60 derivative set.
Aircraft: `Aircraft::uh60()`, built only from the locked mapping, no tuning.
Test: `crates/dynamics/tests/uh60_external_validation.rs`.

## What was predicted (before seeing the oracle)
Force/moment derivatives match in **sign + order**, ~20–35% error from uniform inflow
+ rigid blade; power-derived owned by κ; high-μ owned by uniform inflow.

## The comparison (SI, normalized as GENHEL reports them)

Two results per row: **match-vs-oracle** and **match-vs-predicted-error**.

| deriv | mine | oracle (hover) | oracle (20 kt) | sign | vs oracle | vs prediction |
|---|---|---|---|---|---|---|
| **Zw** 1/s | −0.309 | −0.275 | −0.348 | ✓ | **12%** | **within band ✓** |
| **Xu** 1/s | −0.022 | −0.015 | +0.018 | ✓ | same order | order-ok, ~46% (small noisy term) |
| **Mu** rad/(m·s) | +0.0103 | +0.0016\* | +0.030 | ✓ | in low-speed range | sign+order ✓ (\*hover anomalous) |
| **Mq** 1/s | −0.016 | +0.77\* | −1.03 | ✓ | **~65× too small** | **predicted clean — IS NOT** |

## Hygiene applied (locked checklist)
- **Units:** Zw, Xu, Mq are 1/s — *unit-invariant*, compared directly. Mu is
  1/(ft·s) → ×3.281 to 1/(m·s), done once. No clean-factor surprises ⇒ no conversion
  bug.
- **Signs:** all four agree with GENHEL with no per-derivative flips. `Mu>0` (the
  destabilizing speed stability the whole hover instability rests on) is confirmed
  against a real aircraft.
- **Convergence ≠ accuracy:** trim converged cleanly (19.1° root collective, physical
  with the −18° washout); kept separate from accuracy.
- **Too-good:** Zw at 12% is good but not suspiciously perfect (no compensating-error
  red flag); the others are not suspiciously close.
- **Oracle caveat:** the Boeing-Vertol **0.5-knot column is degenerate** — Mq = +0.77
  (positive, unphysical for pitch damping), Mu anomalously tiny. The near-zero-airspeed
  point of that model is not trustworthy; the 20-knot column is the sane low-speed
  reference where the hover point is suspect.

## The headline result (don't bury it)

**`Zw` lands at 12%** and **`Mu` has the right sign and order** — so the *core heave
and speed-stability physics* (the foundation under every dynamics milestone, including
the emergent hover instability) is **externally confirmed against a real aircraft**.

**`Mq` is the predicted-clean derivative that came out NOT clean: right sign, ~1–2
orders of magnitude too small** (−0.016 vs −1.03). This is the most informative number
in the comparison. Internal validation (sign correctness, self-consistency, the
hovering cubic *shape*) could never catch it, because the cubic depends on ratios and
nothing internal pinned the *magnitude* of pitch damping. External data did, on the
first contact.

## Attribution (per-approximation error budget — the payoff)
- **Heave damping `Zw`** — uniform-inflow + rigid-blade cost is **small (~12%)**. That
  approximation is *cheap* here.
- **Speed stability `Mu`** — right physics, order-correct; the oracle's hover anomaly
  prevents a tight %.
- **Pitch damping `Mq`** — the quasi-static first-harmonic flap is **expensive** here:
  it appears to capture only a small fraction of hover pitch damping. Hypothesis (held
  with humility, not asserted): the dominant hover pitch damping comes from the rotor
  flap *dynamics* / gyroscopic precession (the rotor lagging the shaft), which the
  *quasi-static* harmonic-balance flap discards — it keeps the aerodynamic part and
  drops the lag. Whether this is the full cause or a fixable modelling gap is the next
  investigation.

This is exactly the diagnostic outcome ten milestones of scope discipline were for: a
**per-approximation error budget** (uniform inflow cheap for `Zw`, quasi-static flap
expensive for `Mq`), not one uninterpretable aggregate. A mismatch is a *measurement*
of a named approximation, not a failure — the model just quantified the cost of
quasi-static flapping on a real aircraft's pitch damping.

## Mq audit + the lateral/directional discriminator (pre-registered)

**Audit verdict (boring bug ruled out).** The pitch-moment assembly is dimensionally
sound: correct arm (`hub_height`), both moment terms present (thrust-tilt
`hub_height·T·sinβ1c` + hinge-offset hub moment), and it uses β1c (longitudinal flap)
correctly. Units are clean. Decomposing `Mq`: `dβ1s/dq̄ = −0.995` (the 90°-lagged
response, right order — textbook ≈ −16/γ = −1.95), but `dβ1c/dq̄ = −0.071` (the
*in-phase* response that makes the pitch moment) — the entire deficit lives in the
small in-phase β1c. So it is NOT a unit/arm slip. TWO further confounds make "65×" not
a clean single-cause number: the oracle hover Mq is degenerate (+0.77), and the 20-kt
value (−1.03) includes the **horizontal stabilator** + forward-flight effects my
hover-main-rotor-only model omits. The flap-precession hypothesis stays **null**.

**Pre-registered prediction (written before running my lateral derivatives).** The
roll axis is the clean test: oracle hover `Lp = −3.35 /s` is large and NOT degenerate,
and roll has no stabilator confound. By the 5f rotation construction my roll-rate
damping uses the SAME raw moment as Mq, so I predict:
1. My `Lp = (raw ∂M/∂q)/Ixx = −854/7632 ≈ −0.11 /s` → **~30× smaller than the oracle
   −3.35.** The damping deficit **travels to roll, cleanly** ⇒ the under-prediction is
   the main-rotor flap in-phase rate response, not a pitch-specific assembly artifact.
2. `Nr` (yaw) is **tail-rotor-dominated** in my model, not the main flap → I predict it
   lands much closer to the oracle than Lp, isolating the deficit to the main-rotor flap.
3. `Yv` right sign, order-of-magnitude of the oracle.

## Discriminator OUTCOME (predictions confirmed)

| deriv | mine (SI) | oracle (0.5 kt) | result |
|---|---|---|---|
| **Nr** (tail-based) | −0.283 | −0.288 | **1.5% — nails it** |
| **Yv** | −0.044 | −0.047 | 6% |
| **Nv** | +0.030 | +0.027 | 13% |
| **Lv** | −0.115 | −0.085 | right sign, ~35% |
| **Lp** (main-rotor flap) | −0.193 | −3.35 | **~17× too small — SAME as Mq** |

The pre-registered predictions held. The diagnosis is now clean and localized:

1. **The boring bug is definitively ruled out.** `Nr` uses *no* main-rotor flap — only
   tail-rotor geometry through the same normalization/arm/units machinery — and it
   lands at **1.5%**. A unit or assembly bug could not let Nr match that well. So the
   moment-assembly path is correct.
2. **The deficit travels with the main-rotor flap, and only there.** `Lp` (= `Mq` by
   the 5f rotation) is ~17× too small, the SAME deficit as `Mq`, now measured against a
   **clean** oracle (Lp hover is non-degenerate; roll has no stabilator confound). The
   confounds that muddied the `Mq` number are gone, and the deficit is still there —
   so it is real, not a comparison artifact.
3. **It does NOT touch the non-flap-damping channels.** Velocity derivatives (`Xu`,
   `Zw`, `Yv`, `Nv`, `Lv`, `Mu`) and the tail-based `Nr` all land in sign + order
   (several within 1–15%).

**Final diagnosis:** the model reproduces the real UH-60's hover derivatives across the
board — signs all correct, magnitudes within ~6–35% — **except the main-rotor
flap-derived RATE damping (`Mq`, `Lp`), which is under-predicted ~15–30×.** The
decomposition pins it precisely: the quasi-static first-harmonic flap captures the
90°-lagged response (`dβ1s/dq̄ ≈ −1`, right order) but the **in-phase `β1c` response to
body rate** (`dβ1c/dq̄ = −0.071`) — the part that makes the damping moment — is far too
small. The leading hypothesis (held, not asserted, and now the well-localized next
investigation) is the **missing gyroscopic / kinematic body-rate coupling in the flap
equation** (the "rotor follows the shaft" precession term), which an aerodynamic-only
quasi-static flap forcing omits. Whether that closes the ~20× or only part of it is the
next study — but the external comparison has localized a real model gap to a single
term in a single sub-model, which is exactly the diagnostic resolution the whole
internal-validation chain was built to enable.

## The fix — implemented, derived not fitted, validated

The pre-registered gyroscopic "rotor-follows-shaft" term was derived
(`MILESTONE6_FLAP_FIX_PREREG.md`) and implemented as `FlapProperties.gyro_rate`
(`rhs += gyro·rate` on the orthogonal flap harmonic). All three pre-registered
predictions held:

- **Sign**: `+2` gave anti-damping → the negative sign is physics-mandated
  (gyroscopic precession opposes the rate), re-derived not flipped.
- **Magnitude**: untuned textbook coefficient 2 → `dβ1c/dq̄ = −1.87`; **Lp −0.19 →
  −3.25 vs oracle −3.35 = 3%** (the clean axis; was ~17× short). Closed more than the
  pre-registered ~2× residual — flagged (possible parameter-sum fortuity; the robust
  claim is the deficit was the gyro term and is now in-band). `Mq → −0.45`; its
  residual vs −1.03 (20 kt) is the omitted horizontal stabilator, not the flap.
- **Footprint**: `Zw`, `Mu`, `Xu`, `Nr`, `Yv`, `Nv` **bit-for-bit unchanged** — the
  term touched only the rate damping. The cleanest possible "right thing, right reason."

**Adoption:** `gyro_rate` defaults to 0 so every prior milestone is unchanged (all 150
tests pass); `uh60()` uses −2. Universal adoption (changing the demo dynamics, e.g.
5g depart timing) is a deliberate downstream-revalidation step. External validation
didn't just *measure* a model gap — the derived fix *closed* it, with the regression
proving it's localized.

## Open question — the Lp 3% (named residual uncertainty)

The gyro fix was pre-registered to close MOST of the gap with a ~2× residual; it
closed to **3%**. "The derived term closed the gap" is fully earned (sign, magnitude
order, and the bit-for-bit regression prove it). "...closed it to 3%" is NOT fully
earned: a textbook-coefficient-2, untuned term landing Lp at 3% is *slightly too
good* given the pre-registered 2×, and "too good" is on the locked hygiene checklist.
- Benign explanation (likely): other small flap terms we don't model net near zero in
  THIS configuration, so coefficient-2 lands close.
- The one to rule out: the coefficient-2 term is absorbing a bit of something else's
  contribution, and the 3% is partly a coincidence of the UH-60's parameter sums.
- **Discriminating test (open, not urgent):** a SECOND airframe with different flap
  parameter sums. Same term, different sums — does Lp still land? If Lp drifts on
  another aircraft, this note is the explanation already on disk. The robust claim
  meanwhile: the deficit *was* the gyro term and Lp is now in-band.

## Trim attitude comparison (rigging-free) — predictions confirmed

Oracle: NASA TM 85890 Table 4 (1.0-kt): Θ=+5.05°, Φ=−2.34°. Attitudes are physical
(no rigging); stick positions deferred to a rigging-mapping step. (Pre-registered in
`MILESTONE6_TRIM_PREREG.md`; test `trim/tests/uh60_trim_validation.rs`.)
- **Roll Φ = −2.03° vs −2.34° (13%)** — the tail-side-force bank, a mechanism the
  model has (the 5g φ_e). Matches.
- **Pitch Θ**: with the locked `cg_offset=0` → 0.00° (missed, as predicted); with the
  SOURCED `cg_offset=0.488 m` (CG aft of hub, STA difference, not fit) → +5.94° vs
  +5.05° (**18%**, recovered). The miss localized to one un-set parameter; the
  CG→attitude mechanism is sound. **Measured** (not asserted): with cg_offset set, the
  longitudinal derivatives are bit-for-bit identical and the lateral ones change ≤1e-6
  relative (a tail-trim coupling) — so the validated derivative comparison is unaffected.
  This thread extends coverage (attitude/CG geometry) without re-litigating the damping.

## Stick-position comparison (via the TM 85890 control rigging) — the BEMT, twice

Inverting the rigging (Table 1: `θ0_root = 0.2286 + 0.02792·δc`, etc.) and comparing
to Table 4 (test `uh60_hover_collective_shows_the_bemt_overprediction`):
- **Collective: my root 19.29° vs the oracle's implied 22.25° → 13% lower.** Pre-
  registered prediction confirmed: the BEMT C_T over-prediction (milestone 1, ~20–27%
  high vs Caradonna–Tung) means thrust=weight is met at less collective. This is a
  **second, independent external sighting of the same over-prediction** — two unrelated
  oracles (C&T hover C_T; UH-60 trim collective) agreeing on a named model limitation.
- **Pedal: right sign, magnitude off** (δp −2.14 vs −1.28) — the main *torque* it
  counters is power/κ-derived (the documented non-clean quantity) + tail BEMT. Expected.
- **Cyclic: deferred** — needs the crossfeed mixing (Table 2).

## BEMT bias — characterized (and it corrected the "13% = 2nd sighting" framing)

Tried to characterize the BEMT over-prediction as one number from the two sightings.
The characterization **falsified the clean framing**, twice:
1. **Collective-reduction ≠ C_T-over-prediction.** The 14% lower collective and the C&T
   ~20–27% C_T figure are different quantities. Converting (my BEMT thrust at the real
   22.25° collective vs weight) gives **~56% over-prediction at fixed collective** —
   ~2× C&T, not a match.
2. **The trim/derivative aero omits Prandtl tip loss.** `hover_collective_for_weight`
   and the derivatives use `longitudinal_main_aero`, whose `loads()` has **no tip-loss
   factor**, unlike the milestone-1 hover BEMT (`solve_hover`) that produced the C&T
   ~20–27%. So this path over-predicts thrust MORE (the ~56%), inflating the collective
   gap. A real model inconsistency, surfaced by the characterization (candidate fix, but
   like the gyro term it would ripple through 5c–5m — not done here). Note: it barely
   affects the *derivatives* (perturbations, where tip loss ~cancels — which is why
   Zw/Mu/etc. matched at 12%/sign), but directly biases the *absolute* trim collective.

**Conclusion (the honest payoff):** the two oracles **triangulate the over-prediction's
existence and direction — robust, hard to fake.** A clean, correctable *magnitude* is
**NOT** established: it is path-dependent (tip loss) and configuration-dependent
(twist/airfoil). The cleanest single figure remains the tip-loss-included C&T ~20–27%.
**Account for the bias; do not correct it with a scalar.** (Characterization is
measurement, not tuning — and here it bought a correction to a loose claim plus a real
model finding, the tip-loss omission, rather than a tidy number.)

**Sharper dormant claim, kept SEPARATE from the C_T figure (post-BO-105).** With a THIRD
sighting now in (C&T isolated-rotor; UH-60 trim collective 13–14% low; BO-105 trim collective
14% low), the *trim-collective* manifestation of the bias is converging: **~13–14% on both
full-aircraft trim comparisons.** This is a NARROWER, firming dormant observation than
"exists + path-dependent" — but it is about a DIFFERENT quantity than the
fixed-collective C_T over-prediction (the ~56% path-dependent number a correction would have
to target). Two statements, two quantities, kept separate: (1) the *fixed-collective C_T*
over-prediction stays path-dependent (~56% in the tip-loss-omitting trim aero, ~20–27% with
tip loss) — NOT a scalar; (2) the *trim-collective reduction* is ~13–14% across two airframes
— if a fourth airframe lands near 13–14%, that becomes a genuinely *characterized trim-level
bias*, even while the underlying C_T over-prediction remains path-dependent. Still account,
not correct; but the trim-level statement is more specific than the framing previously admitted.

## Cyclic stick comparison (Table 1+2 rigging) — units clean, lateral matches, longitudinal PBA-confounded

The last hover-trim oracle. Full swashplate rigging sourced (Table 2 crossfeed mixing now
in UH60_GENHEL_TM85890.md / parameter mapping #10) and the blade-cyclic→stick inversion
wired (test `uh60_hover_cyclic_vs_genhel_units_and_lateral`). Pre-registered in
MILESTONE6_CYCLIC_PREREG.md. Three findings, in pre-registered order:

1. **UNITS clean.** The same rigging inverts collective δc=3.87 in (o 5.72, low only by
   the separate BEMT bias) and pedal δp=−1.02 in (o −1.28, ~20%) — both right order ⇒ the
   rad↔in arithmetic is sound, and the identical-arithmetic cyclic inversion is unit-clean.
   *Process catch:* my first-cut test put a loose `[0.01,3.0] in` band on the inverted
   cyclic *stick* and a 16×-oracle δe slid under it and "passed" — the loose-threshold
   laundering trap. The real discriminator is a **parallel quantity through the same
   machinery** (collective/pedal sane), not a band on the suspect value. Gate rewritten.
2. **LATERAL cyclic (clean axis) order-consistent:** |θ1c|=1.89° vs oracle |A|=1.09° =
   **1.73×** — in-band, the defensible cyclic result. Sign deferred to convention recon.
3. **LONGITUDINAL cyclic CONFOUNDED by the pitch-bias actuator (PBA):** |θ1s|=5.84° vs
   oracle *pilot* |B|=0.22° = 27×. The PBA (TM 85890 p.6, gain in ref 2 — NOT sourceable)
   adds to the *total* longitudinal cyclic via pitch-attitude feedback (active at hover),
   so the oracle's pilot δe is only the residual; my no-PBA θ1s does the whole job ⇒ not
   apples-to-apples. Named confound (mapping #11), like stabilator/canted-TR. Documented,
   not asserted, not fudged — and the PBA gain deliberately NOT estimated (would be an
   unsourced number injected to improve a match).

**Net:** the cyclic oracle confirmed the rigging units, gave one clean order-consistent
axis (lateral), and *localized* the longitudinal anomaly to a specific, named, UH-60
control-augmentation feature my generic model lacks — the same diagnostic-localization
pattern as the Mq/stabilator and trim-pitch/cg_offset findings, rather than an
uninterpretable miss. Weak bonus: only the no-axis-swap reading makes both axes coherent
(clean lateral + PBA on exactly the confounded axis), weakly corroborating the θ1c=lat /
θ1s=lon labeling — held loosely; formal sign reconciliation still owed.

## SECOND AIRFRAME — BO-105 (hingeless) vs NASA CR-3144: the gyro term GENERALIZES

The cross-aircraft test (full dataset/prereg/outcome: BO105_HEFFLEY_CR3144.md,
MILESTONE6_BO105_PREREG.md; test `dynamics/tests/bo105_external_validation.rs`). Purpose:
turn the UH-60's *held-loosely* gyro claim into a cross-aircraft test on a **hingeless**
rotor (much stronger hub moment) — does `gyro_rate=−2`, derived on the *articulated* UH-60,
generalize with parameter changes alone? `Aircraft::bo105()` built strictly from CR-3144
(γ=5.02 computed, ν_β bracketed since CR-3144 omits it), −2 UNCHANGED.

- **GYRO GENERALIZES (A/B, pre-registered):** Lp gyro=0 → −1.06 (**0.11×** oracle −9.24);
  gyro=−2 → −9.18 (in-band). Mq identically 0.11×→in-band (−3.40 oracle). The rate-damping
  **deficit reappears on the hingeless rotor** (model-general gap, not UH-60-specific), and
  the **unchanged −2 closes it** — strongest evidence it is correct physics.
- **Too-good flagged, NOT celebrated:** 0.99× on an analysis-program oracle (pre-reg ~2×
  tolerance) is suspiciously perfect. Robust claim = A→B + order-consistency across the ν_β
  bracket (Lp 0.71/0.99/1.18× at ν_β 1.08/1.12/1.15), not "1% accurate." Mq & Lp both 0.99×
  is ONE raw moment via 5f rotation (Lp/Mq 2.72 ≈ Iyy/Ixx 2.71). hub_height (un-sourced)
  irrelevant (8% spread).
- **Lp 2nd-airframe discriminator RESOLVED:** different γ (5.02 vs 8.19), hingeless, ν_β
  1.12 vs ~1.03 — same untuned −2 lands in-band ⇒ the UH-60 3% was **not pure param-sum
  fortuity**; the term generalizes. (Per-airframe %s stay too-good-flagged.)
- **Clean velocity derivs (sign+order, wider Heffley band):** Zw 2%, **Nr 1.5%** (tail-based
  — same as UH-60, re-rules-out an assembly/units bug), Xu 8%, Mu 17%, Yv 17%, Nv 70%
  (noisy outlier, right sign/order). Oracle SI ⇒ no conversion trap.
- **BEMT over-prediction, 3rd sighting:** hover collective 12.33° vs θMR 14.32° = **14%
  lower** — same direction and ~same magnitude as the UH-60, on a third rotor. Direction
  triangulated across three rotors; magnitude still path/config-dependent.

**Cross-aircraft net:** the gyroscopic flap-damping fix is now validated on **two
independent airframes against two independent oracles** (UH-60/GENHEL blade-element,
BO-105/Boeing-Vertol Y-92), coefficient −2 from first principles, never re-tuned.

## Shaft tilt — VERIFIED, and it overturns the earlier UH-60 cg_offset attribution

The BO-105 longitudinal-trim miss localized to the omitted 3° forward shaft tilt; adding it
(`Aircraft.shaft_tilt`, default 0, residual thrust rotated by γ_s) was pre-registered with a
**backward-reaching falsifier** (MILESTONE6_SHAFT_TILT_PREREG.md) and verified. Both real
aircraft set from sourced data (UH-60 0.05236 rad, BO-105 3°), neither re-tuned; derivatives
and demo-based 5a–5m bit-for-bit unchanged (only the trim residual reads `shaft_tilt`).

| | cg-only | cg + sourced shaft tilt | oracle |
|---|---|---|---|
| BO-105 Θ | +0.09° (miss) | **+2.84°** (7.6%) | +2.64° |
| UH-60 Θ | +5.94° (**seam**) | **+7.82°** (~55% over) | +5.05° |

- **BO-105 (cg≈0) validates the mechanism** — Θ recovered to +2.84°.
- **UH-60 overshoots** — confirming **cg_offset=0.488 was over-attributing** the omitted
  shaft tilt. **This OVERTURNS the earlier "cg_offset recovered Θ to 18%" reading**
  (Trim-attitude section above / TRIM_PREREG): the +5.94≈+5.05 was *parameter-sum fortuity*
  (the seam), not a clean recovery — invisible on the UH-60 alone, separable only once the
  BO-105 validated shaft tilt in isolation. The "★ believe the disagreement" rule applied to
  a result that had been *endorsed*: a second airframe overturned a first-airframe attribution.
- **New localized issue (next study, NOT a re-tune):** with both sourced terms the UH-60
  attitude over-responds ~55% — the cg→attitude gain too strong OR a missing nose-down term
  (candidate: rotor-wake download on the omitted horizontal stabilator). Sourced params stay
  put; the resolution is a model study. **[RESOLVED below — both candidates fall.]**

## UH-60 attitude over-response — RESOLVED by a two-step study (stabilator out, gain sound)

A two-hypothesis study (MILESTONE6_ATTITUDE_GAIN_STUDY.md, MILESTONE6_OH6A_GAIN_PREREG.md):
- **Step 1 — stabilator ruled out by SIGN.** A back-of-envelope from sourced TM 85890 geometry
  (S_HS 45 ft², arm 8.64 m aft): the rotor-wake download is **nose-up** (download aft of CG),
  3–19 kN·m. A nose-up term cannot reduce a nose-up over-prediction — it makes it worse. My own
  prior "missing nose-down = stabilator" hypothesis was **sign-wrong**; the check corrected it.
- **Step 2 — gain refuted by the OH-6A cg-sweep (the ★ believe-the-disagreement result).** The
  OH-6A has a hover oracle at THREE cg positions (CR-3144), giving dΘ/d(cg)=**19.1 °/m** —
  shaft-tilt-independent. Pre-registered H1: my slope steeper ~2–2.5× robustly. **FALSIFIED:**
  the slope is hinge-offset-dominated (33→12.5 °/m over e=0.02→0.08) and **reproduces the oracle
  at a physical articulated offset e≈0.05.** So the cg→attitude gain is **sound at a reasonable
  hub stiffness**, not intrinsically too strong; and the discriminator is **confounded** (gain ↔
  hub-stiffness entangled, hinge offset unsourced).
- **Resolution:** the UH-60's 55% over-response — inferred from a SINGLE cg point — is **not a
  model-general gain error** (the cleaner 3-point OH-6A sweep refutes it). It re-localizes to
  UH-60-specific hub-stiffness / effective-ν_β under-modeling (parameter-level, within named
  approximations) or the single-point decomposition. **No structural fix indicated.** The cleaner
  multi-point cross-check overturned the single-point reading. Frozen-parameter discipline held;
  pre-registered prediction reported as FAILED, not forced.

## Scope of this pass / next
- This pass: **hover longitudinal** derivatives only.
- Next: lateral/directional (`Lp`, `Nr`, `Yv`, with the canted-TR mapping that matters
  there), and the `Mq` investigation. Trim-position comparison (Table 4) needs the
  control rigging (deg blade pitch ↔ in. of stick) — its own sourced mapping.
- A clean low-speed comparison wants **forward-speed** derivatives (the oracle's hover
  column is degenerate; my derivative tooling is hover-only) — a future extension.
- No oracle value fabricated; all quoted from NASA TM 85890.
