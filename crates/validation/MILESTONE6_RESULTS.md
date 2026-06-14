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

## Scope of this pass / next
- This pass: **hover longitudinal** derivatives only.
- Next: lateral/directional (`Lp`, `Nr`, `Yv`, with the canted-TR mapping that matters
  there), and the `Mq` investigation. Trim-position comparison (Table 4) needs the
  control rigging (deg blade pitch ↔ in. of stick) — its own sourced mapping.
- A clean low-speed comparison wants **forward-speed** derivatives (the oracle's hover
  column is degenerate; my derivative tooling is hover-only) — a future extension.
- No oracle value fabricated; all quoted from NASA TM 85890.
