# UH-60 hover-attitude over-response — two-hypothesis study (step 1: the magnitude check)

The shaft-tilt verification left a localized issue: with both longitudinal params sourced
(cg_offset 0.488 m + shaft_tilt 3°), the UH-60 hover attitude **over-responds ~55%** (Θ +7.82°
vs oracle +5.05°). Two hypotheses were named (MILESTONE6_SHAFT_TILT_PREREG.md outcome):
(H1) the **cg→attitude gain is too strong**; (H2) a **missing nose-down term** — leading
candidate the rotor-wake download on the omitted horizontal stabilator. This is the
discriminating magnitude check, run BEFORE touching either sourced parameter.

## Pre-registration (before sourcing/computing)
The over-response is in the **nose-up** direction (+7.82° > +5.05°), so a *missing* term that
fixed it would have to be **nose-down**. But the stabilator sits **aft of the CG**, and a wake
**download (downward force) aft of the CG is a NOSE-UP moment** — the SAME sign as cg/shaft.
Predicted: (a) the stabilator download is nose-up; (b) magnitude order ~15–25% of the cg moment
(cg·W ≈ 35,600 N·m → ~5,000–9,000 N·m); (c) **if nose-up, it is the wrong sign to absorb a
nose-up over-response → H2 dies by SIGN, independent of magnitude, and H1 (gain) is implicated.**

## Sourced geometry (NASA TM 85890 Table 1) + back-of-envelope (no model change)
Stabilator area S_HS = 45 ft² = 4.18 m²; station STA_HS = 700.4 in; CG STA 360.4 in ⇒ arm =
8.64 m **aft** of CG. Hover induced velocity v_i = √(W/2ρA) = 11.9 m/s (W=72 977 N, A=210 m²).
Download D = ½ρV²·S·C_D at the tail, moment = D·arm. Over wake immersion (V from v_i at the
wake edge to 2v_i fully contracted) and C_D 1.0–1.5:

| V | C_D=1.0 | C_D=1.5 |
|---|---|---|
| v_i (11.9, edge) | 3 135 N·m (8.8%) | 4 702 N·m (13%) |
| 1.5 v_i (17.9) | 7 053 N·m (20%) | 10 580 N·m (30%) |
| 2 v_i (23.8, full) | 12 540 N·m (35%) | 18 809 N·m (53%) |

(% of the cg moment 35 613 N·m.) All **NOSE-UP**. A dynamic-pressure ratio η_HS<1 (Table 1, not
applied) would lower these; the realistic partial-immersion value is the lower end (~3–7 kN·m).

## Result — H2 (stabilator) RULED OUT by sign; H1 (gain) implicated
1. **Sign is decisive.** The stabilator download is NOSE-UP (download aft of CG). It cannot be
   the missing nose-down term; adding it to the model would make the over-prediction *worse*.
   H2-as-stated is dead — and my last-turn "missing nose-down term = stabilator" hypothesis was
   **sign-wrong**; the magnitude check corrected it (the point of running it).
2. **The real aircraft strengthens H1.** The real UH-60 HAS this nose-up stabilator download AND
   still trims LOWER (+5.05°) than my stabilator-free model (+7.82°). So to match +5.05° *with*
   an extra nose-up term, the real cg/shaft contribution must be even smaller than the raw 55%
   implies ⇒ **my cg→attitude gain is too strong, decisively** (the stabilator's presence widens
   the gap, not narrows it).
3. **Magnitude is moot here** (the user's "too small ⇒ gain implicated" is superseded by "wrong
   sign ⇒ gain implicated"): even the generous 53% upper bound can't help, being nose-up.

## What "gain too strong" points at (mechanism hypothesis, not asserted)
The chain cg_offset·wz (pitch moment) → β1c flap → thrust tilt → Θ (via the fx balance) over-
responds. Leading mechanism: the **hub-moment-per-flap / cg→flap response** — e.g. an
under-estimated effective hub stiffness means more β1c is needed to react the cg moment, more
thrust tilt, more Θ. Not asserted; it's the next model question.

## Step 2 — the positive confirmation (downstream, airframe choice now justified)
H2 ruled out leaves H1 the surviving explanation, but by *elimination* + the real-aircraft
argument, not yet a positive measurement. The clean positive test (pre-registered for whenever a
third airframe is built): an airframe with a **non-zero cg offset** where H1 predicts a
*proportional* over-response. **OH-6A** (CR-3144: small, articulated, conventional, well
documented) is the pick — IF it has a non-zero hover cg lever. Build it like the BO-105 (strict
mapping, no tuning), check whether its hover attitude over-responds in proportion to its cg
moment. Frozen-parameter discipline carries over: cg_offset and shaft_tilt stay at sourced
values on every airframe; the study is about the gain/term, never a re-trade of the two
disciplined parameters.

## Status
Step 1 DONE (this note): stabilator hypothesis ruled out by sign; cg→attitude gain implicated.
**Step 2 DONE (MILESTONE6_OH6A_GAIN_PREREG.md) — and it REFUTED the gain hypothesis.** The
OH-6A 3-point cg-sweep (dΘ/d(cg)=19.1 °/m, shaft-tilt-independent) is **reproduced at a physical
articulated hinge offset e≈0.05** — the cg→attitude gain is SOUND, not intrinsically too strong;
H1 (pre-registered as steeper ~2–2.5×) was FALSIFIED (believe-the-disagreement). The slope is
hub-stiffness-DOMINATED, so this discriminator is confounded (gain ↔ hub-stiffness entangled).
**Net resolution:** the UH-60 over-response (a single-point inference) is NOT a model-general
gain error — re-localized to UH-60-specific hub-stiffness / effective-ν_β under-modeling
(parameter-level) or the single-point decomposition. **Stabilator out (step 1, sign), gain out
(step 2, OH-6A) ⇒ no structural model fix indicated; the cg→attitude physics is sound.** Thread
CLOSED. All sourced from TM 85890 / CR-3144; no oracle value fabricated; nothing tuned.
