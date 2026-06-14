# Tip-loss path-inconsistency fix — pre-registered prediction (before the fix)

## What this banks, and why now

The BEMT-bias characterization (MILESTONE6_RESULTS.md "BEMT bias") surfaced a real
model inconsistency: the dynamics/trim aero path `longitudinal_main_aero → loads()`
applies **no Prandtl tip-loss factor**, while the milestone-1 hover BEMT `solve_hover`
(which produced the clean C&T ~20–27% C_T figure) does. Two code paths for the same
physics, disagreeing.

Fixing it (adding tip loss to `loads()`) is a model-physics change that ripples through
5c–5m exactly like the gyro term — so it is **deferred to its own milestone**, NOT done
on a characterization turn.

But the characterization turn produced a **load-bearing argument** that currently lets
both prior external-validation results stand without disturbing anything validated:

> "Tip loss barely touches the *derivatives* (perturbations, where a near-tip
> multiplicative load factor largely divides out in a difference) while directly biasing
> the *absolute* trim collective."

That is the explanation I'd *want* to be true — it reconciles "Zw/Mu matched at
12%/sign" with "trim collective biased ~56% at fixed collective" without re-opening the
derivative comparison. It is physically plausible (tip loss ≈ a multiplicative load
reduction near the tip; a common multiplicative factor divides out of a first-order
difference) AND the milestone-6 derivative matches are independent evidence for it.
**But it is an argument, not a measurement.** It is the same shape as the cg_offset
"trivially trivial" claim that was only trustworthy *after* it was measured (≤1e-6).

So: commit the prediction the argument makes, **now, while the reasoning is fresh**, so
the eventual fix turn is a **test of today's claim** rather than a fresh rationalization
that conveniently agrees with whatever happens.

## Locked predictions (to be checked WHEN tip loss is added to `loads()`)

Setup of the eventual test: add Prandtl tip loss to `longitudinal_main_aero`'s blade-
element integral (the `solve_hover` form), rebuild `uh60()`, re-run derivatives + trim.

1. **Derivatives HOLD (the cancellation claim) — stated RELATIVE, the resolution the
   physics actually lives at.** The cancellation argument is fundamentally a *relative*
   claim: a ~multiplicative near-tip load factor **divides out of the perturbation
   ratio** `∂T/∂w ÷ T` to first order. So the derived falsifier is **relative, not an
   absolute band** — the derivatives must be insulated *relative to how much the level
   moves*:
   - **Primary (derived) falsifier:** let `s_trim` = the fractional shift in trim
     collective (prediction 2 — expected *tens of %*) and `s_deriv` = the fractional
     shift in each force/moment derivative. The cancellation claim predicts
     **`s_deriv ≪ s_trim`** — commit **`s_deriv < ¼·s_trim`** for each of `Zw`, `Xu`,
     `Mu`, `Mq`, `Lp`. Rationale: the level shift is the *full* `(1−F̄)` load reduction
     (F̄ the span-integrated tip-loss factor); the derivative shift is only the
     *second-order* residual from F's nonuniformity *across the loading-change region*,
     which is a small fraction of `(1−F̄)`. If `s_deriv` is an appreciable fraction of
     `s_trim`, the factor is NOT dividing out and the cancellation story is wrong —
     **even if `s_deriv` is itself a "small" absolute number.** This is the
     laundering-as-pass case the absolute band would miss.
   - **Secondary (sanity, NOT derived) band:** as a coarse cross-check, each derivative
     shift should also land **< ~5% absolute**. This **5% is a generous round margin,
     not a tolerance the cancellation argument predicts** — it is here only to flag a
     gross movement. A shift that is <5% absolute but ≥¼·`s_trim` is a **partial-
     cancellation FAILURE** (report it as "weak cancellation," not "confirmed, <5%"). A
     shift >5% absolute is an unambiguous fail. The primary relative test governs; the
     absolute band only adds a floor.
   - Either way the outcome is the one to *want to catch*: a real (even partial) failure
     means the omission was biasing a validated derivative and the milestone-6 matches
     were partly luck/compensation — do not explain it away.

   **1a. Ordering sub-claim (a distinct, separately-falsifiable prediction — do NOT fold
   it into 1).** The rate dampings `Mq`, `Lp` should shift **strictly less** than the
   velocity-derived force/moment derivatives `Zw`, `Xu`, `Mu`, because post-fix the rate
   dampings are **gyro-dominated** (the aero-independent −2 term carries most of them, so
   the aero tip-loss change touches a smaller share of them). **Falsifier with its own
   meaning:** if the ordering INVERTS — `Mq`/`Lp` shift *more* than `Zw`/`Xu`/`Mu` — that
   is NOT a generic cancellation failure; it is evidence that **tip loss and the gyro
   term INTERACT** (a more disruptive finding than weak cancellation). Report an
   ordering inversion as its own signal, named, not absorbed into the pass/fail of 1.

2. **Trim collective RISES toward the oracle (the bias claim).** Adding tip loss reduces
   thrust at fixed collective ⇒ thrust=weight needs MORE collective ⇒ my hover trim
   root collective rises from 19.29° toward the oracle 22.25°, shrinking the **13%-lower
   gap** toward single digits.
   - **Direction is the firm commit.** Magnitude: the gap should shrink but **not
     vanish/overshoot** — a genuine C&T-consistent ~20–27% C_T over-prediction remains,
     so the trim collective should stay **≤ oracle** (still somewhat low), not jump above
     it. Falsifier on direction: collective falls or stays flat. Falsifier on magnitude:
     collective overshoots above 22.25° (would mean tip loss over-corrected ⇒ some other
     compensating term was hiding).

3. **The two claims are coupled and must BOTH land.** If (2) holds (trim moves a lot)
   but (1) fails (derivatives also move a lot), the "biases trim but not derivatives"
   split was false — the omission was a global thrust-level error, not a
   cancels-in-perturbation one. If (1) holds and (2) holds, the argument is **upgraded
   from argument to measurement**, and the BEMT bias's path-dependence is confirmed as
   the stated cause.

## Status
Argument banked as falsifiable prediction. The fix itself is deferred (own milestone,
5c–5m revalidation, like the gyro adoption). When it runs, it tests this file.
