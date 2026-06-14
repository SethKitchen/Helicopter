# Shaft-tilt term — pre-registered prediction (BEFORE the fix), with a BACKWARD-reaching
# falsifier into the UH-60 cg_offset attribution

## What this banks, and why now
The BO-105 hover trim localized its longitudinal miss (θ1s≈0, Θ≈0 vs B1S=−0.42°, Θ=+2.64°)
to the **omitted 3° forward shaft tilt** (the trim residual has no shaft-tilt term). Both
external airframes omit it (UH-60 TM 85890 Table 1: 3.0° = 0.05236 rad; BO-105 CR-3144 Table
III-1: 3° fwd). Adding it is deferred (a sign-sensitive trim-residual change; default 0 to
spare UH-60/demo/5a–5m, like `gyro_rate`). But it carries a **falsifier that reaches backward
into a result already closed** — the UH-60 cg_offset attribution — so the prediction is banked
now, while the reasoning is fresh, so the future shaft-tilt turn TESTS today's claim about
cg_offset rather than re-rationalizing it.

## The backward-reaching claim (the leading hypothesis)
The UH-60 nose-up attitude (+5.05° oracle) was recovered by setting **cg_offset = 0.488 m**
(sourced: STA 360.4 − 341.2 = 19.2 in), which gave **Θ = +5.94° (cg-only, 18% over)**. At the
time the 18% over-shoot read as "loose end of the band (omitted stabilator + small hover
terms)." The BO-105 exposes a **better explanation: cg_offset was doing two jobs** — the real
CG lever AND the missing shaft tilt's nose-up — so the sourced 0.488 + the absorbed shaft tilt
summed to ≈ the right total, and the 18% over-shoot is **the seam showing**. With only the
UH-60 this is invisible: "cg_offset = CG lever" and "cg_offset = CG lever + absorbed shaft
tilt" are **degenerate on one airframe, separable across two.** This is the parameter-sum
fortuity risk (tracked on Lp) reappearing on the trim side.

## Cross-airframe separability (the arithmetic that makes the prediction sharp)
- **Shaft tilt is ~geometric:** vertical-izing a 3°-tilted thrust in hover pitches the
  fuselage ~+3° nose-up on *any* aircraft (minus small flap/cyclic), roughly airframe-
  independent.
- **BO-105 isolates it** (cg≈0.018 m → cg-only Θ=+0.09°): so Θ_oracle +2.64° ≈ shaft(~+2.6°)
  alone. The missing +2.55° IS the shaft tilt.
- **UH-60 then tests additivity:** Θ_oracle +5.05° = shaft(~+3°) + true-cg-lever. ⇒ the *true*
  cg contribution should be **~+2°**. But my model gives **+5.94° from cg=0.488 alone** ⇒ the
  cg_offset→attitude response is **~2–3× too strong**, i.e. cg_offset over-attributed.

## Locked predictions (to check WHEN shaft tilt is added; both params INDEPENDENTLY sourced)
Add a shaft-tilt term to the trim residual (default 0). Set BOTH airframes from sourced data,
no tuning: shaft_tilt = 3° (both), cg_offset = 0.488 m (UH-60, station lines) / 0.018 m
(BO-105). The attitudes are **PREDICTIONS, not targets.**

1. **BO-105 (validates the shaft-tilt mechanism):** Θ moves +0.09° → **~+2.6° (in-band with
   +2.64°)**, and θ1s moves toward the small −0.42°. *Falsifier:* if Θ does not reach ~+2.6°,
   shaft tilt is not the full longitudinal story.
2. **UH-60 (the backward-reaching falsifier):** with the sourced cg=0.488 AND shaft=3° BOTH
   in, Θ **OVERSHOOTS +5.05° substantially (predict ~+8–9°)** — confirming cg=0.488 was
   over-compensating (the +5.94≈+5.05 was coincidence). *Direction is the firm commit; the
   magnitude is approximate (the coupled 6-eqn trim re-solves, not a literal sum).*
   - **Outcome A (overshoot, predicted):** cg_offset was absorbing the omission. The cg→
     attitude response is too strong OR a nose-down term is missing (candidate: the
     stabilator/fuselage hover moment). The 18% cg-only over-shoot was the early warning, not
     measurement looseness. A real attribution finding on a closed result.
   - **Outcome B (lands ~+5.05°):** cg and shaft tilt are NOT simply additive in the coupled
     solve; the original cg-only attribution stands and the two effects weren't redundant.
   Either outcome resolves a question that was **invisible with one airframe.**

## The discipline that keeps this from becoming a 2-parameter fit (locked)
Two parameters (cg_offset, shaft_tilt) and one attitude target per airframe ⇒ under-determined
per-airframe; one could trade between them to hit +5.05°. **What breaks the degeneracy: both
values are INDEPENDENTLY SOURCED** (CG from station lines, shaft tilt from Table 1 / III-1) and
the attitude is then a **prediction, not a target.** If both sourced values land both airframes'
attitudes in-band, that is a real **two-airframe validation of the trim geometry.** If I find
myself adjusting either to hit a target, I have re-entered fitting — STOP. Pre-register both
sourced values AND the predicted attitudes for BOTH airframes before wiring (done above).

## Footprint / adoption
Shaft-tilt term defaults to 0 (every prior milestone unchanged); set −3°/3° on the two real
aircraft. Touches the trim residual (force/moment rotation by the shaft angle) — sign-sensitive,
so gate on a known case (e.g. zero flap/cyclic hover: a γ_s shaft tilt ⇒ Θ≈γ_s). Its own
focused step with BOTH airframes revalidated; NOT rushed at a session tail. Derivatives are
perturbations about trim ⇒ shaft tilt should barely move Zw/Mu/Mq/Lp (a rotation of the
operating point), but that is itself a prediction to check, not asserted.

## OUTCOME (verified — `shaft_tilt` field + residual rotation; 159 tests pass)
Implemented as `Aircraft.shaft_tilt` (default 0; demo unchanged), resolving the main-rotor
thrust into body axes through γ_s in `residual.rs`. Set both real aircraft from sourced data
(UH-60 0.05236 rad, BO-105 3°), NEITHER re-tuned. **Derivatives + demo-based 5a–5m bit-for-bit
unchanged** (the dynamics/derivative path does not read `shaft_tilt`; only the trim residual
does) — so the gyro/derivative comparison is untouched.

|  | cg-only (shaft off) | cg + shaft (both sourced) | oracle |
|---|---|---|---|
| **BO-105** Θ | +0.09° (the miss) | **+2.84°** (7.6%) | +2.64° |
| **UH-60** Θ | +5.94° (**the seam**) | **+7.82°** (~55% over) | +5.05° |

- **P1 CONFIRMED — BO-105 validates the shaft-tilt mechanism** (cg≈0 isolates it): Θ +0.09°→
  +2.84° (in-band), θ1s into order with B1S=−0.42°. The mechanism is RIGHT.
- **P2 CONFIRMED — Outcome A.** cg-only gives +5.94° (≈ oracle +5.05°, the seam); the SOURCED
  shaft tilt pushes Θ to **+7.82°, overshooting +5.05° by ~55%.** So **cg_offset=0.488 was
  over-attributing** the omitted shaft-tilt nose-up — the +5.94≈+5.05 was parameter-sum
  fortuity. Because the BO-105 validated shaft tilt in isolation, the overshoot is attributable
  to cg, **not** to a shaft-tilt bug. Exactly the question that was *invisible on one airframe*.
- **Discipline held:** both params independently sourced, attitude PREDICTED not targeted; I did
  NOT trade between cg and shaft to recover +5.05°. The overshoot is reported, not fitted away.

## What the overshoot localizes (the NEXT investigation — banked, not fixed by tuning)
With both sourced terms the UH-60 longitudinal attitude **over-responds by ~55%.** Shaft→attitude
is correct (BO-105), so the excess is on the cg side: either the **cg_offset→attitude gain is too
strong** (the cg-moment → β1c-flap → thrust-tilt → Θ chain over-responds — possibly the
hub-spring/flap balance), OR a **nose-down term is missing** (leading candidate: rotor-wake
download on the UH-60 horizontal STABILATOR at hover — a surface the model omits, and the BO-105
result is clean partly because its stabilizer effect is small). Held as a hypothesis, not
asserted; the resolution is a model study, NOT a re-tune of the two sourced parameters.
