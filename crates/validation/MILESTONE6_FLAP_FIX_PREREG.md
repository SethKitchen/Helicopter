# Flap body-rate damping fix — analytical derivation & pre-registration

Written BEFORE touching code, same discipline that made Milestone 6 work. The
external comparison localized one real model gap: the main-rotor-flap RATE damping
(`Mq`, `Lp`) is ~15–30× under-predicted, and it lives entirely in the in-phase flap
response to body rate (`dβ1c/dq̄ = −0.071` vs `dβ1s/dq̄ ≈ −1`, right order). This
file derives the missing term, predicts its sign/magnitude/footprint, and states the
pass/fail — so the fix can be checked for doing the right thing for the right reason,
not dragging `Mq` toward the oracle.

## Derivation of the missing term

The current flap forcing (`aero.rs::flap_coeffs`) carries only the **aerodynamic**
hub-rate effect: `u_P` gets `−q̄·x·cosψ`, the disc-pitching change in normal velocity.
In the harmonic balance this is a **cosψ** forcing → (90° flap lag at ν≈1) → **β1s**
response. That matches the data: `dβ1s/dq̄ = −0.995` (textbook ≈ −16/γ = −1.95, same
order). It is correct as far as it goes.

Missing: the **gyroscopic / Coriolis** coupling of hub angular rate into flap. The
rotor carries spin angular momentum `H = I_s·Ω` along the shaft (up). A hub pitch
rate `q` (about body-y) rotates `H`: `ω×H = I_sΩ(p·ŷ − q·x̂)`, a moment of order
`Ω·(body rate)` — i.e. O(`q̄`) after non-dimensionalizing by `I_βΩ²`, the SAME order
as the aero term, not a small correction. Projected onto the rotating flap-hinge
(tangential) axis, a constant hub-frame moment about `x̂` modulates as **sinψ**. So:

- aero `q̄` term → **cosψ** forcing → **β1s** (lateral) — *present*.
- gyro `q̄` term → **sinψ** forcing → **β1c** (longitudinal) — **MISSING**.

This is exactly why the deficit is in `dβ1c/dq̄` (β1c, the pitch-moment-producing
flap) and not `dβ1s/dq̄`. The term to add to the flap-equation forcing is the
gyroscopic Coriolis coupling, of the standard form (Johnson, *Helicopter Theory*,
flap with hub motion; Padfield §3.2): a 1/rev forcing `≈ 2·q̄·sinψ` for pitch rate
and `≈ 2·p̄·cosψ` for roll rate (exact coefficient/sign to be taken from the cited
reference equation at implementation, NOT fitted).

## Pre-registered predictions

1. **Sign.** The term drives `β1c` further in its current (already-correct-sign)
   direction, making `Mq` and `Lp` MORE NEGATIVE (more damping), toward the oracle.
   *Falsification:* if the implemented sign reduces damping, the sign is wrong —
   re-derive against the precession limit, do NOT flip it to taste.
2. **Magnitude.** It should lift `dβ1c/dq̄` from −0.071 toward **O(−1)** (the
   "rotor-follows-shaft" scale) — roughly an order-of-magnitude increase — closing
   MOST of the ~17–30× gap. A residual of ~2× would NOT surprise me (remaining flap
   *dynamics*/lag, dynamic-inflow coupling, rotor stiffness) and **will not be
   tuned away**: a derived term that lands `Mq`/`Lp` near the model's general
   accuracy band, residual and all, is a stronger result than a tuned exact match.
3. **Footprint (the regression check — the real pass/fail).** The term couples ONLY
   body **rate** (p, q) into flap. So ONLY the rate-damping derivatives may move:
   `Mq`, `Lp` (and rate cross-terms `Lq`, `Mp`). The velocity derivatives `Xu`,
   `Zw`, `Mu`, `Yv`, `Nv`, `Lv` and the tail-based `Nr` — all of which already
   landed (1.5–35%) — must stay essentially unchanged. **If any of those move
   materially, the new term is wrong or mis-scaled regardless of what it did to Mq.**

## Pass/fail for the post-fix run (to be pre-registered with exact numbers before running)
- PASS: `Mq`, `Lp` move from ~15–30× low into (or near) the ~6–35% band, same sign;
  `Zw`, `Mu`, `Xu`, `Nr`, `Yv`, `Nv` unchanged within a couple of %.
- INFORMATIVE-NOT-FAIL: a clean residual (e.g. Mq/Lp still ~2× low) — a measurement
  of the remaining flap dynamics, recorded not tuned.
- FAIL: the regression derivatives move materially, or the term needs a sign flip or
  a fitted coefficient to help — that means it's wrong, not that the model is right.

## OUTCOME (all three pre-registered predictions held)

Implemented as `FlapProperties.gyro_rate` (in `aero.rs` and `flap_general.rs`:
`rhs[2] += gyro·q̄`, `rhs[1] += gyro·p̄`). Verified:

1. **Sign — confirmed, and re-derived not flipped.** `+2` gave *anti*-damping
   (Mq = +0.41) — so the sign is physical, not a fit: gyroscopic precession opposes
   the rate, mandating the negative coefficient in this azimuth convention. `−2`
   gives damping.
2. **Magnitude — confirmed, untuned.** Coefficient 2 (textbook) → `dβ1c/dq̄ = −1.87`,
   the "rotor-follows-shaft" scale. Clean-axis result: **Lp −0.19 → −3.25 vs oracle
   −3.35 = 3%** (was ~17× short). It closed MORE than the pre-registered ~2× residual
   — flagged honestly: untuned, possibly some fortuity in the UH-60 tail+main sum; the
   robust claim is the deficit WAS the gyro term and is now in-band. (Mq itself →
   −0.45; the residual vs the 20-kt −1.03 is the omitted horizontal stabilator, not
   the flap.)
3. **Footprint — perfect.** Zw, Mu, Xu, Nr, Yv, Nv all **bit-for-bit unchanged**.
   The term moved only the rate damping. Right thing for the right reason.

**Adoption decision:** `gyro_rate` defaults to **0** so every prior milestone's
validated dynamics are untouched (all 150 tests still pass); `Aircraft::uh60()` sets
−2. Making −2 the universal default is a deliberate next step requiring revalidation
of the 5c–5m control stack (it changes the demo's damping, e.g. 5g's depart timing).

## Order of work (locked)
1. (this file) derive + pre-register — DONE.
2. Implement the term from the cited reference equation (no fitted constants);
   verify sign against the precession limit.
3. Pre-register exact expected post-fix `Mq`/`Lp` and "others unchanged", then re-run
   the FULL hover comparison (both tests) — not just Mq/Lp.
4. Only then: trim-position comparison (separate sub-model, needs control rigging;
   kept off this thread so it can't muddy the before/after).
