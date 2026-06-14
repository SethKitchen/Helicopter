# Milestone 6 — external validation: predictions, recorded BEFORE comparison

This file is the falsifiable record. The whole stack so far is validated against
*itself* (closed forms, reduction-to-known-case, derivative signs, route-vs-route).
Milestone 6 is a **category change**: match published data for ONE real, documented
aircraft. Before entering any oracle, we predict where the model will match and
where it will carry error — turning the comparison into a test of our understanding
of the model's *own* error structure, not just of the model. If the mismatches land
where predicted, that is the emergent-validation pattern one level up.

**Hard rule (unchanged):** never fabricate oracle values. The aircraft parameters
*and* the published results must come from a single citable source (so the
comparison is apples-to-apples), or we say so and stop. Candidate sources: Padfield,
*Helicopter Flight Dynamics* (Bo-105 and UH-60 parameter sets + stability
derivatives tabulated); GARTEUR / ADS-33 literature. Choose the aircraft whose FULL
parameter set is findable, not the one with the nicer published results.

## Ordering principle (least-approximated first)

Compare first where the model is least approximated, so a match is meaningful and a
mismatch is diagnostic:

1. **Trimmed control positions vs. airspeed** — collective, longitudinal &
   lateral cyclic, pedal across the speed range. These rest on the **force/moment
   balance**, the part validated hardest (hover cross-check to 0.3%, milestones
   5a–5c), and are forgiving of inflow detail.
2. **Stability derivatives** (Mu, Mq, Zw, Xu, Lv, Lp, Nr, Nv, …) vs. published.
   This is where κ-calibration and uniform-inflow error first show their magnitude.

## Predictions

Tagged by the *named, isolated* approximation each leans on, so a mismatch
quantifies that specific approximation rather than producing one uninterpretable
"off by 20%."

### Expected to MATCH well (force/moment-derived; clean of κ)

- **Trim collective vs. speed** — the power-bucket *shape* / collective dip then
  rise. Force balance (thrust = weight) is exact-ish; PREDICT correct trend and
  magnitude within ~10–15%.
- **Longitudinal cyclic vs. speed** — nose-down stick-forward trend with speed.
  PREDICT correct sign and trend; magnitude within ~15–20% (flap modelling).
- **Stability derivatives Mu, Mq, Zw, Xu and lateral Lv, Lp, Yv, Nr, Nv** — all
  force/moment-based (validation ledger: clean of the κ caveat). PREDICT correct
  **signs** (these emerged unprompted in 5c/5e) and the right **order of
  magnitude**; quantitative error ~20–35% from uniform inflow + rigid blade.
- **Open-loop hover instability** — the unstable oscillatory pitch–speed mode.
  PREDICT the simulator reproduces the *qualitative* instability (it already does,
  unprompted, as every real helicopter is hover-unstable) and the period/growth to
  within a few tens of percent.

### Expected to CARRY ERROR (and which approximation owns it)

- **Anything power/torque-derived** — owned by the **κ calibration** (anchored at
  hover; CLAUDE.md ledger says treat no power-derived quantity as independent).
  PREDICT larger error off-hover; do NOT report a power match as a model success.
- **High-speed (high-μ) derivatives & trim** — owned by **uniform inflow**. The
  real lateral inflow gradient and advancing/retreating asymmetry are
  under-modelled. PREDICT growing error with airspeed; lateral/cross-coupling
  derivatives at high μ the worst.
- **Heave damping Zw and inflow-timescale-sensitive terms** — partly owned by the
  **Pitt–Peters finite-state inflow** (good lag scale, but 3-state). PREDICT
  reasonable but not tight.
- **Anything needing elastic blade modes / higher harmonics** — owned by the
  **rigid-blade, first-harmonic-flap** scope. PREDICT real error wherever blade
  flexibility or 2/rev+ matters (vibration, high-frequency response). Low-frequency
  trim/derivatives least affected.
- **Absolute control *gradients* (deg/(m/s))** — depend on control phasing and
  flap, which the first-harmonic rigid model approximates. PREDICT right trend,
  loose magnitude.

### The diagnostic payoff

If the force/moment derivatives match (signs + order of magnitude) while the
power-derived and high-μ quantities miss by the predicted amounts, that is success:
the external test will have *measured* each scoped approximation (κ, uniform inflow,
rigid blade, first-harmonic flap) separately, because the internal structure is
clean. A mismatch is a measurement, not a failure.

## Status

Predictions recorded (above, before any comparison). **Data sourced** (web sourcing
authorized): the chosen aircraft is the **UH-60A**, from a single public, citable,
apples-to-apples source — NASA TM 85890 (GENHEL) — which holds both the parameters
and the oracle (Table 4 trim control positions vs. airspeed; Tables 12+ stability
derivatives). The flybar-confounded R-50 and the export-restricted Fletcher TM were
deliberately rejected. Captured parameter set + citation: `UH60_GENHEL_TM85890.md`.
That file also lists **UH-60-specific** fidelity caveats (canted tail rotor,
stabilator, SC1095 airfoil) as expected-error sources *separate from* the general
approximations above — so a mismatch there is attributed to the right cause.

Next focused step (unblocked): build `Aircraft::uh60()`, run hover trim + derivatives,
compare against the Table 4 / Table 12 oracle and the predictions above. No oracle
numbers fabricated — all quoted from NASA TM 85890.
