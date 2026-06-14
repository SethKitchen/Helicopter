# Milestone 6 — parameter-mapping decisions, LOCKED before the comparison

The comparison is the **one irreversible epistemic moment** in the project: the first
time the model meets ground truth, and its value depends entirely on the predictions
AND the parameter mapping being fixed *before* the results are seen. The vulnerable
step is `Aircraft::uh60()` — wherever NASA TM 85890's Table 1 does not map one-to-one
onto our model, a judgment call enters, and each judgment call is a place where one
could (consciously or not) tune toward the Table 4 / Table 12 answer.

**Rule for this file:** every non-trivial mapping decision below is justified by
geometry/physics, recorded here, and LOCKED. None may be revised during the build on
the basis of whether it improves the match. If a decision must change, the change and
its physical reason are appended here with a timestamp — never silently.

## Direct, one-to-one (no judgment; just transcription from Table 1)

R=8.178 m, chord=0.527 m, N_b=4, Ω=27.0 rad/s, σ=0.0821, a=5.73 /rad, γ=8.1936,
hinge offset e=0.04659, MR precone=0, K_β=0 (⇒ ν_β from e alone — our model's form),
mass=7439 kg, longitudinal shaft tilt=3.0° fwd. Geometry from station/waterlines:
hub height 1.722 m, tail arm 9.44 m, tail height 1.969 m. (See `UH60_GENHEL_TM85890.md`.)
Consistency check (locked as a build assertion): σ = N_b·c/(πR) = 4·0.527/(π·8.178)
= 0.0821 ✓ — chord and solidity agree, so both can be entered without contradiction.

## Judgment calls — decided on physics, locked

1. **SC1095 airfoil → our NACA-0012-class section.** Decision: use our `LinearAirfoil`
   with the **report's lift-curve slope a = 5.73 /rad**; accept different camber
   (zero-lift angle), stall, and drag polar as unmodeled. Physics: the first-order
   rotor force/moment response is set by the *lift slope*, which we match exactly;
   camber shifts the zero-lift angle (a collective offset, absorbed in trim) and the
   drag polar (a power effect — already owned by κ, not a force/moment effect). So
   this choice is neutral for the *predicted-clean* force/moment derivatives.

2. **Canted tail rotor (cant angle K, UH-60 ≈ 20°/0.349 rad — confirm exact value
   in TM 85890 Table 1 at build).** Decision: resolve the TR thrust vector by the
   cant — lateral (anti-torque) component `T_tr·cos K`, vertical component
   `T_tr·sin K` — and apply BOTH: yaw moment `T_tr·cos K · arm`, an added vertical
   force `T_tr·sin K`, and the pitch moment `T_tr·sin K · arm`. Physics: the cant
   geometrically tilts the thrust; resolving by the angle is exact, independent of
   the trim answer. This is the headline judgment call — locked on geometry, NOT on
   whether it improves the yaw/heave trim. (If the build first runs uncanted, the
   cant is then a *named* error source, but the intended mapping is the resolution
   above.)

3. **Horizontal stabilator (variable incidence) → omitted (our model has none).**
   Decision: omit; do the **hover** comparison first, where the stabilator sees
   negligible dynamic pressure and its effect is small; name it as a *forward-flight*
   error source (it drives Mw/Mq and pitch trim with speed). Physics: stabilator load
   ∝ ½ρV² → ~0 at hover.

4. **Cross-inertia Ixz = 1670 slug·ft² → omitted (our `Inertia` is diagonal).**
   Decision: omit; name as a small lateral-directional coupling error. Physics:
   Ixz/√(Ixx·Izz) = 1670/√(5629·37200) ≈ 0.12 — small; the *predicted-clean*
   longitudinal derivatives (Mu, Mq, Zw, Xu) are unaffected by Ixz; only lateral
   roll-yaw coupling (Lp/Nr cross-terms) is modestly touched.

5. **Nonlinear −18° twist → linear −18° (−0.3142 rad).** Decision: enter the
   report's single twist value as our linear twist. Physics: −18° is the published
   total twist; the spanwise *distribution* difference is second-order for integrated
   trim/derivatives. Named as a minor unmodeled effect.

6. **Fuselage parasite f: report gives drag/lift/moment vs AoA (figs), not a single
   `f`.** Decision: the **hover** comparison is parasite-free (V=0) — run it first,
   `f`-independent. For forward flight, source `f` from the report's drag data (or the
   known UH-60 ≈ 35 ft²) *as its own sourced number*, not tuned. Physics: parasite
   power ∝ ½ρV³ → 0 at hover.

7. **TR pitch-flap coupling δ3 (FKITR=0.7002), TR precone (0.01309 rad).** Our TR is a
   simple thrust model and may not use δ3. Decision: if omitted, name it; it affects
   TR thrust phasing, a second-order anti-torque effect. Locked as named-if-omitted.

8. **Air density.** Decision: use the density at the report's stated trim/derivative
   validation condition (confirm altitude in TM 85890 at build; default standard sea
   level 1.225 kg/m³ if unspecified). Lock: match the oracle's flight condition, not a
   density that improves the match.

## Reporting structure for the comparison session (locked)

1. **Lead with the locked predictions** (`MILESTONE6_PREDICTIONS.md`) and these
   mappings — both stated before any oracle number is shown.
2. For each quantity report **two** results: match-vs-oracle (did we hit the UH-60
   number?) AND match-vs-our-predicted-error (did the error land where we predicted?).
   These are different findings.
3. **Flag every judgment call from this file** at the point where it could have
   influenced the number, so a reviewer sees exactly where model-fitting risk entered.
4. **Do not let a clean aggregate bury an individual mismatch we predicted should be
   clean.** A force/moment derivative we predicted would match but doesn't is the most
   informative number in the comparison — it means an approximation we thought was
   clean isn't. Surface it, don't average it away.
5. **Framing:** a mismatch is a *measurement of a named approximation*, not a failure.
   The goal is a per-approximation error budget (κ, uniform inflow, rigid blade,
   first-harmonic flap, + the UH-60-specific canted-TR/stabilator/SC1095), not one
   aggregate "off by X%".

## Execution hygiene — reading the numbers correctly (locked before measurement)

These are how a *spurious* mismatch (or match) sneaks in during execution. None
change the spec; all must be applied when the numbers come in.

1. **Units (the most likely spurious mismatch).** TM 85890 is English units (slug,
   ft, lb); our model is SI. Dimensional derivatives carry the trap: `Mu` is
   1/(ft·s) vs 1/(m·s), `Zw` mixes, `Mq` (1/s) is clean. Discipline: convert the
   oracle to SI **once, explicitly**, and write the converted table next to the
   original. A derivative off by a clean **3.281** (ft→m) or **14.594** (slug→kg) —
   or a product/ratio of these — is a **conversion bug, not a model finding**. Cross-
   check the **dimensionless** derivatives (unit-invariant) as a separate cleaner
   gate: if a non-dimensional form matches but its dimensional form doesn't, it's
   units, full stop.

2. **Sign conventions (bitten twice — flapping cyclic, lateral ±90°).** GENHEL has
   its own body-axis and control-sign conventions. Before comparing magnitudes,
   confirm our and GENHEL's definitions of positive `u`, positive `M`, positive
   control deflection agree; if they don't, **transform once, explicitly** — never
   flip a sign per-derivative "to match" (that's assert-not-derive in its purest
   form). `Mu>0` is the physics-meaningful sign (the speed-stability instability the
   whole hover dynamics rests on); it must be right for the right reason.

3. **Trim convergence ≠ trim accuracy.** Our Newton trim was validated on small
   rotors / our own parameter sets. The UH-60 (heavier, faster tip, different σ) may
   not converge from the default initial guess, or land in a different basin. Keep
   **"did trim converge"** and **"is the converged answer accurate"** as strictly
   separate questions. A convergence hiccup is a solver-robustness issue with a
   mechanical fix (speed/collective continuation, hover-analytic starting point) —
   it is **not** a physics mismatch and must not be logged as one.

4. **Too-good is a flag, same as too-far.** A derivative predicted at 20–35% error
   that lands within ~5% of GENHEL is **suspicious**, not a triumph: likely a
   compensating-error coincidence or a units artifact making two different numbers
   look equal. Give suspiciously-good matches a second look, symmetric with the
   predicted-clean-but-off headline.

## The one result to watch above all

Whether the four **longitudinal force/moment derivatives — `Mu`, `Mq`, `Zw`, `Xu`**
— land in their predicted band. These have the least approximation between our model
and the real aircraft (force/moment-based, clean of κ, longitudinal so untouched by
Ixz/canted-TR/stabilator). If those four land where predicted, the **core physics**
(not the control stack, not the calibrations) is externally confirmed, and every
larger mismatch above them is cleanly attributable to a named approximation. That is
the number that matters most when the session runs.
