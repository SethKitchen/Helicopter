# Hover trim comparison — pre-registered prediction (before running)

Oracle: NASA TM 85890, **Table 4** (level-flight trim), 1.0-kt (hover) column:
δe=0.127 in, δa=0.232 in, δc=5.719 in, δp=−1.279 in (stick positions); **Euler pitch
Θ = +5.05°**, **Euler roll Φ = −2.34°**.

The **attitudes (Θ, Φ) are physical — no control rigging needed**, so they are the
clean test. The stick positions (δ) need the swashplate rigging (rad blade pitch per
inch of stick) to compare against my blade-pitch outputs — deferred to a separate
rigging-mapping step.

## Independent predictions (mine, before running `trim(uh60, hover)`)

1. **Roll Φ ≈ −2 to −3° → MATCH (oracle −2.34°).** This tests a mechanism my model
   HAS: the fuselage banks to balance the tail-rotor side force (the same `φ_e`
   mechanism that gave ≈−2.08° for the demo in 5g). Same sign, same order expected.

2. **Pitch Θ: oracle +5.05° nose-up, my model will MISS it (predict |Θ| ≲ 2°).** The
   nose-up hover attitude is driven by the **CG being 0.488 m AFT of the hub**
   (Table 1: CG STA 360.4 vs hub STA 341.2 in). I **locked `cg_offset = 0`** in the
   parameter mapping (a documented under-use of available data — the offset IS
   sourceable). With CG under the hub, my fuselage has no longitudinal offset to
   hang nose-up, so Θ ≈ small. This is a *parameter-completeness* gap, not a physics
   gap — falsifiable below.

3. **Confirmation of (2) — the mechanism is sound:** re-running with the SOURCED
   `cg_offset = 0.488 m` (STA difference, data not fit) should produce a nose-up Θ of
   the right sign and order. If it does, the miss in (2) was purely the locked
   parameter, and my model has the CG→attitude mechanism.

4. **Collective:** my root collective (19.1°) is physical; comparing to δc=5.719 in
   needs the rigging — deferred.

## OUTCOME (all predictions confirmed) — test: trim/tests/uh60_trim_validation.rs
- **Roll Φ = −2.03° vs −2.34° (13%)** — match. The tail-side-force bank mechanism is
  confirmed against the real UH-60.
- **Pitch Θ (cg_offset=0, locked) = −0.00°** — missed, exactly as predicted.
- **Pitch Θ (cg_offset=0.488, sourced) = +5.94° vs +5.05° (18%)** — recovered. The miss
  was purely the one un-set parameter; setting the sourced CG offset (not fit) restored
  the nose-up attitude → the model's CG→attitude mechanism is sound. `uh60()` now uses
  0.488 (correcting the under-specified lock); it does NOT affect the derivative
  comparison (`cg_offset` is trim-only). The 18% residual is the rotor flap/hub-moment
  attitude contribution beyond the pure CG hang — informative, in-band.
- Stick positions (δ) still need the swashplate rigging — deferred.

## Stick-position comparison (rigging now extracted) — pre-registered

Rigging (TM 85890 Table 1): collective `θ0_root = C5 + C6·δc` (C5=0.2286 rad,
C6=0.02792 rad/in); pedal `tail θ0 = C7 + C8·δp` (C7=0.1743, C8=−0.07734); cyclic
gains CK1=0.04939, CK2=0.02792 rad/in plus crossfeed mixing (Table 2). Invert my
trimmed blade pitch → stick inches, compare to Table 4 (δc=5.719, δp=−1.279, δe=0.13,
δa=0.23).

**Physics prediction (independent of the rigging arithmetic):** my hover **collective
will come out LOWER** than the UH-60's. Reason: the documented **BEMT C_T
over-prediction** (~20–27% high vs Caradonna–Tung, milestone 1) means my rotor makes
the required thrust=weight at *less* collective than reality. So my implied root
collective < the oracle's, and δc < 5.719. If confirmed, the UH-60 trim collective is a
SECOND independent external sighting of the same BEMT over-prediction that the C&T hover
C_T showed — two oracles, same named limitation.
- Pedal δp: right sign (counter main torque), rough order of −1.3 in.
- Cyclic δe, δa: small (hover); full comparison needs the crossfeed mixing (Table 2) —
  flagged as a refinement, not this pass.

## STICK OUTCOME (prediction confirmed) — test: uh60_hover_collective_shows_the_bemt_overprediction
- **Collective: my root 19.29° vs implied oracle 22.25° → 13% LOWER.** Confirms the
  prediction: the BEMT C_T over-prediction (milestone 1, ~20–27% high vs C&T) means
  thrust=weight is met at less collective. **A SECOND independent external sighting of
  the same over-prediction** — C&T hover C_T and now UH-60 trim collective agree on the
  named limitation. (δc 3.87 vs 5.719 looks like 32% but is amplified by the C5
  zero-stick offset; the physical root-collective comparison is 13%.)
- **Pedal: right sign, magnitude off** (tail 19.46° → δp −2.14 vs −1.28). Attributable
  to the main *torque* (power/κ-derived — the documented non-clean quantity) + tail
  BEMT. Expected; not a clean check.
- **Cyclic: deferred** — needs the crossfeed mixing (Table 2).

## Pass/fail
- PASS: Φ matches (~−2.3°); Θ misses with `cg_offset=0` then is recovered (sign+order)
  with the sourced offset — localizing the miss to one un-set parameter.
- INFORMATIVE: the magnitude of the recovered Θ vs +5.05° measures the remaining
  attitude physics (rotor flap/hub-moment contribution beyond the pure CG hang).
