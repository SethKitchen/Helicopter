# Battery cell model — external validation PRE-REGISTRATION

**Status: predictions LOCKED before the discharge-test oracle is sourced.**
Per the project's hard lesson (the first external comparison is the one
irreversible epistemic moment — lock everything before you see the oracle), this
file is written and committed *before* any new measured discharge data is fetched.
The cell model (`TheveninCell`, OCV curve + single series `R`) was built and fitted
ONLY on the Samsung INR18650-25R. Applying it to the four 21700 benchmark cells and
comparing to *their* published behaviour is therefore a genuine external test on
cells the model was not built on.

## What is already "seen" (build inputs — NOT oracles)
- Datasheet capacity / voltage window / mass / label current (used to construct the cells).
- Battery Mooch **measured DCIR** (P50B 9.5, JP40 5.4, BAK 6.0, 40PL 5.1 mΩ) — used as `R`.
- Battery Mooch **true continuous** ratings (JP40 45 A, BAK 30 A) — used in `true_continuous_current`.

These cannot serve as clean oracles (the model has already absorbed them). The
emergent-continuous comparison below is therefore labelled **semi-external** and
weighted accordingly.

## Oracle to be sourced (NOT yet seen at write time)
Measured **delivered capacity vs C-rate** and **loaded-voltage / sag** for the
P50B (and others where available), from published discharge tests (manufacturer
discharge curves, Battery Mooch / lygte-info capacity tables).

## Predictions (locked)

**P1 — Delivered capacity is nearly flat across C-rate (within a few %).**
The model ends discharge when terminal `V = OCV(soc) − I·R` reaches the 2.5 V
cutoff. With `R` only a few mΩ, the `I·R` sag is small, so predicted delivered Ah
falls only slightly from low to high rate. Quantitative: predict the P50B delivers
**≥ 95 % of its 0.2C capacity at 2C (~10 A)** and **≥ 88 % at 10C (~50 A)**.

**P2 — Direction of error: the model OVER-predicts high-rate capacity.** It is
purely ohmic; it omits rate-dependent diffusion / concentration polarisation that
grows at high C. So at the highest rates the *measured* capacity should fall a bit
MORE than predicted (model optimistic). Predict the gap is small for these
low-impedance tabless cells (they genuinely hold capacity well — independent
testing already noted "retains >95 % at 10C" for JP40), so the over-prediction is
≤ ~5 percentage-points at 10C.

**P3 — Voltage sag ≈ I·R, and the model UNDER-predicts measured sag.** Predicted
mid-SoC sag at 30 A on the P50B ≈ 30 × 9.5 mΩ = **0.29 V**. Measured sag should be
LARGER than this ohmic-only figure (real sag adds polarisation that grows during
the pulse) — the same direction as the 25R fit, where the fitted `R` (21 mΩ) had to
sit above the DCIR (14.8 mΩ). Predict measured 30 A sag ≥ predicted, by up to ~1.5×.

**P4 — Emergent continuous current (SEMI-external).** The thermal-envelope
`discharge_continuous` (cell `R` + 2-node thermal + cooling) should: (a) order the
cells by resistance and capacity, and (b) under **natural** convection come out
LOWER than Mooch's true-continuous ratings, because Mooch's de-rated numbers are
not a still-air steady-80 °C result. Predict natural-air emergent continuous is in
the tens of amps and **below** the sourced 45 A (JP40) / 30 A (BAK); matching the
sourced numbers should require forced cooling — and the dominant uncertainty is the
convection coefficient `h` (a stated assumption, exactly like the R22 autorotation
`f`/`C_d0`). This is a "right order + right ordering + error attributable to a named
input" test, NOT a precision match.

## Parameter mapping (where the cells don't map 1:1 onto the model)
- **OCV curve** is a shared representative NMC shape, not a per-cell measured curve
  → the near-cutoff knee (which sets high-rate capacity) may differ per cell.
- **`R`** is the 25 °C ohmic-ish DCIR; omits the time-growing polarisation → P3 direction.
- **Thermal**: specific heat is the generic 900 J/(kg·K); 21700 geometry/area set in
  `ThermalEnvelope::for_21700`; cooling `h` is an explicit assumption (P4).
- **Test conditions** (ambient, airflow, cutoff criterion) of the sourced data are
  not fully controllable → P4 weighted as semi-external.

## Falsifiers
- P1 fails if measured capacity drops > ~12 % by 10C (would indict the ohmic-only model).
- P3 fails if measured sag is BELOW the ohmic `I·R` (would mean `R` is over-stated).
- P4 fails if natural-air emergent continuous EXCEEDS the sourced ratings (would
  mean the thermal model under-predicts heating).
