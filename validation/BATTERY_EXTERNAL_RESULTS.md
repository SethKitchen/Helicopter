# Battery cell model — external validation RESULTS

Compared against the predictions locked in `BATTERY_EXTERNAL_PREREG.md`. Test:
`crates/bms/tests/battery_external_validation.rs`. The `TheveninCell` model was
built only on the Samsung 25R, so this is a genuine external test on the four
21700 benchmark cells.

## P1 — Capacity retention at 10C: **CONFIRMED (clean external match)**
Sourced oracle: tabless 21700s "retain over 95 % capacity at 10C" (About:Energy
JP40 review / Battery Mooch). Model prediction (delivered Ah at 10C ÷ at 0.2C):

| Cell | 10C retention (model) | Oracle |
|------|----------------------|--------|
| Molicel P50B | 96.2 % | ≥95 % ✓ |
| Ampace JP40 | 98.4 % | ≥95 % ✓ |
| BAK 45D | 97.8 % | ≥95 % ✓ |
| EVE 40PL | 98.4 % | ≥95 % ✓ |

All four clear the 95 % floor — the low-impedance tabless cells genuinely hold
capacity, and the ohmic model captures it. **P2 (direction)** also holds: the
model sits at/above the measured floor (mildly optimistic, as predicted, because it
omits diffusion losses). Exact per-rate mAh are paywalled, so P2 is a direction
check, not a number.

## P4 — Emergent continuous current: **PREREG FALSIFIED, then a better result**
The prereg's P4 ("still-air surface-limited emergent continuous < rating") is
**falsified**, and per the project's ★ rule the disagreement is believed, not
reconciled. What the model actually says (still air = natural convection, 25 °C,
80 °C cutoff):

| Cell | steady-state surface | full-discharge core | full-discharge surface | **measured rating** |
|------|------|------|------|------|
| Ampace JP40 | **47 A** | 126 A | 249 A | 45 A |
| Molicel P50B | **36 A** | 63 A | 86 A | 35 A |
| BAK 45D | 45 A | 102 A | 161 A | 30 A |

**Three findings, each believed because the routes are independent:**

1. **A skin-temperature criterion is meaningless at high rate.** The full-discharge
   *surface* limit runs 160–250 A — because a 4–5 Ah cell at those currents empties
   in ~1 minute and the skin lags the core through `R_int`, never reaching 80 °C.
   The 2-node model is exactly what exposes this; the single-node model could not.
   The safety-relevant node is the **core**.

2. **The steady-state still-air surface limit emergently reproduces the measured
   continuous rating to ~4 % for two of three cells** — JP40 47 vs 45 A, P50B
   36 vs 35 A — with NO number fitted. Physically: Mooch's "true continuous" *is*
   "discharge in still air until the skin hits 80 °C", which at steady state is this
   limit. **The temperature-dependent `R` (task 6) is load-bearing here:** at 80 °C
   the Arrhenius factor drops `R` to ~0.18× its 25 °C value, so a hot cell generates
   far less heat and sustains roughly double the current a fixed-`R` model predicts
   (~20 A → ~45 A). Without temp-dependent `R` this match would not appear.

3. **BAK 45D is over-predicted (45 vs 30 A).** Believed, not fudged: BAK's 30 A is
   the datasheet's own conservative "without cutoff" figure, and independent testing
   flagged poor cell-to-cell consistency — its rating is more cautious than the
   shared-`R`/shared-cooling physics predicts. The model has no BAK-specific thermal
   data to do better; named, not patched.

**Dominant named uncertainty:** the convection coefficient `h` (still-air `h≈7.5`)
— a stated assumption, exactly the role `f`/`C_d0` played in the R22 autorotation
validation. This is a "right order + two clean ~4 % matches + one attributable
over-prediction" result, NOT a universal precision match, and is declared as such.

## Honest gaps (declared, never fabricated)
- **Per-rate delivered mAh and loaded voltage-sag curves** for these cells are
  paywalled (Battery Mooch Patreon) or published only as datasheet graphs → P2/P3
  are checked by direction, not exact numbers; **P3 (sag magnitude) is left as a
  gap** rather than reading numbers off a graph.
- **Per-cell measured OCV curves** are similarly unavailable in clean numeric form
  (About:Energy is login-gated); the shared representative NMC curve is retained and
  the gap is named (see `ORACLE_COVERAGE`).
- **Published full eVTOL pack** specs (cell-to-pack mass, S/P, BMS) are proprietary;
  no citable apples-to-apples pack oracle was found, so pack-level external
  validation is deferred rather than sourced from a non-citable figure.

## Process note (the project's own lesson)
The first external comparison is the irreversible epistemic moment, and the lesson
says it ideally runs as its own session. This one was run at the tail of a larger
build; the lock discipline was kept (predictions written and committed before the
oracle was fetched; a prediction was falsified and believed), but a reviewer should
treat the P4 numbers as a first, honest pass worth re-running deliberately.
