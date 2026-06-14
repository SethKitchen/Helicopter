# BO-105C — sourced second-airframe validation dataset (Milestone 6, cross-aircraft)

**Single citable, public source** (parameters AND oracle in one document):

> Heffley, R. K., Jewell, W. F., Lehman, J. M., Van Winkle, R. A., *A Compilation and
> Analysis of Helicopter Handling Qualities Data — Volume One: Data Compilation*, NASA
> CR-3144 (STI TR 1087-1), August 1979.
> NTRS: https://ntrs.nasa.gov/api/citations/19800002851/downloads/19800002851.pdf
> (Vol 2, analysis: NASA CR-3145, citation 19790023051 — used only for context, not params.)

## Why this airframe / source (the cross-aircraft rationale)
- **Public, single-source, sourceable by curl + pdftotext** (same workflow as TM 85890).
- **Hingeless (soft-in-plane) rotor** — the ADVERSARIAL test for the gyro flap-damping
  term derived & validated on the *articulated* UH-60. Different hub mechanics, much
  stronger hub moment ⇒ stresses whether `gyro_rate=−2` is correct *physics* (generalizes
  with parameter changes only) or was UH-60-specific.
- **Modeled as an equivalent flapping hinge** (p.67: "flexible blade attachment modeled as
  an equivalent flapping hinge; parameters matched on the natural frequencies") — SAME
  model *structure* as ours (hinge-offset ν_β), so the flap comparison is apples-to-apples.
- **Derivative data contains NO cross-product-of-inertia (Ixz=0)** (p.67) — matches our
  diagonal-inertia model exactly (UH-60 had Ixz=1670 we omitted as a named error; here the
  oracle omits it too, removing that error source).
- **Derivatives are in SI units** — removes the English→SI conversion trap entirely.
- **No pitch-bias actuator confound on the BO-105 longitudinal axis** — reopens the
  longitudinal-cyclic comparison the UH-60 PBA blocked (mapping #11).

## Airframe parameters (Table III-1, p.67–68; transcribed, no tuning)
Main rotor: 4 blades, R=4.91 m, chord=0.27 m, NACA 23012 mod, **hingeless**, twist −8°
linear, shaft tilt 3° fwd, **424 rpm for the tabulated data** (⇒ Ω=44.40 rad/s, tip speed
218 m/s), hub FS 98.44 / WL 61.2, **blade flapping inertia I_β=219.50 kg·m²**.
Tail rotor: 2 blades, R=0.95 m, chord=0.18 m, zero twist, gear ratio 5.24 (⇒ Ω=232.7
rad/s), hub FS 335 / WL 68.7 / BL −12.5. Horizontal stabilizer: 0.809 m², AR 8.09, QC at
FS 277.5 / WL 25.84, zero incidence.
Control travels (Fig III-2): collective 22.86 cm (9 in); long cyclic 30.78 cm (12.12 in);
lat cyclic 21.97 cm (8.65 in); pedal 11.02 cm (4.34 in).

## Mass & inertia — Figure III-3b (rendered from the PDF graphic; text layer was empty)
The CASE 29 hover oracle is at **2096 kg, MID CG** = the "Nominal Weight" row:
| condition | mass | Ixx | Iyy | Izz | Ixz |
|---|---|---|---|---|---|
| Nominal (CASE 29) | **2096 kg** | **1803** | **4892** | **4428** | **0** kg·m² |
| Heavy | 2300 kg | 1924 | 5063 | 4515 | 0 |
| Light | 1814 kg | 1638 | 4655 | 4298 | 0 |

## Oracle (Table III-3, CASE 29 = 0 KT, sea level, 2096 kg, mid CG) — hover derivatives
Body-fixed (FRL) axis, SI units. Matrix: rows X, Z, M, L′, N′ (force/moment derivatives) ×
columns u, w, q, v, p, r, δc, δB, δA, δp (controls = collective, lon-cyc, lat-cyc, pedal).
COMPLETE — all of Xu, Zw, Mu, Mq + lateral Lp, Nr, Yv, Nv + control derivatives present.
The exact per-row NORMALIZATION (M-row appears inertia-normalized, 1/s — Mq≈−3.4 is
1/s-scale) is to be pinned from the report's format-definition section as a units-hygiene
step BEFORE the value comparison (the UH-60 units discipline). **Values intentionally not
transcribed here** to limit the pre-comparison leak (see prereg honesty note).

## The ONE parameter NOT in CR-3144 — flap frequency ν_β (equivalent-hinge stiffness)
CR-3144 gives geometry + inertia + *output* derivatives, but NOT the internal rotor-model
ν_β (the hingeless stiffness) — the headline parameter for the gyro test. By the PBA
precedent (don't reach into a cited background ref to inject a match-forcing parameter), it
is **not pulled from Ref 4 / the DLR literature as a single number.** Resolution (locked in
the prereg): **bracket ν_β across the hingeless physical range [1.08, 1.15]** (e_eff ≈
[0.10, 0.18] via ν_β²=1+1.5e/(1−e)) and require the gyro conclusion to hold across the
whole bracket. Lock number γ IS computable from CR-3144 (ρ, c, R, I_β) + a lift slope.

## Oracle hover values (CASE 29, read cleanly from the rendered page — units 1/s, per m/s)
Xu −0.0166, Zw −0.3317, Mq −3.3972, Mu +0.0663; Yv −0.0320, Lp −9.2439, Lv −0.2075,
Nr −0.3270, Nv +0.0325. Trim: θMR (main collective) 14.32°, θTR 10.17°, Θ 2.64°, Φ −2.97°.
(Inertia/mass-normalized: a raw −3.4 N·m/(rad/s) would be negligible for 2096 kg ⇒ M-row
is /Iyy.)

## Status
SOURCED, ENTERED (`Aircraft::bo105()`), and COMPARED — hover derivatives DONE
(`dynamics/tests/bo105_external_validation.rs`; outcome in MILESTONE6_BO105_PREREG.md).
Headline: the gyro flap-damping term (−2, unchanged from UH-60) GENERALIZES to the
hingeless rotor (Lp/Mq deficit 0.11× without it → ~1× with it, order-consistent across the
ν_β bracket); BEMT over-prediction 3rd sighting (collective 14% low). No oracle values
fabricated; all quoted/rendered from NASA CR-3144.
