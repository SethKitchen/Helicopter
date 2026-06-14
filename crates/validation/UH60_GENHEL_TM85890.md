# UH-60A Black Hawk — sourced external-validation dataset (Milestone 6)

**Single citable, public, apples-to-apples source** (parameters AND the comparison
oracle are in the same report):

> Howlett, J. J., *UH-60A Black Hawk Engineering Simulation Program: Volume I —
> Mathematical Model*, NASA TM 85890 / USAAVSCOM TM 84-A-2, 1981.
> NTRS: https://ntrs.nasa.gov/api/citations/19840015585/downloads/19840015585.pdf

Why this source and aircraft:
- **Public.** The Fletcher hover-ID report (NASA TM 110362) is the other standard
  reference but carries a U.S.-Government distribution restriction — deliberately
  NOT used.
- **Flybar-free, full-size.** The Yamaha R-50 dataset (arXiv 0804.4757) is clean and
  open but has a Bell–Hiller stabilizer bar my model cannot represent — its
  mismatches wouldn't map to *my* named approximations. UH-60 has no flybar.
- **One source for both halves.** This report tabulates the configuration parameters
  (Table 1) AND the validation oracle: **Table 4** (trim control positions vs.
  airspeed → milestone-6 comparison #1) and **Tables 12+** (dimensional stability
  derivatives → comparison #2). GENHEL is the accepted detailed UH-60 reference,
  validated against flight test, so matching it tests our model against the standard.

All values below are quoted from Table 1 (extracted via `pdftotext -layout`, line
numbers in the extracted text); SI conversions are ours.

## Main rotor (Table 1)
| quantity | published | SI |
|---|---|---|
| radius RMR | 26.83 ft | 8.178 m |
| chord CMR | 1.73 ft | 0.527 m |
| rotational speed Ω | 27.0 rad/s | 27.0 rad/s |
| number of blades | 4 | 4 |
| Lock number γ | 8.1936 | — |
| hinge offset e | 0.04659 (4.66%) | — |
| blade twist | −0.3142 rad | −18.0° (linear) |
| solidity σ | 0.0821 | — |
| lift-curve slope a | 5.73 /rad | — |
| longitudinal shaft tilt (fwd) | 0.05236 rad | 3.0° |
| hub stationline / waterline | 341.2 / 315.0 in | — |
| CTmax | 0.1846 | — |

## Tail rotor (Table 1)
| quantity | published | SI |
|---|---|---|
| radius | 5.5 ft | 1.676 m |
| rotational speed Ω | 124.62 rad/s | 124.62 rad/s |
| Lock number γ | 3.3783 | — |
| solidity σ | 0.1875 | — |
| lift-curve slope a | 5.73 /rad | — |
| blade twist | −0.3142 rad | −18.0° |
| hub stationline / waterline | 732.0 / 324.7 in | — |

## Mass & inertia (Table 1)
| quantity | published | SI |
|---|---|---|
| gross weight | 16 400 lb | 7 439 kg |
| Ixx (roll) | 5 629 slug·ft² | 7 632 kg·m² |
| Iyy (pitch) | 40 000 slug·ft² | 54 233 kg·m² |
| Izz (yaw) | 37 200 slug·ft² | 50 437 kg·m² |
| Ixz (cross) | 1 670 slug·ft² | 2 264 kg·m² |
| CG stationline / waterline | 360.4 / 247.2 in | — |

## Derived geometry (from station/waterlines, 1 in = 0.0254 m)
- tail-rotor arm (TR hub STA − CG STA) = 732.0 − 360.4 = 371.6 in = **9.44 m**
- main-hub height above CG (MR hub WL − CG WL) = 315.0 − 247.2 = 67.8 in = **1.722 m**
- tail-hub height above CG = 324.7 − 247.2 = 77.5 in = **1.969 m**

## Notes / fidelity caveats specific to the UH-60 (added expected-error sources)
Beyond the model's general approximations (κ-calibrated power, uniform→Pitt–Peters
inflow, rigid blade, first-harmonic flap), the UH-60 has features our generic model
does **not** represent — predict extra error from these, separately from the general
approximations:
- **Canted tail rotor** (20° cant) — couples TR thrust into pitch/heave; ours is
  upright.
- **Horizontal stabilator** (variable incidence, scheduled) — affects pitch trim &
  Mq/Mw, especially in forward flight; ours has no stabilator.
- **SC1095/SC1094-R8 airfoils, nonlinear −18° twist** — ours uses a NACA-0012-class
  section and linear twist.
- Fuselage drag is given as drag/lift/moment vs. AoA (figs/tables), not a single
  flat-plate `f`; the UH-60 equivalent flat-plate area (~35 ft²) must itself be
  sourced/estimated for the parasite term.

## Control rigging (Table 1 sensitivities + Table 2 mixing) — for the stick comparison
Pilot stick → blade pitch. Table 1: CK1=0.04939 rad/in (long cyclic), CK2=0.02792 rad/in
(lat cyclic), C5=0.2286 rad & C6=0.02792 rad/in (collective), C7=0.1743 rad &
C8=−0.07734 rad/in (pedal); CAIS, CBIS (swashplate cyclic at zero stick) blank → 0.
Table 2 feedforward (in/in): SK1=SK5=SK9=SK10=1.0 (stick→same-axis control). Table 2
crossfeed (in/in): SK4=−0.1640 (collective→long cyclic), SKM2=−0.5746 (pedal→long
cyclic), SK8=−0.16 (collective→lat cyclic), SK11=−0.2889 (collective→directional). Table 2
feedback SKV(3,2), SKV(6,1) (rate→cyclic): SAS, zero in steady trim. **PBA (p.6):** the
pitch-bias actuator adds to *total* longitudinal cyclic vs pitch attitude/rate/airspeed
(pitch-attitude loop active at hover, 15% authority); gain is in **ref 2, NOT here** →
longitudinal-cyclic comparison is confounded (see mapping #11).

## Table 4 oracle (level-flight trim, 1.0-kt / hover column) — stick positions & attitude
δe=0.1266 in, δa=0.2321 in, δc=5.719 in, δp=−1.279 in; vB=−0.0061 ft/s, wB=0.1485 ft/s;
Euler Θ=+5.052°, Φ=−2.340°. (δe/δa/δc/δp are pilot stick inches; Δδ=0.1 in perturbation.)

## Status
Dataset SOURCED and captured (parameters above; oracle = Table 4 trim positions and
Tables 12+ derivatives, same report). ENTERED into `Aircraft::uh60()`; hover derivatives,
trim attitude, collective, and cyclic comparisons DONE (see MILESTONE6_RESULTS.md).
No oracle numbers were fabricated; all are quoted from NASA TM 85890.
