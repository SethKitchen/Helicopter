//! Rotor **acoustics** — predicting how loud the rotor is, and which design knob
//! turns it down (priority: minimal sound).
//!
//! Electric propulsion removes engine and exhaust noise, which makes the
//! *rotor* the dominant source — so for a quiet electric helicopter the acoustics
//! live almost entirely in the rotor model already built. This crate turns the
//! aerodynamic state (thrust, torque, tip speed, blade count) into radiated sound.
//!
//! # Scope (deliberately bounded, first-principles where the physics allows)
//!
//! 1. **Rotational / loading noise** ([`rotational`]) — the tonal sound of steady
//!    blade loading sweeping around the disk, from **Gutin's** compact-source
//!    closed form. This is the quantitative, validated core. It needs a Bessel
//!    function, so we implement one ([`bessel`], std-only, validated against
//!    tabulated zeros and values).
//! 2. **Thickness-noise lever** ([`thickness`]) — the `∝ M_tip³` scaling that
//!    makes **tip speed the master noise knob**, exposed as a relative indicator
//!    (the full Farassat-1A thickness integral is out of scope).
//! 3. **Level bookkeeping** ([`spl`]) — pressures → dB re 20 µPa and the
//!    energy-summed overall level ([`NoiseSpectrum`]).
//!
//! # Deliberate limitations (documented, per project habit)
//!
//! * **Broadband floor** ([`broadband`]) — the random trailing-edge/vortex noise
//!   is now modelled by its **scaling laws** (6th-power tip-speed lever, blade
//!   area, distance), with the *absolute* anchor a required measured input, not a
//!   fabricated constant. Blade–vortex "slap" and the full thickness integral are
//!   still named and omitted — each would be its own module.
//! * **Compact source.** Gutin treats the loading as concentrated at an effective
//!   radius `R_e ≈ 0.8 R`; good for the low harmonics that dominate a subsonic
//!   rotor, degrading as the tip nears sonic.
//! * **No external SPL oracle yet.** Matching one published rotor's *measured*
//!   noise with its full geometry is a careful-sourcing task (à la the aero
//!   external-validation milestone) and is **not** faked here: validation is
//!   internal — exact Bessel values, Gutin's on-axis null and directivity,
//!   harmonic decay, and the monotone tip-Mach scaling — with external matching
//!   flagged as the next step.
//!
//! One concept per module:
//! * [`bessel`]     — integer-order `J_n(x)`, std-only.
//! * [`rotational`] — Gutin rotational-noise harmonic pressure.
//! * [`thickness`]  — the `∝ M_tip³` tip-speed noise lever.
//! * [`spl`]        — dB bookkeeping + spectrum assembly.
//! * [`solution`]   — [`NoiseSpectrum`] / [`Harmonic`].

pub mod bessel;
pub mod broadband;
pub mod rotational;
pub mod solution;
pub mod spl;
pub mod thickness;

pub use bessel::bessel_j;
pub use broadband::{BroadbandRef, broadband_oaspl_db, broadband_peak_hz, combined_oaspl_db};
pub use rotational::{RotorNoise, blade_passage_frequency, gutin_harmonic_pressure};
pub use solution::{Harmonic, NoiseSpectrum};
pub use spl::{P_REF, combine_rms, rotational_spectrum, spl_db};
pub use thickness::{thickness_noise_db_delta, thickness_noise_index};
