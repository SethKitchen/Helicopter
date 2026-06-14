//! **Cost + buildability** — priorities #2 (vertical integration) and #3 (cost),
//! the two the aero/safety stack did not touch.
//!
//! Your goal is to build as much as possible from raw materials. That makes the
//! interesting question not just "what does it cost" but "what can I *make*, and
//! what am I forced to *buy*". This crate answers both from a coarse aircraft
//! specification (mass split, motor power, pack energy) by building a bill of
//! materials where every line carries a **buildability** tag, then rolling it up
//! into a **vertical-integration index** and the list of irreducible buy-items.
//!
//! # Honesty about provenance (the hard rule applied to money)
//!
//! Costs are a **parametric model with named inputs** ([`UnitCosts`]), not sourced
//! market facts. The defaults are representative small-scale order-of-magnitude
//! figures, explicitly flagged as assumptions to override with real quotes. So
//! only the *relative* breakdown and the buildability split are treated as
//! findings; absolute totals are model-with-inputs. The buildability
//! `self_fraction` values are likewise a documented taxonomy, not measurements.
//!
//! # Validation character
//!
//! No physics, so no new oracle — the tests are accounting consistency
//! (subsystem costs sum to the total, fractions in `[0,1]`), monotonicity (a
//! bigger pack costs more and lowers the self-build index because cells are
//! bought), and that the taxonomy puts cells / ESC / sensors in the irreducible
//! buy-list. See `tests/`.
//!
//! One concept per module:
//! * [`component`] — a BOM line item + the [`Buildability`] taxonomy.
//! * [`costs`]     — [`UnitCosts`], the named cost inputs.
//! * [`bom`]       — [`build_bom`]: spec → bill of materials.
//! * [`report`]    — [`summarize`]: BOM → cost + vertical-integration findings.

pub mod bom;
pub mod component;
pub mod costs;
pub mod report;

pub use bom::{AircraftSpec, Bom, build_bom};
pub use component::{Buildability, Component};
pub use costs::UnitCosts;
pub use report::{CostReport, summarize};
