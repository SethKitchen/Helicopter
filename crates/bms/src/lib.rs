//! Battery management system (BMS) — the safety/estimation/sizing layer the bare
//! [`Cell`](helisim_cell::Cell)/[`Pack`](helisim_pack::Pack) models lack.
//!
//! Built cell- and topology-agnostic on purpose: every piece works per-cell or
//! per-target, so the same code manages a 6S model pack and a 96S human-scale
//! pack. That modularity is the whole point — the battery scales from a model
//! helicopter to a human-carrying one by changing the [`sizing::Target`], not the
//! BMS.
//!
//! One concept per module:
//! * [`protection`]    — OV/UV/OC/OT safe-operating envelope + fault classification.
//! * [`soc_estimator`] — coulomb counting + OCV re-anchoring (the standard pair).
//! * [`balancing`]     — series-string imbalance + passive balancing (weakest cell).
//! * [`sizing`]          — parametric S/P sizing to a (voltage, power, energy) target.
//! * [`benchmark`]       — the four library cells compared on one target.
//! * [`thermal_envelope`] — emergent (computed) continuous-current limit from the
//!   cell's R + 2-node thermal model + cooling (true continuous as an output).
//! * [`components`]      — purchasable catalog: sourced, dated, overridable prices + links.
//! * [`pack_build`]      — a sized topology → buildable BOM, tools, and assembly steps.

pub mod balancing;
pub mod benchmark;
pub mod components;
pub mod life_sizing;
pub mod pack_build;
pub mod protection;
pub mod sizing;
pub mod soc_estimator;
pub mod thermal_envelope;

pub use balancing::CellSpread;
pub use benchmark::{BenchmarkRow, best_by_mass, run_benchmark};
pub use components::{BomLine, Buildability, UnitPrice};
pub use life_sizing::{LifeSizing, size_for_life};
pub use pack_build::{PackBuild, build_pack};
pub use protection::{Fault, ProtectionLimits};
pub use sizing::{Limiting, PackSizing, Target, size_for_target};
pub use soc_estimator::SocEstimator;
pub use thermal_envelope::ThermalEnvelope;
