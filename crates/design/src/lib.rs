//! Model-scale **sizing study** — the priorities made quantitative.
//!
//! The aero and powertrain milestones answer "what does *this* rotor do"; this
//! crate answers the design question on top of them: *given the goal of a safe,
//! buildable, long-flying, quiet model electric helicopter, what rotor should I
//! build?* It introduces **no new physics**. It composes the already-validated
//! cores — hover BEMT trim ([`helisim_mission`]), autorotation margin
//! ([`helisim_autorotation`]), and rotor noise ([`helisim_acoustics`]) — into one
//! [`DesignReport`] organised by the stated priority order:
//!
//! 1. **Safety** — vertical-autorotation descent rate, flare-height equivalent,
//!    autorotation index.
//! 2. *(Vertical integration / cost)* — reported as the physical drivers (size,
//!    mass proxy, RPM), not scored: they depend on how it is built, not the
//!    physics.
//! 3. **Airtime** — hover endurance (energy bound).
//! 4. **Efficiency** — figure of merit, power loading, disk loading.
//! 5. **Noise** — overall rotational-noise SPL, tip Mach.
//!
//! [`sweep_radius`] then walks the central trade (disk loading at fixed tip
//! speed) so the recommended point can be defended from the priority *ordering*
//! rather than collapsed into one fabricated objective function.
//!
//! # Validation character
//!
//! Being a composition, its tests are not new oracle comparisons but
//! **consistency + trade-direction** checks: the autorotation and noise numbers
//! must equal the underlying crates called directly (it really composes the
//! trusted models), and growing the disk must move every priority metric in the
//! physically correct direction (lower power, longer airtime, quieter, safer —
//! against larger/slower). See `tests/`.
//!
//! One concept per module:
//! * [`candidate`] — the [`DesignCandidate`] design point (builder knobs).
//! * [`report`]    — the [`DesignReport`] (computed consequences).
//! * [`metrics`]   — [`evaluate`]: compose the cores into the report.
//! * [`sweep`]     — [`sweep_radius`]: the disk-loading trade.

pub mod candidate;
pub mod metrics;
pub mod recommend;
pub mod report;
pub mod sweep;
pub mod upsizing;

pub use candidate::DesignCandidate;
pub use metrics::evaluate;
pub use recommend::{DesignSpace, Recommendation, ScoredCandidate, recommend};
pub use report::DesignReport;
pub use sweep::{SweepPoint, sweep_radius};
pub use upsizing::{
    UpsizeParams, UpsizeResult, hover_power_per_kg, required_pack_fraction, size_for_daily_life,
};
