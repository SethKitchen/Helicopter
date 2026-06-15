//! **Actuation** — parametric, modular motor + control-servo selection.
//!
//! The structural parts of the build are already sized from physics (mast by
//! torsion, boom by bending, fasteners by load). This crate does the same for the
//! *active* hardware the model was previously hand-waving: it selects a real,
//! buyable **brushless motor** and the **swashplate/tail servos** for a design,
//! and does it **parametrically** so a larger aircraft takes a bigger member of
//! the *same* product family — the modularity the roadmap needs to scale a model
//! up to a human-usable electric helicopter.
//!
//! # The pattern (reused from `manufacture::fasteners`)
//!
//! 1. **Catalogue of real cited parts** — Scorpion HK/HKII motors
//!    ([`scorpion_hk_catalogue`]) and Align HV servos ([`align_hv_catalogue`]).
//!    The datasheet numbers are the validation oracle.
//! 2. **Demand derived from the design** — motor continuous power + a Kv/voltage
//!    feasibility gate; servo centrifugal **propeller-moment** control load
//!    ([`crate::loads`]).
//! 3. **Select smallest adequate** — the lightest part whose rating meets
//!    `demand · SF`, over the [`Selectable`] trait ([`select_smallest_adequate`]);
//!    chosen passes, next-down fails.
//! 4. **Scale honestly** — beyond the catalogue, extrapolate along the family's
//!    specific power / torque density and **flag** the regime change
//!    ([`crate::scaling`]) rather than fabricate a part.
//!
//! [`select_actuation`] ties it together into an [`ActuationPlan`].
//!
//! One concept per module:
//! * [`selectable`] — the [`Selectable`] trait + the smallest-adequate rule.
//! * [`motor`]      — [`BldcMotor`] + the Scorpion catalogue.
//! * [`servo`]      — [`Servo`] + the Align catalogue.
//! * [`loads`]      — design-derived demands (power, Kv gate, propeller moment).
//! * [`scaling`]    — beyond-catalogue extrapolation + the honest flag.
//! * [`plan`]       — [`ActuationPlan`] + [`select_actuation`].

pub mod loads;
pub mod motor;
pub mod plan;
pub mod scaling;
pub mod selectable;
pub mod servo;

pub use motor::{BldcMotor, scorpion_hk_catalogue};
pub use plan::{ActuationConfig, ActuationPlan, select_actuation, select_actuation_with};
pub use scaling::{Sized, size_or_extrapolate};
pub use selectable::{Selectable, select_smallest_adequate};
pub use servo::{Servo, ServoRole, align_hv_catalogue, kgcm_to_nm};
