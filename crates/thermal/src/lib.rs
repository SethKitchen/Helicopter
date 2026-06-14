//! Lumped-mass cell thermal model.
//!
//! A single representative cell is treated as one isothermal lump:
//!
//! ```text
//! C dT/dt = Q_gen - Q_cool
//! Q_gen  = I^2 * R          (irreversible Joule + polarisation heat)
//! Q_cool = h * A * (T - T_ambient)   (Newtonian convective cooling)
//! C      = m * c_p          (lumped heat capacity)
//! ```
//!
//! The reversible (entropic) heat term is neglected — it is small compared with
//! I^2R at the high discharge rates that drive the safety question.
//!
//! One concept per module:
//! * [`cooling`] — the [`Cooling`] trait and the [`Convective`] model.
//! * [`lumped`]  — [`LumpedThermalCell`]: the heat-capacity lump and its step.
//! * [`limits`]  — [`ThermalLimits`]: the safe / warning / over-temp band.

pub mod cooling;
pub mod limits;
pub mod lumped;

pub use cooling::{Convective, Cooling};
pub use limits::{ThermalLimits, ThermalStatus};
pub use lumped::LumpedThermalCell;
