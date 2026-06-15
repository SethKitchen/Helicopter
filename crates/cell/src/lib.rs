//! Battery cell equivalent-circuit models.
//!
//! The [`Cell`] trait is the polymorphism boundary: the pack and mission code
//! depend only on the trait, so any cell chemistry/model can be swapped in.
//!
//! One concept per module:
//! * [`cell`]     — the [`Cell`] trait.
//! * [`thevenin`] — first-order Thévenin model (OCV-SoC curve + series resistance).
//! * [`library`]  — sourced benchmark cells (Molicel P50B, Ampace JP40, BAK 45D,
//!   EVE 40PL) for the battery + BMS study.

pub mod aging;
pub mod cell;
pub mod library;
pub mod temperature;
pub mod thevenin;

pub use aging::{CalendarLoad, DegradationModel, equivalent_full_cycles};
pub use cell::Cell;
pub use library::{
    ampace_jp40, bak_45d, benchmark_cells, eve_40pl, max_charge_current, molicel_p50b,
    true_continuous_current,
};
pub use thevenin::TheveninCell;
