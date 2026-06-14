//! Battery cell equivalent-circuit models.
//!
//! The [`Cell`] trait is the polymorphism boundary: the pack and mission code
//! depend only on the trait, so any cell chemistry/model can be swapped in.
//!
//! One concept per module:
//! * [`cell`]     — the [`Cell`] trait.
//! * [`thevenin`] — first-order Thévenin model (OCV-SoC curve + series resistance).

pub mod cell;
pub mod thevenin;

pub use cell::Cell;
pub use thevenin::TheveninCell;
