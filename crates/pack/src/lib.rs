//! Series/parallel battery pack built from a cell model.
//!
//! One concept per module:
//! * [`pack`] — [`Pack`]: `S` cells in series × `P` strings in parallel.

pub mod pack;

pub use pack::Pack;
