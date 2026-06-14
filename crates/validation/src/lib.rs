//! Published-oracle validation cases for the hover BEMT solver.
//!
//! Each benchmark rotor is a [`oracle::ValidationCase`] — a trait object — so the
//! runner in [`oracle`] treats every case uniformly and new benchmarks drop in
//! without changing the runner.
//!
//! One concept per module:
//! * [`oracle`]         — the [`oracle::ValidationCase`] trait + generic runner.
//! * [`caradonna_tung`] — the primary oracle (Caradonna & Tung 1981, C_T vs θ).
//! * [`harrington`]     — secondary figure-of-merit sanity check (Harrington 1951).

pub mod caradonna_tung;
pub mod harrington;
pub mod oracle;

pub use caradonna_tung::CaradonnaTung;
pub use harrington::HarringtonRotor1;
pub use oracle::{OraclePoint, PointResult, ValidationCase, run_case};
