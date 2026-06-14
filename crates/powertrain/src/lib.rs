//! Motor + ESC powertrain models mapping mechanical shaft power (from BEMT) to
//! the electrical power drawn from the pack.
//!
//! The [`Powertrain`] trait is the polymorphism boundary; start with a constant
//! efficiency and refine to a torque/RPM efficiency map later without touching
//! callers.
//!
//! One concept per module:
//! * [`powertrain`]          — the [`Powertrain`] trait.
//! * [`constant_efficiency`] — [`ConstantEfficiency`], a flat-η first cut.

pub mod constant_efficiency;
pub mod powertrain;

pub use constant_efficiency::ConstantEfficiency;
pub use powertrain::Powertrain;
