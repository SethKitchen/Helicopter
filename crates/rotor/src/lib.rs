//! Rotor geometry and operating point for the helisim BEMT solver.
//!
//! One concept per module:
//! * [`rotor`]     — blade geometry as functions of the radial station `x = r/R`.
//! * [`operating`] — rotational speed and the fluid environment.

pub mod operating;
pub mod rotor;

pub use operating::Operating;
pub use rotor::Rotor;

/// Standard sea-level air density, kg/m^3.
pub const RHO_SEA_LEVEL: f64 = 1.225;
/// Speed of sound at ~15 °C, m/s. Maps tip speed to tip Mach number.
pub const SPEED_OF_SOUND: f64 = 340.0;
