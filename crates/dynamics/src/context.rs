//! Shared rotor + flight-condition context.
//!
//! Every main-rotor aero entry point needs the same bundle — the rotor geometry,
//! its operating point, the section airfoil, the flap properties, the hub height
//! above the CG, and the (trim) controls. Grouping them into one borrowed struct
//! keeps the force/moment and inflow signatures narrow (the body state, rates and
//! inflow stay as explicit small arrays) without scattering six references through
//! every call.

use helisim_airfoil::Airfoil;
use helisim_flapping::{Controls, FlapProperties};
use helisim_rotor::{Operating, Rotor};

/// Borrowed rotor + condition context for the main-rotor aero functions.
#[derive(Clone, Copy)]
pub struct RotorAero<'a> {
    pub rotor: &'a Rotor,
    pub op: &'a Operating,
    pub airfoil: &'a dyn Airfoil,
    pub props: &'a FlapProperties,
    /// Hub height above the CG, m.
    pub hub_height: f64,
    pub controls: &'a Controls,
}
