//! The shared mission context — the rotor + powertrain + thermal environment a
//! hover or climb analysis runs in. Grouping it keeps [`crate::analyze_hover`] and
//! [`crate::analyze_climb`] to the few inputs that actually vary between calls
//! (gross mass, and for a climb the rate and duration).

use crate::endurance::MissionConfig;
use helisim_airfoil::Airfoil;
use helisim_bemt::Config;
use helisim_pack::Pack;
use helisim_powertrain::Powertrain;
use helisim_rotor::{Operating, Rotor};
use helisim_thermal::{Cooling, ThermalLimits};

/// Borrowed rotor + electrical + thermal context for a mission analysis.
#[derive(Clone, Copy)]
pub struct MissionScenario<'a> {
    pub rotor: &'a Rotor,
    pub op: &'a Operating,
    pub airfoil: &'a dyn Airfoil,
    pub pack: &'a Pack,
    pub powertrain: &'a dyn Powertrain,
    pub cooling: &'a dyn Cooling,
    pub limits: ThermalLimits,
    pub bemt_cfg: &'a Config,
    pub mission_cfg: &'a MissionConfig,
}
