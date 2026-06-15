//! **Charging** — how to refill the pack the BMS protects.
//!
//! Two real-world sources, behind one [`ChargeSource`] trait so the CC/CV model
//! depends only on the abstraction:
//! * [`MainsCharger`] — a residential **120 V** AC socket through an AC→DC charger
//!   (circuit- and NEC-limited).
//! * [`SolarArray`] — a **solar PV** array through an MPPT controller
//!   (irradiance- and energy-limited).
//!
//! [`charge`] runs the standard constant-current/constant-voltage profile against
//! the Thévenin [`Pack`](helisim_pack::Pack), capping the current at the cell's
//! charge rating and at what the source can deliver — so the answer (charge time,
//! energy, and whether the *source* or the *cell* is the bottleneck) falls out of
//! the same physics the discharge/sizing side uses. It scales model→human by the
//! pack and source passed in.
//!
//! One concept per module:
//! * [`source`]   — the [`ChargeSource`] trait.
//! * [`mains`]    — [`MainsCharger`] (120 V residential + 240 V Level-2).
//! * [`solar`]    — [`SolarArray`] (PV + MPPT).
//! * [`fast`]     — [`DcFastCharger`] (high-power DC, the path to ~1:1).
//! * [`charge`]   — the CC/CV charge model.
//! * [`ratio`]    — charge:flight ratio (= P_flight/P_charge; size-independent).
//! * [`solution`] — [`ChargeReport`].

pub mod charge;
pub mod equipment;
pub mod fast;
pub mod mains;
pub mod ratio;
pub mod solar;
pub mod solution;
pub mod source;

pub use charge::{ChargeConfig, charge};
pub use equipment::{ChargeKit, EquipLine, kit_120v, kit_240v, kit_dc_fast, kit_solar};
pub use fast::DcFastCharger;
pub use mains::MainsCharger;
pub use ratio::{
    cell_charge_power_ceiling_w, charge_flight_ratio, charge_power_for_ratio, flight_time_h,
};
pub use solar::SolarArray;
pub use solution::ChargeReport;
pub use source::ChargeSource;
