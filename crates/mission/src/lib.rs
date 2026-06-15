//! End-to-end coupling of the aerodynamic and electrical models.
//!
//! The physics chain, all first-principles:
//! 1. BEMT gives mechanical shaft power `P_mech` at the hover operating point.
//! 2. Powertrain efficiency maps it to electrical power `P_elec = P_mech / eta`.
//! 3. The pack must supply `P_elec` as a *constant-power* load, so the current
//!    and the sagging terminal voltage are coupled — solved the same way as the
//!    BEMT inflow (bisection on a monotone residual). See [`electrical`].
//! 4. Integrating state of charge over time until the cut-off voltage gives the
//!    hover endurance. See [`endurance`].
//!
//! [`hover_mission`] ties it together: given a rotor, a pack, a powertrain, and a
//! gross mass, it trims the rotor to hover (thrust = weight), then answers the
//! question that needs all of the above at once — *can this pack hover this
//! aircraft, at what C-rate, and for how long.*
//!
//! One concept per module:
//! * [`electrical`]    — coupled constant-power pack-current solve.
//! * [`hover_trim`]    — find the collective that makes thrust = weight.
//! * [`endurance`]     — integrate SoC (and temperature) over a discharge.
//! * [`hover_mission`] — the end-to-end hover report.
//! * [`climb`]         — sustained-climb thermal safety check.

pub mod climb;
pub mod electrical;
pub mod endurance;
pub mod hover_mission;
pub mod hover_trim;
pub mod scenario;

pub use climb::{ClimbReport, analyze_climb};
pub use electrical::{ElectricalState, solve_pack_current};
pub use endurance::{
    EnduranceResult, MissionConfig, ThermalResult, simulate_discharge_thermal,
    simulate_hover_endurance,
};
pub use hover_mission::{HoverReport, analyze_hover};
pub use hover_trim::trim_hover_collective;
pub use scenario::MissionScenario;

/// Standard gravity, m/s^2.
pub const G: f64 = 9.80665;
