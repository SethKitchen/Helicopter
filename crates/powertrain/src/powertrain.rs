//! The [`Powertrain`] trait — the polymorphism boundary for the motor + ESC.

/// Maps mechanical shaft power to the electrical power drawn from the pack.
pub trait Powertrain {
    /// Combined motor + ESC efficiency (0, 1] at the given shaft power (W).
    fn efficiency(&self, mech_power: f64) -> f64;

    /// Electrical power drawn from the pack to deliver `mech_power` at the shaft.
    /// `P_elec = P_mech / eta`.
    fn electrical_power(&self, mech_power: f64) -> f64 {
        mech_power / self.efficiency(mech_power)
    }
}
