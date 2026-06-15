//! The [`ChargeSource`] trait — the polymorphism boundary for "what pushes power
//! into the pack". Both a wall charger and a solar array are charge sources; the
//! CC/CV model ([`crate::charge`]) depends only on the trait, so any source (DC
//! fast-charger, generator, …) drops in.

/// Something that can deliver DC power into the pack.
pub trait ChargeSource {
    /// Steady DC power available to the pack at the source's rated condition, W
    /// (already net of the source's own conversion losses — e.g. the AC→DC charger
    /// efficiency, or the PV array's MPPT + derate).
    fn dc_power_w(&self) -> f64;

    /// Short human-readable description.
    fn label(&self) -> String;

    /// Energy the source must take *from its input* to deliver `delivered_wh` to
    /// the pack terminals (the upstream cost: AC from the wall, or PV-array energy).
    /// Default: lossless beyond `dc_power_w`.
    fn input_energy_wh(&self, delivered_wh: f64) -> f64 {
        delivered_wh
    }

    /// Energy the source can supply in a day, Wh — `None` for an effectively
    /// unlimited source (the grid), `Some` for an energy-limited one (solar).
    fn daily_energy_wh(&self) -> Option<f64> {
        None
    }
}
