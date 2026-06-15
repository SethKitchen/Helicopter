//! [`ChargeReport`] — the outcome of a CC/CV charge: how long, how much energy,
//! and whether the source or the cell set the rate.

/// Result of charging a pack from a source.
#[derive(Clone, Debug)]
pub struct ChargeReport {
    /// Source description.
    pub source_label: String,
    /// Starting state of charge.
    pub soc_start: f64,
    /// Charge current on the CC plateau (initial), A.
    pub cc_current_a: f64,
    /// Constant-current phase duration, hours.
    pub cc_time_h: f64,
    /// Constant-voltage (taper) phase duration, hours.
    pub cv_time_h: f64,
    /// Total charge time, hours.
    pub total_time_h: f64,
    /// Energy delivered to the pack terminals, Wh.
    pub energy_into_pack_wh: f64,
    /// Energy drawn from the source's input (AC wall / PV array), Wh.
    pub source_input_energy_wh: f64,
    /// True if the source's power (not the cell/C-rate) capped the charge rate.
    pub source_limited: bool,
    /// True if the charge did not complete within the simulated time budget.
    pub timed_out: bool,
}

impl ChargeReport {
    /// Average charge power into the pack over the whole charge, W.
    pub fn average_power_w(&self) -> f64 {
        if self.total_time_h > 0.0 {
            self.energy_into_pack_wh / self.total_time_h
        } else {
            0.0
        }
    }

    /// Overall charge efficiency, energy into pack ÷ energy from source.
    pub fn efficiency(&self) -> f64 {
        if self.source_input_energy_wh > 0.0 {
            self.energy_into_pack_wh / self.source_input_energy_wh
        } else {
            1.0
        }
    }
}
