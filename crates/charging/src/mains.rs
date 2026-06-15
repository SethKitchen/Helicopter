//! Charging from a **residential 120 V AC socket** through an AC→DC charger.
//!
//! The deliverable DC power is set by the branch circuit, not the charger you wish
//! you had: a US 120 V general-purpose circuit is 15 A (or 20 A), and the NEC caps
//! a *continuous* load (≥3 h, which charging is) at **80 %** of the breaker rating
//! (NEC 210.20(A)/210.23(A)). So a 15 A circuit yields 120 V × 12 A = 1440 W of
//! real AC power; the charger then converts AC→DC at ~85–93 % (≈90 % typical).
//!
//! Sources: NEC continuous-load 80 % rule; charger AC-DC efficiency 85–93 %
//! (Battery University / onboard-charger studies). The efficiency is a stated,
//! overridable assumption.

use crate::source::ChargeSource;

/// A wall charger on a residential AC branch circuit.
#[derive(Clone, Copy, Debug)]
pub struct MainsCharger {
    /// Mains RMS voltage, V (US residential ≈ 120).
    pub mains_v: f64,
    /// Branch-circuit breaker rating, A (15 or 20 typical).
    pub circuit_a: f64,
    /// Continuous-load derate (NEC 80 % for loads ≥ 3 h).
    pub continuous_derate: f64,
    /// AC→DC charger conversion efficiency (≈0.90).
    pub charger_eff: f64,
}

impl MainsCharger {
    /// Standard US 15 A / 120 V general-purpose circuit.
    pub fn residential_15a() -> Self {
        MainsCharger {
            mains_v: 120.0,
            circuit_a: 15.0,
            continuous_derate: 0.80,
            charger_eff: 0.90,
        }
    }

    /// US 20 A / 120 V circuit (e.g. a dedicated kitchen/garage outlet).
    pub fn residential_20a() -> Self {
        MainsCharger {
            mains_v: 120.0,
            circuit_a: 20.0,
            continuous_derate: 0.80,
            charger_eff: 0.90,
        }
    }

    /// A 240 V Level-2 circuit (EV-charger class) at `breaker_a` amps — e.g. a
    /// 40 A breaker gives 240 V × 32 A continuous ≈ 7.7 kW AC. The big step up from
    /// a 120 V socket without going to DC fast charge.
    pub fn level2_240v(breaker_a: f64) -> Self {
        MainsCharger {
            mains_v: 240.0,
            circuit_a: breaker_a,
            continuous_derate: 0.80,
            charger_eff: 0.90,
        }
    }

    /// Real AC power the circuit can supply continuously, W (`V·I·derate`).
    pub fn ac_power_w(&self) -> f64 {
        self.mains_v * self.circuit_a * self.continuous_derate
    }
}

impl ChargeSource for MainsCharger {
    fn dc_power_w(&self) -> f64 {
        self.ac_power_w() * self.charger_eff
    }

    fn label(&self) -> String {
        format!(
            "{:.0} V / {:.0} A mains ({:.0} W AC × {:.0}% charger)",
            self.mains_v,
            self.circuit_a,
            self.ac_power_w(),
            self.charger_eff * 100.0
        )
    }

    /// AC energy drawn from the wall to deliver `delivered_wh` to the pack.
    fn input_energy_wh(&self, delivered_wh: f64) -> f64 {
        delivered_wh / self.charger_eff
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// SOURCED — a 15 A / 120 V circuit at the NEC 80 % continuous derate is
    /// 1440 W AC; at a 90 % charger that is 1296 W DC into the pack.
    #[test]
    fn residential_15a_power_matches_nec_and_efficiency() {
        let c = MainsCharger::residential_15a();
        assert!(
            (c.ac_power_w() - 1440.0).abs() < 1e-6,
            "AC {}",
            c.ac_power_w()
        );
        assert!(
            (c.dc_power_w() - 1296.0).abs() < 1e-6,
            "DC {}",
            c.dc_power_w()
        );
    }

    #[test]
    fn twenty_amp_circuit_is_stronger() {
        assert!(
            MainsCharger::residential_20a().dc_power_w()
                > MainsCharger::residential_15a().dc_power_w()
        );
        // 20 A: 120 × 16 × 0.9 = 1728 W DC.
        assert!((MainsCharger::residential_20a().dc_power_w() - 1728.0).abs() < 1e-6);
    }

    #[test]
    fn wall_energy_exceeds_delivered_by_charger_loss() {
        let c = MainsCharger::residential_15a();
        // Deliver 1000 Wh to the pack → 1000/0.9 ≈ 1111 Wh off the wall.
        assert!((c.input_energy_wh(1000.0) - 1111.11).abs() < 0.1);
    }
}
