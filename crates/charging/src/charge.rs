//! The CC/CV charge model — the standard Li-ion charge profile, run against the
//! Thévenin pack and a [`ChargeSource`], respecting the cell/BMS limits.
//!
//! * **Constant current (CC):** push the limiting current until the terminal
//!   voltage reaches the per-cell ceiling (`4.2 V × S`). The current is the
//!   smallest of: the recommended C-rate, the cell's max charge rating, and what
//!   the source can deliver into the present terminal voltage.
//! * **Constant voltage (CV):** hold the ceiling; the Thévenin model then sets the
//!   current as `(V_cv − OCV(soc))/R`, which tapers toward zero as the cell fills.
//!   Stop at a small cutoff current (≈C/50).
//!
//! The source-limited current solves the same power balance as the discharge
//! solve: `V·I = P` with `V = OCV + I·R` ⇒ `R·I² + OCV·I − P = 0` ⇒
//! `I = (−OCV + √(OCV² + 4RP))/(2R)`.

use crate::solution::ChargeReport;
use crate::source::ChargeSource;
use helisim_pack::Pack;

/// Charge knobs.
#[derive(Clone, Copy, Debug)]
pub struct ChargeConfig {
    /// Recommended CC charge rate (1/h). 0.5C is a sound life/speed compromise;
    /// the cell's max charge rating caps it.
    pub charge_c_rate: f64,
    /// CV-phase termination current as a C-rate (charge ends below this). ≈C/50.
    pub cutoff_c_rate: f64,
    /// Starting state of charge.
    pub soc_start: f64,
}

impl Default for ChargeConfig {
    fn default() -> Self {
        ChargeConfig {
            charge_c_rate: 0.5,
            cutoff_c_rate: 0.02,
            soc_start: 0.2,
        }
    }
}

/// Power-limited charge current at terminal voltage rise: solves
/// `R·I² + OCV·I − P = 0`.
fn power_limited_current(ocv: f64, r: f64, power_w: f64) -> f64 {
    (-ocv + (ocv * ocv + 4.0 * r * power_w).sqrt()) / (2.0 * r)
}

/// Charge `pack` from `source`, with each cell limited to `cell_max_charge_a` and
/// the recommended rate / cutoff from `cfg`.
pub fn charge(
    pack: &Pack,
    source: &dyn ChargeSource,
    cell_max_charge_a: f64,
    cfg: ChargeConfig,
) -> ChargeReport {
    let cap = pack.capacity_ah();
    let r = pack.internal_resistance(0.5);
    let v_cv = pack.ocv(1.0); // per-cell 4.2 V × S
    let p_source = source.dc_power_w();
    let parallel = pack.parallel as f64;

    // CC current ceiling = min(recommended C-rate, cell charge rating × P).
    let i_cc_limit = (cfg.charge_c_rate * cap).min(cell_max_charge_a * parallel);
    let i_cutoff = cfg.cutoff_c_rate * cap;

    let dt_s = 5.0;
    let dt_h = dt_s / 3600.0;
    let max_steps = 4_000_000; // ~5500 h backstop

    let mut soc = cfg.soc_start;
    let (mut cc_h, mut cv_h, mut e_in) = (0.0, 0.0, 0.0);
    let mut cc_current_a = 0.0;
    let mut source_limited = false;
    let mut timed_out = true;

    for step in 0..max_steps {
        if soc >= 1.0 {
            timed_out = false;
            break;
        }
        let ocv = pack.ocv(soc);
        let i_pow = power_limited_current(ocv, r, p_source);

        // CC unless holding the ceiling would need less current than the CC limit.
        let i_cv = (v_cv - ocv) / r; // current that holds exactly V_cv
        let in_cv = i_cv < i_cc_limit.min(i_pow);

        let (i, v) = if in_cv {
            // Constant voltage: current set by the cell, capped by the source.
            let i_pow_cv = p_source / v_cv;
            if i_pow_cv < i_cv {
                source_limited = true;
            }
            (i_cv.min(i_pow_cv).max(0.0), v_cv)
        } else {
            // Constant current: recommended/cell limit, capped by the source.
            if i_pow < i_cc_limit {
                source_limited = true;
            }
            let i = i_cc_limit.min(i_pow);
            (i, ocv + i * r)
        };

        if step == 0 || (!in_cv && i > cc_current_a) {
            cc_current_a = i; // the CC plateau current
        }

        // End in CV once the taper falls below cutoff.
        if in_cv && i <= i_cutoff {
            timed_out = false;
            break;
        }

        let dq = i * dt_h;
        soc = (soc + dq / cap).min(1.0);
        e_in += v * i * dt_h;
        if in_cv {
            cv_h += dt_h;
        } else {
            cc_h += dt_h;
        }
    }

    ChargeReport {
        source_label: source.label(),
        soc_start: cfg.soc_start,
        cc_current_a,
        cc_time_h: cc_h,
        cv_time_h: cv_h,
        total_time_h: cc_h + cv_h,
        energy_into_pack_wh: e_in,
        source_input_energy_wh: source.input_energy_wh(e_in),
        source_limited,
        timed_out,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mains::MainsCharger;
    use crate::solar::SolarArray;
    use helisim_cell::{max_charge_current, molicel_p50b};
    use helisim_pack::Pack;

    fn model_pack() -> Pack {
        // A representative model-heli pack: P50B 6S2P ≈ 21.6 V nom, 10 Ah, ~216 Wh.
        Pack::new(Box::new(molicel_p50b()), 6, 2)
    }

    fn p50b_charge_a() -> f64 {
        max_charge_current("Molicel P50B").unwrap()
    }

    /// On a wall socket the small pack is NOT source-limited (1296 W ≫ the 0.5C
    /// charge power), and the CC plateau is the 0.5C current = 0.5 × 10 Ah = 5 A.
    #[test]
    fn model_pack_on_mains_is_cell_limited_at_half_c() {
        let pack = model_pack();
        let r = charge(
            &pack,
            &MainsCharger::residential_15a(),
            p50b_charge_a(),
            ChargeConfig::default(),
        );
        assert!(!r.source_limited, "should be cell/C-rate limited");
        assert!(
            (r.cc_current_a - 5.0).abs() < 0.2,
            "cc current {}",
            r.cc_current_a
        );
        assert!(!r.timed_out);
        // 0.2→full at 0.5C: order ~1.6 h CC + CV taper; well under 3 h.
        assert!(
            r.total_time_h > 1.0 && r.total_time_h < 3.0,
            "t {}",
            r.total_time_h
        );
        // CV phase exists and the wall energy exceeds what reached the pack.
        assert!(r.cv_time_h > 0.0);
        assert!(r.source_input_energy_wh > r.energy_into_pack_wh);
    }

    /// CC time matches the closed form ΔSoC·cap/I (charge is coulomb-counting).
    #[test]
    fn cc_time_matches_closed_form() {
        let pack = model_pack();
        let r = charge(
            &pack,
            &MainsCharger::residential_15a(),
            p50b_charge_a(),
            ChargeConfig::default(),
        );
        // CC runs from soc_start to the CV knee; the delivered CC charge is
        // cc_current × cc_time. That charge ÷ cap is the SoC covered in CC.
        let soc_cc = r.cc_current_a * r.cc_time_h / pack.capacity_ah();
        assert!(soc_cc > 0.3 && soc_cc < 0.8, "CC covered ΔSoC {soc_cc}");
    }

    /// At the SAME requested rate, more panels = more power: 4 panels (~1.2 kW)
    /// meet a 2C charge of the small pack, a single ~310 W panel cannot, so it is
    /// source-limited to a lower current and charges slower.
    #[test]
    fn solar_source_limit_depends_on_array_size() {
        let pack = model_pack();
        let cfg = ChargeConfig {
            charge_c_rate: 2.0,
            ..ChargeConfig::default()
        };
        let big = charge(&pack, &SolarArray::typical(4), p50b_charge_a(), cfg);
        let one = charge(&pack, &SolarArray::typical(1), p50b_charge_a(), cfg);
        assert!(
            !big.source_limited,
            "4 panels (~1.2 kW) shouldn't limit a 2C charge"
        );
        assert!(
            one.source_limited,
            "one ~310 W panel should limit a 2C charge"
        );
        assert!(
            one.cc_current_a < big.cc_current_a,
            "one {} big {}",
            one.cc_current_a,
            big.cc_current_a
        );
        assert!(one.total_time_h > big.total_time_h);
    }

    /// Charging respects the cell rating: the CC current never exceeds
    /// cell_max_charge × P even if a high C-rate is requested.
    #[test]
    fn cc_current_capped_by_cell_rating() {
        let pack = model_pack(); // P=2
        let cell_max = p50b_charge_a(); // 25 A
        let r = charge(
            &pack,
            &MainsCharger::residential_20a(),
            cell_max,
            ChargeConfig {
                charge_c_rate: 100.0, // absurd request
                ..ChargeConfig::default()
            },
        );
        assert!(
            r.cc_current_a <= cell_max * 2.0 + 1e-6,
            "cc {} > {}",
            r.cc_current_a,
            cell_max * 2.0
        );
    }
}
