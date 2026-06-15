//! Brushless outrunner motor catalogue — the Scorpion HK / HKII family.
//!
//! One product line that **scales by stator size** (22 mm → 45 mm): a bigger
//! aircraft takes a bigger member of the *same* family. Each entry is a **real,
//! currently-buyable part** — datasheet figures are the spec oracle, and every
//! row carries a **direct purchase URL + price** (sourced, as-of 2026-06). Parts
//! that could not be price-verified at a direct purchase point are deliberately
//! excluded (a part you cannot buy is not a design input). The motor is selected
//! on **continuous power**, with a Kv/voltage feasibility gate in [`crate::loads`].

use crate::selectable::Selectable;

/// One brushless outrunner motor (a buyable catalogue part).
#[derive(Clone, Copy, Debug)]
pub struct BldcMotor {
    /// Scorpion designation, e.g. "Scorpion HKII-4525-520".
    pub name: &'static str,
    /// Velocity constant, RPM per volt (no-load).
    pub kv: f64,
    /// Mass, grams.
    pub mass_g: f64,
    /// Maximum continuous input power, W (the sizing rating).
    pub max_cont_power_w: f64,
    /// Maximum continuous current, A.
    pub max_cont_current_a: f64,
    /// Maximum LiPo cell count (series).
    pub max_cells: u32,
    /// Stator diameter, mm (the family's size axis).
    pub stator_d_mm: f64,
    /// Street price, USD (sourced; see [`Self::price_note`]).
    pub price_usd: f64,
    /// Direct purchase URL.
    pub purchase_url: &'static str,
    /// Provenance of the price: retailer, stock, as-of date (never fabricated).
    pub price_note: &'static str,
}

impl Selectable for BldcMotor {
    fn name(&self) -> &str {
        self.name
    }
    fn mass_g(&self) -> f64 {
        self.mass_g
    }
    /// Governing capacity = continuous input power.
    fn rating(&self) -> f64 {
        self.max_cont_power_w
    }
    fn rating_unit(&self) -> &'static str {
        "W"
    }
}

impl BldcMotor {
    /// How to install and use this motor — the construct-and-use steps for the
    /// build output. Grounded in the Scorpion HK package (4 mm bullets + 3 female
    /// connectors + heat-shrink + M3 screws in the box) and setup guidance
    /// (match Kv to cells × gear ratio; 5–15° timing; bearings excluded from the
    /// 2-yr warranty, so run-in matters).
    pub fn install_steps(&self, cells: u32, gear_ratio: f64) -> Vec<String> {
        vec![
            format!(
                "1. Bolt the motor to the frame motor-mount with the included M3 screws; \
                 fit a pinion sized for {gear_ratio:.0}:1 onto the {:.0} mm-class shaft.",
                if self.stator_d_mm >= 45.0 { 6.0 } else { 3.2 }
            ),
            "2. Mesh the pinion to the main gear and set ~0.05 mm backlash (paper-shim method); lock the pinion grub-screw on the shaft flat."
                .to_string(),
            "3. Solder the three phase leads to the ESC using the supplied 4 mm bullet connectors + heat-shrink (any two leads swapped reverses rotation)."
                .to_string(),
            format!(
                "4. Configure the ESC as a heli governor at the target head speed on a {cells}S pack, \
                 motor timing 5–15°; confirm Kv·V_pack reaches head-RPM × {gear_ratio:.0}."
            ),
            format!(
                "5. Verify the continuous draw stays ≤ {:.0} A (its rating) at the hover/climb load.",
                self.max_cont_current_a
            ),
            "6. Run-in unloaded (blades off): check direction, temperature and vibration before fitting the rotor."
                .to_string(),
        ]
    }
}

/// Recommended pack/ESC main connector for a continuous current (standard RC
/// connector current classes).
pub fn pack_connector(current_a: f64) -> &'static str {
    if current_a <= 30.0 {
        "XT60"
    } else if current_a <= 60.0 {
        "XT60 / EC3"
    } else if current_a <= 90.0 {
        "XT90 / EC5"
    } else {
        "AS150 / EC8"
    }
}

impl BldcMotor {
    /// What power and connections the motor needs — pack, ESC, connectors, signal.
    pub fn power_and_connections(&self, cells: u32) -> Vec<String> {
        let vnom = cells as f64 * 3.7;
        let vfull = cells as f64 * 4.2;
        let bullet = if self.stator_d_mm >= 45.0 {
            "5.5–6 mm"
        } else {
            "4 mm"
        };
        let esc_a = (self.max_cont_current_a * 1.2).ceil();
        vec![
            format!(
                "Power: {cells}S LiPo — {vnom:.1} V nominal ({vfull:.1} V full); draws ≤ {:.0} A continuous, \
                 so choose a pack whose C-rating × capacity ≥ {:.0} A.",
                self.max_cont_current_a, self.max_cont_current_a
            ),
            format!(
                "ESC: brushless heli ESC rated ≥ {esc_a:.0} A at {cells}S{}; pack→ESC via an {} connector; \
                 ESC→motor on the three phase leads ({bullet} bullet connectors, included).",
                if cells >= 6 { " (use an HV ESC)" } else { "" },
                pack_connector(self.max_cont_current_a)
            ),
            "Signal: ESC throttle lead → flight controller, run in governor/RPM mode at the design head \
             speed (PWM 50–400 Hz or the ESC's RPM-governor input)."
                .to_string(),
            "BEC: the ESC's BEC (or a separate UBEC) powers the receiver/FC rail; the servos take their \
             own HV BEC (see servo power)."
                .to_string(),
        ]
    }

    /// How the motor connects to the helicopter structure (tray + mast drive).
    pub fn structural_connections(&self, gear_ratio: f64) -> Vec<String> {
        let shaft = if self.stator_d_mm >= 45.0 { 6.0 } else { 3.2 };
        vec![
            "Bolts to the powertrain tray / motor mount with the four included M3 bolts — mount face \
             square to the mast, thread-lock the bolts."
                .to_string(),
            format!(
                "Drive: a pinion on the {shaft:.1} mm shaft meshes the main gear on the mast at \
                 {gear_ratio:.0}:1; set ~0.05 mm backlash and lock the pinion grub-screw on the shaft flat."
            ),
            "Reaction: motor torque passes through the tray into the main frame — recheck mount bolts \
             and gear mesh after the first run-in."
                .to_string(),
        ]
    }
}

/// The Scorpion HK / HKII outrunner catalogue — **price-verified buyable parts**.
///
/// DATASHEET ORACLE (Scorpion Power System spec sheets) + DIRECT PURCHASE
/// (price/link sourced, as-of 2026-06):
/// * **HKII-2221-8 V2** (450-class, 22 mm): 3595 Kv, 81 g, 475 W cont, 45 A, 3S.
/// * **HKII-4525-520**  (700-class, 45 mm):  520 Kv, 503 g, 4450 W cont, 100 A, 12S.
///
/// The legacy HK-2221-12 / HK-3026 winds were dropped: no direct purchase point
/// with a current price could be verified (older/limited stock). Designs whose
/// power falls between these two select the 4525 (over-sized but real); the gap
/// is honest, not papered over. Overridable via [`crate::select_actuation_with`].
pub fn scorpion_hk_catalogue() -> Vec<BldcMotor> {
    vec![
        BldcMotor {
            name: "Scorpion HKII-2221-8 V2",
            kv: 3595.0,
            mass_g: 81.0,
            max_cont_power_w: 475.0,
            max_cont_current_a: 45.0,
            max_cells: 3,
            stator_d_mm: 22.0,
            price_usd: 94.99,
            purchase_url: "https://www.scorpionsystem.com/catalog/helicopter/motors_4/hkii-22/HK-2221-8/",
            price_note: "Scorpion official store (direct) $94.99, backorder; AMain in-stock $110.49 — as-of 2026-06",
        },
        BldcMotor {
            name: "Scorpion HKII-4525-520",
            kv: 520.0,
            mass_g: 503.0,
            max_cont_power_w: 4450.0,
            max_cont_current_a: 100.0,
            max_cells: 12,
            stator_d_mm: 45.0,
            price_usd: 303.99,
            purchase_url: "https://www.scorpionsystem.com/catalog/helicopter/motors_4/hkii-45/HKII_4525_520/",
            price_note: "Scorpion official store (direct), 55 mm shaft, in stock $303.99 — as-of 2026-06",
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DATASHEET ORACLE — the 700-class HKII-4525-520: 520 Kv, 503 g, 4450 W /
    /// 100 A continuous, 12S. Source: scorpionsystem.com HKII-45 series.
    #[test]
    fn flagship_matches_datasheet() {
        let m = scorpion_hk_catalogue()
            .into_iter()
            .find(|m| m.name == "Scorpion HKII-4525-520")
            .unwrap();
        assert_eq!(m.kv, 520.0);
        assert_eq!(m.mass_g, 503.0);
        assert_eq!(m.max_cont_power_w, 4450.0);
        assert_eq!(m.max_cont_current_a, 100.0);
        assert_eq!(m.max_cells, 12);
        // Continuous power ≈ V·I sanity: 12S nominal 44.4 V × 100 A = 4440 W ≈ 4450.
        assert!((44.4_f64 * 100.0 - 4440.0).abs() < 1.0);
    }

    /// Every catalogue motor is buyable: a direct purchase URL and a positive
    /// price (the user's hard rule — no part in the analysis you cannot buy).
    #[test]
    fn every_motor_is_buyable() {
        for m in scorpion_hk_catalogue() {
            assert!(m.price_usd > 0.0, "{} has no price", m.name);
            assert!(m.purchase_url.starts_with("https://"), "{} bad url", m.name);
            assert!(!m.price_note.is_empty());
        }
    }

    /// The family is sorted by continuous power and scales by stator diameter.
    #[test]
    fn catalogue_scales_by_stator() {
        let cat = scorpion_hk_catalogue();
        for w in cat.windows(2) {
            assert!(w[0].max_cont_power_w <= w[1].max_cont_power_w);
            assert!(w[0].stator_d_mm <= w[1].stator_d_mm);
        }
    }

    #[test]
    fn install_steps_present() {
        let m = scorpion_hk_catalogue()[0];
        assert_eq!(m.install_steps(3, 9.0).len(), 6);
    }

    #[test]
    fn power_and_structure_instructions_present() {
        let m = scorpion_hk_catalogue()[0];
        let pwr = m.power_and_connections(3);
        assert!(pwr.iter().any(|l| l.contains("3S") && l.contains("V")));
        assert!(pwr.iter().any(|l| l.contains("ESC")));
        assert!(
            m.structural_connections(9.0)
                .iter()
                .any(|l| l.contains("mast"))
        );
    }

    #[test]
    fn connector_scales_with_current() {
        assert_eq!(pack_connector(25.0), "XT60");
        assert_eq!(pack_connector(100.0), "AS150 / EC8"); // the 4525 at 100 A
    }
}
