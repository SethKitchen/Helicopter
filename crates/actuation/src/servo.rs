//! Control-servo catalogue — the Align HV digital servo family.
//!
//! The swashplate (cyclic/collective) and tail-pitch are driven by digital
//! servos; one family spans mini (DS450/DS455) → standard HV (DS820/DS825). Each
//! entry is a **real, currently-buyable part**: datasheet figures are the spec
//! oracle, and every row carries a **direct purchase URL + price** (sourced,
//! as-of 2026-06). A servo is selected on **stall torque** against the control
//! load from [`crate::loads`].

use crate::selectable::Selectable;

/// Approximate EUR→USD used to put EU-shop prices in the USD BOM. A stated
/// assumption (FX moves), as-of 2026-06; the native price + retailer is recorded
/// in each part's `price_note` so the figure is auditable.
pub const EUR_USD_2026_06: f64 = 1.08;

/// What a servo drives — same rating basis, different count (a CCPM swashplate
/// needs several cyclic servos; the tail needs one).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ServoRole {
    /// Swashplate / collective-cyclic (CCPM) servo.
    Cyclic,
    /// Tail-rotor pitch servo.
    Tail,
}

/// One digital control servo (a buyable catalogue part).
#[derive(Clone, Copy, Debug)]
pub struct Servo {
    /// Align designation, e.g. "Align DS820".
    pub name: &'static str,
    /// Stall torque, N·m (the sizing rating), at the rated HV.
    pub stall_torque_nm: f64,
    /// Mass, grams.
    pub mass_g: f64,
    /// Rated voltage, V (HV digital).
    pub voltage_v: f64,
    /// Transit speed, seconds per 60° at the rated voltage (responsiveness).
    pub speed_s_per_60: f64,
    /// Intended role (cyclic vs tail) — informational; either can be selected.
    pub role: ServoRole,
    /// Street price, USD (sourced; EU prices converted at [`EUR_USD_2026_06`]).
    pub price_usd: f64,
    /// Direct purchase URL.
    pub purchase_url: &'static str,
    /// Provenance of the price: retailer, native price, stock, as-of date.
    pub price_note: &'static str,
}

impl Selectable for Servo {
    fn name(&self) -> &str {
        self.name
    }
    fn mass_g(&self) -> f64 {
        self.mass_g
    }
    /// Governing capacity = stall torque.
    fn rating(&self) -> f64 {
        self.stall_torque_nm
    }
    fn rating_unit(&self) -> &'static str {
        "N·m"
    }
}

/// kg·cm → N·m (1 kgf·cm = 9.80665e-2 N·m). Servo torque is quoted in kg·cm.
pub fn kgcm_to_nm(kgcm: f64) -> f64 {
    kgcm * 9.80665e-2
}

impl Servo {
    /// How to install and use this servo on the swashplate / tail — the
    /// construct-and-use steps for the build output (standard CCPM setup).
    pub fn install_steps(&self, role: ServoRole) -> Vec<String> {
        let mount = match role {
            ServoRole::Cyclic => {
                "1. Mount the three cyclic servos at 120° (CCPM) on the frame swashplate deck."
            }
            ServoRole::Tail => {
                "1. Mount the tail servo on the boom, output toward the pitch slider."
            }
        };
        let (neutral, travel) = match role {
            ServoRole::Cyclic => (
                "3. Fit the servo arm perpendicular to the pushrod at neutral; set pushrod length so the swashplate is level (0° collective) at mid-stick.",
                "4. Set channel travel/endpoints to the collective+cyclic pitch range (≈ ±12°) with no binding or buzzing at the extremes.",
            ),
            ServoRole::Tail => (
                "3. Fit the servo arm perpendicular to the tail pushrod at neutral; set length so the tail pitch slider is centred (zero yaw trim) at stick centre.",
                "4. Set travel to the tail-pitch range for full anti-torque authority both ways, without the slider binding at the extremes.",
            ),
        };
        vec![
            mount.to_string(),
            format!(
                "2. Power the servo at its rated {:.1} V (HV BEC); centre it electronically (sub-trim) BEFORE fitting the arm.",
                self.voltage_v
            ),
            neutral.to_string(),
            travel.to_string(),
            format!(
                "5. Confirm direction (reverse the channel if wrong); the digital servo holds to {:.2} N·m stall at {:.3} s/60°.",
                self.stall_torque_nm, self.speed_s_per_60
            ),
            "6. Range-test and check for jitter under load before flight.".to_string(),
        ]
    }
}

impl Servo {
    /// What power and connections the servo needs — supply voltage, BEC, signal.
    pub fn power_and_connections(&self, role: ServoRole) -> Vec<String> {
        let signal = match role {
            ServoRole::Cyclic => {
                "Signal: from the flight-controller swashplate mixer (digital, up to ~333 Hz update rate)."
            }
            ServoRole::Tail => {
                "Signal: from the gyro / AFCS tail (rudder) channel — keep it on the gyro's high update rate."
            }
        };
        vec![
            format!(
                "Power: {:.1} V (HV) from an HV BEC — NOT a 5 V receiver logic rail; budget ~1–2 A peak \
                 under load per digital servo.",
                self.voltage_v
            ),
            "Connector: 3-wire JR/Futaba lead (signal / +V / ground).".to_string(),
            signal.to_string(),
        ]
    }

    /// How the servo connects to the helicopter structure.
    pub fn structural_connections(&self, role: ServoRole) -> Vec<String> {
        match role {
            ServoRole::Cyclic => vec![
                "Bolts to the frame servo-deck around the mast (three at 120° for CCPM)."
                    .to_string(),
                "Ball-link pushrod from the servo arm to the swashplate's non-rotating outer ring."
                    .to_string(),
                "Keep the three cyclic linkages equal length so the swashplate stays level."
                    .to_string(),
            ],
            ServoRole::Tail => vec![
                "Bolts to the tail boom (or boom-end servo mount).".to_string(),
                "Pushrod runs along the boom to the tail-rotor pitch slider; add a guide to stop it flexing."
                    .to_string(),
            ],
        }
    }
}

/// The Align HV digital servo catalogue — **price-verified buyable parts**.
///
/// DATASHEET ORACLE (Align spec sheets; torque at the HV rating, via
/// [`kgcm_to_nm`]) + DIRECT PURCHASE (price/link sourced, as-of 2026-06):
/// * **DS455** mini tail: 2.9 kg·cm = 0.284 N·m, 17.3 g — €37.39 Lindinger.
/// * **DS450** mini cyclic: 4.0 kg·cm = 0.392 N·m, 17.5 g — $33.99 RotorQuest.
/// * **DS825** std tail: 12.5 kg·cm = 1.226 N·m, 62 g — $134.99 HeliDirect.
/// * **DS820** std cyclic: 23 kg·cm = 2.256 N·m, 70 g — €49.16 FLASH RC (in stock).
///
/// Sorted by stall torque. Overridable via [`crate::select_actuation_with`].
pub fn align_hv_catalogue() -> Vec<Servo> {
    vec![
        Servo {
            name: "Align DS455",
            stall_torque_nm: kgcm_to_nm(2.9),
            mass_g: 17.3,
            voltage_v: 8.4,
            speed_s_per_60: 0.04,
            role: ServoRole::Tail,
            price_usd: 37.39 * EUR_USD_2026_06,
            purchase_url: "https://www.lindinger.at/en/RC-ELECTRONICS/Servos-Accessories/Servos-Digital-High-Voltage/ALIGN-DS-455-DIGITAL-SERVO/9737794",
            price_note: "Lindinger €37.39 (special order 7–21 d), 2026-06; €→$ @1.08",
        },
        Servo {
            name: "Align DS450",
            stall_torque_nm: kgcm_to_nm(4.0),
            mass_g: 17.5,
            voltage_v: 8.4,
            speed_s_per_60: 0.06,
            role: ServoRole::Cyclic,
            price_usd: 33.99,
            purchase_url: "https://rotorquest.com/align-ds450-digital-servo-hsd45002-108621-html",
            price_note: "RotorQuest (US) $33.99, in stock — as-of 2026-06",
        },
        Servo {
            name: "Align DS825",
            stall_torque_nm: kgcm_to_nm(12.5),
            mass_g: 62.0,
            voltage_v: 8.4,
            speed_s_per_60: 0.02,
            role: ServoRole::Tail,
            price_usd: 134.99,
            purchase_url: "https://www.helidirect.com/products/align-ds825-high-voltage-brushless-servo",
            price_note: "HeliDirect (US) $134.99, in stock — as-of 2026-06",
        },
        Servo {
            name: "Align DS820",
            stall_torque_nm: kgcm_to_nm(23.0),
            mass_g: 70.0,
            voltage_v: 8.4,
            speed_s_per_60: 0.055,
            role: ServoRole::Cyclic,
            price_usd: 49.16 * EUR_USD_2026_06,
            purchase_url: "https://flashrc.com/align/24745-servo_numerique_align_ds820_brushless_hv_hsd82002_70g_23kgcm_0055s_60.html",
            price_note: "FLASH RC €49.16, in stock — as-of 2026-06; €→$ @1.08",
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// DATASHEET ORACLE — DS820 cyclic servo: 23 kg·cm @8.4 V → 2.256 N·m, 70 g,
    /// 0.055 s/60°. Source: Align DS820 (HSD82002) spec sheet.
    #[test]
    fn cyclic_flagship_matches_datasheet() {
        let s = align_hv_catalogue()
            .into_iter()
            .find(|s| s.name == "Align DS820")
            .unwrap();
        assert!((s.stall_torque_nm - 2.2555).abs() < 1e-3); // 23 × 0.0980665
        assert_eq!(s.mass_g, 70.0);
        assert!((s.speed_s_per_60 - 0.055).abs() < 1e-9);
        assert_eq!(s.role, ServoRole::Cyclic);
    }

    /// The kg·cm → N·m conversion the datasheets are entered through.
    #[test]
    fn torque_conversion_is_standard_gravity() {
        // 10 kg·cm = 0.980665 N·m.
        assert!((kgcm_to_nm(10.0) - 0.980665).abs() < 1e-9);
    }

    /// Every catalogue servo is buyable: direct purchase URL + positive price.
    #[test]
    fn every_servo_is_buyable() {
        for s in align_hv_catalogue() {
            assert!(s.price_usd > 0.0, "{} has no price", s.name);
            assert!(s.purchase_url.starts_with("https://"), "{} bad url", s.name);
            assert!(!s.price_note.is_empty());
        }
    }

    #[test]
    fn catalogue_sorted_by_torque() {
        let cat = align_hv_catalogue();
        for w in cat.windows(2) {
            assert!(w[0].stall_torque_nm <= w[1].stall_torque_nm);
        }
    }

    #[test]
    fn install_steps_present() {
        let s = align_hv_catalogue()[0];
        assert_eq!(s.install_steps(ServoRole::Cyclic).len(), 6);
    }

    #[test]
    fn power_and_structure_instructions_present() {
        let s = align_hv_catalogue()[1]; // DS450 cyclic
        let pwr = s.power_and_connections(ServoRole::Cyclic);
        assert!(pwr.iter().any(|l| l.contains("HV BEC")));
        assert!(pwr.iter().any(|l| l.contains("8.4 V")));
        // Structural connection differs by role (swashplate vs boom).
        assert!(
            s.structural_connections(ServoRole::Cyclic)
                .iter()
                .any(|l| l.contains("swashplate"))
        );
        assert!(
            s.structural_connections(ServoRole::Tail)
                .iter()
                .any(|l| l.contains("boom"))
        );
    }
}
