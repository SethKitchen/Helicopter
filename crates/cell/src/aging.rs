//! Capacity-fade (aging) model — the cost of every cycle and every year, so the
//! fast-charge-vs-longevity trade has numbers behind it.
//!
//! ## Model form (literature-standard, cited)
//! * **Cycle fade** — a Wang-style throughput power law: `fade_cyc = A · (C/C_ref)^p
//!   · Q10^((T−25)/10) · EFC^z`, where EFC is equivalent full cycles and `z≈0.55`
//!   (Wang et al., "Cycle-life model for graphite-LiFePO₄ cells", J. Power Sources
//!   196 (2011); extended to NMC/NCA by Wang et al. 2014). Higher C-rate and higher
//!   temperature both accelerate it.
//! * **Calendar fade** — diffusion-limited SEI growth: `fade_cal = A_cal ·
//!   Q10^((T−25)/10) · √t` (Broussely 2005; Schmalstieg et al., "A holistic aging
//!   model for Li(NiMnCo)O₂", J. Power Sources 257 (2014)).
//! * **Q10 = 2** — the standard rule that aging rate roughly doubles per +10 °C.
//! * **End of life = 80 %** retention (20 % fade), the automotive convention.
//!
//! ## Honesty (provenance)
//! Precise aging curves for these specific 21700 cells are NOT public, so the
//! COEFFICIENTS are representative and **overridable**, calibrated to two points:
//! the **BAK 45D datasheet** (≥600 cycles @ 30 A = 6.7C to 60 %, a real sourced
//! anchor) and a literature-typical **~1500 cycles @ 1C to 80 %** (representative,
//! flagged). The FINDING is the trade-off shape and the sweet spot — NOT an
//! absolute cell-specific cycle count. (Web sourcing of fresh curves was
//! unavailable at build time; the model forms above are the standard ones.)

/// A parametric capacity-fade model. Defaults are calibrated as described above.
#[derive(Clone, Copy, Debug)]
pub struct DegradationModel {
    /// Cycle pre-factor `A`: fade per EFC^z at 1C, 25 °C.
    pub cycle_coeff: f64,
    /// Throughput exponent `z` (Wang ≈ 0.55).
    pub throughput_exp: f64,
    /// C-rate exponent `p` (calibrated to the BAK anchor).
    pub c_rate_exp: f64,
    /// Reference C-rate (1.0).
    pub ref_c_rate: f64,
    /// Calendar pre-factor: fade per √year at 25 °C, mid SoC (~2.5 %/yr).
    pub calendar_coeff: f64,
    /// Temperature acceleration per 10 °C (Q10 = 2.0).
    pub q10: f64,
    /// Reference temperature, °C.
    pub ref_temp_c: f64,
    /// End-of-life fade fraction (0.20 = 80 % retention).
    pub eol_fade: f64,
    /// Depth-of-discharge stress exponent: a cycle of depth `d` costs `d^dod_exp`
    /// of a full cycle, so shallow cycling is gentler than its Ah-throughput alone
    /// (the Wöhler/rainflow DoD exponent ≈ 1.5–2; Xu et al., "Modeling of
    /// Lithium-Ion Battery Degradation for Cell Life Assessment", IEEE Trans. Smart
    /// Grid 2018). 1.5 here is the conservative end. Representative/overridable.
    pub dod_exp: f64,
}

impl Default for DegradationModel {
    fn default() -> Self {
        DegradationModel {
            cycle_coeff: 0.003578,
            throughput_exp: 0.55,
            c_rate_exp: 0.63,
            ref_c_rate: 1.0,
            calendar_coeff: 0.025,
            q10: 2.0,
            ref_temp_c: 25.0,
            eol_fade: 0.20,
            dod_exp: 1.5,
        }
    }
}

impl DegradationModel {
    /// Temperature acceleration factor `Q10^((T−Tref)/10)` (1.0 at Tref).
    pub fn temp_factor(&self, temp_c: f64) -> f64 {
        self.q10.powf((temp_c - self.ref_temp_c) / 10.0)
    }

    /// C-rate stress factor `(C/C_ref)^p` (1.0 at the reference C-rate).
    pub fn c_rate_factor(&self, c_rate: f64) -> f64 {
        (c_rate / self.ref_c_rate).powf(self.c_rate_exp)
    }

    /// Cycle capacity fade (fraction) after `efc` equivalent full cycles at an
    /// effective `c_rate` and operating temperature `temp_c`.
    pub fn cycle_fade(&self, efc: f64, c_rate: f64, temp_c: f64) -> f64 {
        self.cycle_coeff
            * self.c_rate_factor(c_rate)
            * self.temp_factor(temp_c)
            * efc.powf(self.throughput_exp)
    }

    /// Calendar capacity fade (fraction) after `years` at storage temperature
    /// `temp_c`. `soc_factor` (≈1.0 at mid SoC, higher near full) scales it.
    pub fn calendar_fade(&self, years: f64, temp_c: f64, soc_factor: f64) -> f64 {
        self.calendar_coeff * self.temp_factor(temp_c) * soc_factor * years.sqrt()
    }

    /// Cycle life to end-of-life (cycles to 80 %) at `c_rate`, `temp_c`, ignoring
    /// calendar fade — `N = (eol / (A · crf · tf))^(1/z)`.
    pub fn cycle_life_to_eol(&self, c_rate: f64, temp_c: f64) -> f64 {
        let per_cycle = self.cycle_coeff * self.c_rate_factor(c_rate) * self.temp_factor(temp_c);
        (self.eol_fade / per_cycle).powf(1.0 / self.throughput_exp)
    }

    /// Total fade after a usage: `cycle_fade(EFC, …) + calendar_fade(…)`.
    pub fn total_fade(&self, efc: f64, c_rate: f64, op_temp_c: f64, cal: CalendarLoad) -> f64 {
        self.cycle_fade(efc, c_rate, op_temp_c)
            + self.calendar_fade(cal.years, cal.storage_temp_c, cal.soc_factor)
    }

    /// Does the pack still retain ≥ (1 − eol) after this usage?
    pub fn meets_life(&self, efc: f64, c_rate: f64, op_temp_c: f64, cal: CalendarLoad) -> bool {
        self.total_fade(efc, c_rate, op_temp_c, cal) <= self.eol_fade
    }
}

/// The calendar-aging inputs: how long, at what storage temperature, and a
/// state-of-charge factor (≈1.0 at mid SoC, higher near full).
#[derive(Clone, Copy, Debug)]
pub struct CalendarLoad {
    pub years: f64,
    pub storage_temp_c: f64,
    pub soc_factor: f64,
}

/// Equivalent full cycles from a usage pattern: `flights_per_year · years · dod`
/// (one flight ≈ one discharge of depth `dod`).
pub fn equivalent_full_cycles(flights_per_year: f64, years: f64, dod: f64) -> f64 {
    flights_per_year * years * dod
}

impl DegradationModel {
    /// DoD-stressed effective full cycles: `n_cycles · dod^dod_exp`. Equals
    /// `n_cycles` at full depth (dod=1), and is LESS than the Ah-throughput
    /// (`n·dod`) for shallow cycles — the extra gentleness of partial cycling.
    pub fn effective_efc(&self, n_cycles: f64, dod: f64) -> f64 {
        n_cycles * dod.powf(self.dod_exp)
    }

    /// Total fade for a usage given the raw flight count and per-flight depth.
    pub fn fade_over_life(
        &self,
        n_cycles: f64,
        dod: f64,
        c_rate: f64,
        op_temp_c: f64,
        cal: CalendarLoad,
    ) -> f64 {
        self.cycle_fade(self.effective_efc(n_cycles, dod), c_rate, op_temp_c)
            + self.calendar_fade(cal.years, cal.storage_temp_c, cal.soc_factor)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// ANCHOR 1 (real datasheet) — BAK 45D: ≥600 cycles @ 6.7C to **60 %**
    /// (0.40 fade). NOTE this is a *60 %* EOL; `cycle_life_to_eol` uses the *80 %*
    /// EOL, which is reached far sooner at this harsh rate (≈170 cycles) — both
    /// from the same per-cycle rate, a consistency check, not a contradiction.
    #[test]
    fn bak_high_rate_anchor() {
        let m = DegradationModel::default();
        let fade = m.cycle_fade(600.0, 6.7, 25.0);
        assert!(
            (fade - 0.40).abs() < 0.02,
            "BAK 600cyc@6.7C fade {fade} (want ~0.40)"
        );
        // The 80 %-EOL life at 6.7C is much shorter than the 60 % point (170 < 600)
        // and far shorter than the 1C life — fast cycling is punishing.
        let life_67 = m.cycle_life_to_eol(6.7, 25.0);
        assert!(life_67 < 600.0 && life_67 < m.cycle_life_to_eol(1.0, 25.0));
        // Self-consistency: scaling that 80 %-life by the throughput law back to
        // 0.40 fade reproduces the 600-cycle datasheet point.
        assert!(
            (m.cycle_fade(600.0, 6.7, 25.0) / m.cycle_fade(life_67, 6.7, 25.0) - 2.0).abs() < 0.05
        );
    }

    /// ANCHOR 2 (representative) — ~1500 cycles @ 1C to 80 % (0.20 fade).
    #[test]
    fn one_c_reference_anchor() {
        let m = DegradationModel::default();
        assert!(
            (m.cycle_fade(1500.0, 1.0, 25.0) - 0.20).abs() < 0.01,
            "1C 1500cyc fade"
        );
        assert!((m.cycle_life_to_eol(1.0, 25.0) - 1500.0).abs() < 40.0);
    }

    /// Higher C-rate and higher temperature both shorten cycle life.
    #[test]
    fn faster_and_hotter_age_more() {
        let m = DegradationModel::default();
        assert!(m.cycle_life_to_eol(3.0, 25.0) < m.cycle_life_to_eol(1.0, 25.0));
        assert!(m.cycle_life_to_eol(1.0, 40.0) < m.cycle_life_to_eol(1.0, 25.0));
        // Q10 = 2: +10 °C doubles the rate (halves the life-at-fixed-fade per EFC).
        assert!((m.temp_factor(35.0) - 2.0).abs() < 1e-9);
        assert!((m.temp_factor(25.0) - 1.0).abs() < 1e-12);
    }

    /// Calendar fade grows as √t and is ~2.5 % at 1 yr / 25 °C / mid-SoC.
    #[test]
    fn calendar_is_sqrt_time() {
        let m = DegradationModel::default();
        assert!((m.calendar_fade(1.0, 25.0, 1.0) - 0.025).abs() < 1e-9);
        // 4 years → ×2 (√4); 10 years → ~7.9 %.
        assert!((m.calendar_fade(4.0, 25.0, 1.0) - 0.05).abs() < 1e-9);
        assert!((m.calendar_fade(10.0, 25.0, 1.0) - 0.0791).abs() < 1e-3);
    }

    #[test]
    fn efc_from_usage() {
        assert!((equivalent_full_cycles(52.0, 10.0, 1.0) - 520.0).abs() < 1e-9);
    }

    /// Shallow cycling is gentler than its Ah-throughput: a 50%-DoD cycle costs
    /// less than half a full cycle, and full depth is unchanged (anchors hold).
    #[test]
    fn shallow_dod_is_gentler_than_throughput() {
        let m = DegradationModel::default();
        assert!((m.effective_efc(100.0, 1.0) - 100.0).abs() < 1e-9); // full depth unchanged
        let half = m.effective_efc(100.0, 0.5);
        assert!(
            half < 50.0,
            "50% DoD eff EFC {half} should be < 50 (throughput)"
        );
        // Same flight count at lower depth → less total fade.
        let cal = CalendarLoad { years: 5.0, storage_temp_c: 25.0, soc_factor: 1.0 };
        let full = m.fade_over_life(1000.0, 1.0, 2.0, 25.0, cal);
        let shallow = m.fade_over_life(1000.0, 0.5, 2.0, 25.0, cal);
        assert!(shallow < full);
    }
}
