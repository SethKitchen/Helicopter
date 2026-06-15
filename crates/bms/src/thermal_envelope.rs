//! Emergent continuous-current limit — true continuous as a thermal *output*.
//!
//! In `library.rs` the true continuous rating is a **sourced input** (Battery
//! Mooch's measured de-rate). Here it is **computed**: the largest steady draw
//! whose cell temperature stays under the cutoff, from the cell's own `R`
//! (Joule heat `I²R`, temperature-dependent), the [`TwoNodeThermalCell`] core/
//! surface model, and the cooling. This is the coupling the project's thermal
//! track set up — and it is where the tabless story pays off: a lower-`R` cell
//! generates less heat, so its emergent continuous current is *higher*, with no
//! number hand-entered.
//!
//! Two limits are offered because they answer different questions:
//! * **steady-state** — the asymptotic draw a cell could hold *forever*; very
//!   conservative for a cell that empties before it thermally saturates.
//! * **full-discharge (transient)** — the largest draw whose *peak* temperature
//!   over a complete discharge stays under the cutoff. This is the physically
//!   meaningful "can it sustain this to empty", and it is higher than steady-state
//!   because a 4–5 Ah cell at tens of amps empties in minutes, before it reaches
//!   thermal equilibrium. The gap between the two is itself the finding: a
//!   steady-state model alone under-rates these cells.

use helisim_cell::Cell;
use helisim_thermal::{Cooling, TwoNodeThermalCell};

/// Upper clamp (°C) on the temperature used to evaluate the Arrhenius `R(T)`. The
/// resistance model is only valid across the normal operating band; without this
/// clamp the temp→R→heat→temp fixed point can chase a runaway to thousands of °C,
/// where the (extrapolated) Arrhenius factor collapses `R` and falsely reports a
/// cool cell. Clamping keeps the root-finder physical.
const R_EVAL_MAX_C: f64 = 80.0;

/// Geometry/cutoff context for an emergent-limit calculation.
#[derive(Clone, Copy, Debug)]
pub struct ThermalEnvelope {
    /// Ambient temperature, °C.
    pub ambient_c: f64,
    /// Surface-temperature cutoff, °C (datasheet cells cut off at 80 °C skin).
    pub t_limit_c: f64,
    /// Cell length, m — sets the core→surface resistance `R_int` (21700 ≈ 0.070).
    pub length_m: f64,
    /// External convecting surface area, m² (21700 ≈ 5.34e-3; the [`Cell`] trait's
    /// own default is an 18650, so geometry is set here, not read from the cell).
    pub surface_area_m2: f64,
    /// Fraction of thermal mass in the core (jelly roll ≈ 0.9).
    pub core_fraction: f64,
}

impl ThermalEnvelope {
    /// A 21700 (0.070 m long, ≈5.34e-3 m² skin: π·D·L + two end caps, D=21.1 mm)
    /// cutting off at `t_limit_c`.
    pub fn for_21700(ambient_c: f64, t_limit_c: f64) -> Self {
        ThermalEnvelope {
            ambient_c,
            t_limit_c,
            length_m: 0.070,
            surface_area_m2: helisim_cell::SURFACE_AREA_21700_M2,
            core_fraction: 0.9,
        }
    }

    fn two_node(&self, cell: &dyn Cell) -> TwoNodeThermalCell {
        TwoNodeThermalCell::from_geometry(
            cell.heat_capacity(),
            self.core_fraction,
            self.length_m,
            self.surface_area_m2,
            self.ambient_c,
        )
    }

    /// `R` at `soc=0.5` and temperature `t`, with the evaluation temperature
    /// clamped to the model's valid band (see [`R_EVAL_MAX_C`]).
    fn r_at(&self, cell: &dyn Cell, t: f64) -> f64 {
        cell.internal_resistance_at(0.5, t.clamp(-20.0, R_EVAL_MAX_C))
    }

    /// Steady-state surface temperature for a constant discharge at `current_a`,
    /// with the temperature-dependent `R` resolved as a fixed point (warmer cell →
    /// lower `R` → less heat).
    pub fn steady_surface_temp(
        &self,
        cell: &dyn Cell,
        cooling: &dyn Cooling,
        current_a: f64,
    ) -> f64 {
        let two = self.two_node(cell);
        let mut t = self.ambient_c;
        for _ in 0..40 {
            let r = self.r_at(cell, t);
            let q = current_a * current_a * r;
            let (_core, surf) = two.steady_state(q, cooling);
            t = surf;
        }
        t
    }

    /// Peak `(core, surface)` temperatures reached during a *complete* discharge at
    /// `current_a` (duration = capacity / current), integrating the two-node model
    /// from ambient with live, temperature-dependent heat generation.
    ///
    /// The two peaks diverge at high rate: a 4–5 Ah cell at tens of amps empties in
    /// minutes, and the **surface lags the core** through `R_int`, so on a fast
    /// discharge the skin barely warms while the core soars. The core peak is the
    /// safety-relevant one; the surface peak is what a skin thermocouple reads.
    pub fn peak_temps(&self, cell: &dyn Cell, cooling: &dyn Cooling, current_a: f64) -> (f64, f64) {
        let two = self.two_node(cell);
        let duration_s = cell.capacity_ah() / current_a * 3600.0;
        let dt = 0.5;
        let steps = (duration_s / dt).ceil() as usize;
        let (mut tc, mut ts) = (self.ambient_c, self.ambient_c);
        let (mut peak_c, mut peak_s) = (tc, ts);
        for _ in 0..steps {
            let r = self.r_at(cell, tc);
            let q = current_a * current_a * r;
            let (a, b) = two.step(tc, ts, q, cooling, dt);
            tc = a;
            ts = b;
            if tc > peak_c {
                peak_c = tc;
            }
            if ts > peak_s {
                peak_s = ts;
            }
        }
        (peak_c, peak_s)
    }

    /// Peak surface temperature over a full discharge (skin-thermocouple view).
    pub fn peak_surface_temp(&self, cell: &dyn Cell, cooling: &dyn Cooling, current_a: f64) -> f64 {
        self.peak_temps(cell, cooling, current_a).1
    }

    /// Peak core temperature over a full discharge (the safety-relevant node).
    pub fn peak_core_temp(&self, cell: &dyn Cell, cooling: &dyn Cooling, current_a: f64) -> f64 {
        self.peak_temps(cell, cooling, current_a).0
    }

    /// Largest steady draw whose steady-state surface temperature stays at the
    /// cutoff — bisection on the (monotone) temperature-vs-current curve.
    pub fn steady_continuous(&self, cell: &dyn Cell, cooling: &dyn Cooling) -> f64 {
        self.bisect(|i| self.steady_surface_temp(cell, cooling, i))
    }

    /// Largest draw whose peak *surface* temperature over a full discharge stays at
    /// the cutoff. NOTE: on fast discharges the skin lags the core, so this is
    /// lenient — use [`discharge_continuous_core`](Self::discharge_continuous_core)
    /// for the safety-relevant limit.
    pub fn discharge_continuous(&self, cell: &dyn Cell, cooling: &dyn Cooling) -> f64 {
        self.bisect(|i| self.peak_surface_temp(cell, cooling, i))
    }

    /// Largest draw whose peak *core* temperature over a full discharge stays at
    /// the cutoff — the safety-relevant continuous rating (the core is what runs
    /// away). This is the one that tracks measured continuous ratings at high rate.
    pub fn discharge_continuous_core(&self, cell: &dyn Cell, cooling: &dyn Cooling) -> f64 {
        self.bisect(|i| self.peak_core_temp(cell, cooling, i))
    }

    /// Bisect for the current at which `temp_of_current(I) == t_limit`. Monotone
    /// increasing, so robust and derivative-free (the project's shape-1 solver).
    fn bisect(&self, temp_of_current: impl Fn(f64) -> f64) -> f64 {
        let (mut lo, mut hi) = (0.1_f64, 300.0_f64);
        // If even the top of the bracket stays cool, the limit is cooling-bounded
        // beyond any sane rating — report the bracket top.
        if temp_of_current(hi) <= self.t_limit_c {
            return hi;
        }
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if temp_of_current(mid) < self.t_limit_c {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        0.5 * (lo + hi)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use helisim_cell::{TheveninCell, ampace_jp40, molicel_p50b};
    use helisim_thermal::Convective;

    fn ocv() -> Vec<(f64, f64)> {
        helisim_cell::library::representative_nmc_ocv()
    }

    /// At the returned steady current the steady-state surface temperature really
    /// is the cutoff (self-consistent root).
    #[test]
    fn steady_continuous_sits_at_the_limit() {
        let env = ThermalEnvelope::for_21700(25.0, 80.0);
        let cell = ampace_jp40();
        let cooling = Convective::forced_air();
        let i = env.steady_continuous(&cell, &cooling);
        let t = env.steady_surface_temp(&cell, &cooling, i);
        assert!((t - 80.0).abs() < 0.5, "i {i} t {t}");
    }

    /// The transient (full-discharge) rating exceeds the steady-state one: the
    /// cell empties before it thermally saturates, so a steady-state model
    /// under-rates it. This gap is the documented finding.
    #[test]
    fn transient_rating_exceeds_steady() {
        let env = ThermalEnvelope::for_21700(25.0, 80.0);
        let cell = ampace_jp40();
        let cooling = Convective::natural_air();
        let steady = env.steady_continuous(&cell, &cooling);
        let transient = env.discharge_continuous(&cell, &cooling);
        assert!(transient > steady, "transient {transient} steady {steady}");
    }

    /// Lower internal resistance → higher emergent continuous current (the tabless
    /// advantage flowing through, with no number hand-entered). Two cells identical
    /// but for `R`.
    #[test]
    fn lower_resistance_raises_emergent_continuous() {
        let env = ThermalEnvelope::for_21700(25.0, 80.0);
        let cooling = Convective::forced_air();
        // 4 Ah cells differing only in R: 10 mΩ (tabbed-like) vs 5 mΩ (tabless).
        let tabbed = TheveninCell::new(&ocv(), 0.010, 4.0, 3.6, 2.5, 60.0, 0.070);
        let tabless = TheveninCell::new(&ocv(), 0.005, 4.0, 3.6, 2.5, 60.0, 0.070);
        let i_tabbed = env.steady_continuous(&tabbed, &cooling);
        let i_tabless = env.steady_continuous(&tabless, &cooling);
        assert!(
            i_tabless > i_tabbed,
            "tabless {i_tabless} tabbed {i_tabbed}"
        );
    }

    /// The two-node core runs hotter than the surface during a hard discharge, so
    /// a core limit binds sooner than a surface limit — the gradient the single
    /// node could not see.
    #[test]
    fn core_is_hotter_than_surface_under_load() {
        let env = ThermalEnvelope::for_21700(25.0, 80.0);
        let cell = molicel_p50b();
        let cooling = Convective::natural_air();
        let two = env.two_node(&cell);
        let q = 40.0 * 40.0 * cell.internal_resistance(0.5);
        let (core, surf) = two.steady_state(q, &cooling);
        assert!(core > surf + 1.0, "core {core} surf {surf}");
    }
}
