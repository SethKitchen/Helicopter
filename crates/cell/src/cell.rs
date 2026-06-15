//! The [`Cell`] trait — the polymorphism boundary for cell models.

/// Convecting skin area of an 18650 cylinder (18 mm × 65 mm): lateral + two end
/// caps ≈ 4.09e-3 m². The single shared value used by the trait default and any
/// 18650 cell, so geometry never drifts between crates.
pub const SURFACE_AREA_18650_M2: f64 = 4.09e-3;
/// Convecting skin area of a 21700 cylinder (21.1 mm × 70 mm): `π·D·L` + two end
/// caps ≈ 5.34e-3 m². Shared with `helisim_bms`'s thermal envelope.
pub const SURFACE_AREA_21700_M2: f64 = 5.34e-3;

/// A single battery cell modelled as an equivalent circuit. State of charge
/// `soc` is in `[0, 1]`; discharge current is positive.
pub trait Cell {
    /// Open-circuit (rested) terminal voltage at state of charge `soc`, volts.
    fn ocv(&self, soc: f64) -> f64;

    /// Internal series resistance at `soc`, ohms. By convention this is the
    /// reference (25 °C) value — temperature dependence is layered on by
    /// [`internal_resistance_at`](Self::internal_resistance_at).
    fn internal_resistance(&self, soc: f64) -> f64;

    /// Internal series resistance at `soc` and cell temperature `temp_c`, ohms.
    /// Default: the 25 °C value scaled by the Arrhenius factor
    /// [`resistance_temp_factor`](crate::temperature::resistance_temp_factor)
    /// (1.0 at 25 °C, rising in the cold). Override for a per-cell temperature fit.
    fn internal_resistance_at(&self, soc: f64, temp_c: f64) -> f64 {
        self.internal_resistance(soc) * crate::temperature::resistance_temp_factor(temp_c)
    }

    /// Usable charge capacity, amp-hours.
    fn capacity_ah(&self) -> f64;

    /// Nominal (label) voltage, volts.
    fn nominal_voltage(&self) -> f64;

    /// Discharge cut-off voltage, volts.
    fn cutoff_voltage(&self) -> f64;

    /// Maximum continuous discharge current, amps.
    fn max_continuous_current(&self) -> f64;

    /// Cell mass, kilograms.
    fn mass_kg(&self) -> f64;

    /// Specific heat capacity, J/(kg·K). Default: generic 18650 Li-ion (~900,
    /// the central value of the 800–1100 J/(kg·K) measured literature range).
    fn specific_heat(&self) -> f64 {
        900.0
    }

    /// External surface area available for convective cooling, m². Default: an
    /// 18650 cylinder ([`SURFACE_AREA_18650_M2`]); cells with other form factors
    /// (e.g. 21700) override this.
    fn surface_area(&self) -> f64 {
        SURFACE_AREA_18650_M2
    }

    /// Lumped heat capacity `m · c_p`, J/K.
    fn heat_capacity(&self) -> f64 {
        self.mass_kg() * self.specific_heat()
    }

    /// Entropic coefficient `∂OCV/∂T` at `soc`, V/K — the driver of reversible
    /// (entropic) heat. **Default 0** ⇒ the thermal model is Joule-only (`I²R`);
    /// supply a *measured* coefficient to include reversible heat. It is chemistry-
    /// and SoC-specific (NMC: a few ×0.1 mV/K, sign-changing with SoC), so a value
    /// is only baked in where it can be sourced — never fabricated.
    fn entropic_coefficient(&self, _soc: f64) -> f64 {
        0.0
    }

    /// Reversible (entropic) heat at `soc` under discharge current `i` (A, positive)
    /// and cell temperature `temp_c`: Bernardi `Q_rev = −I·T·(∂OCV/∂T)`, watts
    /// (positive = heating). Zero unless an entropic coefficient is supplied.
    fn reversible_heat(&self, soc: f64, i: f64, temp_c: f64) -> f64 {
        -i * (temp_c + 273.15) * self.entropic_coefficient(soc)
    }

    /// Terminal voltage under a load `current` (A, discharge positive) at `soc`.
    /// First-order equivalent circuit: `V = OCV(soc) - I * R(soc)`.
    fn terminal_voltage(&self, soc: f64, current: f64) -> f64 {
        self.ocv(soc) - current * self.internal_resistance(soc)
    }

    /// Maximum power the cell can deliver into a matched load at `soc`,
    /// `OCV^2 / (4 R)` (watts). Beyond this no current solution exists.
    fn max_power(&self, soc: f64) -> f64 {
        let v = self.ocv(soc);
        v * v / (4.0 * self.internal_resistance(soc))
    }

    /// Continuous discharge rating expressed as a C-rate (1/h).
    fn max_continuous_c_rate(&self) -> f64 {
        self.max_continuous_current() / self.capacity_ah()
    }
}
