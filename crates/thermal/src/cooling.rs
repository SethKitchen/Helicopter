//! The [`Cooling`] trait — the polymorphism boundary for heat rejection — and a
//! convective implementation.

/// A model for the heat rejected from a body to its surroundings.
pub trait Cooling {
    /// Heat removed (W) from a surface of area `area_m2` at `temp_c` into an
    /// ambient at `ambient_c`.
    fn heat_removed(&self, temp_c: f64, ambient_c: f64, area_m2: f64) -> f64;

    /// Short human-readable description.
    fn label(&self) -> String;
}

/// Newtonian convective cooling: `Q = h · A · (T − T_ambient)`.
#[derive(Clone, Copy, Debug)]
pub struct Convective {
    /// Convective heat-transfer coefficient, W/(m²·K).
    pub h: f64,
}

impl Convective {
    /// Explicit coefficient.
    pub fn new(h: f64) -> Self {
        Convective { h }
    }

    /// Natural convection of a small cylinder in still air, h ≈ 7.5 W/(m²·K).
    /// Free convection of an ~18 mm horizontal cylinder at a modest ΔT gives
    /// `Nu ≈ 4–6` (Churchill–Bernstein correlation), so with air `k ≈ 0.026 W/(m·K)`
    /// and `D = 0.018 m`, `h = Nu·k/D ≈ 6–9 W/(m²·K)` — the 7.5 used is central.
    /// Source: Incropera & DeWitt, *Fundamentals of Heat and Mass Transfer*
    /// (free convection from cylinders); the regime under which 18650 cells are
    /// datasheet-characterised.
    pub fn natural_air() -> Self {
        Convective { h: 7.5 }
    }

    /// Forced air over the cells, e.g. a cooling fan (~2 m/s), h ≈ 40 W/(m²·K).
    /// Cross-flow over an 18 mm cylinder at `Re ≈ 2000` gives `Nu ≈ 25–30`
    /// (Hilpert/Churchill correlation) → `h ≈ 35–45 W/(m²·K)`. Source: Incropera &
    /// DeWitt, forced convection over a cylinder in cross-flow.
    pub fn forced_air() -> Self {
        Convective { h: 40.0 }
    }

    /// Strong rotor-downwash cooling (~4–5 m/s), h ≈ 80 W/(m²·K) — higher-Re
    /// cross-flow; an upper estimate, to be confirmed by test if used in anger.
    pub fn rotor_downwash() -> Self {
        Convective { h: 80.0 }
    }
}

impl Cooling for Convective {
    fn heat_removed(&self, temp_c: f64, ambient_c: f64, area_m2: f64) -> f64 {
        self.h * area_m2 * (temp_c - ambient_c)
    }

    fn label(&self) -> String {
        format!("convective h={:.0} W/m²K", self.h)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_heat_removed_at_ambient() {
        let c = Convective::natural_air();
        assert!((c.heat_removed(25.0, 25.0, 0.004)).abs() < 1e-12);
    }

    #[test]
    fn heat_scales_with_delta_t_and_area() {
        let c = Convective::new(10.0);
        assert!((c.heat_removed(35.0, 25.0, 0.004) - 10.0 * 0.004 * 10.0).abs() < 1e-12);
    }

    /// DOCUMENTED — the convection coefficients sit in the textbook free/forced
    /// convection bands for an ~18 mm cylinder (Incropera & DeWitt): free
    /// convection h ≈ 5–10, forced (fan) h ≈ 30–60. See the constructor docs for
    /// the Nu·k/D derivation.
    #[test]
    fn coefficients_are_in_the_literature_convection_bands() {
        assert!((5.0..=10.0).contains(&Convective::natural_air().h));
        assert!((30.0..=60.0).contains(&Convective::forced_air().h));
    }

    #[test]
    fn rotor_downwash_is_strongest_and_label_reports_h() {
        let dw = Convective::rotor_downwash();
        assert!(dw.h > Convective::forced_air().h);
        // Stronger cooling removes more heat for the same ΔT and area.
        let q_dw = dw.heat_removed(40.0, 25.0, 0.004);
        let q_nat = Convective::natural_air().heat_removed(40.0, 25.0, 0.004);
        assert!(q_dw > q_nat);
        assert!(dw.label().contains("h=80"));
    }
}
