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
    /// This is the "free convection" condition under which 18650 cells are
    /// thermally characterised.
    pub fn natural_air() -> Self {
        Convective { h: 7.5 }
    }

    /// Forced air over the cells, e.g. a cooling fan, h ≈ 40 W/(m²·K).
    pub fn forced_air() -> Self {
        Convective { h: 40.0 }
    }

    /// Strong rotor-downwash cooling, h ≈ 80 W/(m²·K).
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
}
