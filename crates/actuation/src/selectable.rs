//! The [`Selectable`] trait — the polymorphism boundary for catalogue selection.
//!
//! Both motors and servos are "the smallest standard part whose rating meets the
//! demand". That rule is identical to [`helisim_manufacture`]'s bolt/bearing
//! selection; here it is generalised once over a trait so a single
//! [`select_smallest_adequate`] serves every actuator family. A part exposes a
//! single scalar *rating* in whichever unit governs it (continuous power for a
//! motor, stall torque for a servo) plus its mass — the cost of carrying it.

/// A catalogue part that can be chosen by a scalar capacity against a demand.
pub trait Selectable: Clone {
    /// Catalogue designation, e.g. "HKII-4525-520" or "DS820".
    fn name(&self) -> &str;
    /// Installed mass, grams — the penalty for over-sizing (lighter is preferred).
    fn mass_g(&self) -> f64;
    /// The governing capacity, in [`Self::rating_unit`] (e.g. continuous watts,
    /// stall N·m). Selection requires `rating ≥ demand · safety_factor`.
    fn rating(&self) -> f64;
    /// Human-readable unit of [`Self::rating`], for reporting (e.g. "W", "N·m").
    fn rating_unit(&self) -> &'static str;
}

/// Pick the **lightest** catalogue part whose rating meets `demand · sf`.
///
/// Sorting by mass (not by rating) is what makes "smallest adequate" honest: a
/// family is not strictly ordered by rating once you span sub-families, so the
/// lightest qualifying part is the right minimiser. Returns `None` when even the
/// heaviest part is inadequate — the caller then extrapolates and flags it
/// (see [`crate::scaling`]). Mirrors `manufacture::fasteners::select_bolt`.
pub fn select_smallest_adequate<T: Selectable>(catalogue: &[T], demand: f64, sf: f64) -> Option<T> {
    let need = demand * sf;
    catalogue
        .iter()
        .filter(|p| p.rating() >= need)
        .min_by(|a, b| a.mass_g().total_cmp(&b.mass_g()))
        .cloned()
}

/// The heaviest (largest-rating proxy) part in a catalogue — the ceiling beyond
/// which [`crate::scaling`] must extrapolate. `None` for an empty catalogue.
pub fn largest_part<T: Selectable>(catalogue: &[T]) -> Option<T> {
    catalogue
        .iter()
        .max_by(|a, b| a.rating().total_cmp(&b.rating()))
        .cloned()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone)]
    struct Dummy {
        name: &'static str,
        mass_g: f64,
        rating: f64,
    }
    impl Selectable for Dummy {
        fn name(&self) -> &str {
            self.name
        }
        fn mass_g(&self) -> f64 {
            self.mass_g
        }
        fn rating(&self) -> f64 {
            self.rating
        }
        fn rating_unit(&self) -> &'static str {
            "u"
        }
    }

    fn cat() -> Vec<Dummy> {
        vec![
            Dummy {
                name: "s",
                mass_g: 10.0,
                rating: 100.0,
            },
            Dummy {
                name: "m",
                mass_g: 20.0,
                rating: 300.0,
            },
            Dummy {
                name: "l",
                mass_g: 40.0,
                rating: 900.0,
            },
        ]
    }

    #[test]
    fn picks_lightest_adequate() {
        // Demand 250 with SF 1: "s" (100) fails, "m" (300) is the lightest pass.
        let p = select_smallest_adequate(&cat(), 250.0, 1.0).unwrap();
        assert_eq!(p.name(), "m");
    }

    #[test]
    fn safety_factor_bumps_the_choice_up() {
        // 250 × SF 1.5 = 375 → "m" (300) now fails, "l" (900) is chosen.
        let p = select_smallest_adequate(&cat(), 250.0, 1.5).unwrap();
        assert_eq!(p.name(), "l");
    }

    #[test]
    fn none_when_over_ceiling() {
        assert!(select_smallest_adequate(&cat(), 2000.0, 1.0).is_none());
        assert_eq!(largest_part(&cat()).unwrap().name(), "l");
    }
}
