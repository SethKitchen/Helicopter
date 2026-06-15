//! Scaling beyond the catalogue — extrapolate, and be honest about it.
//!
//! The catalogues are real hobby hardware, so they end. When a design's demand
//! exceeds the largest member (it will, climbing toward the human-scale roadmap
//! target), we don't fail silently and we don't fabricate a part: we **estimate**
//! the mass by extrapolating along the family's best specific rating
//! (power-per-gram for motors, torque-per-gram for servos) and **flag** the
//! regime change. At human scale a "servo" is really an electro-hydraulic
//! actuator and a "motor" a custom traction unit — different hardware — so the
//! flag is a named scope boundary, mirroring the recommender's grid-edge flag.

use crate::selectable::{Selectable, largest_part};

/// The family's best specific rating, (rating-unit) per gram — taken from the
/// largest member (its asymptotic efficiency). Used as the extrapolation slope.
pub fn family_specific_rating<T: Selectable>(catalogue: &[T]) -> f64 {
    catalogue
        .iter()
        .map(|p| p.rating() / p.mass_g())
        .fold(0.0, f64::max)
}

/// Outcome of sizing against a catalogue: either a real chosen part, or an
/// extrapolated estimate flagged as beyond the catalogue.
#[derive(Clone, Debug)]
pub struct Sized<T> {
    /// The chosen catalogue part, if one was adequate.
    pub part: Option<T>,
    /// Estimated mass, grams (the part's mass, or the extrapolated estimate).
    pub mass_g: f64,
    /// True when no catalogue part met the demand and `mass_g` is extrapolated.
    pub beyond_catalogue: bool,
    /// Human-readable note (the regime-change flag when extrapolated).
    pub note: Option<String>,
}

/// Size against a catalogue: return the chosen part, or extrapolate + flag.
pub fn size_or_extrapolate<T: Selectable>(
    catalogue: &[T],
    demand: f64,
    sf: f64,
    kind: &str,
) -> Sized<T> {
    use crate::selectable::select_smallest_adequate;
    if let Some(part) = select_smallest_adequate(catalogue, demand, sf) {
        let mass_g = part.mass_g();
        return Sized {
            part: Some(part),
            mass_g,
            beyond_catalogue: false,
            note: None,
        };
    }
    // Beyond the catalogue: extrapolate mass from the family's best specific rating.
    let spec = family_specific_rating(catalogue);
    let mass_g = if spec > 0.0 {
        demand * sf / spec
    } else {
        f64::NAN
    };
    let ceiling = largest_part(catalogue);
    let unit = ceiling.as_ref().map(|p| p.rating_unit()).unwrap_or("");
    let ceil_rating = ceiling.as_ref().map(|p| p.rating()).unwrap_or(0.0);
    let note = Some(format!(
        "⚠ {kind} demand {:.1} {unit} exceeds the catalogue ceiling ({:.1} {unit}); \
         mass extrapolated to ~{:.0} g along the family. At this scale the hardware \
         changes regime (EHA actuator / custom traction motor) — confirm a real part.",
        demand * sf,
        ceil_rating,
        mass_g
    ));
    Sized {
        part: None,
        mass_g,
        beyond_catalogue: true,
        note,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motor::scorpion_hk_catalogue;
    use crate::servo::align_hv_catalogue;

    #[test]
    fn specific_rating_from_largest_member() {
        // Motors: best is the 4525 (4450 W / 503 g ≈ 8.85 W/g).
        let spec = family_specific_rating(&scorpion_hk_catalogue());
        assert!((spec - 4450.0 / 503.0).abs() < 1e-6);
    }

    #[test]
    fn within_catalogue_returns_real_part_no_flag() {
        let s = size_or_extrapolate(&scorpion_hk_catalogue(), 1000.0, 1.0, "motor");
        assert!(!s.beyond_catalogue);
        assert!(s.part.is_some());
        assert!(s.note.is_none());
    }

    #[test]
    fn beyond_catalogue_extrapolates_and_flags() {
        // 50 kW demand is far past the 4450 W ceiling → flag + extrapolated mass.
        let s = size_or_extrapolate(&scorpion_hk_catalogue(), 50_000.0, 1.0, "motor");
        assert!(s.beyond_catalogue);
        assert!(s.part.is_none());
        assert!(s.note.is_some());
        // ~50000 / 8.85 ≈ 5650 g.
        assert!((s.mass_g - 50_000.0 / (4450.0 / 503.0)).abs() < 1.0);
    }

    #[test]
    fn servo_beyond_catalogue_flags_regime_change() {
        // 20 N·m is far past the DS820 ceiling (2.26 N·m).
        let s = size_or_extrapolate(&align_hv_catalogue(), 20.0, 1.0, "servo");
        assert!(s.beyond_catalogue);
        assert!(s.note.unwrap().contains("regime"));
    }
}
