//! A single design point — the geometry and powertrain knobs a builder controls.

use helisim_rotor::{Operating, Rotor};
use std::f64::consts::PI;

/// One model-helicopter design point. Everything the sizing metrics need that the
/// builder actually chooses; the *consequences* (power, airtime, noise,
/// autorotation margin) are computed in [`crate::evaluate`].
#[derive(Clone, Copy, Debug)]
pub struct DesignCandidate {
    /// Gross mass to hover, kg.
    pub gross_mass_kg: f64,
    /// Number of blades.
    pub n_blades: usize,
    /// Rotor radius, m.
    pub radius_m: f64,
    /// Blade chord, m (rectangular blade).
    pub chord_m: f64,
    /// Root cutout as a fraction of radius.
    pub root_cutout: f64,
    /// Tip speed `ΩR`, m/s — the primary noise/efficiency lever, held instead of
    /// RPM so a radius sweep can keep the tip speed fixed.
    pub tip_speed_ms: f64,
    /// Blade mean profile-drag coefficient (for autorotation profile power).
    pub blade_cd0: f64,
    /// Equivalent parasite flat-plate area `f`, m² (for forward-flight glide).
    pub flat_plate_area_m2: f64,
    /// Blade areal mass density, kg/m² (for estimating rotor inertia from
    /// geometry — model blades ~2–4, full-scale ~10–20).
    pub blade_areal_density_kg_m2: f64,
    /// Rotor polar moment of inertia about the shaft, kg·m² (flare energy).
    pub rotor_inertia: f64,
    /// Usable pack energy, Wh (nominal × usable fraction already applied is *not*
    /// assumed — see [`Self::usable_fraction`]).
    pub pack_energy_wh: f64,
    /// Fraction of pack energy usable before cut-off.
    pub usable_fraction: f64,
    /// Combined motor + ESC efficiency, mechanical→electrical.
    pub powertrain_eta: f64,
    /// Noise observer distance from the hub, m.
    pub observer_distance_m: f64,
    /// Noise observer angle from the rotor axis, degrees.
    pub observer_angle_deg: f64,
}

impl DesignCandidate {
    /// A starting model-scale electric helicopter: ~3.5 kg, 0.6 m rotor, 2 blades,
    /// ~125 m/s tip speed. A concrete point to sweep around, not a recommendation.
    pub fn model() -> Self {
        DesignCandidate {
            gross_mass_kg: 3.5,
            n_blades: 2,
            radius_m: 0.6,
            chord_m: 0.05,
            root_cutout: 0.15,
            tip_speed_ms: 125.0,
            blade_cd0: 0.011,
            flat_plate_area_m2: 0.02,
            blade_areal_density_kg_m2: 2.5,
            rotor_inertia: 0.012,
            pack_energy_wh: 120.0,
            usable_fraction: 0.8,
            powertrain_eta: 0.85,
            observer_distance_m: 10.0,
            observer_angle_deg: 45.0,
        }
    }

    /// Rotational speed, rad/s (`Ω = V_tip / R`).
    pub fn omega(&self) -> f64 {
        self.tip_speed_ms / self.radius_m
    }

    /// Disk area `πR²`, m².
    pub fn disk_area(&self) -> f64 {
        PI * self.radius_m * self.radius_m
    }

    /// Nominal solidity `σ = N_b c / (πR)`.
    pub fn solidity(&self) -> f64 {
        self.n_blades as f64 * self.chord_m / (PI * self.radius_m)
    }

    /// Build the (zero-collective) rotor geometry; trim sets the collective.
    pub fn rotor(&self) -> Rotor {
        Rotor::rectangular(
            self.n_blades,
            self.radius_m,
            self.chord_m,
            0.0,
            self.root_cutout,
        )
    }

    /// Build the operating point at the candidate's tip speed.
    pub fn operating(&self) -> Operating {
        Operating::from_rpm(self.omega() * 60.0 / (2.0 * PI))
    }

    /// A copy at a different radius, **holding tip speed fixed** (RPM is implied).
    /// The clean way to isolate the disk-loading trade in a sweep.
    pub fn with_radius(&self, radius_m: f64) -> Self {
        DesignCandidate { radius_m, ..*self }
    }

    /// Estimate the rotor polar inertia from blade geometry, treating each blade
    /// as a uniform lamina of linear density `μ = chord · areal_density` running
    /// from the cutout to the tip: `I = N_b · μ R³ (1−e³)/3`, `e = root_cutout`.
    /// This makes the safety flare-margin respond physically when the recommender
    /// varies radius/chord, instead of holding inertia fixed.
    pub fn estimate_rotor_inertia(&self) -> f64 {
        let mu = self.chord_m * self.blade_areal_density_kg_m2;
        let e = self.root_cutout;
        self.n_blades as f64 * mu * self.radius_m.powi(3) * (1.0 - e * e * e) / 3.0
    }

    /// A copy with new rotor geometry (blades/radius/chord/tip speed) **and** the
    /// rotor inertia re-estimated from that geometry — the search primitive the
    /// recommender uses so each candidate is self-consistent.
    pub fn with_geometry(
        &self,
        n_blades: usize,
        radius_m: f64,
        chord_m: f64,
        tip_speed_ms: f64,
    ) -> Self {
        let mut c = DesignCandidate {
            n_blades,
            radius_m,
            chord_m,
            tip_speed_ms,
            ..*self
        };
        c.rotor_inertia = c.estimate_rotor_inertia();
        c
    }

    /// A representative **human-scale 2-passenger** electric helicopter — the
    /// ultimate target. R22-class geometry; parameters are defensible
    /// order-of-magnitude assumptions (not a specific aircraft), to show how the
    /// priority vector and the safety findings scale up from the model.
    pub fn human_scale_2pax() -> Self {
        DesignCandidate {
            gross_mass_kg: 700.0, // 2 occupants + structure + motor + pack
            n_blades: 2,
            radius_m: 4.0,
            chord_m: 0.18,
            root_cutout: 0.15,
            tip_speed_ms: 200.0,
            blade_cd0: 0.010,
            flat_plate_area_m2: 0.5,
            blade_areal_density_kg_m2: 12.0,
            rotor_inertia: 1200.0,
            pack_energy_wh: 40_000.0, // ~40 kWh
            usable_fraction: 0.85,
            powertrain_eta: 0.88,
            observer_distance_m: 150.0, // a flyover-class distance
            observer_angle_deg: 45.0,
        }
    }
}
