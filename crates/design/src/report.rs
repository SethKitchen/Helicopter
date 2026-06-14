//! The computed consequences of a design point, organised by the priority vector.

/// Everything the sizing study computes for one [`crate::DesignCandidate`],
/// grouped by the user's stated priority order (safety → airtime → efficiency →
/// noise). Cost and vertical-integration are reported as the physical drivers
/// (mass, size, RPM) rather than scored, since they depend on the build, not the
/// physics.
#[derive(Clone, Copy, Debug)]
pub struct DesignReport {
    // --- feasibility ---
    /// Whether the rotor can be trimmed to hover the gross mass at this tip speed.
    pub hover_feasible: bool,
    /// Trimmed collective, degrees (NaN if infeasible).
    pub collective_deg: f64,

    // --- priority 1: safety (autorotation) ---
    /// Steady vertical-autorotation descent rate, ft/min (the bounding case).
    pub autorotation_descent_fpm: f64,
    /// Descent rate normalised by hover induced velocity `V_d/v_h`.
    pub autorotation_ratio: f64,
    /// Forward-flight minimum-sink descent rate, ft/min (the realistic case).
    pub forward_min_sink_fpm: f64,
    /// Airspeed at minimum sink, m/s.
    pub forward_min_sink_speed_ms: f64,
    /// Best-glide airspeed (shallowest angle), m/s.
    pub best_glide_speed_ms: f64,
    /// Best-glide angle below horizontal, degrees.
    pub best_glide_angle_deg: f64,
    /// Flare-height equivalent `½IΩ²/W`, m — stored rotor energy as a height.
    pub flare_height_m: f64,
    /// Autorotation index (comparative flare-margin sizing metric).
    pub autorotation_index: f64,
    /// Flare margin `E_flare / descent_KE` (energy bound; >1 necessary, >1.5 good).
    pub flare_margin: f64,
    /// Whether the flare energy bound is met (necessary, not sufficient).
    pub can_flare: bool,
    /// Worst-case rotor-speed decay time after power loss, s — how long you have
    /// to react before RPM is unrecoverable (`E_flare/P_hover`).
    pub rotor_decay_time_s: f64,

    // --- priority 4/5: airtime + efficiency ---
    /// Hover shaft (mechanical) power, W.
    pub hover_shaft_power_w: f64,
    /// Hover electrical power, W (`shaft / η`).
    pub hover_elec_power_w: f64,
    /// Hover endurance, minutes (energy bound: usable Wh / electrical power).
    pub endurance_min: f64,
    /// Rotor figure of merit at the hover point.
    pub figure_of_merit: f64,
    /// Disk loading `W/A`, N/m².
    pub disk_loading: f64,
    /// Power loading `W/P_shaft`, N/W (higher = more efficient lift).
    pub power_loading: f64,

    // --- priority 6: noise ---
    /// Overall rotational-noise level at the observer, dB re 20 µPa.
    pub oaspl_db: f64,
    /// Blade-passage (fundamental) frequency, Hz.
    pub blade_passage_hz: f64,
    /// Tip Mach number (the master noise/efficiency lever).
    pub tip_mach: f64,

    // --- priority 2/3: vertical integration + cost ---
    /// Estimated total build cost (parametric model; default unit costs).
    pub total_cost: f64,
    /// Vertical-integration index (cost-weighted self-build fraction, 0..1).
    pub vertical_integration_index: f64,
    /// Fraction of cost that is irreducible buy-items.
    pub purchased_cost_fraction: f64,
}
