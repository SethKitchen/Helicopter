//! Assembled rotor-noise spectrum at one observer.

/// One blade-passage harmonic of the rotational-noise spectrum.
#[derive(Clone, Copy, Debug)]
pub struct Harmonic {
    /// Harmonic number (1 = fundamental blade-passage tone).
    pub m: usize,
    /// Tone frequency, Hz.
    pub frequency_hz: f64,
    /// Signed rms acoustic pressure, Pa.
    pub pressure_pa: f64,
    /// Sound pressure level of this tone, dB re 20 µPa.
    pub spl_db: f64,
}

/// Rotational-noise spectrum and overall level at a single observer location.
#[derive(Clone, Debug)]
pub struct NoiseSpectrum {
    /// Per-harmonic tones, `m = 1 ..`.
    pub harmonics: Vec<Harmonic>,
    /// Overall (energy-summed) sound pressure level, dB re 20 µPa.
    pub oaspl_db: f64,
    /// Observer distance from the hub, m.
    pub observer_distance_m: f64,
    /// Observer angle from the rotor axis, radians.
    pub observer_angle_rad: f64,
}
