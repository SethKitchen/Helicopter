//! Control-input time histories (5i). The aircraft becomes *driven*: the four
//! pilot controls are functions of time, expressed as **deltas from trim**.
//!
//! # Control conventions (pinned to physical effect, validated in the gates)
//!
//! Body axes x-fwd / y-right / z-down; roll φ right-down +, pitch θ nose-up +,
//! yaw r nose-right +. All control deltas are in **radians** of blade pitch.
//!
//! * [`Channel::Collective`] `Δθ₀` — positive raises main-rotor thrust → climb
//!   (`ẇ < 0`, since `w` is body-down).
//! * [`Channel::LatCyclic`] `Δθ1c` — positive → positive roll moment → **right
//!   roll** (consistent with `∂Mx/∂θ1c > 0`).
//! * [`Channel::LonCyclic`] `Δθ1s` — positive → pitch moment (sign pinned to the
//!   measured `∂My/∂θ1s` and reported by `helisim fly`).
//! * [`Channel::Pedal`] `Δθ₀_tail` — positive raises tail-rotor thrust → yaw
//!   reaction about the tail arm.
//!
//! The polymorphism boundary is [`ControlSchedule`]: implementations are swapped
//! freely (`&dyn ControlSchedule`) — trim hold, step, pulse, doublet.

/// The four control channels, indexing the delta vector `[Δθ₀, Δθ1c, Δθ1s, Δθ_tail]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Channel {
    Collective = 0,
    LatCyclic = 1,
    LonCyclic = 2,
    Pedal = 3,
}

impl Channel {
    pub fn name(self) -> &'static str {
        match self {
            Channel::Collective => "collective θ₀",
            Channel::LatCyclic => "lateral cyclic θ1c",
            Channel::LonCyclic => "longitudinal cyclic θ1s",
            Channel::Pedal => "pedal θ₀_tail",
        }
    }
}

/// A control-input time history, as deltas from trim. Swappable behind `&dyn`.
pub trait ControlSchedule {
    /// Control deltas `[Δθ₀, Δθ1c, Δθ1s, Δθ_tail]` (rad) at time `t` (s).
    fn deltas(&self, t: f64) -> [f64; 4];
}

/// Hold trim — all deltas zero (the equilibrium / free-response schedule).
pub struct Trim;
impl ControlSchedule for Trim {
    fn deltas(&self, _t: f64) -> [f64; 4] {
        [0.0; 4]
    }
}

/// A step on one channel: zero until `t_start`, then `amplitude` forever.
pub struct Step {
    pub channel: Channel,
    pub amplitude: f64,
    pub t_start: f64,
}
impl ControlSchedule for Step {
    fn deltas(&self, t: f64) -> [f64; 4] {
        let mut d = [0.0; 4];
        if t >= self.t_start {
            d[self.channel as usize] = self.amplitude;
        }
        d
    }
}

/// A rectangular pulse on one channel, then back to trim (the open-loop-divergence
/// excitation: kick it, release, watch it diverge along the unstable modes).
pub struct Pulse {
    pub channel: Channel,
    pub amplitude: f64,
    pub t_start: f64,
    pub duration: f64,
}
impl ControlSchedule for Pulse {
    fn deltas(&self, t: f64) -> [f64; 4] {
        let mut d = [0.0; 4];
        if t >= self.t_start && t < self.t_start + self.duration {
            d[self.channel as usize] = self.amplitude;
        }
        d
    }
}
