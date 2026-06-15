//! Power-off **autorotation** — the safety case the powered-flight stack omits.
//!
//! Every milestone so far (hover BEMT → trim → 6-DOF → control loops) assumes the
//! rotor is *driven*. For an electric helicopter the defining safety question is
//! the opposite: when the motor or pack fails, the rotor must keep turning on the
//! energy of the descent and the aircraft must reach the ground at a survivable
//! rate. That is autorotation, and it lives in a flow regime the powered solver
//! never visits — air coming **up** through the disk.
//!
//! # Scope (deliberately bounded, first-principles)
//!
//! 1. **Descent-regime inflow** ([`inflow`]) — axial momentum theory extended to
//!    descent. Climb and the *windmill-brake state* (`V_c/v_h ≤ -2`, air driving
//!    the rotor) have exact momentum closed forms; the **vortex-ring / turbulent-
//!    wake** band in between (`-2 < V_c/v_h < 0`) is where momentum theory is
//!    *physically invalid* (recirculation), so there we use the empirically
//!    measured induced-velocity curve. Real steady autorotation sits in exactly
//!    this band — so the empirical curve is load-bearing, and named as such.
//! 2. **Steady vertical autorotation** ([`descent`]) — the equilibrium rate of
//!    descent from the rotor power balance `0 = T(V_c + v_i) + P₀`, i.e.
//!    `V_d = v_i + P₀/T`: the descent supplies the induced + profile power the
//!    motor no longer does. Solved by the project's monotone-residual bisection.
//!    The worst case (no forward speed).
//! 3. **Forward-flight autorotation** ([`forward`]) — the realistic, survivable
//!    case: the glide polar `RoD(V) = P_req(V)/W` and the minimum-sink and
//!    best-glide speeds it yields. Far gentler than the vertical bound.
//! 4. **Flare energy** ([`index`]) — the rotor's stored kinetic energy `½IΩ²` and
//!    the **autorotation index**, the standard measure of how much energy is
//!    available to arrest the descent in the final flare. Small rotors have little
//!    stored energy relative to their weight — the first thing model-scale design
//!    must respect, so it feeds the sizing study.
//! 5. **Flare survivability** ([`survivability`]) — composes the steady descent
//!    rate and the flare energy into the go/no-go energy bound: can the stored
//!    rotor energy arrest the descent? An energy *bound* (no transient dynamics —
//!    named future work), reported as a flare margin + critical hover height.
//! 6. **Height-velocity envelope** ([`height_velocity`]) — the low-speed "dead
//!    man's curve" `h_crit(V)=h_crit_hover−V²/2g` (energy method, no free
//!    parameter, anchored to the validated vertical critical height). The
//!    high-speed lobe is deferred to a dynamic flare model, not faked.
//! 7. **Rotor-speed decay** ([`rotor_decay`]) — the dynamic entry: how many
//!    seconds before rotor RPM is unrecoverable, `t_decay = E_flare/P_hover`
//!    (analytic worst case) plus an RK4 march GATED against that closed form. The
//!    one transient piece, made honest by an exact oracle.
//!
//! # Deliberate limitations (documented, per project habit)
//!
//! * **Vertical autorotation only.** Forward-flight autorotation (lower descent
//!   rate, the practical case) needs the forward-flight power curve; this is the
//!   conservative bounding case and the clean place to anchor the physics.
//! * **Steady state.** The transient entry, rotor-RPM decay, and the flare itself
//!   are dynamic; here we size the *equilibrium* descent and the *available* flare
//!   energy, not the time history.
//! * **Empirical VRS curve.** The induced velocity in the vortex-ring/turbulent-
//!   wake band is a measured fit (cited in [`inflow`]), not first-principles —
//!   because momentum theory genuinely does not hold there. The result is
//!   validated against the measured ideal-autorotation descent-rate band, not a
//!   closed form.
//!
//! One concept per module:
//! * [`inflow`]   — descent-regime induced velocity (closed form + measured curve).
//! * [`descent`]  — [`steady_autorotation`]: the equilibrium vertical descent rate.
//! * [`forward`]  — [`glide_polar`]: forward-flight glide polar + min-sink/best-glide.
//! * [`index`]    — rotor kinetic energy + autorotation index (flare margin).
//! * [`survivability`] — [`assess_vertical`]: the flare energy bound (go/no-go).
//! * [`height_velocity`] — [`build_low_speed_hv`]: the low-speed dead-man's curve.
//! * [`rotor_decay`] — [`decay_time_constant_power`]: entry RPM-decay time.
//! * [`solution`] — [`AutorotationSolution`], the assembled result.

pub mod descent;
pub mod forward;
pub mod height_velocity;
pub mod index;
pub mod inflow;
pub mod rotor_decay;
pub mod solution;
pub mod survivability;

pub use descent::{profile_power, steady_autorotation};
pub use forward::{
    GlidePoint, GlidePolar, forward_descent_rate, forward_induced_velocity, glide_polar,
    power_required,
};
pub use height_velocity::{
    HeightVelocityDiagram, HvPoint, build_low_speed_hv, knee_speed, low_speed_critical_height,
};
pub use index::{autorotation_index, flare_height_equivalent, rotor_kinetic_energy};
pub use inflow::{descent_inflow_ratio, hover_induced_velocity, windmill_brake_inflow_ratio};
pub use rotor_decay::{decay_time_constant_power, simulate_decay, time_to_min_rpm};
pub use solution::AutorotationSolution;
pub use survivability::{FlareAssessment, FlareParams, assess_vertical};

/// Standard gravity, m/s². Weight `W = m g`.
pub const G: f64 = 9.80665;
