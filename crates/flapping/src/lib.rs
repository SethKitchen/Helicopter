//! Rigid-blade, first-harmonic blade flapping.
//!
//! # Physics
//!
//! A rigid blade flaps about its hinge under aerodynamic, centrifugal-restoring
//! and inertial moments. In nondimensional azimuth `ψ = Ωt` (`' = d/dψ`):
//!
//! ```text
//! β'' + ν_β² β = (γ/2) ∫₀¹ x (u_T² θ − u_T u_P) dx
//! ```
//!
//! where the Lock number `γ = ρ a c R⁴ / I_β` governs the ratio of aerodynamic
//! to inertial flapping forces, `ν_β` is the rotating flap frequency (1 for a
//! central hinge, `>1` with hinge offset), and the through-disk velocity carries
//! the flap motion: `u_P = λ + x β' + μ β cosψ`.
//!
//! We take the **first-harmonic** solution `β(ψ) = β₀ − β₁c cosψ − β₁s sinψ` and
//! solve for the three coefficients by **harmonic balance** — projecting the ODE
//! onto {1, cosψ, sinψ} gives a 3×3 linear system. This is a *new solver shape*
//! for the project: the first that is not the monotone-residual bisection used
//! for hover/forward inflow and trim — it is a small linear solve ([`linalg`]).
//!
//! # The result that matters
//!
//! Cyclic flapping tilts the tip-path plane. The large uncommanded rolling
//! moment a *rigid* blade produced in forward flight (see the `forward` crate)
//! is reacted by flapping instead of reaching the hub: for a central hinge the
//! hub moment is zero and the moment reappears as a tip-path-plane tilt; with
//! hinge offset a residual hub moment remains, proportional to the offset. And
//! the gyroscopic **90° phase lag** — peak flap response a quarter-revolution
//! after peak aerodynamic moment — falls out of the solve on its own.
//!
//! # Deliberate limitations (documented)
//!
//! * Rigid blade flapping about a hinge, **first harmonic only** — no elastic
//!   bending, no lead-lag, no higher harmonics.
//! * **Flapping responds to a fixed inflow.** Flapping and inflow are coupled
//!   (flap changes loading changes inflow), but we reuse the forward-flight
//!   inflow `λ` and treat flapping as responding to it, rather than re-coupling.
//! * Linear lift (constant slope `a`, no stall/compressibility) and the reverse-
//!   flow region is *not* nulled — both to match the analytic closed-form oracle.
//!   The root cutout is neglected in the flap integral (integrated 0→1).
//!
//! One concept per module:
//! * [`properties`]  — [`FlapProperties`] (Lock number, hinge offset, `ν_β`).
//! * [`controls`]    — [`Controls`] (cyclic pitch inputs).
//! * [`config`]      — [`FlapConfig`] (integration resolution).
//! * [`linalg`]      — 3×3 linear solve.
//! * [`harmonics`]   — harmonic-balance assembly of the flap system.
//! * [`closed_form`] — analytic first-harmonic coefficients (the oracle).
//! * [`solution`]    — [`FlapSolution`].
//! * [`solver`]      — [`solve_flapping`].

pub mod closed_form;
pub mod config;
pub mod controls;
pub mod harmonics;
pub mod linalg;
pub mod properties;
pub mod solution;
pub mod solver;

pub use closed_form::closed_form_coefficients;
pub use config::FlapConfig;
pub use controls::Controls;
pub use harmonics::build_system;
pub use linalg::solve3;
pub use properties::FlapProperties;
pub use solution::FlapSolution;
pub use solver::{solve_flapping, solve_flapping_with_inflow};
