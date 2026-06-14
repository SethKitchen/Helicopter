//! Forward-flight Blade Element Momentum Theory for a single rigid rotor.
//!
//! # Scope (deliberately bounded)
//!
//! This is BEMT with a forward-flight *inflow*, not a free-wake or unsteady
//! model. Two pieces, both first-principles:
//!
//! 1. **Glauert momentum inflow** — the induced velocity now depends on advance
//!    ratio `μ` and disk tilt: `λ = μ tanα + C_T / (2√(μ²+λ²))`. Solved by the
//!    same monotone-residual bisection used for hover ([`inflow`]).
//! 2. **Azimuthal blade-element integration** — each element sees
//!    `U_T = Ωr + V sinψ` (advancing) … `Ωr − V sinψ` (retreating). Loads are
//!    integrated over azimuth `ψ ∈ [0, 2π)` and radius, which is where the
//!    advancing/retreating asymmetry — and the resulting **uncommanded rolling
//!    moment** — appears ([`solver`]).
//!
//! The inflow solve and the integral are coupled (inflow depends on thrust,
//! thrust depends on inflow), so the outer inflow bisection wraps the inner
//! azimuthal+radial double integral.
//!
//! # Deliberate limitations (documented, per project habit)
//!
//! * **Uniform inflow** (Glauert), not a contracted or even linearly-varying
//!   (Drees) wake. BEMT in forward flight is a known approximation: it gets
//!   trends and the trim ballpark right but degrades at high `μ`.
//! * **No flapping.** The blade is rigid at fixed pitch. The large rolling
//!   moment this produces is *the result* — it is the physical reason blade
//!   flapping exists, and it motivates the next milestone rather than being
//!   crammed into this one.
//! * **Reverse-flow region** (inboard retreating area where `U_T < 0`) has its
//!   lift nulled as a first cut, and its area fraction is reported. Tiny at low
//!   `μ`, it grows with speed.
//!
//! One concept per module:
//! * [`condition`] — the [`ForwardCondition`] (advance ratio + disk tilt).
//! * [`config`]    — solver settings ([`ForwardConfig`]).
//! * [`inflow`]    — Glauert momentum inflow + its analytic closed form.
//! * [`solution`]  — integrated result ([`ForwardSolution`]).
//! * [`solver`]    — [`solve_forward`]: outer inflow bisection + double integral.

pub mod condition;
pub mod config;
pub mod inflow;
pub mod solution;
pub mod solver;

pub use condition::ForwardCondition;
pub use config::ForwardConfig;
pub use inflow::{glauert_inflow, glauert_inflow_closed_form, induced_power_ratio, momentum_ct};
pub use solution::ForwardSolution;
pub use solver::solve_forward;
