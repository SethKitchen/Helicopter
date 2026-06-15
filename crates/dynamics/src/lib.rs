//! Linearized flight dynamics: stability & control derivatives and the modal
//! eigenvalues that reveal the aircraft's open-loop character.
//!
//! # Scope (5c)
//!
//! Linear, small-perturbation analysis about a trusted trim equilibrium — NOT
//! nonlinear time-marching (that is 5d). Two pieces:
//!
//! 1. **Stability & control derivatives.** Perturb the trimmed body state
//!    (Δu, Δw, Δq, …) and each control, and numerically measure the force/moment
//!    response — the derivatives `Xu, Zw, Mu, Mq, …`. This is the *fourth* use of
//!    the perturbation engine: the same numerical-Jacobian machinery the trim
//!    Newton used, applied to body states instead of trim unknowns.
//! 2. **Linear modes.** Assemble the system matrix `A` from the derivatives and
//!    take its eigenvalues.
//!
//! # The headline
//!
//! Hover helicopter dynamics have a famous signature: stable fast subsidences
//! plus an **unstable low-frequency oscillatory mode** (the "hovering cubic" /
//! pitch–speed instability). A hovering helicopter is open-loop unstable — that
//! is why they are hard to fly and why stability augmentation exists. If that
//! unstable oscillatory root falls out of the eigenvalues without being put
//! there, the milestone has landed (the 6-DOF analogue of FM-without-calibration
//! and the 90° lag).
//!
//! # New solver shape
//!
//! Eigenvalue extraction for a modest real matrix, std-only: the characteristic
//! polynomial via Faddeev–LeVerrier ([`eigen::char_poly`]) and its complex roots
//! via Durand–Kerner ([`eigen::roots`]). Validated against the analytically-
//! rootable longitudinal characteristic polynomial — the analytic anchor for the
//! new primitive.
//!
//! # Validation ledger note
//!
//! The dynamics rest on **force/moment** derivatives (Mu, Mq, Zw, Xu), which come
//! from the trusted force/moment residuals — NOT from the κ-calibrated forward
//! power. So this validation layer is clean of the power calibration.
//!
//! One concept per module:
//! * [`complex`]      — minimal complex number.
//! * [`eigen`]        — char-poly + complex root finder (eigenvalues).
//! * [`aero`]         — perturbable main-rotor body forces/moments.
//! * [`derivatives`]  — the stability-derivative matrix.
//! * [`model`]        — assemble `A`, eigenvalues, classify modes.

pub mod aero;
pub mod complex;
pub mod context;
pub mod coupled8;
pub mod derivatives;
pub mod eigen;
pub mod flap_general;
pub mod full_aero;
pub mod inflow_coupling;
pub mod lateral;
pub mod model;
pub mod pitt_peters;
pub mod schur;

pub use complex::Complex;
pub use context::RotorAero;
pub use coupled8::{CoupledModal, Inertia, analyze_coupled_hover};
pub use derivatives::{LongitudinalDerivatives, longitudinal_derivatives};
pub use eigen::{char_poly, eigenvalues, eigenvalues_via_char_poly, roots};
pub use full_aero::{Forces6, InflowAero, main_rotor_full, rotate6, uniform_inflow};
pub use inflow_coupling::{inflow_rate, main_rotor_with_inflow, march_inflow, quasi_static_inflow};
pub use lateral::{
    LateralDerivatives, LateralModal, analyze_hover_lateral, lateral_cubic, lateral_derivatives,
    main_rollrate_response, main_velocity_response, tail_thrust,
};
pub use model::{
    ModalAnalysis, Mode, analyze_hover_longitudinal, hover_collective_for_weight, hovering_cubic,
};
pub use pitt_peters::{
    apparent_mass, gravest_time_constant, inflow_derivative, l_matrix, steady_inflow_for,
};
pub use schur::schur_eigenvalues;
