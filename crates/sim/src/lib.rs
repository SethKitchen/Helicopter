//! Nonlinear longitudinal time-marching of the helicopter rigid body.
//!
//! # Scope (5d)
//!
//! Nonlinear integration of the **longitudinal** state `[u, w, q, θ]` only —
//! the same subsystem the 5c linear analysis described, so the time-march can be
//! validated against the pre-computed eigenvalues. NOT lateral-directional (5e),
//! not maneuvers, not control-input time histories, not dynamic inflow, not
//! stability augmentation — each a clean later increment.
//!
//! # The architecture shift
//!
//! This is the first time the rotor model is a **callee inside an integration
//! loop** rather than solved once per condition. Each RK4 substep evaluates the
//! body-state derivative, which re-solves the rotor at the current `(u, w, q)`
//! with controls held at trim. We use the **quasi-static rotor** (the flap↔inflow
//! fixed point re-converged every substep) — exactly the assumption under which
//! the 5c derivatives were measured, so the time-march should reproduce the
//! linear eigenvalues. Finite-state **dynamic inflow** (Pitt–Peters), where the
//! rotor responds fast but not instantly, is the documented next refinement, not
//! this milestone.
//!
//! # The pre-computed validation gate
//!
//! 5c predicted an unstable oscillation: `0.64 ± 1.17i` → period ~5.4 s, doubling
//! ~1.1 s. Perturbing trimmed hover and fitting the early growth of the nonlinear
//! trajectory must match that period and doubling time (before nonlinearity bends
//! it). That is a falsifiable, quantitative gate — not a "looks unstable" check.
//! Verified at more than one step size so the match is not a step-size artifact.
//!
//! One concept per module:
//! * [`rk4`]      — fixed-step RK4 integrator.
//! * [`eom`]      — the nonlinear longitudinal equations of motion.
//! * [`simulate`] — run a trajectory.
//! * [`analysis`] — fit period / growth from a trajectory (for the gate).

pub mod analysis;
pub mod attitude_hold;
pub mod control;
pub mod coupled_march;
pub mod driven_equilibrium;
pub mod driven_march;
pub mod eom;
pub mod pi_attitude;
pub mod rk4;
pub mod sas;
pub mod sim_setup;
pub mod simulate;
pub mod velocity_hold;

pub use analysis::{ModeFit, fit_growing_oscillation};
pub use sim_setup::Sim11Setup;
pub use attitude_hold::attitude_hold;
pub use control::{Channel, ControlSchedule, Pulse, Step, Trim};
pub use coupled_march::{
    State8, equilibrium_state8, linearize8, simulate8, solve_equilibrium8, state_derivative8,
};
pub use driven_equilibrium::{
    equilibrium_state11, equilibrium_state11_at, model11, model11_at, solve_equilibrium11,
    solve_equilibrium11_at,
};
pub use driven_march::{
    Model11, State11, control_matrix11, control_matrix11_at, deriv11, linearize11, linearize11_at,
    simulate11, state_derivative11,
};
pub use eom::{LongState, state_derivative};
pub use pi_attitude::{PiAttitudeHold, State13, augmented_matrix, simulate13};
pub use rk4::{rk4_step, rk4_step_t};
pub use sas::{RateSas, closed_loop_matrix, simulate11_sas, simulate11_sas_dist};
pub use simulate::{Trajectory, simulate_hover_longitudinal, simulate_linear, simulate_linear_nd};
pub use velocity_hold::{State15, VelocityHold, deriv15, linearize15, simulate15};
