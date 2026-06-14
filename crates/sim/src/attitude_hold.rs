//! Attitude hold (5k): proportional attitude feedback layered on top of the 5j
//! rate damper — the standard inner-rate / outer-attitude cascade. The rate loop
//! (validated in 5j) is the inner loop; this wraps it with `θ→lon-cyclic` and
//! `φ→lat-cyclic` to regulate the trim attitude.
//!
//! **What it fixes (pre-computed target):** the slow speed/phugoid mode the rate
//! damper structurally could not reach (left at +0.024 at hover in 5j). The
//! phugoid is a pitch-attitude/speed oscillation, and attitude feedback is a
//! pitch *stiffness* — exactly the loop with authority over it.
//!
//! **Scope (named, to keep 5k from sprawling into guidance):** this is attitude
//! *hold* — regulate to the trim attitude against a disturbance. NOT attitude
//! *command tracking* (follow a time-varying reference) and NOT guidance (follow a
//! position trajectory): those are the outer-loop milestone (5l+). Holding to trim
//! is what gives 5k its clean steady-state-error oracle.
//!
//! **Seam discipline (second application):** design and validate off the wake-skew
//! seam (small forward speed, χ differentiable, closed-loop eigenvalues
//! trustworthy), then confirm across the seam at hover with the nonlinear march.

use crate::control::Channel;
use crate::sas::RateSas;

/// Layer proportional attitude hold onto an inner rate damper: `Δθ1s += −k_θ·θ`,
/// `Δθ1c += −k_φ·φ` (same restoring sign as the rate terms, since ∂q̇/∂θ1s>0 and
/// ∂ṗ/∂θ1c>0). Returns the combined state-feedback law (a single gain matrix), so
/// the closed-loop matrix and the augmented march handle it unchanged.
pub fn attitude_hold(mut inner: RateSas, k_theta: f64, k_phi: f64) -> RateSas {
    inner.gain[Channel::LonCyclic as usize][3] += -k_theta; // θ is state index 3
    inner.gain[Channel::LatCyclic as usize][7] += -k_phi; // φ is state index 7
    inner
}
