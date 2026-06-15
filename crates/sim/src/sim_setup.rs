//! The shared plant setup for the closed-loop time-marches — the aircraft, its
//! inertia, and the trimmed body velocity the equilibrium is taken at. Grouping it
//! (with the integration window passed as a `[dt, t_end]` span) keeps the
//! `simulate*` signatures narrow across the SAS / attitude / velocity-hold layers.

use helisim_dynamics::Inertia;
use helisim_trim::Aircraft;

/// Plant + equilibrium-velocity context for an 11-state-based time march.
#[derive(Clone, Copy)]
pub struct Sim11Setup<'a> {
    pub ac: &'a Aircraft,
    pub j: Inertia,
    /// Body velocity the equilibrium is trimmed at, m/s.
    pub vel: [f64; 3],
}
