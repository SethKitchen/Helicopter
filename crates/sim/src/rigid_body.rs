//! Shared 6-DOF rigid-body rate equations — the body dynamics + kinematics common
//! to the nonlinear marches (gravity components, Coriolis cross-products, and the
//! Euler-rate kinematics). Extracted so the 8-state and 11-state EOM, which used
//! to carry byte-identical copies, can never drift apart.

/// Standard gravity, m/s².
pub const G: f64 = 9.80665;

/// The eight rigid-body rates `[u̇, ẇ, q̇, θ̇, v̇, ṗ, ṙ, φ̇]` from the body state
/// `[u, w, q, θ, v, p, r, φ]` (body axes x-fwd/y-right/z-down), the resolved aero
/// forces/moments `[X, Y, Z, L, M, N]`, the `mass`, and the principal inertia
/// `[Iₓ, I_y, I_z]`. The output order matches the state order.
pub fn rigid_body_rates(
    state: [f64; 8],
    forces: [f64; 6],
    mass: f64,
    inertia: [f64; 3],
) -> [f64; 8] {
    let [u, w, q, theta, v, p, r, phi] = state;
    let [xf, yf, zf, lm, mm, nm] = forces;
    let [ixx, iyy, izz] = inertia;

    let (gx, gy, gz) = (
        -G * theta.sin(),
        G * theta.cos() * phi.sin(),
        G * theta.cos() * phi.cos(),
    );
    let udot = -(q * w - r * v) + gx + xf / mass;
    let vdot = -(r * u - p * w) + gy + yf / mass;
    let wdot = -(p * v - q * u) + gz + zf / mass;
    let pdot = (lm + (iyy - izz) * q * r) / ixx;
    let qdot = (mm + (izz - ixx) * r * p) / iyy;
    let rdot = (nm + (ixx - iyy) * p * q) / izz;
    let thetadot = q * phi.cos() - r * phi.sin();
    let phidot = p + (q * phi.sin() + r * phi.cos()) * theta.tan();

    [udot, wdot, qdot, thetadot, vdot, pdot, rdot, phidot]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// At rest and level (all states 0), only gravity acts: `ẇ = g + Z/m` and all
    /// other rates are zero — a hand-checkable anchor.
    #[test]
    fn level_rest_only_gravity_and_applied_z() {
        let m = 10.0;
        let rates = rigid_body_rates([0.0; 8], [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], m, [1.0, 2.0, 3.0]);
        // udot=0+(-g·sin0)+0=0; wdot=0+g·cos0·cos0+0=g; all rates zero except none here.
        assert!((rates[1] - G).abs() < 1e-12, "ẇ = g with no Z force");
        for (i, &x) in rates.iter().enumerate() {
            if i != 1 {
                assert!(x.abs() < 1e-12, "rate {i} should be 0");
            }
        }
        // An applied vertical force adds Z/m.
        let r2 = rigid_body_rates(
            [0.0; 8],
            [0.0, 0.0, -m * G, 0.0, 0.0, 0.0],
            m,
            [1.0, 2.0, 3.0],
        );
        assert!(r2[1].abs() < 1e-12, "Z = -mg exactly cancels gravity → ẇ=0");
    }

    /// Inertia cross-coupling: a roll·yaw rate produces a pitch acceleration
    /// `q̇ = (I_z − Iₓ)·r·p / I_y` when no moment is applied.
    #[test]
    fn inertia_cross_coupling() {
        let (ixx, iyy, izz) = (1.0, 2.0, 4.0);
        let (p, r) = (0.3, 0.5);
        let rates = rigid_body_rates(
            [0.0, 0.0, 0.0, 0.0, 0.0, p, r, 0.0],
            [0.0; 6],
            10.0,
            [ixx, iyy, izz],
        );
        assert!((rates[2] - (izz - ixx) * r * p / iyy).abs() < 1e-12);
    }
}
