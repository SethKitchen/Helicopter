//! The six force/moment residuals — the function trim drives to zero.
//!
//! Body axes: x forward, y right, z down. Unknowns
//! `x = [θ₀, θ₁c, θ₁s, θ₀_tr, θ_f (pitch), φ_f (roll)]`.

use crate::G;
use crate::aircraft::Aircraft;
use crate::condition::TrimCondition;
use helisim_bemt::{Config, solve_hover};
use helisim_coupled::{CoupledConfig, solve_coupled};
use helisim_flapping::{Controls, FlapConfig, solve_flapping_with_inflow};
use helisim_forward::ForwardCondition;

/// Main-rotor loads at a control setting and flight condition.
pub struct MainLoads {
    pub thrust: f64,
    pub power: f64,
    pub torque: f64,
    pub beta1c: f64,
    pub beta1s: f64,
    pub hub_roll: f64,
    pub hub_pitch: f64,
}

/// Full evaluation at a trim state: residuals plus the loads behind them.
pub struct Eval {
    pub residuals: [f64; 6],
    pub main: MainLoads,
    pub tail_thrust: f64,
    pub tail_power: f64,
    pub parasite_power: f64,
    pub mu: f64,
}

fn bemt_cfg() -> Config {
    Config {
        n_stations: 40,
        ..Config::default()
    }
}
fn flap_cfg() -> FlapConfig {
    FlapConfig {
        n_azimuth: 48,
        n_radial: 24,
    }
}
fn coupled_cfg() -> CoupledConfig {
    CoupledConfig {
        n_azimuth: 36,
        n_radial: 30,
        flap: flap_cfg(),
        ..CoupledConfig::default()
    }
}

/// Steady hub roll/pitch moments from the flap and hinge offset (zero for a
/// central hinge): `M = (N_b/2)·I_β Ω²(ν²−1)·β₁`.
fn hub_moments(ac: &Aircraft, beta1c: f64, beta1s: f64) -> (f64, f64) {
    let nu2 = ac.flap.nu_beta_sq();
    let i_beta = ac.rho * ac.flap.lift_slope * ac.main.tip_chord * ac.main.radius.powi(4)
        / ac.flap.lock_number;
    let k = i_beta * ac.main_op.omega * ac.main_op.omega * (nu2 - 1.0);
    let half_nb = 0.5 * ac.main.n_blades as f64;
    (half_nb * k * beta1s, half_nb * k * beta1c)
}

/// Calibrate the induced power factor κ so the forward-flight power decomposition
/// (`κ·C_T·λ + profile`) reproduces the hover BEMT power at the hover collective —
/// keeping the trim hover cross-check exact and the power continuous into forward
/// flight.
pub fn calibrate_kappa(ac: &Aircraft, hover_collective: f64) -> f64 {
    let rotor = ac.main.with_collective(hover_collective);
    let s = solve_hover(&rotor, &ac.main_op, ac.main_airfoil.as_ref(), &bemt_cfg());
    let vt = ac.main_op.tip_speed(ac.main.radius);
    let q = ac.rho * ac.main.disk_area() * vt * vt * vt;
    let lam_h = (s.ct.max(1e-9) / 2.0).sqrt();
    // Profile power at hover from the coupled (μ=0) drag integral.
    let cs = solve_coupled(
        &rotor,
        &ac.main_op,
        ac.main_airfoil.as_ref(),
        &ForwardCondition::new(0.0, 0.0),
        &ac.flap,
        &Controls::none(),
        &coupled_cfg(),
    );
    let induced_ref = s.ct * lam_h * q;
    ((s.power - cs.cp_profile * q) / induced_ref).clamp(0.5, 2.0)
}

/// Main-rotor thrust/torque/power (hover BEMT at μ≈0, two-way coupled forward
/// solve otherwise) and the flap response (always, so cyclic tilts the thrust
/// even in hover). `kappa` is the calibrated induced power factor (forward only).
fn main_loads(
    ac: &Aircraft,
    theta0: f64,
    theta1c: f64,
    theta1s: f64,
    mu: f64,
    disk_aoa: f64,
    kappa: f64,
) -> MainLoads {
    let rotor = ac.main.with_collective(theta0);
    let controls = Controls {
        theta_1c: theta1c,
        theta_1s: theta1s,
    };

    if mu < 1e-6 {
        // Hover uses the tip-loss hover BEMT (so trim hover matches milestone 1);
        // flapping still computed so cyclic tilts the thrust.
        let s = solve_hover(&rotor, &ac.main_op, ac.main_airfoil.as_ref(), &bemt_cfg());
        let lam = (s.ct.max(0.0) / 2.0).sqrt();
        let cond = ForwardCondition::new(0.0, 0.0);
        let flap = solve_flapping_with_inflow(
            &rotor,
            &ac.main_op,
            lam,
            &cond,
            &ac.flap,
            &controls,
            &flap_cfg(),
        );
        MainLoads {
            thrust: s.thrust,
            power: s.power,
            torque: s.torque,
            beta1c: flap.beta1c,
            beta1s: flap.beta1s,
            hub_roll: flap.hub_roll_moment,
            hub_pitch: flap.hub_pitch_moment,
        }
    } else {
        // Forward flight uses the two-way flap↔inflow coupled solve.
        let cond = ForwardCondition::new(mu, disk_aoa);
        let cs = solve_coupled(
            &rotor,
            &ac.main_op,
            ac.main_airfoil.as_ref(),
            &cond,
            &ac.flap,
            &controls,
            &coupled_cfg(),
        );
        let power = cs.rotor_power_w(&ac.main_op, &rotor, kappa);
        let (hub_roll, hub_pitch) = hub_moments(ac, cs.beta1c, cs.beta1s);
        MainLoads {
            thrust: cs.thrust_n(&ac.main_op, &rotor),
            power,
            torque: power / ac.main_op.omega,
            beta1c: cs.beta1c,
            beta1s: cs.beta1s,
            hub_roll,
            hub_pitch,
        }
    }
}

/// Tail-rotor thrust and power from its own hover BEMT.
fn tail_loads(ac: &Aircraft, theta0_tr: f64) -> (f64, f64) {
    let rotor = ac.tail.rotor.with_collective(theta0_tr);
    let s = solve_hover(&rotor, &ac.tail.op, ac.tail.airfoil.as_ref(), &bemt_cfg());
    (s.thrust, s.power)
}

/// Evaluate all six residuals (and the loads) at trim state `x`, using induced
/// power factor `kappa` (forward flight only; ignored in hover).
pub fn evaluate(ac: &Aircraft, cond: &TrimCondition, x: &[f64], kappa: f64) -> Eval {
    let (theta0, theta1c, theta1s, theta0_tr, pitch, roll) = (x[0], x[1], x[2], x[3], x[4], x[5]);

    let vt = ac.main_op.tip_speed(ac.main.radius);
    let mu = cond.forward_speed * pitch.cos() / vt;
    let disk_aoa = pitch;

    let m = main_loads(ac, theta0, theta1c, theta1s, mu, disk_aoa, kappa);
    let (tail_thrust, tail_power) = tail_loads(ac, theta0_tr);

    // Main-rotor thrust in the SHAFT frame: ⟂ tip-path plane, tilted from the shaft
    // (shaft −z) by the flapping. β1c>0 (blow-back) tilts thrust aft (−x).
    let t = m.thrust;
    let tx_s = -t * m.beta1c.sin();
    let ty = t * m.beta1s.sin();
    let tz_s = -t * m.beta1c.cos() * m.beta1s.cos();
    // Resolve into body axes through the forward shaft tilt γ_s (mast leans forward):
    // a pitch rotation about +y that gives the up-the-mast thrust a forward (+x) component
    // `T·sin γ_s`, so in hover the fuselage must pitch ~γ_s nose-up to keep thrust vertical
    // (the fx balance). γ_s defaults to 0 (no change for prior aircraft). Hub moments are
    // left in the shaft frame — a γ_s≈3° rotation of a moment vector is a ~0.1% mixing,
    // negligible vs the thrust-vector effect that sets the attitude.
    let gs = ac.shaft_tilt;
    let (tx, tz) = (
        tx_s * gs.cos() - tz_s * gs.sin(),
        tx_s * gs.sin() + tz_s * gs.cos(),
    );

    // Weight resolved into body axes from the attitude.
    let w = ac.mass * G;
    let wx = -w * pitch.sin();
    let wy = w * pitch.cos() * roll.sin();
    let wz = w * pitch.cos() * roll.cos();

    // Airframe parasite drag (along −x) and the power to overcome it (D·V) — the
    // term that produces the high-speed rise of the power bucket.
    let drag = 0.5 * ac.rho * cond.forward_speed * cond.forward_speed * ac.parasite_area;
    let parasite_power = drag * cond.forward_speed;

    // Force balance.
    let fx = tx - drag + wx;
    let fy = ty + tail_thrust + wy;
    let fz = tz + wz;

    // Moment balance about the CG. Hub at (0,0,−h); tail rotor at (−arm,0,−height).
    let h = ac.hub_height;
    let l_tr = ac.tail.arm;
    let h_tr = ac.tail.height;
    let roll_m = h * ty + m.hub_roll + h_tr * tail_thrust;
    let pitch_m = -h * tx + m.hub_pitch + ac.cg_offset * wz;
    let yaw_m = m.torque - l_tr * tail_thrust;

    Eval {
        residuals: [fx, fy, fz, roll_m, pitch_m, yaw_m],
        main: m,
        tail_thrust,
        tail_power,
        parasite_power,
        mu,
    }
}
