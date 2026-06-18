//! Flight-envelope limits validated against their closed forms and monotone trends.
//! Without these, an "optimum" could sit past Vne or above the power ceiling.

use helisim_design::{
    AircraftPower, EnvelopeLimits, advancing_tip_vne, analyze_envelope, isa_density_altitude_m,
    max_level_speed, retreating_stall_mu,
};

fn aircraft() -> AircraftPower {
    AircraftPower {
        gross_mass_kg: 3.5,
        rho: 1.225,
        disk_area_m2: 1.54,
        figure_of_merit: 0.7,
        flat_plate_area_m2: 0.05,
        profile_power_w: 60.0,
        powertrain_eta: 0.8,
    }
}

fn limits() -> EnvelopeLimits {
    EnvelopeLimits {
        sound_speed_mps: 340.0,
        advancing_tip_mach_limit: 0.85,
        cl_max: 1.2,
        power_available_w: 1200.0,
    }
}

/// ADVANCING-TIP MACH (closed form): `Vne = a·M_limit − V_tip`, and the advancing
/// tip Mach AT that Vne is exactly the limit.
#[test]
fn advancing_tip_vne_matches_closed_form() {
    let (a, m, vtip) = (340.0, 0.85, 120.0);
    let vne = advancing_tip_vne(vtip, a, m);
    assert!((vne - (a * m - vtip)).abs() < 1e-9);
    // Advancing tip Mach at Vne == limit.
    assert!(((vtip + vne) / a - m).abs() < 1e-12);
    // A tip already past the limit has zero margin (clamped).
    assert_eq!(advancing_tip_vne(a * m + 10.0, a, m), 0.0);
}

/// RETREATING STALL (closed form): `μ_stall = 1 − √(6·(C_T/σ)/C_Lmax)`, and a more
/// heavily loaded blade (higher C_T/σ) stalls at a LOWER advance ratio.
#[test]
fn retreating_stall_mu_matches_closed_form_and_trend() {
    let cl_max = 1.2;
    let light = retreating_stall_mu(0.06, cl_max);
    let heavy = retreating_stall_mu(0.10, cl_max);
    // Closed form for the light case.
    let expect = 1.0 - (6.0 * 0.06 / cl_max).sqrt();
    assert!((light - expect).abs() < 1e-12);
    assert!(
        heavy < light,
        "heavier blade loading stalls earlier (lower μ)"
    );
    // A blade already stalled in hover (C̄_L ≥ C_Lmax) has μ_stall = 0.
    assert_eq!(retreating_stall_mu(cl_max / 6.0 + 0.01, cl_max), 0.0);
}

/// Vne is the LOWER of the two limits, and `analyze_envelope` selects it correctly.
#[test]
fn vne_is_the_binding_limit() {
    let p = aircraft();
    let lim = limits();
    // Slow tip (90 m/s): advancing Mach is generous; check the min selection holds.
    let env = analyze_envelope(&p, 90.0, 0.07, &lim);
    assert!(
        (env.vne_mps - env.vne_advancing_mach_mps.min(env.vne_retreating_stall_mps)).abs() < 1e-9
    );
    assert!(env.vne_mps <= env.vne_advancing_mach_mps);
    assert!(env.vne_mps <= env.vne_retreating_stall_mps);
}

/// POWER-LIMITED MAX SPEED: at the returned speed, required power equals available
/// power (on the rising branch), and more installed power buys more speed.
#[test]
fn max_level_speed_sits_on_the_power_limit() {
    let p = aircraft();
    let p_avail = 900.0;
    let v_hi = 80.0;
    let v_max = max_level_speed(&p, p_avail, v_hi);
    assert!(v_max > 0.0 && v_max < v_hi, "power-limited, interior");
    assert!(
        (p.forward_shaft_power_w(v_max) - p_avail).abs() < 1.0,
        "P_required ≈ P_available at the max-level speed"
    );
    // More power → faster.
    let v_max2 = max_level_speed(&p, p_avail + 300.0, v_hi);
    assert!(
        v_max2 > v_max,
        "more installed power → higher max level speed"
    );
}

/// HOVER CEILING (closed form): at the limiting density, hover power equals available
/// power; more power → lower limiting density → higher altitude. ISA inversion is a
/// clean round trip at sea level.
#[test]
fn hover_ceiling_density_and_altitude() {
    let p = aircraft();
    let lim = limits();
    let env = analyze_envelope(&p, 100.0, 0.07, &lim);

    // P_hover evaluated at the ceiling density must equal P_available.
    let p_ceiling = AircraftPower {
        rho: env.hover_ceiling_density,
        ..p
    };
    assert!(
        (p_ceiling.hover_shaft_power_w() - lim.power_available_w).abs() < 1e-3,
        "hover power at the ceiling density == available power"
    );

    // More installed power lowers the limiting density (higher ceiling).
    let lim2 = EnvelopeLimits {
        power_available_w: lim.power_available_w + 400.0,
        ..lim
    };
    let env2 = analyze_envelope(&p, 100.0, 0.07, &lim2);
    assert!(env2.hover_ceiling_density < env.hover_ceiling_density);
    assert!(env2.hover_ceiling_altitude_m > env.hover_ceiling_altitude_m);

    // ISA round trip: sea-level density → 0 m altitude.
    assert!(isa_density_altitude_m(1.225).abs() < 1e-6);
    assert!(
        isa_density_altitude_m(1.0) > 0.0,
        "thinner air → positive altitude"
    );
}
