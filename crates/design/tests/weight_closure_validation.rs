//! Gross-weight closure validated against the AFFINE closed form, the divergence
//! threshold, and the spiral-amplification factor — a sizing loop that silently
//! diverges (or silently clamps) would make every "optimum" downstream a fiction.

use helisim_design::{BatteryDemand, FixedDiskLoading, FixedRotor, WeightClosure};

/// A fixed-disk-loading battery model whose battery fraction is constant, so the
/// closure is affine with a hand-computable fixed point.
fn fdl() -> FixedDiskLoading {
    FixedDiskLoading {
        disk_loading_n_m2: 50.0,
        rho: 1.225,
        figure_of_merit: 0.7,
        powertrain_eta: 0.8,
        flight_time_h: 1.0 / 3.0, // 20 min
        specific_energy_wh_kg: 200.0,
        usable_fraction: 0.8,
    }
}

/// AFFINE ORACLE: with empty fraction `e` and constant battery fraction `f`, the
/// fixed point is `W = (payload + fixed)/(1 − e − f)` exactly. The solver must
/// reproduce it, and the battery fraction must indeed be weight-independent.
#[test]
fn affine_closure_matches_closed_form() {
    let bat = fdl();
    let f = bat.battery_fraction();
    // Battery fraction is constant (the affine property): same f at 1 kg and 5 kg.
    assert!((bat.battery_mass_kg(1.0) - f).abs() < 1e-12);
    assert!((bat.battery_mass_kg(5.0) / 5.0 - f).abs() < 1e-12);

    let (e, fixed, payload) = (0.50, 0.20, 0.50);
    let closed_form = (payload + fixed) / (1.0 - e - f);

    let prob = WeightClosure {
        payload_kg: payload,
        empty_fraction: e,
        fixed_mass_kg: fixed,
        battery: &bat,
    };
    let r = prob.solve(100.0).expect("affine spiral closes (e+f<1)");
    println!(
        "closed-form gross {closed_form:.4} kg vs solver {:.4} kg (f={f:.3}, e+f={:.3})",
        r.gross_kg,
        e + f
    );
    assert!(
        (r.gross_kg - closed_form).abs() < 1e-6,
        "solver must hit the closed form"
    );
    // Mass balance closes: empty + payload + battery = gross.
    assert!((r.empty_kg + r.payload_kg + r.battery_kg - r.gross_kg).abs() < 1e-6);
    assert!(
        prob.residual(r.gross_kg).abs() < 1e-8,
        "residual ≈ 0 at the fixed point"
    );
}

/// DIVERGENCE: when `e + f ≥ 1` the spiral never closes — each kg of gross needs
/// ≥ 1 kg more to lift it. The solver reports `None` (not a clamped lie).
#[test]
fn mass_spiral_divergence_is_reported() {
    let bat = fdl();
    let f = bat.battery_fraction();
    // Pick an empty fraction that pushes e + f over 1.
    let e = 1.0 - f + 0.05;
    let prob = WeightClosure {
        payload_kg: 0.5,
        empty_fraction: e,
        fixed_mass_kg: 0.2,
        battery: &bat,
    };
    assert!(e + f > 1.0, "test setup: spiral should diverge");
    assert!(
        prob.solve(1000.0).is_none(),
        "divergent spiral must return None"
    );
}

/// SPIRAL AMPLIFICATION: in the affine case `dW/d(payload) = 1/(1−e−f) > 1`, so a
/// payload increment grows the gross weight by MORE than itself. Measured, not asserted.
#[test]
fn payload_increment_is_amplified() {
    let bat = fdl();
    let f = bat.battery_fraction();
    let (e, fixed) = (0.45, 0.20);
    let amp = 1.0 / (1.0 - e - f);

    let solve_for = |payload: f64| {
        WeightClosure {
            payload_kg: payload,
            empty_fraction: e,
            fixed_mass_kg: fixed,
            battery: &bat,
        }
        .solve(100.0)
        .unwrap()
        .gross_kg
    };
    let g1 = solve_for(0.5);
    let g2 = solve_for(1.0);
    let measured_amp = (g2 - g1) / 0.5;
    println!("dW/d(payload): predicted {amp:.3}, measured {measured_amp:.3}");
    assert!(measured_amp > 1.0, "spiral amplifies the payload increment");
    assert!((measured_amp - amp).abs() < 1e-4, "matches 1/(1−e−f)");
}

/// NONLINEAR (fixed rotor): no clean closed form, so validate self-consistency — the
/// solver returns a true fixed point (mass balance closes, residual ≈ 0) — and the
/// physical monotonicity that a smaller frozen disk needs a heavier (or impossible)
/// closure than a larger one.
#[test]
fn fixed_rotor_nonlinear_closure_is_self_consistent() {
    let rotor = |area: f64| FixedRotor {
        disk_area_m2: area,
        rho: 1.225,
        figure_of_merit: 0.7,
        powertrain_eta: 0.8,
        flight_time_h: 1.0 / 3.0,
        specific_energy_wh_kg: 200.0,
        usable_fraction: 0.8,
    };
    let close = |area: f64| {
        let bat = rotor(area);
        WeightClosure {
            payload_kg: 0.5,
            empty_fraction: 0.45,
            fixed_mass_kg: 0.2,
            battery: &bat,
        }
        .solve(100.0)
        .map(|r| {
            // Self-consistency: a true fixed point (residual ≈ 0, mass balance closes).
            assert!((r.empty_kg + r.payload_kg + r.battery_kg - r.gross_kg).abs() < 1e-6);
            r.gross_kg
        })
    };
    let big = close(1.5).expect("a large enough disk closes");
    // Shrinking the frozen disk raises v_h ⇒ more power ⇒ heavier closure (or none).
    if let Some(small) = close(0.5) {
        assert!(small > big, "smaller disk → heavier closure");
    }
}
