//! Cost + buildability validation.
//!
//! This is a parametric accounting model, not physics — so the checks are
//! consistency, monotonicity, and that the buildability taxonomy lands the right
//! parts in the irreducible buy-list. Absolute costs depend on the named
//! [`UnitCosts`] inputs (representative defaults, override with quotes); only the
//! structure of the result is asserted.

use helisim_cost::{AircraftSpec, Buildability, UnitCosts, build_bom, summarize};

fn model_spec() -> AircraftSpec {
    // The ~3.5 kg model design point, with a plausible mass split.
    AircraftSpec {
        n_blades: 2,
        blade_mass_kg: 0.06,
        hub_mass_kg: 0.15,
        structure_mass_kg: 0.8,
        powertrain_mass_kg: 0.4,
        motor_power_kw: 0.35,
        pack_energy_wh: 120.0,
        pack_mass_kg: 0.7,
    }
}

#[test]
fn subsystem_costs_sum_to_total_and_fractions_are_bounded() {
    let r = summarize(&build_bom(&model_spec(), &UnitCosts::default()));
    let subsystem_sum: f64 = r.by_subsystem.iter().map(|(_, c)| c).sum();
    assert!((subsystem_sum - r.total_cost).abs() < 1e-9);
    assert!((0.0..=1.0).contains(&r.vertical_integration_index));
    assert!((0.0..=1.0).contains(&r.purchased_cost_fraction));
    assert!(r.total_cost > 0.0 && r.total_mass_kg > 0.0);
}

#[test]
fn cells_esc_and_sensors_are_the_irreducible_buy_items() {
    let r = summarize(&build_bom(&model_spec(), &UnitCosts::default()));
    let names: Vec<&str> = r.buy_items.iter().map(|(n, _)| *n).collect();
    assert!(names.iter().any(|n| n.contains("cells")));
    assert!(names.iter().any(|n| n.contains("ESC")));
    assert!(
        names
            .iter()
            .any(|n| n.contains("controller") || n.contains("sensors"))
    );
    // Purchased cost is a real, non-trivial share — you cannot self-build it away.
    assert!(r.purchased_cost > 0.0 && r.purchased_cost_fraction > 0.1);
}

#[test]
fn a_bigger_pack_costs_more_and_lowers_self_build_index() {
    let base = model_spec();
    let mut big = base;
    big.pack_energy_wh = 400.0; // triple the energy
    big.pack_mass_kg = 2.3;

    let r0 = summarize(&build_bom(&base, &UnitCosts::default()));
    let r1 = summarize(&build_bom(&big, &UnitCosts::default()));
    // More (purchased) cells → higher total cost...
    assert!(r1.total_cost > r0.total_cost);
    // ...and a larger purchased share drags the vertical-integration index down.
    assert!(r1.vertical_integration_index < r0.vertical_integration_index);
}

#[test]
fn self_fraction_taxonomy_is_ordered() {
    // Raw stock is the most self-buildable, purchased the least.
    assert!(Buildability::RawStock.self_fraction() > Buildability::Fabricated.self_fraction());
    assert!(Buildability::Fabricated.self_fraction() > Buildability::Assembled.self_fraction());
    assert!(Buildability::Assembled.self_fraction() > Buildability::Purchased.self_fraction());
    assert_eq!(Buildability::Purchased.self_fraction(), 0.0);
}

#[test]
fn doubling_unit_costs_doubles_total_but_not_the_index() {
    // The vertical-integration index is a cost-weighted ratio, so a uniform price
    // scaling leaves it unchanged — a good invariant for a relative metric.
    let spec = model_spec();
    let base = UnitCosts::default();
    let doubled = UnitCosts {
        structure_per_kg: base.structure_per_kg * 2.0,
        blade_per_kg: base.blade_per_kg * 2.0,
        fabrication_per_kg: base.fabrication_per_kg * 2.0,
        motor_per_kw: base.motor_per_kw * 2.0,
        esc_per_kw: base.esc_per_kw * 2.0,
        cell_per_wh: base.cell_per_wh * 2.0,
        pack_assembly_per_wh: base.pack_assembly_per_wh * 2.0,
        avionics_flat: base.avionics_flat * 2.0,
    };
    let r0 = summarize(&build_bom(&spec, &base));
    let r1 = summarize(&build_bom(&spec, &doubled));
    assert!((r1.total_cost - 2.0 * r0.total_cost).abs() < 1e-6);
    assert!((r1.vertical_integration_index - r0.vertical_integration_index).abs() < 1e-9);
}
