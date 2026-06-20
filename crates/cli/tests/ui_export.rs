//! Smoke test for the design-studio data export.
//!
//! The `ui` command serves a static frontend (blocking), so it can't run in the
//! `dispatch` smoke loop. The testable part is the data bundle: building it must
//! succeed for the recommended design and produce well-formed JSON with the parts
//! the UI relies on. (Structure check, not an oracle — the numbers are the
//! already-validated `build` results; here we only confirm they serialize.)

use helisim_cli::ui_export::build_bundle;

#[test]
fn bundle_builds_and_is_well_formed() {
    let bundle = build_bundle().expect("recommended design should produce a studio bundle");

    // Geometry: a JSON object with a non-empty "parts" array and mm units.
    let g = &bundle.geometry;
    assert!(
        g.trim_start().starts_with('{'),
        "geometry must be a JSON object"
    );
    assert!(g.contains("\"units\":\"mm\""), "geometry declares mm units");
    assert!(
        g.contains("\"positions\":["),
        "geometry carries vertex positions"
    );
    assert!(
        g.contains("\"normals\":["),
        "geometry carries vertex normals"
    );
    // The studio needs every aircraft part represented, not just the old four
    // assembly solids.
    for grp in [
        "fuselage",
        "mast",
        "blade",
        "boom",
        "landing_gear",
        "tail_rotor",
        "tail_fin",
        "battery",
        "motor",
        "esc",
        "avionics",
        "hub",
    ] {
        assert!(
            g.contains(&format!("\"group\":\"{grp}\"")),
            "geometry should contain the {grp} group"
        );
    }

    // Manifest: design summary + parts + structure + assembly present.
    let m = &bundle.manifest;
    assert!(
        m.trim_start().starts_with('{'),
        "manifest must be a JSON object"
    );
    for key in [
        "\"design\"",
        "\"figure_of_merit\"",
        "\"parts\"",
        "\"structure\"",
        "\"assembly\"",
        "\"fea\"",
        "\"balance\"",
        "\"cg_offset_m\"",
        "\"trim_pitch_deg\"",
        "\"inertia_kg_m2\"",
        "\"stability\"",
        "\"avionics_effect\"",
        "\"cfd\"",
        "\"optimality\"",
        "\"tutorials\"",
        "\"ream\"",
        "\"bond\"",
    ] {
        assert!(m.contains(key), "manifest should contain {key}");
    }
}
