//! Build-instruction **diagrams** as SVG — the things text explains poorly: the
//! blade section, the rotor-head load path, the swashplate/CCPM linkage, and the
//! whole-aircraft layout. Each returns a standalone `.svg` string (see [`crate::svg`]).
//!
//! These are schematics for understanding/assembly, not manufacturing geometry (that
//! is the STL/DXF/STEP). Tests check each is well-formed and carries its key labels.

use crate::blade::BladeSpec;
use crate::svg::Svg;
use helisim_design::{DesignCandidate, DesignReport};

const BLUE: &str = "#1565c0";
const LBLUE: &str = "#cfe8ff";
const RED: &str = "#d32f2f";
const GRAY: &str = "#666";
const METAL: &str = "#b0bec5";
const DARK: &str = "#222";

/// Dimensioned cross-section of the blade root: airfoil, chord, max thickness, the
/// pitch axis (25% chord) and the spanwise spar/fiber region.
pub fn blade_section_svg(blade: &BladeSpec) -> String {
    let contour = blade.section_contour_mm(80);
    let chord = blade.chord_m * 1000.0;
    let thick = blade.max_thickness_m * 1000.0;
    let scale = 320.0 / chord.max(1.0);
    let (ox, oy) = (60.0, 140.0);
    let tx = |x: f64| ox + x * scale;
    let ty = |y: f64| oy - y * scale;

    let mut g = Svg::new(440.0, 250.0);
    // Spar / continuous-fiber region (25–45% chord), drawn under the skin.
    g.rect(
        tx(0.25 * chord),
        ty(0.45 * thick),
        0.20 * chord * scale,
        0.9 * thick * scale,
        "#ffe0b2",
        "#e65100",
    );
    // Airfoil skin.
    let pts: Vec<(f64, f64)> = contour.iter().map(|p| (tx(p.x), ty(p.y))).collect();
    g.poly(&pts, LBLUE, BLUE, true);
    // Chord line + pitch axis.
    g.line(tx(0.0), ty(0.0), tx(chord), ty(0.0), GRAY, 0.6);
    g.circle(tx(0.25 * chord), ty(0.0), 3.5, RED, "#900");
    g.text(
        tx(0.25 * chord),
        ty(0.0) - 10.0,
        "pitch axis (25% c)",
        10.0,
        "middle",
        "#900",
    );
    g.text(
        tx(0.35 * chord),
        ty(-0.7 * thick) + 4.0,
        "spar / fiber",
        10.0,
        "middle",
        "#e65100",
    );
    // Labels + dimensions.
    g.text(tx(0.0) - 8.0, ty(0.0) + 4.0, "LE", 11.0, "end", DARK);
    g.text(tx(chord) + 8.0, ty(0.0) + 4.0, "TE", 11.0, "start", DARK);
    g.dim(
        tx(0.0),
        oy + 60.0,
        tx(chord),
        oy + 60.0,
        &format!("chord {chord:.1} mm"),
    );
    g.dim(
        tx(chord) + 30.0,
        ty(thick * 0.5),
        tx(chord) + 30.0,
        ty(-thick * 0.5),
        &format!("t {thick:.1} mm @30%"),
    );
    g.render(&format!("Blade section — {} (root)", blade.airfoil))
}

/// The rotor-head LOAD PATH (side schematic): blade root → bonded metal tang →
/// retention bolt (the flap/feather pivot) → grip → pitch bearing → pitch horn →
/// pitch link. Shows where the centrifugal force goes.
pub fn rotor_head_svg(bolt_d_mm: f64, grip_bearing: &str) -> String {
    let mut g = Svg::new(560.0, 280.0);
    let cy = 120.0;
    // Mast + hub (left).
    g.rect(40.0, cy - 50.0, 26.0, 120.0, METAL, "#607d8b");
    g.text(53.0, cy + 90.0, "mast / hub", 11.0, "middle", DARK);
    // Grip jaws (clamp the tang) around a pivot bolt.
    g.rect(150.0, cy - 26.0, 90.0, 16.0, "#cfd8dc", "#607d8b");
    g.rect(150.0, cy + 10.0, 90.0, 16.0, "#cfd8dc", "#607d8b");
    g.text(195.0, cy - 34.0, "grip jaws", 11.0, "middle", DARK);
    // Pitch (feather) bearing seat shown as two small circles in the grip.
    g.circle(165.0, cy - 18.0, 5.0, "none", "#607d8b");
    g.circle(165.0, cy + 18.0, 5.0, "none", "#607d8b");
    g.text(
        120.0,
        cy - 50.0,
        &format!("pitch bearings ({grip_bearing})"),
        10.0,
        "middle",
        "#607d8b",
    );
    // Aluminium doubler plates (bonded to the root faces), clamped by the grip.
    g.rect(225.0, cy - 11.0, 110.0, 5.0, METAL, "#455a64");
    g.rect(225.0, cy + 6.0, 110.0, 5.0, METAL, "#455a64");
    g.text(
        280.0,
        cy + 34.0,
        "Al doublers + steel bushing",
        11.0,
        "middle",
        "#455a64",
    );
    // Blade root (printed) continues right.
    g.poly(
        &[
            (335.0, cy - 9.0),
            (520.0, cy - 5.0),
            (520.0, cy + 5.0),
            (335.0, cy + 9.0),
        ],
        LBLUE,
        BLUE,
        true,
    );
    g.text(450.0, cy + 26.0, "printed blade", 11.0, "middle", BLUE);
    // Retention bolt (vertical pin) = flap/feather pivot, through grip + tang.
    g.circle(195.0, cy, 6.0, RED, "#900");
    g.line(195.0, cy - 40.0, 195.0, cy + 40.0, RED, 2.0);
    g.text(
        195.0,
        cy - 46.0,
        &format!("retention bolt M{:.0} = pivot", bolt_d_mm.round()),
        10.0,
        "middle",
        "#900",
    );
    // Pitch horn + link (control input).
    g.line(195.0, cy + 26.0, 215.0, cy + 70.0, DARK, 2.0);
    g.circle(215.0, cy + 70.0, 4.0, "white", DARK);
    g.line(215.0, cy + 70.0, 215.0, cy + 120.0, "#388e3c", 2.0);
    g.text(
        250.0,
        cy + 95.0,
        "pitch link → swashplate",
        10.0,
        "middle",
        "#388e3c",
    );
    g.text(150.0, cy + 70.0, "pitch horn", 10.0, "end", DARK);
    // Centrifugal force arrow (outboard).
    g.arrow(360.0, cy - 40.0, 470.0, cy - 40.0, RED);
    g.text(
        415.0,
        cy - 46.0,
        "centrifugal load (carried by tang + bolt)",
        10.0,
        "middle",
        RED,
    );
    g.render("Rotor head — load path & control (side schematic)")
}

/// The swashplate / CCPM linkage in two panels: TOP (3 servos at 120°, the 90°
/// pitch-lead) and SIDE (collective = plate up, cyclic = plate tilt).
pub fn swashplate_svg(n_blades: usize) -> String {
    let mut g = Svg::new(620.0, 320.0);

    // ---- TOP view (left) ----
    let (cx, cy, r) = (160.0, 170.0, 90.0);
    g.text(160.0, 60.0, "TOP view — 120° CCPM", 13.0, "middle", DARK);
    g.circle(cx, cy, r, "none", BLUE); // swashplate rim
    g.circle(cx, cy, 8.0, METAL, "#607d8b"); // mast
    g.text(cx, cy + 4.0, "mast", 9.0, "middle", DARK);
    // 3 servo pickups at 120° (top, lower-left, lower-right).
    let servo_ang = [-90.0_f64, 30.0, 150.0];
    for (i, a) in servo_ang.iter().enumerate() {
        let (ar, ay) = (a.to_radians().cos(), a.to_radians().sin());
        let (px, py) = (cx + r * ar, cy + r * ay);
        g.circle(px, py, 6.0, "#388e3c", "#1b5e20");
        g.text(
            px + 10.0 * ar,
            py + 10.0 * ay + 3.0,
            &format!("S{}", i + 1),
            11.0,
            "middle",
            "#1b5e20",
        );
    }
    // Rotation arrow + 90° pitch-lead note.
    g.arrow(cx + r + 14.0, cy - 10.0, cx + r + 14.0, cy + 20.0, "#900");
    g.text(cx + r + 30.0, cy + 6.0, "rot", 10.0, "start", "#900");
    g.text(
        cx,
        cy + r + 22.0,
        &format!("{n_blades} pitch links; horn leads blade ~90°",),
        10.0,
        "middle",
        "#900",
    );

    // ---- SIDE view (right) ----
    let (mx, top, bot) = (440.0, 90.0, 250.0);
    g.text(
        460.0,
        60.0,
        "SIDE view — collective vs cyclic",
        13.0,
        "middle",
        DARK,
    );
    g.line(mx, top - 10.0, mx, bot + 10.0, METAL, 6.0); // mast
    g.text(mx, bot + 26.0, "mast", 9.0, "middle", DARK);
    // Rotating (upper) + stationary (lower) plates, bearing between.
    g.rect(mx - 70.0, 150.0, 140.0, 10.0, "#cfd8dc", "#607d8b");
    g.rect(mx - 70.0, 162.0, 140.0, 10.0, "#eeeeee", "#607d8b");
    g.text(mx + 95.0, 156.0, "rotating", 9.0, "start", "#607d8b");
    g.text(mx + 95.0, 170.0, "stationary", 9.0, "start", "#607d8b");
    // Servos below pushing pushrods up to the stationary plate.
    for dx in [-50.0, 0.0, 50.0] {
        g.rect(mx + dx - 8.0, bot - 4.0, 16.0, 16.0, "#388e3c", "#1b5e20");
        g.line(mx + dx, bot - 4.0, mx + dx, 172.0, "#1b5e20", 1.5);
    }
    g.text(
        mx,
        bot + 26.0 + 14.0,
        "3 servos → pushrods",
        10.0,
        "middle",
        "#1b5e20",
    );
    // Pitch links up to the grips from the rotating plate.
    g.line(mx - 60.0, 150.0, mx - 60.0, 110.0, "#388e3c", 1.5);
    g.line(mx + 60.0, 150.0, mx + 60.0, 110.0, "#388e3c", 1.5);
    g.text(mx, 105.0, "pitch links → grips", 10.0, "middle", "#388e3c");
    // Motion annotations.
    g.arrow(mx - 95.0, 165.0, mx - 95.0, 140.0, RED);
    g.text(mx - 95.0, 134.0, "collective (up)", 9.0, "middle", RED);
    g.arrow(mx + 80.0, 175.0, mx + 80.0, 150.0, RED);
    g.text(mx + 80.0, 190.0, "cyclic (tilt)", 9.0, "middle", RED);

    g.render("Swashplate & CCPM control — servos are the actuator")
}

/// Whole-aircraft side elevation with the key dimensions (rotor Ø, height, boom).
pub fn assembly_svg(c: &DesignCandidate, report: &DesignReport) -> String {
    let r_mm = c.radius_m * 1000.0;
    let scale = 360.0 / (2.0 * r_mm).max(1.0);
    let cx = 300.0;
    let disk_y = 70.0;
    let body_y = disk_y + 70.0;
    let half = r_mm * scale;
    let mut g = Svg::new(620.0, 280.0);

    // Rotor disk (edge-on) + mast.
    g.line(cx - half, disk_y, cx + half, disk_y, BLUE, 3.0);
    g.circle(cx, disk_y, 4.0, METAL, "#607d8b");
    g.line(cx, disk_y, cx, body_y, METAL, 5.0);
    g.dim(
        cx - half,
        disk_y - 18.0,
        cx + half,
        disk_y - 18.0,
        &format!("rotor Ø {:.0} mm", 2.0 * r_mm),
    );
    // Fuselage pod.
    let pod_w = 0.5 * half;
    g.rect(cx - 0.5 * pod_w, body_y, pod_w, 40.0, LBLUE, BLUE);
    g.text(cx, body_y + 24.0, "fuselage / pack", 10.0, "middle", DARK);
    // Tail boom + tail rotor.
    let boom_len = 1.1 * half;
    g.line(
        cx + 0.5 * pod_w,
        body_y + 16.0,
        cx + 0.5 * pod_w + boom_len,
        body_y + 16.0,
        "#607d8b",
        4.0,
    );
    g.circle(
        cx + 0.5 * pod_w + boom_len,
        body_y + 16.0,
        16.0,
        "none",
        RED,
    );
    g.text(
        cx + 0.5 * pod_w + boom_len,
        body_y + 50.0,
        "tail rotor",
        10.0,
        "middle",
        RED,
    );
    // Skids.
    g.line(
        cx - 0.6 * pod_w,
        body_y + 60.0,
        cx + 0.6 * pod_w,
        body_y + 60.0,
        DARK,
        3.0,
    );
    g.line(
        cx - 0.3 * pod_w,
        body_y + 40.0,
        cx - 0.4 * pod_w,
        body_y + 60.0,
        DARK,
        2.0,
    );
    g.line(
        cx + 0.3 * pod_w,
        body_y + 40.0,
        cx + 0.4 * pod_w,
        body_y + 60.0,
        DARK,
        2.0,
    );
    g.text(cx, body_y + 76.0, "landing skids", 10.0, "middle", DARK);
    // Spec caption.
    g.text(
        10.0,
        265.0,
        &format!(
            "{} blades · gross {:.1} kg · {:.0} W hover · {:.0} rpm",
            c.n_blades,
            c.gross_mass_kg,
            report.hover_shaft_power_w,
            c.operating().rpm()
        ),
        11.0,
        "start",
        DARK,
    );
    g.render("Aircraft layout (side elevation)")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blade::blade_from_design;
    use helisim_airfoil::LinearAirfoil;
    use helisim_bemt::Config;
    use helisim_design::evaluate;

    fn setup() -> (DesignCandidate, DesignReport) {
        let c = DesignCandidate::model();
        let r = evaluate(&c, &LinearAirfoil::naca0012(), &Config::default());
        (c, r)
    }

    #[test]
    fn all_diagrams_are_well_formed_and_labeled() {
        let (c, r) = setup();
        let blade = blade_from_design(&c, 0.0);
        let docs = [
            (blade_section_svg(&blade), "chord"),
            (rotor_head_svg(3.0, "623"), "pivot"),
            (swashplate_svg(c.n_blades), "CCPM"),
            (assembly_svg(&c, &r), "rotor"),
        ];
        for (svg, needle) in docs {
            assert!(svg.starts_with("<svg"), "starts with <svg");
            assert!(svg.contains("viewBox="), "has viewBox");
            assert!(svg.trim_end().ends_with("</svg>"), "closed");
            assert!(svg.contains(needle), "missing key label '{needle}'");
        }
    }

    #[test]
    fn blade_section_scales_to_the_chord() {
        let (c, _) = setup();
        let blade = blade_from_design(&c, 0.0);
        let svg = blade_section_svg(&blade);
        // The chord dimension label carries the real chord in mm.
        assert!(svg.contains(&format!("chord {:.1} mm", c.chord_m * 1000.0)));
    }
}
