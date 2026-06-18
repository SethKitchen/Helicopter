//! From a recommended design to a **buildable blade**: real dimensions and the
//! step-by-step instructions to MAKE it — by 3D printing, not hand-carving.
//!
//! A twisted, tapered NACA blade is not something you whittle from balsa: the loft
//! geometry is exact math, so the right process is to **print it from the exported
//! solid** ([`crate::lofted_blade_to_stl`] → `blade.stl`) in a structural composite
//! filament (Markforged **Onyx + continuous Fiberglass**, the material the build's
//! control-authority analysis already selects). The retention-bolt hole is PRINTED
//! (an undersized pilot, then reamed — never drilled from solid) and the bolt bears
//! on a BONDED bushing (never a press-fit, which a polymer relaxes out of). How the
//! centrifugal load reaches the bushing is set by the print route ([`RootLoadPath`]):
//! a desktop continuous-fiber blade winds the fiber as a LOOP around the bushing (the
//! load is carried in fiber tension); a larger SLS/molded blade (chopped fiber, no
//! loop) uses bonded aluminium doublers. Only blades too large for FDM (human scale)
//! fall back to a molded
//! carbon spar + skin laid up in a CNC'd mould — still from the exported geometry,
//! still not hand-cut. The geometry/dimensions below feed the slicer (or the mould).

use crate::airfoil_coords::{Point, naca00xx_contour};
use crate::part::{BuildPart, Source};
use helisim_design::DesignCandidate;

/// NACA 0012 thickness fraction (the section the aero stack is built around).
const THICKNESS_FRAC: f64 = 0.12;

/// Representative desktop composite-FDM bed edge, mm (Markforged X7 / Bambu / Prusa
/// class). A blade longer than this is whole-printed by an SLS service instead.
const DEFAULT_BED_MM: f64 = 320.0;
/// Representative large-format SLS print-service bed edge, mm (e.g. EOS / Farsoon via
/// PCBWay / Xometry). A blade longer than this is molded, not printed.
const SERVICE_BED_MM: f64 = 750.0;
/// Retention/feather-bolt diameter, mm — the structural M3 floor (see
/// [`crate::retention_bolt`]); matches the root-fitting and shopping-list bolt.
const ROOT_BOLT_MM: f64 = 3.0;

/// How the blade root reacts the centrifugal force `F_cf` — decided by whether the
/// print route can lay a CONTINUOUS structural fiber (not by preference).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RootLoadPath {
    /// **Fiber-loop** (the textbook composite root): the continuous structural tow
    /// is wound as a LOOP around the retention bushing and both legs run back out
    /// into the spar, so `F_cf` is carried in fiber **tension** (its strongest mode),
    /// bearing on the steel bushing — no bond-shear or plastic-bearing primary path.
    /// Available only on the continuous-fiber desktop route (Markforged Onyx+fiber).
    FiberLoop,
    /// **Bonded doublers**: an SLS (chopped-fiber PA-CF) or molded root cannot loop a
    /// continuous tow, so two bonded 6061 aluminium doubler plates carry `F_cf` into
    /// metal by bond shear + bolt bearing. The metal is the load path; plastic locates.
    BondedDoublers,
}

/// The root load path for a blade of this lifting `span_m`, set by the print route:
/// a span that fits a desktop continuous-fiber bed can wind a [`RootLoadPath::FiberLoop`];
/// anything larger is SLS/molded and uses [`RootLoadPath::BondedDoublers`]. (Matches the
/// route decision in [`blade_from_design_tapered`], so the structural check and the
/// build instructions agree.)
pub fn root_load_path(span_m: f64) -> RootLoadPath {
    if span_m * 1000.0 <= DEFAULT_BED_MM {
        RootLoadPath::FiberLoop
    } else {
        RootLoadPath::BondedDoublers
    }
}

/// A buildable blade specification with real (SI) dimensions.
#[derive(Clone, Debug)]
pub struct BladeSpec {
    /// Airfoil designation.
    pub airfoil: &'static str,
    /// Number of blades to make.
    pub n_blades: usize,
    /// Inboard (root cutout) radius, m.
    pub root_radius_m: f64,
    /// Tip radius, m.
    pub tip_radius_m: f64,
    /// Lifting span (tip − root), m — the length to shape.
    pub span_m: f64,
    /// Root chord, m (the representative chord; equals tip for a rectangular blade).
    pub chord_m: f64,
    /// Tip chord, m (`= chord_m` for a rectangular blade; `< chord_m` if tapered).
    pub tip_chord_m: f64,
    /// Linear twist (washout) root→tip, degrees (applied tip-down for loft/export).
    pub twist_deg: f64,
    /// Maximum section thickness, m (`0.12 · chord`).
    pub max_thickness_m: f64,
    /// Print/blank bounding box per blade `(length, width, thickness)`, mm, with a
    /// small allowance — the envelope the print (or mould blank) must fit.
    pub stock_block_mm: (f64, f64, f64),
    /// Material + construction method (3D-printed composite, or molded at large scale).
    pub method: &'static str,
    /// True if the blade is 3D-printed (whole); false ⇒ molded composite (human scale).
    pub printed: bool,
    /// True if it is too big for a desktop bed and must be whole-printed by an SLS
    /// **print service** (PA-CF nylon); false ⇒ desktop composite (or molded).
    pub service_print: bool,
}

impl BladeSpec {
    /// The dimensional section contour at this blade's chord, in mm — the profile
    /// to cut. `n` points per surface.
    pub fn section_contour_mm(&self, n: usize) -> Vec<Point> {
        naca00xx_contour(THICKNESS_FRAC, n)
            .into_iter()
            .map(|p| Point {
                x: p.x * self.chord_m * 1000.0,
                y: p.y * self.chord_m * 1000.0,
            })
            .collect()
    }

    /// Chord (m) at span fraction `s ∈ [0,1]` (root→tip), linearly interpolated.
    pub fn local_chord_m(&self, s: f64) -> f64 {
        self.chord_m + (self.tip_chord_m - self.chord_m) * s
    }

    /// Geometric twist (deg) at span fraction `s`: linear washout, 0 at the root
    /// to `−twist_deg` at the tip (leading-edge-down toward the tip).
    pub fn local_twist_deg(&self, s: f64) -> f64 {
        -self.twist_deg * s
    }

    /// Whether the blade is tapered or twisted (so a loft is needed, not a simple
    /// extrusion).
    pub fn is_lofted(&self) -> bool {
        (self.tip_chord_m - self.chord_m).abs() > 1e-9 || self.twist_deg.abs() > 1e-9
    }

    /// Step-by-step instructions to MAKE one blade — 3D-printed from the exported
    /// solid (the twist/taper are in the geometry, nothing is hand-shaped), or molded
    /// from a CNC'd mould at human scale.
    pub fn instructions(&self) -> Vec<String> {
        if self.printed {
            self.print_instructions()
        } else {
            self.mold_instructions()
        }
    }

    /// Whole-print workflow — concrete, reality-checked, every referenced part in the
    /// shopping list (§ Hardware & consumables). The blade is printed in ONE PIECE, so
    /// the exported solid `blade.stl` is exactly what is printed: no internal spar
    /// channel to model, and the only post-print machining is drilled/reamed holes. See
    /// `blade_section.svg` and `rotor_head.svg`.
    fn print_instructions(&self) -> Vec<String> {
        let span_mm = self.span_m * 1000.0;
        let shape = if self.is_lofted() {
            let mut s = format!("washout {:.1}°", self.twist_deg.abs());
            if (self.tip_chord_m - self.chord_m).abs() > 1e-9 {
                s.push_str(&format!(", tip chord {:.1} mm", self.tip_chord_m * 1000.0));
            }
            s
        } else {
            "constant section, no twist".to_string()
        };
        // Step 2 differs by route (desktop composite vs SLS service); both are WHOLE.
        let print_step = if self.service_print {
            format!(
                "2. The {span_mm:.0} mm span is bigger than a desktop bed, so order it WHOLE from an \
                 SLS print service in carbon/glass-filled nylon (PA-CF): upload `blade.stl`, get an \
                 instant quote, they ship the finished blade (×{}). The service IS the purchase — see \
                 the SLS-service links in the shopping list. (No splitting, no spar tube — a thin \
                 airfoil cannot take a desktop-split spar; whole-printing avoids it entirely.)",
                self.n_blades
            )
        } else {
            format!(
                "2. Print WHOLE in {} (×{} blades) — the {span_mm:.0} mm span fits a desktop bed. Lay \
                 each blade flat, span along the bed's long axis, so the layers + continuous fiber run \
                 root→tip (centrifugal load along the fiber, not peeling layers). Settings: 4+ solid \
                 walls, ≥40% gyroid infill, continuous fiberglass in the spar to ~80% span, no support \
                 in the airfoil.",
                self.method, self.n_blades
            )
        };
        vec![
            format!(
                "1. The geometry is done for you: the exported solid `blade.stl` (NACA {}, {shape}) \
                 already has the airfoil, taper and twist — NOTHING is hand-shaped. View \
                 `blade_section.svg`; check the profile against `blade_section.dxf`.",
                self.airfoil.trim_start_matches("NACA ")
            ),
            print_step,
            // Step 3 — the retention-bolt hole. The hole is PRINTED (not drilled from
            // solid: drilling a layered/sintered root delaminates it at the worst spot).
            // Print it undersized as a pilot so the walls/fiber route AROUND it, then REAM
            // to size, and BOND (not press) the bushing — a press-fit in a polymer
            // stress-relaxes and loosens over time, so the bond is what retains it.
            format!(
                "3. ROOT HOLE — it is PRINTED, not drilled (see `rotor_head.svg`): the model is \
                 exported with an undersized Ø{:.1} mm pilot at the pitch axis (~25% chord) so the \
                 walls and fiber lay AROUND the hole — never drill a finished root, it delaminates. \
                 REAM the pilot to the Ø{:.1} mm bolt size, then BOND the steel bushing into the \
                 reamed bore with structural epoxy (scuff + degrease + cure) so the bolt bears on \
                 STEEL, not plastic. (Bonded, NOT press-fit: an interference fit in a printed \
                 polymer relaxes and loosens — the epoxy is the retention.)",
                ROOT_BOLT_MM - 1.0,
                ROOT_BOLT_MM
            ),
            match root_load_path(self.span_m) {
                // Desktop continuous-fiber route: wind the structural tow as a LOOP around
                // the bushing — F_cf is then carried in fiber tension, the textbook root.
                RootLoadPath::FiberLoop => {
                    "4. ROOT FIBER LOOP (the centrifugal load path): this desktop route lays \
                     CONTINUOUS fiber, so route the fiberglass tow as a U-turn LOOP that wraps the \
                     bonded bushing and runs both legs back out into the spar (set this in the \
                     slicer's continuous-fiber routing — see `rotor_head.svg`). The centrifugal \
                     pull is then carried in fiber TENSION around the bushing — the strongest \
                     path, with no bond-shear or plastic bearing in the primary load path. The \
                     bolt through the bushing only reacts the loop; the plastic only fairs the \
                     section."
                        .to_string()
                }
                // SLS/molded route: chopped fiber can't loop, so bonded aluminium doublers
                // carry F_cf into metal (the bushing bond keeps the bolt off the plastic).
                RootLoadPath::BondedDoublers => {
                    "4. ROOT DOUBLERS (the centrifugal load path): this route is chopped-fiber \
                     PA-CF (no continuous tow to loop), so cut two aluminium doubler plates from \
                     the 6061 flat bar (listed mini-hacksaw + files, or send the DXF to a \
                     laser/water-jet service — both in the shopping list). Bond one to each face \
                     of the root over the bushing with structural epoxy (scuff + degrease, clamp, \
                     FULL cure); the bolt then clamps doubler+root+doubler. The metal doublers \
                     carry the centrifugal force; the bonded bushing keeps the bolt bearing on \
                     steel; the plastic only locates."
                        .to_string()
                }
            },
            format!(
                "5. BALANCE (mandatory — see the magnetic balancer in the shopping list): mount the \
                 matched set of {} finished blades on the balancer arbor. Let it settle; the heavy \
                 blade sinks. Add tip tape / sand the lighter tips until every blade stays level at \
                 any angle (spanwise), then repeat for leading-vs-trailing (chordwise). Verify max \
                 thickness {:.2} mm @30% chord against the template first. An unbalanced rotor will \
                 shake the aircraft apart.",
                self.n_blades,
                self.max_thickness_m * 1000.0
            ),
        ]
    }

    /// Molded-composite workflow (human scale, beyond FDM).
    fn mold_instructions(&self) -> Vec<String> {
        vec![
            "1. CNC a two-part mould directly from the exported lofted geometry (STEP/STL) — the \
             twist/taper come from the math, not hand-shaping."
                .to_string(),
            format!(
                "2. Lay up a carbon-fibre spar + composite skin to the NACA {} section ({} blades); \
                 vacuum-bag and cure.",
                self.airfoil.trim_start_matches("NACA "),
                self.n_blades
            ),
            format!(
                "3. Bond the metal root fitting / pitch-bearing race into the inboard {:.0} mm (the \
                 centrifugal load path), then dynamically balance the set.",
                self.root_radius_m * 1000.0
            ),
        ]
    }
}

/// Derive a buildable blade spec from a design candidate. `twist_deg` is the
/// linear washout to apply (0 for the untwisted designs the aero stack uses).
/// Rectangular planform (`tip_chord = chord`); use [`blade_from_design_tapered`]
/// for a taper.
pub fn blade_from_design(c: &DesignCandidate, twist_deg: f64) -> BladeSpec {
    blade_from_design_tapered(c, twist_deg, 1.0)
}

/// As [`blade_from_design`] but with a tip-to-root chord `taper_ratio` (1.0 =
/// rectangular, e.g. 0.6 = a 60%-tip taper). The stock block is sized on the root
/// (largest) chord.
pub fn blade_from_design_tapered(
    c: &DesignCandidate,
    twist_deg: f64,
    taper_ratio: f64,
) -> BladeSpec {
    let root_radius = c.root_cutout * c.radius_m;
    let span = c.radius_m - root_radius;
    let max_thickness = THICKNESS_FRAC * c.chord_m;
    // Print/blank envelope: span + 10% length allowance, chord + 20% width, thickness +20%.
    let stock = (
        span * 1000.0 * 1.10,
        c.chord_m * 1000.0 * 1.20,
        max_thickness * 1000.0 * 1.20,
    );
    // The blade is printed WHOLE (the exported solid IS the part — no internal spar
    // channel to model/fake). Three size routes:
    //  • span ≤ desktop bed → whole on a desktop composite printer (Onyx + fiber);
    //  • desktop < span ≤ SLS service → whole, ordered from an SLS service (PA-CF nylon);
    //  • span > SLS service → molded carbon (human scale, beyond printing).
    let span_mm = span * 1000.0;
    let printed = span_mm <= SERVICE_BED_MM;
    let service_print = printed && span_mm > DEFAULT_BED_MM;
    let method = if !printed {
        "molded carbon-fibre spar + composite skin (laid up in a CNC'd mould)"
    } else if service_print {
        "SLS-printed WHOLE in carbon/glass-filled nylon (PA-CF) via a print service"
    } else {
        "3D-printed WHOLE in Markforged Onyx + continuous Fiberglass (fiber spanwise)"
    };
    BladeSpec {
        airfoil: "NACA 0012",
        n_blades: c.n_blades,
        root_radius_m: root_radius,
        tip_radius_m: c.radius_m,
        span_m: span,
        chord_m: c.chord_m,
        tip_chord_m: c.chord_m * taper_ratio,
        twist_deg,
        max_thickness_m: max_thickness,
        stock_block_mm: stock,
        method,
        printed,
        service_print,
    }
}

impl BuildPart for BladeSpec {
    fn name(&self) -> &str {
        "main-rotor blades"
    }
    fn material(&self) -> &str {
        self.method
    }
    fn source(&self) -> Source {
        Source::Fabricated // 3D-printed (or molded at large scale), not cut from stock
    }
    fn key_dimensions_mm(&self) -> Vec<(&'static str, f64)> {
        vec![
            ("span", self.span_m * 1000.0),
            ("chord", self.chord_m * 1000.0),
            ("max thickness", self.max_thickness_m * 1000.0),
        ]
    }
    fn build_steps(&self) -> Vec<String> {
        self.instructions()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> BladeSpec {
        blade_from_design(&DesignCandidate::model(), 0.0)
    }

    #[test]
    fn dimensions_are_consistent_with_the_design() {
        let c = DesignCandidate::model();
        let s = spec();
        assert!((s.tip_radius_m - c.radius_m).abs() < 1e-12);
        assert!((s.root_radius_m - c.root_cutout * c.radius_m).abs() < 1e-12);
        assert!((s.span_m - (c.radius_m - c.root_cutout * c.radius_m)).abs() < 1e-12);
        assert!((s.max_thickness_m - 0.12 * c.chord_m).abs() < 1e-12);
        assert_eq!(s.n_blades, c.n_blades);
    }

    #[test]
    fn stock_block_has_machining_allowance() {
        let s = spec();
        // Stock must be at least as big as the finished part.
        assert!(s.stock_block_mm.0 >= s.span_m * 1000.0);
        assert!(s.stock_block_mm.1 >= s.chord_m * 1000.0);
        assert!(s.stock_block_mm.2 >= s.max_thickness_m * 1000.0);
    }

    #[test]
    fn section_contour_scales_to_chord() {
        let s = spec();
        let contour = s.section_contour_mm(60);
        // Max |y| of the dimensional contour is the half-thickness in mm.
        let max_y = contour.iter().map(|p| p.y.abs()).fold(0.0_f64, f64::max);
        assert!((max_y - 0.06 * s.chord_m * 1000.0).abs() < 0.05);
        // Chordwise extent equals the chord (mm).
        let max_x = contour.iter().map(|p| p.x).fold(0.0_f64, f64::max);
        assert!((max_x - s.chord_m * 1000.0).abs() < 0.5);
    }

    #[test]
    fn model_blade_is_printed_from_the_exported_geometry() {
        let s = spec();
        assert!(s.printed, "a model-scale blade is 3D-printed, not hand-cut");
        assert_eq!(s.source(), Source::Fabricated);
        assert!(s.material().contains("printed"));
        let steps = s.instructions();
        assert!(steps.len() >= 5);
        // Step 1 points at the exported STL — nothing is hand-shaped.
        assert!(
            steps[0].contains("blade.stl"),
            "step 1 should reference the exported solid"
        );
        // The centrifugal load path is a steel bushing + aluminium doublers, not plastic.
        assert!(
            steps.iter().any(|s| s.contains("bushing")),
            "names the steel bushing"
        );
        assert!(
            steps.iter().any(|s| s.contains("doubler")),
            "names the metal doublers"
        );
        // The hole is PRINTED + reamed and the bushing is BONDED — never drilled, never
        // press-fit (a polymer interference fit relaxes; drilling delaminates).
        let hole = steps
            .iter()
            .find(|s| s.contains("ROOT HOLE"))
            .expect("a root-hole step");
        assert!(
            hole.contains("PRINTED") && hole.contains("REAM"),
            "print + ream, not drill"
        );
        assert!(
            hole.contains("BOND") && hole.contains("NOT press-fit"),
            "bonded, not pressed"
        );
        // The balance step explains how to use the balancer.
        assert!(
            steps.iter().any(|s| s.contains("balancer arbor")),
            "balancer how-to"
        );
    }

    #[test]
    fn a_small_blade_uses_a_fiber_loop_root() {
        // A short-span blade prints on a desktop continuous-fiber bed → the root is a
        // fiber LOOP (tension), not bonded doublers.
        let mut c = DesignCandidate::model();
        c.radius_m = 0.25;
        let s = blade_from_design(&c, 0.0);
        assert_eq!(root_load_path(s.span_m), RootLoadPath::FiberLoop);
        let steps = s.instructions();
        assert!(
            steps.iter().any(|s| s.contains("FIBER LOOP")),
            "names the fiber-loop root"
        );
        assert!(
            steps.iter().any(|s| s.contains("TENSION")),
            "F_cf carried in fiber tension"
        );
        // The hole is still printed + bonded (route-independent fix).
        assert!(
            steps
                .iter()
                .any(|s| s.contains("ROOT HOLE") && s.contains("BOND"))
        );
    }

    #[test]
    fn human_scale_blade_is_molded_not_printed() {
        // A 5 m-radius blade is beyond FDM → molded composite (still from the geometry).
        let mut c = DesignCandidate::model();
        c.radius_m = 5.0;
        c.root_cutout = 0.2;
        let s = blade_from_design(&c, 0.0);
        assert!(!s.printed, "a 5 m blade cannot be FDM-printed");
        assert!(s.material().contains("molded"));
        assert!(s.instructions()[0].contains("mould"));
    }

    #[test]
    fn tapered_twisted_blade_lofts_and_emits_washout_step() {
        // A 0.6-taper, 8°-washout blade exercises the loft helpers and the twist
        // branch of the build instructions.
        let b = blade_from_design_tapered(&DesignCandidate::model(), 8.0, 0.6);
        assert!(b.is_lofted());
        assert!((b.tip_chord_m - 0.6 * b.chord_m).abs() < 1e-12);
        // Linear chord/twist interpolation: midspan is the mean.
        assert!((b.local_chord_m(0.5) - 0.5 * (b.chord_m + b.tip_chord_m)).abs() < 1e-12);
        assert!((b.local_twist_deg(1.0) + 8.0).abs() < 1e-12); // tip = −washout
        assert!((b.local_twist_deg(0.0)).abs() < 1e-12); // root = 0
        assert!(b.instructions().iter().any(|s| s.contains("washout")));
        // A rectangular untwisted blade is not lofted.
        assert!(!blade_from_design(&DesignCandidate::model(), 0.0).is_lofted());
    }
}
