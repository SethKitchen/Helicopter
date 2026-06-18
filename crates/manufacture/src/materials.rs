//! Allowable-stress constants for sizing structural parts (documented, overridable
//! in spirit). Values are conservative working stresses (yield ÷ a safety factor),
//! not handbook ultimates — sizing here is a first cut, to be checked before flight.

/// Young's modulus of 6061-T6 aluminium, Pa (boom/mast deflection + resonance).
pub const E_AL: f64 = 69.0e9;
/// Young's modulus of a pultruded carbon-fibre tube, Pa, and its density, kg/m³ —
/// the stiffer/lighter boom material when an aluminium tube can't clear resonance.
pub const E_CF_TUBE: f64 = 110.0e9;
pub const RHO_CF_TUBE: f64 = 1600.0;
/// Density of 6061-T6 aluminium, kg/m³.
pub const RHO_AL: f64 = 2700.0;

/// Allowable shear stress for a 6061-T6 aluminium shaft, Pa. 6061-T6 shear yield
/// ≈ 165 MPa (MMPDS / ASM 6061-T6 data); with a safety factor of ~3 → ~55 MPa
/// working. Used for mast torsion.
pub const TAU_ALLOW_AL: f64 = 55.0e6;

/// Allowable bending stress for a 6061-T6 aluminium tube, Pa. 6061-T6 tensile
/// yield ≈ 276 MPa (MMPDS / ASM); ÷ 3 safety factor → ~90 MPa working.
pub const SIGMA_ALLOW_AL: f64 = 90.0e6;

/// Allowable BEARING stress for 6061-T6 (a bolt bearing on a hole), Pa. Bearing
/// yield ≈ 1.5× tensile yield (MMPDS bearing factors); take ~1.5×SIGMA_ALLOW_AL.
pub const SIGMA_BEARING_AL: f64 = 135.0e6;

/// Working lap-shear allowable for a structural 2-part epoxy bond, Pa. J-B Weld /
/// Loctite-EA lap-shear ultimate is ~10–20 MPa on prepared faces; with a ~2.5 safety
/// factor and a derate for a printed-plastic faying surface → ~6 MPa working. This is
/// the allowable for the blade-root doubler bond (the as-built load path).
pub const TAU_ALLOW_EPOXY: f64 = 6.0e6;

/// Safety factor applied to the Markforged continuous-Fiberglass datasheet ultimates
/// for the fiber-loop root: the ASTM coupons are unidirectional 0° plies, while the
/// as-printed loop has bends, off-axis plies and layer interfaces — a ~6× knockdown
/// is a conservative first cut (overridable in spirit).
pub const CFF_SAFETY_FACTOR: f64 = 6.0;
/// Working TENSILE allowable for a Markforged continuous-Fiberglass tow, Pa — the
/// load path of the fiber-loop root (the fiber wraps the bushing and carries the
/// centrifugal force in tension). SOURCED: Markforged Composites Datasheet REV 5.0
/// (08/01/2021), Continuous Fiber / Fiberglass tensile strength **590 MPa** (ASTM
/// D3039); ÷ [`CFF_SAFETY_FACTOR`] → ~98 MPa working. Far stronger in tension than the
/// ~40 MPa whole-blade laminate value, which is why the loop is the better root.
/// <https://static.markforged.com/downloads/composites-data-sheet.pdf>
pub const SIGMA_ALLOW_CFF_GLASS: f64 = 590.0e6 / CFF_SAFETY_FACTOR;
/// Working COMPRESSIVE allowable for Markforged Fiberglass, Pa — the allowable where
/// the fiber loop BEARS on the steel bushing (a compression, not a tension). SOURCED:
/// same datasheet, Fiberglass compressive strength **180 MPa** (ASTM D6641); ÷
/// [`CFF_SAFETY_FACTOR`] → ~30 MPa working.
pub const SIGMA_COMPR_CFF_GLASS: f64 = 180.0e6 / CFF_SAFETY_FACTOR;

/// Young's modulus of printed structural materials, Pa (for the as-built blade FEA):
/// SLS carbon/glass-filled nylon (PA-CF) ≈ 4 GPa; Markforged Onyx + continuous
/// Fiberglass ≈ 22 GPa (Markforged composites datasheet); a molded carbon laminate
/// ≈ 30 GPa. The blade is NOT a 30 GPa solid unless it is molded — using the right
/// modulus (and the infill knockdown) is what makes the deflection honest.
pub const E_SLS_PA_CF: f64 = 4.0e9;
pub const E_ONYX_FIBERGLASS: f64 = 22.0e9;
pub const E_MOLDED_CARBON: f64 = 30.0e9;
