//! Allowable-stress constants for sizing structural parts (documented, overridable
//! in spirit). Values are conservative working stresses (yield ÷ a safety factor),
//! not handbook ultimates — sizing here is a first cut, to be checked before flight.

/// Allowable shear stress for a 6061-T6 aluminium shaft, Pa. 6061-T6 shear yield
/// ≈ 165 MPa (MMPDS / ASM 6061-T6 data); with a safety factor of ~3 → ~55 MPa
/// working. Used for mast torsion.
pub const TAU_ALLOW_AL: f64 = 55.0e6;

/// Allowable bending stress for a 6061-T6 aluminium tube, Pa. 6061-T6 tensile
/// yield ≈ 276 MPa (MMPDS / ASM); ÷ 3 safety factor → ~90 MPa working.
pub const SIGMA_ALLOW_AL: f64 = 90.0e6;
