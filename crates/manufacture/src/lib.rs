//! **Manufacture** — from a recommended design to a complete, buildable machine.
//!
//! The end goal of the project is step-by-step build instructions ("get a block
//! of this size and cut it into this shape"). This crate turns a
//! [`helisim_design::DesignCandidate`] (plus its evaluated report) into the full
//! set of physically-sized parts, the order to assemble them, and exportable
//! geometry files to print or cut.
//!
//! Every part is sized as a *consequence* of the design — the mast diameter from
//! the torsion of the actual hover torque, the tail boom from the bending of the
//! main torque, the grips from the blade root — not guessed. Each implements the
//! [`BuildPart`] trait (the polymorphism boundary), so a complete build is a list
//! of parts plus an assembly sequence ([`BuildPackage`]).
//!
//! Geometry is exact math, so the tests are geometric/engineering oracles
//! (published NACA ordinates, the torsion/bending stress limits, well-formed
//! STL/DXF), never fabricated numbers.
//!
//! One concept per module:
//! * [`part`]          — the [`BuildPart`] trait + [`part::Source`].
//! * [`materials`]     — allowable-stress constants for sizing.
//! * [`airfoil_coords`]— exact NACA 4-digit section coordinates.
//! * [`blade`]         — the rotor [`BladeSpec`].
//! * [`hub`]           — hub + blade grips.
//! * [`mast`]          — drive mast, torsion-sized.
//! * [`swashplate`]    — the control swashplate.
//! * [`boom`]          — tail boom, bending-sized.
//! * [`mount`]         — powertrain tray.
//! * [`assembly`]      — [`BuildPackage`]: all parts + the assembly sequence.
//! * [`export`]        — STL (print) and DXF (cut) geometry files.

pub mod airfoil_coords;
pub mod assembly;
pub mod assembly_export;
pub mod blade;
pub mod boom;
pub mod export;
pub mod fasteners;
pub mod fea_structural;
pub mod fuselage;
pub mod hub;
pub mod mesh;
pub mod mast;
pub mod materials;
pub mod mount;
pub mod part;
pub mod root_fitting;
pub mod step_brep;
pub mod structural;
pub mod swashplate;
pub mod tail_rotor;

pub use airfoil_coords::{naca00xx_contour, naca4_half_thickness, Point};
pub use assembly::{build_package, BuildPackage};
pub use assembly_export::{aircraft_parts, aircraft_to_step, aircraft_to_step_ap203, aircraft_to_stl};
pub use blade::{blade_from_design, blade_from_design_tapered, BladeSpec};
pub use boom::{boom_for, BoomSpec};
pub use fasteners::{
    bearing_catalogue, bolt_catalogue, hardware_schedule, select_bearing, select_bolt, Bearing,
    Bolt, HardwareItem,
};
pub use fea_structural::{naca0012_flap_inertia, run_fea, FeaPart, FeaReport};
pub use fuselage::{fuselage_for, FuselageSpec};
pub use export::{
    airfoil_to_dxf, blade_to_stl, lofted_blade_to_stl, lofted_facet_count, stl_facet_count,
};
pub use root_fitting::{root_fitting_for, RootFitting};
pub use step_brep::{
    assembly_to_step_ap203, blade_to_step_brep, is_closed_manifold, mesh_to_step_brep,
    mesh_topology,
};
pub use structural::{check_structure, MarginItem, StructuralReport};
pub use tail_rotor::{tail_rotor_for, TailRotorSpec};
pub use hub::{hub_from_blade, HubSpec};
pub use mast::{mast_for_torque, MastSpec};
pub use mount::{mount_for, MountSpec};
pub use part::{BuildPart, Source};
pub use swashplate::{swashplate_for, SwashplateSpec};
