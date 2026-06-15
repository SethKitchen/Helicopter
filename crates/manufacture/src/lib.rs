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
pub mod build_volume;
pub mod export;
pub mod fasteners;
pub mod fea_structural;
pub mod fuselage;
pub mod hub;
pub mod joint_structural;
pub mod landing_gear;
pub mod mast;
pub mod materials;
pub mod mesh;
pub mod mount;
pub mod naca_section;
pub mod part;
pub mod print_plan;
pub mod root_fitting;
pub mod sizing;
pub mod split;
pub mod split_geometry;
pub mod step_brep;
pub mod structural;
pub mod swashplate;
pub mod tail_rotor;

pub use airfoil_coords::{Point, naca00xx_contour, naca4_half_thickness};
pub use assembly::{BuildPackage, build_package};
pub use assembly_export::{
    aircraft_parts, aircraft_to_step, aircraft_to_step_ap203, aircraft_to_stl,
};
pub use blade::{BladeSpec, blade_from_design, blade_from_design_tapered};
pub use boom::{BoomSpec, boom_for};
pub use build_volume::{
    BuildVolume, build_volumes, cnc_envelope, eos_sls_pa12, hp_mjf_4200, markforged_x7, onyx_pro,
    smallest_fitting,
};
pub use export::{
    airfoil_to_dxf, blade_to_stl, lofted_blade_to_stl, lofted_facet_count, stl_facet_count,
};
pub use fasteners::{
    Bearing, Bolt, HardwareItem, bearing_catalogue, bolt_catalogue, hardware_schedule,
    select_bearing, select_bolt,
};
pub use fea_structural::{FeaPart, FeaReport, naca0012_flap_inertia, run_fea};
pub use fuselage::{FuselageSpec, fuselage_for};
pub use hub::{HubSpec, hub_from_blade};
pub use joint_structural::{BladeJointEffect, blade_joint_effect};
pub use landing_gear::{LandingGearSpec, landing_gear_for};
pub use mast::{MastSpec, mast_for_torque};
pub use mount::{MountSpec, mount_for};
pub use part::{BuildPart, Source};
pub use print_plan::{joint_load_for, largest_part_bbox, plan_prints, recommend_printer};
pub use root_fitting::{RootFitting, root_fitting_for};
pub use split::{Joint, SplitPlan, plan_split};
pub use split_geometry::{
    annular_boss, blade_piece_tris, blade_splice_plate, blade_split_meshes, splice_plate,
};
pub use step_brep::{
    assembly_to_step_ap203, blade_to_step_brep, is_closed_manifold, is_oriented_manifold,
    mesh_to_step_brep, mesh_topology,
};
pub use structural::{MarginItem, StructuralReport, check_structure};
pub use swashplate::{SwashplateSpec, swashplate_for};
pub use tail_rotor::{TailRotorSpec, tail_rotor_for};
