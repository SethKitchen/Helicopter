//! Split an oversized part to fit a build volume and **bolt** the pieces back
//! together with a generated splice.
//!
//! When a part exceeds the printer ([`crate::build_volume`]) it is cut into pieces
//! that fit. Every cut becomes a **bolted splice** — concrete, realistic, and
//! fully auto-produced: the splice plate/sleeve is generated as real watertight
//! geometry ([`crate::split_geometry`]) and the bolt is sized for the load at that
//! section by [`crate::fasteners::select_bolt`]. Bolts (not snap-fits, which a
//! kernel-free toolkit can't make reliably, and not vague adhesive) carry every
//! joint: a light shell seam gets the smallest bolt (M2) through a flange, a rotor
//! or landing load gets a bigger bolt through a spar/sleeve splice. The form of
//! the splice is realistic per part — a spar plate for the blade, a sleeve for the
//! tube boom, bolting flanges for the shell.

use crate::build_volume::BuildVolume;
use crate::fasteners::select_bolt;
use crate::part::BuildPart;

/// Bolts per bolted joint (a splice needs at least two to react moment).
pub const BOLTS_PER_JOINT: usize = 2;

/// The realistic form a splice takes for a given part — drives the join wording.
#[derive(Clone, Copy, Debug)]
enum JointForm {
    /// Blade: a bolted spar splice plate along the ¼-chord line.
    Blade,
    /// Tube boom: a bolted sleeve over the butted ends.
    Tube,
    /// Shell/pod: bolted flanges at the seam + sealant.
    Shell,
    /// Anything else: a bolted splice plate.
    Generic,
}

/// How a split joint is fastened — always bolted (auto-generated splice), or the
/// rare case where the load outruns the bolt catalogue.
#[derive(Clone, Debug, PartialEq)]
pub enum Joint {
    /// No split — one piece, no joint.
    None,
    /// Bolted splice (a generated plate/sleeve): bolt size + total bolt count.
    Bolted { size: String, count: usize },
    /// Load exceeds the largest catalogue bolt — needs a redesign / metal insert.
    Overloaded,
}

impl Joint {
    pub fn label(&self) -> String {
        match self {
            Joint::None => "—".to_string(),
            Joint::Bolted { size, count } => format!("{count}× {size} bolts + splice"),
            Joint::Overloaded => "BOLT > M10 (redesign / metal insert)".to_string(),
        }
    }
}

/// The plan to make one part on a given build volume.
#[derive(Clone, Debug)]
pub struct SplitPlan {
    pub part: String,
    pub bbox_mm: (f64, f64, f64),
    pub volume: &'static str,
    /// Fits in one piece?
    pub fits: bool,
    /// Number of pieces to print.
    pub pieces: usize,
    /// Number of joints (`pieces − 1`).
    pub joints: usize,
    /// The internal load each joint must carry, N.
    pub joint_load_n: f64,
    /// How the joints are fastened.
    pub joint: Joint,
}

/// Plan how to make `part` on `vol`, given the load `joint_load_n` a split joint
/// would carry. Every split joint is bolted, the bolt sized for the load (the
/// smallest catalogue bolt, M2, for a light seam).
pub fn plan_split(part: &dyn BuildPart, vol: &BuildVolume, joint_load_n: f64) -> SplitPlan {
    let bbox = part.bounding_box_mm();
    let fits = vol.fits(bbox);
    let pieces = vol.pieces_needed(bbox);
    let joints = pieces.saturating_sub(1);

    let joint = if joints == 0 {
        Joint::None
    } else {
        // Bolted: BOLTS_PER_JOINT share the load in double shear, SF 2.
        let per_bolt = joint_load_n / BOLTS_PER_JOINT as f64;
        match select_bolt(per_bolt, true, 2.0) {
            Some(b) => Joint::Bolted {
                size: b.name.to_string(),
                count: BOLTS_PER_JOINT * joints,
            },
            None => Joint::Overloaded,
        }
    };

    SplitPlan {
        part: part.name().to_string(),
        bbox_mm: bbox,
        volume: vol.name,
        fits,
        pieces,
        joints,
        joint_load_n,
        joint,
    }
}

impl SplitPlan {
    /// Step-by-step instructions to JOIN the printed pieces back into the part —
    /// woven into the build output so "how do I assemble the pieces" is answered.
    /// Empty when the part prints whole.
    pub fn join_instructions(&self) -> Vec<String> {
        if self.pieces <= 1 {
            return Vec::new();
        }
        let name = self.part.to_lowercase();
        let kind = if name.contains("blade") {
            JointForm::Blade
        } else if name.contains("boom") {
            JointForm::Tube
        } else if name.contains("fuselage") || name.contains("pod") {
            JointForm::Shell
        } else {
            JointForm::Generic
        };
        let mut steps = vec![format!(
            "Print {} pieces (exceeds the {} bed); the printed locating pins/sockets on each cut \
             face register the mating pieces.",
            self.pieces, self.volume
        )];
        match &self.joint {
            Joint::Bolted { size, count } => {
                let per_joint = count / self.joints.max(1);
                let detail = match kind {
                    JointForm::Blade => format!(
                        "Butt the blade pieces; fit the generated spar splice plate (splice_plate.stl) \
                         along the ¼-chord spar line and bolt through both halves with {per_joint}× \
                         {size} bolts per joint — it carries the ~{:.0} N centrifugal tension. \
                         Thread-lock and torque.",
                        self.joint_load_n
                    ),
                    JointForm::Tube => format!(
                        "Slide the generated splice sleeve over the butted tube ends and bolt through \
                         both with {per_joint}× {size} bolts per joint (~{:.0} N). Thread-lock and torque.",
                        self.joint_load_n
                    ),
                    JointForm::Shell => format!(
                        "Each piece has a printed bolting flange at the seam; align on the locating pins \
                         and bolt the flanges with {per_joint}× {size} bolts per joint (~{:.0} N), with \
                         a sealant bead for weatherproofing.",
                        self.joint_load_n
                    ),
                    JointForm::Generic => format!(
                        "Fit the generated splice plate across each joint and bolt with {per_joint}× \
                         {size} bolts per joint (~{:.0} N). Thread-lock and torque.",
                        self.joint_load_n
                    ),
                };
                steps.push(detail);
                if matches!(kind, JointForm::Blade) {
                    steps.push(
                        "Re-balance the blade spanwise & chordwise against the others and re-track \
                         after joining — the joint shifts mass."
                            .to_string(),
                    );
                }
            }
            Joint::Overloaded => steps.push(
                "Joint load exceeds the bolt catalogue — redesign with a metal insert / larger \
                 splice, or print whole on a larger bed."
                    .to_string(),
            ),
            Joint::None => {}
        }
        steps.push("Check straightness/alignment across the joint before assembly.".to_string());
        steps
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::build_volume::onyx_pro;
    use crate::fuselage::fuselage_for;

    /// A part that fits → one piece, no joint.
    #[test]
    fn fitting_part_has_no_joint() {
        // A small pod that fits the Onyx Pro.
        let pod = fuselage_for(0.3, 0.2); // tiny
        let plan = plan_split(&pod, &onyx_pro(), 100.0);
        if plan.fits {
            assert_eq!(plan.pieces, 1);
            assert_eq!(plan.joint, Joint::None);
        }
    }

    /// Every split joint is bolted; a light load gets the smallest bolt (M2), a
    /// heavy load a bigger one — both fully specified, no vague/manual step.
    #[test]
    fn every_split_joint_is_bolted_sized_by_load() {
        let pod = fuselage_for(700.0, 4.0); // big → exceeds Onyx Pro → splits
        let light = plan_split(&pod, &onyx_pro(), 30.0);
        assert!(light.pieces > 1);
        match &light.joint {
            Joint::Bolted { size, .. } => assert_eq!(size, "M2", "light seam → smallest bolt"),
            other => panic!("expected bolted, got {other:?}"),
        }
        let heavy = plan_split(&pod, &onyx_pro(), 5000.0);
        match (&light.joint, &heavy.joint) {
            (Joint::Bolted { size: ls, .. }, Joint::Bolted { size: hs, .. }) => {
                assert!(hs != ls, "heavier load → bigger bolt ({hs} vs {ls})")
            }
            _ => panic!("both should be bolted"),
        }
    }

    /// Split parts get join instructions; whole parts get none.
    #[test]
    fn join_instructions_present_only_when_split() {
        let big = fuselage_for(700.0, 4.0);
        let split = plan_split(&big, &onyx_pro(), 800.0);
        assert!(split.pieces > 1);
        let steps = split.join_instructions();
        assert!(!steps.is_empty());
        assert!(
            steps.iter().any(|s| s.contains("bolts")),
            "bolted joint mentions bolts"
        );

        let small = fuselage_for(0.3, 0.2);
        if small.bounding_box_mm().0 <= onyx_pro().longest_mm() {
            assert!(
                plan_split(&small, &onyx_pro(), 10.0)
                    .join_instructions()
                    .is_empty()
            );
        }
    }

    /// Part-aware splice wording: the blade joint reads as a spar splice, the
    /// boom as a sleeve — concrete, realistic ways the pieces go together.
    #[test]
    fn join_wording_is_part_specific() {
        use crate::blade::blade_from_design;
        use crate::boom::boom_for;
        use helisim_design::DesignCandidate;
        let c = DesignCandidate::model();
        let blade = blade_from_design(&c, 0.0);
        let bsteps = plan_split(&blade, &onyx_pro(), 363.0).join_instructions();
        assert!(bsteps.iter().any(|s| s.contains("spar splice")));
        let boom = boom_for(2.0, c.radius_m);
        let msteps = plan_split(&boom, &onyx_pro(), 69.0).join_instructions();
        assert!(msteps.iter().any(|s| s.contains("sleeve")));
    }
}
