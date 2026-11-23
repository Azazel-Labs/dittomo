//! Retargeting between made-up humanoids that differ the ways real rigs do: names from different tools, extra bones,
//! longer spines, A-pose against T-pose, bone axes pointing different ways, facing, units and proportions.

use std::f32::consts::TAU;

use dittomo::{Bone, Clip, Mapping, Quat, Skeleton, Transform, Vec3};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Side {
    L,
    R,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Part {
    Root,
    Hips,
    /// A bone at the hips holding both legs, as some rigs have.
    Pelvis,
    /// A bone on the floor under the hips that moves nothing (a facial rig's anchor, say).
    Stray,
    Spine(u8),
    Neck,
    Head,
    HeadTop,
    Clavicle(Side),
    UpperArm(Side),
    ForeArm(Side),
    Hand(Side),
    Finger(Side),
    ArmTwist(Side),
    Thigh(Side),
    Shin(Side),
    Foot(Side),
    Toe(Side),
    ToeEnd(Side),
    LegTwist(Side),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Naming {
    Mixamo,
    Unreal,
    Blank,
}

#[derive(Clone, Copy)]
struct Style {
    naming: Naming,
    spine: u8,
    /// Degrees the arms hang below level at rest: 0 for a T-pose, 45 for an A-pose.
    arms_down: f32,
    legs: f32,
    unit: f32,
    /// Degrees turned about up from facing +z.
    facing: f32,
    root: bool,
    /// Legs hung off a pelvis bone at the hips, and a stray bone on the floor beside them.
    pelvis: bool,
    twists: bool,
    fingers: bool,
    /// How bones' axes sit at rest: 0 all the world's, 1 y along the bone, 2 x along the bone and rolled.
    axes: u8,
}

impl Default for Style {
    fn default() -> Self {
        Self {
            naming: Naming::Mixamo,
            spine: 3,
            arms_down: 0.0,
            legs: 1.0,
            unit: 1.0,
            facing: 0.0,
            root: false,
            pelvis: false,
            twists: false,
            fingers: true,
            axes: 1,
        }
    }
}

struct Rig {
    skeleton: Skeleton,
    parts: Vec<Part>,
}

impl Rig {
    fn find(&self, p: Part) -> Option<usize> {
        self.parts.iter().position(|&q| q == p)
    }
}

fn name(p: Part, naming: Naming, i: usize) -> String {
    let side = |s: Side, left: &str, right: &str| {
        if s == Side::L {
            left.to_owned()
        } else {
            right.to_owned()
        }
    };
    match naming {
        Naming::Blank => {
            if i == 0 {
                "Bone".into()
            } else {
                format!("Bone.{i:03}")
            }
        }
        Naming::Mixamo => {
            let m = |s: Side, x: &str| format!("mixamorig:{}{x}", side(s, "Left", "Right"));
            match p {
                Part::Root => "Root".into(),
                Part::Hips => "mixamorig:Hips".into(),
                Part::Pelvis => "mixamorig:Pelvis".into(),
                Part::Stray => "FacialAnim_Joint".into(),
                Part::Spine(0) => "mixamorig:Spine".into(),
                Part::Spine(k) => format!("mixamorig:Spine{k}"),
                Part::Neck => "mixamorig:Neck".into(),
                Part::Head => "mixamorig:Head".into(),
                Part::HeadTop => "mixamorig:HeadTop_End".into(),
                Part::Clavicle(s) => m(s, "Shoulder"),
                Part::UpperArm(s) => m(s, "Arm"),
                Part::ForeArm(s) => m(s, "ForeArm"),
                Part::Hand(s) => m(s, "Hand"),
                Part::Finger(s) => m(s, "HandIndex1"),
                Part::ArmTwist(s) => m(s, "ArmRoll"),
                Part::Thigh(s) => m(s, "UpLeg"),
                Part::Shin(s) => m(s, "Leg"),
                Part::Foot(s) => m(s, "Foot"),
                Part::Toe(s) => m(s, "ToeBase"),
                Part::ToeEnd(s) => m(s, "Toe_End"),
                Part::LegTwist(s) => m(s, "UpLegRoll"),
            }
        }
        Naming::Unreal => {
            let u = |s: Side, x: &str| format!("{x}_{}", side(s, "l", "r"));
            match p {
                Part::Root => "root".into(),
                Part::Hips => "pelvis".into(),
                Part::Pelvis => "hip_hub".into(),
                Part::Stray => "facial_anchor".into(),
                Part::Spine(k) => format!("spine_{:02}", k + 1),
                Part::Neck => "neck_01".into(),
                Part::Head => "head".into(),
                Part::HeadTop => "head_end".into(),
                Part::Clavicle(s) => u(s, "clavicle"),
                Part::UpperArm(s) => u(s, "upperarm"),
                Part::ForeArm(s) => u(s, "lowerarm"),
                Part::Hand(s) => u(s, "hand"),
                Part::Finger(s) => u(s, "index_01"),
                Part::ArmTwist(s) => u(s, "upperarm_twist_01"),
                Part::Thigh(s) => u(s, "thigh"),
                Part::Shin(s) => u(s, "calf"),
                Part::Foot(s) => u(s, "foot"),
                Part::Toe(s) => u(s, "ball"),
                Part::ToeEnd(s) => u(s, "ball_end"),
                Part::LegTwist(s) => u(s, "thigh_twist_01"),
            }
        }
    }
}

fn humanoid(st: Style) -> Rig {
    let h = st.legs;
    let mut list: Vec<(Part, Option<Part>, Vec3)> = Vec::new();
    let mut top = Part::Hips;
    if st.root {
        list.push((Part::Root, None, Vec3::ZERO));
    }
    list.push((Part::Hips, st.root.then_some(Part::Root), Vec3::new(0.0, h, 0.0)));
    let legs_from = if st.pelvis {
        list.push((Part::Pelvis, Some(Part::Hips), Vec3::new(0.0, h, 0.0)));
        list.push((Part::Stray, Some(Part::Hips), Vec3::new(0.01, 0.01, -0.08)));
        Part::Pelvis
    } else {
        Part::Hips
    };
    for k in 0..st.spine {
        let y = h + 0.5 * f32::from(k + 1) / f32::from(st.spine);
        list.push((Part::Spine(k), Some(top), Vec3::new(0.0, y - 0.04, 0.0)));
        top = Part::Spine(k);
    }
    list.push((Part::Neck, Some(top), Vec3::new(0.0, h + 0.56, 0.0)));
    list.push((Part::Head, Some(Part::Neck), Vec3::new(0.0, h + 0.66, 0.02)));
    list.push((Part::HeadTop, Some(Part::Head), Vec3::new(0.0, h + 0.88, 0.02)));
    let down = st.arms_down.to_radians();
    for s in [Side::L, Side::R] {
        let x = if s == Side::L { 1.0 } else { -1.0 };
        let dir = Vec3::new(down.cos() * x, -down.sin(), 0.0);
        let upper = Vec3::new(0.17 * x, h + 0.44, 0.0);
        list.push((Part::Clavicle(s), Some(top), Vec3::new(0.04 * x, h + 0.44, 0.0)));
        list.push((Part::UpperArm(s), Some(Part::Clavicle(s)), upper));
        if st.twists {
            list.push((Part::ArmTwist(s), Some(Part::UpperArm(s)), upper + 0.14 * dir));
        }
        list.push((Part::ForeArm(s), Some(Part::UpperArm(s)), upper + 0.28 * dir));
        list.push((Part::Hand(s), Some(Part::ForeArm(s)), upper + 0.53 * dir));
        if st.fingers {
            list.push((Part::Finger(s), Some(Part::Hand(s)), upper + 0.62 * dir));
        }
        let thigh = Vec3::new(0.1 * x, h - 0.05, 0.0);
        let shin = Vec3::new(0.1 * x, 0.52 * h, 0.01);
        list.push((Part::Thigh(s), Some(legs_from), thigh));
        if st.twists {
            list.push((Part::LegTwist(s), Some(Part::Thigh(s)), (thigh + shin) / 2.0));
        }
        list.push((Part::Shin(s), Some(Part::Thigh(s)), shin));
        list.push((Part::Foot(s), Some(Part::Shin(s)), Vec3::new(0.1 * x, 0.08, 0.0)));
        list.push((Part::Toe(s), Some(Part::Foot(s)), Vec3::new(0.1 * x, 0.02, 0.12)));
        list.push((Part::ToeEnd(s), Some(Part::Toe(s)), Vec3::new(0.1 * x, 0.02, 0.19)));
    }
    let face = Quat::from_rotation_y(st.facing.to_radians());
    let parts: Vec<Part> = list.iter().map(|l| l.0).collect();
    let index = |p: Part| parts.iter().position(|&q| q == p).expect("part");
    let pos: Vec<Vec3> = list.iter().map(|l| face * l.2 * st.unit).collect();
    // Each bone's turn in the world at rest, by the style's axes.
    let rot: Vec<Quat> = (0..list.len())
        .map(|i| {
            let child = list.iter().position(|l| l.1 == Some(list[i].0));
            let along = match (child, list[i].1) {
                (Some(c), _) => pos[c] - pos[i],
                (None, Some(p)) => pos[i] - pos[index(p)],
                (None, None) => Vec3::Y,
            }
            .try_normalize()
            .unwrap_or(Vec3::Y);
            match st.axes {
                0 => Quat::IDENTITY,
                1 => Quat::from_rotation_arc(Vec3::Y, along),
                _ => Quat::from_rotation_arc(Vec3::X, along) * Quat::from_rotation_x(0.7 + i as f32 * 0.3),
            }
        })
        .collect();
    let bones = list
        .iter()
        .enumerate()
        .map(|(i, (p, parent, _))| {
            let parent = parent.map(index);
            let rest = match parent {
                Some(q) => {
                    let back = rot[q].inverse();
                    Transform::new(back * (pos[i] - pos[q]), back * rot[i])
                }
                None => Transform::new(pos[i], rot[i]),
            };
            Bone {
                name: name(*p, st.naming, i),
                parent,
                rest,
            }
        })
        .collect();
    Rig {
        skeleton: Skeleton::new(bones).expect("parents first"),
        parts,
    }
}

/// A second of every bone swinging about its own axis, the hips bobbing and walking forward.
fn swing(rig: &Rig, unit: f32) -> Clip {
    let sk = &rig.skeleton;
    let mut clip = Clip::default();
    for f in 0..=24 {
        let t = f as f32 / 24.0;
        let pose = sk
            .bones()
            .iter()
            .enumerate()
            .map(|(i, b)| {
                let k = i as f32;
                let axis = Vec3::new((k * 1.3).sin(), (k * 0.7).cos(), (k * 2.1).sin() + 0.3).normalize();
                let angle = 0.5 * (TAU * t + k).sin();
                let mut tr = b.rest;
                tr.rotation = (b.rest.rotation * Quat::from_axis_angle(axis, angle)).normalize();
                if rig.parts[i] == Part::Hips {
                    tr.translation += Vec3::new(0.0, 0.05 * (2.0 * TAU * t).sin(), 0.8 * t) * unit;
                }
                tr
            })
            .collect();
        clip.times.push(t);
        clip.poses.push(pose);
    }
    clip
}

/// A part on each side.
type Sided = fn(Side) -> Part;

const LIMBS: &[(Sided, Sided)] = &[
    (Part::Clavicle, Part::UpperArm),
    (Part::UpperArm, Part::ForeArm),
    (Part::ForeArm, Part::Hand),
    (Part::Thigh, Part::Shin),
    (Part::Shin, Part::Foot),
    (Part::Foot, Part::Toe),
];

/// The worst angle, in degrees, between a segment's direction in the source and the target, each in its own body's
/// frame, over the clip.
fn worst(from: &Rig, to: &Rig, map: &Mapping, clip: &Clip, out: &Clip) -> f32 {
    let mut segs: Vec<(Part, Part)> = vec![(Part::Neck, Part::Head)];
    for s in [Side::L, Side::R] {
        segs.extend(LIMBS.iter().map(|(a, b)| (a(s), b(s))));
    }
    let mut worst = 0.0f32;
    for (ps, pt) in clip.poses.iter().zip(&out.poses) {
        let ws = from.skeleton.world(ps);
        let wt = to.skeleton.world(pt);
        for &(a, b) in &segs {
            let (Some(sa), Some(sb), Some(ta), Some(tb)) = (from.find(a), from.find(b), to.find(a), to.find(b)) else {
                continue;
            };
            let ds = map.turn() * (ws[sb].position() - ws[sa].position()).normalize();
            let dt = (wt[tb].position() - wt[ta].position()).normalize();
            worst = worst.max(ds.angle_between(dt).to_degrees());
        }
    }
    worst
}

/// The parts the target's bones were mapped to, where they're wrong: (target part, source part or none).
fn wrong(from: &Rig, to: &Rig, map: &Mapping, ignore: &[Part]) -> Vec<(Part, Option<Part>)> {
    let mut bad = Vec::new();
    for (b, &p) in to.parts.iter().enumerate() {
        if ignore.contains(&p) || matches!(p, Part::Spine(_)) {
            continue;
        }
        let got = map.source_of(b).map(|a| from.parts[a]);
        let want = from.find(p).map(|_| p);
        if got != want {
            bad.push((p, got));
        }
    }
    bad
}

#[test]
fn a_skeleton_onto_itself_plays_the_same() {
    let rig = humanoid(Style::default());
    let clip = swing(&rig, 1.0);
    let map = Mapping::new(&rig.skeleton, &rig.skeleton);
    assert!(map.quality() > 0.99, "quality {}", map.quality());
    for b in 0..rig.skeleton.len() {
        assert_eq!(map.source_of(b), Some(b));
    }
    let out = map.apply(&rig.skeleton, &rig.skeleton, &clip).expect("plays");
    for (a, b) in clip.poses.iter().flatten().zip(out.poses.iter().flatten()) {
        assert!(a.translation.distance(b.translation) < 1e-4, "{a:?} {b:?}");
        assert!(a.rotation.angle_between(b.rotation) < 1e-3, "{a:?} {b:?}");
    }
}

#[test]
fn names_from_another_tool_extra_bones_and_another_spine_still_match() {
    let from = humanoid(Style::default());
    let to = humanoid(Style {
        naming: Naming::Unreal,
        spine: 5,
        root: true,
        twists: true,
        axes: 2,
        legs: 1.1,
        ..Style::default()
    });
    let map = Mapping::new(&from.skeleton, &to.skeleton);
    assert_eq!(wrong(&from, &to, &map, &[]), vec![]);
    let clip = swing(&from, 1.0);
    let out = map.apply(&from.skeleton, &to.skeleton, &clip).expect("plays");
    let w = worst(&from, &to, &map, &clip, &out);
    assert!(w < 1.0, "worst {w}°");
}

#[test]
fn legs_hung_off_a_pelvis_both_match() {
    // The pelvis stands between the hips and both legs on one rig; the other hangs its legs and its spine off the
    // hips alike. Both legs must find their match, whichever way round, by names or by shape alone.
    for naming in [Naming::Mixamo, Naming::Blank] {
        let with = humanoid(Style {
            pelvis: true,
            naming,
            ..Style::default()
        });
        let without = humanoid(Style {
            naming: if naming == Naming::Blank {
                Naming::Blank
            } else {
                Naming::Unreal
            },
            legs: 1.1,
            ..Style::default()
        });
        for (from, to) in [(&with, &without), (&without, &with)] {
            let map = Mapping::new(&from.skeleton, &to.skeleton);
            assert_eq!(
                wrong(from, to, &map, &[Part::Pelvis, Part::Stray]),
                vec![],
                "{}",
                naming as u8
            );
            let clip = swing(from, 1.0);
            let out = map.apply(&from.skeleton, &to.skeleton, &clip).expect("plays");
            let w = worst(from, to, &map, &clip, &out);
            assert!(w < 1.0, "worst {w}°");
        }
    }
}

#[test]
fn unnamed_bones_match_by_shape_from_a_t_pose_to_an_a_pose() {
    let from = humanoid(Style {
        naming: Naming::Blank,
        axes: 0,
        ..Style::default()
    });
    let to = humanoid(Style {
        naming: Naming::Blank,
        arms_down: 45.0,
        axes: 2,
        legs: 0.9,
        ..Style::default()
    });
    let map = Mapping::new(&from.skeleton, &to.skeleton);
    assert_eq!(wrong(&from, &to, &map, &[]), vec![]);
    let clip = swing(&from, 1.0);
    let out = map.apply(&from.skeleton, &to.skeleton, &clip).expect("plays");
    let w = worst(&from, &to, &map, &clip, &out);
    assert!(w < 1.0, "worst {w}°");
}

#[test]
fn names_on_one_side_only_still_match() {
    let from = humanoid(Style::default());
    let to = humanoid(Style {
        naming: Naming::Blank,
        fingers: false,
        ..Style::default()
    });
    let map = Mapping::new(&from.skeleton, &to.skeleton);
    assert_eq!(wrong(&from, &to, &map, &[]), vec![]);
}

#[test]
fn a_character_facing_another_way_in_centimetres_plays_the_same() {
    let from = humanoid(Style::default());
    let to = humanoid(Style {
        facing: 90.0,
        unit: 100.0,
        naming: Naming::Unreal,
        ..Style::default()
    });
    let map = Mapping::new(&from.skeleton, &to.skeleton);
    assert_eq!(wrong(&from, &to, &map, &[]), vec![]);
    assert!(
        map.turn().angle_between(Quat::from_rotation_y(90f32.to_radians())) < 1e-3,
        "{:?}",
        map.turn()
    );
    let clip = swing(&from, 1.0);
    let out = map.apply(&from.skeleton, &to.skeleton, &clip).expect("plays");
    let w = worst(&from, &to, &map, &clip, &out);
    assert!(w < 1.0, "worst {w}°");
}

#[test]
fn knows_which_way_a_body_faces() {
    for (facing, naming) in [
        (0.0, Naming::Mixamo),
        (90.0, Naming::Unreal),
        (180.0, Naming::Blank),
        (-135.0, Naming::Blank),
    ] {
        let rig = humanoid(Style {
            facing,
            naming,
            ..Style::default()
        });
        let want = Quat::from_rotation_y(f32::to_radians(facing)) * Vec3::Z;
        let got = rig.skeleton.forward();
        assert!(got.angle_between(want) < 1e-3, "facing {facing}°: {got}");
    }
}

#[test]
fn the_hips_travel_as_far_as_their_height_allows() {
    let from = humanoid(Style::default());
    let to = humanoid(Style {
        legs: 0.6,
        naming: Naming::Unreal,
        ..Style::default()
    });
    let map = Mapping::new(&from.skeleton, &to.skeleton);
    let clip = swing(&from, 1.0);
    let out = map.apply(&from.skeleton, &to.skeleton, &clip).expect("plays");
    let (hs, ht) = (from.find(Part::Hips).unwrap(), to.find(Part::Hips).unwrap());
    let rest_s = from.skeleton.world(&from.skeleton.rest())[hs].position();
    let rest_t = to.skeleton.world(&to.skeleton.rest())[ht].position();
    // Heights over the lowest bone, the toes' ends at 0.02.
    let k = (rest_t.y - 0.02) / (rest_s.y - 0.02);
    assert!((k - 0.58 / 0.98).abs() < 1e-5, "{k}");
    for (ps, pt) in clip.poses.iter().zip(&out.poses) {
        let ms = from.skeleton.world(ps)[hs].position() - rest_s;
        let mt = to.skeleton.world(pt)[ht].position() - rest_t;
        assert!((ms * k).distance(mt) < 1e-4, "{ms} {mt}");
    }
}

#[test]
fn skeletons_with_little_alike_share_little() {
    let from = humanoid(Style::default());
    let snake = Skeleton::new(
        (0..12usize)
            .map(|i| Bone {
                name: format!("Bone.{i:03}"),
                parent: i.checked_sub(1),
                rest: Transform::new(
                    if i == 0 {
                        Vec3::new(0.0, 0.05, 0.0)
                    } else {
                        Vec3::new(0.0, 0.0, 0.15)
                    },
                    Quat::IDENTITY,
                ),
            })
            .collect(),
    )
    .expect("parents first");
    let map = Mapping::new(&from.skeleton, &snake);
    assert!(map.quality() < 0.4, "quality {}", map.quality());
    let out = map.apply(&from.skeleton, &snake, &swing(&from, 1.0)).expect("plays");
    assert!(
        out.poses
            .iter()
            .flatten()
            .all(|t| t.rotation.is_finite() && t.translation.is_finite())
    );
}

#[test]
fn everything_different_at_once_still_matches_by_shape() {
    let from = humanoid(Style {
        naming: Naming::Blank,
        axes: 0,
        ..Style::default()
    });
    let to = humanoid(Style {
        naming: Naming::Blank,
        arms_down: 40.0,
        spine: 5,
        twists: true,
        root: true,
        facing: 180.0,
        unit: 100.0,
        axes: 2,
        legs: 1.15,
        ..Style::default()
    });
    let map = Mapping::new(&from.skeleton, &to.skeleton);
    let twists: Vec<Part> = [Side::L, Side::R]
        .into_iter()
        .flat_map(|s| [Part::ArmTwist(s), Part::LegTwist(s)])
        .collect();
    let mut ignore = twists.clone();
    ignore.push(Part::Root);
    assert_eq!(wrong(&from, &to, &map, &ignore), vec![]);
    let clip = swing(&from, 1.0);
    let out = map.apply(&from.skeleton, &to.skeleton, &clip).expect("plays");
    let w = worst(&from, &to, &map, &clip, &out);
    assert!(w < 1.0, "worst {w}°");
}

/// A four-legged animal facing +z: hips, a spine forward to the neck and head, a tail back, and four legs of three
/// bones and a paw. Each bone's tag says what it is.
fn quadruped(named: bool, body: f32, tail: usize, legs: f32) -> (Skeleton, Vec<String>) {
    let mut list: Vec<(String, Option<usize>, Vec3)> = Vec::new();
    let mut push = |tag: &str, parent: Option<usize>, at: Vec3| {
        list.push((tag.to_owned(), parent, at));
        list.len() - 1
    };
    let h = 0.6 * legs;
    let hips = push("hips", None, Vec3::new(0.0, h, -0.4 * body));
    let mut top = hips;
    for k in 0..3 {
        top = push(
            &format!("spine{k}"),
            Some(top),
            Vec3::new(0.0, h + 0.02, (-0.4 + 0.27 * (k + 1) as f32) * body),
        );
    }
    let chest = top;
    let neck = push("neck", Some(chest), Vec3::new(0.0, h + 0.2, 0.5 * body));
    push("head", Some(neck), Vec3::new(0.0, h + 0.3, 0.62 * body));
    let mut t = hips;
    for k in 0..tail {
        t = push(
            &format!("tail{k}"),
            Some(t),
            Vec3::new(0.0, h + 0.02 * k as f32, -0.4 * body - 0.12 * (k + 1) as f32),
        );
    }
    for (end, from, z) in [("front", chest, 0.41 * body), ("back", hips, -0.4 * body)] {
        for (side, x) in [("l", 0.12f32), ("r", -0.12)] {
            let a = push(&format!("{end}_upper_{side}"), Some(from), Vec3::new(x, h - 0.05, z));
            let b = push(
                &format!("{end}_lower_{side}"),
                Some(a),
                Vec3::new(x, 0.55 * h, z + 0.03),
            );
            let c = push(&format!("{end}_foot_{side}"), Some(b), Vec3::new(x, 0.08, z - 0.02));
            push(&format!("{end}_paw_{side}"), Some(c), Vec3::new(x, 0.0, z + 0.06));
        }
    }
    let bones = list
        .iter()
        .enumerate()
        .map(|(i, (tag, parent, at))| Bone {
            name: if named { tag.clone() } else { format!("Bone.{i:03}") },
            parent: *parent,
            rest: Transform::new(parent.map_or(*at, |p| *at - list[p].2), Quat::IDENTITY),
        })
        .collect();
    (
        Skeleton::new(bones).expect("parents first"),
        list.into_iter().map(|l| l.0).collect(),
    )
}

#[test]
fn a_four_legged_animal_maps_onto_another_by_shape() {
    let (from, from_tags) = quadruped(true, 1.0, 4, 1.0);
    let (to, to_tags) = quadruped(false, 1.3, 7, 0.8);
    let map = Mapping::new(&from, &to);
    let mut bad = Vec::new();
    for (b, tag) in to_tags.iter().enumerate() {
        if tag.starts_with("tail") || tag.starts_with("spine") {
            continue;
        }
        let got = map.source_of(b).map(|a| from_tags[a].clone());
        if got.as_deref() != Some(tag.as_str()) {
            bad.push((tag.clone(), got));
        }
    }
    assert_eq!(bad, vec![]);
    let tails = (0..to.len())
        .filter(|&b| to_tags[b].starts_with("tail") && map.source_of(b).is_some())
        .count();
    assert!(tails >= 3, "{tails} tail bones played");
}

#[test]
fn a_sweep_of_rigs_all_match_and_play() {
    let namings = [Naming::Mixamo, Naming::Unreal, Naming::Blank];
    let mut failures = Vec::new();
    for i in 0..300u32 {
        let pick = |n: u32, k: u32| (i / k) % n;
        let from = Style {
            naming: namings[pick(3, 1) as usize],
            axes: pick(3, 7) as u8,
            ..Style::default()
        };
        let to = Style {
            naming: namings[pick(3, 3) as usize],
            arms_down: [0.0, 30.0, 50.0][pick(3, 2) as usize],
            spine: [2, 3, 5][pick(3, 5) as usize],
            axes: pick(3, 11) as u8,
            root: pick(2, 13) == 1,
            twists: pick(2, 17) == 1,
            fingers: pick(2, 19) == 1,
            facing: [0.0, 90.0, 200.0][pick(3, 23) as usize],
            legs: [0.8, 1.0, 1.25][pick(3, 29) as usize],
            unit: [1.0, 100.0][pick(2, 31) as usize],
            pelvis: pick(2, 37) == 1,
        };
        let (from, to) = (humanoid(from), humanoid(to));
        let map = Mapping::new(&from.skeleton, &to.skeleton);
        let mut ignore: Vec<Part> = [Side::L, Side::R]
            .into_iter()
            .flat_map(|s| [Part::ArmTwist(s), Part::LegTwist(s)])
            .collect();
        ignore.extend([Part::Root, Part::Pelvis, Part::Stray]);
        let bad = wrong(&from, &to, &map, &ignore);
        let clip = swing(&from, 1.0);
        let out = map.apply(&from.skeleton, &to.skeleton, &clip).expect("plays");
        let w = worst(&from, &to, &map, &clip, &out);
        if !bad.is_empty() || w > 1.0 {
            failures.push((i, bad, w));
        }
    }
    assert!(
        failures.is_empty(),
        "{} of 300 failed: {:#?}",
        failures.len(),
        &failures[..failures.len().min(5)]
    );
}

/// Joints as (name, parent, world position).
type Joints = Vec<(String, Option<String>, [f32; 3])>;

/// A skeleton from joints named with their parents and world positions, every bone unturned.
fn placed(joints: &Joints) -> Skeleton {
    let at = |n: &str| joints.iter().position(|j| j.0 == n).expect("parent listed");
    Skeleton::new(
        joints
            .iter()
            .map(|(name, parent, p)| {
                let from = parent.as_deref().map_or(Vec3::ZERO, |q| Vec3::from(joints[at(q)].2));
                Bone {
                    name: name.clone(),
                    parent: parent.as_deref().map(at),
                    rest: Transform::new(Vec3::from(*p) - from, Quat::IDENTITY),
                }
            })
            .collect(),
    )
    .expect("parents first")
}

fn joint(name: impl Into<String>, parent: Option<&str>, at: [f32; 3]) -> (String, Option<String>, [f32; 3]) {
    (name.into(), parent.map(str::to_owned), at)
}

/// Mixamo's layout, T-pose, to the top of the head, with three fingers on each hand straight from the hand.
fn mixamo_like() -> Skeleton {
    let mut j: Joints = vec![
        joint("Hips", None, [0.0, 1.0, 0.0]),
        joint("Spine", Some("Hips"), [0.0, 1.1, 0.0]),
        joint("Spine1", Some("Spine"), [0.0, 1.22, 0.0]),
        joint("Spine2", Some("Spine1"), [0.0, 1.35, 0.0]),
        joint("Neck", Some("Spine2"), [0.0, 1.5, 0.0]),
        joint("Head", Some("Neck"), [0.0, 1.6, 0.02]),
        joint("HeadTop_End", Some("Head"), [0.0, 1.82, 0.02]),
    ];
    for (side, x) in [("Left", 1.0f32), ("Right", -1.0)] {
        let n = |s: &str| format!("{side}{s}");
        j.push(joint(n("Shoulder"), Some("Spine2"), [0.06 * x, 1.45, 0.0]));
        j.push(joint(n("Arm"), Some(&n("Shoulder")), [0.18 * x, 1.45, 0.0]));
        j.push(joint(n("ForeArm"), Some(&n("Arm")), [0.46 * x, 1.45, 0.0]));
        j.push(joint(n("Hand"), Some(&n("ForeArm")), [0.72 * x, 1.45, 0.0]));
        for (f, z) in [("Thumb", 0.04f32), ("Index", 0.025), ("Middle", 0.005)] {
            let mut parent = n("Hand");
            for k in 1..=3 {
                let k_ = k as f32;
                let at = if f == "Thumb" {
                    [(0.74 + 0.025 * k_) * x, 1.43, z + 0.02 * k_]
                } else {
                    [(0.81 + 0.03 * k_) * x, 1.45, z]
                };
                let me = n(&format!("Hand{f}{k}"));
                j.push(joint(me.clone(), Some(&parent), at));
                parent = me;
            }
        }
        j.push(joint(n("UpLeg"), Some("Hips"), [0.09 * x, 0.95, 0.0]));
        j.push(joint(n("Leg"), Some(&n("UpLeg")), [0.09 * x, 0.52, 0.0]));
        j.push(joint(n("Foot"), Some(&n("Leg")), [0.09 * x, 0.08, -0.02]));
        j.push(joint(n("ToeBase"), Some(&n("Foot")), [0.09 * x, 0.0, 0.08]));
        j.push(joint(n("Toe_End"), Some(&n("ToeBase")), [0.09 * x, 0.0, 0.18]));
    }
    placed(&j)
}

fn assert_plays(from: &Skeleton, to: &Skeleton, map: &Mapping, pairs: &[(&str, &str)]) {
    for &(src, dst) in pairs {
        let b = to.index_of(dst).expect("target bone");
        assert_eq!(
            map.source_of(b).map(|a| from.bones()[a].name.as_str()),
            Some(src),
            "{dst}"
        );
    }
}

#[test]
fn a_palm_bone_holding_two_fingers_still_gets_both() {
    let from = mixamo_like();
    let mut j: Joints = vec![
        joint("hips", None, [0.0, 0.95, 0.0]),
        joint("spine", Some("hips"), [0.0, 1.1, 0.0]),
        joint("chest", Some("spine"), [0.0, 1.3, 0.0]),
        joint("neck", Some("chest"), [0.0, 1.45, 0.0]),
        joint("head", Some("neck"), [0.0, 1.55, 0.0]),
        joint("head_end", Some("head"), [0.0, 1.75, 0.0]),
    ];
    for (side, x) in [("L", 1.0f32), ("R", -1.0)] {
        let n = |s: &str| format!("{s}.{side}");
        j.push(joint(n("shoulder"), Some("chest"), [0.05 * x, 1.4, 0.0]));
        j.push(joint(n("upper_arm"), Some(&n("shoulder")), [0.17 * x, 1.4, 0.0]));
        j.push(joint(n("forearm"), Some(&n("upper_arm")), [0.44 * x, 1.4, 0.0]));
        j.push(joint(n("hand"), Some(&n("forearm")), [0.69 * x, 1.4, 0.0]));
        // Index and thumb share a palm bone; the middle finger has its own.
        j.push(joint(n("palm.01"), Some(&n("hand")), [0.72 * x, 1.4, 0.02]));
        j.push(joint(n("palm.02"), Some(&n("hand")), [0.72 * x, 1.4, 0.0]));
        for k in 1..=3 {
            let k_ = k as f32;
            let prev = |f: &str, root: &str| if k == 1 { n(root) } else { n(&format!("{f}.0{}", k - 1)) };
            let index = [(0.78 + 0.03 * k_) * x, 1.4, 0.025];
            let thumb = [(0.71 + 0.025 * k_) * x, 1.38, 0.04 + 0.02 * k_];
            let middle = [(0.78 + 0.03 * k_) * x, 1.4, 0.005];
            j.push(joint(
                n(&format!("f_index.0{k}")),
                Some(&prev("f_index", "palm.01")),
                index,
            ));
            j.push(joint(n(&format!("thumb.0{k}")), Some(&prev("thumb", "palm.01")), thumb));
            j.push(joint(
                n(&format!("f_middle.0{k}")),
                Some(&prev("f_middle", "palm.02")),
                middle,
            ));
        }
        j.push(joint(n("thigh"), Some("hips"), [0.09 * x, 0.9, 0.0]));
        j.push(joint(n("shin"), Some(&n("thigh")), [0.09 * x, 0.5, 0.0]));
        j.push(joint(n("foot"), Some(&n("shin")), [0.09 * x, 0.08, 0.0]));
        j.push(joint(n("toe"), Some(&n("foot")), [0.09 * x, 0.0, 0.1]));
    }
    let to = placed(&j);
    let map = Mapping::new(&from, &to);
    assert_plays(
        &from,
        &to,
        &map,
        &[
            ("LeftHandIndex1", "f_index.01.L"),
            ("LeftHandIndex3", "f_index.03.L"),
            ("LeftHandThumb1", "thumb.01.L"),
            ("LeftHandMiddle1", "f_middle.01.L"),
            ("RightHandIndex1", "f_index.01.R"),
            ("RightHandThumb2", "thumb.02.R"),
        ],
    );
}

#[test]
fn a_rig_stopping_at_the_base_of_the_head_in_an_a_pose_with_numbered_limbs() {
    // Laid out as COLLADA exports often are: no bone at the top of the head, limbs numbered along the chain, the arms
    // hanging down.
    let from = mixamo_like();
    let mut j: Joints = vec![
        joint("torso_joint_1", None, [0.0, 0.68, 0.0]),
        joint("torso_joint_2", Some("torso_joint_1"), [0.0, 0.84, 0.0]),
        joint("torso_joint_3", Some("torso_joint_2"), [0.0, 1.07, 0.0]),
        joint("neck_joint_1", Some("torso_joint_3"), [0.0, 1.13, 0.0]),
        joint("neck_joint_2", Some("neck_joint_1"), [0.0, 1.19, 0.0]),
    ];
    for (side, x) in [("L", 1.0f32), ("R", -1.0)] {
        let n = |s: &str, k: u32| format!("{s}_joint_{side}_{k}");
        j.push(joint(n("arm", 1), Some("torso_joint_3"), [0.09 * x, 1.07, 0.0]));
        j.push(joint(n("arm", 2), Some(&n("arm", 1)), [0.31 * x, 0.96, 0.0]));
        j.push(joint(n("arm", 3), Some(&n("arm", 2)), [0.45 * x, 0.88, 0.05]));
        j.push(joint(n("leg", 1), Some("torso_joint_1"), [0.07 * x, 0.61, 0.0]));
        j.push(joint(n("leg", 2), Some(&n("leg", 1)), [0.08 * x, 0.35, 0.05]));
        j.push(joint(n("leg", 3), Some(&n("leg", 2)), [0.08 * x, 0.08, 0.0]));
        j.push(joint(n("leg", 5), Some(&n("leg", 3)), [0.08 * x, 0.02, 0.04]));
    }
    let to = placed(&j);
    let map = Mapping::new(&from, &to);
    assert_plays(
        &from,
        &to,
        &map,
        &[
            ("Hips", "torso_joint_1"),
            ("Spine2", "torso_joint_3"),
            ("Neck", "neck_joint_1"),
            ("Head", "neck_joint_2"),
            ("LeftArm", "arm_joint_L_1"),
            ("LeftForeArm", "arm_joint_L_2"),
            ("LeftHand", "arm_joint_L_3"),
            ("RightForeArm", "arm_joint_R_2"),
            ("LeftUpLeg", "leg_joint_L_1"),
            ("LeftLeg", "leg_joint_L_2"),
            ("LeftFoot", "leg_joint_L_3"),
            ("RightToeBase", "leg_joint_R_5"),
        ],
    );
}

#[test]
fn a_nameless_rig_stopping_at_the_base_of_the_head_with_one_hand_missing() {
    // No names to go by, no bone at the top of the head, and a right arm that stops at the forearm: the height comes
    // from the shoulders, and the arms are still a pair.
    let from = mixamo_like();
    let mut j: Joints = vec![
        joint("node0", None, [0.0, 0.0, 0.0]),
        joint("node1", Some("node0"), [0.0, 0.95, 0.0]),
        joint("node2", Some("node1"), [0.0, 1.12, 0.0]),
        joint("node3", Some("node2"), [0.0, 1.32, 0.0]),
        joint("node4", Some("node3"), [0.0, 1.5, 0.0]),
        joint("node5", Some("node4"), [0.0, 1.6, 0.02]),
    ];
    let mut k = 6;
    let mut next = |j: &mut Joints, parent: &str, at: [f32; 3]| {
        let me = format!("node{k}");
        j.push(joint(me.clone(), Some(parent), at));
        k += 1;
        me
    };
    for x in [1.0f32, -1.0] {
        let shoulder = next(&mut j, "node3", [0.06 * x, 1.45, 0.0]);
        let arm = next(&mut j, &shoulder, [0.18 * x, 1.45, 0.0]);
        let forearm = next(&mut j, &arm, [0.46 * x, 1.45, 0.0]);
        if x > 0.0 {
            let hand = next(&mut j, &forearm, [0.72 * x, 1.45, 0.0]);
            for z in [0.04f32, 0.02, 0.0] {
                let f1 = next(&mut j, &hand, [0.82 * x, 1.45, z]);
                next(&mut j, &f1, [0.9 * x, 1.45, z]);
            }
        }
        let thigh = next(&mut j, "node1", [0.09 * x, 0.9, 0.0]);
        let shin = next(&mut j, &thigh, [0.09 * x, 0.5, 0.0]);
        let foot = next(&mut j, &shin, [0.09 * x, 0.08, -0.02]);
        next(&mut j, &foot, [0.09 * x, 0.0, 0.08]);
    }
    let to = placed(&j);
    let map = Mapping::new(&from, &to);
    // node7..: the left shoulder, arm, forearm, hand; the right shoulder, arm and forearm follow the left leg.
    let name_of = |at: [f32; 3]| {
        j.iter()
            .find(|q| q.2 == at)
            .map(|q| q.0.clone())
            .expect("a joint there")
    };
    let (l_arm, r_arm, r_forearm) = (
        name_of([0.18, 1.45, 0.0]),
        name_of([-0.18, 1.45, 0.0]),
        name_of([-0.46, 1.45, 0.0]),
    );
    assert_plays(
        &from,
        &to,
        &map,
        &[
            ("Hips", "node1"),
            ("Neck", "node4"),
            ("Head", "node5"),
            ("LeftArm", &l_arm),
            ("RightArm", &r_arm),
            ("RightForeArm", &r_forearm),
        ],
    );
}

#[test]
fn a_chest_bends_the_same_whether_its_collarbones_sit_above_it_or_beside_it() {
    // Mixamo's collarbones start just above the top spine bone; a cartoon's start out at its sides, level with it.
    // The chest is aimed by all its children together, so one rig's collarbones don't tip the other's chest.
    let from = mixamo_like();
    let to = {
        let mut j: Joints = vec![
            joint("Hips", None, [0.0, 1.0, 0.0]),
            joint("Spine", Some("Hips"), [0.0, 1.1, 0.0]),
            joint("Spine1", Some("Spine"), [0.0, 1.22, 0.0]),
            joint("Spine2", Some("Spine1"), [0.0, 1.35, 0.0]),
            joint("Neck", Some("Spine2"), [0.0, 1.55, 0.0]),
            joint("Head", Some("Neck"), [0.0, 1.65, 0.0]),
            joint("HeadTop_End", Some("Head"), [0.0, 1.85, 0.0]),
        ];
        for (side, x) in [("Left", 1.0f32), ("Right", -1.0)] {
            let n = |s: &str| format!("{side}{s}");
            j.push(joint(n("Shoulder"), Some("Spine2"), [0.2 * x, 1.37, 0.0]));
            j.push(joint(n("Arm"), Some(&n("Shoulder")), [0.26 * x, 1.4, 0.0]));
            j.push(joint(n("ForeArm"), Some(&n("Arm")), [0.52 * x, 1.4, 0.0]));
            j.push(joint(n("Hand"), Some(&n("ForeArm")), [0.76 * x, 1.4, 0.0]));
            j.push(joint(n("UpLeg"), Some("Hips"), [0.1 * x, 0.95, 0.0]));
            j.push(joint(n("Leg"), Some(&n("UpLeg")), [0.1 * x, 0.52, 0.0]));
            j.push(joint(n("Foot"), Some(&n("Leg")), [0.1 * x, 0.08, -0.02]));
            j.push(joint(n("ToeBase"), Some(&n("Foot")), [0.1 * x, 0.0, 0.08]));
        }
        placed(&j)
    };
    let map = Mapping::new(&from, &to);
    let (chest, neck) = (from.index_of("Spine2").unwrap(), from.index_of("Neck").unwrap());
    let (t_chest, t_neck) = (to.index_of("Spine2").unwrap(), to.index_of("Neck").unwrap());
    assert_eq!(map.source_of(t_chest), Some(chest));
    let mut clip = Clip::default();
    for (k, degrees) in [0.0f32, 30.0, -20.0].into_iter().enumerate() {
        let mut pose = from.rest();
        pose[chest].rotation = Quat::from_rotation_x(degrees.to_radians());
        clip.times.push(k as f32);
        clip.poses.push(pose);
    }
    let out = map.apply(&from, &to, &clip).unwrap();
    for (pose, played) in clip.poses.iter().zip(&out.poses) {
        let (ws, wt) = (from.world(pose), to.world(played));
        let ds = (ws[neck].position() - ws[chest].position()).normalize();
        let dt = (wt[t_neck].position() - wt[t_chest].position()).normalize();
        let off = ds.dot(dt).clamp(-1.0, 1.0).acos().to_degrees();
        assert!(off < 3.0, "the neck leans {off:.1}° away from the source's");
    }
}
