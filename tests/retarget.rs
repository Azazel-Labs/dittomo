//! Retargeting between made-up humanoids that differ the ways real rigs do: names from different tools, extra bones,
//! longer spines, A-pose against T-pose, bone axes pointing different ways, facing, units and proportions.

use std::f32::consts::TAU;

use dittomo::{Bone, Clip, Crowd, Fixes, Mapping, Player, Quat, Skeleton, Transform, Vec3};

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
fn end_bones_under_every_leaf_change_nothing() {
    // As a BVH reads: a bone for each End Site, under twist bones as much as under fingers and toes.
    let rig = humanoid(Style {
        naming: Naming::Unreal,
        twists: true,
        fingers: true,
        ..Style::default()
    });
    let mut bones = rig.skeleton.bones().to_vec();
    for (i, b) in rig.skeleton.bones().iter().enumerate() {
        if rig.skeleton.bones().iter().all(|c| c.parent != Some(i)) {
            bones.push(Bone {
                name: format!("{}_end", b.name),
                parent: Some(i),
                rest: Transform::new(0.3 * b.rest.translation, Quat::IDENTITY),
            });
        }
    }
    let ended = Skeleton::new(bones).expect("parents first");
    for (from, to) in [(&rig.skeleton, &ended), (&ended, &rig.skeleton)] {
        let map = Mapping::new(from, to);
        // The end bones themselves play nothing, which costs a little.
        assert!(map.quality() > 0.93, "quality {}", map.quality());
        for b in 0..rig.skeleton.len() {
            assert_eq!(map.source_of(b), Some(b), "{}", rig.skeleton.bones()[b].name);
        }
    }
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
fn a_character_facing_another_way_in_centimeters_plays_the_same() {
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
fn a_crowd_plays_each_character_as_a_player_does() {
    let from = humanoid(Style {
        root: true,
        ..Style::default()
    });
    let to = humanoid(Style {
        naming: Naming::Unreal,
        spine: 5,
        facing: 90.0,
        unit: 100.0,
        pelvis: true,
        twists: true,
        axes: 2,
        ..Style::default()
    });
    let (s, t) = (&from.skeleton, &to.skeleton);
    let map = Mapping::new(s, t);
    let clip = swing(&from, 1.0);
    let mut player = Player::new(&map, s, t).expect("fits");
    player.moving_in(&clip).expect("poses the source");
    let mut crowd = Crowd::new(player.clone());
    // Eleven characters, not a whole number of lanes, each at its own frame.
    let frames: Vec<usize> = (0..11).map(|c| (3 * c) % clip.poses.len()).collect();
    let poses: Vec<Transform> = frames.iter().flat_map(|&f| clip.poses[f].iter().copied()).collect();
    let mut out = vec![Transform::IDENTITY; frames.len() * t.len()];
    crowd.pose(&poses, &mut out).expect("plays");
    let mut one = t.rest();
    for (c, &f) in frames.iter().enumerate() {
        player.pose(&clip.poses[f], &mut one).expect("plays");
        for (a, b) in one.iter().zip(&out[c * t.len()..]) {
            assert!(a.translation.distance(b.translation) < 1e-3, "{a:?} {b:?}");
            assert!(a.rotation.dot(b.rotation).abs() > 1.0 - 1e-6, "{a:?} {b:?}");
            assert_eq!(a.scale, b.scale);
        }
    }
    assert!(crowd.pose(&poses[1..], &mut out).is_err());
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

/// A robot laid out as three.js's RobotExpressive: a long neck under a big head, its head's root its highest joint;
/// legs hung from a body bone, the spine from a pelvis hung below it, one bone up to the chest; a palm three fingers
/// hang from; the feet hung from the root, for IK, the legs ending at the knee.
fn expressive_like() -> Skeleton {
    let mut j: Joints = vec![
        joint("Bone", None, [0.0, -0.01, -0.2]),
        joint("Body", Some("Bone"), [0.0, 1.44, 0.11]),
        joint("Hips", Some("Body"), [0.0, 1.34, 0.0]),
        joint("Abdomen", Some("Hips"), [0.0, 1.66, -0.08]),
        joint("Torso", Some("Abdomen"), [0.0, 2.15, -0.08]),
        joint("Neck", Some("Torso"), [0.0, 2.73, -0.02]),
        joint("Head", Some("Neck"), [0.0, 3.04, -0.01]),
    ];
    for (side, x) in [("L", 1.0f32), ("R", -1.0)] {
        let n = |s: &str| format!("{s}.{side}");
        j.push(joint(n("Shoulder"), Some("Torso"), [0.58 * x, 2.44, -0.02]));
        j.push(joint(n("UpperArm"), Some(&n("Shoulder")), [0.67 * x, 2.44, -0.02]));
        j.push(joint(n("LowerArm"), Some(&n("UpperArm")), [1.32 * x, 2.43, 0.02]));
        j.push(joint(n("Palm"), Some(&n("LowerArm")), [2.69 * x, 2.45, -0.03]));
        for (f, y) in [("Index", 2.63f32), ("Middle", 2.45), ("Ring", 2.26)] {
            j.push(joint(n(&format!("{f}1")), Some(&n("Palm")), [3.0 * x, y, 0.18]));
            j.push(joint(
                n(&format!("{f}2")),
                Some(&n(&format!("{f}1"))),
                [3.15 * x, y, 0.3],
            ));
        }
        j.push(joint(n("UpperLeg"), Some("Body"), [0.63 * x, 1.54, -0.07]));
        j.push(joint(n("LowerLeg"), Some(&n("UpperLeg")), [0.63 * x, 0.94, 0.04]));
        j.push(joint(n("Foot"), Some("Bone"), [0.63 * x, 0.02, -0.14]));
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
fn ik_goals_at_the_hands_and_feet_play_nothing() {
    // Laid out as Unreal's mannequins are: beside the body's own bones, IK goals for the feet and hands under the root,
    // each standing where its foot or hand stands and named for it.
    let from = mixamo_like();
    let mut j: Joints = vec![
        joint("root", None, [0.0, 0.0, 0.0]),
        joint("pelvis", Some("root"), [0.0, 1.0, 0.0]),
        joint("spine_01", Some("pelvis"), [0.0, 1.1, 0.0]),
        joint("spine_02", Some("spine_01"), [0.0, 1.22, 0.0]),
        joint("spine_03", Some("spine_02"), [0.0, 1.35, 0.0]),
        joint("neck_01", Some("spine_03"), [0.0, 1.5, 0.0]),
        joint("head", Some("neck_01"), [0.0, 1.6, 0.02]),
        joint("ik_foot_root", Some("root"), [0.0, 0.0, 0.0]),
        joint("ik_hand_root", Some("root"), [0.0, 0.0, 0.0]),
        joint("ik_hand_gun", Some("ik_hand_root"), [-0.72, 1.45, 0.0]),
    ];
    for (side, x) in [("l", 1.0f32), ("r", -1.0)] {
        let n = |s: &str| format!("{s}_{side}");
        j.push(joint(n("clavicle"), Some("spine_03"), [0.06 * x, 1.45, 0.0]));
        j.push(joint(n("upperarm"), Some(&n("clavicle")), [0.18 * x, 1.45, 0.0]));
        j.push(joint(n("lowerarm"), Some(&n("upperarm")), [0.46 * x, 1.45, 0.0]));
        j.push(joint(n("hand"), Some(&n("lowerarm")), [0.72 * x, 1.45, 0.0]));
        j.push(joint(n("index_01"), Some(&n("hand")), [0.82 * x, 1.45, 0.02]));
        j.push(joint(n("index_02"), Some(&n("index_01")), [0.86 * x, 1.45, 0.02]));
        j.push(joint(n("thigh"), Some("pelvis"), [0.09 * x, 0.95, 0.0]));
        j.push(joint(n("calf"), Some(&n("thigh")), [0.09 * x, 0.52, 0.0]));
        j.push(joint(n("foot"), Some(&n("calf")), [0.09 * x, 0.08, -0.02]));
        j.push(joint(n("ball"), Some(&n("foot")), [0.09 * x, 0.0, 0.08]));
        j.push(joint(n("ik_foot"), Some("ik_foot_root"), [0.09 * x, 0.08, -0.02]));
        j.push(joint(n("ik_hand"), Some("ik_hand_gun"), [0.72 * x, 1.45, 0.0]));
    }
    let to = placed(&j);
    let map = Mapping::new(&from, &to);
    assert_plays(
        &from,
        &to,
        &map,
        &[
            ("Hips", "pelvis"),
            ("LeftHand", "hand_l"),
            ("RightHand", "hand_r"),
            ("LeftFoot", "foot_l"),
            ("RightFoot", "foot_r"),
            ("LeftToeBase", "ball_l"),
        ],
    );
    for goal in ["ik_foot_l", "ik_foot_r", "ik_hand_l", "ik_hand_r", "ik_hand_gun"] {
        let b = to.index_of(goal).unwrap();
        assert_eq!(map.source_of(b), None, "{goal}");
    }
    // And the other way, the body's bones are played from the body's, not the goals.
    let back = Mapping::new(&to, &from);
    assert_plays(
        &to,
        &from,
        &back,
        &[
            ("hand_l", "LeftHand"),
            ("hand_r", "RightHand"),
            ("foot_l", "LeftFoot"),
            ("foot_r", "RightFoot"),
        ],
    );
}

#[test]
fn an_upper_arm_after_a_stub_at_the_shoulder_plays_an_upper_arm_from_the_chest() {
    // Three.js's expressive robot starts each arm with a stub, a tenth as long as its upper arm, out from the chest; the
    // BrainStem robot's upper arm starts at its chest. Each stands about where the other's upper arm starts, and the stub
    // is as far from the chest as BrainStem's upper arm: so BrainStem's upper arm played the stub, and stayed down as
    // the robot raised its arms. Where each goes on to tells them apart.
    let mut robot: Joints = vec![
        joint("RootNode", None, [0.0, 0.0, 0.0]),
        joint("Bone", Some("RootNode"), [0.0, -0.01, -0.2]),
        joint("Body", Some("Bone"), [0.0, 1.44, 0.11]),
        joint("Hips", Some("Body"), [0.0, 1.34, 0.0]),
        joint("Abdomen", Some("Hips"), [0.0, 1.66, -0.08]),
        joint("Torso", Some("Abdomen"), [0.0, 2.15, -0.08]),
        joint("Neck", Some("Torso"), [0.0, 2.73, -0.02]),
        joint("Head", Some("Neck"), [0.0, 3.04, -0.01]),
    ];
    let mut stem: Joints = vec![
        joint("node0", None, [0.0, 0.0, 0.0]),
        joint("node3", Some("node0"), [0.0, 0.91, 0.0]),
        joint("node12", Some("node3"), [0.0, 1.02, -0.02]),
        joint("node20", Some("node12"), [0.0, 1.55, -0.04]),
    ];
    for (side, s, n) in [("L", 1.0, 0), ("R", -1.0, 5)] {
        let named = |part: &str| format!("{part}.{side}");
        let chain = |j: &mut Joints, from: &str, links: &[(String, [f32; 3])]| {
            let mut parent = from.to_owned();
            for (name, [x, y, z]) in links {
                j.push(joint(name.clone(), Some(&parent), [s * x, *y, *z]));
                parent = name.clone();
            }
        };
        chain(
            &mut robot,
            "Torso",
            &[
                (named("Shoulder"), [0.58, 2.44, -0.02]),
                (named("UpperArm"), [0.67, 2.44, -0.02]),
                (named("LowerArm"), [1.32, 2.43, 0.02]),
            ],
        );
        for (palm, finger, y) in [
            ("Palm1", "Index", 2.63),
            ("Palm2", "Middle", 2.45),
            ("Palm3", "Ring", 2.26),
        ] {
            let lower = named("LowerArm");
            chain(
                &mut robot,
                &lower,
                &[
                    (named(palm), [2.69, 2.45, -0.03]),
                    (named(&format!("{finger}1")), [3.0, y, 0.18]),
                    (named(&format!("{finger}2")), [3.15, y, 0.3]),
                ],
            );
        }
        chain(
            &mut robot,
            "Hips",
            &[
                (named("UpperLeg"), [0.63, 1.54, -0.07]),
                (named("LowerLeg"), [0.63, 0.94, 0.04]),
            ],
        );
        chain(&mut robot, "Bone", &[(named("Foot"), [0.63, 0.02, -0.14])]);
        let node = |i: usize| format!("node{}", i + n);
        chain(
            &mut stem,
            "node12",
            &[(node(13), [0.42, 1.36, -0.04]), (node(14), [0.6, 1.37, -0.04])],
        );
        // BrainStem's left hand holds three bones, its right forearm none.
        if n == 0 {
            for (i, y, z) in [(15, 1.35, 0.02), (16, 1.39, 0.02), (17, 1.37, -0.02)] {
                chain(&mut stem, "node14", &[(node(i), [1.02, y, z])]);
            }
        }
        let n4 = if n == 0 { 4 } else { 8 };
        let leg = |i: usize| format!("node{}", n4 + i);
        chain(
            &mut stem,
            "node3",
            &[
                (leg(0), [0.14, 0.88, 0.0]),
                (leg(1), [0.12, 0.52, -0.03]),
                (leg(2), [0.11, 0.12, -0.06]),
                (leg(3), [0.11, 0.04, 0.04]),
            ],
        );
    }
    let (robot, stem) = (placed(&robot), placed(&stem));
    let (left, right) = (["node13", "node14"], ["node18", "node19"]);
    let map = Mapping::new(&robot, &stem);
    assert_plays(
        &robot,
        &stem,
        &map,
        &[
            ("UpperArm.L", left[0]),
            ("LowerArm.L", left[1]),
            ("UpperArm.R", right[0]),
            ("LowerArm.R", right[1]),
        ],
    );
    let map = Mapping::new(&stem, &robot);
    assert_plays(
        &stem,
        &robot,
        &map,
        &[
            (left[0], "UpperArm.L"),
            (left[1], "LowerArm.L"),
            (right[0], "UpperArm.R"),
            (right[1], "LowerArm.R"),
        ],
    );
}

#[test]
fn a_chest_just_above_the_hips_is_still_the_chest() {
    // As a robot's might be: one short bone from the hips up to where the arms and the neck hang, far below where a
    // chest stands on a person, and no names to go by. The arms hang from it, so it plays the chest, not the spine.
    let from = mixamo_like();
    let mut j: Joints = vec![
        joint("node0", None, [0.0, 0.0, 0.0]),
        joint("node1", Some("node0"), [0.0, 0.92, 0.0]),
        joint("node2", Some("node1"), [0.0, 1.02, -0.02]),
        joint("node3", Some("node2"), [0.0, 1.5, -0.02]),
        joint("node4", Some("node3"), [0.0, 1.6, 0.0]),
    ];
    for (k, x) in [(5, 1.0f32), (12, -1.0)] {
        let n = |i: usize| format!("node{}", k + i);
        j.push(joint(n(0), Some("node2"), [0.2 * x, 1.42, -0.02]));
        j.push(joint(n(1), Some(&n(0)), [0.46 * x, 1.42, -0.02]));
        j.push(joint(n(2), Some(&n(1)), [0.72 * x, 1.42, -0.02]));
        j.push(joint(n(3), Some("node1"), [0.1 * x, 0.88, 0.0]));
        j.push(joint(n(4), Some(&n(3)), [0.1 * x, 0.5, 0.0]));
        j.push(joint(n(5), Some(&n(4)), [0.1 * x, 0.08, -0.02]));
        j.push(joint(n(6), Some(&n(5)), [0.1 * x, 0.0, 0.08]));
    }
    let to = placed(&j);
    let map = Mapping::new(&from, &to);
    assert_plays(
        &from,
        &to,
        &map,
        &[
            ("Hips", "node1"),
            ("Spine2", "node2"),
            ("Neck", "node3"),
            ("LeftArm", "node5"),
            ("LeftForeArm", "node6"),
            ("RightArm", "node12"),
            ("LeftUpLeg", "node8"),
            ("RightFoot", "node17"),
        ],
    );
}

/// A fox as glTF's sample Fox is laid out, its front legs ahead of the spine bone they hang from.
fn fox() -> Skeleton {
    fox_with_right_hand_up(0.0)
}

/// The fox, its right forepaw raised a little, its right foreleg that much shorter.
fn fox_with_right_hand_up(up: f32) -> Skeleton {
    placed(&fox_joints(up))
}

/// [`fox_with_right_hand_up`]'s joints.
fn fox_joints(up: f32) -> Joints {
    let mut j: Joints = vec![
        joint("b_Root_00", None, [0.0, 0.0, 0.0]),
        joint("b_Hip_01", Some("b_Root_00"), [0.0, 0.707, -0.44]),
        joint("b_Spine01_02", Some("b_Hip_01"), [0.0, 0.905, -0.365]),
        joint("b_Spine02_03", Some("b_Spine01_02"), [0.0, 0.885, -0.009]),
        joint("b_Neck_04", Some("b_Spine02_03"), [0.0, 0.876, 0.413]),
        joint("b_Head_05", Some("b_Neck_04"), [0.0, 1.0, 0.595]),
        joint("b_Tail01_012", Some("b_Hip_01"), [0.0, 0.866, -0.661]),
        joint("b_Tail02_013", Some("b_Tail01_012"), [0.0, 0.716, -0.8]),
        joint("b_Tail03_014", Some("b_Tail02_013"), [0.0, 0.462, -1.108]),
    ];
    for (side, x) in [("Left", 0.115), ("Right", -0.115)] {
        let arm = [
            ("UpperArm", [0.808, 0.297]),
            ("ForeArm", [0.429, 0.283]),
            ("Hand", [0.11, 0.294]),
        ];
        let leg = [
            ("Leg01", [0.811, -0.492]),
            ("Leg02", [0.502, -0.452]),
            ("Foot01", [0.262, -0.625]),
            ("Foot02", [0.016, -0.542]),
        ];
        for (chain, from) in [(&arm[..], "b_Spine02_03"), (&leg[..], "b_Hip_01")] {
            let mut parent = from.to_owned();
            for &(part, [y, z]) in chain {
                let name = format!("b_{side}{part}");
                let y = if part == "Hand" && side == "Right" { y + up } else { y };
                j.push(joint(name.clone(), Some(&parent), [x, y, z]));
                parent = name;
            }
        }
    }
    j
}

#[test]
fn a_tail_lying_down_lies_on_the_floor() {
    // The fox lying on its belly, its hips brought down near the floor, played on itself with its tail playing nothing:
    // left as it hangs at rest, the tail would go a tenth of the fox's height through the floor. Each bone of it turns
    // up just enough to lay the next along the floor; the legs, playing the source's, come as low as its do.
    let fox = fox();
    let mut map = Mapping::new(&fox, &fox);
    for tail in ["b_Tail01_012", "b_Tail02_013", "b_Tail03_014"] {
        map.set(fox.index_of(tail).unwrap(), None);
    }
    let hip = fox.index_of("b_Hip_01").unwrap();
    let mut pose = fox.rest();
    pose[hip].translation.y -= 0.56;
    let clip = Clip {
        times: vec![0.0],
        poses: vec![pose],
    };
    let tip = fox.index_of("b_Tail03_014").unwrap();
    let lowest = |fixes: Fixes| {
        let out = map.apply_with(&fox, &fox, &clip, fixes).unwrap();
        let world = fox.world(&out.poses[0]);
        (world[tip].position().y, world[hip].position().y)
    };
    let (sunk, _) = lowest(Fixes::NONE);
    assert!(sunk < -0.05, "the tail's tip at {sunk} unfixed");
    let (tip_y, hip_y) = lowest(Fixes {
        floor: true,
        ..Fixes::NONE
    });
    assert!(tip_y.abs() < 1e-4, "the tail's tip at {tip_y}");
    assert!((hip_y - 0.147).abs() < 1e-4, "the hips at {hip_y}");
}

#[test]
fn a_crouch_deeper_than_a_leg_folds_rests_on_its_hocks() {
    // A dog whose hind legs fold tight (short above the hock, long below) crouches with its hips low and its paws planted
    // under them, played on a husky's build, long above the hock: its legs can't fold that far, and folded flat, the
    // hocks point through the floor. Swinging about hip to paw can't lift a leg folded flat, so it turns about its hip,
    // as it bends, laying the hock on the floor behind; the paw comes up a little.
    let hock_held = |sk: Skeleton| {
        let w = sk.world(&sk.rest());
        let mut bones = sk.bones().to_vec();
        for side in ["L", "R"] {
            let (ik, hock) = (
                sk.index_of(&format!("IKBackLeg.{side}")),
                sk.index_of(&format!("BackLowerLeg.{side}")),
            );
            let (ik, hock) = (ik.unwrap(), hock.unwrap());
            bones[ik].parent = Some(hock);
            bones[ik].rest.translation = w[ik].position() - w[hock].position();
        }
        bones
    };
    let target = Skeleton::new(hock_held(dog("Neck3"))).unwrap();
    let mut bones = hock_held(dog("Neck3"));
    for b in 0..bones.len() {
        if bones[b].name.starts_with("BackLowerLeg") {
            let up = Vec3::new(0.0, 0.15, 0.2);
            bones[b].rest.translation += up;
            let paw = (0..bones.len()).find(|&c| bones[c].parent == Some(b)).unwrap();
            bones[paw].rest.translation -= up;
        }
    }
    let source = Skeleton::new(bones).unwrap();
    let down = 0.17;
    let mut pose = source.rest();
    pose[source.index_of("Body").unwrap()].translation.y -= down;
    let rest = source.world(&source.rest());
    for side in ["L", "R"] {
        let at = |n: &str| source.index_of(&format!("{n}.{side}")).unwrap();
        let limb = dittomo::contact::Limb {
            upper: at("BackUpperLeg"),
            lower: at("BackLowerLeg"),
            end: at("IKBackLeg"),
            bend: Vec3::X,
        };
        let (hip, paw) = (rest[limb.upper].position(), rest[limb.end].position());
        dittomo::contact::reach(&source, &mut pose, &limb, Vec3::new(paw.x, paw.y, hip.z));
    }
    let hock = source.index_of("BackLowerLeg.L").unwrap();
    assert!(source.world(&pose)[hock].position().y > 0.3, "the source's hock is up");
    let clip = Clip {
        times: (0..30).map(|i| i as f32 / 30.0).collect(),
        poses: vec![pose; 30],
    };
    let map = Mapping::new(&source, &target);
    let played = |fixes: Fixes| {
        let out = map.apply_with(&source, &target, &clip, fixes).unwrap();
        let world = target.world(&out.poses[15]);
        let at = |n: &str| world[target.index_of(n).unwrap()].position();
        (at("BackLowerLeg.L"), at("IKBackLeg.L"))
    };
    let (sunk, _) = played(Fixes {
        floor: false,
        ..Fixes::ALL
    });
    assert!(sunk.y < -0.1, "the hock at {sunk} unfixed");
    let (hock, paw) = played(Fixes::ALL);
    assert!(hock.y.abs() < 1e-3, "the hock at {hock}");
    assert!(
        (hock.x - 0.127).abs() < 0.02 && hock.z < -0.35,
        "the hock behind, beside the hip: {hock}"
    );
    assert!(paw.y > 0.045 && paw.y < 0.1, "the paw at {paw}");
}

#[test]
fn a_paw_ending_at_the_wrist_is_still_a_leg() {
    // The fox's forelegs end at the wrist, a tenth of its height up, with no bone below: the paw is skin. Before, they
    // weren't legs, so nothing kept them out of the floor, and a cow lying down on its side, played on the fox, put its
    // forepaws a seventh of its height under it.
    let fox = fox();
    let name = |b: usize| fox.bones()[b].name.clone();
    let mut ends: Vec<String> = dittomo::contact::legs(&fox).iter().map(|l| name(l.end)).collect();
    ends.sort();
    assert_eq!(ends, ["b_LeftFoot02", "b_LeftHand", "b_RightFoot02", "b_RightHand"]);
}

/// A chain of bones hanging from a bone, each named with its world position.
type Chain<'a> = (&'a str, &'a [(&'a str, [f32; 3])]);

/// A dog as the Quaternius animals are laid out: front legs hanging ahead of and below a spine bone, ears of four bones
/// rising from `ears` (the neck beside the head, or the head), and handles for IK and its poles at the ground, named
/// for a side and an end.
fn dog(ears: &str) -> Skeleton {
    let mut j: Joints = vec![
        joint("AnimalArmature", None, [0.0, 0.0, 0.0]),
        joint("Body", Some("AnimalArmature"), [0.0, 0.135, 0.0]),
        joint("Back", Some("Body"), [0.0, 0.594, -0.228]),
        joint("Torso", Some("Back"), [0.0, 0.56, -0.058]),
        joint("Torso2", Some("Torso"), [0.0, 0.553, 0.111]),
        joint("Torso3", Some("Torso2"), [0.0, 0.549, 0.326]),
        joint("Neck1", Some("Torso3"), [0.0, 0.623, 0.436]),
        joint("Neck2", Some("Neck1"), [0.0, 0.697, 0.495]),
        joint("Neck3", Some("Neck2"), [0.0, 0.783, 0.548]),
        joint("Head", Some("Neck3"), [0.0, 0.86, 0.606]),
        joint("Tail1", Some("Back"), [0.0, 0.579, -0.27]),
        joint("Tail2", Some("Tail1"), [0.0, 0.633, -0.328]),
        joint("Tail3", Some("Tail2"), [0.0, 0.706, -0.387]),
        joint("Tail4", Some("Tail3"), [0.0, 0.783, -0.431]),
    ];
    for (side, s) in [("L", 1.0), ("R", -1.0)] {
        let chains: [Chain; 5] = [
            (
                ears,
                &[
                    ("Ear1", [0.059, 0.913, 0.607]),
                    ("Ear2", [0.075, 0.945, 0.594]),
                    ("Ear3", [0.086, 0.973, 0.603]),
                    ("Ear4", [0.093, 1.0, 0.613]),
                ],
            ),
            (
                "Torso2",
                &[
                    ("FrontShoulder", [0.065, 0.445, 0.328]),
                    ("FrontUpperLeg", [0.099, 0.449, 0.328]),
                    ("FrontLowerLeg", [0.1, 0.333, 0.296]),
                ],
            ),
            (
                "Back",
                &[
                    ("BackShoulder", [0.082, 0.535, -0.205]),
                    ("BackLeg", [0.125, 0.54, -0.204]),
                    ("BackUpperLeg", [0.122, 0.388, -0.106]),
                    ("BackLowerLeg", [0.127, 0.175, -0.409]),
                ],
            ),
            (
                "AnimalArmature",
                &[("IKFrontLeg", [0.099, 0.049, 0.35]), ("FF", [0.099, 0.001, 0.39])],
            ),
            (
                "AnimalArmature",
                &[("IKBackLeg", [0.125, 0.045, -0.342]), ("FFB", [0.125, 0.0, -0.294])],
            ),
        ];
        for (from, chain) in chains {
            let mut parent = from.to_owned();
            for &(part, [x, y, z]) in chain {
                let name = format!("{part}.{side}");
                j.push(joint(name.clone(), Some(&parent), [s * x, y, z]));
                parent = name;
            }
        }
        j.push(joint(
            format!("PoleTarget.{side}"),
            Some("Body"),
            [s * 0.072, 0.251, 1.293],
        ));
        j.push(joint(
            format!("PoleTargetBack.{side}"),
            Some("Body"),
            [s * 0.072, 0.25, 1.389],
        ));
    }
    placed(&j)
}

#[test]
fn a_fox_and_a_dog_built_by_different_hands_find_each_others_legs() {
    // Measured from the Khronos Fox and the Quaternius Husky. Before, the dog's front legs, hanging ahead of the bone
    // they hang from, weren't taken for limbs; its ears were taken for the arms, and its IK handles, a front one on
    // one side and a back one on the other, for a pair of legs. The paws, IK handles under the root, end the legs.
    let (fox, dog) = (fox(), dog("Neck3"));
    // Which leg a bone is in, and on which side; none for the trunk, ears, tail and pole targets.
    let leg = |name: &str| -> Option<(&str, &str)> {
        let side = if name.contains("Left") || name.ends_with(".L") {
            "left"
        } else if name.contains("Right") || name.ends_with(".R") {
            "right"
        } else {
            return None;
        };
        let front = ["Arm", "Hand", "Front", "FF."];
        if front.iter().any(|f| name.contains(f)) {
            Some(("front", side))
        } else if ["Leg0", "Foot", "Back", "FFB"].iter().any(|f| name.contains(f)) {
            Some(("hind", side))
        } else {
            None
        }
    };
    for (from, to) in [(&fox, &dog), (&dog, &fox)] {
        let map = Mapping::new(from, to);
        let name = |s: &Skeleton, b: usize| s.bones()[b].name.clone();
        let (mut front, mut hind) = (0, 0);
        for b in 0..to.len() {
            let Some(a) = map.source_of(b) else { continue };
            let (got, want) = (leg(&name(from, a)), leg(&name(to, b)));
            assert_eq!(got, want, "{} plays {}", name(to, b), name(from, a));
            match want {
                Some(("front", _)) => front += 1,
                Some(_) => hind += 1,
                None => {}
            }
        }
        assert!(
            front >= 4 && hind >= 6,
            "{front} front and {hind} hind leg bones played"
        );
        let source = |n: &str| {
            let b = to.bones().iter().position(|b| b.name == n).expect("bone");
            map.source_of(b).map(|a| name(from, a))
        };
        let (head, chest) = if to.len() == dog.len() {
            (("Head", "b_Head_05"), ("Torso2", "b_Spine02_03"))
        } else {
            (("b_Head_05", "Head"), ("b_Spine02_03", "Torso2"))
        };
        assert_eq!(source(head.0).as_deref(), Some(head.1));
        assert_eq!(source(chest.0).as_deref(), Some(chest.1));
    }
}

#[test]
fn forelegs_a_hair_apart_in_length_play_alike() {
    // The Khronos Fox's right foreleg is a few millionths shorter than its left. Its right forepaw stood just
    // short of where the hand begins, and so was matched by how far along the leg, the left by where: on the husky, the
    // left forepaw played the paw and the right the toe beyond it.
    let (fox, dog) = (fox_with_right_hand_up(1e-4), dog("Neck3"));
    let map = Mapping::new(&fox, &dog);
    let source = |n: &str| {
        let b = dog.bones().iter().position(|b| b.name == n).expect("bone");
        map.source_of(b).map(|a| fox.bones()[a].name.clone())
    };
    for bone in ["FrontShoulder", "FrontUpperLeg", "FrontLowerLeg", "IKFrontLeg", "FF"] {
        let (left, right) = (source(&format!("{bone}.L")), source(&format!("{bone}.R")));
        assert_eq!(left.map(|n| n.replace("Left", "Right")), right, "{bone}");
    }
    assert_eq!(source("IKFrontLeg.R").as_deref(), Some("b_RightHand"));
}

#[test]
fn paws_held_from_the_root_go_with_their_legs() {
    // The Quaternius animals hang each paw from the root, placed by IK at the end of its leg. Played as they were, the
    // paws stayed where they stood while the body walked off on its legs; played as the ends of their legs, they go
    // with them, and come back under the root as the rig has them.
    let (fox, dog) = (fox(), dog("Neck3"));
    let map = Mapping::new(&fox, &dog);
    let (hip, thigh) = (fox.index_of("b_Hip_01").unwrap(), fox.index_of("b_LeftLeg01").unwrap());
    let n = 10;
    let clip = Clip {
        times: (0..n).map(|f| f as f32 / 10.0).collect(),
        poses: (0..n)
            .map(|f| {
                let mut pose = fox.rest();
                pose[hip].translation.z += 0.05 * f as f32;
                pose[thigh].rotation = Quat::from_rotation_x(0.4 * (f as f32 * 0.7).sin());
                pose
            })
            .collect(),
    };
    let out = map.apply(&fox, &dog, &clip).expect("plays");
    let rest = dog.world(&dog.rest());
    for side in ["L", "R"] {
        for (paw, leg) in [("IKFrontLeg", "FrontLowerLeg"), ("IKBackLeg", "BackLowerLeg")] {
            let (p, l) = (
                dog.index_of(&format!("{paw}.{side}")).unwrap(),
                dog.index_of(&format!("{leg}.{side}")).unwrap(),
            );
            assert_eq!(dog.bones()[p].parent, Some(0));
            let reach = rest[p].position().distance(rest[l].position());
            let mut went = 0.0f32;
            for pose in &out.poses {
                let w = dog.world(pose);
                let off = w[p].position().distance(w[l].position()) - reach;
                assert!(off.abs() < 1e-4, "{paw}.{side} {off} from the end of its leg");
                went = went.max(w[p].position().z - rest[p].position().z);
            }
            assert!(went > 0.2, "{paw}.{side} went {went}");
        }
    }
}

#[test]
fn hips_with_the_legs_above_them_play_hips_with_the_legs_below_untwisted() {
    // The fox's hips start its spine, tail and legs all above them, the husky's its legs below. Fitting the one's
    // directions to the other's turned the husky's hips half round, legs swapped, and rolled them as the fox's hips
    // swayed. They turn from rest as the fox's do.
    let (fox, dog) = (fox(), dog("Neck3"));
    let map = Mapping::new(&fox, &dog);
    let (hip, back) = (fox.index_of("b_Hip_01").unwrap(), dog.index_of("Back").unwrap());
    assert_eq!(map.source_of(back), Some(hip));
    let n = 12;
    let clip = Clip {
        times: (0..n).map(|f| f as f32 / 12.0).collect(),
        poses: (0..n)
            .map(|f| {
                let t = f as f32 * 0.5;
                let mut pose = fox.rest();
                pose[hip].rotation = Quat::from_rotation_z(0.2 * t.sin()) * Quat::from_rotation_x(0.1 * t.cos());
                pose
            })
            .collect(),
    };
    let out = map.apply(&fox, &dog, &clip).expect("plays");
    let (fox_rest, dog_rest) = (fox.world(&fox.rest()), dog.world(&dog.rest()));
    for (f, pose) in out.poses.iter().enumerate() {
        let swayed = fox.world(&clip.poses[f])[hip].rotation * fox_rest[hip].rotation.inverse();
        let played = dog.world(pose)[back].rotation * dog_rest[back].rotation.inverse();
        let off = swayed.angle_between(played).to_degrees();
        assert!(off < 1.0, "frame {f}: the husky's hips {off}° from the fox's sway");
    }
}

#[test]
fn legs_bent_otherwise_at_rest_keep_their_paws_out_of_the_floor() {
    // The fox's hind legs stand straighter than the husky's: played at the fox's angles, the husky's hind paws reached
    // under the floor, and weren't planted at all, as their IK handles pair with nothing (the toes under them do).
    let (fox, dog) = (fox(), dog("Neck3"));
    let map = Mapping::new(&fox, &dog);
    let thighs = ["b_LeftLeg01", "b_RightLeg01"].map(|n| fox.index_of(n).unwrap());
    let n = 24;
    let clip = Clip {
        times: (0..n).map(|f| f as f32 / 24.0).collect(),
        poses: (0..n)
            .map(|f| {
                let t = f as f32 / n as f32 * TAU;
                let mut pose = fox.rest();
                for (i, &thigh) in thighs.iter().enumerate() {
                    pose[thigh].rotation = Quat::from_rotation_x(0.35 * (t + i as f32 * std::f32::consts::PI).sin());
                }
                pose
            })
            .collect(),
    };
    let out = map.apply_with(&fox, &dog, &clip, Fixes::default()).expect("plays");
    let (fox_rest, dog_rest) = (fox.world(&fox.rest()), dog.world(&dog.rest()));
    let paw = fox.index_of("b_LeftFoot02").unwrap();
    let dipped = clip
        .poses
        .iter()
        .map(|p| fox_rest[paw].position().y - fox.world(p)[paw].position().y)
        .fold(0.0f32, f32::max);
    for side in ["L", "R"] {
        let ik = dog.index_of(&format!("IKBackLeg.{side}")).unwrap();
        for (f, pose) in out.poses.iter().enumerate() {
            let sunk = dog_rest[ik].position().y - dog.world(pose)[ik].position().y;
            assert!(
                sunk < dipped + 1e-3,
                "frame {f}: IKBackLeg.{side} {sunk} under rest, the fox's paw {dipped}"
            );
        }
    }
}

#[test]
fn a_paw_coming_down_from_above_the_floor_at_rest_stops_on_it() {
    // The Quaternius horse stands at rest with its hooves above its root, and plays its clips with them down on the
    // root's level. The fox's hind paws, its lowest bones, stand nearly on its floor at rest; planted as far down from
    // there as the horse's hooves came, they went through it. They come no further under the floor than the hooves do.
    let mut bones = dog("Neck3").bones().to_vec();
    let raised = 0.04;
    for b in &mut bones {
        if b.name.starts_with("IKBackLeg") {
            b.rest.translation.y += raised;
        }
    }
    let (dog, fox) = (Skeleton::new(bones).unwrap(), fox());
    let map = Mapping::new(&dog, &fox);
    let n = 20;
    let clip = Clip {
        times: (0..n).map(|f| f as f32 / 30.0).collect(),
        poses: (0..n)
            .map(|f| {
                let mut pose = dog.rest();
                for b in 0..dog.len() {
                    if dog.bones()[b].name.starts_with("IKBackLeg") {
                        pose[b].translation.y -= raised * (f as f32 / (n - 1) as f32).min(1.0);
                    }
                }
                pose
            })
            .collect(),
    };
    let out = map.apply_with(&dog, &fox, &clip, Fixes::default()).expect("plays");
    let paw = fox.index_of("b_LeftFoot02").unwrap();
    let fox_rest = fox.world(&fox.rest());
    let floor = fox_rest.iter().map(|w| w.position().y).fold(f32::MAX, f32::min);
    for (f, pose) in out.poses.iter().enumerate() {
        let y = fox.world(pose)[paw].position().y;
        assert!(
            y > floor - 1e-3,
            "frame {f}: the fox's paw {y} under the floor at {floor}"
        );
    }
    let y = fox.world(&out.poses[n - 1])[paw].position().y;
    assert!(
        y < fox_rest[paw].position().y - 0.01,
        "the fox's paw {y} didn't come down"
    );
}

#[test]
fn a_head_with_the_ears_beside_it_plays_one_with_the_ears_on_it() {
    // The Quaternius husky hangs its ears from the neck beside the head, and the horse from the head. The match keeps
    // what's under a bone under its partner, so it paired the eight ear bones and left the heads out.
    let (beside, on) = (dog("Neck3"), dog("Head"));
    for (from, to) in [(&beside, &on), (&on, &beside)] {
        let map = Mapping::new(from, to);
        for (b, bone) in to.bones().iter().enumerate() {
            let played = map.source_of(b).map(|a| from.bones()[a].name.as_str());
            assert_eq!(played, Some(bone.name.as_str()), "{} plays {played:?}", bone.name);
        }
    }
}

#[test]
fn wide_hips_with_legs_that_stop_at_the_knee_still_start_the_legs() {
    // A toy robot's legs hang from far out at the sides of its hips and stop at the knee, its feet hanging from the
    // root for IK (one listed before its leg, one after) and ending the legs all the same, and its hands reach far
    // forward; a slim figure's legs hang from close in and go on to the toes. By
    // length and reach the figure's shin looks more like the robot's thigh than the figure's thigh does; but the legs
    // start where they hang from the hips. (Both laid out as glTF's CesiumMan and RobotExpressive are.)
    let rig = |j: &[(&str, Option<&str>, [f32; 3])]| placed(&j.iter().map(|&(n, p, at)| joint(n, p, at)).collect());
    let figure = rig(&[
        ("Z_UP", None, [0.0, 0.0, 0.0]),
        ("Skeleton_torso_joint_1", Some("Z_UP"), [0.004, 0.571, 0.0]),
        (
            "Skeleton_torso_joint_2",
            Some("Skeleton_torso_joint_1"),
            [0.004, 0.692, 0.009],
        ),
        ("torso_joint_3", Some("Skeleton_torso_joint_2"), [0.004, 0.903, -0.004]),
        ("Skeleton_neck_joint_1", Some("torso_joint_3"), [0.004, 0.956, 0.005]),
        (
            "Skeleton_neck_joint_2",
            Some("Skeleton_neck_joint_1"),
            [0.004, 1.0, 0.007],
        ),
        (
            "Skeleton_arm_joint_L__4_",
            Some("torso_joint_3"),
            [0.081, 0.903, -0.004],
        ),
        (
            "Skeleton_arm_joint_L__3_",
            Some("Skeleton_arm_joint_L__4_"),
            [0.262, 0.811, -0.013],
        ),
        (
            "Skeleton_arm_joint_L__2_",
            Some("Skeleton_arm_joint_L__3_"),
            [0.382, 0.735, 0.056],
        ),
        ("Skeleton_arm_joint_R", Some("torso_joint_3"), [-0.072, 0.903, -0.004]),
        (
            "Skeleton_arm_joint_R__2_",
            Some("Skeleton_arm_joint_R"),
            [-0.253, 0.811, -0.013],
        ),
        (
            "Skeleton_arm_joint_R__3_",
            Some("Skeleton_arm_joint_R__2_"),
            [-0.374, 0.735, 0.056],
        ),
        ("leg_joint_L_1", Some("Skeleton_torso_joint_1"), [0.061, 0.516, 0.02]),
        ("leg_joint_L_2", Some("leg_joint_L_1"), [0.069, 0.296, 0.057]),
        ("leg_joint_L_3", Some("leg_joint_L_2"), [0.07, 0.072, -0.004]),
        ("leg_joint_L_5", Some("leg_joint_L_3"), [0.071, 0.018, 0.023]),
        ("leg_joint_R_1", Some("Skeleton_torso_joint_1"), [-0.053, 0.516, 0.02]),
        ("leg_joint_R_2", Some("leg_joint_R_1"), [-0.061, 0.296, 0.057]),
        ("leg_joint_R_3", Some("leg_joint_R_2"), [-0.062, 0.072, -0.004]),
        ("leg_joint_R_5", Some("leg_joint_R_3"), [-0.063, 0.018, 0.023]),
    ]);
    let robot = rig(&[
        ("RootNode", None, [0.0, 0.002, 0.0]),
        ("Bone", Some("RootNode"), [-0.001, 0.0, -0.066]),
        ("Foot.L", Some("Bone"), [0.213, 0.01, -0.046]),
        ("Body", Some("Bone"), [0.001, 0.463, 0.038]),
        ("Hips", Some("Body"), [0.0, 0.431, 0.0]),
        ("Abdomen", Some("Hips"), [0.0, 0.538, -0.026]),
        ("Torso", Some("Abdomen"), [0.0, 0.701, -0.027]),
        ("Neck", Some("Torso"), [0.0, 0.896, -0.006]),
        ("Head", Some("Neck"), [-0.006, 1.0, -0.004]),
        ("Shoulder.L", Some("Torso"), [0.196, 0.799, -0.006]),
        ("UpperArm.L", Some("Shoulder.L"), [0.207, 0.773, -0.006]),
        ("LowerArm.L", Some("UpperArm.L"), [0.302, 0.577, -0.044]),
        ("Palm2.L", Some("LowerArm.L"), [0.457, 0.305, 0.293]),
        ("Middle1.L", Some("Palm2.L"), [0.418, 0.227, 0.381]),
        ("Middle2.L", Some("Middle1.L"), [0.354, 0.219, 0.382]),
        ("Thumb.L", Some("LowerArm.L"), [0.414, 0.369, 0.313]),
        ("Thumb2.L", Some("Thumb.L"), [0.373, 0.379, 0.331]),
        ("Palm1.L", Some("LowerArm.L"), [0.457, 0.305, 0.293]),
        ("Index.L", Some("Palm1.L"), [0.416, 0.270, 0.424]),
        ("Index2.L", Some("Index.L"), [0.353, 0.265, 0.424]),
        ("Palm3.L", Some("LowerArm.L"), [0.457, 0.305, 0.293]),
        ("Ring1.L", Some("Palm3.L"), [0.422, 0.176, 0.343]),
        ("Ring2.L", Some("Ring1.L"), [0.358, 0.171, 0.342]),
        ("Shoulder.R", Some("Torso"), [-0.214, 0.799, -0.006]),
        ("UpperArm.R", Some("Shoulder.R"), [-0.229, 0.768, -0.006]),
        ("LowerArm.R", Some("UpperArm.R"), [-0.314, 0.596, 0.022]),
        ("Palm2.R", Some("LowerArm.R"), [-0.433, 0.257, 0.314]),
        ("Middle1.R", Some("Palm2.R"), [-0.387, 0.159, 0.376]),
        ("Middle2.R", Some("Middle1.R"), [-0.325, 0.145, 0.364]),
        ("Thumb.R", Some("LowerArm.R"), [-0.384, 0.313, 0.340]),
        ("Thumb2.R", Some("Thumb.R"), [-0.356, 0.333, 0.371]),
        ("Palm1.R", Some("LowerArm.R"), [-0.433, 0.257, 0.314]),
        ("Index.R", Some("Palm1.R"), [-0.375, 0.192, 0.426]),
        ("Index2.R", Some("Index.R"), [-0.313, 0.180, 0.424]),
        ("Palm3.R", Some("LowerArm.R"), [-0.433, 0.257, 0.314]),
        ("Ring1.R", Some("Palm3.R"), [-0.400, 0.118, 0.328]),
        ("Ring2.R", Some("Ring1.R"), [-0.339, 0.111, 0.312]),
        ("UpperLeg.L", Some("Body"), [0.212, 0.496, -0.023]),
        ("LowerLeg.L", Some("UpperLeg.L"), [0.227, 0.306, 0.053]),
        ("UpperLeg.R", Some("Body"), [-0.216, 0.496, -0.023]),
        ("LowerLeg.R", Some("UpperLeg.R"), [-0.24, 0.306, 0.051]),
        ("Foot.R", Some("Bone"), [-0.215, 0.01, -0.046]),
        ("PoleTarget.L", Some("Bone"), [0.306, 0.356, 0.509]),
        ("PoleTarget.R", Some("Bone"), [-0.371, 0.356, 0.509]),
    ]);
    for (from, to, pairs) in [
        (
            &figure,
            &robot,
            [
                ("leg_joint_L_1", "UpperLeg.L"),
                ("leg_joint_L_2", "LowerLeg.L"),
                ("leg_joint_L_3", "Foot.L"),
                ("leg_joint_R_1", "UpperLeg.R"),
                ("leg_joint_R_2", "LowerLeg.R"),
                ("leg_joint_R_3", "Foot.R"),
            ],
        ),
        (
            &robot,
            &figure,
            [
                ("UpperLeg.L", "leg_joint_L_1"),
                ("LowerLeg.L", "leg_joint_L_2"),
                ("Foot.L", "leg_joint_L_3"),
                ("UpperLeg.R", "leg_joint_R_1"),
                ("LowerLeg.R", "leg_joint_R_2"),
                ("Foot.R", "leg_joint_R_3"),
            ],
        ),
    ] {
        assert_plays(from, to, &Mapping::new(from, to), &pairs);
    }
    // The figure's arms are named unevenly, `Skeleton_arm_joint_L__4_` against `Skeleton_arm_joint_R`, but they're
    // still a pair, the chest being where they hang from.
    let mixamo = mixamo_like();
    assert_plays(
        &figure,
        &mixamo,
        &Mapping::new(&figure, &mixamo),
        &[
            ("torso_joint_3", "Spine2"),
            ("Skeleton_neck_joint_1", "Neck"),
            ("Skeleton_arm_joint_L__4_", "LeftArm"),
            ("Skeleton_arm_joint_L__3_", "LeftForeArm"),
            ("Skeleton_arm_joint_R", "RightArm"),
            ("Skeleton_arm_joint_R__2_", "RightForeArm"),
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
fn a_sleeve_beside_the_upper_arm_comes_down_with_it() {
    // VRoid hangs each sleeve from a helper beside the upper arm, under the collarbone, which its app turns with the
    // arm. Left at rest, the sleeves stayed out in a T-pose as the arms came down.
    let from = mixamo_like();
    let to = {
        let mut bones = from.bones().to_vec();
        for side in ["Left", "Right"] {
            let (shoulder, arm) = (
                from.index_of(&format!("{side}Shoulder")).unwrap(),
                from.index_of(&format!("{side}Arm")).unwrap(),
            );
            let out = from.bones()[from.index_of(&format!("{side}ForeArm")).unwrap()]
                .rest
                .translation
                * 0.5;
            let at = bones[arm].rest.translation;
            bones.push(Bone {
                name: format!("{side}Sleeve"),
                parent: Some(shoulder),
                rest: Transform::new(at, Quat::IDENTITY),
            });
            let sleeve = bones.len() - 1;
            bones.push(Bone {
                name: format!("{side}SleeveEnd"),
                parent: Some(sleeve),
                rest: Transform::new(out, Quat::IDENTITY),
            });
            bones.push(Bone {
                name: format!("{side}Roll"),
                parent: Some(shoulder),
                rest: Transform::new(at, Quat::IDENTITY),
            });
        }
        Skeleton::new(bones).unwrap()
    };
    let map = Mapping::new(&from, &to);
    let mut pose = from.rest();
    for (side, down) in [("Left", -1.2f32), ("Right", 1.2)] {
        pose[from.index_of(&format!("{side}Arm")).unwrap()].rotation = Quat::from_rotation_z(down);
    }
    let clip = Clip {
        times: vec![0.0],
        poses: vec![pose],
    };
    let played = map.apply(&from, &to, &clip).unwrap();
    let w = to.world(&played.poses[0]);
    let at = |n: &str| w[to.index_of(n).unwrap()];
    for side in ["Left", "Right"] {
        let n = |s: &str| format!("{side}{s}");
        let arm = (at(&n("ForeArm")).position() - at(&n("Arm")).position()).normalize();
        let sleeve = (at(&n("SleeveEnd")).position() - at(&n("Sleeve")).position()).normalize();
        let off = arm.dot(sleeve).clamp(-1.0, 1.0).acos().to_degrees();
        assert!(off < 1.0, "the {side} sleeve is {off:.1}° off the arm");
        let roll = at(&n("Roll")).rotation.dot(at(&n("Arm")).rotation).abs();
        assert!(roll > 0.9999, "the {side} roll bone didn't turn with the arm");
    }
}

#[test]
fn a_chest_bends_the_same_whether_its_collarbones_sit_above_it_or_beside_it() {
    // Mixamo's collarbones start just above the top spine bone; a cartoon's start out at its sides, level with it.
    // The chest is aimed by its neck alone, so one rig's collarbones don't tip the other's chest.
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

#[test]
fn hips_stay_level_whether_the_hip_joints_sit_below_them_or_behind() {
    // Mixamo's hip joints sit just below its hips bone; another rig's sit level with its hips bone and behind it. Aimed
    // at its thighs, the pelvis would tip some thirty degrees at rest; aimed at its spine, it stands as the
    // source's does.
    let from = mixamo_like();
    let to = {
        let mut j: Joints = vec![
            joint("Hips", None, [0.0, 1.0, 0.04]),
            joint("Spine", Some("Hips"), [0.0, 1.12, 0.04]),
            joint("Chest", Some("Spine"), [0.0, 1.35, 0.03]),
            joint("Neck", Some("Chest"), [0.0, 1.52, 0.02]),
            joint("Head", Some("Neck"), [0.0, 1.62, 0.03]),
        ];
        for (side, x) in [("Left", 1.0f32), ("Right", -1.0)] {
            let n = |s: &str| format!("{side}{s}");
            j.push(joint(n("Shoulder"), Some("Chest"), [0.05 * x, 1.42, 0.0]));
            j.push(joint(n("Arm"), Some(&n("Shoulder")), [0.18 * x, 1.44, 0.0]));
            j.push(joint(n("ForeArm"), Some(&n("Arm")), [0.46 * x, 1.44, 0.0]));
            j.push(joint(n("Hand"), Some(&n("ForeArm")), [0.72 * x, 1.44, 0.0]));
            j.push(joint(n("UpLeg"), Some("Hips"), [0.09 * x, 1.0, -0.04]));
            j.push(joint(n("Leg"), Some(&n("UpLeg")), [0.09 * x, 0.52, -0.04]));
            j.push(joint(n("Foot"), Some(&n("Leg")), [0.09 * x, 0.08, -0.06]));
            j.push(joint(n("ToeBase"), Some(&n("Foot")), [0.09 * x, 0.0, 0.04]));
        }
        placed(&j)
    };
    let map = Mapping::new(&from, &to);
    let (hips, t_hips) = (from.index_of("Hips").unwrap(), to.index_of("Hips").unwrap());
    assert_eq!(map.source_of(t_hips), Some(hips));
    let clip = Clip {
        times: vec![0.0],
        poses: vec![from.rest()],
    };
    let played = &map.apply(&from, &to, &clip).unwrap().poses[0];
    let (now, rest) = (to.world(played)[t_hips].rotation, to.world(&to.rest())[t_hips].rotation);
    let tipped = now.angle_between(rest).to_degrees();
    assert!(tipped < 1.0, "the pelvis tips {tipped:.1}° at rest");
}

#[test]
fn a_curved_back_stands_as_it_was_built_and_bends_as_the_source_bends() {
    // The target's back curves: its spine leans back from the hips and its neck forward from the chest, where the
    // source's rises straight. Every rig stands upright at rest, so the curve is anatomy: played the source's rest, the
    // target stands at its own, and bent, it bends as far from there.
    let from = mixamo_like();
    let to = {
        let mut j: Joints = vec![
            joint("Hips", None, [0.0, 1.0, 0.0]),
            joint("Spine", Some("Hips"), [0.0, 1.1, -0.04]),
            joint("Chest", Some("Spine"), [0.0, 1.33, -0.05]),
            joint("Neck", Some("Chest"), [0.0, 1.5, 0.0]),
            joint("Head", Some("Neck"), [0.0, 1.6, 0.03]),
        ];
        for (side, x) in [("Left", 1.0f32), ("Right", -1.0)] {
            let n = |s: &str| format!("{side}{s}");
            j.push(joint(n("Shoulder"), Some("Chest"), [0.06 * x, 1.44, -0.04]));
            j.push(joint(n("Arm"), Some(&n("Shoulder")), [0.18 * x, 1.44, -0.04]));
            j.push(joint(n("ForeArm"), Some(&n("Arm")), [0.46 * x, 1.44, -0.04]));
            j.push(joint(n("Hand"), Some(&n("ForeArm")), [0.72 * x, 1.44, -0.04]));
            j.push(joint(n("UpLeg"), Some("Hips"), [0.09 * x, 0.95, 0.0]));
            j.push(joint(n("Leg"), Some(&n("UpLeg")), [0.09 * x, 0.52, 0.0]));
            j.push(joint(n("Foot"), Some(&n("Leg")), [0.09 * x, 0.08, 0.0]));
            j.push(joint(n("ToeBase"), Some(&n("Foot")), [0.09 * x, 0.0, 0.1]));
        }
        placed(&j)
    };
    let map = Mapping::new(&from, &to);
    let pairs: Vec<(usize, usize)> = [("Hips", "Hips"), ("Spine2", "Chest"), ("Head", "Head")]
        .iter()
        .map(|&(a, b)| (from.index_of(a).unwrap(), to.index_of(b).unwrap()))
        .collect();
    for &(a, b) in &pairs {
        assert_eq!(map.source_of(b), Some(a));
    }
    let bent = {
        let mut pose = from.rest();
        let spine = from.index_of("Spine").unwrap();
        pose[spine].rotation = Quat::from_rotation_x(0.5) * pose[spine].rotation;
        pose
    };
    let clip = Clip {
        times: vec![0.0, 1.0],
        poses: vec![from.rest(), bent],
    };
    let played = map.apply(&from, &to, &clip).unwrap();
    let (s_rest, t_rest) = (from.world(&from.rest()), to.world(&to.rest()));
    for (f, pose) in clip.poses.iter().enumerate() {
        let (sw, tw) = (from.world(pose), to.world(&played.poses[f]));
        for &(a, b) in &pairs {
            let s_turn = sw[a].rotation * s_rest[a].rotation.inverse();
            let t_turn = tw[b].rotation * t_rest[b].rotation.inverse();
            let off = s_turn.angle_between(t_turn).to_degrees();
            assert!(
                off < 1.0,
                "{} turns {off:.1}° away from the source's on frame {f}",
                to.bones()[b].name
            );
        }
    }
}

#[test]
fn a_neck_carried_high_stays_high_under_one_carried_low() {
    // A fox carries its head forward, low; a llama's neck rises straight up from its shoulders. Each is that animal
    // standing as it stands, so the llama walking as the fox walks holds its neck up and turns it as the fox turns its
    // own. Before, the shoulders of a body with four legs and no arms weren't found, nor the neck above them, and the
    // llama's neck was laid forward along the fox's.
    let fox = fox();
    let llama = {
        let mut bones = dog("Head").bones().to_vec();
        for b in &mut bones {
            if ["Neck2", "Neck3", "Head"].contains(&b.name.as_str()) {
                b.rest.translation = Vec3::new(0.0, 0.12, 0.01);
            }
        }
        Skeleton::new(bones).unwrap()
    };
    let map = Mapping::new(&fox, &llama);
    let (neck, head) = (fox.index_of("b_Neck_04").unwrap(), fox.index_of("b_Head_05").unwrap());
    let pairs: Vec<(usize, usize)> = (0..llama.len())
        .filter_map(|b| map.source_of(b).filter(|a| [neck, head].contains(a)).map(|a| (a, b)))
        .collect();
    assert!(pairs.len() >= 2, "the neck and head play {pairs:?}");
    let nod = {
        let mut pose = fox.rest();
        pose[neck].rotation = Quat::from_rotation_x(0.4) * pose[neck].rotation;
        pose
    };
    let clip = Clip {
        times: vec![0.0, 1.0],
        poses: vec![fox.rest(), nod],
    };
    let played = map.apply(&fox, &llama, &clip).unwrap();
    let (s_rest, t_rest) = (fox.world(&fox.rest()), llama.world(&llama.rest()));
    for (f, pose) in clip.poses.iter().enumerate() {
        let (sw, tw) = (fox.world(pose), llama.world(&played.poses[f]));
        for &(a, b) in &pairs {
            let s_turn = sw[a].rotation * s_rest[a].rotation.inverse();
            let t_turn = tw[b].rotation * t_rest[b].rotation.inverse();
            let off = s_turn.angle_between(t_turn).to_degrees();
            assert!(
                off < 1.0,
                "{} turns {off:.1}° away from the fox's on frame {f}",
                llama.bones()[b].name
            );
        }
    }
}

/// The Quaternius Alpaca, as its skin was bound, over its height: the dog's bones, its neck rising straight up from
/// its shoulders, as high again as its legs are long.
fn alpaca() -> Skeleton {
    let mut j: Joints = vec![
        joint("AnimalArmature", None, [0.0, 0.0, 0.0]),
        joint("Body", Some("AnimalArmature"), [0.0, 0.141, 0.0]),
        joint("Back", Some("Body"), [0.0, 0.483, -0.238]),
        joint("Torso", Some("Back"), [0.0, 0.459, -0.111]),
        joint("Torso2", Some("Torso"), [0.0, 0.442, 0.014]),
        joint("Torso3", Some("Torso2"), [0.0, 0.441, 0.144]),
        joint("Neck1", Some("Torso3"), [0.0, 0.482, 0.247]),
        joint("Neck2", Some("Neck1"), [0.0, 0.594, 0.29]),
        joint("Neck3", Some("Neck2"), [0.0, 0.747, 0.31]),
        joint("Head", Some("Neck3"), [0.0, 0.877, 0.312]),
        joint("Tail1", Some("Back"), [0.0, 0.483, -0.238]),
        joint("Tail2", Some("Tail1"), [0.0, 0.521, -0.279]),
        joint("Tail3", Some("Tail2"), [0.0, 0.507, -0.303]),
    ];
    for (side, s) in [("L", 1.0), ("R", -1.0)] {
        let chains: [Chain; 5] = [
            (
                "Head",
                &[
                    ("Ear1", [0.018, 0.92, 0.321]),
                    ("Ear2", [0.027, 0.95, 0.323]),
                    ("Ear3", [0.035, 0.969, 0.32]),
                    ("Ear4", [0.044, 1.0, 0.317]),
                ],
            ),
            (
                "Torso2",
                &[
                    ("FrontShoulder", [0.034, 0.371, 0.187]),
                    ("FrontUpperLeg", [0.065, 0.375, 0.187]),
                    ("FrontLowerLeg", [0.065, 0.201, 0.211]),
                ],
            ),
            (
                "Back",
                &[
                    ("BackShoulder", [0.038, 0.472, -0.172]),
                    ("BackLeg", [0.076, 0.476, -0.172]),
                    ("BackUpperLeg", [0.076, 0.352, -0.136]),
                    ("BackLowerLeg", [0.076, 0.199, -0.304]),
                ],
            ),
            (
                "AnimalArmature",
                &[("IKFrontLeg", [0.065, 0.061, 0.192]), ("FF", [0.065, 0.022, 0.208])],
            ),
            (
                "AnimalArmature",
                &[("IKBackLeg", [0.076, 0.061, -0.287]), ("FFB", [0.076, 0.022, -0.272])],
            ),
        ];
        for (from, chain) in chains {
            let mut parent = from.to_owned();
            for &(part, [x, y, z]) in chain {
                let name = format!("{part}.{side}");
                j.push(joint(name.clone(), Some(&parent), [s * x, y, z]));
                parent = name;
            }
        }
        j.push(joint(
            format!("PoleTarget.{side}"),
            Some("Body"),
            [s * 0.065, 0.245, 0.631],
        ));
        j.push(joint(
            format!("PoleTargetBack.{side}"),
            Some("Body"),
            [s * 0.065, 0.245, 0.718],
        ));
    }
    placed(&j)
}

#[test]
fn a_long_neck_leaves_the_hind_legs_hip_to_hip() {
    // An alpaca stands twice as tall as its legs are long, a fox not much more: by height, the alpaca's hips stand where
    // the fox's knees do. Before, the fox's knee played the alpaca's hip, its thigh nothing, and the alpaca's hind legs
    // reached back as the fox's shins do. How far down the leg each bone is holds whatever the neck.
    let (fox, alpaca) = (fox(), alpaca());
    let map = Mapping::new(&fox, &alpaca);
    for (side, s) in [("Left", "L"), ("Right", "R")] {
        for (from, to) in [
            ("Leg01", "BackLeg"),
            ("Leg02", "BackUpperLeg"),
            ("Foot01", "BackLowerLeg"),
        ] {
            let (a, b) = (format!("b_{side}{from}"), format!("{to}.{s}"));
            let got = map
                .source_of(alpaca.index_of(&b).unwrap())
                .map(|a| fox.bones()[a].name.clone());
            assert_eq!(got.as_deref(), Some(a.as_str()), "{b}");
        }
    }
}

#[test]
fn a_tail_carried_above_a_lowered_head_is_still_a_tail() {
    // A cow walks with its head low and its tail's root above it: the highest bone is in the tail. Before, the spine
    // was taken up the tail, all four legs hanging from where it starts, and the neck and head, found nowhere, were
    // laid along the fox's.
    let fox = fox();
    let cow = {
        let mut bones = alpaca().bones().to_vec();
        for b in &mut bones {
            if b.name == "Tail2" {
                b.rest.translation = Vec3::new(0.0, 0.6, -0.05);
            }
        }
        Skeleton::new(bones).unwrap()
    };
    let map = Mapping::new(&fox, &cow);
    let rest = Clip {
        times: vec![0.0],
        poses: vec![fox.rest()],
    };
    let played = map.apply(&fox, &cow, &rest).unwrap();
    let (t_rest, t_now) = (cow.world(&cow.rest()), cow.world(&played.poses[0]));
    for name in ["Back", "Torso2", "Neck2", "Head"] {
        let b = cow.index_of(name).unwrap();
        let off = (t_now[b].rotation * t_rest[b].rotation.inverse()).angle_between(Quat::IDENTITY);
        assert!(
            off.to_degrees() < 1.0,
            "{name} turns {:.1}° standing as the fox stands",
            off.to_degrees()
        );
    }
}

#[test]
fn a_body_rolled_over_by_a_bone_the_target_lacks_lies_down() {
    // The Quaternius animals turn the whole body with a bone between the root and the hips that a fox has no match
    // for. Rolled onto its side by it, the alpaca's hips come down near the floor; before, the fox's only turned where
    // they stood, floating at full height with their legs out through the floor.
    let (alpaca, fox) = (alpaca(), fox());
    let map = Mapping::new(&alpaca, &fox);
    let body = alpaca.index_of("Body").unwrap();
    let mut rolled = alpaca.rest();
    rolled[body].rotation = Quat::from_rotation_z(std::f32::consts::FRAC_PI_2) * rolled[body].rotation;
    let clip = Clip {
        times: vec![0.0, 1.0],
        poses: vec![alpaca.rest(), rolled],
    };
    let played = map.apply(&alpaca, &fox, &clip).unwrap();
    let hips = fox.index_of("b_Hip_01").unwrap();
    let (rest, now) = (fox.world(&fox.rest()), fox.world(&played.poses[1]));
    let (from, to) = (rest[hips].position().y, now[hips].position().y);
    assert!(to < 0.5 * from, "the fox's hips stay {to:.2} high of {from:.2}");
}

/// [`mixamo_like`] with its collarbones built otherwise: `collar` gives where each upper arm starts, on the left, and
/// `split` adds a bone halfway along each collarbone.
fn collared(collar: [f32; 3], split: bool) -> Skeleton {
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
        let start = [0.06 * x, 1.45, 0.0];
        let [cx, cy, cz] = collar;
        let end = [cx * x, cy, cz];
        let mut parent = n("Shoulder");
        j.push(joint(n("Shoulder"), Some("Spine2"), start));
        if split {
            let mid = [
                0.5 * (start[0] + end[0]),
                0.5 * (start[1] + end[1]),
                0.5 * (start[2] + end[2]),
            ];
            j.push(joint(n("Shoulder_split"), Some(&parent), mid));
            parent = n("Shoulder_split");
        }
        let (ax, ay, az) = (end[0], end[1], end[2]);
        j.push(joint(n("Arm"), Some(&parent), end));
        j.push(joint(n("ForeArm"), Some(&n("Arm")), [ax + 0.28 * x, ay, az]));
        j.push(joint(n("Hand"), Some(&n("ForeArm")), [ax + 0.54 * x, ay, az]));
        j.push(joint(n("HandMiddle1"), Some(&n("Hand")), [ax + 0.66 * x, ay, az]));
        j.push(joint(n("UpLeg"), Some("Hips"), [0.09 * x, 0.95, 0.0]));
        j.push(joint(n("Leg"), Some(&n("UpLeg")), [0.09 * x, 0.52, 0.0]));
        j.push(joint(n("Foot"), Some(&n("Leg")), [0.09 * x, 0.08, -0.02]));
        j.push(joint(n("ToeBase"), Some(&n("Foot")), [0.09 * x, 0.0, 0.08]));
    }
    placed(&j)
}

/// A clip of `from` turning its left `bone` in the world by each of `turns`, the rest held.
fn turning(from: &Skeleton, bone: &str, turns: &[Quat]) -> Clip {
    let b = from.index_of(bone).expect("bone");
    let mut clip = Clip::default();
    for (k, &q) in turns.iter().enumerate() {
        let mut pose = from.rest();
        pose[b].rotation = q;
        clip.times.push(k as f32);
        clip.poses.push(pose);
    }
    clip
}

#[test]
fn a_collarbone_sloped_back_at_rest_turns_as_the_source_turns() {
    // One rig's collarbones run straight out to the shoulders, another's back to them. A shrug lifts both as it lifts
    // the source's; it isn't turned about another axis to make up for how the bones were built.
    let from = collared([0.18, 1.45, 0.0], false);
    let to = collared([0.16, 1.45, -0.08], false);
    let map = Mapping::new(&from, &to);
    let shrugs = [
        Quat::from_rotation_z(0.35),
        Quat::from_rotation_x(0.3) * Quat::from_rotation_z(0.2),
    ];
    let clip = turning(&from, "LeftShoulder", &shrugs);
    let out = map.apply(&from, &to, &clip).unwrap();
    let (s, t) = (
        from.index_of("LeftShoulder").unwrap(),
        to.index_of("LeftShoulder").unwrap(),
    );
    let (s0, t0) = (from.world(&from.rest())[s].rotation, to.world(&to.rest())[t].rotation);
    for (pose, played) in clip.poses.iter().zip(&out.poses) {
        let ds = from.world(pose)[s].rotation * s0.inverse();
        let dt = to.world(played)[t].rotation * t0.inverse();
        let off = ds.angle_between(dt).to_degrees();
        assert!(off < 0.5, "the collarbone turns {off:.1}° off the source's turn");
    }
}

#[test]
fn a_collarbone_in_two_stays_put_as_the_arm_rises() {
    // A collarbone split halfway, as Mixamo's skeletons in SAN's test set have, sits between the collarbone and the
    // upper arm. It stays with the collarbone as the arm rises, twisting only as the arm twists about it.
    let from = mixamo_like();
    let to = collared([0.18, 1.45, 0.0], true);
    let map = Mapping::new(&from, &to);
    let clip = turning(
        &from,
        "LeftArm",
        &[Quat::from_rotation_z(1.2), Quat::from_rotation_y(0.8)],
    );
    let out = map.apply(&from, &to, &clip).unwrap();
    let (split, arm) = (
        to.index_of("LeftShoulder_split").unwrap(),
        to.index_of("LeftArm").unwrap(),
    );
    let rest = to.world(&to.rest());
    let way = |w: &[dittomo::Placed]| (w[arm].position() - w[split].position()).normalize();
    for played in &out.poses {
        let w = to.world(played);
        let off = way(&w).angle_between(way(&rest)).to_degrees();
        assert!(off < 0.5, "the split collarbone swings {off:.1}° with the arm");
    }
}

#[test]
fn a_robot_named_from_in_front_with_no_hands_maps_by_its_knees() {
    // Godot's platformer robot is named as its viewer sees it: its `r-` limbs are on its own left, where its knees, bent
    // the way it faces, put them. Its arms stop at the elbow, with no hand.
    let mut robot: Joints = vec![
        joint("hip", None, [0.0, 0.332, 0.0]),
        joint("waist", Some("hip"), [0.0, 0.392, 0.0]),
        joint("chest", Some("waist"), [0.0, 0.521, 0.0]),
        joint("neck", Some("chest"), [0.0, 0.718, -0.008]),
        joint("head", Some("neck"), [0.0, 0.793, -0.025]),
        joint("vent", Some("head"), [0.0, 0.987, 0.025]),
    ];
    for (side, x) in [("r", 1.0f32), ("l", -1.0)] {
        let n = |s: &str| format!("{side}-{s}");
        robot.push(joint(n("arm"), Some("chest"), [0.148 * x, 0.644, 0.012]));
        robot.push(joint(n("forearm"), Some(&n("arm")), [0.252 * x, 0.551, -0.036]));
        robot.push(joint(n("thigh"), Some("hip"), [0.055 * x, 0.357, 0.01]));
        robot.push(joint(n("leg"), Some(&n("thigh")), [0.064 * x, 0.256, 0.014]));
        robot.push(joint(n("foot"), Some(&n("leg")), [0.082 * x, 0.032, 0.0]));
    }
    let robot = placed(&robot);
    let man = mixamo_like();
    let pairs = [
        ("LeftArm", "r-arm"),
        ("LeftForeArm", "r-forearm"),
        ("RightArm", "l-arm"),
        ("RightForeArm", "l-forearm"),
        ("LeftUpLeg", "r-thigh"),
        ("LeftLeg", "r-leg"),
        ("LeftFoot", "r-foot"),
        ("RightUpLeg", "l-thigh"),
        ("RightLeg", "l-leg"),
        ("RightFoot", "l-foot"),
    ];
    assert_plays(&man, &robot, &Mapping::new(&man, &robot), &pairs);
    let back: Vec<(&str, &str)> = pairs.iter().map(|&(a, b)| (b, a)).collect();
    assert_plays(&robot, &man, &Mapping::new(&robot, &man), &back);
    // A nameless rig, its arms from the chest with no collarbones, finds the arms the same.
    let mut figure: Joints = vec![
        joint("n0", None, [0.0, 0.571, 0.0]),
        joint("n1", Some("n0"), [0.0, 0.692, 0.009]),
        joint("n2", Some("n1"), [0.0, 0.903, -0.004]),
        joint("n3", Some("n2"), [0.0, 0.956, 0.005]),
        joint("n4", Some("n3"), [0.0, 1.0, 0.007]),
    ];
    for (side, x) in [("a", 1.0f32), ("b", -1.0)] {
        let n = |k: u32| format!("n{side}{k}");
        figure.push(joint(n(1), Some("n2"), [0.081 * x, 0.903, -0.004]));
        figure.push(joint(n(2), Some(&n(1)), [0.262 * x, 0.811, -0.013]));
        figure.push(joint(n(3), Some(&n(2)), [0.382 * x, 0.735, 0.056]));
        figure.push(joint(n(4), Some("n0"), [0.061 * x, 0.516, 0.02]));
        figure.push(joint(n(5), Some(&n(4)), [0.069 * x, 0.296, 0.057]));
        figure.push(joint(n(6), Some(&n(5)), [0.07 * x, 0.072, -0.004]));
        figure.push(joint(n(7), Some(&n(6)), [0.071 * x, 0.018, 0.023]));
    }
    let figure = placed(&figure);
    let pairs = [
        ("na1", "r-arm"),
        ("na2", "r-forearm"),
        ("nb1", "l-arm"),
        ("nb2", "l-forearm"),
        ("na4", "r-thigh"),
        ("nb4", "l-thigh"),
    ];
    assert_plays(&figure, &robot, &Mapping::new(&figure, &robot), &pairs);
}

#[test]
fn a_head_with_nothing_beyond_it_plays_the_head_however_long_the_neck() {
    // Nameless rigs whose heads end at the head's root: one with a short neck under it, as a Rigify metarig's, the head
    // standing about where a man's neck does for the height the shoulders give; one with no neck, as BrainStem's.
    let limbs = |j: &mut Joints, chest: &str, collar: bool, shoulder: f32| {
        for (side, x) in [("a", 1.0f32), ("b", -1.0)] {
            let n = |k: u32| format!("n{side}{k}");
            let mut arm = chest.to_string();
            if collar {
                j.push(joint(n(0), Some(chest), [0.02 * x, shoulder + 0.02, 0.05]));
                arm = n(0);
            }
            j.push(joint(n(1), Some(&arm), [0.195 * x, shoulder, -0.03]));
            j.push(joint(n(2), Some(&n(1)), [0.3 * x, shoulder - 0.29, -0.03]));
            j.push(joint(n(3), Some(&n(2)), [0.38 * x, shoulder - 0.54, 0.0]));
            j.push(joint(n(4), Some(&n(3)), [0.4 * x, shoulder - 0.64, 0.01]));
            j.push(joint(n(5), Some("n0"), [0.098 * x, 0.98, 0.0]));
            j.push(joint(n(6), Some(&n(5)), [0.1 * x, 0.52, 0.0]));
            j.push(joint(n(7), Some(&n(6)), [0.1 * x, 0.08, -0.02]));
            j.push(joint(n(8), Some(&n(7)), [0.1 * x, 0.0, 0.12]));
        }
    };
    let mut necked: Joints = vec![
        joint("n0", None, [0.0, 0.92, 0.0]),
        joint("n1", Some("n0"), [0.0, 1.067, 0.0]),
        joint("n2", Some("n1"), [0.0, 1.293, 0.0]),
        joint("n3", Some("n2"), [0.0, 1.466, 0.0]),
        joint("n4", Some("n3"), [0.0, 1.648, -0.04]),
        joint("n5", Some("n4"), [0.0, 1.706, -0.04]),
    ];
    limbs(&mut necked, "n3", true, 1.585);
    let mut neckless: Joints = vec![
        joint("n0", None, [0.0, 0.915, 0.0]),
        joint("n1", Some("n0"), [0.0, 1.022, -0.02]),
        joint("n2", Some("n1"), [0.0, 1.546, -0.04]),
    ];
    limbs(&mut neckless, "n1", false, 1.36);
    // Its spine, the first bone up from the hips, plays the first of the other's, however many each has.
    for (other, spine) in [(mixamo_like(), "Spine"), (expressive_like(), "Abdomen")] {
        for (rig, pairs) in [
            (placed(&necked), &[(spine, "n1"), ("Neck", "n4"), ("Head", "n5")][..]),
            (placed(&neckless), &[("Head", "n2")][..]),
        ] {
            assert_plays(&other, &rig, &Mapping::new(&other, &rig), pairs);
            let back: Vec<(&str, &str)> = pairs.iter().map(|&(a, b)| (b, a)).collect();
            assert_plays(&rig, &other, &Mapping::new(&rig, &other), &back);
        }
    }
}

#[test]
fn the_spine_is_the_first_bone_up_from_the_hips_however_many_there_are() {
    // A robot with one bone between its hips and its chest, its pelvis hung below the bone its legs hang from, against
    // a man with two.
    let (robot, man) = (expressive_like(), mixamo_like());
    let pairs = [("Spine", "Abdomen"), ("Spine2", "Torso"), ("Head", "Head")];
    assert_plays(&man, &robot, &Mapping::new(&man, &robot), &pairs);
    let back: Vec<(&str, &str)> = pairs.iter().map(|&(a, b)| (b, a)).collect();
    assert_plays(&robot, &man, &Mapping::new(&robot, &man), &back);
}
