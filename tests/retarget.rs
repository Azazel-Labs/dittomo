//! Retargeting between made-up humanoids that differ the ways real rigs do: names from different tools, extra bones,
//! longer spines, A-pose against T-pose, bone axes pointing different ways, facing, units and proportions.

use std::f32::consts::TAU;

use mummer::{Bone, Clip, Mapping, Quat, Skeleton, Transform, Vec3};

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Side {
    L,
    R,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Part {
    Root,
    Hips,
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
        list.push((Part::Thigh(s), Some(Part::Hips), thigh));
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
            .normalize();
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
        };
        let (from, to) = (humanoid(from), humanoid(to));
        let map = Mapping::new(&from.skeleton, &to.skeleton);
        let mut ignore: Vec<Part> = [Side::L, Side::R]
            .into_iter()
            .flat_map(|s| [Part::ArmTwist(s), Part::LegTwist(s)])
            .collect();
        ignore.push(Part::Root);
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
