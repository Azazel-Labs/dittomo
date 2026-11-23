//! Legs and arms found without names, two-bone IK, planted feet, travel for clips walked in place, and push-out.

#[path = "../benches/common/mod.rs"]
#[allow(dead_code)]
mod common;

use dittomo::contact::{self, Limb};
use dittomo::{Clip, Fixes, Mapping, Skeleton, Transform, Vec3};

fn names(sk: &Skeleton, limbs: &[Limb]) -> Vec<[String; 3]> {
    let n = |b: usize| sk.bones()[b].name.clone();
    let mut v: Vec<[String; 3]> = limbs.iter().map(|l| [n(l.upper), n(l.lower), n(l.end)]).collect();
    v.sort();
    v
}

fn at(sk: &Skeleton, pose: &[Transform], bone: usize) -> Vec3 {
    sk.world(pose)[bone].position()
}

/// The same rig with shins and feet set `by` times as far down.
fn long_legs(sk: &Skeleton, by: f32) -> Skeleton {
    let mut bones = sk.bones().to_vec();
    for b in &mut bones {
        if (b.name.ends_with("Leg") && !b.name.ends_with("UpLeg")) || b.name.ends_with("Foot") {
            b.rest.translation *= by;
        }
    }
    Skeleton::new(bones).unwrap()
}

/// How far the target's feet slide, on average, between frames where the source's are planted, over its height.
fn skate(map: &Mapping, source: &Skeleton, target: &Skeleton, from: &Clip, played: &Clip) -> f32 {
    let world = target.world(&target.rest());
    let (lo, hi) = world.iter().fold((f32::MAX, f32::MIN), |(lo, hi), p| {
        (lo.min(p.position().y), hi.max(p.position().y))
    });
    let (mut slid, mut count) = (0.0, 0);
    for leg in contact::legs(target) {
        let a = map.source_of(leg.end).unwrap();
        let down = contact::planted(source, from, a);
        for f in 1..played.poses.len() {
            if down[f] && down[f - 1] {
                let d = at(target, &played.poses[f], leg.end) - at(target, &played.poses[f - 1], leg.end);
                slid += Vec3::new(d.x, 0.0, d.z).length();
                count += 1;
            }
        }
    }
    assert!(count > 10, "the clip has planted frames ({count})");
    slid / count as f32 / (hi - lo)
}

#[test]
fn finds_legs_and_arms_without_names() {
    let mixamo = common::mixamo();
    let m = |s: String| format!("mixamorig:{s}");
    let side = |s: &str, a: &str, b: &str, c: &str| [m(format!("{s}{a}")), m(format!("{s}{b}")), m(format!("{s}{c}"))];
    assert_eq!(
        names(&mixamo, &contact::legs(&mixamo)),
        vec![
            side("Left", "UpLeg", "Leg", "Foot"),
            side("Right", "UpLeg", "Leg", "Foot")
        ]
    );
    assert_eq!(
        names(&mixamo, &contact::arms(&mixamo)),
        vec![
            side("Left", "Arm", "ForeArm", "Hand"),
            side("Right", "Arm", "ForeArm", "Hand")
        ]
    );
    let mannequin = common::mannequin();
    let u = |s: &str, a: &str, b: &str, c: &str| [format!("{a}_{s}"), format!("{b}_{s}"), format!("{c}_{s}")];
    assert_eq!(
        names(&mannequin, &contact::legs(&mannequin)),
        vec![u("l", "thigh", "calf", "foot"), u("r", "thigh", "calf", "foot")]
    );
    assert_eq!(
        names(&mannequin, &contact::arms(&mannequin)),
        vec![
            u("l", "upperarm", "lowerarm", "hand"),
            u("r", "upperarm", "lowerarm", "hand")
        ]
    );
}

#[test]
fn reaches_the_goal_keeping_the_foot_turned() {
    for sk in [common::mixamo(), common::mannequin()] {
        for leg in contact::legs(&sk) {
            let rest = sk.rest();
            let world = sk.world(&rest);
            let foot = world[leg.end].position();
            let knee = world[leg.lower].position();
            for goal in [
                foot + Vec3::new(0.0, 10.0, 15.0),
                foot + Vec3::new(3.0, 25.0, -10.0),
                foot + Vec3::new(-2.0, 4.0, 12.0),
            ] {
                let mut pose = rest.clone();
                contact::reach(&sk, &mut pose, &leg, goal);
                let now = sk.world(&pose);
                let name = &sk.bones()[leg.end].name;
                assert!(now[leg.end].position().distance(goal) < 1e-2, "{name} reaches {goal}");
                assert!(
                    now[leg.end].rotation.angle_between(world[leg.end].rotation) < 1e-3,
                    "{name} keeps its turn"
                );
                assert!(
                    (now[leg.lower].position() - knee).z > -5.0,
                    "{name}'s knee bends forward"
                );
            }
            // Out of reach: the leg points straight at it.
            let mut pose = rest.clone();
            let far = foot + Vec3::new(0.0, -200.0, 50.0);
            contact::reach(&sk, &mut pose, &leg, far);
            let now = sk.world(&pose);
            let (a, c) = (now[leg.upper].position(), now[leg.end].position());
            assert!((c - a).normalize().dot((far - a).normalize()) > 0.999);
        }
    }
}

fn feet_only() -> Fixes {
    Fixes {
        feet: true,
        ..Fixes::NONE
    }
}

#[test]
fn planted_feet_stay_still_on_longer_legs() {
    let source = common::mixamo();
    let target = long_legs(&source, 1.3);
    let walk = common::gait(&source, 90, false);
    let map = Mapping::new(&source, &target);
    let plain = map.apply(&source, &target, &walk).unwrap();
    let fixed = map.apply_with(&source, &target, &walk, feet_only()).unwrap();
    let before = skate(&map, &source, &target, &walk, &plain);
    let after = skate(&map, &source, &target, &walk, &fixed);
    eprintln!("long legs, skate per planted frame over height: {before:.5} -> {after:.5}");
    assert!(after < 0.2 * before, "{before} -> {after}");
    assert!(after < 1e-3, "{after}");
}

#[test]
fn planted_feet_stay_still_on_the_mannequin() {
    let source = common::mixamo();
    let target = common::mannequin();
    let walk = common::gait(&source, 90, false);
    let map = Mapping::new(&source, &target);
    let plain = map.apply(&source, &target, &walk).unwrap();
    let fixed = map.apply_with(&source, &target, &walk, feet_only()).unwrap();
    let before = skate(&map, &source, &target, &walk, &plain);
    let after = skate(&map, &source, &target, &walk, &fixed);
    eprintln!("mannequin, skate per planted frame over height: {before:.5} -> {after:.5}");
    assert!(after < 1e-3, "{after}");
}

#[test]
fn a_clip_walked_in_place_gets_its_travel() {
    let sk = common::mixamo();
    let frames = 90;
    let in_place = common::gait(&sk, frames, true);
    let walked = contact::travel(&sk, &in_place);
    let z = |c: &Clip| c.poses[frames - 1][0].translation.z - in_place.poses[frames - 1][0].translation.z;
    let (went, should) = (z(&walked), z(&common::gait(&sk, frames, false)));
    assert!(
        (went - should).abs() < 0.1 * should,
        "travelled {went}, should {should}"
    );
    // And its planted feet now stay put.
    let map = Mapping::new(&sk, &sk);
    let slid = skate(&map, &sk, &sk, &walked, &walked);
    assert!(slid < 2e-3, "{slid}");
}

#[test]
fn a_travelling_clip_is_left_alone() {
    let sk = common::mixamo();
    let walk = common::gait(&sk, 60, false);
    assert_eq!(contact::travel(&sk, &walk), walk);
    let still = Clip {
        times: vec![0.0, 1.0],
        poses: vec![sk.rest(), sk.rest()],
    };
    assert_eq!(contact::travel(&sk, &still), still);
}

#[test]
fn pushes_a_hand_out_of_the_chest() {
    let sk = common::mixamo();
    let map = Mapping::new(&sk, &sk);
    let push = Fixes {
        push_out: true,
        ..Fixes::NONE
    };
    // Nothing to push at rest.
    let rest = Clip {
        times: vec![0.0],
        poses: vec![sk.rest()],
    };
    assert_eq!(
        map.apply_with(&sk, &sk, &rest, push).unwrap(),
        map.apply(&sk, &sk, &rest).unwrap()
    );

    let arm = contact::arms(&sk)[0];
    let chest = at(&sk, &sk.rest(), sk.index_of("mixamorig:Spine2").unwrap());
    let mut pose = sk.rest();
    contact::reach(&sk, &mut pose, &arm, chest + Vec3::new(0.0, -5.0, 2.0));
    let inside = Clip {
        times: vec![0.0],
        poses: vec![pose.clone()],
    };
    let out = map.apply_with(&sk, &sk, &inside, push).unwrap();
    let spine: Vec<usize> = ["Hips", "Spine", "Spine1", "Spine2"]
        .iter()
        .map(|n| sk.index_of(&format!("mixamorig:{n}")).unwrap())
        .collect();
    let clearance = |p: &[Transform]| {
        let w = sk.world(p);
        spine
            .iter()
            .map(|&s| w[s].position().distance(w[arm.end].position()))
            .fold(f32::MAX, f32::min)
    };
    let (was, now) = (clearance(&pose), clearance(&out.poses[0]));
    assert!(now > was + 5.0, "{was} -> {now}");
    // The spine itself didn't move.
    let w = sk.world(&out.poses[0]);
    assert!(w[spine[3]].position().distance(chest) < 1e-3);
}

/// A skeleton from joints named with their parents and world positions, every bone unturned at rest.
fn placed(joints: &[(&str, Option<&str>, [f32; 3])]) -> Skeleton {
    let index = |n: &str| joints.iter().position(|j| j.0 == n).expect("parent listed");
    let bones = joints
        .iter()
        .map(|&(name, parent, at)| {
            let at = Vec3::from(at) * 170.0;
            let parent = parent.map(index);
            let from = parent.map_or(Vec3::ZERO, |p| Vec3::from(joints[p].2) * 170.0);
            dittomo::Bone {
                name: name.to_owned(),
                parent,
                rest: Transform::new(at - from, dittomo::Quat::IDENTITY),
            }
        })
        .collect();
    Skeleton::new(bones).unwrap()
}

/// Both sides of a body from its left side, mirrored across x.
fn mirrored(
    middle: &[(&'static str, Option<&'static str>, [f32; 3])],
    left: &[(String, Option<String>, [f32; 3])],
) -> Skeleton {
    let mut names: Vec<(String, Option<String>, [f32; 3])> = middle
        .iter()
        .map(|&(n, p, at)| (n.to_owned(), p.map(str::to_owned), at))
        .collect();
    for (n, p, at) in left {
        names.push((
            format!("{n}.L"),
            p.clone().map(|p| {
                if middle.iter().any(|m| m.0 == p) {
                    p
                } else {
                    format!("{p}.L")
                }
            }),
            *at,
        ));
        let [x, y, z] = *at;
        names.push((
            format!("{n}.R"),
            p.clone().map(|p| {
                if middle.iter().any(|m| m.0 == p) {
                    p
                } else {
                    format!("{p}.R")
                }
            }),
            [-x, y, z],
        ));
    }
    let joints: Vec<(&str, Option<&str>, [f32; 3])> =
        names.iter().map(|(n, p, at)| (n.as_str(), p.as_deref(), *at)).collect();
    placed(&joints)
}

fn side(joints: &[(&str, Option<&str>, [f32; 3])]) -> Vec<(String, Option<String>, [f32; 3])> {
    joints
        .iter()
        .map(|&(n, p, at)| (n.to_owned(), p.map(str::to_owned), at))
        .collect()
}

fn hand(left: &mut Vec<(String, Option<String>, [f32; 3])>, at: [f32; 3], down: bool) {
    left.push(("hand".into(), Some("forearm".into()), at));
    for f in 0..4 {
        let mut parent = "hand".to_owned();
        for k in 1..=3 {
            let name = format!("finger{f}_{k}");
            let step = 0.02 * k as f32;
            let p = if down {
                [at[0], at[1] - step, at[2] + 0.01 * f as f32]
            } else {
                [at[0] + step, at[1], at[2] + 0.01 * f as f32]
            };
            left.push((name.clone(), Some(parent), p));
            parent = name;
        }
    }
}

/// An avatar's limbs, laid out as VRM exports write them: a twist bone in the chain between forearm and hand, helper
/// bones with children of their own off the collarbone, a ponytail off the head and the chest bone well below the
/// shoulders. The elbow is the elbow, the shoulder the shoulder, and the ponytail no arm.
#[test]
fn finds_arms_past_twist_bones_helpers_and_hair() {
    let middle = [
        ("hips", None, [0.0, 0.53, 0.0]),
        ("spine", Some("hips"), [0.0, 0.63, 0.0]),
        ("chest", Some("spine"), [0.0, 0.71, 0.0]),
        ("neck", Some("chest"), [0.0, 0.87, 0.0]),
        ("head", Some("neck"), [0.0, 0.90, 0.0]),
        ("crown", Some("head"), [0.0, 1.0, 0.0]),
        ("tail1", Some("head"), [0.0, 0.90, -0.05]),
        ("tail2", Some("tail1"), [0.0, 0.86, -0.06]),
        ("tail3", Some("tail2"), [-0.02, 0.82, -0.08]),
        ("tail4", Some("tail3"), [-0.06, 0.79, -0.10]),
        ("tail5", Some("tail4"), [-0.09, 0.76, -0.12]),
        ("tail6", Some("tail5"), [-0.12, 0.73, -0.15]),
    ];
    let mut left = side(&[
        ("shoulder", Some("chest"), [0.03, 0.855, 0.0]),
        ("aim", Some("shoulder"), [0.07, 0.83, 0.0]),
        ("roll", Some("aim"), [0.15, 0.83, 0.0]),
        ("sleeve1", Some("shoulder"), [0.07, 0.83, 0.0]),
        ("sleeve2", Some("sleeve1"), [0.10, 0.80, 0.0]),
        ("sleeve3", Some("sleeve2"), [0.12, 0.75, 0.0]),
        ("upper_arm", Some("shoulder"), [0.10, 0.83, 0.0]),
        ("forearm", Some("upper_arm"), [0.27, 0.83, 0.0]),
        ("twist", Some("forearm"), [0.31, 0.83, 0.0]),
        ("thigh", Some("hips"), [0.06, 0.54, 0.0]),
        ("shin", Some("thigh"), [0.06, 0.31, 0.0]),
        ("foot", Some("shin"), [0.06, 0.04, 0.0]),
        ("toe", Some("foot"), [0.06, 0.0, 0.1]),
    ]);
    hand(&mut left, [0.42, 0.83, 0.0], false);
    // The hand hangs from the twist bone.
    let at = left.iter().position(|j| j.0 == "hand").unwrap();
    left[at].1 = Some("twist".into());
    let sk = mirrored(&middle, &left);
    let lr = |a: &str, b: &str, c: &str| {
        ["L", "R"]
            .map(|s| [format!("{a}.{s}"), format!("{b}.{s}"), format!("{c}.{s}")])
            .to_vec()
    };
    assert_eq!(names(&sk, &contact::arms(&sk)), lr("upper_arm", "forearm", "hand"));
    assert_eq!(names(&sk, &contact::legs(&sk)), lr("thigh", "shin", "foot"));
}

/// Arms hanging to the knees, fingers reaching below the lowest seventh of the body as a cartoon robot's do: the
/// fingers are no legs.
#[test]
fn fingers_hanging_low_are_no_legs() {
    let middle = [
        ("hips", None, [0.0, 0.45, 0.0]),
        ("chest", Some("hips"), [0.0, 0.70, 0.0]),
        ("head", Some("chest"), [0.0, 0.90, 0.0]),
        ("crown", Some("head"), [0.0, 1.0, 0.0]),
    ];
    let mut left = side(&[
        ("upper_arm", Some("chest"), [0.20, 0.77, 0.0]),
        ("forearm", Some("upper_arm"), [0.25, 0.55, 0.0]),
        ("thigh", Some("hips"), [0.10, 0.45, 0.0]),
        ("shin", Some("thigh"), [0.10, 0.24, 0.0]),
        ("foot", Some("shin"), [0.10, 0.03, 0.0]),
        ("toe", Some("foot"), [0.10, 0.0, 0.1]),
    ]);
    hand(&mut left, [0.28, 0.25, 0.0], true);
    let sk = mirrored(&middle, &left);
    let lr = |a: &str, b: &str, c: &str| {
        ["L", "R"]
            .map(|s| [format!("{a}.{s}"), format!("{b}.{s}"), format!("{c}.{s}")])
            .to_vec()
    };
    assert_eq!(names(&sk, &contact::legs(&sk)), lr("thigh", "shin", "foot"));
    assert_eq!(names(&sk, &contact::arms(&sk)), lr("upper_arm", "forearm", "hand"));
}

/// Skin on the mixamo rig: a trunk `r` wide around the spine from the hips to the chest, and a ball round each hand
/// and its fingers.
fn skin(sk: &Skeleton, r: f32) -> dittomo::Surface {
    let rest = sk.world(&sk.rest());
    let bone = |n: &str| sk.index_of(&format!("mixamorig:{n}")).unwrap();
    let mut points = Vec::new();
    for (lo, hi, b) in [(95.0, 110.0, "Hips"), (110.0, 122.0, "Spine"), (122.0, 140.0, "Spine1")] {
        let mut y = lo;
        while y < hi {
            for k in 0..48 {
                let a = k as f32 / 48.0 * std::f32::consts::TAU;
                let n = Vec3::new(a.cos(), 0.0, a.sin());
                points.push((Vec3::new(0.0, y, 0.0) + n * r, n, bone(b)));
            }
            y += 1.0;
        }
    }
    for side in ["Left", "Right"] {
        let hand = bone(&format!("{side}Hand"));
        // The palm and fingers, along the hand from the wrist: points on a rod 3 thick.
        let (wrist, tip) = (
            rest[hand].position(),
            rest[bone(&format!("{side}HandMiddle4"))].position(),
        );
        for i in 0..=20 {
            let at = wrist.lerp(tip, i as f32 / 20.0);
            for k in 0..12 {
                let a = k as f32 / 12.0 * std::f32::consts::TAU;
                let n = Vec3::new(0.0, a.cos(), a.sin());
                points.push((at + n * 1.5, n, hand));
            }
        }
    }
    dittomo::Surface::new(sk, points, 1.0)
}

#[test]
fn a_surface_keeps_its_outside() {
    let sk = common::mixamo();
    let s = skin(&sk, 10.0);
    assert!(!s.is_empty());
    // The same points facing in, as a lining would, are all left out.
    let hips = sk.index_of("mixamorig:Hips").unwrap();
    let lining = dittomo::Surface::new(
        &sk,
        (0..48).map(|k| {
            let a = k as f32 / 48.0 * std::f32::consts::TAU;
            let n = Vec3::new(a.cos(), 0.0, a.sin());
            (Vec3::new(0.0, 100.0, 0.0) + n * 10.0, -n, hips)
        }),
        1.0,
    );
    assert!(lining.is_empty(), "{} points of lining kept", lining.len());
}

#[test]
fn a_hand_resting_on_a_thin_body_rests_on_a_thick_one() {
    let sk = common::mixamo();
    let map = Mapping::new(&sk, &sk);
    let (thin, thick) = (skin(&sk, 10.0), skin(&sk, 16.0));
    let arm = contact::arms(&sk)[0];
    let x = at(&sk, &sk.rest(), arm.end).x.signum();
    let uppers: Vec<usize> = contact::arms(&sk).iter().map(|a| a.upper).collect();
    let gap = |s: &dittomo::Surface, pose: &[Transform]| s.gap(&sk, &sk.world(pose), arm.end, &uppers).unwrap();
    let touch = Fixes {
        touch: true,
        ..Fixes::NONE
    };
    let play = |pose: &[Transform], skins: [&dittomo::Surface; 2], fixes| {
        let clip = Clip {
            times: vec![0.0],
            poses: vec![pose.to_vec()],
        };
        map.apply_touching(&sk, &sk, skins, &clip, fixes)
            .unwrap()
            .poses
            .remove(0)
    };

    // The hand on the thin body's hip, about a centimetre off it.
    let mut rest_on_hip = sk.rest();
    contact::reach(&sk, &mut rest_on_hip, &arm, Vec3::new(x * 11.5, 105.0, 0.0));
    let source = gap(&thin, &rest_on_hip);
    assert!((0.0..2.5).contains(&source), "source hand {source} off its hip");
    let plain = play(&rest_on_hip, [&thin, &thick], Fixes::NONE);
    assert!(
        gap(&thick, &plain) < -4.0,
        "played as is, the hand is in the thick body: {}",
        gap(&thick, &plain)
    );
    let fixed = play(&rest_on_hip, [&thin, &thick], touch);
    let target = gap(&thick, &fixed);
    assert!(
        (target - source).abs() < 0.5,
        "source {source} off its hip, target {target}"
    );
    // The hand keeps its turn.
    let (a, b) = (sk.world(&plain)[arm.end].rotation, sk.world(&fixed)[arm.end].rotation);
    assert!(a.angle_between(b) < 1e-3);

    // A hand held out is left alone.
    let out = play(&sk.rest(), [&thin, &thick], touch);
    assert_eq!(out, play(&sk.rest(), [&thin, &thick], Fixes::NONE));
    // And without skins, touch does nothing.
    let empty = dittomo::Surface::default();
    assert_eq!(play(&rest_on_hip, [&empty, &empty], touch), plain);
}

/// The same rig with its upper arms, forearms and hands `by` times as long.
fn long_arms(sk: &Skeleton, by: f32) -> Skeleton {
    let mut bones = sk.bones().to_vec();
    for b in &mut bones {
        if b.name.contains("ForeArm") || b.name.contains("Hand") {
            b.rest.translation *= by;
        }
    }
    Skeleton::new(bones).unwrap()
}

/// Jumping jacks, a second each: the arms swing up out to the sides and clap overhead, then swing forward and clap in
/// front of the chest. The wrists meet 3 cm apart.
fn claps(sk: &Skeleton) -> Clip {
    use std::f32::consts::{FRAC_PI_2, PI};
    let frames = 120;
    let rest = sk.rest();
    let world = sk.world(&rest);
    let smooth = |x: f32, a: f32, b: f32| {
        let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    };
    let poses = (0..frames)
        .map(|f| {
            let t = f as f32 / frames as f32 * 2.0;
            let u = (PI * t.fract()).sin().powi(2);
            let mut pose = rest.clone();
            for arm in contact::arms(sk) {
                let (a, b, c) = (
                    world[arm.upper].position(),
                    world[arm.lower].position(),
                    world[arm.end].position(),
                );
                let len = 0.95 * (a.distance(b) + b.distance(c));
                let x = a.x.signum();
                let (swing, clap) = if t < 1.0 {
                    let th = 0.95 * PI * u;
                    (
                        a + len * Vec3::new(x * th.sin(), -th.cos(), 0.05),
                        Vec3::new(x * 1.5, 190.0, 4.0),
                    )
                } else {
                    let th = FRAC_PI_2 * u;
                    (
                        a + len * Vec3::new(x * 0.1, -th.cos(), th.sin()),
                        Vec3::new(x * 1.5, 135.0, 30.0),
                    )
                };
                contact::reach(sk, &mut pose, &arm, swing.lerp(clap, smooth(u, 0.7, 1.0)));
            }
            pose
        })
        .collect();
    Clip {
        times: (0..frames).map(|f| f as f32 / 60.0).collect(),
        poses,
    }
}

#[test]
fn hands_clap_together_on_arms_short_and_long_and_stay_out_of_the_head() {
    let source = common::mixamo();
    let clip = claps(&source);
    let arms = contact::arms(&source);
    let wrists = |sk: &Skeleton, pose: &[Transform]| {
        let w = sk.world(pose);
        [w[arms[0].end].position(), w[arms[1].end].position()]
    };
    let gap = |sk: &Skeleton, pose: &[Transform]| {
        let [a, b] = wrists(sk, pose);
        a.distance(b)
    };
    // The head: from the base of the skull at 160 cm to its top at 182.
    let head = Vec3::new(0.0, 171.0, 2.0);
    let push = Fixes {
        push_out: true,
        ..Fixes::NONE
    };
    let both = Fixes { hands: true, ..push };
    let clapping: Vec<usize> = (0..clip.poses.len())
        .filter(|&f| gap(&source, &clip.poses[f]) < 5.0)
        .collect();
    assert!(clapping.len() > 6, "{} frames clapping", clapping.len());
    for (name, by) in [("short", 0.6), ("long", 1.5), ("same", 1.0)] {
        let target = long_arms(&source, by);
        let map = Mapping::new(&source, &target);
        let plain = map.apply_with(&source, &target, &clip, push).unwrap();
        let fixed = map.apply_with(&source, &target, &clip, both).unwrap();
        let (mut worst, mut was, mut clear) = (0.0f32, 0.0f32, f32::MAX);
        for &f in &clapping {
            worst = worst.max(gap(&target, &fixed.poses[f]));
            was = was.max(gap(&target, &plain.poses[f]));
            for p in wrists(&target, &fixed.poses[f]) {
                clear = clear.min(p.distance(head));
            }
        }
        eprintln!(
            "{name} arms: wrists apart while clapping, at most {was:.1} cm -> {worst:.1} cm; {clear:.1} cm from the head's middle"
        );
        assert!(worst < 6.0, "{name} arms: wrists {worst} cm apart while clapping");
        assert!(
            clear > 9.0,
            "{name} arms: a wrist {clear} cm from the middle of the head"
        );
        // Hands well apart are left as they were, and every hand keeps its turn.
        for f in 0..clip.poses.len() {
            let (p, q) = (target.world(&plain.poses[f]), target.world(&fixed.poses[f]));
            for arm in &arms {
                assert!(p[arm.end].rotation.angle_between(q[arm.end].rotation) < 1e-3);
                if gap(&source, &clip.poses[f]) > 40.0 {
                    assert!(p[arm.end].position().distance(q[arm.end].position()) < 1e-3);
                }
            }
        }
    }
}

/// The same rig with its head `by` times as tall, as a cartoon's, and its arms `arms` times as long.
fn big_head(sk: &Skeleton, by: f32, arms: f32) -> Skeleton {
    let mut bones = long_arms(sk, arms).bones().to_vec();
    for b in &mut bones {
        if b.name.ends_with("HeadTop_End") {
            b.rest.translation *= by;
        }
    }
    Skeleton::new(bones).unwrap()
}

/// The middle of the head and its radius at rest, as dittomo sees it on a rig with a bone at the top of the head: the
/// head bone up to there, a little under half of that across.
fn head_ball(sk: &Skeleton) -> (usize, Vec3, f32) {
    let find = |n: &str| sk.bones().iter().position(|b| b.name.ends_with(n)).unwrap();
    let (head, top) = (find(":Head"), find("HeadTop_End"));
    let world = sk.world(&sk.rest());
    let (a, b) = (world[head].position(), world[top].position());
    let r = 0.45 * a.distance(b);
    let centre = b - (b - world[find("Neck")].position()).normalize() * r;
    (head, world[head].matrix.inverse().transform_point3(centre), r)
}

/// Hands to the ears, held there while the head turns from side to side; then up onto the top of the head, held there
/// while it nods. A second each.
fn covers_ears(sk: &Skeleton) -> Clip {
    let frames = 120;
    let rest = sk.rest();
    let world = sk.world(&rest);
    let (head, centre, r) = head_ball(sk);
    let smooth = |x: f32, a: f32, b: f32| {
        let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
        t * t * (3.0 - 2.0 * t)
    };
    let poses = (0..frames)
        .map(|f| {
            let t = f as f32 / 60.0;
            let mut pose = rest.clone();
            let turn = if t < 1.0 {
                dittomo::Quat::from_rotation_y(0.5 * (std::f32::consts::TAU * t).sin() * smooth(t, 0.3, 0.4))
            } else {
                dittomo::Quat::from_rotation_x(0.35 * (std::f32::consts::TAU * t).sin() * smooth(t, 1.3, 1.4))
            };
            pose[head].rotation = turn * pose[head].rotation;
            let placed = sk.world(&pose)[head].matrix;
            for arm in contact::arms(sk) {
                let x = world[arm.end].position().x.signum();
                let ear = Vec3::new(x * 1.15, -0.3, -0.1) * r;
                let top = Vec3::new(x * 0.6, 1.15, 0.0) * r;
                let on = ear.lerp(top, smooth(t, 1.0, 1.25));
                let goal = placed.transform_point3(centre + on);
                let from = world[arm.end].position();
                contact::reach(sk, &mut pose, &arm, from.lerp(goal, smooth(t, 0.0, 0.25)));
            }
            pose
        })
        .collect();
    Clip {
        times: (0..frames).map(|f| f as f32 / 60.0).collect(),
        poses,
    }
}

#[test]
fn hands_stay_on_heads_big_and_small_covering_the_ears_and_on_top() {
    let source = common::mixamo();
    let clip = covers_ears(&source);
    // Where a wrist is on its head: in the head's own frame, over the head's radius.
    let on_head = |sk: &Skeleton, pose: &[Transform], end: usize| {
        let (head, centre, r) = head_ball(sk);
        let w = sk.world(pose);
        let rest = sk.world(&sk.rest())[head].rotation;
        let turn = w[head].rotation * rest.inverse();
        turn.inverse() * (w[end].position() - w[head].matrix.transform_point3(centre)) / r
    };
    let arms = contact::arms(&source);
    let held: Vec<usize> = (0..clip.poses.len())
        .filter(|&f| {
            arms.iter()
                .all(|a| on_head(&source, &clip.poses[f], a.end).length() < 1.4)
        })
        .collect();
    assert!(held.len() > 60, "{} frames on the head", held.len());
    let push = Fixes {
        push_out: true,
        hands: true,
        ..Fixes::NONE
    };
    let fixed = Fixes { head: true, ..push };
    for (name, head, by) in [
        ("big head", 1.6, 1.0),
        ("big head, short arms", 1.6, 0.75),
        ("small head", 0.7, 1.0),
        ("long arms", 1.0, 1.5),
        ("same", 1.0, 1.0),
    ] {
        let target = big_head(&source, head, by);
        let map = Mapping::new(&source, &target);
        let t_arms = contact::arms(&target);
        // How far off its place on the head a wrist gets, and how far off the head's surface (in or out), at most.
        let worst = |played: &Clip| {
            let (mut off, mut lift) = (0.0f32, 0.0f32);
            for &f in &held {
                for (a, ta) in arms.iter().zip(&t_arms) {
                    let want = on_head(&source, &clip.poses[f], a.end);
                    let got = on_head(&target, &played.poses[f], ta.end);
                    off = off.max(got.distance(want));
                    lift = lift.max((got.length() - want.length()).abs());
                }
            }
            (off, lift)
        };
        let plain = map.apply_with(&source, &target, &clip, push).unwrap();
        let played = map.apply_with(&source, &target, &clip, fixed).unwrap();
        let ((was, was_lift), (now, lift)) = (worst(&plain), worst(&played));
        eprintln!(
            "{name}: wrist off its place on the head by at most {was:.2} -> {now:.2} head radii, off its height over the head {was_lift:.2} -> {lift:.2}"
        );
        // Where the arms reach, the wrists are where the source's are; where they don't, as near as they get, on the
        // head, neither in it nor off it.
        assert!(
            lift < 0.05,
            "{name}: a wrist {lift} head radii off its height over the head"
        );
        if by >= 1.0 {
            assert!(now < 0.05, "{name}: a wrist {now} head radii off its place on the head");
        } else {
            assert!(
                now < 0.6 * was,
                "{name}: a wrist {now} head radii off its place on the head, {was} unfixed"
            );
        }
        // Hands keep their turn, and forearms stay out of the head.
        let (head, centre, r) = head_ball(&target);
        let mut clear = f32::MAX;
        for f in 0..clip.poses.len() {
            let (p, q) = (target.world(&plain.poses[f]), target.world(&played.poses[f]));
            let c = q[head].matrix.transform_point3(centre);
            for arm in &t_arms {
                assert!(p[arm.end].rotation.angle_between(q[arm.end].rotation) < 1e-3);
                let (b, e) = (q[arm.lower].position(), q[arm.end].position());
                for k in 0..=8 {
                    clear = clear.min(b.lerp(e, k as f32 / 8.0).distance(c) / r);
                }
            }
        }
        let (sh, sc, sr) = head_ball(&source);
        let mut s_clear = f32::MAX;
        for pose in &clip.poses {
            let q = source.world(pose);
            let c = q[sh].matrix.transform_point3(sc);
            for arm in &arms {
                let (b, e) = (q[arm.lower].position(), q[arm.end].position());
                for k in 0..=8 {
                    s_clear = s_clear.min(b.lerp(e, k as f32 / 8.0).distance(c) / sr);
                }
            }
        }
        eprintln!("{name}: forearms at least {clear:.2} head radii from its middle (source {s_clear:.2})");
        assert!(
            clear > s_clear - 0.05,
            "{name}: a forearm {clear} head radii from the middle of the head, the source's {s_clear}"
        );
    }
}
