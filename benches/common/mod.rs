//! Rigs laid out the way real tools write them, for benchmarks: Mixamo's 65-bone humanoid and a
//! game-engine mannequin with a root, a five-bone spine, twist bones and metacarpals.

use std::f32::consts::TAU;

use mummer::{Bone, Clip, Quat, Skeleton, Transform, Vec3};

/// How bones' axes sit at rest.
#[derive(Clone, Copy)]
pub enum Axes {
    /// y along the bone (Blender, Mixamo).
    YAlong,
    /// x along the bone, rolled (Maya, Unreal).
    XAlong,
}

/// A skeleton from joints named with their parents and world positions, in centimetres.
fn build(joints: &[(String, Option<String>, Vec3)], axes: Axes) -> Skeleton {
    let index = |n: &str| joints.iter().position(|j| j.0 == n).expect("parent listed");
    let parents: Vec<Option<usize>> = joints.iter().map(|j| j.1.as_deref().map(index)).collect();
    let pos: Vec<Vec3> = joints.iter().map(|j| j.2).collect();
    let rot: Vec<Quat> = (0..joints.len())
        .map(|i| {
            let child = parents.iter().position(|&p| p == Some(i));
            let along = match (child, parents[i]) {
                (Some(c), _) => pos[c] - pos[i],
                (None, Some(p)) => pos[i] - pos[p],
                (None, None) => Vec3::Y,
            };
            let along = if along.length() > 1e-6 {
                along.normalize()
            } else {
                Vec3::Y
            };
            match axes {
                Axes::YAlong => Quat::from_rotation_arc(Vec3::Y, along),
                Axes::XAlong => Quat::from_rotation_arc(Vec3::X, along) * Quat::from_rotation_x(0.3 * i as f32),
            }
        })
        .collect();
    let bones = (0..joints.len())
        .map(|i| {
            let rest = match parents[i] {
                Some(p) => {
                    let back = rot[p].inverse();
                    Transform::new(back * (pos[i] - pos[p]), back * rot[i])
                }
                None => Transform::new(pos[i], rot[i]),
            };
            Bone {
                name: joints[i].0.clone(),
                parent: parents[i],
                rest,
            }
        })
        .collect();
    Skeleton::new(bones).expect("parents first")
}

fn j(name: impl Into<String>, parent: Option<&str>, at: Vec3) -> (String, Option<String>, Vec3) {
    (name.into(), parent.map(str::to_owned), at)
}

/// Mixamo's humanoid: 65 bones, T-pose, y along each bone, centimetres.
#[must_use]
pub fn mixamo() -> Skeleton {
    let m = |s: &str| format!("mixamorig:{s}");
    let mut v = vec![
        j(m("Hips"), None, Vec3::new(0.0, 100.0, 0.0)),
        j(m("Spine"), Some(&m("Hips")), Vec3::new(0.0, 110.0, 0.0)),
        j(m("Spine1"), Some(&m("Spine")), Vec3::new(0.0, 122.0, 0.0)),
        j(m("Spine2"), Some(&m("Spine1")), Vec3::new(0.0, 135.0, 0.0)),
        j(m("Neck"), Some(&m("Spine2")), Vec3::new(0.0, 150.0, 0.0)),
        j(m("Head"), Some(&m("Neck")), Vec3::new(0.0, 160.0, 2.0)),
        j(m("HeadTop_End"), Some(&m("Head")), Vec3::new(0.0, 182.0, 2.0)),
    ];
    for (side, x) in [("Left", 1.0f32), ("Right", -1.0)] {
        let n = |s: &str| m(&format!("{side}{s}"));
        v.push(j(n("Shoulder"), Some(&m("Spine2")), Vec3::new(6.0 * x, 145.0, 0.0)));
        v.push(j(n("Arm"), Some(&n("Shoulder")), Vec3::new(18.0 * x, 145.0, 0.0)));
        v.push(j(n("ForeArm"), Some(&n("Arm")), Vec3::new(46.0 * x, 145.0, 0.0)));
        v.push(j(n("Hand"), Some(&n("ForeArm")), Vec3::new(72.0 * x, 145.0, 0.0)));
        for (f, (finger, z)) in [
            ("Thumb", 4.0f32),
            ("Index", 2.5),
            ("Middle", 0.5),
            ("Ring", -1.5),
            ("Pinky", -3.5),
        ]
        .into_iter()
        .enumerate()
        {
            let start = if f == 0 { 74.0 } else { 80.0 };
            let mut parent = n("Hand");
            for k in 1..=4 {
                let name = if k == 4 {
                    n(&format!("Hand{finger}4"))
                } else {
                    n(&format!("Hand{finger}{k}"))
                };
                let at = Vec3::new(
                    (start + 3.0 * (k - 1) as f32) * x,
                    145.0 - if f == 0 { 2.0 } else { 0.0 },
                    z,
                );
                v.push(j(name.clone(), Some(&parent), at));
                parent = name;
            }
        }
        v.push(j(n("UpLeg"), Some(&m("Hips")), Vec3::new(9.0 * x, 95.0, 0.0)));
        v.push(j(n("Leg"), Some(&n("UpLeg")), Vec3::new(9.0 * x, 52.0, 1.0)));
        v.push(j(n("Foot"), Some(&n("Leg")), Vec3::new(9.0 * x, 8.0, 0.0)));
        v.push(j(n("ToeBase"), Some(&n("Foot")), Vec3::new(9.0 * x, 2.0, 12.0)));
        v.push(j(n("Toe_End"), Some(&n("ToeBase")), Vec3::new(9.0 * x, 2.0, 19.0)));
    }
    assert_eq!(v.len(), 65);
    build(&v, Axes::YAlong)
}

/// A game-engine mannequin: a root on the floor, a five-bone spine, two neck bones, twist bones
/// beside each limb, metacarpals; A-pose, x along each bone and rolled, centimetres, a little
/// shorter than [`mixamo`]. 76 bones.
#[must_use]
pub fn mannequin() -> Skeleton {
    let mut v = vec![
        j("root", None, Vec3::ZERO),
        j("pelvis", Some("root"), Vec3::new(0.0, 95.0, 0.0)),
    ];
    let mut up = "pelvis".to_owned();
    for k in 1..=5 {
        let name = format!("spine_{k:02}");
        v.push(j(name.clone(), Some(&up), Vec3::new(0.0, 95.0 + 9.0 * k as f32, 0.0)));
        up = name;
    }
    v.push(j("neck_01", Some("spine_05"), Vec3::new(0.0, 145.0, 1.0)));
    v.push(j("neck_02", Some("neck_01"), Vec3::new(0.0, 150.0, 1.5)));
    v.push(j("head", Some("neck_02"), Vec3::new(0.0, 155.0, 2.0)));
    let down = 40f32.to_radians();
    for (s, x) in [("l", 1.0f32), ("r", -1.0)] {
        let n = |b: &str| format!("{b}_{s}");
        let dir = Vec3::new(down.cos() * x, -down.sin(), 0.0);
        let shoulder = Vec3::new(16.0 * x, 140.0, 0.0);
        v.push(j(n("clavicle"), Some("spine_05"), Vec3::new(4.0 * x, 140.0, 0.0)));
        v.push(j(n("upperarm"), Some(&n("clavicle")), shoulder));
        v.push(j(n("upperarm_twist_01"), Some(&n("upperarm")), shoulder + 9.0 * dir));
        v.push(j(n("upperarm_twist_02"), Some(&n("upperarm")), shoulder + 18.0 * dir));
        let elbow = shoulder + 27.0 * dir;
        v.push(j(n("lowerarm"), Some(&n("upperarm")), elbow));
        v.push(j(n("lowerarm_twist_01"), Some(&n("lowerarm")), elbow + 8.0 * dir));
        v.push(j(n("lowerarm_twist_02"), Some(&n("lowerarm")), elbow + 16.0 * dir));
        let wrist = elbow + 25.0 * dir;
        v.push(j(n("hand"), Some(&n("lowerarm")), wrist));
        let side = Vec3::Z;
        let mut parent = n("hand");
        for k in 1..=3 {
            let name = n(&format!("thumb_{k:02}"));
            v.push(j(
                name.clone(),
                Some(&parent),
                wrist + dir * (2.0 + 3.0 * k as f32) + side * 4.0,
            ));
            parent = name;
        }
        for (finger, z) in [("index", 2.5f32), ("middle", 0.5), ("ring", -1.5), ("pinky", -3.5)] {
            let meta = n(&format!("{finger}_metacarpal"));
            v.push(j(meta.clone(), Some(&n("hand")), wrist + dir * 1.0 + side * z));
            let mut parent = meta;
            for k in 1..=3 {
                let name = n(&format!("{finger}_{k:02}"));
                v.push(j(
                    name.clone(),
                    Some(&parent),
                    wrist + dir * (7.0 + 3.0 * k as f32) + side * z,
                ));
                parent = name;
            }
        }
        let hip = Vec3::new(9.0 * x, 90.0, 0.0);
        let knee = Vec3::new(9.0 * x, 50.0, 1.0);
        v.push(j(n("thigh"), Some("pelvis"), hip));
        v.push(j(n("thigh_twist_01"), Some(&n("thigh")), (hip + knee) / 2.0));
        v.push(j(n("calf"), Some(&n("thigh")), knee));
        v.push(j(n("calf_twist_01"), Some(&n("calf")), Vec3::new(9.0 * x, 30.0, 0.5)));
        v.push(j(n("foot"), Some(&n("calf")), Vec3::new(9.0 * x, 8.0, 0.0)));
        v.push(j(n("ball"), Some(&n("foot")), Vec3::new(9.0 * x, 2.0, 13.0)));
    }
    assert_eq!(v.len(), 76);
    build(&v, Axes::XAlong)
}

/// `frames` frames at 30 a second of every bone swinging about its own axis and the hips
/// bobbing and walking forward.
#[must_use]
pub fn walk(sk: &Skeleton, frames: usize) -> Clip {
    let mut clip = Clip::default();
    for f in 0..frames {
        let t = f as f32 / 30.0;
        let pose = sk
            .bones()
            .iter()
            .enumerate()
            .map(|(i, b)| {
                let k = i as f32;
                let axis = Vec3::new((k * 1.3).sin(), (k * 0.7).cos(), (k * 2.1).sin() + 0.3).normalize();
                let mut tr = b.rest;
                tr.rotation = (b.rest.rotation * Quat::from_axis_angle(axis, 0.4 * (TAU * t + k).sin())).normalize();
                if b.parent.is_none() {
                    tr.translation += Vec3::new(0.0, 3.0 * (2.0 * TAU * t).sin(), 140.0 * t);
                }
                tr
            })
            .collect();
        clip.times.push(t);
        clip.poses.push(pose);
    }
    clip
}

/// `frames` frames at 30 a second of a walk: the hips travel (or, `in_place`, stay put while the
/// feet slide back under them) and each foot is planted for 55% of each 0.9 s step cycle, reached
/// with two-bone IK.
#[must_use]
pub fn gait(sk: &Skeleton, frames: usize, in_place: bool) -> Clip {
    use std::f32::consts::PI;
    let legs = mummer::contact::legs(sk);
    let rest = sk.rest();
    let world = sk.world(&rest);
    let (lo, hi) = world.iter().fold((f32::MAX, f32::MIN), |(lo, hi), p| {
        (lo.min(p.position().y), hi.max(p.position().y))
    });
    let h = hi - lo;
    let (speed, cycle, stance) = (0.5 * h, 0.9f32, 0.55f32);
    let root = (0..sk.len()).find(|&b| sk.bones()[b].parent.is_none()).expect("a root");
    let mut clip = Clip::default();
    for f in 0..frames {
        let t = f as f32 / 30.0;
        let travel = if in_place { 0.0 } else { speed * t };
        let mut pose = rest.clone();
        let bob = -0.035 * h + 0.008 * h * (2.0 * TAU * t / cycle).sin();
        pose[root].translation += Vec3::new(0.0, bob, travel);
        for (i, leg) in legs.iter().enumerate() {
            let at = world[leg.end].position();
            let offset = 0.5 * (i % 2) as f32;
            let k = t / cycle + offset;
            let (step, phase) = (k.floor(), k.fract());
            let plant = |c: f32| at.z + speed * (c + stance / 2.0 - offset) * cycle;
            let (z, lift) = if phase < stance {
                (plant(step), 0.0)
            } else {
                let s = (phase - stance) / (1.0 - stance);
                let s2 = s * s * (3.0 - 2.0 * s);
                (
                    plant(step) + (plant(step + 1.0) - plant(step)) * s2,
                    0.06 * h * (PI * s).sin(),
                )
            };
            let z = if in_place { z - speed * t } else { z };
            mummer::contact::reach(sk, &mut pose, leg, Vec3::new(at.x, at.y + lift, z));
        }
        clip.times.push(t);
        clip.poses.push(pose);
    }
    clip
}
