//! A skeleton's shape at rest, measured so two skeletons can be compared whatever their size, units or facing: each
//! bone's place in a frame of the body (its left, up and forward) over the body's height, how far it is along the body
//! from its middle, how much body lies beyond it and which way it points.

use glam::{Mat3, Quat, Vec3};

use crate::names::{self, Name, Side};
use crate::{Placed, Skeleton};

pub(crate) struct Shape {
    /// Each bone at rest in the world.
    pub rest: Vec<Placed>,
    pub children: Vec<Vec<usize>>,
    /// The body's frame in the world: its columns the body's left, up and forward.
    pub frame: Quat,
    /// Its height in the world's units.
    pub height: f32,
    /// The world height of its lowest bone.
    pub floor: f32,
    /// The bone in the middle of the body, the one with the least distance along it to the farthest bone.
    pub middle: usize,
    /// Each bone's place in the body's frame over its height, from the spot on the floor under the first bone that
    /// branches (the hips, or a root on the floor under them).
    pub at: Vec<Vec3>,
    /// Which way each bone points, to its biggest child, in the body's frame; a leaf, the way its parent points to it.
    pub dir: Vec<Vec3>,
    /// Which way its parent points to it, in the body's frame; zero for a root.
    pub inward: Vec<Vec3>,
    /// How far it is from its parent, over the height; zero for a root.
    pub length: Vec<f32>,
    /// How far along the body each bone is from the middle bone, over the height.
    pub reach: Vec<f32>,
    /// How far along the body its farthest descendant is, over the height.
    pub beyond: Vec<f32>,
    /// Whether two or more limbs of more than one bone hang from it, as from hips, a chest and a hand.
    pub forks: Vec<bool>,
    /// The bones the arms and the legs hang from, as a chest and hips: the parents of the two biggest pairs of limbs
    /// that mirror each other, the higher pair being the arms.
    pub girdles: [Option<usize>; 2],
    /// The two bones the arms start from, hanging from the chest, then the two the legs start from.
    pub limbs: [Option<[usize; 2]>; 2],
    /// Which limb each bone is in, the arms (0) or the legs (1), and whether on the body's left.
    pub limb: Vec<Option<(usize, bool)>>,
    pub names: Vec<Name>,
}

impl Shape {
    pub(crate) fn of(skeleton: &Skeleton) -> Self {
        Self::build(skeleton, true)
    }

    /// Just the rest pose, the children, the height, the floor and the middle: all playing needs.
    pub(crate) fn light(skeleton: &Skeleton) -> Self {
        Self::build(skeleton, false)
    }

    fn build(skeleton: &Skeleton, whole: bool) -> Self {
        let rest = skeleton.world(&skeleton.rest());
        let children = skeleton.children();
        let names: Vec<Name> = if whole {
            skeleton.bones().iter().map(|b| names::parse(&b.name)).collect()
        } else {
            Vec::new()
        };
        let pos: Vec<Vec3> = rest.iter().map(Placed::position).collect();
        let n = pos.len();
        if n == 0 {
            return Self {
                rest,
                children,
                frame: Quat::IDENTITY,
                height: 1.0,
                floor: 0.0,
                middle: 0,
                at: Vec::new(),
                dir: Vec::new(),
                inward: Vec::new(),
                length: Vec::new(),
                reach: Vec::new(),
                beyond: Vec::new(),
                forks: Vec::new(),
                girdles: [None; 2],
                limbs: [None; 2],
                limb: Vec::new(),
                names,
            };
        }
        let (lo, hi) = pos
            .iter()
            .fold((Vec3::splat(f32::MAX), Vec3::splat(f32::MIN)), |(lo, hi), p| {
                (lo.min(*p), hi.max(*p))
            });
        let floor = lo.y;
        let mut height = hi.y - lo.y;
        if height <= 1e-6 * (hi - lo).length().max(1e-12) {
            height = (hi - lo).max_element();
        }
        if height <= 0.0 {
            height = 1.0;
        }

        // Distances along the bones, from every bone to every other.
        let mut near: Vec<Vec<(usize, f32)>> = vec![Vec::new(); n];
        for (i, b) in skeleton.bones().iter().enumerate() {
            if let Some(p) = b.parent {
                let d = pos[i].distance(pos[p]);
                near[i].push((p, d));
                near[p].push((i, d));
            }
        }
        let along = |from: usize| -> Vec<f32> {
            let mut d = vec![f32::INFINITY; n];
            d[from] = 0.0;
            let mut stack = vec![from];
            while let Some(i) = stack.pop() {
                for &(j, len) in &near[i] {
                    if d[j].is_infinite() {
                        d[j] = d[i] + len;
                        stack.push(j);
                    }
                }
            }
            d
        };
        // How far each bone is from the bone farthest from it. In a tree that bone is always an end of the longest path,
        // so it's the farther of the two ends: three walks per tree rather than one from every bone.
        let mut far = vec![f32::NAN; n];
        for start in 0..n {
            if !far[start].is_nan() {
                continue;
            }
            let farthest = |d: &[f32]| {
                (0..n)
                    .filter(|&i| d[i].is_finite())
                    .max_by(|&i, &j| d[i].total_cmp(&d[j]).then(j.cmp(&i)))
                    .unwrap_or(start)
            };
            let from_a = along(farthest(&along(start)));
            let from_b = along(farthest(&from_a));
            for i in 0..n {
                if from_a[i].is_finite() {
                    far[i] = from_a[i].max(from_b[i]);
                }
            }
        }
        let mut middle = 0;
        let mut best = f32::INFINITY;
        for (i, &far) in far.iter().enumerate() {
            if far < best - 1e-6 * height {
                best = far;
                middle = i;
            }
        }
        if !whole {
            return Self {
                rest,
                children,
                frame: Quat::IDENTITY,
                height,
                floor,
                middle,
                at: Vec::new(),
                dir: Vec::new(),
                inward: Vec::new(),
                length: Vec::new(),
                reach: Vec::new(),
                beyond: Vec::new(),
                forks: Vec::new(),
                girdles: [None; 2],
                limbs: [None; 2],
                limb: Vec::new(),
                names,
            };
        }
        let mut under = vec![1usize; n];
        for i in (0..n).rev() {
            if let Some(p) = skeleton.bones()[i].parent {
                under[p] += under[i];
            }
        }
        let pairs = mirrored(skeleton, &pos, &children, &under, &names, height);
        let height = stature(&pairs, height, floor);
        let (girdles, limbs) = girdles(&pairs);
        let from_middle = along(middle);
        let reach: Vec<f32> = from_middle
            .iter()
            .map(|d| if d.is_finite() { d / height } else { 1.0 })
            .collect();
        let mut beyond = vec![0.0f32; n];
        for i in (0..n).rev() {
            for &c in &children[i] {
                beyond[i] = beyond[i].max(beyond[c] + pos[c].distance(pos[i]) / height);
            }
        }

        let frame = body_frame(&pos, &names, &children, &pairs, height, floor);
        let hub = {
            let roots = (0..n).filter(|&i| skeleton.bones()[i].parent.is_none());
            let mut at = roots.max_by_key(|&i| (under[i], std::cmp::Reverse(i))).unwrap_or(0);
            while let [only] = children[at].as_slice() {
                at = *only;
            }
            at
        };
        let origin = Vec3::new(pos[hub].x, floor, pos[hub].z);
        let back = frame.inverse();
        let at: Vec<Vec3> = pos.iter().map(|p| back * (*p - origin) / height).collect();
        let mut limb = vec![None; n];
        for i in 0..n {
            limb[i] = skeleton.bones()[i].parent.and_then(|p| limb[p]);
            for (r, pair) in limbs.iter().enumerate() {
                if pair.is_some_and(|two| two.contains(&i)) {
                    limb[i] = Some((r, at[i].x > 0.0));
                }
            }
        }
        let dir: Vec<Vec3> = (0..n)
            .map(|i| {
                // To its biggest child: an average of children can cancel out, as a pelvis's spine and legs do.
                let main = children[i].iter().copied().max_by(|&a, &b| {
                    (under[a], beyond[a])
                        .partial_cmp(&(under[b], beyond[b]))
                        .unwrap_or(std::cmp::Ordering::Equal)
                        .then(b.cmp(&a))
                });
                main.map_or(Vec3::ZERO, |c| (back * (pos[c] - pos[i])).normalize_or_zero())
            })
            .collect();
        let inward: Vec<Vec3> = (0..n)
            .map(|i| {
                skeleton.bones()[i]
                    .parent
                    .map_or(Vec3::ZERO, |p| (back * (pos[i] - pos[p])).normalize_or_zero())
            })
            .collect();
        let dir = dir
            .into_iter()
            .zip(&inward)
            .map(|(d, w): (Vec3, &Vec3)| if d == Vec3::ZERO { *w } else { d })
            .collect();
        // A bone marking where a chain ends (a BVH's End Site, Mixamo's `_End`) makes no limb of the one it hangs from,
        // a twist bone say.
        let mut joints = vec![0usize; n];
        for i in (0..n).rev() {
            joints[i] += usize::from(!names[i].end());
            if let Some(p) = skeleton.bones()[i].parent {
                joints[p] += joints[i];
            }
        }
        let forks = children
            .iter()
            .map(|kids| kids.iter().filter(|&&c| joints[c] >= 2).count() >= 2)
            .collect();
        let length = (0..n)
            .map(|i| {
                skeleton.bones()[i]
                    .parent
                    .map_or(0.0, |p| pos[i].distance(pos[p]) / height)
            })
            .collect();
        Self {
            rest,
            children,
            frame,
            height,
            floor,
            middle,
            at,
            dir,
            inward,
            length,
            reach,
            beyond,
            forks,
            girdles,
            limbs,
            limb,
            names,
        }
    }
}

/// The body's left, up and forward, as a rotation from them to the world's x, y and z. Up is the world's up (glTF's
/// y). Left is across the body: the way between limbs that mirror each other (two legs from the hips, two arms from the
/// chest) or, failing those, across the plane the bones are most nearly mirrored in. It's towards the bones named
/// left; failing names, towards where the feet point being forward; failing that, facing the world's +z, as glTF
/// characters do.
fn body_frame(
    pos: &[Vec3],
    names: &[Name],
    children: &[Vec<usize>],
    pairs: &[Mirrored],
    height: f32,
    floor: f32,
) -> Quat {
    let n = pos.len() as f32;
    let center = pos.iter().copied().sum::<Vec3>() / n;
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let across = mirrored_limbs(pairs).unwrap_or_else(|| mirror_plane(pos, center, height));
    let mut left_named = 0.0f32;
    for (p, name) in pos.iter().zip(names) {
        match name.side {
            Side::Left => left_named += (*p - center).dot(across),
            Side::Right => left_named -= (*p - center).dot(across),
            Side::Middle => {}
        }
    }
    let left = if left_named.abs() > 1e-6 * height {
        across * left_named.signum()
    } else {
        // Feet point forward: low bones' steps to their children.
        let ahead = across.cross(Vec3::Y);
        let mut toes = 0.0f32;
        for (i, p) in pos.iter().enumerate() {
            if p.y - floor < 0.15 * height {
                for &c in &children[i] {
                    toes += flat(pos[c] - *p).dot(ahead);
                }
            }
        }
        let facing = if toes.abs() > 0.02 * height {
            ahead * toes.signum()
        } else if ahead.z.abs() > 1e-3 {
            ahead * ahead.z.signum()
        } else {
            ahead * ahead.x.signum()
        };
        // left × up = forward.
        Vec3::Y.cross(facing)
    };
    let forward = left.cross(Vec3::Y);
    Quat::from_mat3(&Mat3::from_cols(left, Vec3::Y, forward)).normalize()
}

/// Two children of one bone that mirror each other: each with as much below it, at one height, on either side of their
/// parent. Fingers, side by side in front of the hand, don't count.
struct Mirrored {
    parent: usize,
    /// The two.
    bones: [usize; 2],
    /// The way from one to the other, flat.
    apart: Vec3,
    /// The bones below the two.
    weight: usize,
    /// How high the two are.
    y: f32,
    /// How far the shorter of the two reaches from their parent, over the height: a limb's length, not an ear's. Zero
    /// if they rise to their ends, as ears and antlers do: limbs hang, or reach out level.
    span: f32,
    /// How far their parent is along the bones from its root, over the height.
    depth: f32,
}

fn mirrored(
    skeleton: &Skeleton,
    pos: &[Vec3],
    children: &[Vec<usize>],
    under: &[usize],
    names: &[Name],
    height: f32,
) -> Vec<Mirrored> {
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let n = pos.len();
    // How far along the bones each bone's farthest descendant is, and where it is.
    let mut far = vec![0.0f32; n];
    let mut tip = pos.to_vec();
    let mut depth = vec![0.0f32; n];
    for i in (0..n).rev() {
        for &c in &children[i] {
            let d = far[c] + pos[c].distance(pos[i]);
            if d > far[i] {
                far[i] = d;
                tip[i] = tip[c];
            }
        }
    }
    let rises = |i: usize| {
        let to = tip[i] - pos[i];
        to.y > 0.5 * to.length()
    };
    for i in 0..n {
        if let Some(p) = skeleton.bones()[i].parent {
            depth[i] = depth[p] + pos[i].distance(pos[p]);
        }
    }
    // What a name says the bone is, an arm or a leg alone as much as a numbered one (which the names read as the whole
    // limb's chain, `arm_joint_L_4` against `arm_joint_R`).
    fn part(n: &Name) -> Vec<&str> {
        n.words
            .iter()
            .map(|w| match w.as_str() {
                "upperarm" => "arm",
                "shin" => "leg",
                w => w,
            })
            .collect()
    }
    // Which way a bone points, to the child with most below it.
    let way = |i: usize| {
        let main = children[i]
            .iter()
            .copied()
            .max_by_key(|&c| (under[c], std::cmp::Reverse(c)));
        main.map(|c| (pos[c] - pos[i]).normalize_or_zero())
    };
    let mut out = Vec::new();
    for (p, kids) in children.iter().enumerate() {
        for (k, &i) in kids.iter().enumerate() {
            for &j in &kids[k + 1..] {
                if (pos[i].y - pos[j].y).abs() > 0.05 * height {
                    continue;
                }
                // Named for one side both, as a front and a back leg's IK targets are, for either side but as
                // different parts (a left front leg's and a right back leg's), or one for a side and one for neither
                // (a hind leg and the tail): not mirror images.
                if let (Some(a), Some(b)) = (names.get(i), names.get(j))
                    && (a.side != Side::Middle || b.side != Side::Middle)
                    && (a.side == b.side || a.side == Side::Middle || b.side == Side::Middle || part(a) != part(b))
                {
                    continue;
                }
                let (di, dj) = (flat(pos[i] - pos[p]), flat(pos[j] - pos[p]));
                let apart = di - dj;
                if apart.length() < 0.02 * height {
                    continue;
                }
                // Both ahead of their parent, they may still be limbs pointing away from it, as a four-legged animal's
                // front legs hang down ahead of the spine bone they hang from. Fingers side by side in front of the
                // hand point on the way they lie.
                let ahead = di + dj;
                if ahead.length() > 0.8 * apart.length() {
                    if ahead.dot(apart).abs() > 0.3 * apart.length_squared() {
                        continue;
                    }
                    let (Some(wi), Some(wj)) = (way(i), way(j)) else {
                        continue;
                    };
                    if (wi + wj).normalize_or_zero().dot(ahead.normalize()) > 0.5 {
                        continue;
                    }
                }
                // As much below each; or, placed as nearly exact mirror images, limbs one of which stops short (an arm
                // with a hand and one without).
                let (a, b) = (under[i].min(under[j]), under[i].max(under[j]));
                let exact = a >= 2 && (di + dj).length() < 0.1 * apart.length();
                if 4 * a < 3 * b && !exact {
                    continue;
                }
                out.push(Mirrored {
                    parent: p,
                    bones: [i, j],
                    apart: apart.normalize(),
                    weight: under[i] + under[j],
                    y: (pos[i].y + pos[j].y) / 2.0,
                    span: if rises(i) && rises(j) {
                        0.0
                    } else {
                        (pos[i].distance(pos[p]) + far[i]).min(pos[j].distance(pos[p]) + far[j]) / height
                    },
                    depth: depth[p] / height,
                });
            }
        }
    }
    out
}

/// The way across the body from limbs that mirror each other, their ways apart averaged as axes (a pair says nothing
/// of which side is which), each weighed by the bones below.
fn mirrored_limbs(pairs: &[Mirrored]) -> Option<Vec3> {
    let (mut xx, mut xz, mut zz) = (0.0f32, 0.0f32, 0.0f32);
    for m in pairs {
        let (v, w) = (m.apart, m.weight as f32);
        xx += w * v.x * v.x;
        xz += w * v.x * v.z;
        zz += w * v.z * v.z;
    }
    if pairs.is_empty() {
        return None;
    }
    let angle = 0.5 * (2.0 * xz).atan2(xx - zz);
    Some(Vec3::new(angle.cos(), 0.0, angle.sin()))
}

/// The body's height, for a skeleton that may stop short of the top of the head (many end at the head's base, with no
/// bone at its top). The shoulders are about five sixths of the way up, so the body is at least 1.22 times as tall as
/// they are high: the two biggest mirrored pairs are the arms and the legs, and the higher are the arms.
fn stature(pairs: &[Mirrored], height: f32, floor: f32) -> f32 {
    match biggest(pairs).as_slice() {
        [a, b, ..] if a.parent != b.parent => height.max(1.22 * (a.y.max(b.y) - floor)),
        _ => height,
    }
}

/// The mirrored pairs long enough to be limbs (not ears, eyes or toes), the biggest first.
fn biggest(pairs: &[Mirrored]) -> Vec<&Mirrored> {
    let mut big: Vec<&Mirrored> = pairs.iter().filter(|m| m.span >= 0.2).collect();
    big.sort_by_key(|m| std::cmp::Reverse(m.weight));
    big
}

/// The bones the arms and the legs hang from: the parents of the two biggest mirrored pairs, if they differ, the arms
/// being the pair farther along the bones from the root (a chest above hips, an animal's shoulders ahead of them,
/// whichever stands higher); and the two pairs.
fn girdles(pairs: &[Mirrored]) -> ([Option<usize>; 2], [Option<[usize; 2]>; 2]) {
    match biggest(pairs).as_slice() {
        [a, b, ..] if a.parent != b.parent => {
            let (arms, legs) = if (a.depth, a.y) > (b.depth, b.y) {
                (a, b)
            } else {
                (b, a)
            };
            (
                [Some(arms.parent), Some(legs.parent)],
                [Some(arms.bones), Some(legs.bones)],
            )
        }
        _ => ([None; 2], [None; 2]),
    }
}

/// The way across the plane the bones are most nearly mirrored in.
fn mirror_plane(pos: &[Vec3], center: Vec3, height: f32) -> Vec3 {
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let mirror_error = |a: Vec3| -> f32 {
        pos.iter()
            .map(|p| {
                let q = *p - 2.0 * (*p - center).dot(a) * a;
                pos.iter().map(|o| o.distance_squared(q)).fold(f32::INFINITY, f32::min)
            })
            .sum()
    };
    let mut across = Vec3::X;
    let mut best = f32::INFINITY;
    for step in 0..180 {
        let t = (step as f32).to_radians();
        let a = Vec3::new(t.cos(), 0.0, t.sin());
        let e = mirror_error(a);
        if e < best {
            best = e;
            across = a;
        }
    }
    // Refine from the mirrored pairs themselves: the average step from each bone to its mirror image.
    let mut sum = Vec3::ZERO;
    for p in pos {
        let q = *p - 2.0 * (*p - center).dot(across) * across;
        let Some(o) = pos
            .iter()
            .min_by(|a, b| a.distance_squared(q).total_cmp(&b.distance_squared(q)))
        else {
            continue;
        };
        let step = flat(*p - *o);
        if step.length() > 0.02 * height {
            sum += if step.dot(across) >= 0.0 { step } else { -step };
        }
    }
    if sum.length() > 1e-9 {
        across = sum.normalize();
    }
    across
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Bone, Transform};

    /// A skeleton from bones named with their parents and world positions, every bone unturned.
    fn placed(joints: &[(&str, Option<&str>, [f32; 3])]) -> Skeleton {
        let at = |n: &str| joints.iter().position(|j| j.0 == n).expect("parent listed");
        let bones = joints.iter().map(|&(name, parent, p)| {
            let from = parent.map_or(Vec3::ZERO, |q| Vec3::from(joints[at(q)].2));
            Bone {
                name: name.to_owned(),
                parent: parent.map(at),
                rest: Transform::new(Vec3::from(p) - from, Quat::IDENTITY),
            }
        });
        Skeleton::new(bones.collect()).expect("parents first")
    }

    #[test]
    fn arms_named_unevenly_are_still_a_pair() {
        // As glTF's CesiumMan names them: the left upper arm numbered, which makes it the whole arm's chain, and the
        // right one not, which makes it the upper arm alone. They're still mirror images, and the chest hangs them.
        let s = placed(&[
            ("Skeleton_torso_joint_1", None, [0.004, 0.571, 0.0]),
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
            ("leg_joint_R_1", Some("Skeleton_torso_joint_1"), [-0.053, 0.516, 0.02]),
            ("leg_joint_R_2", Some("leg_joint_R_1"), [-0.061, 0.296, 0.057]),
            ("leg_joint_R_3", Some("leg_joint_R_2"), [-0.062, 0.072, -0.004]),
        ]);
        let shape = Shape::of(&s);
        let name = |i: Option<usize>| i.map(|i| s.bones()[i].name.as_str());
        assert_eq!(name(shape.girdles[0]), Some("torso_joint_3"));
        assert_eq!(name(shape.girdles[1]), Some("Skeleton_torso_joint_1"));
    }
}
