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
                names,
            };
        }
        let mut under = vec![1usize; n];
        for i in (0..n).rev() {
            if let Some(p) = skeleton.bones()[i].parent {
                under[p] += under[i];
            }
        }
        let pairs = mirrored(&pos, &children, &under, height);
        let height = stature(&pairs, height, floor);
        let girdles = girdles(&pairs);
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
        let at = pos.iter().map(|p| back * (*p - origin) / height).collect();
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
        let forks = children
            .iter()
            .map(|kids| kids.iter().filter(|&&c| under[c] >= 2).count() >= 2)
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
    let centre = pos.iter().copied().sum::<Vec3>() / n;
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let across = mirrored_limbs(pairs).unwrap_or_else(|| mirror_plane(pos, centre, height));
    let mut left_named = 0.0f32;
    for (p, name) in pos.iter().zip(names) {
        match name.side {
            Side::Left => left_named += (*p - centre).dot(across),
            Side::Right => left_named -= (*p - centre).dot(across),
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
    /// The way from one to the other, flat.
    apart: Vec3,
    /// The bones below the two.
    weight: usize,
    /// How high the two are.
    y: f32,
}

fn mirrored(pos: &[Vec3], children: &[Vec<usize>], under: &[usize], height: f32) -> Vec<Mirrored> {
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let mut out = Vec::new();
    for (p, kids) in children.iter().enumerate() {
        for (k, &i) in kids.iter().enumerate() {
            for &j in &kids[k + 1..] {
                if (pos[i].y - pos[j].y).abs() > 0.05 * height {
                    continue;
                }
                let (di, dj) = (flat(pos[i] - pos[p]), flat(pos[j] - pos[p]));
                let apart = di - dj;
                if apart.length() < 0.02 * height || (di + dj).length() > 0.8 * apart.length() {
                    continue;
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
                    apart: apart.normalize(),
                    weight: under[i] + under[j],
                    y: (pos[i].y + pos[j].y) / 2.0,
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
    let mut big: Vec<&Mirrored> = pairs.iter().collect();
    big.sort_by_key(|m| std::cmp::Reverse(m.weight));
    match big.as_slice() {
        [a, b, ..] if a.parent != b.parent => height.max(1.22 * (a.y.max(b.y) - floor)),
        _ => height,
    }
}

/// The bones the arms and the legs hang from: the parents of the two biggest mirrored pairs, if they differ, the
/// higher pair being the arms.
fn girdles(pairs: &[Mirrored]) -> [Option<usize>; 2] {
    let mut big: Vec<&Mirrored> = pairs.iter().collect();
    big.sort_by_key(|m| std::cmp::Reverse(m.weight));
    match big.as_slice() {
        [a, b, ..] if a.parent != b.parent => {
            let (arms, legs) = if a.y > b.y { (a, b) } else { (b, a) };
            [Some(arms.parent), Some(legs.parent)]
        }
        _ => [None; 2],
    }
}

/// The way across the plane the bones are most nearly mirrored in.
fn mirror_plane(pos: &[Vec3], centre: Vec3, height: f32) -> Vec3 {
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let mirror_error = |a: Vec3| -> f32 {
        pos.iter()
            .map(|p| {
                let q = *p - 2.0 * (*p - centre).dot(a) * a;
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
        let q = *p - 2.0 * (*p - centre).dot(across) * across;
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
