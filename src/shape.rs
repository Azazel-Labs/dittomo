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
    pub names: Vec<Name>,
}

impl Shape {
    pub(crate) fn of(skeleton: &Skeleton) -> Self {
        let rest = skeleton.world(&skeleton.rest());
        let children = skeleton.children();
        let names: Vec<Name> = skeleton.bones().iter().map(|b| names::parse(&b.name)).collect();
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
        let mut middle = 0;
        let mut best = f32::INFINITY;
        for i in 0..n {
            let far = along(i).into_iter().filter(|d| d.is_finite()).fold(0.0, f32::max);
            if far < best - 1e-6 * height {
                best = far;
                middle = i;
            }
        }
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

        let frame = body_frame(&pos, &names, &children, height, floor);
        let mut under = vec![1usize; n];
        for i in (0..n).rev() {
            if let Some(p) = skeleton.bones()[i].parent {
                under[p] += under[i];
            }
        }
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
            names,
        }
    }
}

/// The body's left, up and forward, as a rotation from them to the world's x, y and z. Up is the world's up (glTF's
/// y). Left is across the plane the bones are most nearly mirrored in, towards the bones named left; failing names,
/// towards where the feet point being forward; failing that, facing the world's +z, as glTF characters do.
fn body_frame(pos: &[Vec3], names: &[Name], children: &[Vec<usize>], height: f32, floor: f32) -> Quat {
    let n = pos.len() as f32;
    let centre = pos.iter().copied().sum::<Vec3>() / n;
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
