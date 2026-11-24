//! A body's skin, bound to its bones, so a hand that rests on a hip or a coat in the source can rest on the target's.
//!
//! Bones alone say where joints are, not how thick the body around them is: a man in a frock coat is wider at the hips
//! than a dancer in a crop top, though their hip joints sit alike. A [`Surface`] keeps points spread over the skin at
//! rest, each with the way it faces, held in the frame of the bone it moves with; posed with the skeleton, it gives how
//! far any point is from the body (`Surface::near`). [`crate::Mapping::apply_touching`] uses it to keep a hand as far
//! from the target's body as the source's is from its own.

use glam::{IVec3, Vec3};
use std::collections::HashMap;

use crate::{Placed, Skeleton};

/// Points on a body's skin at rest, each with the way it faces and the bone it moves with, about evenly spaced.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Surface {
    /// The bone each point moves with.
    pub(crate) bone: Vec<usize>,
    /// Each point in its bone's rest frame.
    pub(crate) at: Vec<Vec3>,
    /// The way it faces, in its bone's rest frame.
    pub(crate) normal: Vec<Vec3>,
    /// About how far apart the points are, in the world's units.
    pub(crate) spacing: f32,
}

impl Surface {
    /// The skin of a skeleton at rest, from points on it in the world (where the skeleton's rest places its bones):
    /// each a position, the way the skin faces there, and the bone it mostly moves with. They are thinned to about one
    /// every `spacing` per bone. Points facing their bone (a coat's lining, the inside of a mouth) are left out, as
    /// nothing touches the body from inside.
    #[must_use]
    pub fn new(skeleton: &Skeleton, points: impl IntoIterator<Item = (Vec3, Vec3, usize)>, spacing: f32) -> Self {
        let rest = skeleton.world(&skeleton.rest());
        let n = skeleton.len();
        // Each bone's axis: from it to the child that holds the most bones, or just itself for a leaf.
        let mut under = vec![1usize; n];
        let mut children: Vec<Vec<usize>> = vec![Vec::new(); n];
        for b in (0..n).rev() {
            if let Some(p) = skeleton.bones()[b].parent {
                under[p] += under[b];
                children[p].push(b);
            }
        }
        let axis: Vec<(Vec3, Vec3)> = (0..n)
            .map(|b| {
                let a = rest[b].position();
                let c = children[b]
                    .iter()
                    .max_by_key(|&&c| under[c])
                    .map_or(a, |&c| rest[c].position());
                (a, c)
            })
            .collect();
        let spacing = spacing.max(1e-6);
        let mut cells: HashMap<(usize, IVec3), (Vec3, Vec3, u32)> = HashMap::new();
        for (p, normal, b) in points {
            if b >= n || !p.is_finite() || !normal.is_finite() {
                continue;
            }
            let (a, c) = axis[b];
            let ac = c - a;
            let t = if ac.length_squared() > 0.0 {
                ((p - a).dot(ac) / ac.length_squared()).clamp(0.0, 1.0)
            } else {
                0.0
            };
            if normal.dot(p - (a + ac * t)) <= 0.0 {
                continue;
            }
            let key = (b, (p / spacing).floor().as_ivec3());
            let e = cells.entry(key).or_insert((Vec3::ZERO, Vec3::ZERO, 0));
            e.0 += p;
            e.1 += normal.normalize_or_zero();
            e.2 += 1;
        }
        let mut cells: Vec<_> = cells.into_iter().collect();
        cells.sort_by_key(|((b, k), _)| (*b, k.x, k.y, k.z));
        let mut s = Self {
            spacing,
            ..Self::default()
        };
        for ((b, _), (sum, nsum, count)) in cells {
            let Some(normal) = nsum.try_normalize() else { continue };
            let inv = rest[b].matrix.inverse();
            s.bone.push(b);
            s.at.push(inv.transform_point3(sum / count as f32));
            s.normal.push((rest[b].rotation.inverse() * normal).normalize());
        }
        s
    }

    /// How many points it keeps.
    #[must_use]
    pub fn len(&self) -> usize {
        self.at.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.at.is_empty()
    }

    /// The points posed with their bones placed in `world`, those `keep` allows: (position, normal).
    pub(crate) fn posed(&self, world: &[Placed], keep: impl Fn(usize) -> bool, out: &mut Vec<(Vec3, Vec3)>) {
        out.clear();
        for i in 0..self.at.len() {
            let b = self.bone[i];
            if keep(b) {
                let w = &world[b];
                out.push((w.matrix.transform_point3(self.at[i]), w.rotation * self.normal[i]));
            }
        }
    }

    /// The skin nearest `p` among posed points, smoothed over a few spacings so it doesn't jump from point to point as
    /// `p` moves: a spot on it and the way it faces there. How far out `p` is from it is `(p - spot).dot(facing)`,
    /// below zero inside.
    pub(crate) fn near(&self, points: &[(Vec3, Vec3)], p: Vec3) -> Option<(Vec3, Vec3)> {
        let nearest = points
            .iter()
            .map(|(q, _)| q.distance_squared(p))
            .fold(f32::INFINITY, f32::min);
        if !nearest.is_finite() {
            return None;
        }
        let sigma2 = (2.0 * self.spacing).powi(2);
        let (mut spot, mut facing, mut total) = (Vec3::ZERO, Vec3::ZERO, 0.0f32);
        for (q, n) in points {
            let w = (-(q.distance_squared(p) - nearest) / sigma2).exp();
            if w > 1e-4 {
                spot += *q * w;
                facing += *n * w;
                total += w;
            }
        }
        Some((spot / total, facing.try_normalize()?))
    }

    /// How far a hand is from the rest of the body, the bones placed as `world` has them: from the middle of the skin
    /// on `hand` to the nearest skin not on any bone under `away` (both arms from the shoulder down, say), less how far
    /// the hand and its fingers reach that way. Below zero the hand is in the body. `None` if either has no skin.
    #[must_use]
    pub fn gap(&self, skeleton: &Skeleton, world: &[Placed], hand: usize, away: &[usize]) -> Option<f32> {
        let off = under(skeleton, away);
        let mut points = Vec::new();
        self.posed(world, |b| !off[b], &mut points);
        let fingers = self.on(&under(skeleton, &[hand]));
        let centre = world.get(hand)?.matrix.transform_point3(self.centre(hand)?);
        self.clearance(&points, world, centre, &fingers).map(|(g, _)| g)
    }

    /// The points on bones `on` marks.
    pub(crate) fn on(&self, on: &[bool]) -> Vec<usize> {
        (0..self.len()).filter(|&i| on[self.bone[i]]).collect()
    }

    /// How far out from the posed `points` a hand whose middle is at `centre` is, less how far its own points (`hand`,
    /// indices into this surface, posed by `world`) reach toward the skin; and the way the skin faces there.
    pub(crate) fn clearance(
        &self,
        points: &[(Vec3, Vec3)],
        world: &[Placed],
        centre: Vec3,
        hand: &[usize],
    ) -> Option<(f32, Vec3)> {
        let (spot, facing) = self.near(points, centre)?;
        let reach = hand
            .iter()
            .map(|&i| (world[self.bone[i]].matrix.transform_point3(self.at[i]) - centre).dot(-facing))
            .fold(0.0f32, f32::max);
        Some(((centre - spot).dot(facing) - reach, facing))
    }

    /// Where the points bound to `bone` centre, in its rest frame: the middle of a hand, say, rather than its wrist.
    pub(crate) fn centre(&self, bone: usize) -> Option<Vec3> {
        let (mut sum, mut count) = (Vec3::ZERO, 0usize);
        for (b, at) in self.bone.iter().zip(&self.at) {
            if *b == bone {
                sum += *at;
                count += 1;
            }
        }
        (count > 0).then(|| sum / count as f32)
    }
}

/// Which bones are any of `tops` or hang under one.
pub(crate) fn under(skeleton: &Skeleton, tops: &[usize]) -> Vec<bool> {
    let mut on = vec![false; skeleton.len()];
    for b in 0..skeleton.len() {
        on[b] = tops.contains(&b) || skeleton.bones()[b].parent.is_some_and(|p| on[p]);
    }
    on
}
