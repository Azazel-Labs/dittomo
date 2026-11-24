//! A body's skin, bound to its bones, so a hand that rests on a hip or a coat in the source can rest on the target's.
//!
//! Bones alone say where joints are, not how thick the body around them is: a man in a frock coat is wider at the hips
//! than a dancer in a crop top, though their hip joints sit alike. A [`Surface`] keeps points spread over the skin at
//! rest, each with the way it faces, held in the frame of the bone it moves with; posed with the skeleton, it gives how
//! far any point is from the body (`Surface::near`). [`crate::Mapping::apply_touching`] uses it to keep a hand as far
//! from the target's body as the source's is from its own.

use glam::{IVec3, Quat, Vec3};
use std::collections::HashMap;

use crate::{Placed, Skeleton};

/// A bone placed in the world, as the skin needs it: where a point in its frame lands, and how it is turned.
pub(crate) trait Frame {
    fn point(&self, p: Vec3) -> Vec3;
    fn turn(&self) -> Quat;
    /// How much it scales lengths, at most.
    fn scale(&self) -> f32;
}

impl Frame for Placed {
    fn point(&self, p: Vec3) -> Vec3 {
        self.matrix.transform_point3(p)
    }

    fn turn(&self) -> Quat {
        self.rotation
    }

    fn scale(&self) -> f32 {
        let m = &self.matrix;
        m.x_axis
            .truncate()
            .length_squared()
            .max(m.y_axis.truncate().length_squared())
            .max(m.z_axis.truncate().length_squared())
            .sqrt()
    }
}

/// Points on the skin, by bone, so that only those near some place need posing: each bone's in a ball, and split in
/// smaller balls.
pub(crate) struct Balls {
    bunches: Vec<Bunch>,
    balls: Vec<Ball>,
}

/// One bone's points in a ball round them, in its frame, and the smaller balls they are split in.
struct Bunch {
    bone: usize,
    center: Vec3,
    radius: f32,
    balls: std::ops::Range<usize>,
}

/// A few of a bone's points on the skin, each with the way it faces, in its frame, and a ball round them.
struct Ball {
    center: Vec3,
    radius: f32,
    at: Vec<Vec3>,
    normal: Vec<Vec3>,
}

/// The middle of the box round these points, and how far the farthest is from it.
fn ball(points: impl Iterator<Item = Vec3> + Clone) -> (Vec3, f32) {
    let (lo, hi) = points
        .clone()
        .fold((Vec3::MAX, Vec3::MIN), |(lo, hi), p| (lo.min(p), hi.max(p)));
    let center = (lo + hi) / 2.0;
    (center, points.map(|p| p.distance(center)).fold(0.0, f32::max))
}

/// How much less than the nearest point's a point's weight in [`Surface::near`] can be and still count, as the most
/// its squared distance can be past the nearest's, over the squared smoothing width.
const WEIGHED: f32 = 9.22; // ln(1e4), and a little.

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
    pub(crate) fn posed<F: Frame>(&self, world: &[F], keep: impl Fn(usize) -> bool, out: &mut Vec<(Vec3, Vec3)>) {
        out.clear();
        for i in 0..self.at.len() {
            let b = self.bone[i];
            if keep(b) {
                let w = &world[b];
                out.push((w.point(self.at[i]), w.turn() * self.normal[i]));
            }
        }
    }

    /// The points `keep` allows, by bone, so that [`Self::posed_near`] needn't pose all of them. A bone's points are
    /// split in halves across the way they spread most, and those again, till each ball holds at most a few dozen, so
    /// a ball near a hand holds little skin far from it.
    pub(crate) fn balls(&self, keep: impl Fn(usize) -> bool) -> Balls {
        let mut groups: Vec<(usize, Vec<usize>)> = Vec::new();
        let mut of: HashMap<usize, usize> = HashMap::new();
        for (i, &b) in self.bone.iter().enumerate() {
            if keep(b) {
                let k = *of.entry(b).or_insert_with(|| {
                    groups.push((b, Vec::new()));
                    groups.len() - 1
                });
                groups[k].1.push(i);
            }
        }
        let mut out = Balls {
            bunches: Vec::with_capacity(groups.len()),
            balls: Vec::new(),
        };
        for (bone, points) in groups {
            let (center, radius) = ball(points.iter().map(|&i| self.at[i]));
            let start = out.balls.len();
            self.split(points, &mut out.balls);
            out.bunches.push(Bunch {
                bone,
                center,
                radius,
                balls: start..out.balls.len(),
            });
        }
        out
    }

    fn split(&self, mut points: Vec<usize>, balls: &mut Vec<Ball>) {
        if points.len() > 32 {
            let (lo, hi) = points.iter().fold((Vec3::MAX, Vec3::MIN), |(lo, hi), &i| {
                (lo.min(self.at[i]), hi.max(self.at[i]))
            });
            let spread = hi - lo;
            let axis = if spread.x >= spread.y.max(spread.z) {
                Vec3::X
            } else if spread.y >= spread.z {
                Vec3::Y
            } else {
                Vec3::Z
            };
            points.sort_by(|&a, &b| self.at[a].dot(axis).total_cmp(&self.at[b].dot(axis)));
            let upper = points.split_off(points.len() / 2);
            self.split(points, balls);
            self.split(upper, balls);
            return;
        }
        let (center, radius) = ball(points.iter().map(|&i| self.at[i]));
        balls.push(Ball {
            center,
            radius,
            at: points.iter().map(|&i| self.at[i]).collect(),
            normal: points.iter().map(|&i| self.normal[i]).collect(),
        });
    }

    /// [`Self::posed`], but only the points in `balls` that [`Self::near`] could weigh for `p`: those not so far from
    /// `p` that the nearest point and the smoothing rule them out. [`Self::near`] finds the same skin in these as in all
    /// of them.
    pub(crate) fn posed_near<F: Frame>(&self, balls: &Balls, world: &[F], p: Vec3, out: &mut Vec<(Vec3, Vec3)>) {
        out.clear();
        // The nearest point is no farther than the farthest any bone's points can be, nor than the nearest in the ball
        // likeliest to hold it: the nearest-reaching one of the nearest-reaching bone.
        let (mut nearest, mut likeliest, mut low) = (f32::INFINITY, None, f32::INFINITY);
        for bunch in &balls.bunches {
            let w = &world[bunch.bone];
            let (d, r) = (w.point(bunch.center).distance(p), bunch.radius * w.scale());
            nearest = nearest.min(d + r);
            if d - r < low {
                (low, likeliest) = (d - r, Some(bunch));
            }
        }
        if let Some(bunch) = likeliest {
            let (w, mut low) = (&world[bunch.bone], f32::INFINITY);
            let scale = w.scale();
            let mut best = None;
            for ball in &balls.balls[bunch.balls.clone()] {
                let d = w.point(ball.center).distance(p) - ball.radius * scale;
                if d < low {
                    (low, best) = (d, Some(ball));
                }
            }
            for &at in best.map_or(&[][..], |b| &b.at[..]) {
                nearest = nearest.min(w.point(at).distance(p));
            }
        }
        // Nor is any point [`Self::near`] weighs farther than this from `p`, squared.
        let within = nearest * nearest + WEIGHED * (2.0 * self.spacing).powi(2);
        let outside = |d: f32, r: f32| d > r && (d - r) * (d - r) > within;
        for bunch in &balls.bunches {
            let w = &world[bunch.bone];
            let scale = w.scale();
            if outside(w.point(bunch.center).distance(p), bunch.radius * scale) {
                continue;
            }
            let turn = w.turn();
            for ball in &balls.balls[bunch.balls.clone()] {
                if outside(w.point(ball.center).distance(p), ball.radius * scale) {
                    continue;
                }
                for (&at, &normal) in ball.at.iter().zip(&ball.normal) {
                    let q = w.point(at);
                    if q.distance_squared(p) <= within {
                        out.push((q, turn * normal));
                    }
                }
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
        let far = nearest + WEIGHED * sigma2;
        for (q, n) in points {
            let d = q.distance_squared(p);
            if d > far {
                continue;
            }
            let w = (-(d - nearest) / sigma2).exp();
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
        let on = under(skeleton, &[hand]);
        let fingers = self.balls(|b| on[b]);
        let center = world.get(hand)?.matrix.transform_point3(self.center(hand)?);
        self.clearance(&points, world, center, &fingers).map(|(g, _)| g)
    }

    /// How far out from the posed `points` a hand whose middle is at `center` is, less how far its own points (`hand`,
    /// posed by `world`) reach toward the skin; and the way the skin faces there.
    pub(crate) fn clearance<F: Frame>(
        &self,
        points: &[(Vec3, Vec3)],
        world: &[F],
        center: Vec3,
        hand: &Balls,
    ) -> Option<(f32, Vec3)> {
        let (spot, facing) = self.near(points, center)?;
        // How far toward the skin a ball's points can reach, at most, and do.
        let most = |w: &F, at: Vec3, r: f32| (w.point(at) - center).dot(-facing) + r * w.scale();
        let farthest = |w: &F, ball: &Ball| {
            ball.at
                .iter()
                .map(|&at| (w.point(at) - center).dot(-facing))
                .fold(f32::MIN, f32::max)
        };
        // The ball that could reach farthest first, then only those that could reach farther than its points do.
        let mut first: Option<(f32, usize, &Ball)> = None;
        for bunch in &hand.bunches {
            let w = &world[bunch.bone];
            for ball in &hand.balls[bunch.balls.clone()] {
                let m = most(w, ball.center, ball.radius);
                if first.is_none_or(|f| m > f.0) {
                    first = Some((m, bunch.bone, ball));
                }
            }
        }
        let mut reach = first.map_or(0.0, |(_, b, ball)| farthest(&world[b], ball)).max(0.0);
        for bunch in &hand.bunches {
            let w = &world[bunch.bone];
            if most(w, bunch.center, bunch.radius) <= reach {
                continue;
            }
            for ball in &hand.balls[bunch.balls.clone()] {
                if most(w, ball.center, ball.radius) > reach {
                    reach = reach.max(farthest(w, ball));
                }
            }
        }
        Some(((center - spot).dot(facing) - reach, facing))
    }

    /// Where the points bound to `bone` center, in its rest frame: the middle of a hand, say, rather than its wrist.
    pub(crate) fn center(&self, bone: usize) -> Option<Vec3> {
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
