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
    /// How far along `d` the points it places land, as `(l, o)` with `self.point(p).dot(d) == p.dot(l) + o`: so a
    /// bone's points can be measured along a way without placing each.
    fn along(&self, d: Vec3) -> (Vec3, f32);
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

    fn along(&self, d: Vec3) -> (Vec3, f32) {
        let m = &self.matrix;
        let l = Vec3::new(
            m.x_axis.truncate().dot(d),
            m.y_axis.truncate().dot(d),
            m.z_axis.truncate().dot(d),
        );
        (l, m.w_axis.truncate().dot(d))
    }
}

/// Points on the skin, by bone, so that only those near some place need posing: each bone's in a ball, and split in
/// smaller balls.
pub(crate) struct Balls {
    bunches: Vec<Bunch>,
    balls: Vec<Ball>,
}

impl Balls {
    /// Each bone's ball placed in the world as `world` has the bones: its middle, its radius, and how much the bone
    /// scales lengths. Placed once, it serves every [`Surface::posed_near`] while those bones keep still.
    pub(crate) fn place<F: Frame>(&self, world: &[F], out: &mut Vec<Around>) {
        out.clear();
        out.extend(self.bunches.iter().map(|bunch| {
            let w = &world[bunch.bone];
            let scale = w.scale();
            Around {
                center: w.point(bunch.center),
                radius: bunch.radius * scale,
                scale,
            }
        }));
    }
}

/// How far a bone's points reach some way, as `p.dot(l) + o` for a point `p` in its frame, and how much the bone
/// stretches lengths that way: `(l, o, l.length())`.
pub(crate) type Toward = (Vec3, f32, f32);

/// A bone's ball of skin placed in the world ([`Balls::place`]).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Around {
    center: Vec3,
    radius: f32,
    scale: f32,
}

impl Around {
    /// How far `p` is from the nearest of these balls, at least: nothing in them is nearer.
    pub(crate) fn nearest(around: &[Self], p: Vec3) -> f32 {
        around
            .iter()
            .map(|a| a.center.distance(p) - a.radius)
            .fold(f32::INFINITY, f32::min)
    }
}

/// One bone's points in a ball round them, in its frame, and the smaller balls they are split in.
struct Bunch {
    bone: usize,
    center: Vec3,
    radius: f32,
    half: Vec3,
    balls: std::ops::Range<usize>,
}

/// A few of a bone's points on the skin, each with the way it faces, in its frame, and a ball round them.
struct Ball {
    center: Vec3,
    radius: f32,
    /// Half the box round them, which `center` is the middle of.
    half: Vec3,
    at: Vec<Vec3>,
    normal: Vec<Vec3>,
}

/// The middle of the box round these points, how far the farthest is from it, and half the box.
fn ball(points: impl Iterator<Item = Vec3> + Clone) -> (Vec3, f32, Vec3) {
    let (lo, hi) = points
        .clone()
        .fold((Vec3::MAX, Vec3::MIN), |(lo, hi), p| (lo.min(p), hi.max(p)));
    let center = (lo + hi) / 2.0;
    let radius = points.map(|p| p.distance(center)).fold(0.0, f32::max);
    (center, radius, (hi - lo).max(Vec3::ZERO) / 2.0)
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

    /// Every point as the bones placed in `world` carry it: where it is, the way it faces, and its bone. For drawing
    /// the skin, or measuring against it.
    #[must_use]
    pub fn points(&self, world: &[Placed]) -> Vec<(Vec3, Vec3, usize)> {
        (0..self.at.len())
            .map(|i| {
                let w = &world[self.bone[i]];
                (w.point(self.at[i]), w.turn() * self.normal[i], self.bone[i])
            })
            .collect()
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
            let (center, radius, half) = ball(points.iter().map(|&i| self.at[i]));
            let start = out.balls.len();
            self.split(points, &mut out.balls);
            out.bunches.push(Bunch {
                bone,
                center,
                radius,
                half,
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
        let (center, radius, half) = ball(points.iter().map(|&i| self.at[i]));
        balls.push(Ball {
            center,
            radius,
            half,
            at: points.iter().map(|&i| self.at[i]).collect(),
            normal: points.iter().map(|&i| self.normal[i]).collect(),
        });
    }

    /// [`Self::posed`], but only the points in `balls` that [`Self::near`] could weigh for `p`: those not so far from
    /// `p` that the nearest point and the smoothing rule them out. [`Self::near`] finds the same skin in these as in all
    /// of them. `around` is `balls` placed by `world` ([`Balls::place`]).
    pub(crate) fn posed_near<F: Frame>(
        &self,
        balls: &Balls,
        around: &[Around],
        world: &[F],
        p: Vec3,
        out: &mut Vec<(Vec3, Vec3)>,
    ) {
        out.clear();
        // The nearest point is no farther than the farthest any bone's points can be, nor than the nearest in the ball
        // likeliest to hold it: the nearest-reaching one of the nearest-reaching bone.
        let (mut nearest, mut likeliest, mut low) = (f32::INFINITY, None, f32::INFINITY);
        for (bunch, a) in balls.bunches.iter().zip(around) {
            let d = a.center.distance(p);
            let near = d - a.radius;
            nearest = nearest.min(d + a.radius);
            if near < low {
                (low, likeliest) = (near, Some((bunch, a.scale)));
            }
        }
        if let Some((bunch, scale)) = likeliest {
            let (w, mut low) = (&world[bunch.bone], f32::INFINITY);
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
        // A ball is out of reach if its middle is farther than its radius and that, compared squared.
        let reach = within.sqrt();
        let outside = |d2: f32, r: f32| d2 > (r + reach) * (r + reach);
        for (bunch, a) in balls.bunches.iter().zip(around) {
            if outside(a.center.distance_squared(p), a.radius) {
                continue;
            }
            let w = &world[bunch.bone];
            let turn = w.turn();
            for ball in &balls.balls[bunch.balls.clone()] {
                if outside(w.point(ball.center).distance_squared(p), ball.radius * a.scale) {
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
    /// `p` moves: a spot on it and the way it faces there, from which [`Self::out_from`] tells how far out `p` is.
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
    /// on `hand` to the nearest skin of the body not on any bone under `away` (both arms from the shoulder down, say),
    /// less how far the hand and its fingers reach that way. Below zero the hand is in the body. `None` if either has
    /// no skin. The body is the bones `hips` hangs from and those hanging from it: a bone hung beside the hips, as a
    /// 3ds Max Biped's prop bone carrying the sword in its hand, moves on its own, and its skin is not the body's.
    #[must_use]
    pub fn gap(&self, skeleton: &Skeleton, world: &[Placed], hand: usize, away: &[usize], hips: usize) -> Option<f32> {
        let off = under(skeleton, away);
        let body = on_body(skeleton, hips);
        let mut points = Vec::new();
        self.posed(world, |b| body[b] && !off[b], &mut points);
        let on = under(skeleton, &[hand]);
        let fingers = self.balls(|b| on[b]);
        let center = world.get(hand)?.matrix.transform_point3(self.center(hand)?);
        self.clearance(&points, world, center, &fingers, &mut Vec::new())
            .map(|(g, _)| g)
    }

    /// How far out from the posed `points` a hand whose middle is at `center` is, less how far its own points (`hand`,
    /// posed by `world`) reach toward the skin; and the way it is out ([`Self::out_from`]). `frames` is room to work in.
    pub(crate) fn clearance<F: Frame>(
        &self,
        points: &[(Vec3, Vec3)],
        world: &[F],
        center: Vec3,
        hand: &Balls,
        frames: &mut Vec<(Toward, f32)>,
    ) -> Option<(f32, Vec3)> {
        let (spot, facing) = self.near(points, center)?;
        let (out, facing) = self.out_from(center - spot, facing);
        // How far toward the skin a bone's points reach past the middle, as `p.dot(l) + o` for a point `p` in its
        // frame, and how much it stretches lengths that way.
        let toward = |w: &F| {
            let (l, o) = w.along(-facing);
            (l, o + center.dot(facing), l.length())
        };
        // How far toward the skin a ball's points can reach, at most (no farther than its ball or its box), and do.
        let most = |(l, o, n): Toward, at: Vec3, r: f32, half: Vec3| at.dot(l) + o + (r * n).min(half.dot(l.abs()));
        let farthest =
            |(l, o, _): Toward, ball: &Ball| ball.at.iter().map(|&at| at.dot(l)).fold(f32::MIN, f32::max) + o;
        // A ball likely to reach farthest first (that of the bone that could), then only those that could reach farther
        // than its points do.
        frames.clear();
        frames.extend(hand.bunches.iter().map(|bunch| {
            let t = toward(&world[bunch.bone]);
            (t, most(t, bunch.center, bunch.radius, bunch.half))
        }));
        let likeliest = hand
            .bunches
            .iter()
            .zip(frames.iter())
            .max_by(|a, b| a.1.1.total_cmp(&b.1.1));
        let first = likeliest.and_then(|(bunch, &(t, _))| {
            hand.balls[bunch.balls.clone()]
                .iter()
                .max_by(|a, b| most(t, a.center, a.radius, a.half).total_cmp(&most(t, b.center, b.radius, b.half)))
                .map(|ball| farthest(t, ball))
        });
        let mut reach = first.unwrap_or(0.0).max(0.0);
        for (bunch, &(t, can)) in hand.bunches.iter().zip(frames.iter()) {
            if can <= reach {
                continue;
            }
            for ball in &hand.balls[bunch.balls.clone()] {
                if most(t, ball.center, ball.radius, ball.half) > reach {
                    reach = reach.max(farthest(t, ball));
                }
            }
        }
        Some((out - reach, facing))
    }

    /// How far out a point is from the skin, given the way from the nearest spot on it ([`Self::near`]) to the point
    /// and the way the skin faces there, below zero inside; and the way it is out. Under skin sampled all round, the way
    /// from the spot runs along the facing; running well off to the side of it, the point is past the edge of the skin
    /// sampled (a sleeve left open where the arm is cut off, a face whose middle isn't known) rather than under it, and
    /// is that far out from the edge, that way. In between, the two blend, so a point moving round the edge doesn't
    /// jump.
    fn out_from(&self, way: Vec3, facing: Vec3) -> (f32, Vec3) {
        let depth = way.dot(facing);
        let side = way - depth * facing;
        // The smoothed spot is off the point's foot by up to about the smoothing's width even under the skin.
        let aside = (side.length() - 2.0 * self.spacing).max(0.0);
        let edge = depth.hypot(aside);
        let from_edge = (depth * facing + aside * side.normalize_or_zero()).normalize_or(facing);
        let trust = if depth >= 0.0 {
            0.0
        } else {
            let x = (aside / -depth - 0.5).clamp(0.0, 1.0);
            1.0 - x * x * (3.0 - 2.0 * x)
        };
        (
            edge + trust * (depth - edge),
            from_edge.lerp(facing, trust).normalize_or(facing),
        )
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

/// Which bones are the body's, given its hips: those the hips hang from, and those hanging from them. A bone hung from
/// one above the hips but beside them, as a 3ds Max Biped's prop bone beside its pelvis, carrying the sword the hand
/// holds, moves on its own: its skin is not the body's, and a hand on its hilt is not resting on the body.
pub(crate) fn on_body(skeleton: &Skeleton, hips: usize) -> Vec<bool> {
    let mut on = under(skeleton, &[hips]);
    let mut b = skeleton.bones().get(hips).and_then(|x| x.parent);
    while let Some(p) = b {
        on[p] = true;
        b = skeleton.bones()[p].parent;
    }
    on
}

/// Which bones are any of `tops` or hang under one.
pub(crate) fn under(skeleton: &Skeleton, tops: &[usize]) -> Vec<bool> {
    let mut on = vec![false; skeleton.len()];
    for b in 0..skeleton.len() {
        on[b] = tops.contains(&b) || skeleton.bones()[b].parent.is_some_and(|p| on[p]);
    }
    on
}
