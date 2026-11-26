//! Where a hand is by its body, said the same way for any part of it: how far along the part, which way out from it,
//! and how far off its surface. Said so against the source's body and read back against the target's, a hand on a hip
//! lands on the target's hip and one on a knee on its knee, however the two are built.

use std::f32::consts::TAU;

use glam::{Quat, Vec3};

use super::{At, Body, Limb, Scratch, legs_of, paths, place, played, reach_in};
use crate::shape::Shape;
use crate::surface::{Frame, Surface};
use crate::{Mapping, Skeleton, Transform};

/// How many places along a part, and round it, its surface is known at.
const ALONG: usize = 6;
const ROUND: usize = 8;

/// Within `NEAR` of a part's surface (over the height) a hand is wholly by it; past `FAR`, not at all.
const NEAR: f32 = 0.05;
const FAR: f32 = 0.15;

/// How far off a part (over the height) still counts toward where a hand goes.
const WIDE: f32 = 0.45;

/// A hand is brought all the way to its place from within `SURE` of it (over the height), and not at all from past
/// `UNSURE`.
const SURE: f32 = 0.03;
const UNSURE: f32 = 0.10;

/// How much of an arm's length, toward its full reach, is given up softly.
const SOFT: f32 = 0.1;

/// How far along a trunk, from between its hips' joints to its head's, its shoulders count as.
const SHOULDERS: f32 = 0.6;

/// A part of one body: a line through joints (a trunk from hips to head, a thigh, a forearm), or a ball on one (a
/// head), with how far its surface is from it along and round.
struct Side {
    /// The joints the line runs through, or the one carrying the ball.
    joints: Vec<usize>,
    /// A ball's middle in its joint's frame.
    center: Vec3,
    /// Where the line passes each joint, in the joint's frame: at it, but for a trunk's first, which is between the
    /// legs' joints, wherever the hips' own joint is.
    offsets: Vec<Vec3>,
    /// Each joint's turn at rest, undone.
    rest: Vec<Quat>,
    /// The body's left, up and forward at rest.
    axes: [Vec3; 3],
    /// How far along the line each joint is, from 0 to 1.
    cuts: Vec<f32>,
    /// How far out the surface is: along the line (or up a ball, bottom to top), then round it from the front.
    radius: [[f32; ROUND]; ALONG],
}

/// A point by a part: how far along, which way out in the part's own frame, and how far off the surface.
#[derive(Clone, Copy)]
struct By {
    along: f32,
    out: Vec3,
    gap: f32,
}

impl Side {
    /// A line through joints, the first passed at `start` in its frame. With a `mark` (a point at rest, and how far
    /// along the line it is to count as), the line is measured so the point nearest it is that far along: a trunk's
    /// shoulders are as far along one body's as another's, wherever their spines' joints are.
    fn line<F: Frame>(
        joints: Vec<usize>,
        start: Vec3,
        mark: Option<(Vec3, f32)>,
        rest: &[F],
        (axes, height): ([Vec3; 3], f32),
    ) -> Option<Self> {
        let offsets: Vec<Vec3> = (0..joints.len())
            .map(|i| if i == 0 { start } else { Vec3::ZERO })
            .collect();
        let at = |i: usize| rest[joints[i]].point(offsets[i]);
        let lengths: Vec<f32> = (1..joints.len()).map(|i| at(i - 1).distance(at(i))).collect();
        let whole: f32 = lengths.iter().sum();
        if joints.len() < 2 || whole <= 1e-6 * height {
            return None;
        }
        let mut cuts = vec![0.0];
        for l in &lengths {
            cuts.push(cuts[cuts.len() - 1] + l / whole);
        }
        if let Some((p, share)) = mark {
            let found = (0..joints.len() - 1)
                .map(|i| {
                    let (a, b) = (at(i), at(i + 1));
                    let t = ((p - a).dot(b - a) / (b - a).length_squared().max(1e-20)).clamp(0.0, 1.0);
                    (a.lerp(b, t).distance_squared(p), cuts[i] + t * (cuts[i + 1] - cuts[i]))
                })
                .min_by(|x, y| x.0.total_cmp(&y.0))
                .map_or(share, |(_, u)| u);
            if found > 1e-3 && found < 1.0 - 1e-3 {
                for c in &mut cuts {
                    *c = if *c <= found {
                        *c / found * share
                    } else {
                        share + (*c - found) / (1.0 - found) * (1.0 - share)
                    };
                }
            }
        }
        Some(Self {
            offsets,
            rest: joints.iter().map(|&b| rest[b].turn().inverse()).collect(),
            joints,
            center: Vec3::ZERO,
            axes,
            cuts,
            radius: [[0.0; ROUND]; ALONG],
        })
    }

    /// The part's frame at joint `i`, the bones placed in `world`: across, along and the third, with `along` the way
    /// the line runs there (a ball's is the body's up as its joint carries it).
    fn frame<F: Frame>(&self, world: &[F], i: usize, along: Option<Vec3>) -> [Vec3; 3] {
        let turn = world[self.joints[i]].turn() * self.rest[i];
        let [left, up, forward] = self.axes.map(|a| turn * a);
        let Some(y) = along else {
            return [left, up, forward];
        };
        let x = (forward - y * forward.dot(y))
            .try_normalize()
            .or_else(|| (left - y * left.dot(y)).try_normalize())
            .unwrap_or(Vec3::X);
        [x, y, y.cross(x)]
    }

    fn at<F: Frame>(&self, world: &[F], i: usize) -> Vec3 {
        world[self.joints[i]].point(self.offsets[i])
    }

    /// The surface's distance at `along` the part and round it the way `out` points.
    fn surface(&self, along: f32, out: Vec3) -> f32 {
        let u = along.clamp(0.0, 1.0) * (ALONG - 1) as f32;
        let a = (out.z.atan2(out.x) / TAU).rem_euclid(1.0) * ROUND as f32;
        let (i, j) = ((u as usize).min(ALONG - 2), (a as usize) % ROUND);
        let (fu, fa) = (u - i as f32, a - a.floor());
        let ring = |i: usize| self.radius[i][j] * (1.0 - fa) + self.radius[i][(j + 1) % ROUND] * fa;
        ring(i) * (1.0 - fu) + ring(i + 1) * fu
    }

    /// Where on the line is nearest `p`: the joint before it, how far to the next, and the point.
    fn nearest<F: Frame>(&self, world: &[F], p: Vec3) -> (usize, f32, Vec3) {
        let at = |i: usize| self.at(world, i);
        (0..self.joints.len() - 1)
            .map(|i| {
                let (a, b) = (at(i), at(i + 1));
                let ab = b - a;
                let t = ((p - a).dot(ab) / ab.length_squared().max(1e-20)).clamp(0.0, 1.0);
                (i, t, a + ab * t)
            })
            .min_by(|x, y| x.2.distance_squared(p).total_cmp(&y.2.distance_squared(p)))
            .unwrap_or((0, 0.0, p))
    }

    /// Where `p` is by the part, and how far from its line or its ball's middle.
    fn by<F: Frame>(&self, world: &[F], p: Vec3) -> (By, f32) {
        let (c, along, [x, y, z]) = if self.cuts.is_empty() {
            (
                world[self.joints[0]].point(self.center),
                None,
                self.frame(world, 0, None),
            )
        } else {
            let (i, t, c) = self.nearest(world, p);
            let at = |i: usize| self.at(world, i);
            let u = self.cuts[i] + t * (self.cuts[i + 1] - self.cuts[i]);
            let dir = (at(i + 1) - at(i)).normalize_or(Vec3::Y);
            (c, Some(u), self.frame(world, i, Some(dir)))
        };
        let d = p - c;
        let far = d.length();
        let out = if far > 1e-9 {
            Vec3::new(d.dot(x), d.dot(y), d.dot(z)) / far
        } else {
            Vec3::X
        };
        // Up a ball, bottom to top.
        let along = along.unwrap_or((out.y + 1.0) / 2.0);
        let by = By {
            along,
            out,
            gap: far - self.surface(along, out),
        };
        (by, far)
    }

    /// The point `by` says, on this part.
    fn point<F: Frame>(&self, world: &[F], by: By) -> Vec3 {
        let (c, [x, y, z]) = if self.cuts.is_empty() {
            (world[self.joints[0]].point(self.center), self.frame(world, 0, None))
        } else {
            let i = self
                .cuts
                .partition_point(|&c| c <= by.along)
                .clamp(1, self.cuts.len() - 1)
                - 1;
            let at = |i: usize| self.at(world, i);
            let t = ((by.along - self.cuts[i]) / (self.cuts[i + 1] - self.cuts[i]).max(1e-9)).clamp(0.0, 1.0);
            let (a, b) = (at(i), at(i + 1));
            (a.lerp(b, t), self.frame(world, i, Some((b - a).normalize_or(Vec3::Y))))
        };
        let far = (self.surface(by.along, by.out) + by.gap).max(0.0);
        c + (x * by.out.x + y * by.out.y + z * by.out.z) * far
    }

    /// Sets the surface from points of skin on the part at rest, where there are enough; elsewhere it stays as it was.
    fn fit<F: Frame>(&mut self, rest: &[F], points: &[Vec3]) {
        if points.len() < 4 * ALONG {
            return;
        }
        let mut sum = [[(0.0f32, 0u32); ROUND]; ALONG];
        for &p in points {
            let (by, far) = self.by(rest, p);
            let i = (by.along.clamp(0.0, 1.0) * (ALONG - 1) as f32).round() as usize;
            let j = ((by.out.z.atan2(by.out.x) / TAU).rem_euclid(1.0) * ROUND as f32).round() as usize % ROUND;
            sum[i][j].0 += far;
            sum[i][j].1 += 1;
        }
        let mean = |cells: &[(f32, u32)]| {
            let (total, count) = cells.iter().fold((0.0, 0), |(t, c), &(s, n)| (t + s, c + n));
            (count > 0).then(|| total / count as f32)
        };
        let all = mean(sum.as_flattened());
        for (i, ring) in sum.iter().enumerate() {
            let round = mean(ring).or(all);
            for (j, &(total, count)) in ring.iter().enumerate() {
                let known = (count > 0).then(|| total / count as f32);
                if let Some(r) = known.or(round) {
                    self.radius[i][j] = r;
                }
            }
        }
    }
}

/// A part of the source's body and the same part of the target's, and the arm it is of, if any: a hand isn't placed by
/// its own arm.
struct Part {
    sides: [Side; 2],
    arm: Option<usize>,
}

/// An arm the two rigs share: the target's limb, the source's hand, each hand's middle in its own frame, and the
/// target's bones to place again once the arm has moved.
struct Reach {
    limb: Limb,
    source: usize,
    middle: [Vec3; 2],
    below: Vec<usize>,
}

/// Keeps each target hand where the source's is by its body: against each part of the body it is near, the hand is as
/// far along the part, the same way out from it and as far off its surface, scaled by height; near several, between
/// where each says, by how near; near none, left where it is. Reached with two-bone IK, so the hand keeps its turn.
pub(crate) struct Charts {
    parts: Vec<Part>,
    arms: Vec<Reach>,
    s_path: Vec<usize>,
    t_path: Vec<usize>,
    heights: [f32; 2],
    /// How much each hand is to be placed: as the pose shows, and as it was last pose.
    on: Vec<f32>,
    eased: Vec<f32>,
}

/// One rig's parts by what they are, to pair with the other's: the trunk, the head, each leg's thigh and shin by its
/// foot, each arm's upper and lower by its hand.
struct Found {
    trunk: Option<Side>,
    legs: Vec<(usize, [Side; 2])>,
    arms: Vec<(usize, [Side; 2])>,
}

fn find(skeleton: &Skeleton, s: &Shape, body: &Body, skin: &Surface, axes: [Vec3; 3]) -> Found {
    let sized = (axes, s.height);
    let rest = &s.rest;
    let at = |b: usize| rest[b].position();
    let parent = |b: usize| skeleton.bones()[b].parent;
    let legs = legs_of(skeleton, s);
    // Half the way between the hips' joints, and between the shoulders'.
    let half = |ends: Vec<usize>, or: f32| match ends[..] {
        [a, b, ..] => at(a).distance(at(b)) / 2.0,
        _ => or,
    };
    let shoulders = half(body.arms.iter().map(|a| a.limb.upper).collect(), body.torso);
    let hips = half(legs.iter().map(|l| l.upper).collect(), 0.8 * shoulders);
    let round = |side: &mut Side, wide: &dyn Fn(f32) -> (f32, f32)| {
        for i in 0..ALONG {
            let (across, deep) = wide(i as f32 / (ALONG - 1) as f32);
            for j in 0..ROUND {
                let (sin, cos) = (j as f32 / ROUND as f32 * TAU).sin_cos();
                side.radius[i][j] = 1.0 / ((cos / deep).powi(2) + (sin / across).powi(2)).sqrt();
            }
        }
    };
    let mut joints = body.spine.clone();
    if let Some(h) = body
        .head
        .filter(|h| super::below(skeleton, h.bone, joints[joints.len() - 1]))
    {
        joints.push(h.bone);
    }
    let chest = body.spine.len() - 1;
    // From between the legs' joints, the shoulders three fifths of the way to the head's.
    let mid = |ends: Vec<usize>| match ends[..] {
        [a, b, ..] => Some((at(a) + at(b)) / 2.0),
        _ => None,
    };
    let start = mid(legs.iter().map(|l| l.upper).collect())
        .map_or(Vec3::ZERO, |p| rest[joints[0]].matrix.inverse().transform_point3(p));
    let mark = mid(body.arms.iter().map(|a| a.limb.upper).collect()).map(|p| (p, SHOULDERS));
    let mut trunk = Side::line(joints, start, mark, rest, sized);
    if let Some(side) = &mut trunk {
        let top = if mark.is_some() {
            SHOULDERS
        } else {
            side.cuts[chest].max(1e-3)
        };
        round(side, &|u| {
            if u <= top {
                let w = 1.4 * hips + (0.9 * shoulders - 1.4 * hips) * (u / top);
                (w, 0.75 * w)
            } else {
                let w = 0.9 * shoulders + (0.4 - 0.9) * shoulders * ((u - top) / (1.0 - top).max(1e-3));
                (w, w.min(0.65 * shoulders))
            }
        });
    }
    let pair = |limb: &Limb, thick: [f32; 3]| -> Option<[Side; 2]> {
        let mut upper = Side::line(vec![limb.upper, limb.lower], Vec3::ZERO, None, rest, sized)?;
        let mut lower = Side::line(vec![limb.lower, limb.end], Vec3::ZERO, None, rest, sized)?;
        round(&mut upper, &|u| {
            let r = thick[0] + (thick[1] - thick[0]) * u;
            (r, r)
        });
        round(&mut lower, &|u| {
            let r = thick[1] + (thick[2] - thick[1]) * u;
            (r, r)
        });
        Some([upper, lower])
    };
    let mut found = Found {
        trunk,
        legs: legs
            .iter()
            .filter_map(|l| Some((l.end, pair(l, [1.1 * hips, 0.9 * hips, 0.55 * hips])?)))
            .collect(),
        arms: body
            .arms
            .iter()
            .filter_map(|a| {
                Some((
                    a.limb.end,
                    pair(&a.limb, [1.4 * body.limb, 1.2 * body.limb, body.limb])?,
                ))
            })
            .collect(),
    };
    if skin.is_empty() {
        return found;
    }
    // Each bone's part: its own if it is on one's line, or that of the first bone above it that is; a hand or a foot
    // and what hangs from it is of none.
    let between = |from: usize, to: usize| {
        let mut bones = vec![to];
        let mut b = parent(from);
        while let Some(x) = b.filter(|&x| x != to) {
            bones.push(x);
            b = parent(x);
        }
        bones
    };
    let mut owner: Vec<Option<Option<usize>>> = vec![None; skeleton.len()];
    let mut sides: Vec<&mut Side> = Vec::new();
    if let Some(side) = &mut found.trunk {
        for &b in &body.spine {
            owner[b] = Some(Some(sides.len()));
        }
        sides.push(side);
    }
    let limbs: Vec<Limb> = legs.iter().copied().chain(body.arms.iter().map(|a| a.limb)).collect();
    for (limb, (_, [upper, lower])) in limbs.iter().zip(found.legs.iter_mut().chain(found.arms.iter_mut())) {
        for (side, bones) in [
            (upper, between(limb.lower, limb.upper)),
            (lower, between(limb.end, limb.lower)),
        ] {
            for b in bones {
                owner[b] = Some(Some(sides.len()));
            }
            sides.push(side);
        }
    }
    for limb in &limbs {
        owner[limb.end] = Some(None);
    }
    for b in 0..skeleton.len() {
        if owner[b].is_none() {
            owner[b] = Some(parent(b).and_then(|p| owner[p].flatten()));
        }
    }
    let mut points: Vec<Vec<Vec3>> = vec![Vec::new(); sides.len()];
    for (p, _, b) in skin.points(rest) {
        if let Some(Some(k)) = owner[b] {
            points[k].push(p);
        }
    }
    for (side, points) in sides.into_iter().zip(&points) {
        side.fit(rest, points);
    }
    drop(limbs);
    found
}

impl Charts {
    /// Nothing if no arm plays the source's, or the bodies share no part.
    pub(crate) fn new(
        map: &Mapping,
        rigs: [(&Skeleton, &Shape, &Surface); 2],
        [s_body, t_body]: [&Body; 2],
    ) -> Option<Self> {
        let [(source, s, s_skin), (target, t, t_skin)] = rigs;
        // Without a skin, halfway to where its fingers start: the bones hanging from it no farther off than half its
        // forearm is long, as a sword's bone or a sleeve's may hang from a hand too.
        let middle = |skeleton: &Skeleton, shape: &Shape, skin: &Surface, hand: usize, lower: usize| {
            skin.center(hand).unwrap_or_else(|| {
                let rest = skeleton.rest();
                let most = 0.5 * shape.rest[lower].position().distance(shape.rest[hand].position());
                let scale = shape.rest[hand].matrix.x_axis.truncate().length().max(1e-9);
                let (sum, count) = shape.children[hand]
                    .iter()
                    .map(|&c| rest[c].translation)
                    .filter(|t| t.length() * scale <= most)
                    .fold((Vec3::ZERO, 0), |(s, n), t| (s + t, n + 1));
                sum / (2 * count.max(1)) as f32
            })
        };
        let arms: Vec<Reach> = played(map, s_body, t_body)
            .into_iter()
            .filter_map(|(limb, a)| {
                let from = s_body.arms.iter().find(|sa| sa.limb.end == a)?.limb;
                Some((limb, a, from))
            })
            .map(|(limb, a, from)| Reach {
                limb,
                source: a,
                middle: [
                    middle(source, s, s_skin, a, from.lower),
                    middle(target, t, t_skin, limb.end, limb.lower),
                ],
                below: paths(target, [limb.end])
                    .into_iter()
                    .filter(|&b| b == limb.upper || super::below(target, b, limb.upper))
                    .collect(),
            })
            .collect();
        if arms.is_empty() {
            return None;
        }
        // The source body's left, up and forward, and the target's as the mapping turns them: the two then agree on
        // which way is which, whichever way each faces.
        let s_axes = [s.frame * Vec3::X, s.frame * Vec3::Y, s.frame * Vec3::Z];
        let mut t_axes = s_axes.map(|a| map.turn() * a);
        let mut s_axes = s_axes;
        // Better, from two arms the rigs share: across from one's shoulder to the other's is the same way on both.
        let shoulder = |r: &Reach| {
            s_body
                .arms
                .iter()
                .find(|sa| sa.limb.end == r.source)
                .map(|sa| sa.limb.upper)
        };
        if let [p, q, ..] = &arms[..]
            && let (Some(sp), Some(sq)) = (shoulder(p), shoulder(q))
        {
            let across = |a: Vec3, b: Vec3| {
                let d = b - a;
                Vec3::new(d.x, 0.0, d.z).try_normalize()
            };
            let from = across(s.rest[sp].position(), s.rest[sq].position());
            let to = across(t.rest[p.limb.upper].position(), t.rest[q.limb.upper].position());
            if let (Some(from), Some(to)) = (from, to) {
                s_axes = [from, Vec3::Y, from.cross(Vec3::Y)];
                t_axes = [to, Vec3::Y, to.cross(Vec3::Y)];
            }
        }
        let (fs, ft) = (
            find(source, s, s_body, s_skin, s_axes),
            find(target, t, t_body, t_skin, t_axes),
        );
        let mut parts = Vec::new();
        if let (Some(a), Some(b)) = (fs.trunk, ft.trunk) {
            parts.push(Part {
                sides: [a, b],
                arm: None,
            });
        }
        let mut s_legs: Vec<_> = fs.legs.into_iter().map(Some).collect();
        for (end, [upper, lower]) in ft.legs {
            let from = map.source_of(end);
            let Some((_, [su, sl])) = s_legs
                .iter_mut()
                .find(|l| l.as_ref().is_some_and(|l| Some(l.0) == from))
                .and_then(Option::take)
            else {
                continue;
            };
            parts.extend([(su, upper), (sl, lower)].map(|(a, b)| Part {
                sides: [a, b],
                arm: None,
            }));
        }
        let mut s_arms: Vec<_> = fs.arms.into_iter().map(Some).collect();
        for (end, [upper, lower]) in ft.arms {
            let Some(i) = arms.iter().position(|r| r.limb.end == end) else {
                continue;
            };
            let Some((_, [su, sl])) = s_arms
                .iter_mut()
                .find(|l| l.as_ref().is_some_and(|l| l.0 == arms[i].source))
                .and_then(Option::take)
            else {
                continue;
            };
            parts.extend([(su, upper), (sl, lower)].map(|(a, b)| Part {
                sides: [a, b],
                arm: Some(i),
            }));
        }
        if parts.is_empty() {
            return None;
        }
        let joints = |i: usize| parts.iter().flat_map(move |p| p.sides[i].joints.iter().copied());
        Some(Self {
            s_path: paths(source, joints(0).chain(arms.iter().map(|a| a.source))),
            t_path: paths(target, joints(1).chain(arms.iter().map(|a| a.limb.end))),
            on: vec![0.0; arms.len()],
            eased: vec![0.0; arms.len()],
            parts,
            arms,
            heights: [s.height, t.height],
        })
    }

    /// The source bones to place ([`super::place_source`]) before [`Self::placed`].
    pub(crate) fn s_path(&self) -> &[usize] {
        &self.s_path
    }

    /// Forgets how far each hand was placed, as after a cut.
    pub(crate) fn reset(&mut self) {
        self.eased.fill(0.0);
    }

    /// Fixes `played`, the target playing a pose of the source placed in `scratch` ([`Self::s_path`]), `dt` seconds
    /// after the pose before: each hand is placed no more than it was last pose but for what `ONTO` allows, so it
    /// comes to its place late when the source's darts there, and leaves it as slowly.
    pub(crate) fn placed(&mut self, target: &Skeleton, dt: f32, played: &mut [Transform], scratch: &mut Scratch) {
        let step = if dt > 0.0 { dt / super::ONTO } else { 1.0 };
        self.weigh(target, played, scratch);
        for (was, on) in self.eased.iter_mut().zip(&mut self.on) {
            *was = on.max(*was - step).min(*was + step);
            *on = *was;
        }
        let on = std::mem::take(&mut self.on);
        self.put(target, played, scratch, &on);
        self.on = on;
    }

    /// Where arm `i`'s hand's middle goes, the bones of both placed, and how much it is to be brought there: nothing
    /// if no part is anywhere near.
    fn aim(&self, i: usize, sw: &[At], tw: &[At]) -> Option<(Vec3, f32)> {
        let [hs, ht] = self.heights;
        let arm = &self.arms[i];
        let from = sw[arm.source].matrix.transform_point3(arm.middle[0]);
        // Between where each part says, the nearer the more; brought there as much as the nearest is near.
        let (mut goal, mut all, mut most) = (Vec3::ZERO, 0.0, 0.0f32);
        for p in self.parts.iter().filter(|p| p.arm != Some(i)) {
            let (by, _) = p.sides[0].by(sw, from);
            let gap = by.gap / hs;
            // Parts count toward where from farther off than they bring a hand there, so that a hand eased off its
            // place still has one.
            let wide = super::smooth(1.0 - (gap - NEAR) / (WIDE - NEAR));
            if wide <= 0.0 {
                continue;
            }
            let to = By {
                gap: by.gap * ht / hs,
                ..by
            };
            let share = wide / (gap.max(0.0) + 0.01).powi(2);
            goal += p.sides[1].point(tw, to) * share;
            all += share;
            most = most.max(super::smooth(1.0 - (gap - NEAR) / (FAR - NEAR)));
        }
        if all <= 0.0 {
            return None;
        }
        let goal = goal / all;
        // Only what is nearly there already is brought there: a broad body's hands hang far from its thighs, and
        // aren't to be pulled in because a slight one's hang by its own.
        let now = tw[arm.limb.end].matrix.transform_point3(arm.middle[1]);
        let sure = super::smooth(1.0 - (goal.distance(now) / ht - SURE) / (UNSURE - SURE));
        Some((goal, most * sure))
    }

    /// Sets how much each hand is to be placed ([`Self::on`]), as this pose alone shows.
    pub(crate) fn weigh(&mut self, target: &Skeleton, played: &[Transform], scratch: &mut Scratch) {
        let Scratch { sw, tw, .. } = scratch;
        place(target, played, &self.t_path, tw);
        for i in 0..self.arms.len() {
            self.on[i] = self.aim(i, sw, tw).map_or(0.0, |(_, w)| w);
        }
    }

    /// How much each hand is to be placed, as [`Self::weigh`] set it.
    pub(crate) fn on(&self) -> &[f32] {
        &self.on
    }

    /// Places each hand `on` of the way to where it goes.
    pub(crate) fn put(&self, target: &Skeleton, played: &mut [Transform], scratch: &mut Scratch, on: &[f32]) {
        let Scratch { sw, tw, .. } = scratch;
        let [hs, ht] = self.heights;
        let mut ready = false;
        for (i, arm) in self.arms.iter().enumerate() {
            if on[i] <= 0.0 {
                continue;
            }
            if !ready {
                place(target, played, &self.t_path, tw);
                ready = true;
            }
            let Some((to, _)) = self.aim(i, sw, tw) else {
                continue;
            };
            let from = sw[arm.source].matrix.transform_point3(arm.middle[0]);
            let end: &At = &tw[arm.limb.end];
            let now = end.matrix.transform_point3(arm.middle[1]);
            let mut goal = now.lerp(to, on[i]);
            // Between two parts' answers it can come nearer one than the source's is to its own: then out to as far
            // off that part as the source's, the nearer the source's is to it the more.
            for p in self.parts.iter().filter(|p| p.arm != Some(i)) {
                let (by, _) = p.sides[0].by(sw, from);
                let w = on[i] * super::smooth(1.0 - (by.gap / hs - NEAR) / (FAR - NEAR));
                if w <= 0.0 {
                    continue;
                }
                let (at, _) = p.sides[1].by(tw, goal);
                let want = by.gap * ht / hs;
                if at.gap < want {
                    let out = By { gap: want, ..at };
                    goal = goal.lerp(p.sides[1].point(tw, out), w);
                }
            }
            // Toward its full length an arm's elbow moves ever faster for the same move of its hand: the last tenth of
            // its reach is given up softly, never bending an arm straighter than this as it was played.
            let (a, b, c) = (
                tw[arm.limb.upper].position(),
                tw[arm.limb.lower].position(),
                end.position(),
            );
            let mut wrist = c + goal - now;
            let (length, was, far) = (a.distance(b) + b.distance(c), a.distance(c), a.distance(wrist));
            let soft = SOFT * length;
            if far > length - soft {
                let eased = length - soft * (-(far - (length - soft)) / soft).exp();
                wrist = a + (wrist - a) * (eased.max(far.min(was)) / far);
            }
            reach_in(target, played, tw, &arm.limb, wrist);
            place(target, played, &arm.below, tw);
        }
    }
}
