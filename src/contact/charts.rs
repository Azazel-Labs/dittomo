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

    fn ball<F: Frame>(joint: usize, center: Vec3, radius: f32, rest: &[F], axes: [Vec3; 3]) -> Self {
        Self {
            joints: vec![joint],
            center,
            offsets: vec![Vec3::ZERO],
            rest: vec![rest[joint].turn().inverse()],
            axes,
            cuts: Vec::new(),
            radius: [[radius; ROUND]; ALONG],
        }
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
}

/// One rig's parts by what they are, to pair with the other's: the trunk, the head, each leg's thigh and shin by its
/// foot, each arm's upper and lower by its hand.
struct Found {
    trunk: Option<Side>,
    head: Option<Side>,
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
    let head = body.head.map(|h| Side::ball(h.bone, h.center, h.radius, rest, axes));
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
        head,
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
    if let (Some(side), Some(h)) = (&mut found.head, body.head) {
        owner[h.bone] = Some(Some(sides.len()));
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
        let middle = |skeleton: &Skeleton, shape: &Shape, skin: &Surface, hand: usize| {
            skin.center(hand).unwrap_or_else(|| {
                let kids = &shape.children[hand];
                let rest = skeleton.rest();
                kids.iter().map(|&c| rest[c].translation).sum::<Vec3>() / (2 * kids.len().max(1)) as f32
            })
        };
        let arms: Vec<Reach> = played(map, s_body, t_body)
            .into_iter()
            .map(|(limb, a)| Reach {
                limb,
                source: a,
                middle: [middle(source, s, s_skin, a), middle(target, t, t_skin, limb.end)],
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
        let t_axes = s_axes.map(|a| map.turn() * a);
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
        if let (Some(a), Some(b)) = (fs.head, ft.head) {
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
            parts,
            arms,
            heights: [s.height, t.height],
        })
    }

    /// The source bones to place ([`super::place_source`]) before [`Self::placed`].
    pub(crate) fn s_path(&self) -> &[usize] {
        &self.s_path
    }

    /// Fixes `played`, the target playing a pose of the source placed in `scratch` ([`Self::s_path`]).
    pub(crate) fn placed(&self, target: &Skeleton, played: &mut [Transform], scratch: &mut Scratch) {
        let Scratch { sw, tw, .. } = scratch;
        let [hs, ht] = self.heights;
        let mut ready = false;
        for (i, arm) in self.arms.iter().enumerate() {
            let from = sw[arm.source].matrix.transform_point3(arm.middle[0]);
            // Each part the source's hand is near, and how near; placed against the target's only once one is.
            let mut near = self
                .parts
                .iter()
                .filter(|p| p.arm != Some(i))
                .filter_map(|p| {
                    let (by, _) = p.sides[0].by(sw, from);
                    let w = 1.0 - (by.gap / hs - NEAR) / (FAR - NEAR);
                    (w > 0.0).then(|| (p, by, w.min(1.0)))
                })
                .peekable();
            if near.peek().is_none() {
                continue;
            }
            if !ready {
                place(target, played, &self.t_path, tw);
                ready = true;
            }
            // Between where each part near says, the nearer the more, and as much of the way there from where it is
            // as the nearest is near.
            let (mut goal, mut all, mut most) = (Vec3::ZERO, 0.0, 0.0f32);
            for (p, by, w) in near {
                let to = By {
                    gap: by.gap * ht / hs,
                    ..by
                };
                let share = w / (by.gap.max(0.0) / hs + 0.01).powi(2);
                goal += p.sides[1].point(tw, to) * share;
                all += share;
                most = most.max(w);
            }
            let end: &At = &tw[arm.limb.end];
            let now = end.matrix.transform_point3(arm.middle[1]);
            let goal = now.lerp(goal / all, most);
            reach_in(target, played, tw, &arm.limb, end.position() + goal - now);
            place(target, played, &arm.below, tw);
        }
    }
}
