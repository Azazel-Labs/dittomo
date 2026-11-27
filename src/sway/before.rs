//! The step as it was first written, kept for the tests: [`Sway::pose`] is held to it, pose by pose.
//!
//! It is the plain way round: every turn made a unit one by a root, every direction divided by its length, each rod's
//! nearest point found with a division of its own, and all of it a number at a time. What it reads of a bone is what
//! the step reads, but for the three lists kept here.

use glam::{Affine3A, Quat, Vec3};

use super::{STEP, STEPS, Sway, THICK, carry, nearest};
use crate::Transform;

/// What the first step kept that this one doesn't: by chain bone, which way its tail is from its head in its own
/// frame, which way down is in its parent's frame at rest, and which way that was under the bone above a step ago.
#[derive(Clone, Debug, Default)]
pub(super) struct Before {
    axis: Vec<Vec3>,
    down: Vec<Vec3>,
    lean: Vec<Vec3>,
}

impl Before {
    pub(super) fn push(&mut self, axis: Vec3, down: Vec3) {
        self.axis.push(axis);
        self.down.push(down);
        self.lean.push(Vec3::ZERO);
    }
}

impl Sway {
    /// This sway as the first step would have left it, to step with [`Sway::pose_before`].
    pub(super) fn as_before(&self) -> Self {
        let mut first = self.clone();
        first.before.lean = self.lean.iter().map(|&lean| lean.into()).collect();
        first
    }

    /// [`Sway::place`], as it was.
    fn place_before(&mut self, pose: &[Transform]) {
        let n = self.mats.len() - 1;
        assert_eq!(pose.len(), n, "one transform per bone");
        for (&b, &p) in self.above.iter().zip(&self.above_parent) {
            let local = &pose[b];
            let rotation = local.rotation.normalize();
            self.mats[b] =
                self.mats[p] * Affine3A::from_scale_rotation_translation(local.scale, rotation, local.translation);
            self.rots[b] = (self.rots[p] * rotation).normalize();
            self.heads[b] = self.mats[b].translation.into();
        }
    }

    /// [`Sway::pose`], as it was. A sway stepped by this one must be stepped by it alone.
    pub(super) fn pose_before(&mut self, dt: f32, pose: &mut [Transform]) {
        self.place_before(pose);
        let moving = self.live && dt.is_finite() && dt > 0.0;
        let steps = if moving {
            (dt / STEP - 1e-3).ceil().clamp(1.0, STEPS)
        } else {
            1.0
        };
        let h = if moving { dt / steps } else { self.step };
        if moving && h != self.step {
            for i in 0..self.bones.len() {
                self.links[i].carry = carry(h, self.stiff[i], self.damp[i]);
            }
            self.step = h;
        }
        let per_h = 1.0 / h;
        let rods = self.rods.len();
        let collide = moving && self.body && rods > 0;
        let mut now = vec![[Vec3::ZERO; 2]; rods * STEPS as usize];
        let mut near = Vec::new();
        for (k, rod) in self.rods.iter().enumerate() {
            let ends = rod.ends.map(|(b, at)| self.mats[b].transform_point3(at));
            let from = std::mem::replace(&mut self.was[k], ends);
            if !collide {
                continue;
            }
            let (mut step, last) = (1.0, steps as usize - 1);
            for at in 0..last {
                let t = step / steps;
                now[at * rods + k] = [from[0].lerp(ends[0], t), from[1].lerp(ends[1], t)];
                step += 1.0;
            }
            now[last * rods + k] = ends;
        }
        for c in 0..self.chains.len() {
            let hangs = self.chains[c].parent;
            let (turn, start) = (self.rots[hangs], self.heads[hangs] + self.rots[hangs] * self.offsets[c]);
            let range = self.starts[c]..self.starts[c + 1];
            if !moving {
                let (mut head, mut up) = (start, turn);
                for i in range {
                    let b = self.bones[i];
                    let (length, cos, sin) = (self.length[i], self.links[i].cos, self.links[i].sin);
                    let rested = (up * self.rest[i]).normalize();
                    let toward = rested * self.before.axis[i];
                    let goal = head + length * toward;
                    let turned = if self.live {
                        let out = self.at[i] - head;
                        let far = out.length_squared();
                        let dir = if far > 1e-12 * length * length {
                            out / far.sqrt()
                        } else {
                            toward
                        };
                        let dir = cone(dir, toward, cos, sin).map_or(dir, |(dir, _)| dir);
                        self.at[i] = head + length * dir;
                        (arc(toward, dir) * rested).normalize()
                    } else {
                        (self.at[i], self.speed[i]) = (goal, Vec3::ZERO);
                        rested
                    };
                    self.goal[i] = goal;
                    self.before.lean[i] = up * self.before.down[i];
                    pose[b].rotation = if self.live {
                        (up.inverse() * turned).normalize()
                    } else {
                        self.rest[i]
                    };
                    (self.rots[b], self.heads[b]) = (turned, head);
                    (head, up) = (self.at[i], turned);
                }
                (self.root_at[c], self.root_turn[c]) = (start, turn);
                continue;
            }
            if self.still {
                let (mut head, mut up) = (start, turn);
                for i in range.clone() {
                    up = (up * self.rest[i]).normalize();
                    head += self.length[i] * (up * self.before.axis[i]);
                    self.speed[i] = (head - self.goal[i]) / dt;
                }
            }
            let (from_at, from_turn) = (self.root_at[c], self.root_turn[c]);
            let to_turn = if from_turn.dot(turn) < 0.0 { -turn } else { turn };
            let within = self.chains[c].length + THICK * self.height;
            let (mut k, mut root_was) = (1.0, from_at);
            while k <= steps {
                let last = k == steps;
                let (mut head, mut up) = if last {
                    (start, turn)
                } else {
                    let t = k / steps;
                    (from_at.lerp(start, t), from_turn.lerp(to_turn, t).normalize())
                };
                let placed = if collide {
                    let at = (k as usize - 1) * rods;
                    &now[at..at + rods]
                } else {
                    &[]
                };
                near.clear();
                for (r, &[a, b]) in placed.iter().enumerate() {
                    let reach = within + self.rods[r].radius;
                    if head.distance_squared(nearest(a, b, head)) < reach * reach {
                        near.push(r);
                    }
                }
                let mut head_was = std::mem::replace(&mut root_was, head);
                let mut drift = (head - head_was) * per_h;
                for i in range.clone() {
                    let (length, link) = (self.length[i], self.links[i]);
                    let rested = up * self.rest[i];
                    let toward = rested * self.before.axis[i];
                    let goal = head + length * toward;
                    let pace = (goal - self.goal[i]) * per_h;
                    let lean = up * self.before.down[i];
                    let carried = 0.5 * (lean + std::mem::replace(&mut self.before.lean[i], lean));
                    let pull = link.gravity * (Vec3::NEG_Y - carried) - link.drag * pace;
                    let held = pull * link.loose;
                    let (off, rate) = (self.at[i] - self.goal[i] - held, self.speed[i] - pace);
                    let [a, b, c, d] = link.carry;
                    let free = goal + (a * off + b * rate) + held;
                    let speed = c * off + d * rate + pace;
                    let out = free - head;
                    let far = out.length_squared();
                    let mut dir = if far > 1e-12 * length * length {
                        out / far.sqrt()
                    } else {
                        toward
                    };
                    let (cos, sin) = (link.cos, link.sin);
                    let mut edge = cone(dir, toward, cos, sin);
                    if let Some((at, _)) = edge {
                        dir = at;
                    }
                    let reach = (
                        &self.clear[i * rods..(i + 1) * rods],
                        &self.ahead[i * rods..(i + 1) * rods],
                    );
                    let (kept, struck, floored) = keep_out(head, length, dir, reach, &near, placed, link.level);
                    if struck.is_some() || floored {
                        (dir, edge) = (kept, None);
                    }
                    let was = self.at[i];
                    let before = (was - head_was) / length;
                    let across = speed - drift;
                    let across = across - across.dot(before) * before;
                    let mut speed =
                        across - (across.dot(dir) / (1.0 + before.dot(dir)).max(1e-6)) * (before + dir) + drift;
                    if let Some((_, side)) = edge {
                        let outward = cos * side - sin * toward;
                        speed -= (speed - pace).dot(outward).max(0.0) * outward;
                    }
                    if let Some(away) = struck {
                        speed -= (speed - pace).dot(away).min(0.0) * away;
                    }
                    if floored {
                        speed.y = speed.y.max(0.0);
                    }
                    let at = head + length * dir;
                    self.speed[i] = speed;
                    self.at[i] = at;
                    self.goal[i] = goal;
                    let turned = (arc(toward, dir) * rested).normalize();
                    if last {
                        let bone = self.bones[i];
                        pose[bone].rotation = (up.inverse() * turned).normalize();
                        (self.rots[bone], self.heads[bone]) = (turned, head);
                    }
                    (head, head_was, drift, up) = (at, was, speed, turned);
                }
                k += 1.0;
            }
            (self.root_at[c], self.root_turn[c]) = (start, turn);
        }
        self.still &= !moving;
        self.live = true;
    }
}

fn cone(dir: Vec3, toward: Vec3, cos: f32, sin: f32) -> Option<(Vec3, Vec3)> {
    let bent = dir.dot(toward);
    if bent >= cos {
        return None;
    }
    let side = dir - bent * toward;
    let wide = side.length_squared();
    let side = if wide > 1e-12 {
        side / wide.sqrt()
    } else {
        toward.any_orthonormal_vector()
    };
    Some((cos * toward + sin * side, side))
}

fn keep_out(
    head: Vec3,
    length: f32,
    mut dir: Vec3,
    (clear, ahead): (&[f32], &[f32]),
    near: &[usize],
    rods: &[[Vec3; 2]],
    level: f32,
) -> (Vec3, Option<Vec3>, bool) {
    let mut out = None;
    for _ in 0..8 {
        let mut any = false;
        for &k in near {
            let reach = clear[k];
            if reach <= 0.0 {
                continue;
            }
            let [a, b] = rods[k];
            let tail = head + length * dir;
            let from = nearest(a, b, tail);
            let gap = tail.distance_squared(from);
            if gap >= reach * reach {
                continue;
            }
            let reach = reach.min(head.distance(nearest(a, b, head)) + ahead[k]);
            if gap >= reach * reach {
                continue;
            }
            let to = from - head;
            let far = to.length();
            if far <= 1e-6 * length {
                continue;
            }
            let to = to / far;
            let cos = (length * length + far * far - reach * reach) / (2.0 * length * far);
            if cos >= 1.0 {
                continue;
            }
            let cos = cos.max(-1.0);
            let side = dir - dir.dot(to) * to;
            let wide = side.length_squared();
            let side = if wide > 1e-12 {
                side / wide.sqrt()
            } else {
                to.any_orthonormal_vector()
            };
            dir = cos * to + (1.0 - cos * cos).sqrt() * side;
            out = Some((head + length * dir - from).normalize_or(side));
            any = true;
        }
        if !any {
            break;
        }
    }
    let low = (level - head.y) / length;
    if dir.y >= low {
        return (dir, out, false);
    }
    if low >= 1.0 {
        return (Vec3::Y, out, true);
    }
    let flat = Vec3::new(dir.x, 0.0, dir.z);
    let wide = flat.length_squared();
    let flat = if wide > 1e-12 { flat / wide.sqrt() } else { Vec3::X };
    (low * Vec3::Y + (1.0 - low * low).sqrt() * flat, out, true)
}

fn arc(from: Vec3, to: Vec3) -> Quat {
    let (axis, w) = (from.cross(to), 1.0 + from.dot(to));
    let long = w * w + axis.length_squared();
    if long > 1e-12 {
        let by = 1.0 / long.sqrt();
        Quat::from_xyzw(axis.x * by, axis.y * by, axis.z * by, w * by)
    } else {
        let across = from.any_orthonormal_vector();
        Quat::from_xyzw(across.x, across.y, across.z, 0.0)
    }
}
