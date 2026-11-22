//! Playing a clip on the target through a mapping.
//!
//! A mapped bone takes its source bone's turn away from rest in the world, carried into the target's frame and
//! corrected for the two rest poses: at rest the source bone and its next mapped bones point one way, the target's
//! another, and the turn between those (`fix`) is undone first. So the target's bones point where the source's do, frame
//! by frame, whatever each rig's rest pose and bone axes. A bone between two mapped bones takes its share of their turns
//! by how far along it is; any other keeps its rest pose under its parent. The topmost mapped bones and any whose source
//! moves (not just turns) travel as their source does, scaled by how high each stands.

use glam::{Mat3, Mat4, Quat, Vec3};

use crate::shape::Shape;
use crate::{Clip, Mapping, Skeleton, Transform};

#[derive(Clone, Copy)]
enum Plan {
    Rest,
    Mapped {
        source: usize,
        unfix: Quat,
        travel: Option<f32>,
    },
    Between {
        above: usize,
        below: usize,
        along: f32,
    },
}

pub(crate) fn play(map: &Mapping, source: &Skeleton, target: &Skeleton, clip: &Clip) -> Clip {
    let s = Shape::of(source);
    let t = Shape::of(target);
    let turn = map.turn();
    let plans = plan(map, &s, &t, source, clip);
    let rest_s: Vec<Quat> = s.rest.iter().map(|p| p.rotation.inverse()).collect();
    let back = turn.inverse();
    let bones = target.bones();
    let mut poses = Vec::with_capacity(clip.poses.len());
    for pose in &clip.poses {
        let sw = source.world(pose);
        let turned: Vec<Quat> = plans
            .iter()
            .map(|p| match *p {
                Plan::Mapped { source: a, unfix, .. } => (turn * sw[a].rotation * rest_s[a] * back * unfix).normalize(),
                _ => Quat::IDENTITY,
            })
            .collect();
        let mut out: Vec<Transform> = Vec::with_capacity(bones.len());
        let mut world: Vec<(Mat4, Quat)> = Vec::with_capacity(bones.len());
        for (i, b) in bones.iter().enumerate() {
            let (up_m, up_r) = b.parent.map_or((Mat4::IDENTITY, Quat::IDENTITY), |p| world[p]);
            let rest = b.rest;
            let (rotation, translation) = match plans[i] {
                Plan::Rest => (rest.rotation, rest.translation),
                Plan::Mapped { source: a, travel, .. } => {
                    let r = (up_r.inverse() * turned[i] * t.rest[i].rotation).normalize();
                    let at = travel.map_or(rest.translation, |k| {
                        let moved = turn * (sw[a].position() - s.rest[a].position()) * k;
                        up_m.inverse().transform_point3(t.rest[i].position() + moved)
                    });
                    (r, at)
                }
                Plan::Between { above, below, along } => {
                    let w = turned[above].slerp(turned[below], along) * t.rest[i].rotation;
                    ((up_r.inverse() * w).normalize(), rest.translation)
                }
            };
            let local = Transform {
                translation,
                rotation,
                scale: rest.scale,
            };
            world.push((up_m * local.matrix(), (up_r * rotation).normalize()));
            out.push(local);
        }
        poses.push(out);
    }
    Clip {
        times: clip.times.clone(),
        poses,
    }
}

fn plan(map: &Mapping, s: &Shape, t: &Shape, source: &Skeleton, clip: &Clip) -> Vec<Plan> {
    let n = t.children.len();
    let turn = map.turn();
    let mapped = |b: usize| map.source_of(b);
    // The first mapped bones down each path from every bone, and how many bones are under each.
    let mut under = vec![1usize; n];
    for i in (0..n).rev() {
        for &c in &t.children[i] {
            under[i] += under[c];
        }
    }
    let mut next: Vec<Vec<usize>> = vec![Vec::new(); n];
    for i in (0..n).rev() {
        let mut v = Vec::new();
        for &c in &t.children[i] {
            if mapped(c).is_some() {
                v.push(c);
            } else {
                v.extend(next[c].iter().copied());
            }
        }
        v.sort_by_key(|&c| (std::cmp::Reverse(under[c]), c));
        next[i] = v;
    }
    let parent: Vec<Option<usize>> = {
        let mut p = vec![None; n];
        for (i, kids) in t.children.iter().enumerate() {
            for &c in kids {
                p[c] = Some(i);
            }
        }
        p
    };
    let above = |mut b: usize| -> Option<usize> {
        while let Some(p) = parent[b] {
            if mapped(p).is_some() {
                return Some(p);
            }
            b = p;
        }
        None
    };

    // Which source bones move, not just turn.
    let moves: Vec<bool> = (0..source.len())
        .map(|a| {
            let r = source.bones()[a].rest.translation;
            clip.poses
                .iter()
                .any(|p| p[a].translation.distance(r) > 1e-4 * s.height)
        })
        .collect();
    let stands = |shape: &Shape, b: usize| shape.rest[b].position().y - shape.floor;
    let fallback = {
        let (hs, ht) = (stands(s, s.middle), stands(t, t.middle));
        if hs > 0.1 * s.height && ht > 0.1 * t.height {
            ht / hs
        } else {
            t.height / s.height
        }
    };

    let mut fixes: Vec<Option<Quat>> = vec![None; n];
    let mut plans = vec![Plan::Rest; n];
    for b in 0..n {
        let Some(a) = mapped(b) else { continue };
        let pairs: Vec<(Vec3, Vec3)> = next[b]
            .iter()
            .filter_map(|&b2| {
                let a2 = mapped(b2)?;
                let u = turn * (s.rest[a2].position() - s.rest[a].position());
                let v = t.rest[b2].position() - t.rest[b].position();
                (u.length() > 1e-9 && v.length() > 1e-9).then_some((u, v))
            })
            .collect();
        let fix = match pairs.as_slice() {
            [] => above(b).and_then(|p| fixes[p]).unwrap_or(Quat::IDENTITY),
            [(u, v)] => Quat::from_rotation_arc(u.normalize(), v.normalize()),
            [(u1, v1), rest @ ..] => {
                let second = rest.iter().find(|(u2, v2)| {
                    u1.normalize().cross(u2.normalize()).length() > 0.1
                        && v1.normalize().cross(v2.normalize()).length() > 0.1
                });
                match second {
                    Some((u2, v2)) => align(*u1, *u2, *v1, *v2),
                    None => Quat::from_rotation_arc(u1.normalize(), v1.normalize()),
                }
            }
        };
        fixes[b] = Some(fix);
        let top = above(b).is_none();
        let travel = (top || moves[a]).then(|| {
            let (hs, ht) = (stands(s, a), stands(t, b));
            if hs > 0.1 * s.height && ht > 0.1 * t.height {
                ht / hs
            } else {
                fallback
            }
        });
        plans[b] = Plan::Mapped {
            source: a,
            unfix: fix.inverse(),
            travel,
        };
    }
    for b in 0..n {
        if mapped(b).is_some() {
            continue;
        }
        let (Some(top), Some(&low)) = (above(b), next[b].first()) else {
            continue;
        };
        // How far along from the mapped bone above to the mapped one below this one is: the path from the one below
        // up to the one above passes through it.
        let mut path = vec![low];
        while let Some(p) = parent[path[path.len() - 1]] {
            path.push(p);
            if p == top {
                break;
            }
        }
        let (mut total, mut to_here) = (0.0f32, 0.0f32);
        for w in path.windows(2).rev() {
            if w[1] == b {
                to_here = total;
            }
            total += t.rest[w[0]].position().distance(t.rest[w[1]].position());
        }
        let along = if total > 0.0 {
            (to_here / total).clamp(0.0, 1.0)
        } else {
            0.5
        };
        plans[b] = Plan::Between {
            above: top,
            below: low,
            along,
        };
    }
    plans
}

/// The rotation taking `u1` to `v1`'s direction, and the plane of `u1` and `u2` to that of `v1` and `v2`.
fn align(u1: Vec3, u2: Vec3, v1: Vec3, v2: Vec3) -> Quat {
    let frame = |a: Vec3, b: Vec3| {
        let e1 = a.normalize();
        let e2 = (b - b.dot(e1) * e1).normalize();
        Mat3::from_cols(e1, e2, e1.cross(e2))
    };
    Quat::from_mat3(&(frame(v1, v2) * frame(u1, u2).transpose())).normalize()
}
