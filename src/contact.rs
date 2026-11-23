//! Contacts: feet that stay put while planted, travel for clips walked in place, and arms kept out of the body.
//!
//! Limbs are found from the rest pose alone, so no names are needed. A leg is a bone near the floor whose parent stands
//! well above it, with the bone above that: foot, knee, hip. An arm is a chain off the spine above where the legs
//! branch, down to where it splits into fingers or ends: hand, elbow, shoulder. Everything after is closed form, with
//! no solver: [`reach`] is two-bone IK, and the push-out is one step against spheres along the spine (after the ultrafast
//! online retargeting of Wang et al., 2025, which runs the same step against spheres fitted to the mesh).

use glam::{Affine3A, Quat, Vec3};

use crate::shape::Shape;
use crate::surface::{Surface, under};
use crate::{Clip, Mapping, Skeleton, Transform};

/// The fixes [`Mapping::apply_with`] makes. All are on by default.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fixes {
    /// Before playing a clip walked in place, give it the travel its planted feet imply ([`travel`]).
    pub travel: bool,
    /// While a source foot is planted, keep the target's still, reaching it with two-bone IK.
    pub feet: bool,
    /// Push the target's elbows and hands out of its body.
    pub push_out: bool,
    /// Keep the target's hands as far from its skin as the source's are from theirs, when both skins are known
    /// ([`crate::Mapping::apply_touching`]).
    pub touch: bool,
}

impl Fixes {
    pub const ALL: Self = Self {
        travel: true,
        feet: true,
        push_out: true,
        touch: true,
    };
    pub const NONE: Self = Self {
        travel: false,
        feet: false,
        push_out: false,
        touch: false,
    };
}

impl Default for Fixes {
    fn default() -> Self {
        Self::ALL
    }
}

/// Three bones of a limb: the one turning at the hip or shoulder, the one turning at the knee or elbow, and the foot or
/// hand. The upper is above the lower, and the lower above the end, with any bones between.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Limb {
    pub upper: usize,
    pub lower: usize,
    pub end: usize,
    /// The axis the limb bends about at rest, in the upper bone's frame: used when the limb is straight.
    pub bend: Vec3,
}

/// The skeleton's legs, found from its rest pose: a bone standing in the lowest 15% of the body whose parent stands
/// above that, with bones below it reaching the floor, on a chain hanging from the spine. The knee and hip are picked
/// from that chain ([`split`]), so twist bones along it don't count.
#[must_use]
pub fn legs(skeleton: &Skeleton) -> Vec<Limb> {
    legs_of(skeleton, &Shape::light(skeleton))
}

fn legs_of(skeleton: &Skeleton, s: &Shape) -> Vec<Limb> {
    trunk(skeleton, s).map_or_else(Vec::new, |t| legs_in(skeleton, s, &t))
}

fn legs_in(skeleton: &Skeleton, s: &Shape, trunk: &Trunk) -> Vec<Limb> {
    let stands = |b: usize| (s.rest[b].position().y - s.floor) / s.height;
    let parent = |b: usize| skeleton.bones()[b].parent;
    let pos = |b: usize| s.rest[b].position();
    let mut lowest: Vec<f32> = (0..skeleton.len()).map(stands).collect();
    for b in (0..skeleton.len()).rev() {
        if let Some(p) = parent(b) {
            lowest[p] = lowest[p].min(lowest[b]);
        }
    }
    (0..skeleton.len())
        .filter_map(|b| {
            let p = parent(b)?;
            if trunk.on_spine[b] || stands(b) >= 0.15 || stands(p) < 0.15 || lowest[b] > 0.05 {
                return None;
            }
            let mut chain = vec![p];
            let base = loop {
                let a = parent(chain[chain.len() - 1])?;
                if trunk.on_spine[a] {
                    break a;
                }
                chain.push(a);
            };
            chain.reverse();
            let (upper, lower) = split(&chain, pos(b), pos, |c| pos(c).distance(pos(base)))?;
            (stands(upper) > stands(lower) && stands(lower) >= 0.15).then(|| limb(s, upper, lower, b, Vec3::Z))
        })
        .collect()
}

/// Picks a limb's upper and lower bones from the chain between the spine and its end, top first, given how far each
/// bone stands from the spine. The lower is the joint standing farthest from both the spine and the end: the knee or
/// elbow, not a twist bone partway along the shin or forearm. The upper is the joint above it standing farthest from
/// both the spine and the lower: the hip or shoulder, not a collarbone by the spine or a twist bone partway along the
/// thigh or upper arm.
fn split(
    chain: &[usize],
    end: Vec3,
    pos: impl Fn(usize) -> Vec3,
    spine: impl Fn(usize) -> f32,
) -> Option<(usize, usize)> {
    let apart = |b: usize, from: Vec3| pos(b).distance(from).min(spine(b));
    let lower = (1..chain.len()).max_by(|&i, &j| apart(chain[i], end).total_cmp(&apart(chain[j], end)))?;
    let to = pos(chain[lower]);
    let upper = (0..lower).max_by(|&i, &j| apart(chain[i], to).total_cmp(&apart(chain[j], to)))?;
    Some((chain[upper], chain[lower]))
}

/// The body's trunk: how many bones each bone holds (itself included), and the spine, from the biggest root to the
/// highest bone.
struct Trunk {
    under: Vec<usize>,
    spine: Vec<usize>,
    on_spine: Vec<bool>,
}

fn trunk(skeleton: &Skeleton, s: &Shape) -> Option<Trunk> {
    let n = skeleton.len();
    let parent = |b: usize| skeleton.bones()[b].parent;
    let mut under = vec![1usize; n];
    for b in (0..n).rev() {
        if let Some(p) = parent(b) {
            under[p] += under[b];
        }
    }
    let root = (0..n).filter(|&b| parent(b).is_none()).max_by_key(|&b| under[b])?;
    let mut top_of = vec![None; n];
    top_of[root] = Some(root);
    for b in root + 1..n {
        top_of[b] = parent(b).and_then(|p| top_of[p]);
    }
    let pos = |b: usize| s.rest[b].position();
    let top = (0..n)
        .filter(|&b| top_of[b] == Some(root))
        .max_by(|&a, &b| pos(a).y.total_cmp(&pos(b).y))?;
    let mut spine = vec![top];
    while let Some(p) = parent(spine[spine.len() - 1]) {
        spine.push(p);
    }
    spine.reverse();
    let mut on_spine = vec![false; n];
    for &b in &spine {
        on_spine[b] = true;
    }
    Some(Trunk { under, spine, on_spine })
}

/// The skeleton's arms, found from its rest pose: each chain off the spine above where the legs branch, down to where
/// it splits (into fingers) or ends. The spine is the path from the biggest root to the highest bone.
#[must_use]
pub fn arms(skeleton: &Skeleton) -> Vec<Limb> {
    body(skeleton, &Shape::light(skeleton))
        .map(|b| b.arms.into_iter().map(|a| a.limb).collect())
        .unwrap_or_default()
}

/// The spine bones above the chest, where the arms branch: the neck and head, up to the highest bone. Each marks itself
/// in the result; a skeleton with no arms found has none.
pub(crate) fn above_chest(skeleton: &Skeleton, s: &Shape) -> Vec<bool> {
    let mut out = vec![false; skeleton.len()];
    if let (Some(trunk), Some(body)) = (trunk(skeleton, s), body(skeleton, s))
        && let Some(&chest) = body.spine.last()
        && let Some(i) = trunk.spine.iter().position(|&b| b == chest)
    {
        for &b in &trunk.spine[i + 1..] {
            out[b] = true;
        }
    }
    out
}

/// Turns a limb's upper and lower bones so its end reaches `goal`, or as near as the limb's length allows, bending the
/// way it already bends. The end keeps its turn in the world; bones below it follow it.
pub fn reach(skeleton: &Skeleton, pose: &mut [Transform], limb: &Limb, goal: Vec3) {
    let mut world = vec![At::IDENTITY; skeleton.len()];
    place(skeleton, pose, &paths(skeleton, [limb.end]), &mut world);
    reach_in(skeleton, pose, &world, limb, goal);
}

/// [`reach`], with the bones from the root to the limb's end already placed in `world`.
fn reach_in(skeleton: &Skeleton, pose: &mut [Transform], world: &[At], limb: &Limb, goal: Vec3) {
    let parent = |b: usize| skeleton.bones()[b].parent;
    let (up, low, end) = (world[limb.upper], world[limb.lower], world[limb.end]);
    let (a, b, c) = (up.position(), low.position(), end.position());
    let (lab, lcb) = ((b - a).length(), (c - b).length());
    if lab < 1e-6 || lcb < 1e-6 {
        return;
    }
    let lat = (goal - a)
        .length()
        .clamp((lab - lcb).abs() + 1e-4 * (lab + lcb), (lab + lcb) * (1.0 - 1e-4));

    // Bend at the knee until the end is as far from the hip as the goal is.
    let mut axis = (c - a).cross(b - a);
    if axis.length() < 1e-3 * lab * lcb {
        axis = up.rotation * limb.bend;
    }
    let axis = axis.normalize_or(Vec3::X);
    let angle = |u: Vec3, v: Vec3| u.normalize().dot(v.normalize()).clamp(-1.0, 1.0).acos();
    let now = angle(a - b, c - b);
    let want = ((lab * lab + lcb * lcb - lat * lat) / (2.0 * lab * lcb))
        .clamp(-1.0, 1.0)
        .acos();
    let bend = [want - now, now - want]
        .into_iter()
        .map(|t| Quat::from_axis_angle(axis, t))
        .min_by(|p, q| {
            let off = |r: &Quat| ((b + *r * (c - b)) - a).length() - lat;
            off(p).abs().total_cmp(&off(q).abs())
        })
        .unwrap_or(Quat::IDENTITY);
    let bent = b + bend * (c - b);
    // Then swing at the hip to point the end at the goal.
    let swing = arc(bent - a, goal - a);

    let world_rot = |bone: Option<usize>| bone.map_or(Quat::IDENTITY, |p| world[p].rotation);
    let above_upper = world_rot(parent(limb.upper));
    let above_lower = world_rot(parent(limb.lower));
    // Whatever is between the lower bone and the end turns with the lower bone.
    let above_end_new = swing * bend * world_rot(parent(limb.end));
    pose[limb.upper].rotation = (above_upper.inverse() * swing * up.rotation).normalize();
    pose[limb.lower].rotation = (above_lower.inverse() * bend * low.rotation).normalize();
    pose[limb.end].rotation = (above_end_new.inverse() * end.rotation).normalize();
}

/// For each frame, whether this bone (a foot) is planted: near the lowest it gets in the clip and barely moving.
#[must_use]
pub fn planted(skeleton: &Skeleton, clip: &Clip, bone: usize) -> Vec<bool> {
    let s = Shape::light(skeleton);
    let at = track(skeleton, clip, bone);
    contacts(&at, &clip.times, s.rest[bone].position().y, s.height)
}

/// Where a bone is in the world, frame by frame.
fn track(skeleton: &Skeleton, clip: &Clip, bone: usize) -> Vec<Vec3> {
    let path = paths(skeleton, [bone]);
    let mut world = vec![At::IDENTITY; skeleton.len()];
    clip.poses
        .iter()
        .map(|p| {
            place(skeleton, p, &path, &mut world);
            world[bone].position()
        })
        .collect()
}

fn contacts(at: &[Vec3], times: &[f32], rest_y: f32, height: f32) -> Vec<bool> {
    let n = at.len();
    let Some(ground) = at.iter().map(|p| p.y).min_by(f32::total_cmp) else {
        return Vec::new();
    };
    if ground > rest_y + 0.1 * height {
        return vec![false; n];
    }
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let mut out: Vec<bool> = (0..n)
        .map(|f| {
            let (i, j) = (f.saturating_sub(1), (f + 1).min(n - 1));
            let dt = times[j] - times[i];
            let speed = if dt > 0.0 {
                flat(at[j] - at[i]).length() / dt
            } else {
                0.0
            };
            at[f].y - ground < 0.04 * height && speed < 0.25 * height
        })
        .collect();
    // Fill one-frame gaps, then drop one-frame plants.
    for f in 1..n.saturating_sub(1) {
        if !out[f] && out[f - 1] && out[f + 1] {
            out[f] = true;
        }
    }
    for f in 0..n {
        let before = f > 0 && out[f - 1];
        let after = f + 1 < n && out[f + 1];
        if out[f] && !before && !after {
            out[f] = false;
        }
    }
    out
}

/// A clip walked in place, given the travel its feet imply: each frame the body moves as far as the lowest foot slid
/// back, so planted feet stay put. A clip that already travels, or whose feet don't slide, comes back as it was.
#[must_use]
pub fn travel(skeleton: &Skeleton, clip: &Clip) -> Clip {
    travelled(skeleton, &Shape::light(skeleton), clip).unwrap_or_else(|| clip.clone())
}

/// [`travel`], or nothing if the clip is to be left as it is.
pub(crate) fn travelled(skeleton: &Skeleton, s: &Shape, clip: &Clip) -> Option<Clip> {
    let legs = legs_of(skeleton, s);
    let n = clip.poses.len();
    if legs.is_empty() || n < 2 || clip.poses.iter().any(|p| p.len() != skeleton.len()) {
        return None;
    }
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    let path = paths(skeleton, legs.iter().map(|l| l.end));
    let mut world = vec![At::IDENTITY; skeleton.len()];
    let mut feet: Vec<Vec<Vec3>> = vec![Vec::with_capacity(n); legs.len()];
    let mut hips: Vec<Vec3> = Vec::with_capacity(n);
    for p in &clip.poses {
        place(skeleton, p, &path, &mut world);
        for (l, leg) in legs.iter().enumerate() {
            feet[l].push(world[leg.end].position());
        }
        hips.push(legs.iter().map(|l| world[l.upper].position()).sum::<Vec3>() / legs.len() as f32);
    }
    let wander = hips.iter().map(|h| flat(*h - hips[0]).length()).fold(0.0, f32::max);
    if wander > 0.2 * s.height {
        return None;
    }
    let ground: Vec<f32> = feet
        .iter()
        .map(|f| f.iter().map(|p| p.y).fold(f32::INFINITY, f32::min))
        .collect();
    let mut moved = vec![Vec3::ZERO; n];
    let mut slid = 0.0;
    let mut last = Vec3::ZERO;
    for f in 1..n {
        let lowest = (0..legs.len())
            .filter(|&l| feet[l][f].y - ground[l] < 0.04 * s.height)
            .min_by(|&l, &m| (feet[l][f].y - ground[l]).total_cmp(&(feet[m][f].y - ground[m])));
        let step = match lowest {
            Some(l) => -flat(feet[l][f] - feet[l][f - 1]),
            None => last,
        };
        slid += step.length();
        last = step;
        moved[f] = moved[f - 1] + step;
    }
    if slid < 0.1 * s.height {
        return None;
    }
    let roots: Vec<usize> = (0..skeleton.len())
        .filter(|&b| skeleton.bones()[b].parent.is_none())
        .collect();
    let mut out = clip.clone();
    for (pose, m) in out.poses.iter_mut().zip(&moved) {
        for &r in &roots {
            pose[r].translation += *m;
        }
    }
    Some(out)
}

/// Keeps the target's feet still while the source's are planted.
pub(crate) fn plant(map: &Mapping, rigs: [(&Skeleton, &Shape); 2], from: &Clip, out: &mut Clip) {
    let [(source, s), (target, t)] = rigs;
    let n = out.poses.len();
    if n == 0 {
        return;
    }
    let mut world = vec![At::IDENTITY; target.len()];
    let k = t.height / s.height;
    let mut gaps: Vec<f32> = from
        .times
        .windows(2)
        .map(|w| w[1] - w[0])
        .filter(|d| *d > 0.0)
        .collect();
    gaps.sort_by(f32::total_cmp);
    let ramp = gaps
        .get(gaps.len() / 2)
        .map_or(4, |dt| ((0.12 / dt).round() as usize).clamp(1, 10));
    for leg in legs_of(target, t) {
        let Some(a) = map.source_of(leg.end) else { continue };
        let s_at = track(source, from, a);
        let down = contacts(&s_at, &from.times, s.rest[a].position().y, s.height);
        let mut runs = Vec::new();
        let mut f = 0;
        while f < n {
            if down[f] {
                let start = f;
                while f < n && down[f] {
                    f += 1;
                }
                runs.push((start, f));
            } else {
                f += 1;
            }
        }
        if runs.is_empty() {
            continue;
        }
        let t_at = track(target, out, leg.end);
        let path = paths(target, [leg.end]);
        let locks: Vec<Vec3> = runs
            .iter()
            .map(|&(a, b)| t_at[a..b].iter().sum::<Vec3>() / (b - a) as f32)
            .collect();
        let rest_y = t.rest[leg.end].position().y;
        let s_rest_y = s.rest[a].position().y;
        for f in 0..n {
            let (d, r) = runs
                .iter()
                .enumerate()
                .map(|(r, &(a, b))| (if f < a { a - f } else { (f + 1).saturating_sub(b) }, r))
                .min()
                .unwrap_or((usize::MAX, 0));
            if d > ramp {
                continue;
            }
            let x = 1.0 - d as f32 / (ramp + 1) as f32;
            let w = x * x * (3.0 - 2.0 * x);
            let y = rest_y + (s_at[f].y - s_rest_y) * k;
            let goal = t_at[f].lerp(Vec3::new(locks[r].x, y, locks[r].z), w);
            let pose = &mut out.poses[f];
            place(target, pose, &path, &mut world);
            reach_in(target, pose, &world, &leg, goal);
        }
    }
}

struct Arm {
    limb: Limb,
    /// Away from the body at rest, for a point right on the spine.
    out: Vec3,
}

struct Body {
    arms: Vec<Arm>,
    /// Spine bones from the hips to the chest; spheres sit on them and halfway between.
    spine: Vec<usize>,
    torso: f32,
    limb: f32,
}

fn body(skeleton: &Skeleton, s: &Shape) -> Option<Body> {
    let n = skeleton.len();
    let parent = |b: usize| skeleton.bones()[b].parent;
    let pos = |b: usize| s.rest[b].position();
    let trunk = trunk(skeleton, s)?;
    let (under, spine, on_spine) = (&trunk.under, &trunk.spine, &trunk.on_spine);
    let mut legged = vec![false; n];
    for l in legs_in(skeleton, s, &trunk) {
        legged[l.end] = true;
    }
    for b in (0..n).rev() {
        if let Some(p) = parent(b) {
            legged[p] |= legged[b];
        }
    }
    let hips = spine
        .iter()
        .rposition(|&b| s.children[b].iter().any(|&c| !on_spine[c] && legged[c]))
        .unwrap_or(0);
    let mut found = Vec::new();
    for (i, &sb) in spine.iter().enumerate().skip(hips + 1) {
        for &c in &s.children[sb] {
            if on_spine[c] || legged[c] {
                continue;
            }
            // Down the chain to the hand: the first bone no one child of which holds most of what's below it, as
            // fingers split. Helpers hanging off the way (twist bones, sleeves) hold less than the rest of the arm.
            // A split whose every branch starts well away from it is an elbow, its branches starting at the wrist:
            // the biggest is the hand.
            let mut chain = Vec::new();
            let mut end = c;
            while let Some(&next) = s.children[end].iter().max_by_key(|&&k| under[k]) {
                if 2 * under[next] < under[end] {
                    let far = s.children[end]
                        .iter()
                        .all(|&k| pos(k).distance(pos(end)) > 0.1 * s.height);
                    if far {
                        chain.push(end);
                        end = next;
                    }
                    break;
                }
                chain.push(end);
                end = next;
            }
            // Arms are measured out from the spine across, as a chest bone can sit well below the shoulders.
            let across = |b: usize| {
                let d = pos(b) - pos(sb);
                Vec3::new(d.x, 0.0, d.z).length()
            };
            let Some((upper, lower)) = split(&chain, pos(end), pos, across) else {
                continue;
            };
            if pos(upper).distance(pos(end)) < 0.15 * s.height {
                continue;
            }
            let side = pos(end) - pos(sb);
            let arm = Arm {
                limb: limb(s, upper, lower, end, -Vec3::Z),
                out: Vec3::new(side.x, 0.0, side.z).normalize_or(Vec3::X),
            };
            found.push((arm, sb, i));
        }
    }
    // The arms are the chains off one spine bone: the one whose chains end in the most bones, as hands hold fingers,
    // and the lowest of those. Hair off the head and tails of a coat or cape end in nothing.
    let (_, chest) = found
        .iter()
        .map(|(a, _, i)| (under[a.limb.end], std::cmp::Reverse(*i)))
        .max()
        .map(|(u, std::cmp::Reverse(i))| (u, i))?;
    let arms: Vec<(Arm, usize)> = found
        .into_iter()
        .filter(|(_, _, i)| *i == chest)
        .map(|(a, sb, _)| (a, sb))
        .collect();
    let spine: Vec<usize> = spine[hips..=chest].to_vec();
    let torso0 = 0.6
        * arms
            .iter()
            .map(|(a, sb)| pos(a.limb.upper).distance(pos(*sb)))
            .fold(f32::INFINITY, f32::min);
    let limb_r = 0.3 * torso0;
    let centres = spheres(&spine, pos);
    let clear = arms
        .iter()
        .flat_map(|(a, _)| {
            let (b, c) = (pos(a.limb.lower), pos(a.limb.end));
            [b, c, (b + c) / 2.0]
        })
        .flat_map(|p| centres.iter().map(move |q| p.distance(*q)))
        .fold(f32::INFINITY, f32::min);
    let torso = torso0.min(clear - limb_r);
    (torso > 0.2 * torso0).then(|| Body {
        arms: arms.into_iter().map(|(a, _)| a).collect(),
        spine,
        torso,
        limb: limb_r,
    })
}

fn spheres(spine: &[usize], pos: impl Fn(usize) -> Vec3) -> Vec<Vec3> {
    let mut v: Vec<Vec3> = spine.iter().map(|&b| pos(b)).collect();
    for w in spine.windows(2) {
        v.push((pos(w[0]) + pos(w[1])) / 2.0);
    }
    v
}

/// Pushes each pose's elbows and hands out of the spheres along its spine: the upper arm turns at the shoulder to
/// clear the elbow, then the forearm at the elbow to clear the hand.
pub(crate) fn push_out(target: &Skeleton, t: &Shape, clip: &mut Clip) {
    let Some(body) = body(target, t) else {
        return;
    };
    let parent = |b: usize| target.bones()[b].parent;
    let reach = body.torso + body.limb;
    let path = paths(
        target,
        body.spine.iter().copied().chain(body.arms.iter().map(|a| a.limb.end)),
    );
    let mut world = vec![At::IDENTITY; target.len()];
    let mut centres = Vec::with_capacity(2 * body.spine.len());
    for pose in &mut clip.poses {
        // Arms don't move the spine or each other, so the world is worked out once a pose.
        place(target, pose, &path, &mut world);
        centres.clear();
        centres.extend(body.spine.iter().map(|&b| world[b].position()));
        centres.extend(
            body.spine
                .windows(2)
                .map(|w| (world[w[0]].position() + world[w[1]].position()) / 2.0),
        );
        let need = |p: Vec3, out: Vec3| {
            centres
                .iter()
                .filter_map(|q| {
                    let d = p - *q;
                    let len = d.length();
                    (len < reach).then(|| d.normalize_or(out) * (reach - len))
                })
                .max_by(|u, v| u.length_squared().total_cmp(&v.length_squared()))
        };
        let above = |bone: usize| parent(bone).map_or(Quat::IDENTITY, |p| world[p].rotation);
        for arm in &body.arms {
            let l = &arm.limb;
            let (up, low) = (&world[l.upper], &world[l.lower]);
            let (a, b, c) = (up.position(), low.position(), world[l.end].position());
            let turn = need(b, arm.out).map_or(Quat::IDENTITY, |push| arc(b - a, b + push - a));
            if turn != Quat::IDENTITY {
                pose[l.upper].rotation = (above(l.upper).inverse() * turn * up.rotation).normalize();
            }
            let (b2, c2) = (a + turn * (b - a), a + turn * (c - a));
            let mid = need((b2 + c2) / 2.0, arm.out).map(|p| p * 2.0);
            let hand = need(c2, arm.out);
            let push = match (mid, hand) {
                (Some(m), Some(h)) => Some(if m.length_squared() > h.length_squared() { m } else { h }),
                (m, h) => m.or(h),
            };
            if let Some(push) = push {
                let bend = arc(c2 - b2, c2 + push - b2);
                // The lower bone's parent is now turned by `turn`, and the lower bone by `bend` after it.
                let above_new = turn * above(l.lower);
                pose[l.lower].rotation = (above_new.inverse() * bend * turn * low.rotation).normalize();
            }
        }
    }
}

fn limb(s: &Shape, upper: usize, lower: usize, end: usize, knee: Vec3) -> Limb {
    let (a, b, c) = (
        s.rest[upper].position(),
        s.rest[lower].position(),
        s.rest[end].position(),
    );
    let mut axis = (c - a).cross(b - a);
    if axis.length() < 1e-3 * (b - a).length() * (c - b).length() {
        // Straight at rest: knees bend forward, elbows back, for a body facing +z.
        axis = (c - a).cross(knee);
    }
    Limb {
        upper,
        lower,
        end,
        bend: s.rest[upper].rotation.inverse() * axis.normalize_or(Vec3::X),
    }
}

/// The shortest turn from `u`'s direction to `v`'s.
fn arc(u: Vec3, v: Vec3) -> Quat {
    match (u.try_normalize(), v.try_normalize()) {
        (Some(u), Some(v)) => Quat::from_rotation_arc(u, v),
        _ => Quat::IDENTITY,
    }
}

/// A bone placed in the world: its whole transform, and its turn apart from scale.
#[derive(Clone, Copy, Debug)]
struct At {
    matrix: Affine3A,
    rotation: Quat,
}

impl At {
    const IDENTITY: Self = Self {
        matrix: Affine3A::IDENTITY,
        rotation: Quat::IDENTITY,
    };

    fn position(&self) -> Vec3 {
        self.matrix.translation.into()
    }
}

/// The bones on the paths from the roots down to these, parents first.
fn paths(skeleton: &Skeleton, ends: impl IntoIterator<Item = usize>) -> Vec<usize> {
    let mut on = vec![false; skeleton.len()];
    for end in ends {
        let mut b = Some(end);
        while let Some(x) = b.filter(|&x| !on[x]) {
            on[x] = true;
            b = skeleton.bones()[x].parent;
        }
    }
    (0..skeleton.len()).filter(|&b| on[b]).collect()
}

/// Places these bones (parents first, each one's parent among them unless it's a root) in the world under a pose.
fn place(skeleton: &Skeleton, pose: &[Transform], bones: &[usize], world: &mut [At]) {
    for &b in bones {
        let l = &pose[b];
        let local = Affine3A::from_scale_rotation_translation(l.scale, l.rotation, l.translation);
        world[b] = match skeleton.bones()[b].parent {
            Some(p) => At {
                matrix: world[p].matrix * local,
                rotation: (world[p].rotation * l.rotation).normalize(),
            },
            None => At {
                matrix: local,
                rotation: l.rotation.normalize(),
            },
        };
    }
}

/// Keeps each target hand as far from the target's skin as the source's hand is from the source's, scaled by height,
/// while the source's is near its body: a hand resting on a hip rests on the target's hip, over a coat if it wears one.
/// Far from the body the hand is left where it is, except that it is kept out of the skin; in between, the two blend.
/// Reached with two-bone IK, so the hand keeps its turn.
pub(crate) fn touch(map: &Mapping, rigs: [(&Skeleton, &Shape, &Surface); 2], from: &Clip, out: &mut Clip) {
    let [(source, s, s_skin), (target, t, t_skin)] = rigs;
    if s_skin.is_empty() || t_skin.is_empty() {
        return;
    }
    let (Some(s_body), Some(t_body)) = (body(source, s), body(target, t)) else {
        return;
    };
    // Each arm the two share: the target's limb, the source's hand, each hand's middle in its own frame, and the skin
    // points of each hand and its fingers.
    struct Arm {
        limb: Limb,
        source: usize,
        middle: [Vec3; 2],
        hand: [Vec<usize>; 2],
    }
    let arms: Vec<Arm> = t_body
        .arms
        .iter()
        .filter_map(|ta| {
            let a = map.source_of(ta.limb.end)?;
            s_body.arms.iter().find(|sa| sa.limb.end == a)?;
            let (cs, ct) = (s_skin.centre(a)?, t_skin.centre(ta.limb.end)?);
            let (hs, ht) = (
                s_skin.on(&under(source, &[a])),
                t_skin.on(&under(target, &[ta.limb.end])),
            );
            Some(Arm {
                limb: ta.limb,
                source: a,
                middle: [cs, ct],
                hand: [hs, ht],
            })
        })
        .collect();
    if arms.is_empty() {
        return;
    }
    // The skin a hand can touch: all but the arms, from the shoulder down.
    let off_arms = |skeleton: &Skeleton, b: &Body| {
        let off = under(skeleton, &b.arms.iter().map(|a| a.limb.upper).collect::<Vec<_>>());
        off.into_iter().map(|o| !o).collect::<Vec<_>>()
    };
    let (s_keep, t_keep) = (off_arms(source, &s_body), off_arms(target, &t_body));
    let scale = t.height / s.height;
    // Within `near` of the skin (over the height) a source hand is touching it; past `far` it is free.
    let (near, far) = (0.015, 0.05);
    let (mut s_pts, mut t_pts) = (Vec::new(), Vec::new());
    for (pose, played) in from.poses.iter().zip(&mut out.poses) {
        let sw = source.world(pose);
        s_skin.posed(&sw, |b| s_keep[b], &mut s_pts);
        let mut tw = target.world(played);
        t_skin.posed(&tw, |b| t_keep[b], &mut t_pts);
        for Arm {
            limb,
            source: a,
            middle: [cs, ct],
            hand: [s_hand, t_hand],
        } in &arms
        {
            let c = sw[*a].matrix.transform_point3(*cs);
            let Some((g, _)) = s_skin.clearance(&s_pts, &sw, c, s_hand) else {
                continue;
            };
            let g = g.max(0.0);
            let x = ((g / s.height - near) / (far - near)).clamp(0.0, 1.0);
            let w = 1.0 - x * x * (3.0 - 2.0 * x);
            let c = tw[limb.end].matrix.transform_point3(*ct);
            let Some((now, facing)) = t_skin.clearance(&t_pts, &tw, c, t_hand) else {
                continue;
            };
            let mut want = now + w * (g * scale - now);
            // Out of the skin, unless so deep the nearest skin is likely the far side.
            if now > -far * t.height {
                want = want.max(0.0);
            }
            if (want - now).abs() > 1e-4 * t.height {
                let goal = tw[limb.end].position() + facing * (want - now);
                reach(target, played, limb, goal);
                tw = target.world(played);
            }
        }
    }
}
