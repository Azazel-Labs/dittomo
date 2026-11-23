//! Contacts: feet that stay put while planted, travel for clips walked in place, arms kept out of the body, and hands
//! that meet each other or the head where the source's do.
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
    /// Bring the target's hands together where the source's meet (a clap, a clasp, hands pressed in prayer): at a
    /// point both arms reach, outside the body and the head, however long or short the arms.
    pub hands: bool,
    /// Keep the target's hands on its head where the source's are on theirs (covering the ears, clasped on top, a palm
    /// to the forehead): at the same place on the head, scaled to the head's size, turning with it.
    pub head: bool,
}

impl Fixes {
    pub const ALL: Self = Self {
        travel: true,
        feet: true,
        push_out: true,
        touch: true,
        hands: true,
        head: true,
    };
    pub const NONE: Self = Self {
        travel: false,
        feet: false,
        push_out: false,
        touch: false,
        hands: false,
        head: false,
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
/// bone stands from the spine: the pair whose three spans (spine to upper, upper to lower, lower to end) are all long,
/// the shortest of them as long as can be, then the next shortest. That is the hip or shoulder and the knee or elbow:
/// not a collarbone by the spine, nor a twist bone partway along a thigh, shin, upper arm or forearm, as each leaves a
/// span short. A shoulder standing out from the spine about as far as a short forearm is long still loses to the elbow.
fn split(
    chain: &[usize],
    end: Vec3,
    pos: impl Fn(usize) -> Vec3,
    spine: impl Fn(usize) -> f32,
) -> Option<(usize, usize)> {
    let spans = |(u, l): (usize, usize)| {
        let (a, b) = (chain[u], chain[l]);
        let mut s = [spine(a), pos(a).distance(pos(b)), pos(b).distance(end)];
        s.sort_by(f32::total_cmp);
        s
    };
    (1..chain.len())
        .flat_map(|l| (0..l).map(move |u| (u, l)))
        .max_by(|&p, &q| spans(p).partial_cmp(&spans(q)).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(u, l)| (chain[u], chain[l]))
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

/// What arms keep out of and reach for: the arms, the spine and the head, found from the rest pose.
pub(crate) struct Body {
    arms: Vec<Arm>,
    /// Spine bones from the hips to the chest; spheres sit on them and halfway between.
    spine: Vec<usize>,
    torso: f32,
    limb: f32,
    head: Option<Head>,
}

/// A sphere standing for the head, carried by a bone.
#[derive(Clone, Copy)]
struct Head {
    bone: usize,
    /// The centre in the bone's frame.
    centre: Vec3,
    radius: f32,
}

impl Head {
    fn centre(&self, world: &[At]) -> Vec3 {
        world[self.bone].matrix.transform_point3(self.centre)
    }
}

impl Body {
    /// Fits the head to the skin bound to the head bone, where there's enough of it: centred on it, as far round as
    /// the skin is from there, half of it nearer and half farther.
    pub(crate) fn fit_head(&mut self, s: &Shape, skin: &Surface) {
        let Some(head) = self.head.as_mut() else {
            return;
        };
        let rest = s.rest[head.bone].matrix;
        let points: Vec<Vec3> = skin
            .bone
            .iter()
            .zip(&skin.at)
            .filter(|&(&b, _)| b == head.bone)
            .map(|(_, &at)| rest.transform_point3(at))
            .collect();
        if points.len() < 16 {
            return;
        }
        let centre = points.iter().sum::<Vec3>() / points.len() as f32;
        let mut far: Vec<f32> = points.iter().map(|p| p.distance(centre)).collect();
        let half = far.len() / 2;
        let (_, &mut radius, _) = far.select_nth_unstable_by(half, f32::total_cmp);
        if radius > 0.02 * s.height && radius < 0.25 * s.height {
            head.centre = rest.inverse().transform_point3(centre);
            head.radius = radius;
        }
    }

    /// The bones to place for the spheres and the arms.
    fn path(&self, skeleton: &Skeleton) -> Vec<usize> {
        let ends = self.spine.iter().copied().chain(self.arms.iter().map(|a| a.limb.end));
        paths(skeleton, ends.chain(self.head.map(|h| h.bone)))
    }

    /// The spheres an arm keeps out of, posed: on the spine bones and halfway between, and the head. Each radius is
    /// the sphere's own and the arm's together, so the arm's line keeps out of them.
    fn spheres(&self, world: &[At], out: &mut Vec<(Vec3, f32)>) {
        out.clear();
        let reach = self.torso + self.limb;
        out.extend(self.spine.iter().map(|&b| (world[b].position(), reach)));
        out.extend(
            self.spine
                .windows(2)
                .map(|w| ((world[w[0]].position() + world[w[1]].position()) / 2.0, reach)),
        );
        out.extend(self.head.map(|h| (h.centre(world), h.radius + self.limb)));
    }
}

/// The head from the spine bones above the chest, neck first. A rig with a bone at the top of the head has the head
/// below it, as big as the head bone is long (a cartoon's big head too); one stopping at the base of the head, above,
/// and a twentieth of the height across.
fn head(above: &[usize], s: &Shape) -> Option<Head> {
    let pos = |b: usize| s.rest[b].position();
    let (&first, &top) = (above.first()?, above.last()?);
    let (base, top_at) = (pos(first), pos(top));
    let up = (top_at - base).normalize_or(Vec3::Y);
    let (centre, radius) = if (top_at - base).length() > 0.12 * s.height {
        let skull = above.len().checked_sub(2).map_or(base, |i| pos(above[i]));
        let radius = (0.45 * skull.distance(top_at)).clamp(0.035 * s.height, 0.15 * s.height);
        (top_at - up * radius, radius)
    } else {
        let radius = 0.05 * s.height;
        (top_at + up * 0.8 * radius, radius)
    };
    let along = |p: Vec3| (p - base).dot(up);
    let bone = above
        .iter()
        .rev()
        .copied()
        .find(|&b| along(pos(b)) <= along(centre))
        .unwrap_or(first);
    Some(Head {
        bone,
        centre: s.rest[bone].matrix.inverse().transform_point3(centre),
        radius,
    })
}

pub(crate) fn body(skeleton: &Skeleton, s: &Shape) -> Option<Body> {
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
                if 2 * under[next] <= under[end] {
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
    let head = head(&spine[chest + 1..], s);
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
        head,
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
pub(crate) fn push_out(target: &Skeleton, body: &Body, clip: &mut Clip) {
    let parent = |b: usize| target.bones()[b].parent;
    let path = body.path(target);
    let mut world = vec![At::IDENTITY; target.len()];
    let mut centres = Vec::with_capacity(2 * body.spine.len() + 1);
    for pose in &mut clip.poses {
        // Arms don't move the spine or each other, so the world is worked out once a pose.
        place(target, pose, &path, &mut world);
        body.spheres(&world, &mut centres);
        let need = |p: Vec3, out: Vec3| {
            centres
                .iter()
                .filter_map(|&(q, reach)| {
                    let d = p - q;
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
pub(crate) fn touch(
    map: &Mapping,
    rigs: [(&Skeleton, &Shape, &Surface); 2],
    [s_body, t_body]: [&Body; 2],
    from: &Clip,
    out: &mut Clip,
) {
    let [(source, s, s_skin), (target, t, t_skin)] = rigs;
    if s_skin.is_empty() || t_skin.is_empty() {
        return;
    }
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
    let (s_keep, t_keep) = (off_arms(source, s_body), off_arms(target, t_body));
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

/// Brings each two target hands together where the source's meet: a clap, a clasp, hands pressed in prayer. Near, the
/// way from one hand to the other is the source's, turned into the target's frame and scaled by height; farther off it
/// blends back to what the target already does, and past a sixth of the height the hands are left alone. The hands
/// meet at the point nearest where they were that both arms reach and that keeps both out of the body and the head,
/// so arms too short to clap overhead clap in front of the face, and long arms clap rather than cross. Reached with
/// two-bone IK, so each hand keeps its turn.
pub(crate) fn hands(
    map: &Mapping,
    rigs: [(&Skeleton, &Shape); 2],
    [s_body, t_body]: [&Body; 2],
    from: &Clip,
    out: &mut Clip,
) {
    let [(source, s), (target, t)] = rigs;
    let arms = played(map, s_body, t_body);
    let pairs: Vec<(usize, usize)> = (0..arms.len())
        .flat_map(|i| (i + 1..arms.len()).map(move |j| (i, j)))
        .collect();
    if pairs.is_empty() {
        return;
    }
    let (turn, scale) = (map.turn(), t.height / s.height);
    // Within `near` of each other (over the height) the source's hands are together; past `far`, apart.
    let (near, far) = (0.06, 0.15);
    let s_path = paths(source, arms.iter().map(|a| a.1));
    let t_path = t_body.path(target);
    let (mut sw, mut tw) = (vec![At::IDENTITY; source.len()], vec![At::IDENTITY; target.len()]);
    let mut spheres = Vec::new();
    let mut last: Vec<Option<Vec3>> = vec![None; pairs.len()];
    for (pose, played) in from.poses.iter().zip(&mut out.poses) {
        place(source, pose, &s_path, &mut sw);
        for (k, &(i, j)) in pairs.iter().enumerate() {
            let ((li, si), (lj, sj)) = (arms[i], arms[j]);
            let apart = sw[sj].position() - sw[si].position();
            let x = ((apart.length() / s.height - near) / (far - near)).clamp(0.0, 1.0);
            let w = 1.0 - x * x * (3.0 - 2.0 * x);
            if w <= 0.0 {
                last[k] = None;
                continue;
            }
            place(target, played, &t_path, &mut tw);
            t_body.spheres(&tw, &mut spheres);
            let (hi, hj) = (tw[li.end].position(), tw[lj.end].position());
            let way = (hj - hi).lerp(turn * apart * scale, w);
            let shoulder = |l: &Limb| {
                let (a, b, c) = (tw[l.upper].position(), tw[l.lower].position(), tw[l.end].position());
                (a, 0.999 * (a.distance(b) + b.distance(c)))
            };
            let m = meet(
                (hi + hj) / 2.0,
                last[k],
                [-way / 2.0, way / 2.0],
                [shoulder(&li), shoulder(&lj)],
                &spheres,
            );
            last[k] = Some(m);
            // Arms hang apart, so reaching one leaves the other placed as it was.
            reach_in(target, played, &tw, &li, m - way / 2.0);
            reach_in(target, played, &tw, &lj, m + way / 2.0);
        }
    }
}

/// The target's arms whose hands play the source's hands, with those.
fn played(map: &Mapping, s_body: &Body, t_body: &Body) -> Vec<(Limb, usize)> {
    t_body
        .arms
        .iter()
        .filter_map(|ta| {
            let a = map.source_of(ta.limb.end)?;
            s_body.arms.iter().any(|sa| sa.limb.end == a).then_some((ta.limb, a))
        })
        .collect()
}

/// Keeps each target hand on its head where the source's hand is on theirs: covering an ear, clasped on top, a palm to
/// the forehead, scratching the back of the head. The way from the middle of the source's head to its wrist is taken
/// out of the head's turn from rest, turned into the target's frame, scaled from one head to the other and turned with
/// the target's head, so the hand lands on the same place on a head of any size, even one that sits differently on its
/// neck. Within one and a half head radii of the middle of the source's head the hand is put there; past two it is left
/// alone; in between, the two blend. A hand that can't get there (short arms, a big head) goes as near as it can over
/// the head's surface, not into it, and the elbow swings round the line from shoulder to hand till the arm is no deeper
/// in the head than the source's. Reached with two-bone IK, so the hand keeps its turn.
pub(crate) fn on_head(
    map: &Mapping,
    rigs: [(&Skeleton, &Shape); 2],
    [s_body, t_body]: [&Body; 2],
    from: &Clip,
    out: &mut Clip,
) {
    let [(source, s), (target, t)] = rigs;
    let (Some(sh), Some(th)) = (s_body.head, t_body.head) else {
        return;
    };
    let arms = played(map, s_body, t_body);
    if arms.is_empty() {
        return;
    }
    let (near, far) = (1.5, 2.0);
    let (turn, scale) = (map.turn(), th.radius / sh.radius);
    let (s_rest, t_rest) = (s.rest[sh.bone].rotation.inverse(), t.rest[th.bone].rotation.inverse());
    let s_path = paths(source, arms.iter().map(|a| a.1).chain([sh.bone]));
    let t_path = paths(target, arms.iter().map(|a| a.0.end).chain([th.bone]));
    let (mut sw, mut tw) = (vec![At::IDENTITY; source.len()], vec![At::IDENTITY; target.len()]);
    for (pose, played) in from.poses.iter().zip(&mut out.poses) {
        place(source, pose, &s_path, &mut sw);
        let (sc, s_turn) = (sh.centre(&sw), sw[sh.bone].rotation * s_rest);
        let mut placed = false;
        for &(limb, a) in &arms {
            let d = sw[a].position() - sc;
            let x = ((d.length() / sh.radius - near) / (far - near)).clamp(0.0, 1.0);
            let w = 1.0 - x * x * (3.0 - 2.0 * x);
            if w <= 0.0 {
                continue;
            }
            // Arms hang apart, so reaching one leaves the other, and the head, placed as they were.
            if !placed {
                place(target, played, &t_path, &mut tw);
                placed = true;
            }
            let (t_turn, centre) = (tw[th.bone].rotation * t_rest, th.centre(&tw));
            let goal = centre + t_turn * (turn * (s_turn.inverse() * d)) * scale;
            let (sa, sb, sc) = (
                tw[limb.upper].position(),
                tw[limb.lower].position(),
                tw[limb.end].position(),
            );
            let goal = within(goal, centre, (sa, 0.999 * (sa.distance(sb) + sb.distance(sc))));
            reach_in(target, played, &tw, &limb, sc.lerp(goal, w));
            // The arm no deeper in the head than the source's is in theirs.
            let Some(sl) = s_body.arms.iter().find(|sa| sa.limb.end == a) else {
                continue;
            };
            let deep = |p: [Vec3; 3], c: Vec3| off_line(c, p[0], p[1]).min(off_line(c, p[1], p[2]));
            let s_arm = [sl.limb.upper, sl.limb.lower, a].map(|b| sw[b].position());
            let need = deep(s_arm, sc).min(sh.radius) / sh.radius * th.radius;
            place(target, played, &t_path, &mut tw);
            swivel(target, played, &tw, &limb, |arm| deep(arm, centre), need);
        }
    }
}

/// How far `p` is from the line from `a` to `b`.
fn off_line(p: Vec3, a: Vec3, b: Vec3) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_squared().max(1e-12)).clamp(0.0, 1.0);
    p.distance(a + ab * t)
}

/// Turns an arm about the line from its shoulder to its hand, the elbow swinging round while the hand stays where it
/// is and keeps its turn, as little as gets `clear` of the arm's three joints (shoulder, elbow, hand) to `need`;
/// failing that, as far as gets the most. `world` is the pose placed.
fn swivel(
    skeleton: &Skeleton,
    pose: &mut [Transform],
    world: &[At],
    limb: &Limb,
    clear: impl Fn([Vec3; 3]) -> f32,
    need: f32,
) {
    let (a, b, c) = (
        world[limb.upper].position(),
        world[limb.lower].position(),
        world[limb.end].position(),
    );
    let now = clear([a, b, c]);
    let Some(axis) = (c - a).try_normalize().filter(|_| now < need) else {
        return;
    };
    let mut best = (now, Quat::IDENTITY);
    'search: for k in 1..=12 {
        for sign in [1.0, -1.0] {
            let turn = Quat::from_axis_angle(axis, sign * k as f32 * std::f32::consts::PI / 12.0);
            let got = clear([a, a + turn * (b - a), c]);
            if got > best.0 {
                best = (got, turn);
                if got >= need {
                    break 'search;
                }
            }
        }
    }
    let turn = best.1;
    if turn == Quat::IDENTITY {
        return;
    }
    let parent = |b: usize| skeleton.bones()[b].parent;
    let above = parent(limb.upper).map_or(Quat::IDENTITY, |p| world[p].rotation);
    pose[limb.upper].rotation = (above.inverse() * turn * world[limb.upper].rotation).normalize();
    // The hand's parent turns with the arm; the hand is turned back to where it was.
    if let Some(p) = parent(limb.end) {
        pose[limb.end].rotation = ((turn * world[p].rotation).inverse() * world[limb.end].rotation).normalize();
    }
}

/// `goal`, or if it's out of reach (a ball round a shoulder), the point nearest it as far from `centre` as it is, over
/// the head and within reach: a short arm whose hand can't get to the top of a big head rests it as high up the side as
/// it gets, not inside. With nowhere on that sphere in reach, the point on it nearest the shoulder.
fn within(goal: Vec3, centre: Vec3, (shoulder, reach): (Vec3, f32)) -> Vec3 {
    if goal.distance(shoulder) <= reach {
        return goal;
    }
    let (rho, apart) = (goal.distance(centre), shoulder - centre);
    let dist = apart.length();
    let Some(u) = apart.try_normalize() else {
        return goal;
    };
    // The circle where the two spheres cross: on the plane `h` along `u` from the centre, `ring` round.
    let h = (dist * dist + rho * rho - reach * reach) / (2.0 * dist);
    let ring = rho * rho - h * h;
    if ring <= 0.0 || h.abs() > rho {
        return centre + u * rho;
    }
    let off = goal - centre;
    let side = (off - u * off.dot(u))
        .try_normalize()
        .unwrap_or_else(|| u.any_orthonormal_vector());
    centre + u * h + side * ring.sqrt()
}

/// Where two hands meet, at `m + offsets[0]` and `m + offsets[1]`: the `m` nearest `start`, and `last` if given, with
/// each hand within reach (from a shoulder, as far as an arm's length) and outside every sphere (a centre and a
/// radius). Short of that, as near to it as can be found.
fn meet(start: Vec3, last: Option<Vec3>, offsets: [Vec3; 2], reach: [(Vec3, f32); 2], spheres: &[(Vec3, f32)]) -> Vec3 {
    // Met each in turn, a few times over: a reach is a ball the hand must be in, a sphere one it must be out of.
    let settle = |mut m: Vec3| {
        for _ in 0..8 {
            for (o, (a, r)) in offsets.iter().zip(reach) {
                let d = m + *o - a;
                let len = d.length();
                if len > r {
                    m -= d * (1.0 - r / len);
                }
            }
            for &(c, r) in spheres {
                for o in offsets {
                    let d = m + o - c;
                    let len = d.length();
                    if len < r {
                        m += d.normalize_or(Vec3::Z) * (r - len);
                    }
                }
            }
        }
        m
    };
    let miss = |m: Vec3| {
        let mut v = 0.0f32;
        for (o, (a, r)) in offsets.iter().zip(reach) {
            v += ((m + *o).distance(a) - r).max(0.0);
        }
        for &(c, r) in spheres {
            for o in offsets {
                v += (r - (m + o).distance(c)).max(0.0);
            }
        }
        v
    };
    let cost = |m: Vec3| m.distance_squared(start) + last.map_or(0.0, |l| m.distance_squared(l));
    let first = settle(start);
    let tol = 1e-3 * reach[0].1.max(reach[1].1);
    if miss(first) <= tol {
        return first;
    }
    // Caught between a sphere and the reach of an arm, as short arms clapping overhead are caught against the head.
    // Try the points around each sphere still in the way, either hand against it, settled from there.
    let mut best = (miss(first), cost(first), first);
    let tried = |m: Vec3, best: &mut (f32, f32, Vec3)| {
        let m = settle(m);
        let (e, c) = (miss(m), cost(m));
        let better = if e <= tol && best.0 <= tol {
            c < best.1
        } else {
            e < best.0
        };
        if better {
            *best = (e, c, m);
        }
    };
    if let Some(l) = last {
        tried(l, &mut best);
    }
    const N: usize = 32;
    for &(c, r) in spheres {
        if offsets.iter().all(|o| (first + *o).distance(c) >= r - tol) {
            continue;
        }
        for k in 0..N {
            // Points spread evenly over the sphere (a Fibonacci lattice).
            let y = 1.0 - 2.0 * (k as f32 + 0.5) / N as f32;
            let a = k as f32 * 2.399_963;
            let ring = (1.0 - y * y).sqrt();
            let dir = Vec3::new(ring * a.cos(), y, ring * a.sin());
            for o in offsets {
                tried(c + dir * (r + tol) - o, &mut best);
            }
        }
    }
    best.2
}
