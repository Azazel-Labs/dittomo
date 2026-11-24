//! Contacts: feet that stay put while planted, travel for clips walked in place, arms kept out of the body, and hands
//! that meet each other or the head where the source's do.
//!
//! Limbs are found from the rest pose alone, so no names are needed. A leg is a bone near the floor whose parent stands
//! well above it, with the bone above that: foot, knee, hip. An arm is a chain off the spine above where the legs
//! branch, down to where it splits into fingers or ends: hand, elbow, shoulder. Everything after is closed form, with
//! no solver: [`reach`] is two-bone IK, and the push-out is one step against spheres along the spine (after the ultrafast
//! online retargeting of Wang et al., 2025, which runs the same step against spheres fitted to the mesh).

use glam::{Affine3A, Quat, Vec3, Vec3A};

use crate::shape::Shape;
use crate::surface::{Balls, Surface, under};
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
/// from that chain (`split`), so twist bones along it don't count.
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

/// The spine from the hips to the chest, as far as the body is found.
pub(crate) fn spine(skeleton: &Skeleton, s: &Shape) -> Vec<usize> {
    body(skeleton, s).map_or(Vec::new(), |b| b.spine)
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
    let mut out = Vec::with_capacity(n);
    detect(at, times, ground, height, &mut out);
    out
}

/// Into `out`, for each place a foot is at, whether it is planted: within a twenty-fifth of the height of `ground` and
/// moving under a quarter of the height a second coming or going (a foot still on one side has just landed or is about
/// to lift, but stands), with one-pose gaps filled and one-pose plants dropped. A contact then holds on through the
/// poses either side of it the foot is still that slow and within three fiftieths of the height of `ground` in, as one
/// played live does, so it isn't let go of while the foot is only a little higher. Planted poses either side of a quick
/// step are [`apart`]: a contact is a run of planted poses none of which is apart from the one before.
fn detect(at: &[Vec3], times: &[f32], ground: f32, height: f32, out: &mut Vec<bool>) {
    let n = at.len();
    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
    out.clear();
    // How fast the foot moves between two poses, if time passes between them.
    let speed = |i: usize, j: usize| {
        let dt = times[j] - times[i];
        (dt > 0.0).then(|| flat(at[j] - at[i]).length() / dt)
    };
    // How high the foot is off the ground at a pose, and the slower of coming and going.
    let at_pose = |f: usize| {
        let coming = f.checked_sub(1).and_then(|i| speed(i, f));
        let going = (f + 1 < n).then(|| speed(f, f + 1)).flatten();
        let slowest = match (coming, going) {
            (Some(c), Some(g)) => c.min(g),
            (c, g) => c.or(g).unwrap_or(0.0),
        };
        (at[f].y - ground, slowest)
    };
    out.extend((0..n).map(|f| {
        let (low, slowest) = at_pose(f);
        low < 0.04 * height && slowest < 0.25 * height
    }));
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
    let holds = |f: usize| {
        let (low, slowest) = at_pose(f);
        low < 0.06 * height && slowest < 0.25 * height
    };
    for f in 1..n {
        if !out[f] && out[f - 1] && holds(f) {
            out[f] = true;
        }
    }
    for f in (0..n.saturating_sub(1)).rev() {
        if !out[f] && out[f + 1] && holds(f) {
            out[f] = true;
        }
    }
}

/// Whether the foot at `at[f]` is planted somewhere else than at the pose before, having moved there quickly: the end of
/// one contact and the start of the next, a step taken between two poses.
fn apart(at: &[Vec3], times: &[f32], height: f32, f: usize) -> bool {
    f > 0 && {
        let dt = times[f] - times[f - 1];
        let d = at[f] - at[f - 1];
        dt > 0.0 && Vec3::new(d.x, 0.0, d.z).length() >= 0.25 * height * dt
    }
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
                f += 1;
                while f < n && down[f] && !apart(&s_at, &from.times, s.height, f) {
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

/// How long, in seconds, a foot takes to settle where it is held once it plants, and to let go once it lifts.
const SETTLE: f32 = 0.12;

/// The smooth step from 0 to 1 as `x` goes from 0 to 1.
fn smooth(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// Where each of the source's feet comes down to, worked out as the poses come from those so far. Feet that walk on
/// tiptoe never come down to where they stand at rest: once one foot is well above where the lowest came down to, that
/// is the floor, till a foot comes down well below it; and once a foot is well above where it came down to, that is its
/// own ground, as feet needn't stand equally high. Till then a foot's ground is the lowest it has been, or where it
/// stands at rest if lower, so a foot coming down from its first step isn't taken to be on the floor.
#[derive(Clone)]
pub(crate) struct Ground {
    /// Each foot's height at rest.
    rest: Vec<f32>,
    ground: Vec<f32>,
    /// The lowest each foot has been, and whether it has since been well above it.
    low: Vec<(f32, bool)>,
    /// The lowest any foot has been, and whether a foot has since been well above it, so that it is known to be the
    /// floor the feet walk on.
    floor: f32,
    known: bool,
    height: f32,
}

impl Ground {
    /// For feet standing at these heights at rest, on a body this tall.
    fn new(rest: Vec<f32>, height: f32) -> Self {
        let n = rest.len();
        Self {
            ground: rest.clone(),
            rest,
            low: vec![(f32::INFINITY, false); n],
            floor: f32::INFINITY,
            known: false,
            height,
        }
    }

    /// Forgets the poses so far, as after a cut.
    fn reset(&mut self) {
        self.ground.clone_from(&self.rest);
        self.low.fill((f32::INFINITY, false));
        (self.floor, self.known) = (f32::INFINITY, false);
    }

    /// Takes in how high foot `l` is in the next pose, as `at(l)`, for every foot.
    fn see(&mut self, at: impl Fn(usize) -> f32) {
        let h = self.height;
        let (mut bottom, mut top) = (f32::INFINITY, f32::NEG_INFINITY);
        for l in 0..self.rest.len() {
            let y = at(l);
            (bottom, top) = (bottom.min(y), top.max(y));
        }
        if bottom < self.floor {
            self.known &= self.floor - bottom < 0.01 * h;
            self.floor = bottom;
        }
        self.known |= top - self.floor > 0.04 * h;
        for l in 0..self.rest.len() {
            let y = at(l);
            let (own, lifted) = &mut self.low[l];
            if y < *own {
                *lifted &= *own - y < 0.01 * h;
                *own = y;
            }
            *lifted |= y - *own > 0.04 * h;
            self.ground[l] = if *lifted {
                *own
            } else if self.known {
                self.floor
            } else {
                self.ground[l].min(y)
            };
        }
    }

    /// Foot `l`'s ground.
    fn of(&self, l: usize) -> f32 {
        self.ground[l]
    }
}

/// Feet kept still while the source's are planted, worked out as the poses come from those so far, with nothing ahead
/// to look at. A source foot plants as soon as it is down and nearly still, and lifts once it moves off or rises a little
/// higher, so it doesn't flicker between the two. The target's is held where it was the pose before it planted, moved on
/// as far as the source's has since (scaled), at the height the source's stands scaled, and let go over [`SETTLE`]. A foot coming down to the floor is brought to the source's
/// height scaled as it nears it, so it lands without a jump.
pub(crate) struct Feet {
    feet: Vec<Foot>,
    ground: Ground,
    s_path: Vec<usize>,
    t_path: Vec<usize>,
    /// The target's height over the source's.
    k: f32,
    s_height: f32,
}

#[derive(Clone, Copy)]
struct Foot {
    leg: Limb,
    source: usize,
    s_rest_y: f32,
    t_rest_y: f32,
    last: Option<Vec3>,
    /// The source foot's speed across the floor.
    speed: f32,
    down: bool,
    /// Where the target's foot is held, if it is or is letting go.
    lock: Option<Vec3>,
    /// Where the target's foot was shown the pose before.
    shown: Option<Vec3>,
    /// Seconds since the foot planted or lifted.
    since: f32,
}

impl Feet {
    /// Nothing if no target leg's foot plays a source bone.
    pub(crate) fn new(map: &Mapping, [(source, s), (target, t)]: [(&Skeleton, &Shape); 2]) -> Option<Self> {
        let feet: Vec<Foot> = legs_of(target, t)
            .into_iter()
            .filter_map(|leg| {
                let a = map.source_of(leg.end)?;
                Some(Foot {
                    leg,
                    source: a,
                    s_rest_y: s.rest[a].position().y,
                    t_rest_y: t.rest[leg.end].position().y,
                    last: None,
                    speed: 0.0,
                    down: false,
                    lock: None,
                    shown: None,
                    since: 0.0,
                })
            })
            .collect();
        (!feet.is_empty()).then(|| Self {
            ground: Ground::new(feet.iter().map(|f| f.s_rest_y).collect(), s.height),
            s_path: paths(source, feet.iter().map(|f| f.source)),
            t_path: paths(target, feet.iter().map(|f| f.leg.end)),
            feet,
            k: t.height / s.height,
            s_height: s.height,
        })
    }

    /// Forgets every contact, as after a cut.
    pub(crate) fn reset(&mut self) {
        for f in &mut self.feet {
            *f = Foot {
                last: None,
                speed: 0.0,
                down: false,
                lock: None,
                shown: None,
                since: 0.0,
                ..*f
            };
        }
        self.ground.reset();
    }

    /// The source bones to place ([`place_source`]) before each pose.
    pub(crate) fn s_path(&self) -> &[usize] {
        &self.s_path
    }

    /// Fixes `played`, the target playing a pose of the source placed in `scratch` ([`Self::s_path`]), `dt` seconds
    /// after the pose before.
    pub(crate) fn pose(&mut self, target: &Skeleton, dt: f32, played: &mut [Transform], scratch: &mut Scratch) {
        let h = self.s_height;
        let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
        let Scratch { sw, tw, .. } = scratch;
        self.ground.see(|l| sw[self.feet[l].source].position().y);
        place(target, played, &self.t_path, tw);
        for (l, f) in self.feet.iter_mut().enumerate() {
            let at = sw[f.source].position();
            let step = f.last.map_or(Vec3::ZERO, |last| flat(at - last));
            if dt > 0.0 && f.last.is_some() {
                f.speed = step.length() / dt;
            }
            f.last = Some(at);
            let low = at.y - self.ground.of(l);
            f.since += dt;
            if f.down {
                if low > 0.06 * h || f.speed > 0.35 * h {
                    (f.down, f.since) = (false, 0.0);
                }
            } else if low < 0.04 * h && f.speed < 0.35 * h {
                // Held where it was shown the pose before, moved on as the source's foot has since: the foot has only
                // just stopped, and where the target's is now it may have slid to as the legs bent unlike the source's.
                (f.down, f.since) = (true, 0.0);
                f.lock = f.shown.map(|p| p + step * self.k);
            }
            // Held across the floor at once, as the foot is where it planted, and let go over SETTLE. Its height is
            // the source's scaled while planted, and on the way down to the floor from twice the height a foot plants
            // at, so it doesn't jump on landing.
            let ease = match (f.down, f.lock) {
                (true, _) => 1.0,
                (false, Some(_)) if f.since < SETTLE => smooth(1.0 - f.since / SETTLE),
                _ => {
                    f.lock = None;
                    0.0
                }
            };
            let up = ease.max(smooth((0.08 * h - low) / (0.04 * h)));
            let free = tw[f.leg.end].position();
            if up <= 0.0 {
                f.shown = Some(free);
                continue;
            }
            let lock = if f.down {
                *f.lock.get_or_insert(free)
            } else {
                f.lock.unwrap_or(free)
            };
            let y = f.t_rest_y + (at.y - f.s_rest_y) * self.k;
            let goal = Vec3::new(
                free.x + (lock.x - free.x) * ease,
                free.y + (y - free.y) * up,
                free.z + (lock.z - free.z) * ease,
            );
            // Legs hang apart, so reaching one leaves the other placed as it was.
            reach_in(target, played, tw, &f.leg, goal);
            f.shown = Some(goal);
        }
    }
}

/// Feet kept still while the source's are planted, as [`plant`] keeps them over a whole clip, but from the poses so
/// far and a few ahead: each pose is fixed once the `delay` poses after it have come. Whether a foot is planted is
/// judged as over a whole clip, over the poses kept; it is held at the average of where it is over the part of its
/// contact in sight when that contact begins, and eased toward it from as many poses before as [`plant`] would, or as
/// many as are in sight.
pub(crate) struct Ahead {
    feet: Vec<Foot>,
    ground: Ground,
    t_path: Vec<usize>,
    k: f32,
    s_height: f32,
    delay: usize,
    /// For each pose kept (the `delay` ahead of the one fixed, it and the two before it), by its number modulo how
    /// many are kept: when it came, and where each foot is in the source and in the target before any fix.
    times: Vec<f32>,
    s_at: Vec<Vec3>,
    t_at: Vec<Vec3>,
    /// Each foot's contact the poses fixed are in or have just left: where it is held, and the pose it ended at.
    held: Vec<Option<(Vec3, Option<usize>)>>,
    seen: usize,
    clock: f32,
    w_at: Vec<Vec3>,
    w_times: Vec<f32>,
    w_down: Vec<bool>,
}

impl Ahead {
    /// [`Feet`]'s feet, fixed `delay` poses late.
    pub(crate) fn new(feet: &Feet, delay: usize) -> Self {
        let kept = delay + 3;
        let n = feet.feet.len();
        let mut ahead = Self {
            feet: feet.feet.clone(),
            ground: feet.ground.clone(),
            t_path: feet.t_path.clone(),
            k: feet.k,
            s_height: feet.s_height,
            delay,
            times: vec![0.0; kept],
            s_at: vec![Vec3::ZERO; kept * n],
            t_at: vec![Vec3::ZERO; kept * n],
            held: vec![None; n],
            seen: 0,
            clock: 0.0,
            w_at: Vec::with_capacity(kept),
            w_times: Vec::with_capacity(kept),
            w_down: Vec::with_capacity(kept),
        };
        ahead.reset();
        ahead
    }

    /// Forgets every contact, as after a cut.
    pub(crate) fn reset(&mut self) {
        self.ground.reset();
        self.held.fill(None);
        (self.seen, self.clock) = (0, 0.0);
    }

    /// Takes in the next pose of the source, placed in `scratch` as for [`Feet`], `dt` seconds after the one before,
    /// and the target playing it.
    pub(crate) fn see(&mut self, target: &Skeleton, dt: f32, played: &[Transform], scratch: &mut Scratch) {
        let (n, kept) = (self.feet.len(), self.times.len());
        let slot = self.seen % kept;
        if self.seen > 0 {
            self.clock += dt.max(0.0);
        }
        self.times[slot] = self.clock;
        let Scratch { sw, tw, .. } = scratch;
        place(target, played, &self.t_path, tw);
        self.ground.see(|l| sw[self.feet[l].source].position().y);
        for (i, f) in self.feet.iter().enumerate() {
            let at = sw[f.source].position();
            self.s_at[slot * n + i] = at;
            self.t_at[slot * n + i] = tw[f.leg.end].position();
        }
        self.seen += 1;
    }

    /// Fixes `played`, the target playing pose number `o` of those seen, the `delay`th before the last.
    pub(crate) fn fix(&mut self, target: &Skeleton, o: usize, played: &mut [Transform], scratch: &mut Scratch) {
        let (n, kept, h) = (self.feet.len(), self.times.len(), self.s_height);
        let Some(last) = self.seen.checked_sub(1) else {
            return;
        };
        let first = o.saturating_sub(2);
        let slot = |f: usize| f % kept;
        let span = self.times[slot(last)] - self.times[slot(first)];
        let ramp = if last > first && span > 0.0 {
            ((0.12 * (last - first) as f32 / span).round() as usize).clamp(1, 10)
        } else {
            4
        }
        .min(self.delay);
        let at_o = o - first;
        let tw = &mut scratch.tw;
        let mut placed = false;
        for (i, f) in self.feet.iter().enumerate() {
            self.w_at.clear();
            self.w_times.clear();
            for g in first..=last {
                self.w_at.push(self.s_at[slot(g) * n + i]);
                self.w_times.push(self.times[slot(g)]);
            }
            detect(&self.w_at, &self.w_times, self.ground.of(i), h, &mut self.w_down);
            let (down, w_at, w_times) = (&self.w_down, &self.w_at, &self.w_times);
            let t_at = &self.t_at;
            let apart = |w: usize| apart(w_at, w_times, h, w);
            // The average of where the target's foot is over the contact from pose `j` (of those kept) on, in sight.
            let lock_from = |j: usize| {
                let (sum, count) = (j..down.len())
                    .take_while(|&w| down[w] && (w == j || !apart(w)))
                    .fold((Vec3::ZERO, 0.0), |(s, c), w| {
                        (s + t_at[slot(first + w) * n + i], c + 1.0)
                    });
                sum / count
            };
            let mut near: Option<(usize, Vec3)> = None;
            if down[at_o] {
                let lock = match self.held[i] {
                    Some((lock, None)) if !apart(at_o) => lock,
                    _ => lock_from(at_o),
                };
                self.held[i] = Some((lock, None));
                near = Some((0, lock));
            } else {
                if let Some((lock, None)) = self.held[i] {
                    self.held[i] = Some((lock, Some(o)));
                }
                if let Some((lock, Some(end))) = self.held[i] {
                    near = Some((o + 1 - end, lock));
                }
                if let Some(j) = (at_o + 1..down.len()).find(|&w| down[w]) {
                    let d = j - at_o;
                    if near.is_none_or(|(e, _)| d < e) {
                        near = Some((d, lock_from(j)));
                    }
                }
            }
            let Some((d, lock)) = near else { continue };
            if d > ramp {
                continue;
            }
            let x = 1.0 - d as f32 / (ramp + 1) as f32;
            let w = x * x * (3.0 - 2.0 * x);
            if !placed {
                place(target, played, &self.t_path, tw);
                placed = true;
            }
            let y = f.t_rest_y + (self.s_at[slot(o) * n + i].y - f.s_rest_y) * self.k;
            let free = tw[f.leg.end].position();
            // Legs hang apart, so reaching one leaves the other placed as it was.
            reach_in(target, played, tw, &f.leg, free.lerp(Vec3::new(lock.x, y, lock.z), w));
        }
    }
}

/// The travel a clip walked in place implies ([`travelled`]), worked out as the poses come: each pose the body moves as
/// far as the lowest foot slid back since the pose before, and on as it was going while no foot is down. Slides slower
/// than a twentieth of the height a second are taken for a foot shuffling, not walking. Once the hips have wandered a
/// tenth of the height from where they began, the clip travels on its own and the travel stops growing: a walk in place
/// sways less, and a dance in place that sways more mustn't drift off far before it does.
pub(crate) struct Travel {
    /// Each leg's foot and upper bone, in the source.
    legs: Vec<(usize, usize)>,
    path: Vec<usize>,
    roots: Vec<usize>,
    s_height: f32,
    /// Where each foot comes down to.
    ground: Ground,
    last: Vec<Option<Vec3>>,
    start: Option<Vec3>,
    velocity: Vec3,
    moved: Vec3,
    on: bool,
}

impl Travel {
    /// Nothing if the source has no legs.
    pub(crate) fn new(source: &Skeleton, s: &Shape) -> Option<Self> {
        let legs: Vec<(usize, usize)> = legs_of(source, s).iter().map(|l| (l.end, l.upper)).collect();
        (!legs.is_empty()).then(|| Self {
            path: paths(source, legs.iter().map(|l| l.0)),
            roots: (0..source.len())
                .filter(|&b| source.bones()[b].parent.is_none())
                .collect(),
            s_height: s.height,
            ground: Ground::new(legs.iter().map(|l| s.rest[l.0].position().y).collect(), s.height),
            last: vec![None; legs.len()],
            start: None,
            velocity: Vec3::ZERO,
            moved: Vec3::ZERO,
            on: true,
            legs,
        })
    }

    /// Forgets the travel so far, as after a cut.
    pub(crate) fn reset(&mut self) {
        self.ground.reset();
        self.last.fill(None);
        (self.start, self.velocity, self.moved, self.on) = (None, Vec3::ZERO, Vec3::ZERO, true);
    }

    /// The source bones it places.
    pub(crate) fn path(&self) -> &[usize] {
        &self.path
    }

    /// Writes `pose`, `dt` seconds after the pose before, into `out` with the travel so far, and leaves `out` placed
    /// in `scratch` over `also` (bones on paths from the roots, as [`place_source`] takes) as well as its own path.
    pub(crate) fn pose(
        &mut self,
        source: &Skeleton,
        dt: f32,
        pose: &[Transform],
        out: &mut [Transform],
        scratch: &mut Scratch,
        also: Option<&[usize]>,
    ) {
        let h = self.s_height;
        let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
        let sw = &mut scratch.sw;
        let path = also.unwrap_or(&self.path);
        place(source, pose, path, sw);
        let hips = self.legs.iter().map(|&(_, u)| sw[u].position()).sum::<Vec3>() / self.legs.len() as f32;
        let start = *self.start.get_or_insert(hips);
        if flat(hips - start).length() > 0.1 * h {
            self.on = false;
        }
        self.ground.see(|l| sw[self.legs[l].0].position().y);
        let mut lowest: Option<(f32, Vec3)> = None;
        for (l, &(foot, _)) in self.legs.iter().enumerate() {
            let at = sw[foot].position();
            let low = at.y - self.ground.of(l);
            if let Some(last) = self.last[l]
                && low < 0.04 * h
                && lowest.is_none_or(|(m, _)| low < m)
            {
                lowest = Some((low, -flat(at - last)));
            }
            self.last[l] = Some(at);
        }
        if self.on && dt > 0.0 {
            match lowest {
                Some((_, step)) if step.length() > 0.05 * h * dt => {
                    self.velocity = step / dt;
                    self.moved += step;
                }
                Some(_) => self.velocity = Vec3::ZERO,
                None => self.moved += self.velocity * dt,
            }
        }
        out.copy_from_slice(pose);
        for &r in &self.roots {
            out[r].translation += self.moved;
        }
        // Every root moved on, so every bone under one did.
        for &b in path {
            sw[b].matrix.translation += Vec3A::from(self.moved);
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
    /// The center in the bone's frame.
    center: Vec3,
    radius: f32,
}

impl Head {
    fn center(&self, world: &[At]) -> Vec3 {
        world[self.bone].matrix.transform_point3(self.center)
    }
}

impl Body {
    /// Fits the head to the skin bound to the head bone, where there's enough of it: centerd on it, as far round as
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
        let center = points.iter().sum::<Vec3>() / points.len() as f32;
        let mut far: Vec<f32> = points.iter().map(|p| p.distance(center)).collect();
        let half = far.len() / 2;
        let (_, &mut radius, _) = far.select_nth_unstable_by(half, f32::total_cmp);
        if radius > 0.02 * s.height && radius < 0.25 * s.height {
            head.center = rest.inverse().transform_point3(center);
            head.radius = radius;
        }
    }

    /// The bones to place for the spheres and the arms.
    pub(crate) fn path(&self, skeleton: &Skeleton) -> Vec<usize> {
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
        out.extend(self.head.map(|h| (h.center(world), h.radius + self.limb)));
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
    let (center, radius) = if (top_at - base).length() > 0.12 * s.height {
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
        .find(|&b| along(pos(b)) <= along(center))
        .unwrap_or(first);
    Some(Head {
        bone,
        center: s.rest[bone].matrix.inverse().transform_point3(center),
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
    let centers = spheres(&spine, pos);
    let clear = arms
        .iter()
        .flat_map(|(a, _)| {
            let (b, c) = (pos(a.limb.lower), pos(a.limb.end));
            [b, c, (b + c) / 2.0]
        })
        .flat_map(|p| centers.iter().map(move |q| p.distance(*q)))
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

/// What fixing one pose at a time needs besides the pose: each rig placed in the world, as far as needed, and the
/// spheres an arm keeps out of. Made once, so poses are fixed without allocating.
pub(crate) struct Scratch {
    sw: Vec<At>,
    tw: Vec<At>,
    spheres: Vec<(Vec3, f32)>,
}

impl Scratch {
    pub(crate) fn new(source: &Skeleton, target: &Skeleton) -> Self {
        Self {
            sw: vec![At::IDENTITY; source.len()],
            tw: vec![At::IDENTITY; target.len()],
            spheres: Vec::with_capacity(64),
        }
    }
}

/// Pushes each pose's elbows and hands out of the spheres along its spine: the upper arm turns at the shoulder to
/// clear the elbow, then the forearm at the elbow to clear the hand.
pub(crate) fn push_out(target: &Skeleton, body: &Body, clip: &mut Clip) {
    let path = body.path(target);
    let mut scratch = Scratch::new(target, target);
    for pose in &mut clip.poses {
        push_pose(target, body, &path, pose, &mut scratch);
    }
}

/// [`push_out`] for one pose, `path` being the body's.
pub(crate) fn push_pose(target: &Skeleton, body: &Body, path: &[usize], pose: &mut [Transform], scratch: &mut Scratch) {
    let parent = |b: usize| target.bones()[b].parent;
    let (world, centers) = (&mut scratch.tw, &mut scratch.spheres);
    // Arms don't move the spine or each other, so the world is worked out once a pose.
    place(target, pose, path, world);
    body.spheres(world, centers);
    let need = |p: Vec3, out: Vec3| {
        centers
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

impl crate::surface::Frame for At {
    fn point(&self, p: Vec3) -> Vec3 {
        self.matrix.transform_point3(p)
    }

    fn turn(&self) -> Quat {
        self.rotation
    }

    fn scale(&self) -> f32 {
        let m = &self.matrix.matrix3;
        m.x_axis
            .length_squared()
            .max(m.y_axis.length_squared())
            .max(m.z_axis.length_squared())
            .sqrt()
    }
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

/// Places the source's bones on these paths ([`union`]) under a pose, in `scratch`, for the fixes that look at them.
pub(crate) fn place_source(source: &Skeleton, pose: &[Transform], path: &[usize], scratch: &mut Scratch) {
    place(source, pose, path, &mut scratch.sw);
}

/// The bones on any of these paths from the roots, parents first.
pub(crate) fn union(paths: &[&[usize]]) -> Vec<usize> {
    let mut all: Vec<usize> = paths.iter().flat_map(|p| p.iter().copied()).collect();
    all.sort_unstable();
    all.dedup();
    all
}

/// Places these bones (parents first, each one's parent among them unless it's a root) in the world under a pose.
/// Each turn is made whole before it is chained, not after, so the bones down a chain needn't wait on it.
fn place(skeleton: &Skeleton, pose: &[Transform], bones: &[usize], world: &mut [At]) {
    for &b in bones {
        let l = &pose[b];
        let local = Affine3A::from_scale_rotation_translation(l.scale, l.rotation, l.translation);
        world[b] = match skeleton.bones()[b].parent {
            Some(p) => At {
                matrix: world[p].matrix * local,
                rotation: world[p].rotation * l.rotation.normalize(),
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
    bodies: [&Body; 2],
    from: &Clip,
    out: &mut Clip,
) {
    let Some(mut fix) = Touch::new(map, rigs, bodies) else {
        return;
    };
    let [(source, ..), (target, ..)] = rigs;
    let mut scratch = Scratch::new(source, target);
    for (pose, played) in from.poses.iter().zip(&mut out.poses) {
        fix.pose([source, target], pose, played, &mut scratch);
    }
}

/// Each arm the two rigs share, for [`Touch`]: the target's limb, the source's hand, each hand's middle in its own
/// frame, and the skin points of each hand and its fingers.
struct Reaching {
    limb: Limb,
    source: usize,
    middle: [Vec3; 2],
    hand: [Balls; 2],
    /// The target's bones to place again once the arm has moved: those from its upper bone down.
    below: Vec<usize>,
}

/// [`touch`], made ready for two rigs, then run a pose at a time.
pub(crate) struct Touch {
    skins: [Surface; 2],
    arms: Vec<Reaching>,
    /// The skin a hand can touch, by bone: all but the arms', from the shoulder down.
    balls: [Balls; 2],
    heights: [f32; 2],
    s_path: Vec<usize>,
    t_path: Vec<usize>,
    /// The skin near a hand, posed.
    points: Vec<(Vec3, Vec3)>,
}

impl Touch {
    /// Nothing if either skin is empty, or no arm with skin on its hand plays the source's.
    pub(crate) fn new(
        map: &Mapping,
        rigs: [(&Skeleton, &Shape, &Surface); 2],
        [s_body, t_body]: [&Body; 2],
    ) -> Option<Self> {
        let [(source, s, s_skin), (target, t, t_skin)] = rigs;
        if s_skin.is_empty() || t_skin.is_empty() {
            return None;
        }
        // The bones under each hand, the hand's among them, on each rig.
        let hands: Vec<(Limb, usize, [Vec<bool>; 2])> = played(map, s_body, t_body)
            .into_iter()
            .map(|(limb, a)| (limb, a, [under(source, &[a]), under(target, &[limb.end])]))
            .collect();
        let off_arms = |skeleton: &Skeleton, b: &Body| {
            let off = under(skeleton, &b.arms.iter().map(|a| a.limb.upper).collect::<Vec<_>>());
            off.into_iter().map(|o| !o).collect::<Vec<_>>()
        };
        let keep = [off_arms(source, s_body), off_arms(target, t_body)];
        // Every bone the skin a hand can touch moves with, and every bone under a hand with skin.
        let path = |skeleton: &Skeleton, skin: &Surface, k: usize| {
            let on = |b: usize| keep[k][b] || hands.iter().any(|h| h.2[k][b]);
            let ends = hands.iter().map(|h| if k == 0 { h.1 } else { h.0.end });
            paths(skeleton, skin.bone.iter().copied().filter(|&b| on(b)).chain(ends))
        };
        let (s_path, t_path) = (path(source, s_skin, 0), path(target, t_skin, 1));
        let arms: Vec<Reaching> = hands
            .iter()
            .filter_map(|(limb, a, [s_on, t_on])| {
                let moved = under(target, &[limb.upper]);
                Some(Reaching {
                    limb: *limb,
                    source: *a,
                    middle: [s_skin.center(*a)?, t_skin.center(limb.end)?],
                    hand: [s_skin.balls(|b| s_on[b]), t_skin.balls(|b| t_on[b])],
                    below: t_path.iter().copied().filter(|&b| moved[b]).collect(),
                })
            })
            .collect();
        if arms.is_empty() {
            return None;
        }
        Some(Self {
            points: Vec::with_capacity(s_skin.len().max(t_skin.len())),
            balls: [s_skin.balls(|b| keep[0][b]), t_skin.balls(|b| keep[1][b])],
            skins: [s_skin.clone(), t_skin.clone()],
            arms,
            heights: [s.height, t.height],
            s_path,
            t_path,
        })
    }

    /// The source bones to place ([`place_source`]) before [`Self::placed`].
    pub(crate) fn s_path(&self) -> &[usize] {
        &self.s_path
    }

    pub(crate) fn pose(
        &mut self,
        [source, target]: [&Skeleton; 2],
        pose: &[Transform],
        played: &mut [Transform],
        scratch: &mut Scratch,
    ) {
        place(source, pose, &self.s_path, &mut scratch.sw);
        self.placed(target, played, scratch);
    }

    /// [`Self::pose`], with the source's pose placed in `scratch` ([`Self::s_path`]).
    pub(crate) fn placed(&mut self, target: &Skeleton, played: &mut [Transform], scratch: &mut Scratch) {
        // Within `near` of the skin (over the height) a source hand is touching it; past `far` it is free.
        let (near, far) = (0.015, 0.05);
        let ([s_skin, t_skin], [s_balls, t_balls]) = (&self.skins, &self.balls);
        let [s_height, t_height] = self.heights;
        let scale = t_height / s_height;
        let pts = &mut self.points;
        let Scratch { sw, tw, .. } = scratch;
        place(target, played, &self.t_path, tw);
        for arm in &self.arms {
            let ([cs, ct], [s_hand, t_hand]) = (arm.middle, &arm.hand);
            let c = sw[arm.source].matrix.transform_point3(cs);
            s_skin.posed_near(s_balls, sw, c, pts);
            let Some((g, _)) = s_skin.clearance(pts, sw, c, s_hand) else {
                continue;
            };
            let g = g.max(0.0);
            let x = ((g / s_height - near) / (far - near)).clamp(0.0, 1.0);
            let w = 1.0 - x * x * (3.0 - 2.0 * x);
            let end = tw[arm.limb.end];
            let c = end.matrix.transform_point3(ct);
            t_skin.posed_near(t_balls, tw, c, pts);
            let Some((now, facing)) = t_skin.clearance(pts, tw, c, t_hand) else {
                continue;
            };
            let mut want = now + w * (g * scale - now);
            // Out of the skin, unless so deep the nearest skin is likely the far side.
            if now > -far * t_height {
                want = want.max(0.0);
            }
            if (want - now).abs() > 1e-4 * t_height {
                reach_in(target, played, tw, &arm.limb, end.position() + facing * (want - now));
                place(target, played, &arm.below, tw);
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
    let Some(mut fix) = Hands::new(map, rigs, [s_body, t_body]) else {
        return;
    };
    let [(source, _), (target, _)] = rigs;
    let mut scratch = Scratch::new(source, target);
    for (pose, played) in from.poses.iter().zip(&mut out.poses) {
        fix.pose([source, target], t_body, pose, played, &mut scratch);
    }
}

/// [`hands`], made ready for two rigs, then run a pose at a time.
pub(crate) struct Hands {
    arms: Vec<(Limb, usize)>,
    pairs: Vec<(usize, usize)>,
    s_path: Vec<usize>,
    t_path: Vec<usize>,
    turn: Quat,
    scale: f32,
    s_height: f32,
    /// Where each pair of hands met last pose, if they did.
    last: Vec<Option<Vec3>>,
}

impl Hands {
    /// Nothing if no two arms play the source's.
    pub(crate) fn new(map: &Mapping, rigs: [(&Skeleton, &Shape); 2], [s_body, t_body]: [&Body; 2]) -> Option<Self> {
        let [(source, s), (target, t)] = rigs;
        let arms = played(map, s_body, t_body);
        let pairs: Vec<(usize, usize)> = (0..arms.len())
            .flat_map(|i| (i + 1..arms.len()).map(move |j| (i, j)))
            .collect();
        if pairs.is_empty() {
            return None;
        }
        Some(Self {
            s_path: paths(source, arms.iter().map(|a| a.1)),
            t_path: t_body.path(target),
            turn: map.turn(),
            scale: t.height / s.height,
            s_height: s.height,
            last: vec![None; pairs.len()],
            arms,
            pairs,
        })
    }

    /// Forgets where the hands met, as after a cut.
    pub(crate) fn reset(&mut self) {
        self.last.fill(None);
    }

    /// The source bones to place ([`place_source`]) before [`Self::placed`].
    pub(crate) fn s_path(&self) -> &[usize] {
        &self.s_path
    }

    pub(crate) fn pose(
        &mut self,
        [source, target]: [&Skeleton; 2],
        t_body: &Body,
        pose: &[Transform],
        played: &mut [Transform],
        scratch: &mut Scratch,
    ) {
        place(source, pose, &self.s_path, &mut scratch.sw);
        self.placed(target, t_body, played, scratch);
    }

    /// [`Self::pose`], with the source's pose placed in `scratch` ([`Self::s_path`]).
    pub(crate) fn placed(&mut self, target: &Skeleton, t_body: &Body, played: &mut [Transform], scratch: &mut Scratch) {
        // Within `near` of each other (over the height) the source's hands are together; past `far`, apart.
        let (near, far) = (0.06, 0.15);
        let Scratch { sw, tw, spheres } = scratch;
        for (k, &(i, j)) in self.pairs.iter().enumerate() {
            let ((li, si), (lj, sj)) = (self.arms[i], self.arms[j]);
            let apart = sw[sj].position() - sw[si].position();
            let x = ((apart.length() / self.s_height - near) / (far - near)).clamp(0.0, 1.0);
            let w = 1.0 - x * x * (3.0 - 2.0 * x);
            if w <= 0.0 {
                self.last[k] = None;
                continue;
            }
            place(target, played, &self.t_path, tw);
            t_body.spheres(tw, spheres);
            let (hi, hj) = (tw[li.end].position(), tw[lj.end].position());
            let way = (hj - hi).lerp(self.turn * apart * self.scale, w);
            let shoulder = |l: &Limb| {
                let (a, b, c) = (tw[l.upper].position(), tw[l.lower].position(), tw[l.end].position());
                (a, 0.999 * (a.distance(b) + b.distance(c)))
            };
            let m = meet(
                (hi + hj) / 2.0,
                self.last[k],
                [-way / 2.0, way / 2.0],
                [shoulder(&li), shoulder(&lj)],
                spheres,
            );
            self.last[k] = Some(m);
            // Arms hang apart, so reaching one leaves the other placed as it was.
            reach_in(target, played, tw, &li, m - way / 2.0);
            reach_in(target, played, tw, &lj, m + way / 2.0);
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
    let Some(fix) = OnHead::new(map, rigs, [s_body, t_body]) else {
        return;
    };
    let [(source, _), (target, _)] = rigs;
    let mut scratch = Scratch::new(source, target);
    for (pose, played) in from.poses.iter().zip(&mut out.poses) {
        fix.pose([source, target], pose, played, &mut scratch);
    }
}

/// [`on_head`], made ready for two rigs, then run a pose at a time.
pub(crate) struct OnHead {
    heads: [Head; 2],
    /// Each target arm playing a source arm: the target's limb, the source's hand and the source's limb.
    arms: Vec<(Limb, usize, Limb)>,
    turn: Quat,
    scale: f32,
    /// Each head bone's rest turn, undone.
    rests: [Quat; 2],
    s_path: Vec<usize>,
    t_path: Vec<usize>,
}

impl OnHead {
    /// Nothing if either body has no head, or no arm plays the source's.
    pub(crate) fn new(map: &Mapping, rigs: [(&Skeleton, &Shape); 2], [s_body, t_body]: [&Body; 2]) -> Option<Self> {
        let [(source, s), (target, t)] = rigs;
        let (sh, th) = (s_body.head?, t_body.head?);
        let arms: Vec<(Limb, usize, Limb)> = played(map, s_body, t_body)
            .into_iter()
            .filter_map(|(limb, a)| Some((limb, a, s_body.arms.iter().find(|sa| sa.limb.end == a)?.limb)))
            .collect();
        if arms.is_empty() {
            return None;
        }
        Some(Self {
            heads: [sh, th],
            turn: map.turn(),
            scale: th.radius / sh.radius,
            rests: [s.rest[sh.bone].rotation.inverse(), t.rest[th.bone].rotation.inverse()],
            s_path: paths(
                source,
                arms.iter().flat_map(|a| [a.1, a.2.upper, a.2.lower]).chain([sh.bone]),
            ),
            t_path: paths(target, arms.iter().map(|a| a.0.end).chain([th.bone])),
            arms,
        })
    }

    /// The source bones to place ([`place_source`]) before [`Self::placed`].
    pub(crate) fn s_path(&self) -> &[usize] {
        &self.s_path
    }

    pub(crate) fn pose(
        &self,
        [source, target]: [&Skeleton; 2],
        pose: &[Transform],
        played: &mut [Transform],
        scratch: &mut Scratch,
    ) {
        place(source, pose, &self.s_path, &mut scratch.sw);
        self.placed(target, played, scratch);
    }

    /// [`Self::pose`], with the source's pose placed in `scratch` ([`Self::s_path`]).
    pub(crate) fn placed(&self, target: &Skeleton, played: &mut [Transform], scratch: &mut Scratch) {
        let (near, far) = (1.5, 2.0);
        let ([sh, th], [s_rest, t_rest]) = (self.heads, self.rests);
        let Scratch { sw, tw, .. } = scratch;
        let (sc, s_turn) = (sh.center(sw), sw[sh.bone].rotation * s_rest);
        let mut placed = false;
        for &(limb, a, sl) in &self.arms {
            let d = sw[a].position() - sc;
            let x = ((d.length() / sh.radius - near) / (far - near)).clamp(0.0, 1.0);
            let w = 1.0 - x * x * (3.0 - 2.0 * x);
            if w <= 0.0 {
                continue;
            }
            // Arms hang apart, so reaching one leaves the other, and the head, placed as they were.
            if !placed {
                place(target, played, &self.t_path, tw);
                placed = true;
            }
            let (t_turn, center) = (tw[th.bone].rotation * t_rest, th.center(tw));
            let goal = center + t_turn * (self.turn * (s_turn.inverse() * d)) * self.scale;
            let (sa, sb, sc) = (
                tw[limb.upper].position(),
                tw[limb.lower].position(),
                tw[limb.end].position(),
            );
            let goal = within(goal, center, (sa, 0.999 * (sa.distance(sb) + sb.distance(sc))));
            reach_in(target, played, tw, &limb, sc.lerp(goal, w));
            // The arm no deeper in the head than the source's is in theirs.
            let deep = |p: [Vec3; 3], c: Vec3| off_line(c, p[0], p[1]).min(off_line(c, p[1], p[2]));
            let s_arm = [sl.upper, sl.lower, a].map(|b| sw[b].position());
            let need = deep(s_arm, sc).min(sh.radius) / sh.radius * th.radius;
            place(target, played, &self.t_path, tw);
            swivel(target, played, tw, &limb, |arm| deep(arm, center), need);
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

/// `goal`, or if it's out of reach (a ball round a shoulder), the point nearest it as far from `center` as it is, over
/// the head and within reach: a short arm whose hand can't get to the top of a big head rests it as high up the side as
/// it gets, not inside. With nowhere on that sphere in reach, the point on it nearest the shoulder.
fn within(goal: Vec3, center: Vec3, (shoulder, reach): (Vec3, f32)) -> Vec3 {
    if goal.distance(shoulder) <= reach {
        return goal;
    }
    let (rho, apart) = (goal.distance(center), shoulder - center);
    let dist = apart.length();
    let Some(u) = apart.try_normalize() else {
        return goal;
    };
    // The circle where the two spheres cross: on the plane `h` along `u` from the center, `ring` round.
    let h = (dist * dist + rho * rho - reach * reach) / (2.0 * dist);
    let ring = rho * rho - h * h;
    if ring <= 0.0 || h.abs() > rho {
        return center + u * rho;
    }
    let off = goal - center;
    let side = (off - u * off.dot(u))
        .try_normalize()
        .unwrap_or_else(|| u.any_orthonormal_vector());
    center + u * h + side * ring.sqrt()
}

/// Where two hands meet, at `m + offsets[0]` and `m + offsets[1]`: the `m` nearest `start`, and `last` if given, with
/// each hand within reach (from a shoulder, as far as an arm's length) and outside every sphere (a center and a
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
