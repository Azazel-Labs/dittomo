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
use crate::surface::{Around, Balls, Surface, Toward, on_body, under};
use crate::{Clip, Mapping, Skeleton, Transform};

mod charts;
pub(crate) use charts::Charts;

/// The fixes [`Mapping::apply_with`] makes. All but travel are on by default.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fixes {
    /// Before playing a clip walked in place, give it the travel its planted feet imply ([`travel`]). Off by default.
    pub travel: bool,
    /// While a source foot is planted, keep the target's still, reaching it with two-bone IK.
    pub feet: bool,
    /// Keep the target's bones out of the floor: none comes further under it than the bone it plays comes under the
    /// source's, so a knee, a tail or an ear of an animal lying on its side lies on the ground, however its build differs.
    pub floor: bool,
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
    /// Keep the target's hands where the source's are by its body: by a hip, a thigh, the chest, the other arm, when
    /// playing the pose leaves them nearly there, and on the head, from however far, where both bodies have one: this
    /// then stands in for what `head` does ([`crate::Mapping::apply_touching`] knows the skins; without, the bodies
    /// are guessed from their bones). Off by default: being tried.
    pub place: bool,
    /// Swing the target's bones that nothing drives after the body: hair, tails, ears, capes and skirts
    /// ([`crate::Sway`]), last, kept out of the body and over the floor. Off by default: being tried.
    pub sway: bool,
}

impl Fixes {
    pub const ALL: Self = Self {
        travel: true,
        feet: true,
        floor: true,
        push_out: true,
        touch: true,
        hands: true,
        head: true,
        place: false,
        sway: false,
    };
    pub const NONE: Self = Self {
        travel: false,
        feet: false,
        floor: false,
        push_out: false,
        touch: false,
        hands: false,
        head: false,
        place: false,
        sway: false,
    };
}

/// Every fix but travel: a clip walked in place stays in place, as one made for a game that moves the character
/// itself, its planted feet sliding back with the source's.
impl Default for Fixes {
    fn default() -> Self {
        Self {
            travel: false,
            ..Self::ALL
        }
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
/// above that, with bones below it reaching the floor (or with none, as a paw's last bone at the wrist leaves the paw
/// to the skin below it), on a chain hanging from the spine. The knee and hip are picked from that chain (`split`), so
/// twist bones along it don't count.
#[must_use]
pub fn legs(skeleton: &Skeleton) -> Vec<Limb> {
    legs_of(skeleton, &Shape::light(skeleton)).to_vec()
}

pub(crate) fn legs_of<'a>(skeleton: &Skeleton, s: &'a Shape) -> &'a [Limb] {
    parts(skeleton, s).map_or(&[], |p| &p.legs)
}

/// A skeleton's trunk and the legs hanging from it, found once for each [`Shape`].
pub(crate) struct Parts {
    trunk: Trunk,
    legs: Vec<Limb>,
}

/// The skeleton's trunk and legs: `s` is its shape, which keeps them.
fn parts<'a>(skeleton: &Skeleton, s: &'a Shape) -> Option<&'a Parts> {
    s.parts
        .get_or_init(|| {
            let trunk = trunk(skeleton, s)?;
            let legs = legs_in(skeleton, s, &trunk);
            Some(Parts { trunk, legs })
        })
        .as_ref()
}

fn legs_in(skeleton: &Skeleton, s: &Shape, trunk: &Trunk) -> Vec<Limb> {
    let stands = |b: usize| (s.rest[b].position().y - s.floor) / s.height;
    let parent = |b: usize| skeleton.bones()[b].parent;
    let pos = |b: usize| s.rest[b].position();
    let mut lowest: Vec<f32> = (0..skeleton.len()).map(stands).collect();
    let mut leaf = vec![true; skeleton.len()];
    for b in (0..skeleton.len()).rev() {
        if let Some(p) = parent(b) {
            lowest[p] = lowest[p].min(lowest[b]);
            leaf[p] = false;
        }
    }
    (0..skeleton.len())
        .filter_map(|b| {
            let p = parent(b)?;
            if trunk.on_spine[b] || stands(b) >= 0.15 || stands(p) < 0.15 || (lowest[b] > 0.05 && !leaf[b]) {
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
    pick(chain.len(), |u, l| spans(chain, end, &pos, &spine, (u, l))).map(|(u, l)| (chain[u], chain[l]))
}

/// The three spans of [`split`] for the `u`th and `l`th bones of the chain as its upper and lower, shortest first.
fn spans(
    chain: &[usize],
    end: Vec3,
    pos: &impl Fn(usize) -> Vec3,
    spine: &impl Fn(usize) -> f32,
    (u, l): (usize, usize),
) -> [f32; 3] {
    let (a, b) = (chain[u], chain[l]);
    // An upper whose next bone starts right by it is a stub before the joint, as RobotExpressive's shoulders are: it
    // holds none of the limb's length, so it is the next bone that turns at the shoulder.
    if l > u + 1 && pos(chain[u + 1]).distance(pos(a)) < 0.1 * pos(a).distance(end) {
        return [0.0; 3];
    }
    let mut s = [spine(a), pos(a).distance(pos(b)), pos(b).distance(end)];
    s.sort_by(f32::total_cmp);
    s
}

/// The places in a chain of `n` bones for the upper and the lower whose spans are best, as [`split`] judges them.
fn pick(n: usize, spans: impl Fn(usize, usize) -> [f32; 3]) -> Option<(usize, usize)> {
    (1..n)
        .flat_map(|l| (0..l).map(move |u| (u, l)))
        .max_by(|&(u1, l1), &(u2, l2)| {
            spans(u1, l1)
                .partial_cmp(&spans(u2, l2))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
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
    let highest = |under: &dyn Fn(usize) -> bool| {
        (0..n)
            .filter(|&b| under(b))
            .max_by(|&a, &b| pos(a).y.total_cmp(&pos(b).y))
    };
    let to = |top: usize| {
        let mut spine = vec![top];
        while let Some(p) = parent(spine[spine.len() - 1]) {
            spine.push(p);
        }
        spine.reverse();
        let mut on_spine = vec![false; n];
        for &b in &spine {
            on_spine[b] = true;
        }
        (spine, on_spine)
    };
    let top = highest(&|b| top_of[b] == Some(root))?;
    let (spine, on_spine) = to(top);
    let first = Trunk { under, spine, on_spine };
    // An animal carrying its tail above its lowered head (a cow): the way to the highest bone is the tail, and every
    // leg hangs from where it starts. The way on from there that two legs hang from, the shoulders, is the body, if it
    // reaches nearly as high: a pelvis bone holding the legs beside the spine stands far lower than the head.
    let mut legs = vec![0usize; n];
    for l in legs_in(skeleton, s, &first) {
        let mut b = l.end;
        while !first.on_spine[b] {
            legs[b] += 1;
            let Some(p) = parent(b) else { break };
            b = p;
        }
    }
    let stands = |b: usize| pos(b).y - s.floor;
    let branch = (0..n).find(|&c| legs[c] >= 2 && parent(c).is_some_and(|p| first.on_spine[p]));
    if let Some(c) = branch {
        let mut within = vec![false; n];
        within[c] = true;
        for b in c + 1..n {
            within[b] = parent(b).is_some_and(|p| within[p]);
        }
        if let Some(high) = highest(&|b| within[b])
            && stands(high) >= 0.7 * stands(top)
        {
            let (spine, on_spine) = to(high);
            return Some(Trunk {
                under: first.under,
                spine,
                on_spine,
            });
        }
    }
    Some(first)
}

/// The skeleton's arms, found from its rest pose: each chain off the spine above where the legs branch, down to where
/// it splits (into fingers) or ends. The spine is the path from the biggest root to the highest bone.
#[must_use]
pub fn arms(skeleton: &Skeleton) -> Vec<Limb> {
    body(skeleton, &Shape::light(skeleton))
        .map(|b| b.arms.into_iter().map(|a| a.limb).collect())
        .unwrap_or_default()
}

/// A skeleton's parts, found from its rest pose.
pub(crate) struct Frame {
    /// The spine from the hips to the chest. The chest is where the arms branch, or on a body with none, where the
    /// highest legs do: a four-legged animal's shoulders, its neck and head above them however it carries them. A
    /// skeleton with neither has none.
    pub(crate) spine: Vec<usize>,
    /// By bone, whether it stands on the spine above the chest (the neck and head, up to the highest bone).
    pub(crate) above: Vec<bool>,
    pub(crate) legs: Vec<Limb>,
    pub(crate) arms: Vec<Limb>,
    /// By bone, whether it is a collarbone: on an arm's way out from the chest, before the bone turning at the shoulder.
    pub(crate) collar: Vec<bool>,
}

/// The skeleton's spine, neck and head, legs, arms and collarbones, each found once.
pub(crate) fn frame(skeleton: &Skeleton, s: &Shape) -> Frame {
    let n = skeleton.len();
    let mut frame = Frame {
        spine: Vec::new(),
        above: vec![false; n],
        legs: Vec::new(),
        arms: Vec::new(),
        collar: vec![false; n],
    };
    let Some(Parts { trunk, legs }) = parts(skeleton, s) else {
        return frame;
    };
    frame.legs.clone_from(legs);
    frame.spine = if let Some(body) = body_in(skeleton, s, trunk, &frame.legs) {
        for arm in &body.arms {
            frame.arms.push(arm.limb);
            let mut b = skeleton.bones()[arm.limb.upper].parent;
            while let Some(c) = b.filter(|c| !body.spine.contains(c)) {
                frame.collar[c] = true;
                b = skeleton.bones()[c].parent;
            }
        }
        body.spine
    } else {
        // Each spine bone some leg hangs from, off the spine: the lowest is the hips, the highest the shoulders.
        let mut legged = vec![false; n];
        for l in &frame.legs {
            legged[l.end] = true;
        }
        for b in (0..skeleton.len()).rev() {
            if let Some(p) = skeleton.bones()[b].parent {
                legged[p] |= legged[b];
            }
        }
        let bears = |&b: &usize| s.children[b].iter().any(|&c| !trunk.on_spine[c] && legged[c]);
        // Across from a spine bone, how far out and which way the farthest bone under each of its other children
        // stands.
        let reach = |sb: usize| -> Vec<Vec3> {
            let mut within = vec![false; n];
            s.children[sb]
                .iter()
                .filter(|&&c| !trunk.on_spine[c] && !legged[c])
                .map(|&c| {
                    within.fill(false);
                    within[c] = true;
                    for b in c + 1..n {
                        within[b] = skeleton.bones()[b].parent.is_some_and(|p| within[p]);
                    }
                    (c..n)
                        .filter(|&b| within[b])
                        .map(|b| {
                            let d = s.rest[b].position() - s.rest[sb].position();
                            Vec3::new(d.x, 0.0, d.z)
                        })
                        .max_by(|a, b| a.length().total_cmp(&b.length()))
                        .unwrap_or(Vec3::ZERO)
                })
                .collect()
        };
        // Chains well out across on both sides, as arms.
        let armed = |sb: usize| {
            let r = reach(sb);
            let far = |d: &Vec3| d.length() >= 0.15 * s.height;
            r.iter().any(|a| far(a) && r.iter().any(|b| far(b) && a.dot(*b) < 0.0))
        };
        match (trunk.spine.iter().position(bears), trunk.spine.iter().rposition(bears)) {
            (Some(hips), Some(chest)) if chest > hips => trunk.spine[hips..=chest].to_vec(),
            // A biped whose arms stop short of a hand has none found (Godot's platformer robot, its arms ending at
            // the elbow): its chest is the lowest bone above the hips with chains reaching well out on both sides.
            (Some(_), Some(hips)) => trunk.spine[hips + 1..]
                .iter()
                .position(|&b| armed(b))
                .map_or_else(Vec::new, |i| trunk.spine[hips..=hips + 1 + i].to_vec()),
            _ => Vec::new(),
        }
    };
    if let Some(&chest) = frame.spine.last()
        && let Some(i) = trunk.spine.iter().position(|&b| b == chest)
    {
        for &b in &trunk.spine[i + 1..] {
            frame.above[b] = true;
        }
    }
    frame
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
    tracks(skeleton, clip, &[bone]).pop().unwrap_or_default()
}

/// Where each of these bones is in the world, frame by frame, a list each: the bones are placed together, once a frame.
fn tracks(skeleton: &Skeleton, clip: &Clip, bones: &[usize]) -> Vec<Vec<Vec3>> {
    let path = paths(skeleton, bones.iter().copied());
    let mut world = vec![At::IDENTITY; skeleton.len()];
    let mut at = vec![Vec::with_capacity(clip.poses.len()); bones.len()];
    for p in &clip.poses {
        place(skeleton, p, &path, &mut world);
        for (t, &b) in at.iter_mut().zip(bones) {
            t.push(world[b].position());
        }
    }
    at
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
    traveled(skeleton, &Shape::light(skeleton), clip).unwrap_or_else(|| clip.clone())
}

/// The clip moved up or down so that its feet stand on the floor its rest pose stands on, or nothing if they do as it
/// is. Motion capture often has its floor somewhere else than under its skeleton at rest (the hips at rest at the
/// world's middle, the clip's floor there too): every fix that knows the floor by the rest pose would then be off by
/// the length of a leg, and the target played a leg's length off its own floor. The clip's floor is where its feet
/// are lowest (they may stand on it for a few poses only, then hang from a stool). It is taken for another floor than
/// the rest pose's when over a tenth of the height from it and at the world's own, height nought: a clip that hovers
/// over the floor its rest pose stands on, or sinks under it, is meant to.
pub(crate) fn grounded(skeleton: &Skeleton, s: &Shape, clip: &Clip) -> Option<Clip> {
    let off = floor_over(skeleton, s, clip)?;
    let mut out = clip.clone();
    for pose in &mut out.poses {
        for (b, bone) in skeleton.bones().iter().enumerate() {
            if bone.parent.is_none() {
                pose[b].translation.y -= off;
            }
        }
    }
    Some(out)
}

/// How far over the floor its rest pose stands on a clip's own floor is ([`grounded`]), if it is another.
pub(crate) fn floor_over(skeleton: &Skeleton, s: &Shape, clip: &Clip) -> Option<f32> {
    let legs = legs_of(skeleton, s);
    if legs.is_empty() || clip.poses.is_empty() {
        return None;
    }
    // Each foot and what hangs from it.
    let soles: Vec<usize> = (0..skeleton.len())
        .filter(|&b| legs.iter().any(|l| b == l.end || below(skeleton, b, l.end)))
        .collect();
    let path = paths(skeleton, soles.iter().copied());
    let lowest = |world: &dyn Fn(usize) -> f32| soles.iter().map(|&b| world(b)).fold(f32::INFINITY, f32::min);
    let rest = lowest(&|b| s.rest[b].position().y);
    let mut world = vec![At::IDENTITY; skeleton.len()];
    let low = clip
        .poses
        .iter()
        .map(|p| {
            place(skeleton, p, &path, &mut world);
            lowest(&|b| world[b].position().y)
        })
        .fold(f32::INFINITY, f32::min);
    let off = low - rest;
    (off.abs() > 0.1 * s.height && low.abs() < 0.1 * s.height).then_some(off)
}

/// [`travel`], or nothing if the clip is to be left as it is.
pub(crate) fn traveled(skeleton: &Skeleton, s: &Shape, clip: &Clip) -> Option<Clip> {
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

/// A leg, where its foot goes frame by frame if it moves, and where its hip, knee and foot are as played.
type Going = (Limb, Vec<Option<Vec3>>, [Vec<Vec3>; 3]);

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
    // Each leg with the source bone its foot plants with and its sole, then where they all are, each rig placed once a
    // frame: the foot's track, each sole point's, and (the target's) its hip's and knee's.
    let mut legs = Vec::new();
    for &leg in legs_of(target, t) {
        // A foot left unpaired plants with the first paired bone under it, as a paw held for IK above a toe that is.
        let Some(a) = map.source_of(leg.end).or_else(|| {
            (leg.end + 1..target.len())
                .filter(|&b| below(target, b, leg.end))
                .find_map(|b| map.source_of(b))
        }) else {
            continue;
        };
        legs.push((leg, a, sole(map, [(source, s), (target, t)], &leg, a)));
    }
    // Then each arm whose hand plays one of the source's: a hand down on the floor (crawling, a push-up, getting up)
    // is held there as a foot is. Its height is told from the floor, not from where it is at rest.
    let arms_from = legs.len();
    if let Some(body) = body(target, t) {
        for arm in &body.arms {
            if let Some(a) = map.source_of(arm.limb.end) {
                let wrist = Pair {
                    source: a,
                    target: arm.limb.end,
                    s_rest_y: s.floor,
                    t_rest_y: t.floor,
                    s_floor: s.floor,
                    t_floor: t.floor,
                };
                legs.push((arm.limb, a, vec![wrist]));
            }
        }
    }
    let s_bones: Vec<usize> = legs
        .iter()
        .flat_map(|(_, a, sole)| std::iter::once(*a).chain(sole.iter().map(|p| p.source)))
        .collect();
    let t_bones: Vec<usize> = legs
        .iter()
        .flat_map(|(leg, _, sole)| {
            std::iter::once(leg.end)
                .chain(sole.iter().map(|p| p.target))
                .chain([leg.upper, leg.lower])
        })
        .collect();
    let (mut s_tracks, mut t_tracks) = (
        tracks(source, from, &s_bones).into_iter(),
        tracks(target, out, &t_bones).into_iter(),
    );
    let ys = |t: Vec<Vec3>| -> Vec<f32> { t.iter().map(|v| v.y).collect() };
    let mut feet: Vec<Going> = Vec::new();
    for (i, (leg, a, sole)) in legs.into_iter().enumerate() {
        let s_at = s_tracks.next().unwrap_or_default();
        let s_ys: Vec<Vec<f32>> = s_tracks.by_ref().take(sole.len()).map(ys).collect();
        let t_at = t_tracks.next().unwrap_or_default();
        let t_ys: Vec<Vec<f32>> = t_tracks.by_ref().take(sole.len()).map(ys).collect();
        let (hip, knee) = (t_tracks.next().unwrap_or_default(), t_tracks.next().unwrap_or_default());
        let down = if i < arms_from {
            contacts(&s_at, &from.times, s.rest[a].position().y, s.height)
        } else {
            // A wrist is a little over the floor its palm lies on.
            let mut down = Vec::with_capacity(n);
            detect(&s_at, &from.times, s.floor + WRIST * s.height, s.height, &mut down);
            down
        };
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
        // An arm whose hand never comes down is left out of it all.
        if i >= arms_from && runs.is_empty() {
            continue;
        }
        let mut goals = vec![None; n];
        let locks = locks(&runs, &t_at);
        // How far each run moves the foot from where it is played, at its first frame and at its last.
        let off: Vec<(Vec3, Vec3)> = runs
            .iter()
            .zip(&locks)
            .map(|(&(a, b), &lock)| (lock - t_at[a], lock - t_at[b - 1]))
            .collect();
        for f in 0..n {
            // Held where the run it is in holds it. Off a run, moved as it is played, and as far again as the run
            // nearest moves it at its end nearest, eased out: a foot eased from a hold toward where it is played would
            // lag behind one swinging off, and jerk. Between two runs, the move eases from the one before's to the one
            // after's, so a foot that lifts and plants again within a ramp's length isn't jerked halfway.
            let next = runs.partition_point(|&(a, _)| a <= f);
            let (d, held) = match (next.checked_sub(1), (next < runs.len()).then_some(next)) {
                (Some(p), _) if f < runs[p].1 => (0, locks[p]),
                (Some(p), Some(q)) => {
                    let ((_, end), (start, _)) = (runs[p], runs[q]);
                    let u = (f + 1 - end) as f32 / (start - end + 1) as f32;
                    (
                        (f + 1 - end).min(start - f),
                        t_at[f] + off[p].1.lerp(off[q].0, smooth(u)),
                    )
                }
                (Some(p), None) => (f + 1 - runs[p].1, t_at[f] + off[p].1),
                (None, Some(q)) => (runs[q].0 - f, t_at[f] + off[q].0),
                (None, None) => (usize::MAX, Vec3::ZERO),
            };
            // Never deeper under where it stands at rest than the source's foot comes under its own, scaled: legs
            // bent more or less at rest, played at the source's angles, can reach below the floor.
            let low = floor(&sole, k, |i| s_ys[i][f], |i| t_ys[i][f]);
            let free = Vec3::new(t_at[f].x, t_at[f].y.max(low), t_at[f].z);
            goals[f] = if d > ramp {
                (free.y - t_at[f].y >= 1e-5 * t.height).then_some(free)
            } else {
                let x = 1.0 - d as f32 / (ramp + 1) as f32;
                let w = x * x * (3.0 - 2.0 * x);
                let y = height(&sole, k, |i| s_ys[i][f], |i| t_ys[i][f]);
                Some(free.lerp(Vec3::new(held.x, y, held.z), w))
            };
        }
        feet.push((leg, goals, [hip, knee, t_at]));
    }
    let drop = lower(target, t, out, &feet, ramp);
    let roots: Vec<usize> = (0..target.len())
        .filter(|&b| target.bones()[b].parent.is_none())
        .collect();
    let path = paths(target, feet.iter().map(|(leg, ..)| leg.end));
    for (f, pose) in out.poses.iter_mut().enumerate() {
        let dropped = drop[f] > 0.0;
        if drop[f] != 0.0 {
            for &r in &roots {
                pose[r].translation.y -= drop[f];
            }
        }
        let mut placed = false;
        for (leg, goals, [.., t_at]) in &feet {
            // A foot the body was lowered over is kept where it was going.
            let Some(goal) = goals[f].or(dropped.then_some(t_at[f])) else {
                continue;
            };
            // Legs hang apart, so reaching one leaves the other placed as it was.
            if !placed {
                place(target, pose, &path, &mut world);
                placed = true;
            }
            reach_in(target, pose, &world, leg, goal);
        }
    }
}

/// Where to hold a target foot, played at `at`, over each of `runs`: where it is on average. A run split where the
/// source's foot slips, the next starting at once, holds the foot where the one before did but for how far it slips as
/// played, and the runs so chained are held where the foot is on average over them all: each held at its own average,
/// a foot drifting as it is played would jump from one to the next by far more than it slips.
fn locks(runs: &[(usize, usize)], at: &[Vec3]) -> Vec<Vec3> {
    let mut locks = Vec::with_capacity(runs.len());
    let mut i = 0;
    while i < runs.len() {
        let mut j = i + 1;
        while j < runs.len() && runs[j].0 == runs[j - 1].1 {
            j += 1;
        }
        let mut slips = vec![Vec3::ZERO; j - i];
        for q in i + 1..j {
            let f = runs[q].0;
            slips[q - i] = slips[q - i - 1] + at[f] - at[f - 1];
        }
        let (a, b) = (runs[i].0, runs[j - 1].1);
        let mean = runs[i..j]
            .iter()
            .zip(&slips)
            .map(|(&(a, b), &slip)| at[a..b].iter().map(|&p| p - slip).sum::<Vec3>())
            .sum::<Vec3>()
            / (b - a) as f32;
        locks.extend(slips.iter().map(|&slip| mean + slip));
        i = j;
    }
    locks
}

/// How far to lower the target's body, frame by frame, so that each leg reaches where its foot goes ([`plant`]) bent
/// at least as much as it was played, or as a leg standing nearly straight is: a planted foot held further off than the
/// leg reaches would snap the knee straight. Never less than that at any frame, eased over `ramp` frames either side, so
/// the body sinks and rises smoothly; but never more than a tenth of the leg, nor so far that a bone but the feet (and a
/// root standing on the floor at rest) comes under the target's floor, as a belly, a knee or a tail near it would.
/// Where no leg needs it lower, raised as far as [`raise`] lets it instead, given as a drop below nothing.
fn lower(target: &Skeleton, t: &Shape, played: &Clip, feet: &[Going], ramp: usize) -> Vec<f32> {
    let n = played.poses.len();
    let mut need = vec![0.0f32; n];
    let mut lift = vec![(0.0f32, f32::INFINITY); n];
    for (_, goals, [hip, knee, foot]) in feet {
        if goals.iter().all(Option::is_none) {
            continue;
        }
        for f in 0..n {
            let Some(goal) = goals[f] else { continue };
            need[f] = need[f].max(sink([hip[f], knee[f], foot[f]], goal));
            let (want, room) = rise([hip[f], knee[f], foot[f]], goal);
            lift[f] = (lift[f].0.max(want), lift[f].1.min(room));
        }
    }
    let lift: Vec<f32> = lift
        .iter()
        .map(|&(want, room)| if room.is_finite() { want.min(room) } else { room })
        .collect();
    let lift = raise(&need, &lift, ramp);
    if need.iter().all(|&y| y <= 0.0) {
        return lift.iter().map(|&y| -y).collect();
    }
    // How far each frame's body is over the floor, knees and all, but the feet and any bone standing on it at rest (a
    // root, say); only near frames that need lowering.
    let body: Vec<usize> = (0..target.len())
        .filter(|&b| {
            !feet.iter().any(|(leg, ..)| b == leg.end || below(target, b, leg.end))
                && t.rest[b].position().y - t.floor >= 0.01 * t.height
        })
        .collect();
    let path = paths(target, body.iter().copied());
    let mut world = vec![At::IDENTITY; target.len()];
    let room: Vec<f32> = (0..n)
        .map(|f| {
            if need[f.saturating_sub(4 * ramp)..(f + 4 * ramp + 1).min(n)]
                .iter()
                .all(|&y| y <= 0.0)
            {
                return f32::INFINITY;
            }
            place(target, &played.poses[f], &path, &mut world);
            body.iter()
                .map(|&b| world[b].position().y - t.floor)
                .fold(f32::INFINITY, f32::min)
                .max(0.0)
        })
        .collect();
    // The most needed within twice `ramp` frames, then averaged twice over `ramp` either side: each average takes in
    // only frames that saw this frame's need, so it never comes out less. The room the same way, the least, so it never
    // comes out more.
    let spread = |v: &[f32], most: bool| -> Vec<f32> {
        let pick = if most { f32::max } else { f32::min };
        let start = if most { 0.0 } else { f32::INFINITY };
        let far: Vec<f32> = (0..n)
            .map(|f| {
                v[f.saturating_sub(2 * ramp)..(f + 2 * ramp + 1).min(n)]
                    .iter()
                    .copied()
                    .fold(start, pick)
            })
            .collect();
        let average = |v: &[f32]| -> Vec<f32> {
            (0..n)
                .map(|f| {
                    let w = &v[f.saturating_sub(ramp)..(f + ramp + 1).min(n)];
                    w.iter().sum::<f32>() / w.len() as f32
                })
                .collect()
        };
        average(&average(&far))
    };
    let (need, room) = (spread(&need, true), spread(&room, false));
    need.iter()
        .zip(&room)
        .zip(&lift)
        .map(|((&y, &r), &l)| y.min(r) - l)
        .collect()
}

/// How far to raise the target's body, frame by frame, given how far each frame's planted feet would each let it be
/// raised (`lift`, unbounded with none planted), where it needn't be lowered (`need`): the least within twice `ramp`
/// frames, then averaged twice over `ramp` either side, so it never comes out more than every planted foot lets it.
/// Not at all where a long stretch has no foot planted.
fn raise(need: &[f32], lift: &[f32], ramp: usize) -> Vec<f32> {
    let n = lift.len();
    let lift: Vec<f32> = lift
        .iter()
        .zip(need)
        .map(|(&l, &y)| if y > 0.0 { 0.0 } else { l })
        .collect();
    let ramp = (ramp / 2).max(1);
    let least: Vec<f32> = (0..n)
        .map(|f| {
            let l = lift[f.saturating_sub(2 * ramp)..(f + 2 * ramp + 1).min(n)]
                .iter()
                .copied()
                .fold(f32::INFINITY, f32::min);
            if l.is_finite() { l } else { 0.0 }
        })
        .collect();
    let average = |v: &[f32]| -> Vec<f32> {
        (0..n)
            .map(|f| {
                let w = &v[f.saturating_sub(ramp)..(f + ramp + 1).min(n)];
                w.iter().sum::<f32>() / w.len() as f32
            })
            .collect()
    };
    average(&average(&least))
}

/// How long, in seconds, a foot takes to settle where it is held once it plants, and to let go once it lifts.
const SETTLE: f32 = 0.12;

/// How long, in seconds, the lowest foot holds still before the poses so far take it to be on the floor.
const STAND: f32 = 0.15;

/// The smooth step from 0 to 1 as `x` goes from 0 to 1.
fn smooth(x: f32) -> f32 {
    let x = x.clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// How far to lower a leg's hip, its hip, knee and foot at `leg`, for the foot to reach `goal` bent at least as much as
/// it is, or as a leg standing nearly straight is ([`lower`]); not at all if it doesn't need to.
fn sink(leg: [Vec3; 3], goal: Vec3) -> f32 {
    let [hip, knee, foot] = leg;
    let long = hip.distance(knee) + knee.distance(foot);
    if long <= 0.0 {
        return 0.0;
    }
    let reach = long * (hip.distance(foot) / long).clamp(0.98, 0.999);
    // Lowered by `y`, the hip is `reach` from the goal: |v + y Y| = reach, the smaller root. A goal further across than
    // the leg reaches can't be reached by lowering, and the body is lowered a tenth of the leg at most, so a foot held
    // far off isn't dragged down to.
    let v = goal - hip;
    let across = v.x * v.x + v.z * v.z;
    if across >= reach * reach {
        return 0.0;
    }
    (-v.y - (reach * reach - across).sqrt()).clamp(0.0, 0.1 * long)
}

/// How far a leg's hip, its hip, knee and foot at `leg`, wants raising, and may be raised, for the foot to reach
/// `goal`. A foot played under its goal (the floor) wants the body raised for it to reach there bent as much as it is,
/// rather than the knee bent further, as a leg straightening from further bent at rest than the source's goes; one
/// played over it wants nothing. Either lets the body rise as far as it can still reach, a little short of where
/// [`sink`] lets a leg straighten, so a foot held at the edge of its reach as the body moves on doesn't slide. A tenth
/// of the leg at most.
fn rise(leg: [Vec3; 3], goal: Vec3) -> (f32, f32) {
    let [hip, knee, foot] = leg;
    let long = hip.distance(knee) + knee.distance(foot);
    if long <= 0.0 {
        return (0.0, 0.0);
    }
    let played = hip.distance(foot);
    // Raised by `y`, the hip is `reach` from the goal: |v - y Y| = reach, the root with the hip above the goal.
    let v = goal - hip;
    let across = v.x * v.x + v.z * v.z;
    let up = |reach: f32| {
        let reach = reach.min(0.999 * long);
        if across >= reach * reach {
            0.0
        } else {
            (v.y + (reach * reach - across).sqrt()).clamp(0.0, 0.1 * long)
        }
    };
    (
        if goal.y > foot.y { up(played) } else { 0.0 },
        up(played.max(0.97 * long)),
    )
}

/// The body lowered over feet held further off than the legs reach, as [`lower`] lowers it over a whole clip, but as
/// the poses come: as far as the legs need at once, then eased back up over [`SETTLE`] once they need less; never so
/// far that a bone but the feet (or a root standing on the floor at rest) comes under the floor. Or raised for feet
/// played under the floor ([`rise`]), eased toward that, and never past where every planted leg still reaches.
#[derive(Clone)]
pub(crate) struct Sink {
    roots: Vec<usize>,
    /// The bones kept over the floor, and the path placing them.
    body: Vec<usize>,
    path: Vec<usize>,
    floor: f32,
    /// How far the body was lowered the pose before.
    drop: f32,
}

impl Sink {
    fn new<'a>(target: &Skeleton, t: &Shape, legs: impl Iterator<Item = &'a Limb> + Clone) -> Self {
        let body: Vec<usize> = (0..target.len())
            .filter(|&b| {
                !legs.clone().any(|leg| b == leg.end || below(target, b, leg.end))
                    && t.rest[b].position().y - t.floor >= 0.01 * t.height
            })
            .collect();
        Self {
            roots: (0..target.len())
                .filter(|&b| target.bones()[b].parent.is_none())
                .collect(),
            path: paths(target, body.iter().copied()),
            body,
            floor: t.floor,
            drop: 0.0,
        }
    }

    fn reset(&mut self) {
        self.drop = 0.0;
    }

    /// Lowers `played`, placed in `tw` as far as each leg's hip, knee and foot, `dt` seconds after the pose before, for
    /// each leg to reach its goal, if it has one, or raises it for the planted legs to keep the bend they're played
    /// with ([`rise`]), eased toward that; then places it again in `tw` along `t_path`. Whether it was lowered.
    fn lower<'a>(
        &mut self,
        target: &Skeleton,
        dt: f32,
        played: &mut [Transform],
        tw: &mut [At],
        t_path: &[usize],
        goals: impl Iterator<Item = (&'a Limb, Option<Vec3>)> + Clone,
    ) -> bool {
        let at = |leg: &Limb| [leg.upper, leg.lower, leg.end].map(|b| tw[b].position());
        let need = goals
            .clone()
            .filter_map(|(leg, goal)| Some(sink(at(leg), goal?)))
            .fold(0.0, f32::max);
        let (want, room) = goals
            .filter_map(|(leg, goal)| Some(rise(at(leg), goal?)))
            .fold((0.0f32, f32::INFINITY), |(w, r), (want, room)| {
                (w.max(want), r.min(room))
            });
        let lift = if need > 0.0 { 0.0 } else { want.min(room) };
        let keep = if dt > 0.0 { (-dt / SETTLE).exp() } else { 1.0 };
        let eased = self.drop * keep - lift * (1.0 - keep);
        let mut drop = if need > 0.0 { need.max(eased) } else { eased.max(-lift) };
        if drop.abs() < 1e-6 {
            self.drop = 0.0;
            return false;
        }
        if drop > 0.0 {
            place(target, played, &self.path, tw);
            let room = self
                .body
                .iter()
                .map(|&b| tw[b].position().y - self.floor)
                .fold(f32::INFINITY, f32::min);
            drop = drop.min(room.max(0.0));
        }
        self.drop = drop;
        if drop == 0.0 {
            return false;
        }
        for &r in &self.roots {
            played[r].translation.y -= drop;
        }
        place(target, played, t_path, tw);
        drop > 0.0
    }
}

/// A target foot, or a mapped bone under it, and the source bone it plays, with how high each stands at rest and how
/// high its rig's floor is (where its lowest bone stands at rest).
#[derive(Clone, Copy)]
struct Pair {
    source: usize,
    target: usize,
    s_rest_y: f32,
    t_rest_y: f32,
    s_floor: f32,
    t_floor: f32,
}

impl Pair {
    /// How low the target bone may come with the source's at `s_y`: no further under the target's floor than the
    /// source's comes under its own, scaled by `k`. A paw standing nearly on the floor at rest may play a hoof standing
    /// well above its own, as the Quaternius horse's do over its root, that comes down onto it as the clip plays.
    fn lowest(&self, k: f32, s_y: f32) -> f32 {
        self.t_floor - ((self.s_floor - s_y) * k).max(0.0)
    }
}

/// The target leg's foot and the mapped bones under it whose source is under the source's foot `a`, foot first.
fn sole(map: &Mapping, [(source, s), (target, t)]: [(&Skeleton, &Shape); 2], leg: &Limb, a: usize) -> Vec<Pair> {
    let pair = |a: usize, b: usize| Pair {
        source: a,
        target: b,
        s_rest_y: s.rest[a].position().y,
        t_rest_y: t.rest[b].position().y,
        s_floor: s.floor,
        t_floor: t.floor,
    };
    let toes = (leg.end + 1..target.len()).filter_map(|b| {
        let a2 = map.source_of(b)?;
        (below(target, b, leg.end) && below(source, a2, a)).then(|| pair(a2, b))
    });
    std::iter::once(pair(a, leg.end)).chain(toes).collect()
}

/// How high to bring a target foot ([`sole`]): as far from where it stands at rest as the source's has come from its
/// own, scaled, or higher if one of the bones under it would otherwise come lower from its rest than the source's does.
/// Ankles stand at different heights over the toes on different rigs, so a foot rolled onto its toes needs its ankle
/// lifted by more or less than the source's to keep the toes on the floor. `s_y` and `t_y` give each pair's height in
/// the source's pose and in the target's as played; the target's foot keeps its turn as the leg reaches. None comes
/// further under the floor than its source does ([`Pair::lowest`]).
fn height(sole: &[Pair], k: f32, s_y: impl Fn(usize) -> f32, t_y: impl Fn(usize) -> f32) -> f32 {
    let ankle = t_y(0);
    sole.iter()
        .enumerate()
        .map(|(i, p)| {
            let s_y = s_y(i);
            (p.t_rest_y + (s_y - p.s_rest_y) * k).max(p.lowest(k, s_y)) - (t_y(i) - ankle)
        })
        .fold(f32::MIN, f32::max)
}

/// How low a target foot ([`sole`]) may come: as far under where it stands at rest as the source's has come under its
/// own, scaled, and no farther, for each of the bones under it too, nor further under the floor than its source does.
fn floor(sole: &[Pair], k: f32, s_y: impl Fn(usize) -> f32, t_y: impl Fn(usize) -> f32) -> f32 {
    let ankle = t_y(0);
    sole.iter()
        .enumerate()
        .map(|(i, p)| {
            let s_y = s_y(i);
            (p.t_rest_y + ((s_y - p.s_rest_y) * k).min(0.0)).max(p.lowest(k, s_y)) - (t_y(i) - ankle)
        })
        .fold(f32::MIN, f32::max)
}

/// Whether `b` is under `top`.
fn below(skeleton: &Skeleton, mut b: usize, top: usize) -> bool {
    while let Some(p) = skeleton.bones()[b].parent {
        if p == top {
            return true;
        }
        b = p;
    }
    false
}

/// Where each of the source's feet comes down to, worked out as the poses come from those so far. Feet that walk on
/// tiptoe never come down to where they stand at rest: once one foot is well above where the lowest came down to, that
/// is the floor, till a foot comes down well below it; and once a foot is well above where it came down to, that is its
/// own ground, as feet needn't stand equally high. Till then a foot's ground is the lowest it has been, or where it
/// stands at rest if lower, so a foot coming down from its first step isn't taken to be on the floor; but once the
/// lowest foot has held still for [`STAND`], the floor is known: a clip can stand its body higher than its rest pose
/// does, and its feet would never be taken to be down.
#[derive(Clone)]
pub(crate) struct Ground {
    /// Each foot's height at rest.
    rest: Vec<f32>,
    ground: Vec<f32>,
    /// The lowest each foot has been, and whether it has since been well above it.
    low: Vec<(f32, bool)>,
    /// How high each foot was the pose before.
    last: Vec<f32>,
    /// Where each foot was when it last moved, and for how long it has held still near there since.
    held: Vec<(Vec3, f32)>,
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
            last: vec![f32::INFINITY; n],
            held: vec![(Vec3::INFINITY, 0.0); n],
            floor: f32::INFINITY,
            known: false,
            height,
        }
    }

    /// Forgets the poses so far, as after a cut.
    fn reset(&mut self) {
        self.ground.clone_from(&self.rest);
        self.low.fill((f32::INFINITY, false));
        self.last.fill(f32::INFINITY);
        self.held.fill((Vec3::INFINITY, 0.0));
        (self.floor, self.known) = (f32::INFINITY, false);
    }

    /// Takes in where foot `l` is in the next pose, as `at(l)`, for every foot, `dt` seconds after the pose before.
    fn see(&mut self, dt: f32, at: impl Fn(usize) -> Vec3) {
        let h = self.height;
        let (mut bottom, mut top) = (f32::INFINITY, f32::NEG_INFINITY);
        for l in 0..self.rest.len() {
            let y = at(l).y;
            (bottom, top) = (bottom.min(y), top.max(y));
        }
        // The poses begin with the lowest foot standing, unless it is over a tenth of the height above where it stands
        // at rest: a clip can stand its feet higher than the rest pose does, and they would not plant until the floor
        // is known.
        if self.floor == f32::INFINITY {
            for (ground, &rest) in self.ground.iter_mut().zip(&self.rest) {
                if bottom > rest && bottom - rest < 0.1 * h {
                    *ground = bottom;
                }
            }
        }
        if bottom < self.floor {
            self.known &= self.floor - bottom < 0.01 * h;
            self.floor = bottom;
        }
        self.known |= top - self.floor > 0.04 * h;
        for l in 0..self.rest.len() {
            let p = at(l);
            let (from, still) = &mut self.held[l];
            if p.distance(*from) < 0.01 * h {
                *still += dt.max(0.0);
            } else {
                (*from, *still) = (p, 0.0);
            }
            self.known |= *still >= STAND && p.y - bottom < 0.01 * h;
        }
        for l in 0..self.rest.len() {
            let y = at(l).y;
            let (own, lifted) = &mut self.low[l];
            if y < *own {
                *lifted &= *own - y < 0.01 * h;
                *own = y;
            }
            // Once the floor is known, a foot's own ground isn't where it was as the poses began if that is over a
            // tenth of the height above the floor: a clip can start with a foot swinging, and it would plant in the
            // air.
            let near = !self.known || *own - self.floor < 0.1 * h;
            *lifted |= near && y - *own > 0.04 * h;
            // A foot that stops coming down well above the floor, but within a tenth of the height of it, has come down
            // to its own ground: one foot of a run can plant higher than the other, and would slide its first step.
            *lifted |= self.known && near && y >= self.last[l] && *own - self.floor > 0.04 * h;
            self.last[l] = y;
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
/// as far as the source's has since (scaled), at the height the source's stands scaled ([`height`]), and let go over
/// [`SETTLE`]. A foot coming down to the floor is brought to that height as it nears it, so it lands without a jump.
pub(crate) struct Feet {
    feet: Vec<Foot>,
    /// Each foot's [`sole`].
    soles: Vec<Vec<Pair>>,
    ground: Ground,
    s_path: Vec<usize>,
    t_path: Vec<usize>,
    /// The target's height over the source's.
    k: f32,
    s_height: f32,
    sink: Sink,
    /// Where each foot is this pose before it is moved, and where it goes, if it is moved.
    goals: Vec<(Vec3, Option<Vec3>)>,
}

#[derive(Clone, Copy)]
struct Foot {
    leg: Limb,
    source: usize,
    s_rest_y: f32,
    last: Option<Vec3>,
    /// The source foot's speed across the floor.
    speed: f32,
    down: bool,
    /// Where the target's foot is held, if it is or is letting go.
    lock: Option<Vec3>,
    /// How far the hold moved the foot from where it was played, the last pose it was held.
    off: Vec3,
    /// Where the target's foot was shown the pose before.
    shown: Option<Vec3>,
    /// Seconds since the foot planted or lifted.
    since: f32,
}

impl Feet {
    /// Nothing if no target leg's foot plays a source bone.
    pub(crate) fn new(map: &Mapping, [(source, s), (target, t)]: [(&Skeleton, &Shape); 2]) -> Option<Self> {
        let feet: Vec<Foot> = legs_of(target, t)
            .iter()
            .copied()
            .filter_map(|leg| {
                let a = map.source_of(leg.end)?;
                Some(Foot {
                    leg,
                    source: a,
                    s_rest_y: s.rest[a].position().y,
                    last: None,
                    speed: 0.0,
                    down: false,
                    lock: None,
                    off: Vec3::ZERO,
                    shown: None,
                    since: 0.0,
                })
            })
            .collect();
        let soles: Vec<Vec<Pair>> = feet
            .iter()
            .map(|f| sole(map, [(source, s), (target, t)], &f.leg, f.source))
            .collect();
        (!feet.is_empty()).then(|| Self {
            ground: Ground::new(feet.iter().map(|f| f.s_rest_y).collect(), s.height),
            sink: Sink::new(target, t, feet.iter().map(|f| &f.leg)),
            goals: vec![(Vec3::ZERO, None); feet.len()],
            s_path: paths(source, soles.iter().flatten().map(|p| p.source)),
            t_path: paths(target, soles.iter().flatten().map(|p| p.target)),
            feet,
            soles,
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
                off: Vec3::ZERO,
                shown: None,
                since: 0.0,
                ..*f
            };
        }
        self.ground.reset();
        self.sink.reset();
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
        self.ground.see(dt, |l| sw[self.feet[l].source].position());
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
            // Held across the floor at once, as the foot is where it planted, and let go over SETTLE: moved as it is
            // played, and as far again as the hold last moved it, eased out, as a foot eased toward where it is played
            // would lag behind one swinging off. Its height is the source's scaled while planted, and on the way down
            // to the floor from twice the height a foot plants at, so it doesn't jump on landing.
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
            self.goals[l] = (free, None);
            if up <= 0.0 {
                f.shown = Some(free);
                continue;
            }
            let lock = if f.down {
                let lock = *f.lock.get_or_insert(free);
                f.off = lock - free;
                lock
            } else {
                free + f.off
            };
            let sole = &self.soles[l];
            let y = height(
                sole,
                self.k,
                |i| sw[sole[i].source].position().y,
                |i| tw[sole[i].target].position().y,
            );
            let goal = Vec3::new(
                free.x + (lock.x - free.x) * ease,
                free.y + (y - free.y) * up,
                free.z + (lock.z - free.z) * ease,
            );
            self.goals[l].1 = Some(goal);
            f.shown = Some(goal);
        }
        let legs = self.feet.iter().map(|f| &f.leg);
        reach_all(target, played, tw, &self.t_path, &mut self.sink, dt, legs, &self.goals);
    }
}

/// Puts the foot of each of `legs` where it goes in `goals` (where it is, and where it goes if it is moved), `played`
/// placed in `tw` along `t_path`, the body first lowered by `sink` if the legs need it, `dt` seconds after the pose
/// before. A foot not moved over a body lowered is kept where it was.
#[allow(clippy::too_many_arguments)]
fn reach_all<'a>(
    target: &Skeleton,
    played: &mut [Transform],
    tw: &mut [At],
    t_path: &[usize],
    sink: &mut Sink,
    dt: f32,
    legs: impl Iterator<Item = &'a Limb> + Clone,
    goals: &[(Vec3, Option<Vec3>)],
) {
    let lowered = sink.lower(
        target,
        dt,
        played,
        tw,
        t_path,
        legs.clone().zip(goals.iter().map(|g| g.1)),
    );
    for (leg, &(free, goal)) in legs.zip(goals) {
        let Some(goal) = goal.or(lowered.then_some(free)) else {
            continue;
        };
        // Legs hang apart, so reaching one leaves the other placed as it was.
        reach_in(target, played, tw, leg, goal);
    }
}

/// Hands kept still while the source's are down on the floor (crawling, a push-up, getting up), worked out as the poses
/// come from those so far, as [`Feet`] keeps feet. A foot's ground is learned from the lowest it has been ([`Ground`]),
/// but a hand hanging still by the hip is on nothing, so a hand is down only on the floor the source's rest pose stands
/// on: once its wrist is within a few hundredths of the height of it and nearly still, till it moves off or rises a
/// little higher. The target's is held where it was the pose before the source's came down, moved on as far as the
/// source's has since (scaled), as high over its own floor as the source's is over its (scaled), and let go over
/// [`SETTLE`]. A hand coming down to the floor is brought to that height as it nears it, so it lands without a jump.
/// An arm whose hand is well off the floor is left as it is played, and costs next to nothing.
pub(crate) struct HandsDown {
    arms: Vec<Palm>,
    s_path: Vec<usize>,
    t_path: Vec<usize>,
    /// The target's height over the source's.
    k: f32,
    s_height: f32,
    /// How high the source's floor is, and the target's.
    floors: [f32; 2],
}

/// An arm whose hand plays one of the source's, and what [`HandsDown`] keeps of it from pose to pose.
#[derive(Clone, Copy)]
struct Palm {
    arm: Limb,
    source: usize,
    last: Option<Vec3>,
    /// The source hand's speed across the floor.
    speed: f32,
    down: bool,
    /// Where the target's hand is held, if it is or is letting go.
    lock: Option<Vec3>,
    /// How far the hold moved the hand from where it was played, the last pose it was held.
    off: Vec3,
    /// Where the target's hand was shown the pose before, if it was near the floor.
    shown: Option<Vec3>,
    /// Seconds since the hand came down or lifted.
    since: f32,
}

impl HandsDown {
    /// Nothing if no target arm's hand plays a source bone.
    pub(crate) fn new(map: &Mapping, [(source, s), (target, t)]: [(&Skeleton, &Shape); 2]) -> Option<Self> {
        let arms: Vec<Palm> = body(target, t)?
            .arms
            .iter()
            .filter_map(|a| {
                Some(Palm {
                    arm: a.limb,
                    source: map.source_of(a.limb.end)?,
                    last: None,
                    speed: 0.0,
                    down: false,
                    lock: None,
                    off: Vec3::ZERO,
                    shown: None,
                    since: 0.0,
                })
            })
            .collect();
        (!arms.is_empty()).then(|| Self {
            s_path: paths(source, arms.iter().map(|a| a.source)),
            t_path: paths(target, arms.iter().map(|a| a.arm.end)),
            arms,
            k: t.height / s.height,
            s_height: s.height,
            floors: [s.floor, t.floor],
        })
    }

    /// Forgets every contact, as after a cut.
    pub(crate) fn reset(&mut self) {
        for a in &mut self.arms {
            *a = Palm {
                last: None,
                speed: 0.0,
                down: false,
                lock: None,
                off: Vec3::ZERO,
                shown: None,
                since: 0.0,
                ..*a
            };
        }
    }

    /// The source bones to place ([`place_source`]) before each pose.
    pub(crate) fn s_path(&self) -> &[usize] {
        &self.s_path
    }

    /// Fixes `played`, the target playing a pose of the source placed in `scratch` ([`Self::s_path`]), `dt` seconds
    /// after the pose before.
    pub(crate) fn pose(&mut self, target: &Skeleton, dt: f32, played: &mut [Transform], scratch: &mut Scratch) {
        let h = self.s_height;
        let [s_floor, t_floor] = self.floors;
        let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
        let Scratch { sw, tw, .. } = scratch;
        let mut placed = false;
        for a in &mut self.arms {
            let at = sw[a.source].position();
            let step = a.last.map_or(Vec3::ZERO, |last| flat(at - last));
            if dt > 0.0 && a.last.is_some() {
                a.speed = step.length() / dt;
            }
            a.last = Some(at);
            let low = at.y - (s_floor + WRIST * h);
            a.since += dt;
            if a.down {
                if low > 0.06 * h || a.speed > HAND_SLOW * h {
                    (a.down, a.since) = (false, 0.0);
                }
            } else if low < 0.04 * h && a.speed < HAND_SLOW * h {
                // Held where it was shown the pose before, moved on as the source's hand has since, as a foot is.
                (a.down, a.since) = (true, 0.0);
                a.lock = a.shown.map(|p| p + step * self.k);
            }
            let ease = match (a.down, a.lock) {
                (true, _) => 1.0,
                (false, Some(_)) if a.since < SETTLE => smooth(1.0 - a.since / SETTLE),
                _ => {
                    a.lock = None;
                    0.0
                }
            };
            // Its height is the source's scaled while down, and on the way down to the floor from twice the height a
            // hand is down at, as a foot's is.
            let up = ease.max(smooth((0.08 * h - low) / (0.04 * h)));
            if up <= 0.0 {
                a.shown = None;
                continue;
            }
            // Arms hang apart, so reaching one leaves the other placed as it was.
            if !placed {
                place(target, played, &self.t_path, tw);
                placed = true;
            }
            let free = tw[a.arm.end].position();
            let lock = if a.down {
                let lock = *a.lock.get_or_insert(free);
                a.off = lock - free;
                lock
            } else {
                free + a.off
            };
            let y = t_floor + ((at.y - s_floor) * self.k).max(0.0);
            let goal = Vec3::new(
                free.x + (lock.x - free.x) * ease,
                free.y + (y - free.y) * up,
                free.z + (lock.z - free.z) * ease,
            );
            reach_in(target, played, tw, &a.arm, goal);
            a.shown = Some(goal);
        }
    }
}

/// How high over the floor, in heights, the wrist of a hand lying on it is taken to be: more than a palm is thick, as
/// motion capture has a crawling hand's wrist well over the floor its feet find.
const WRIST: f32 = 0.05;

/// How fast, in heights a second, a hand near the floor may move across it and still be down.
const HAND_SLOW: f32 = 0.25;

/// The pose a contact ended at, and how far it moved the foot from where it was played the pose before.
type Ended = (usize, Vec3);

/// Feet kept still while the source's are planted, as [`plant`] keeps them over a whole clip, but from the poses so
/// far and a few ahead: each pose is fixed once the `delay` poses after it have come. Whether a foot is planted is
/// judged as over a whole clip, over the poses kept; it is held at the average of where it is over the part of its
/// contact in sight when that contact begins, and eased toward it from as many poses before as [`plant`] would, or as
/// many as are in sight; off a contact it is moved as [`plant`] moves it, as played and as far again as the contact
/// nearest moves it at its end nearest, eased out, and between two close together, that easing from one to the next.
pub(crate) struct Ahead {
    feet: Vec<Foot>,
    soles: Vec<Vec<Pair>>,
    ground: Ground,
    t_path: Vec<usize>,
    k: f32,
    s_height: f32,
    delay: usize,
    /// For each pose kept (the `delay` ahead of the one fixed, it and the two before it), by its number modulo how
    /// many are kept: when it came, and where each foot is in the source and in the target before any fix.
    /// and how high the target's foot is to be brought ([`height`]).
    times: Vec<f32>,
    s_at: Vec<Vec3>,
    t_at: Vec<Vec3>,
    t_y: Vec<f32>,
    /// Each foot's contact the poses fixed are in or have just left: where it is held, and when it ended ([`Ended`]).
    held: Vec<Option<(Vec3, Option<Ended>)>>,
    seen: usize,
    clock: f32,
    w_at: Vec<Vec3>,
    w_times: Vec<f32>,
    w_down: Vec<bool>,
    sink: Sink,
    goals: Vec<(Vec3, Option<Vec3>)>,
}

impl Ahead {
    /// [`Feet`]'s feet, fixed `delay` poses late.
    pub(crate) fn new(feet: &Feet, delay: usize) -> Self {
        let kept = delay + 3;
        let n = feet.feet.len();
        let mut ahead = Self {
            feet: feet.feet.clone(),
            soles: feet.soles.clone(),
            ground: feet.ground.clone(),
            t_path: feet.t_path.clone(),
            k: feet.k,
            s_height: feet.s_height,
            delay,
            times: vec![0.0; kept],
            s_at: vec![Vec3::ZERO; kept * n],
            t_at: vec![Vec3::ZERO; kept * n],
            t_y: vec![0.0; kept * n],
            held: vec![None; n],
            seen: 0,
            clock: 0.0,
            w_at: Vec::with_capacity(kept),
            w_times: Vec::with_capacity(kept),
            w_down: Vec::with_capacity(kept),
            sink: feet.sink.clone(),
            goals: vec![(Vec3::ZERO, None); n],
        };
        ahead.reset();
        ahead
    }

    /// Forgets every contact, as after a cut.
    pub(crate) fn reset(&mut self) {
        self.ground.reset();
        self.held.fill(None);
        (self.seen, self.clock) = (0, 0.0);
        self.sink.reset();
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
        self.ground.see(dt, |l| sw[self.feet[l].source].position());
        for (i, f) in self.feet.iter().enumerate() {
            let at = sw[f.source].position();
            self.s_at[slot * n + i] = at;
            self.t_at[slot * n + i] = tw[f.leg.end].position();
            let sole = &self.soles[i];
            self.t_y[slot * n + i] = height(
                sole,
                self.k,
                |j| sw[sole[j].source].position().y,
                |j| tw[sole[j].target].position().y,
            );
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
        place(target, played, &self.t_path, tw);
        for (i, f) in self.feet.iter().enumerate() {
            let free = tw[f.leg.end].position();
            self.goals[i] = (free, None);
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
            let near = if down[at_o] {
                // A contact split where the source's foot slips holds the foot where the one before did but for how
                // far it slips as played, as `plant` holds it ([`locks`]).
                let lock = match self.held[i] {
                    Some((lock, None)) if !apart(at_o) => lock,
                    Some((lock, None)) => lock + t_at[slot(o) * n + i] - t_at[slot(o - 1) * n + i],
                    _ => lock_from(at_o),
                };
                self.held[i] = Some((lock, None));
                Some((0, lock))
            } else {
                if let Some((lock, None)) = self.held[i] {
                    let off = lock - t_at[slot(o - 1) * n + i];
                    self.held[i] = Some((lock, Some((o, off))));
                }
                let before = match self.held[i] {
                    Some((_, Some(end))) => Some(end),
                    _ => None,
                };
                let after = (at_o + 1..down.len())
                    .find(|&w| down[w])
                    .map(|j| (first + j, lock_from(j) - t_at[slot(first + j) * n + i]));
                // Between two contacts, moved as far as one easing from how far the one before moved it to how far
                // the one after does, from when that comes in sight if the foot was still letting go of the one
                // before then, as `plant` does, so a quick step never jerks between them.
                match (before, after) {
                    (Some((end, a)), Some((start, b))) => {
                        let from = end.max(start.saturating_sub(self.delay));
                        let u = if from + 1 - end > ramp {
                            1.0
                        } else {
                            (o + 1 - from) as f32 / (start - from + 1) as f32
                        };
                        Some(((o + 1 - end).min(start - o), free + a.lerp(b, smooth(u))))
                    }
                    (Some((end, a)), None) => Some((o + 1 - end, free + a)),
                    (None, Some((start, b))) => Some((start - o, free + b)),
                    (None, None) => None,
                }
            };
            let Some((d, lock)) = near else { continue };
            if d > ramp {
                continue;
            }
            let x = 1.0 - d as f32 / (ramp + 1) as f32;
            let w = x * x * (3.0 - 2.0 * x);
            let y = self.t_y[slot(o) * n + i];
            self.goals[i].1 = Some(free.lerp(Vec3::new(lock.x, y, lock.z), w));
        }
        let dt = if o > first {
            self.times[slot(o)] - self.times[slot(o - 1)]
        } else {
            0.0
        };
        let legs = self.feet.iter().map(|f| &f.leg);
        reach_all(target, played, tw, &self.t_path, &mut self.sink, dt, legs, &self.goals);
    }
}

/// The travel a clip walked in place implies ([`traveled`]), worked out as the poses come: each pose the body moves as
/// far as the lowest foot slid back since the pose before, and on as it was going while no foot is down. Slides slower
/// than a twentieth of the height a second are taken for a foot shuffling, not walking. Once the hips have wandered a
/// fifth of the height from where they began, as far as [`traveled`] lets them over the whole clip, the clip travels on
/// its own and the travel stops growing: a walk or a kick in place sways less, and a dance in place that sways more
/// mustn't drift off far before it does.
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
        if flat(hips - start).length() > 0.2 * h {
            self.on = false;
        }
        self.ground.see(dt, |l| sw[self.legs[l].0].position());
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

#[derive(Clone, Copy)]
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
    let Parts { trunk, legs } = parts(skeleton, s)?;
    body_in(skeleton, s, trunk, legs)
}

/// The body as rods with round ends, for what swings loose to keep out of ([`crate::Sway`]): each from a point in one
/// bone's frame to a point in another's, with how thick it is from its line. Along the spine as thick as the spheres
/// arms keep out of, along the neck, the arms and the legs as thick as an arm is taken to be, and the head, a ball,
/// fitted to the skin if there is enough of it. None for a skeleton with no body found.
pub(crate) fn rods(skeleton: &Skeleton, s: &Shape, skin: &Surface) -> Vec<([(usize, Vec3); 2], f32)> {
    let Some(mut body) = body(skeleton, s) else {
        return Vec::new();
    };
    body.fit_head(s, skin);
    let line = |a: usize, b: usize, radius: f32| ([(a, Vec3::ZERO), (b, Vec3::ZERO)], radius);
    let mut out: Vec<_> = body.spine.windows(2).map(|w| line(w[0], w[1], body.torso)).collect();
    let chest = body.spine[body.spine.len() - 1];
    if body.spine.len() == 1 {
        out.push(line(chest, chest, body.torso));
    }
    if let Some(h) = body.head {
        if below(skeleton, h.bone, chest) {
            out.push(([(chest, Vec3::ZERO), (h.bone, h.center)], body.limb));
        }
        out.push(([(h.bone, h.center); 2], h.radius));
    }
    let limbs = body.arms.iter().map(|a| &a.limb).chain(legs_of(skeleton, s));
    for l in limbs {
        out.push(line(l.upper, l.lower, body.limb));
        out.push(line(l.lower, l.end, body.limb));
    }
    out
}

fn body_in(skeleton: &Skeleton, s: &Shape, trunk: &Trunk, legs: &[Limb]) -> Option<Body> {
    let n = skeleton.len();
    let parent = |b: usize| skeleton.bones()[b].parent;
    let pos = |b: usize| s.rest[b].position();
    let (under, spine, on_spine) = (&trunk.under, &trunk.spine, &trunk.on_spine);
    let mut legged = vec![false; n];
    for l in legs {
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
    // Arms are measured out from the spine across, as a chest bone can sit well below the shoulders.
    let across = |sb: usize, b: usize| {
        let d = pos(b) - pos(sb);
        Vec3::new(d.x, 0.0, d.z).length()
    };
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
            let Some((upper, lower)) = split(&chain, pos(end), pos, |b| across(sb, b)) else {
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
            found.push((arm, sb, i, chain, end));
        }
    }
    // The arms are the chains off one spine bone: the one whose chains end in the most bones, as hands hold fingers,
    // and the lowest of those. Hair off the head and tails of a coat or cape end in nothing.
    let (_, chest) = found
        .iter()
        .map(|(a, _, i, ..)| (under[a.limb.end], std::cmp::Reverse(*i)))
        .max()
        .map(|(u, std::cmp::Reverse(i))| (u, i))?;
    let found: Vec<_> = found.into_iter().filter(|f| f.2 == chest).collect();
    let mut arms: Vec<(Arm, usize)> = found.iter().map(|(a, sb, ..)| (*a, *sb)).collect();
    // An arm and its mirror image, built alike, turn at the same joints: a rest a little lopsided mustn't take one
    // arm's collarbone for its shoulder, as RobotExpressive's right arm, its upper arm a stub, otherwise is.
    let mut paired = vec![false; found.len()];
    for x in 0..found.len() {
        for y in x + 1..found.len() {
            let ((ax, sx, _, cx, ex), (ay, sy, _, cy, ey)) = (&found[x], &found[y]);
            if paired[x] || paired[y] || cx.len() != cy.len() || ax.out.dot(ay.out) >= 0.0 {
                continue;
            }
            let (ex, ey) = (pos(*ex), pos(*ey));
            let both = |u: usize, l: usize| {
                let (p, q) = (
                    spans(cx, ex, &pos, &|b| across(*sx, b), (u, l)),
                    spans(cy, ey, &pos, &|b| across(*sy, b), (u, l)),
                );
                [p[0].min(q[0]), p[1].min(q[1]), p[2].min(q[2])]
            };
            let Some((u, l)) = pick(cx.len(), both) else { continue };
            if pos(cx[u]).distance(ex) < 0.15 * s.height || pos(cy[u]).distance(ey) < 0.15 * s.height {
                continue;
            }
            arms[x].0.limb = limb(s, cx[u], cx[l], found[x].0.limb.end, -Vec3::Z);
            arms[y].0.limb = limb(s, cy[u], cy[l], found[y].0.limb.end, -Vec3::Z);
            (paired[x], paired[y]) = (true, true);
        }
    }
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
    /// Room for [`least_turn`] to work in.
    waves: Vec<(f32, f32, f32)>,
    edges: Vec<f32>,
    /// A mark on each target bone, all clear between uses.
    moved: Vec<bool>,
}

impl Scratch {
    pub(crate) fn new(source: &Skeleton, target: &Skeleton) -> Self {
        Self {
            sw: vec![At::IDENTITY; source.len()],
            tw: vec![At::IDENTITY; target.len()],
            spheres: Vec::with_capacity(64),
            waves: Vec::with_capacity(16),
            edges: Vec::with_capacity(48),
            moved: vec![false; target.len()],
        }
    }
}

/// The fixes to the arms, made in turn over a clip, as if each were made over the whole clip before the next, the source
/// placed once a pose for them all: elbows and hands pushed out of the body ([`push_pose`]), with both bodies then
/// hands brought together ([`Hands`]) and put on the head ([`OnHead`]), and last, as it sees the skin, hands brought
/// to the gap the source's have ([`Touch`]). On the head comes in two passes, as each hand is eased on ahead of the
/// source's: first where it would go, then put there; what comes after it waits for the second. Hands placed by
/// the body ([`Charts`]) come the same way, and are put on the head by that, not by [`OnHead`], if both bodies have
/// one.
pub(crate) fn fix_arms(
    map: &Mapping,
    rigs: [(&Skeleton, &Shape, &Surface); 2],
    [s_body, t_body]: [Option<&Body>; 2],
    fixes: Fixes,
    from: &Clip,
    out: &mut Clip,
) {
    let [(source, s, _), (target, t, _)] = rigs;
    let push = t_body.filter(|_| fixes.push_out).map(|b| (b, b.path(target)));
    let both = s_body.zip(t_body);
    let shapes = [(source, s), (target, t)];
    let mut hands = both
        .filter(|_| fixes.hands)
        .and_then(|(sb, tb)| Hands::new(map, shapes, [sb, tb]));
    let mut charts = both
        .filter(|_| fixes.place)
        .and_then(|(sb, tb)| Charts::new(map, rigs, [sb, tb]));
    let mut head = both
        .filter(|_| fixes.head && !charts.as_ref().is_some_and(Charts::heads))
        .and_then(|(sb, tb)| OnHead::new(map, shapes, [sb, tb]));
    let mut touch = both
        .filter(|_| fixes.touch)
        .and_then(|(sb, tb)| Touch::new(map, rigs, [sb, tb]));
    let none: &[usize] = &[];
    let first = union(&[
        hands.as_ref().map_or(none, |h| h.s_path()),
        head.as_ref().map_or(none, |h| h.s_path()),
        charts.as_ref().map_or(none, |c| c.s_path()),
        touch
            .as_ref()
            .filter(|_| head.is_none() && charts.is_none())
            .map_or(none, |t| t.s_path()),
    ]);
    let mut scratch = Scratch::new(source, target);
    let dt = |f: usize| f.checked_sub(1).map_or(0.0, |e| from.times[f] - from.times[e]);
    let k = head.as_ref().map_or(0, |h| h.arms.len());
    let mut raw = Vec::with_capacity(from.poses.len() * k);
    let kc = charts.as_ref().map_or(0, |c| c.on().len());
    let mut placing: Vec<f32> = Vec::with_capacity(from.poses.len() * kc);
    for (f, (pose, played)) in from.poses.iter().zip(&mut out.poses).enumerate() {
        if let Some((body, path)) = &push {
            push_pose(target, body, path, played, &mut scratch);
        }
        place(source, pose, &first, &mut scratch.sw);
        if let (Some(fix), Some(tb)) = (&mut hands, t_body) {
            fix.placed(target, tb, played, &mut scratch);
        }
        if let Some(fix) = &mut charts {
            fix.weigh(target, dt(f), played, &mut scratch);
            placing.extend_from_slice(fix.on());
        }
        if let Some(fix) = &mut head {
            fix.weigh(dt(f), &scratch.sw);
            fix.aim(target, played, &mut scratch);
            raw.extend_from_slice(&fix.on);
        } else if let (Some(fix), None) = (&mut touch, &charts) {
            fix.placed(target, played, &mut scratch);
        }
    }
    if head.is_none() && charts.is_none() {
        return;
    }
    // Each hand placed by its body no less than a pose `ONTO` before or after has it, but for how long that is away:
    // brought to its place ahead of the source's, and let go as slowly.
    let n = from.poses.len();
    for f in 1..n {
        for i in 0..kc {
            placing[f * kc + i] = placing[f * kc + i].max(placing[(f - 1) * kc + i] - dt(f) / ONTO);
        }
    }
    for f in (1..n).rev() {
        for i in 0..kc {
            placing[(f - 1) * kc + i] = placing[(f - 1) * kc + i].max(placing[f * kc + i] - dt(f) / ONTO);
        }
    }
    // Each hand eased onto the head ahead of the source's and off after it, over `ONTO` at least, never less than the
    // source's is on its own head ([`carry`]).
    let mut eased = raw;
    let mut ease = |p: usize, q: usize, step: f32| eased[q] = carry(eased[p], eased[q], step);
    for f in 1..n {
        for i in 0..k {
            ease((f - 1) * k + i, f * k + i, dt(f) / ONTO);
        }
    }
    for f in (1..n).rev() {
        for i in 0..k {
            ease(f * k + i, (f - 1) * k + i, dt(f) / ONTO);
        }
    }
    let second = union(&[
        head.as_ref().map_or(none, |h| h.s_path()),
        charts.as_ref().map_or(none, |c| c.s_path()),
        touch.as_ref().map_or(none, |t| t.s_path()),
    ]);
    for (f, (pose, played)) in from.poses.iter().zip(&mut out.poses).enumerate() {
        place(source, pose, &second, &mut scratch.sw);
        if let Some(fix) = &mut charts {
            fix.put(target, played, &mut scratch, &placing[f * kc..(f + 1) * kc], false);
        }
        if let Some(fix) = &mut head {
            fix.on.clear();
            fix.on
                .extend(eased[f * k..(f + 1) * k].iter().map(|&(w, aim)| (smooth(w), aim)));
            fix.put(target, played, &mut scratch);
        }
        if let Some(t) = &mut touch {
            t.placed(target, played, &mut scratch);
        }
    }
}

/// Pushes a pose's elbows and hands out of the spheres along its spine, `path` being the body's: the upper arm turns at
/// the shoulder to clear the elbow, then the forearm at the elbow to clear the hand.
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

/// Keeps a target's bones out of the floor: none comes further under where the lowest bone stands at rest than the bone
/// it plays comes under the source's, scaled, nor a bone playing none under it at all. A bone that would turns the
/// bone it hangs from, about where that starts, just enough to lift it to the floor, keeping its heading across it, as
/// an animator lays a tail, an ear or a shin along the ground: so does the next bone down, from that. Only a bone its
/// parent has no other child than is lifted this way, as turning a hip or a chest for one limb would move the others;
/// the bones hanging from it, if more than one, are lifted with it, as the ears with a head.
///
/// Feet placed after ([`Feet`], [`plant`]) may bend a knee back under the floor: [`Self::knees`] then swings the leg
/// about the line from its hip to its foot, the least that brings the knee and the bones down to the foot over it, so
/// the foot stays where it was put, or failing that turns it about its hip.
pub(crate) struct Floor {
    /// Each bone's source bone.
    sources: Vec<Option<usize>>,
    /// For each bone lifted this way, the bones lifted with it: itself, and those hanging from it if more than one.
    lifted: Vec<Vec<usize>>,
    /// Each leg, the bones from the root to its foot, those between its hip and its foot, and the axis it bends about
    /// at rest in the frame of the bone it hangs from.
    legs: Vec<(Limb, Vec<usize>, Vec<usize>, Vec3)>,
    /// The target's bones to place, and the source's ([`place_source`]).
    t_path: Vec<usize>,
    s_path: Vec<usize>,
    floors: [f32; 2],
    /// The target's height over the source's.
    k: f32,
}

impl Floor {
    /// Nothing if no bone can be lifted.
    pub(crate) fn new(map: &Mapping, [(source, s), (target, t)]: [(&Skeleton, &Shape); 2]) -> Option<Self> {
        let parent = |b: usize| target.bones()[b].parent;
        let sources: Vec<Option<usize>> = (0..target.len()).map(|b| map.source_of(b)).collect();
        let lifted: Vec<Vec<usize>> = (0..target.len())
            .map(|b| match parent(b) {
                Some(p) if parent(p).is_some() && t.children[p].len() == 1 => {
                    let with = &t.children[b];
                    std::iter::once(b)
                        .chain(with.iter().copied().filter(|_| with.len() > 1))
                        .collect()
                }
                _ => Vec::new(),
            })
            .collect();
        let legs: Vec<(Limb, Vec<usize>, Vec<usize>, Vec3)> = legs_of(target, t)
            .iter()
            .copied()
            .map(|leg| {
                let mut between = Vec::new();
                let mut b = parent(leg.end);
                while let Some(x) = b.filter(|&x| x != leg.upper) {
                    between.push(x);
                    b = parent(x);
                }
                let hung = parent(leg.upper).map_or(Quat::IDENTITY, |p| t.rest[p].rotation);
                let bend = hung.inverse() * t.rest[leg.upper].rotation * leg.bend;
                (leg, paths(target, [leg.end]), between, bend)
            })
            .collect();
        let ends: Vec<usize> = lifted.iter().flatten().copied().collect();
        if ends.is_empty() && legs.is_empty() {
            return None;
        }
        Some(Self {
            s_path: paths(source, ends.iter().filter_map(|&b| sources[b])),
            t_path: paths(target, ends),
            sources,
            lifted,
            legs,
            floors: [s.floor, t.floor],
            k: t.height / s.height,
        })
    }

    /// The source bones to place ([`place_source`]) before each pose.
    pub(crate) fn s_path(&self) -> &[usize] {
        &self.s_path
    }

    /// Lifts the bones of `pose`, the target playing a pose of the source placed in `scratch` ([`Self::s_path`]); or
    /// given the source's pose and skeleton, placed only if a bone is under the floor, as only then is it looked at.
    pub(crate) fn pose(
        &self,
        target: &Skeleton,
        pose: &mut [Transform],
        scratch: &mut Scratch,
        mut source: Option<(&Skeleton, &[Transform])>,
    ) {
        let Scratch {
            sw,
            tw,
            waves,
            edges,
            moved,
            ..
        } = scratch;
        let [s_floor, t_floor] = self.floors;
        let parent = |b: usize| target.bones()[b].parent;
        // Never over the target's floor.
        let low = |sw: &[At], b: usize| {
            t_floor - self.sources[b].map_or(0.0, |a| (s_floor - sw[a].position().y).max(0.0) * self.k)
        };
        // Placed once: a bone turned places again only what hangs from it.
        place(target, pose, &self.t_path, tw);
        for (i, &b) in self.t_path.iter().enumerate() {
            let with = &self.lifted[b];
            let Some(p) = parent(b).filter(|_| !with.is_empty()) else {
                continue;
            };
            if with.iter().all(|&c| tw[c].position().y >= t_floor) {
                continue;
            }
            if let Some((skeleton, s_pose)) = source.take() {
                place(skeleton, s_pose, &self.s_path, sw);
            }
            let sw = &*sw;
            if with.iter().all(|&c| tw[c].position().y >= low(sw, c)) {
                continue;
            }
            let from = tw[p].position();
            let v = tw[b].position() - from;
            // About the level line across the bone, so it keeps its heading across the floor.
            let Some(axis) = Vec3::Y.cross(v).try_normalize() else {
                continue;
            };
            let points = with.iter().map(|&c| (tw[c].position(), low(sw, c)));
            let Some(angle) = least_turn(from, axis, points, 1e-5 * v.length(), waves, edges) else {
                continue;
            };
            let turn = Quat::from_axis_angle(axis, angle);
            let above = parent(p).map_or(Quat::IDENTITY, |q| tw[q].rotation);
            pose[p].rotation = (above.inverse() * turn * tw[p].rotation).normalize();
            place(target, pose, &[p], tw);
            // `b` is all that hangs from `p`, and it and all under it come after it, parents first.
            moved[p] = true;
            for &x in &self.t_path[i..] {
                if parent(x).is_some_and(|q| moved[q]) {
                    place(target, pose, std::slice::from_ref(&x), tw);
                    moved[x] = true;
                }
            }
            moved[p] = false;
            for &x in &self.t_path[i..] {
                moved[x] = false;
            }
        }
    }

    /// Swings each leg of `pose` whose knee, or a bone between it and the foot, is under the floor about the line from
    /// its hip to its foot, the least that brings them all over it, so the foot stays where it was put. A leg folded
    /// too far for that, its foot nearer the hip than it can bend to reach, or that would have to turn its knee out
    /// to the side, turns about its hip instead, in the plane it bends in at rest, as a crouching animal rests on its
    /// hocks: the foot comes up with it, never down. Failing both, whichever leaves the lowest of them highest.
    pub(crate) fn knees(&self, target: &Skeleton, pose: &mut [Transform], scratch: &mut Scratch) {
        use std::f32::consts::FRAC_PI_4;
        let Scratch { tw, waves, edges, .. } = scratch;
        let low = self.floors[1];
        let parent = |b: usize| target.bones()[b].parent;
        for (leg, path, between, bend) in &self.legs {
            place(target, pose, path, tw);
            let lowest = |tw: &[At]| {
                between
                    .iter()
                    .map(|&b| tw[b].position().y)
                    .fold(f32::INFINITY, f32::min)
            };
            let (a, c) = (tw[leg.upper].position(), tw[leg.end].position());
            let slack = 1e-4 * (c - a).length();
            let mut best = lowest(tw);
            if best >= low - slack {
                continue;
            }
            let kept = [pose[leg.upper].rotation, pose[leg.end].rotation];
            let mut chosen = kept;
            // About the foot, splaying the knee out a little; about the hip; about the foot, however far.
            for (about_foot, most) in [(true, FRAC_PI_4), (false, f32::INFINITY), (true, f32::INFINITY)] {
                [pose[leg.upper].rotation, pose[leg.end].rotation] = kept;
                place(target, pose, path, tw);
                let world_rot = |b: Option<usize>| b.map_or(Quat::IDENTITY, |p| tw[p].rotation);
                let axis = if about_foot {
                    c - a
                } else {
                    world_rot(parent(leg.upper)) * *bend
                };
                let Some(axis) = axis.try_normalize() else {
                    continue;
                };
                let foot = (!about_foot).then(|| (c, low.min(c.y)));
                let points = between.iter().map(|&b| (tw[b].position(), low)).chain(foot);
                let Some(angle) = least_turn(a, axis, points, slack, waves, edges).filter(|t| t.abs() <= most) else {
                    continue;
                };
                let turn = Quat::from_axis_angle(axis, angle);
                pose[leg.upper].rotation =
                    (world_rot(parent(leg.upper)).inverse() * turn * tw[leg.upper].rotation).normalize();
                // The foot keeps its turn, as it does where it was put.
                pose[leg.end].rotation =
                    ((turn * world_rot(parent(leg.end))).inverse() * tw[leg.end].rotation).normalize();
                place(target, pose, path, tw);
                let now = lowest(tw);
                if now > best {
                    (best, chosen) = (now, [pose[leg.upper].rotation, pose[leg.end].rotation]);
                }
                if now >= low - slack {
                    break;
                }
            }
            [pose[leg.upper].rotation, pose[leg.end].rotation] = chosen;
        }
    }
}

/// The least turn about `axis` through `pivot` bringing each point to or over its floor, given as `(point, floor)`
/// pairs, to within `slack`; failing any, the turn leaving the one furthest under its floor least far under. Nothing if
/// none is needed. `waves` and `edges` are room to work in.
fn least_turn(
    pivot: Vec3,
    axis: Vec3,
    points: impl Iterator<Item = (Vec3, f32)>,
    slack: f32,
    waves: &mut Vec<(f32, f32, f32)>,
    edges: &mut Vec<f32>,
) -> Option<f32> {
    use std::f32::consts::{PI, TAU};
    // Each point turned by `t` stands `y + r cos(t - phase)` over its floor.
    waves.clear();
    waves.extend(points.map(|(q, floor)| {
        let off = q - pivot;
        let u = off - axis * off.dot(axis);
        let w = axis.cross(u);
        (pivot.y + axis.y * off.dot(axis) - floor, u.y.hypot(w.y), w.y.atan2(u.y))
    }));
    let lowest = |t: f32| {
        waves
            .iter()
            .map(|&(y, r, phase)| y + r * (t - phase).cos())
            .fold(f32::INFINITY, f32::min)
    };
    if lowest(0.0) >= 0.0 {
        return None;
    }
    // The least turn is one where some point just reaches its floor; failing any, one with some point at its highest.
    edges.clear();
    for &(y, r, phase) in waves.iter() {
        edges.push(phase);
        if r > 1e-9 && (y / r).abs() <= 1.0 {
            let half = (-y / r).acos();
            edges.extend([phase - half, phase + half]);
        }
    }
    let wrap = |t: f32| (t + PI).rem_euclid(TAU) - PI;
    edges
        .iter()
        .map(|&t| wrap(t))
        .filter(|&t| lowest(t) >= -slack)
        .min_by(|p, q| p.abs().total_cmp(&q.abs()))
        .or_else(|| {
            edges
                .iter()
                .map(|&t| wrap(t))
                .max_by(|p, q| lowest(*p).total_cmp(&lowest(*q)))
        })
        .filter(|t| t.abs() > 1e-6)
}

/// Keeps every pose's bones out of the floor ([`Floor`]), or with `knees`, just its knees ([`Floor::knees`]).
pub(crate) fn floor_clip(floor: &Floor, rigs: [&Skeleton; 2], from: &Clip, out: &mut Clip, knees: bool) {
    let [source, target] = rigs;
    let mut scratch = Scratch::new(source, target);
    for (s_pose, pose) in from.poses.iter().zip(&mut out.poses) {
        if knees {
            floor.knees(target, pose, &mut scratch);
        } else {
            floor.pose(target, pose, &mut scratch, Some((source, s_pose)));
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

    fn along(&self, d: Vec3) -> (Vec3, f32) {
        (
            self.matrix.matrix3.transpose() * d,
            Vec3::from(self.matrix.translation).dot(d),
        )
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

/// Each arm the two rigs share, for [`Touch`]: the target's limb, the source's hand, each hand's middle in its own
/// frame, and the skin points of each hand and its fingers.
struct Reaching {
    limb: Limb,
    source: usize,
    middle: [Vec3; 2],
    hand: [Balls; 2],
    /// The skin of each rig's other arms, from the elbow down.
    others: [Balls; 2],
    /// How far the skin of the source's hand and fingers reaches from its middle at rest, at most.
    span: f32,
    /// The target's bones to place again once the arm has moved: those from its upper bone down.
    below: Vec<usize>,
}

/// Keeps each target hand as far from the target's skin as the source's hand is from the source's, scaled by height,
/// while the source's is near its body: a hand resting on a hip rests on the target's hip, over a coat if it wears one.
/// Far from the body the hand is left where it is, except that it is kept out of the skin; in between, the two blend.
/// Then the same against the other arms' skin from the elbow down, for a hand nearer that than its body: a hand laid
/// on the other forearm or hand stays laid on it, however thick the target's is. Reached with two-bone IK, so the hand
/// keeps its turn.
pub(crate) struct Touch {
    skins: [Surface; 2],
    arms: Vec<Reaching>,
    /// The body's skin a hand can touch, by bone: all but the arms', from the shoulder down.
    balls: [Balls; 2],
    heights: [f32; 2],
    s_path: Vec<usize>,
    t_path: Vec<usize>,
    /// The skin near a hand, posed.
    points: Vec<(Vec3, Vec3)>,
    /// Each rig's [`Self::balls`] placed this pose.
    around: [Vec<Around>; 2],
    /// An arm's [`Reaching::others`] placed.
    beside: Vec<Around>,
    /// How far each source hand is from its body's skin this pose.
    gaps: Vec<f32>,
    /// Room for [`Surface::clearance`] to work in.
    frames: Vec<(Toward, f32)>,
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
        // The body's skin, but for the arms: not a prop's beside the hips.
        let off_arms = |skeleton: &Skeleton, b: &Body| {
            let off = under(skeleton, &b.arms.iter().map(|a| a.limb.upper).collect::<Vec<_>>());
            let on = b
                .spine
                .first()
                .map_or_else(|| vec![true; skeleton.len()], |&h| on_body(skeleton, h));
            off.into_iter().zip(on).map(|(o, on)| on && !o).collect::<Vec<_>>()
        };
        let keep = [off_arms(source, s_body), off_arms(target, t_body)];
        // The arms from the elbow down, but for the one with this hand.
        let lowers = |b: &Body, but: Option<usize>| {
            let arms = b.arms.iter().filter(|a| Some(a.limb.end) != but);
            arms.map(|a| a.limb.lower).collect::<Vec<_>>()
        };
        let on_arms = [
            under(source, &lowers(s_body, None)),
            under(target, &lowers(t_body, None)),
        ];
        let s_rest = source.world(&source.rest());
        // Every bone the skin a hand can touch moves with: the body's, and the arms' from the elbow down.
        let path = |skeleton: &Skeleton, skin: &Surface, k: usize| {
            let on = |b: usize| keep[k][b] || on_arms[k][b];
            let ends = hands.iter().map(|h| if k == 0 { h.1 } else { h.0.end });
            paths(skeleton, skin.bone.iter().copied().filter(|&b| on(b)).chain(ends))
        };
        let (s_path, t_path) = (path(source, s_skin, 0), path(target, t_skin, 1));
        let arms: Vec<Reaching> = hands
            .iter()
            .filter_map(|(limb, a, [s_on, t_on])| {
                let moved = under(target, &[limb.upper]);
                let (s_others, t_others) = (
                    under(source, &lowers(s_body, Some(*a))),
                    under(target, &lowers(t_body, Some(limb.end))),
                );
                let middle = [s_skin.center(*a)?, t_skin.center(limb.end)?];
                let at_rest = s_rest[*a].matrix.transform_point3(middle[0]);
                let on_hand = (0..s_skin.len()).filter(|&i| s_on[s_skin.bone[i]]);
                let span = on_hand
                    .map(|i| {
                        s_rest[s_skin.bone[i]]
                            .matrix
                            .transform_point3(s_skin.at[i])
                            .distance(at_rest)
                    })
                    .fold(0.0, f32::max);
                Some(Reaching {
                    limb: *limb,
                    source: *a,
                    middle,
                    hand: [s_skin.balls(|b| s_on[b]), t_skin.balls(|b| t_on[b])],
                    others: [s_skin.balls(|b| s_others[b]), t_skin.balls(|b| t_others[b])],
                    span,
                    below: t_path.iter().copied().filter(|&b| moved[b]).collect(),
                })
            })
            .collect();
        if arms.is_empty() {
            return None;
        }
        Some(Self {
            points: Vec::with_capacity(s_skin.len().max(t_skin.len())),
            around: [Vec::new(), Vec::new()],
            beside: Vec::new(),
            gaps: vec![f32::INFINITY; arms.len()],
            frames: Vec::new(),
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

    /// Fixes `played`, the target playing a pose of the source placed in `scratch` ([`Self::s_path`]).
    pub(crate) fn placed(&mut self, target: &Skeleton, played: &mut [Transform], scratch: &mut Scratch) {
        // Within `near` of the skin (over the height) a source hand is touching it; past `far` it is free.
        let (near, far) = (0.015, 0.05);
        let ([s_skin, t_skin], [s_balls, t_balls]) = (&self.skins, &self.balls);
        let [s_height, t_height] = self.heights;
        let scale = t_height / s_height;
        let pts = &mut self.points;
        let Scratch { sw, tw, .. } = scratch;
        place(target, played, &self.t_path, tw);
        // The skin a hand can touch is off the arms, so it keeps still while they move.
        let [s_around, t_around] = &mut self.around;
        s_balls.place(sw, s_around);
        t_balls.place(tw, t_around);
        for (arm, by_body) in self.arms.iter().zip(&mut self.gaps) {
            *by_body = f32::INFINITY;
            let ([cs, ct], [s_hand, t_hand]) = (arm.middle, &arm.hand);
            let c = sw[arm.source].matrix.transform_point3(cs);
            s_skin.posed_near(s_balls, s_around, sw, c, pts);
            let Some((g, _)) = s_skin.clearance(pts, sw, c, s_hand, &mut self.frames) else {
                continue;
            };
            *by_body = g;
            let g = g.max(0.0);
            let x = ((g / s_height - near) / (far - near)).clamp(0.0, 1.0);
            let w = 1.0 - x * x * (3.0 - 2.0 * x);
            let end = tw[arm.limb.end];
            let c = end.matrix.transform_point3(ct);
            t_skin.posed_near(t_balls, t_around, tw, c, pts);
            let Some((now, facing)) = t_skin.clearance(pts, tw, c, t_hand, &mut self.frames) else {
                continue;
            };
            // Never into the skin, however deep in it the hand has come.
            let want = (now + w * (g * scale - now)).max(0.0);
            if (want - now).abs() > 1e-4 * t_height {
                reach_in(target, played, tw, &arm.limb, end.position() + facing * (want - now));
                place(target, played, &arm.below, tw);
            }
        }
        // Then each hand against the other arms, once every arm is where its body has it. A hand is laid on the skin
        // it is nearer: on its body though an arm hangs by, on the other hand though that rests on the body.
        let beside = &mut self.beside;
        for (arm, &by_body) in self.arms.iter().zip(&self.gaps) {
            let ([cs, ct], [s_hand, t_hand], [s_others, t_others]) = (arm.middle, &arm.hand, &arm.others);
            let c = sw[arm.source].matrix.transform_point3(cs);
            s_others.place(sw, beside);
            if Around::nearest(beside, c) > far * s_height + arm.span {
                continue;
            }
            s_skin.posed_near(s_others, beside, sw, c, pts);
            let Some((g, _)) = s_skin.clearance(pts, sw, c, s_hand, &mut self.frames) else {
                continue;
            };
            let x = ((g.max(0.0) / s_height - near) / (far - near)).clamp(0.0, 1.0);
            let nearer = smooth(0.5 + (by_body - g) / s_height / (2.0 * NEARER));
            let w = (1.0 - x * x * (3.0 - 2.0 * x)) * nearer;
            if w <= 0.0 {
                continue;
            }
            let end = tw[arm.limb.end];
            let c = end.matrix.transform_point3(ct);
            t_others.place(tw, beside);
            t_skin.posed_near(t_others, beside, tw, c, pts);
            let Some((now, facing)) = t_skin.clearance(pts, tw, c, t_hand, &mut self.frames) else {
                continue;
            };
            let by = w * (g.max(0.0) * scale - now);
            if by.abs() > 1e-4 * t_height {
                reach_in(target, played, tw, &arm.limb, end.position() + facing * by);
                place(target, played, &arm.below, tw);
            }
        }
    }
}

/// How much nearer the other arm than its body (over the height) a source hand is wholly by the arm; as much nearer
/// its body, it isn't by the arm at all.
const NEARER: f32 = 0.01;

/// Within `TOGETHER` of each other (over the height) the source's hands are together; past `APART`, apart.
const TOGETHER: f32 = 0.06;
const APART: f32 = 0.15;

/// Brings each two target hands together where the source's meet: a clap, a clasp, hands pressed in prayer. Near, the
/// way from one hand to the other is the source's, turned into the target's frame and scaled by height; farther off it
/// blends back to what the target already does, and past a sixth of the height the hands are left alone. The hands
/// meet at the point nearest where they were that both arms reach and that keeps both out of the body and the head,
/// so arms too short to clap overhead clap in front of the face, and long arms clap rather than cross. Reached with
/// two-bone IK, so each hand keeps its turn.
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

    /// Fixes `played`, the target playing a pose of the source placed in `scratch` ([`Self::s_path`]).
    pub(crate) fn placed(&mut self, target: &Skeleton, t_body: &Body, played: &mut [Transform], scratch: &mut Scratch) {
        let (near, far) = (TOGETHER, APART);
        let Scratch { sw, tw, spheres, .. } = scratch;
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

/// How far onto the head a hand goes, and where it is aimed, `on` as the source's is, carried on from `from` a pose
/// eased by `step` before or after: never less on than `from` but for `step`. Eased on further than the source's own,
/// it is aimed where the source's is for as far on as that is, and where `from` aims it for the rest, so it is aimed
/// where it goes in the pose it is eased toward while the source's isn't on the head, and its aim moves on to its own
/// smoothly as the source's comes on: aimed at the one or the other, it would jump from one to the other.
fn carry(from: (f32, Vec3), on: (f32, Vec3), step: f32) -> (f32, Vec3) {
    let w = from.0 - step;
    if w > on.0 {
        (w, (on.1 * on.0 + from.1 * (w - on.0)) / w)
    } else {
        on
    }
}

/// How long, in seconds, a hand takes at least to be put on the head, and to be taken off.
const ONTO: f32 = 0.25;

/// Keeps each target hand on its head where the source's hand is on theirs: covering an ear, clasped on top, a palm to
/// the forehead, scratching the back of the head. The way from the middle of the source's head to its wrist is taken
/// out of the head's turn from rest, turned into the target's frame, scaled from one head to the other and turned with
/// the target's head, so the hand lands on the same place on a head of any size, even one that sits differently on its
/// neck. Within one and a half head radii of the middle of the source's head the hand is put there; past two it is left
/// alone; in between, the two blend. Nor is a hand on the head with its wrist over half its forearm's length off the
/// head's surface, blending from two fifths: a cartoon's head is so big that a hand raised beside it, well clear, is
/// within two of its radii, and put there, on a head of common size, it would be pulled onto it. A hand sweeping past
/// the head is not on it: one moving over fifteen head radii a
/// second against the head lets go, and past thirty is left alone. A hand that can't get there (short arms, a big head)
/// goes as near as it can over the head's surface, not into it, and the elbow swings round the line from shoulder to
/// hand till the arm is no deeper in the head than the source's. Reached with two-bone IK, so the hand keeps its turn.
///
/// A hand darting to the head would jump onto it, so each is eased on over a quarter of a second at least, ahead of the
/// source's, and off as slowly after it lets go; while easing, the hand moves a share of the way from where it would be
/// to where it lands on the head, so it arrives along its own path, on time ([`carry`]).
pub(crate) struct OnHead {
    heads: [Head; 2],
    /// Each target arm playing a source arm: the target's limb, the source's hand and the source's limb.
    arms: Vec<(Limb, usize, Limb)>,
    /// How long each source arm's forearm is.
    fores: Vec<f32>,
    turn: Quat,
    scale: f32,
    /// Each head bone's rest turn, undone.
    rests: [Quat; 2],
    s_path: Vec<usize>,
    t_path: Vec<usize>,
    /// Where each source hand was last pose, from the middle of the head, in the head's frame.
    last: Vec<Option<Vec3>>,
    /// How far onto the head each target hand was put last pose, before easing in and out, and where it was aimed.
    eased: Vec<(f32, Vec3)>,
    /// How far onto the head each target hand goes this pose, and where to: from where it is to where it goes on the
    /// head, in the target head's frame.
    on: Vec<(f32, Vec3)>,
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
        let fores = arms
            .iter()
            .map(|(_, a, sl)| s.rest[sl.lower].position().distance(s.rest[*a].position()))
            .collect();
        Some(Self {
            heads: [sh, th],
            fores,
            turn: map.turn(),
            scale: th.radius / sh.radius,
            rests: [s.rest[sh.bone].rotation.inverse(), t.rest[th.bone].rotation.inverse()],
            s_path: paths(
                source,
                arms.iter().flat_map(|a| [a.1, a.2.upper, a.2.lower]).chain([sh.bone]),
            ),
            t_path: paths(target, arms.iter().map(|a| a.0.end).chain([th.bone])),
            last: vec![None; arms.len()],
            eased: vec![(0.0, Vec3::ZERO); arms.len()],
            on: Vec::with_capacity(arms.len()),
            arms,
        })
    }

    /// Forgets where the hands were, as after a cut.
    pub(crate) fn reset(&mut self) {
        self.last.fill(None);
        self.eased.fill((0.0, Vec3::ZERO));
    }

    /// The source bones to place ([`place_source`]) before [`Self::placed`].
    pub(crate) fn s_path(&self) -> &[usize] {
        &self.s_path
    }

    /// Fixes `played`, the target playing a pose of the source placed in `scratch` ([`Self::s_path`]), `dt` seconds
    /// after the pose before. Each hand is put on the head over `ONTO` at least, so it gets there late when the
    /// source's darts there, and taken off as slowly, aimed as it was last put ([`carry`]).
    pub(crate) fn placed(&mut self, target: &Skeleton, dt: f32, played: &mut [Transform], scratch: &mut Scratch) {
        let step = if dt > 0.0 { dt / ONTO } else { 0.0 };
        self.weigh(dt, &scratch.sw);
        self.aim(target, played, scratch);
        for ((was, aimed), (on, aim)) in self.eased.iter_mut().zip(&mut self.on) {
            let (w, toward) = carry((*was, *aimed), (*on, *aim), step);
            let w = w.min(*was + step);
            (*was, *aimed) = (w, toward);
            (*on, *aim) = (smooth(w), toward);
        }
        self.put(target, played, scratch);
    }

    /// Sets how far onto the head each source hand is ([`Self::on`]), placed in `sw` `dt` seconds after the pose
    /// before: within `near` head radii of the middle of its head fully, past `far` not at all; its wrist off the
    /// head's surface under `close` of its forearm fully, over `clear` not at all; and moving against the head under
    /// `slow` head radii a second fully, over `fast` (sweeping past) not at all.
    fn weigh(&mut self, dt: f32, sw: &[At]) {
        let (near, far, slow, fast) = (1.5, 2.0, 15.0, 30.0);
        let (close, clear) = (0.4, 0.6);
        let (sh, s_rest) = (self.heads[0], self.rests[0]);
        let (sc, s_turn) = (sh.center(sw), sw[sh.bone].rotation * s_rest);
        self.on.clear();
        self.on.extend(
            self.arms
                .iter()
                .zip(&self.fores)
                .zip(&mut self.last)
                .map(|((&(_, a, _), &fore), last)| {
                    let d = sw[a].position() - sc;
                    let local = s_turn.inverse() * d;
                    let speed = last
                        .filter(|_| dt > 0.0)
                        .map_or(0.0, |l| local.distance(l) / dt / sh.radius);
                    *last = Some(local);
                    let off = (d.length() - sh.radius) / fore.max(1e-6);
                    let w = (1.0 - smooth((d.length() / sh.radius - near) / (far - near)))
                        * (1.0 - smooth((off - close) / (clear - close)))
                        * (1.0 - smooth((speed - slow) / (fast - slow)));
                    (w, Vec3::ZERO)
                }),
        );
    }

    /// Sets where each target hand on the head at all ([`Self::on`]) goes: the way from where it is in `played` to the
    /// place on its head where the source's hand is on theirs, in the target head's frame. The source's pose is placed
    /// in `scratch`.
    fn aim(&mut self, target: &Skeleton, played: &[Transform], scratch: &mut Scratch) {
        let ([sh, th], [s_rest, t_rest]) = (self.heads, self.rests);
        let Scratch { sw, tw, .. } = scratch;
        let (sc, s_turn) = (sh.center(sw), sw[sh.bone].rotation * s_rest);
        let mut placed = false;
        for (&(limb, a, _), (w, aim)) in self.arms.iter().zip(&mut self.on) {
            if *w <= 0.0 {
                continue;
            }
            if !placed {
                place(target, played, &self.t_path, tw);
                placed = true;
            }
            let local = s_turn.inverse() * (sw[a].position() - sc);
            let (t_turn, center) = (tw[th.bone].rotation * t_rest, th.center(tw));
            let goal = center + t_turn * (self.turn * local) * self.scale;
            let (sa, sb, hand) = (
                tw[limb.upper].position(),
                tw[limb.lower].position(),
                tw[limb.end].position(),
            );
            let goal = within(goal, center, (sa, 0.999 * (sa.distance(sb) + sb.distance(hand))));
            *aim = t_turn.inverse() * (goal - hand);
        }
    }

    /// Moves each target hand [`Self::on`] of the way it is aimed, the source's pose placed in `scratch`.
    fn put(&self, target: &Skeleton, played: &mut [Transform], scratch: &mut Scratch) {
        let ([sh, th], t_rest) = (self.heads, self.rests[1]);
        let Scratch { sw, tw, .. } = scratch;
        let sc = sh.center(sw);
        let mut placed = false;
        for (&(limb, a, sl), &(w, aim)) in self.arms.iter().zip(&self.on) {
            if w <= 0.0 {
                continue;
            }
            // Arms hang apart, so reaching one leaves the other, and the head, placed as they were.
            if !placed {
                place(target, played, &self.t_path, tw);
                placed = true;
            }
            let (t_turn, center) = (tw[th.bone].rotation * t_rest, th.center(tw));
            reach_in(target, played, tw, &limb, tw[limb.end].position() + t_turn * aim * w);
            // The arm no deeper in the head than the source's is in theirs.
            let deep = |p: [Vec3; 3], c: Vec3| off_line(c, p[0], p[1]).min(off_line(c, p[1], p[2]));
            let s_arm = [sl.upper, sl.lower, a].map(|b| sw[b].position());
            let need = deep(s_arm, sc).min(sh.radius) / sh.radius * th.radius;
            place(target, played, &self.t_path, tw);
            swivel(target, played, tw, &limb, |arm| deep(arm, center), need, w);
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
/// failing that, as far as gets the most. Then only `share` of that, so an arm eased onto the head swings round as it
/// goes. The least turn is found to within a fraction of a degree, not to the step searched by, so as `need` grows
/// pose by pose the elbow swings round smoothly rather than in steps. `world` is the pose placed.
fn swivel(
    skeleton: &Skeleton,
    pose: &mut [Transform],
    world: &[At],
    limb: &Limb,
    clear: impl Fn([Vec3; 3]) -> f32,
    need: f32,
    share: f32,
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
    const STEP: f32 = std::f32::consts::PI / 12.0;
    let got = |angle: f32| clear([a, a + Quat::from_axis_angle(axis, angle) * (b - a), c]);
    let mut best = (now, 0.0);
    'search: for k in 1..=12 {
        for sign in [1.0, -1.0] {
            let angle = sign * k as f32 * STEP;
            let g = got(angle);
            if g > best.0 {
                best = (g, angle);
                if g >= need {
                    // The least turn clearing it lies within the step before.
                    let (mut lo, mut hi) = (angle - sign * STEP, angle);
                    for _ in 0..10 {
                        let mid = 0.5 * (lo + hi);
                        if got(mid) >= need {
                            hi = mid;
                        } else {
                            lo = mid;
                        }
                    }
                    best.1 = hi;
                    break 'search;
                }
            }
        }
    }
    let angle = best.1 * share.clamp(0.0, 1.0);
    if angle == 0.0 {
        return;
    }
    let turn = Quat::from_axis_angle(axis, angle);
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
