//! Secondary motion for the bones a clip doesn't drive: hair, tails, capes, ears and skirts.
//!
//! Played through a mapping, a target bone no source bone drives keeps its rest pose under its parent, so a ponytail
//! is as stiff as the skull. [`chains`] finds such bones from the mapping and the rest pose alone, with no names: runs
//! of bones with nothing driven under them, hanging from a bone that is driven. [`Sway`] then swings each chain after
//! the body, pose by pose.
//!
//! A chain is a row of particles, one at each bone's tail, solved from the root to the tip. Each is pulled by a damped
//! spring toward where it would be if its bone kept its rest pose under the bone above, then put back at its bone's
//! length from that bone's tail and kept within a cone of its rest pose. The spring is stepped exactly for a goal
//! moving at a steady speed through the step (by a rational approximation of the matrix exponential, so with no
//! library math, and steady at any step), and the tail's speed is carried round with its bone as the bone turns, so
//! the motion is nearly the same at any frame rate: a tail swinging as far as its chain is long ends within a
//! hundredth of that length of where it would at another rate. Whipped about harder, a chain of several bones is as
//! touchy as a real one, and rates tell. Gravity pulls less what the rest pose already carries, so a chain at rest
//! stays at rest. The body never feels a chain: only the chains' own rotations are written.
//!
//! Last, each particle is kept out of the body and over the floor. The body is the one arms are kept out of
//! ([`contact`]): rods with round ends along the spine, the neck, the arms and the legs, and a ball for the head,
//! guessed from the bones. The floor is where the lowest bone stands at rest. A particle slides round its bone's head
//! to get out, so its bone keeps its length, but may leave its cone: a rope let down lies along the floor however
//! far that bends it. Nothing comes nearer a part of the body, or lower, than it is at rest, so a ponytail starting
//! inside the head isn't thrown out of it and the rest pose stays at rest. Nor is a tail asked further out from a
//! part than its bone's head is, but for how much further out it is at rest: a skirt's bone starting on a thigh is
//! laid along the thigh as it swings up through where the bone starts, not turned straight out of it.

use glam::{Affine3A, Quat, Vec3, Vec3A};

use crate::shape::Shape;
use crate::{Clip, Error, Mapping, Skeleton, Surface, Transform, contact, held, transfer};

#[cfg(test)]
mod before;

/// The longest step the spring is advanced by, in seconds: a longer time between poses is split into equal steps.
const STEP: f32 = 1.0 / 60.0;
/// The most steps one pose is split into.
const STEPS: f32 = 4.0;
/// A clip that loops is gone round till its chains start a round within this many radians of how they started the
/// one before, and never more often than this.
const LOOP_CLOSED: f32 = 0.01;
const ROUNDS: usize = 8;
/// What is left between a loop's ends is eased away over this many seconds at its end, or a third of it if shorter.
const LOOP_EASE: f32 = 0.3;
/// A clip loops if the body is this near at its end to where it starts: each chain's root and each part of the body
/// within this share of the height, the roots turned within this many radians.
const LOOP_NEAR: f32 = 0.1;
const LOOP_TURN: f32 = 0.35;
/// And its last pose is its first again if within this share of the height, and this many radians.
const LOOP_SAME: f32 = 0.02;
/// A chain shorter than this share of the rig's height is left out, and so is a bone this share of that nearer its
/// parent: the first is a helper or a marker, the second has no length to swing.
const SHORT: f32 = 0.01;
const NO_LENGTH: f32 = 1e-4;
/// No chain bone is taken to be thicker than this share of the rig's height, from its middle to its side.
const THICK: f32 = 0.02;
/// A particle nearer a part of the body at rest than the two are thick keeps this share of the rig's height less
/// than that away from it, and as much lower than at rest: room for rounding, so the rest pose never touches.
const ROOM: f32 = 1e-4;

/// A run of bones to sway, each hanging from the one before.
#[derive(Clone, Debug, PartialEq)]
pub struct Chain {
    /// The chain's bones, from the one nearest the body to the tip.
    pub bones: Vec<usize>,
    /// The bone the first one hangs from: its parent, which the chain follows and never moves.
    pub parent: usize,
    /// How long the chain is at rest, from where its first bone starts to its tip, in the rig's units. A last bone
    /// with no bone after it is as long as it is far from its parent.
    pub length: f32,
}

/// The chains of the target that playing through `map` leaves at rest: its hair, tails, ears, capes and skirts, and
/// anything else that hangs off the body with no bone of the source to move it. Empty if the mapping was made for
/// another skeleton.
///
/// A bone is in a chain when no source bone drives it or any bone under it, not even by a share of a turn (as a bone
/// between two mapped ones or a twist bone beside one is driven), some bone above it is driven, and it stands away
/// from its parent. A chain starts under a bone that is in none and runs down one bone at a time. Where a bone has
/// several such children, the chain goes on down the one with the most length under it and each of the others starts
/// a chain of its own, hanging from that bone.
///
/// Left out are:
/// - an arm's or a leg's own bones and everything under a hand or a foot (fingers and toes the source lacks), found
///   as [`contact::arms`] and [`contact::legs`] find them, and as the mapping finds where a hand begins;
/// - a bone hanging alone with no bone under it. Nothing but a name tells an ear from an eye, a jaw, a control or a
///   marker for where the head ends, and on real rigs it is nearly always one of those;
/// - a bone lying along the line from its parent to the next driven joint, with all under it: a twist bone;
/// - a chain shorter than a hundredth of the rig's height, a helper or a marker;
/// - a bone standing where its parent does, or right where any driven bone does (a helper a rig's own constraints
///   would turn with that joint). The bones under it may start chains, unless it is under a chain's bone: then it and
///   all under it stay stiff.
///
/// Bones with nothing driven above them, as controls hung from a root, are none of the body's and are left alone.
#[must_use]
pub fn chains(map: &Mapping, target: &Skeleton) -> Vec<Chain> {
    let n = target.len();
    if map.pairs().len() != n || n == 0 {
        return Vec::new();
    }
    // Which bones playing keeps at rest, and the limbs, are worked out as the player does: with feet held from the
    // root hung from their legs.
    let hung = held::held(target);
    let (kept, limb, ends) = {
        let (sk, map) = match &hung {
            Some(h) => (&h.skeleton, map.hung([None, Some(h)])),
            None => (target, std::borrow::Cow::Borrowed(map)),
        };
        let shape = Shape::of(sk);
        let (kept, frame) = (transfer::kept(&map, &shape), contact::frame(sk, &shape));
        let real = |b: usize| hung.as_ref().map_or(b, |h| h.real(b));
        let hung_of = |b: usize| hung.as_ref().map_or(b, |h| h.hung(b));
        let kept: Vec<bool> = (0..n).map(|b| kept[hung_of(b)]).collect();
        // In a limb: on the way from where it starts to its end, or past where a hand begins.
        let (mut limb, mut ends) = (vec![false; n], vec![false; n]);
        for b in 0..n {
            limb[real(b)] = shape.palm[b].is_some();
        }
        for l in frame.arms.iter().chain(&frame.legs) {
            ends[real(l.end)] = true;
            let mut b = Some(l.end);
            while let Some(on) = b {
                limb[real(on)] = true;
                b = sk.bones()[on].parent.filter(|_| on != l.upper);
            }
        }
        (kept, limb, ends)
    };
    let shape = Shape::light(target);
    let (children, height) = (&shape.children, shape.height);
    let parent = |b: usize| target.bones()[b].parent;
    let pos = |b: usize| shape.rest[b].position();

    // Nothing driven in it or under it; something driven above it.
    let mut free = kept.clone();
    for b in (0..n).rev() {
        if let Some(p) = parent(b) {
            free[p] &= free[b];
        }
    }
    let mut under_driven = vec![false; n];
    // Under a hand or a foot.
    let mut handed = vec![false; n];
    // A twist bone: it and all under it lie along the line from its parent to the next joint that leads to driven
    // bones, a child of the parent's or of a bone standing where the parent does (a helper turned with a joint has the
    // joint's twist bones, not its children). Only the top of a free run can be one.
    let mut twist = vec![false; n];
    let along = |b: usize, from: Vec3, to: Vec3| {
        let (line, mut stack) = (to - from, vec![b]);
        let reach = line.length();
        while let Some(c) = stack.pop() {
            let d = pos(c) - from;
            let t = (d.dot(line) / (reach * reach)).clamp(0.0, 1.0);
            if (d - t * line).length() > 0.05 * reach {
                return false;
            }
            stack.extend(&children[c]);
        }
        true
    };
    for b in 0..n {
        let Some(p) = parent(b) else { continue };
        under_driven[b] = under_driven[p] || !kept[p];
        handed[b] = handed[p] || ends[p];
        twist[b] = twist[p]
            || free[b]
                && !free[p]
                && (0..n).any(|d| {
                    !free[d]
                        && parent(d).is_some_and(|q| pos(q).distance(pos(p)) <= NO_LENGTH * height)
                        && pos(d).distance(pos(p)) > NO_LENGTH * height
                        && along(b, pos(p), pos(d))
                });
    }
    // Standing right where a driven bone does: a helper for that joint.
    let driven: Vec<Vec3> = (0..n).filter(|&b| !kept[b]).map(pos).collect();
    let twin = |b: usize| driven.iter().any(|d| d.distance(pos(b)) <= NO_LENGTH * height);
    // Whether a bone can be in a chain, as far as it and the bones above it say.
    let can: Vec<bool> = (0..n)
        .map(|b| {
            free[b]
                && under_driven[b]
                && !limb[b]
                && !handed[b]
                && !twist[b]
                && parent(b).is_some_and(|p| pos(b).distance(pos(p)) > NO_LENGTH * height)
                && !twin(b)
        })
        .collect();
    // The length of the chain starting at each bone, and the child it goes on down.
    let mut reach = vec![0.0f32; n];
    let mut on: Vec<usize> = vec![usize::MAX; n];
    for b in (0..n).rev() {
        if !can[b] {
            continue;
        }
        for &c in &children[b] {
            let length = pos(c).distance(pos(b)) + reach[c];
            if can[c] && (on[b] == usize::MAX || length > reach[b]) {
                (reach[b], on[b]) = (length, c);
            }
        }
        if on[b] == usize::MAX {
            reach[b] = tail(target, &shape, b, None).distance(pos(b));
        }
    }
    // Under a chain's bone, a bone that can't be in one stays stiff with all under it.
    let (mut chained, mut stiff) = (vec![false; n], vec![false; n]);
    let mut out = Vec::new();
    for b in 0..n {
        let Some(p) = parent(b) else { continue };
        chained[b] = can[b] && !stiff[p];
        stiff[b] = !chained[b] && (chained[p] || stiff[p]);
        // A bone alone with nothing under it is left out: it is as likely an eye or a marker as an ear.
        let alone = on[b] == usize::MAX && children[b].is_empty();
        if !chained[b] || chained[p] && on[p] == b || reach[b] < SHORT * height || alone {
            continue;
        }
        let mut bones = vec![b];
        while on[bones[bones.len() - 1]] != usize::MAX {
            bones.push(on[bones[bones.len() - 1]]);
        }
        out.push(Chain {
            bones,
            parent: p,
            length: reach[b],
        });
    }
    out
}

/// Where a chain's bone ends at rest, in the world: where the `next` bone of the chain starts, or for the last bone,
/// the middle of where its children start if that is away from it, else as far on from its parent as it is from it.
fn tail(target: &Skeleton, shape: &Shape, b: usize, next: Option<usize>) -> Vec3 {
    let pos = |b: usize| shape.rest[b].position();
    if let Some(c) = next {
        return pos(c);
    }
    let kids = &shape.children[b];
    let mid = kids.iter().map(|&c| pos(c)).sum::<Vec3>() / kids.len().max(1) as f32;
    if !kids.is_empty() && mid.distance(pos(b)) > NO_LENGTH * shape.height {
        return mid;
    }
    let from = target.bones()[b].parent.map_or(pos(b), pos);
    pos(b) + (pos(b) - from)
}

/// How a chain swings. [`Spring::of`] sets it for a rig by its height; every chain starts with that.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spring {
    /// How hard gravity pulls, in the rig's units a second squared: 9.81 for a rig in meters. It sets how fast every
    /// chain swings, so it must fit the rig's units and be more than nothing.
    pub gravity: f32,
    /// How fast a bone swings back to its rest pose, beside a pendulum as long as the bone: 1 swings as fast as the
    /// pendulum, 2 twice as fast. First for a bone hanging straight down at rest, then for one standing straight
    /// up; a bone between the two takes its share of each.
    pub stiffness: [f32; 2],
    /// How soon a swing dies out: 0 swings on, 1 comes to rest without swinging past. For a bone hanging down, then
    /// for one standing up, as for `stiffness`.
    pub damping: [f32; 2],
    /// How much the still air slows a bone, a second: it trails behind a body on the move, the more the faster.
    pub drag: f32,
    /// How far a bone may bend from its rest pose under the bone above it, in radians. Pushed by the body or the
    /// floor it may bend further.
    pub limit: f32,
    /// How thick a bone is, from its middle to its side, as a share of its length: what is kept out of the body and
    /// over the floor. Never taken for more than a fiftieth of the rig's height.
    pub radius: f32,
}

impl Spring {
    /// The spring for a rig this many of its units to the meter: 1 for meters, 100 for centimeters.
    ///
    /// Set to bend chains about as far as springs set up by hand do, on rigs that carry both: stiff, three times
    /// as fast as a pendulum, with most of the slowing left to the air. A body standing in place hardly stirs them;
    /// a walk bends them a quarter of a radian on average, a run nearly half.
    #[must_use]
    pub fn per_meter(units: f32) -> Self {
        Self {
            gravity: 9.81 * units,
            stiffness: [3.0, 3.0],
            damping: [0.6, 0.6],
            drag: 25.0,
            limit: 75f32.to_radians(),
            radius: 0.2,
        }
    }

    /// The spring for this rig, its units guessed by taking it to stand 1.7 meters tall.
    #[must_use]
    pub fn of(target: &Skeleton) -> Self {
        Self::per_meter(Shape::light(target).height / 1.7)
    }
}

/// Sways the chains of a skeleton, a pose at a time, without allocating.
///
/// ```
/// # use dittomo::{Bone, Mapping, Skeleton, Sway, Transform, Vec3};
/// # let bone = |name: &str, parent| Bone { name: name.into(), parent, rest: Transform::new(Vec3::Y, Default::default()) };
/// # let rig = Skeleton::new(vec![bone("hips", None), bone("spine", Some(0))]).unwrap();
/// let mapping = Mapping::new(&rig, &rig);
/// let mut sway = Sway::new(&mapping, &rig).unwrap();
/// let mut pose = rig.rest();
/// // Each frame, after the pose is played and fixed up:
/// sway.pose(1.0 / 60.0, &mut pose);
/// ```
#[derive(Clone, Debug)]
pub struct Sway {
    chains: Vec<Chain>,
    springs: Vec<Spring>,
    /// The bones above the chains, each after its parent, with where each parent is kept in `mats` and `rots`: these
    /// are placed in the world from the pose, as far as the chains need.
    above: Vec<usize>,
    above_parent: Vec<usize>,
    /// Each bone in the world, as far as needed, and one more that stands for a root's parent. A chain's bone is
    /// kept as its head and its turn only.
    mats: Vec<Affine3A>,
    rots: Vec<Quat>,
    heads: Vec<Vec3>,

    // By chain.
    /// Where each chain's bones begin in the lists by bone, and where the last one's end.
    starts: Vec<usize>,
    /// Where the first bone starts from the bone the chain hangs from, in that bone's frame, in the world's units.
    offsets: Vec<Vec3>,
    /// Where the chain's first bone started and how the bone it hangs from was turned, at the pose before.
    root_at: Vec<Vec3>,
    root_turn: Vec<Quat>,

    // By chain bone, the chains one after another.
    bones: Vec<usize>,
    /// The bone's rest turn under its parent, and how long it is.
    rest: Vec<Quat>,
    length: Vec<f32>,
    /// How far up the bone points at rest (0 straight down, 1 straight up): with its length, what a [`Spring`] is
    /// fitted to.
    lift: Vec<f32>,
    /// The spring's pull for each unit off the goal, and its damping and the air's together.
    stiff: Vec<f32>,
    damp: Vec<f32>,
    /// All a step reads of the bone, side by side.
    links: Vec<Link>,
    /// Where the bone's tail is, how fast it moves, and where its goal was a step ago.
    at: Vec<Vec3>,
    speed: Vec<Vec3>,
    goal: Vec<Vec3>,
    /// Which way the rest pose's down was under the bone above, a step ago.
    lean: Vec<Vec3A>,
    /// How high its tail is at rest.
    rest_y: Vec<f32>,

    /// The parts of the body the chains keep out of, and whether they and the floor are kept out of at all.
    rods: Vec<Rod>,
    body: bool,
    ground: bool,
    /// By chain bone and then by rod: how far the bone's tail is from the rod's line at rest, how near it may
    /// come, and how much further from that line it is at rest than the bone's head.
    apart: Vec<f32>,
    clear: Vec<f32>,
    ahead: Vec<f32>,
    /// Each rod's ends in the world at the pose before, and the rod at each step of this one, the steps one after
    /// another.
    was: Vec<[Vec3; 2]>,
    now: Vec<Laid>,
    /// The rods the chain in hand could reach.
    near: Vec<usize>,
    floor: f32,
    height: f32,

    step: f32,
    /// Whether a pose has been taken since the start or the last reset, and whether time has yet to pass since.
    live: bool,
    still: bool,

    /// What the step as it was first written reads, for the tests that hold this one to it.
    #[cfg(test)]
    before: before::Before,
}

impl Sway {
    /// Sways the chains [`chains`] finds on `target` for this mapping, each with [`Spring::of`] the target.
    ///
    /// # Errors
    /// [`Error::NotTheirMapping`] if the mapping was made for a target of another size.
    pub fn new(map: &Mapping, target: &Skeleton) -> Result<Self, Error> {
        if map.pairs().len() != target.len() {
            return Err(Error::NotTheirMapping);
        }
        Self::skinned(map, target, &Surface::default())
    }

    /// [`Sway::new`], knowing the target's skin: its head is as big as the skin shows.
    pub(crate) fn skinned(map: &Mapping, target: &Skeleton, skin: &Surface) -> Result<Self, Error> {
        if map.pairs().len() != target.len() {
            return Err(Error::NotTheirMapping);
        }
        Ok(Self::build(target, &chains(map, target), Spring::of(target), skin))
    }

    /// Sways these chains of `target`, each with this spring: for chains found another way, as from a rig's own
    /// list. They are kept ordered by their first bones.
    ///
    /// # Panics
    /// If a chain is empty, a bone of one isn't the child of the bone before it (the first, of the chain's parent), a
    /// bone is in two chains, a bone stands where its parent does or ends where it starts, or a chain hangs from a
    /// bone that is under a chain but in none.
    #[must_use]
    pub fn with(target: &Skeleton, chains: &[Chain], spring: Spring) -> Self {
        Self::build(target, chains, spring, &Surface::default())
    }

    fn build(target: &Skeleton, chains: &[Chain], spring: Spring, skin: &Surface) -> Self {
        let n = target.len();
        let mut chains = chains.to_vec();
        chains.sort_by_key(|c| c.bones.first().copied());
        let shape = Shape::light(target);
        let (pos, rot) = (|b: usize| shape.rest[b].position(), |b: usize| shape.rest[b].rotation);
        let parent = |b: usize| target.bones()[b].parent;
        let mut chained = vec![false; n];
        // Where each chain bone ends at rest.
        let mut tails = Vec::new();
        let mut sway = Self {
            springs: vec![spring; chains.len()],
            above: Vec::new(),
            above_parent: Vec::new(),
            mats: vec![Affine3A::IDENTITY; n + 1],
            rots: vec![Quat::IDENTITY; n + 1],
            heads: vec![Vec3::ZERO; n + 1],
            starts: vec![0],
            offsets: Vec::new(),
            root_at: vec![Vec3::ZERO; chains.len()],
            root_turn: vec![Quat::IDENTITY; chains.len()],
            bones: Vec::new(),
            rest: Vec::new(),
            length: Vec::new(),
            lift: Vec::new(),
            stiff: Vec::new(),
            damp: Vec::new(),
            links: Vec::new(),
            at: Vec::new(),
            speed: Vec::new(),
            goal: Vec::new(),
            lean: Vec::new(),
            rest_y: Vec::new(),
            rods: Vec::new(),
            body: true,
            ground: true,
            apart: Vec::new(),
            clear: Vec::new(),
            ahead: Vec::new(),
            was: Vec::new(),
            now: Vec::new(),
            near: Vec::new(),
            floor: shape.floor,
            height: shape.height,
            step: f32::NAN,
            live: false,
            still: true,
            chains: Vec::new(),
            #[cfg(test)]
            before: before::Before::default(),
        };
        for chain in &mut chains {
            assert!(!chain.bones.is_empty(), "a chain with no bones");
            let mut up = chain.parent;
            for (k, &b) in chain.bones.iter().enumerate() {
                assert!(b < n && parent(b) == Some(up), "bone {b} doesn't hang from bone {up}");
                assert!(!std::mem::replace(&mut chained[b], true), "bone {b} is in two chains");
                let end = tail(target, &shape, b, chain.bones.get(k + 1).copied());
                let (along, length) = (end - pos(b), end.distance(pos(b)));
                assert!(length > 0.0, "bone {b} has no length");
                tails.push(end);
                let rest = target.bones()[b].rest.rotation.normalize();
                // Which way its tail is from its head, in its own frame, and which way down is in its parent's.
                let (axis, down) = (rot(b).inverse() * (along / length), rot(up).inverse() * Vec3::NEG_Y);
                sway.bones.push(b);
                sway.rest.push(rest);
                sway.length.push(length);
                sway.lift.push((1.0 + along.y / length) / 2.0);
                sway.links.push(Link {
                    rest,
                    toward: (rest * axis).into(),
                    down: down.into(),
                    length,
                    short: 1.0 / length,
                    ..Link::default()
                });
                #[cfg(test)]
                sway.before.push(axis, down);
                up = b;
            }
            chain.length = sway.length[sway.length.len() - chain.bones.len()..].iter().sum();
            sway.offsets
                .push(rot(chain.parent).inverse() * (pos(chain.bones[0]) - pos(chain.parent)));
            sway.starts.push(sway.bones.len());
        }
        // The body's parts, as the rig with any held feet hung from their legs has them. One carried by a chain's
        // bone, or by a bone under one, is left out: it goes where the chain does.
        let mut swung = chained.clone();
        for b in 0..n {
            swung[b] |= parent(b).is_some_and(|p| swung[p]);
        }
        let hung = held::held(target);
        let rods = match &hung {
            Some(h) => contact::rods(&h.skeleton, &Shape::light(&h.skeleton), &h.skin(skin)),
            None => contact::rods(target, &shape, skin),
        };
        let real = |b: usize| hung.as_ref().map_or(b, |h| h.real(b));
        for (ends, radius) in rods {
            let ends = ends.map(|(b, at)| (real(b), at));
            if ends.iter().all(|&(b, _)| !swung[b]) {
                sway.rods.push(Rod { ends, radius });
            }
        }
        let rods = sway.rods.len();
        for (&tail, &bone) in tails.iter().zip(&sway.bones) {
            sway.rest_y.push(tail.y);
            for rod in &sway.rods {
                let [a, b] = rod.ends.map(|(b, at)| shape.rest[b].matrix.transform_point3(at));
                let (far, head) = (
                    tail.distance(nearest(a, b, tail)),
                    pos(bone).distance(nearest(a, b, pos(bone))),
                );
                sway.apart.push(far);
                sway.ahead.push((far - head).max(0.0));
            }
        }
        sway.clear.resize(sway.apart.len(), 0.0);
        sway.was.resize(rods, [Vec3::ZERO; 2]);
        sway.now.resize(rods * STEPS as usize, Laid::default());
        sway.near.reserve(rods);
        // The bones above the chains that aren't in one, and those carrying the body's parts, to place from each pose.
        let mut needed = vec![false; n];
        let holds = sway.rods.iter().flat_map(|r| r.ends.map(|(b, _)| b));
        for top in chains.iter().map(|c| c.parent).chain(holds) {
            let mut up = Some(top);
            while let Some(b) = up.filter(|&b| !chained[b] && !needed[b]) {
                needed[b] = true;
                up = parent(b);
            }
        }
        for b in (0..n).filter(|&b| needed[b]) {
            assert!(
                parent(b).is_none_or(|p| !chained[p]),
                "bone {b} holds a chain but is in none, under one"
            );
            sway.above.push(b);
            sway.above_parent.push(parent(b).unwrap_or(n));
        }
        let all = sway.bones.len();
        for list in [&mut sway.stiff, &mut sway.damp] {
            list.resize(all, 0.0);
        }
        for list in [&mut sway.at, &mut sway.speed, &mut sway.goal] {
            list.resize(all, Vec3::ZERO);
        }
        sway.lean.resize(all, Vec3A::ZERO);
        sway.chains = chains;
        for c in 0..sway.chains.len() {
            sway.fit(c);
        }
        sway
    }

    /// The chains swayed, ordered by their first bones.
    #[must_use]
    pub fn chains(&self) -> &[Chain] {
        &self.chains
    }

    /// How this chain swings.
    ///
    /// # Panics
    /// If there is no such chain.
    #[must_use]
    pub fn spring(&self, chain: usize) -> Spring {
        self.springs[chain]
    }

    /// Makes this chain swing another way, from the next pose on. Where it is and how fast it moves are kept.
    ///
    /// # Panics
    /// If there is no such chain.
    pub fn set_spring(&mut self, chain: usize, spring: Spring) {
        self.springs[chain] = spring;
        self.fit(chain);
    }

    /// Works out a chain's bones' numbers from its spring.
    fn fit(&mut self, chain: usize) {
        let spring = self.springs[chain];
        let (sin, cos) = spring.limit.clamp(0.0, std::f32::consts::PI).sin_cos();
        for i in self.starts[chain]..self.starts[chain + 1] {
            let lift = self.lift[i];
            let blend = |[hangs, stands]: [f32; 2]| hangs + (stands - hangs) * lift;
            let quick = blend(spring.stiffness);
            // Never quite none: the pull is divided by.
            let stiff = (quick * quick * spring.gravity / self.length[i]).max(1e-6);
            let drag = spring.drag.max(0.0);
            self.stiff[i] = stiff;
            self.damp[i] = 2.0 * blend(spring.damping).max(0.0) * stiff.sqrt() + drag;
            let link = &mut self.links[i];
            (link.loose, link.drag, link.gravity) = (1.0 / stiff, drag, spring.gravity);
            (link.sin, link.cos) = (sin, cos);
            // No nearer the body's parts than the two are thick, nor lower than its own thickness over the floor;
            // but as near and as low as it is at rest.
            let thick = (spring.radius * self.length[i]).clamp(0.0, THICK * self.height);
            let room = ROOM * self.height;
            link.level = if self.ground {
                (self.floor + thick).min(self.rest_y[i] - room)
            } else {
                f32::NEG_INFINITY
            };
            let rods = self.rods.len();
            for (k, rod) in self.rods.iter().enumerate() {
                let at = i * rods + k;
                self.clear[at] = if self.body {
                    (thick + rod.radius).min(self.apart[at] - room)
                } else {
                    0.0
                };
            }
        }
        // The steps carried are worked out again at the next pose.
        self.step = f32::NAN;
    }

    /// Whether the chains are kept out of the body, and over the floor: both, unless said otherwise here.
    pub fn collide(&mut self, body: bool, floor: bool) {
        (self.body, self.ground) = (body, floor);
        for c in 0..self.chains.len() {
            self.fit(c);
        }
    }

    /// Forgets where the chains are and how they move: the next pose puts them at rest under the body as it is then,
    /// moving as it does, as after a cut to another place or another clip.
    pub fn reset(&mut self) {
        (self.live, self.still) = (false, true);
    }

    /// Takes the chains along with a body put at `pose` all at once: a character set down somewhere else, or a clip
    /// that loops starting over. Each chain goes with the bone it hangs from, as it is under it and as it moves
    /// beside it, and its bones' rotations are written to `pose`, the same as at the pose before. The first pose
    /// and the first after [`Sway::reset`] put the chains at rest, as [`Sway::pose`] does.
    ///
    /// # Panics
    /// If `pose` doesn't have one transform per bone.
    pub fn jump(&mut self, pose: &mut [Transform]) {
        if self.live {
            self.place(pose);
            for c in 0..self.chains.len() {
                let hangs = self.chains[c].parent;
                let (turn, start) = (self.rots[hangs], self.heads[hangs] + self.rots[hangs] * self.offsets[c]);
                let round = (turn * self.root_turn[c].inverse()).normalize();
                let from = std::mem::replace(&mut self.root_at[c], start);
                self.root_turn[c] = turn;
                for i in self.starts[c]..self.starts[c + 1] {
                    self.at[i] = start + round * (self.at[i] - from);
                    self.speed[i] = round * self.speed[i];
                    // Its bone too, for a chain hanging from it.
                    let b = self.bones[i];
                    self.heads[b] = start + round * (self.heads[b] - from);
                    self.rots[b] = (round * self.rots[b]).normalize();
                }
            }
        }
        // Held there: each bone points at where its tail now is.
        self.pose(0.0, pose);
    }

    /// Moves the chains with a body moved `by` all at once, nothing turned.
    fn shift(&mut self, by: Vec3) {
        for at in self.at.iter_mut().chain(&mut self.goal).chain(&mut self.root_at) {
            *at += by;
        }
        for ends in &mut self.was {
            *ends = ends.map(|end| end + by);
        }
    }

    /// Places the bones above the chains, and those carrying the body's parts, in the world.
    fn place(&mut self, pose: &[Transform]) {
        let n = self.mats.len() - 1;
        assert_eq!(pose.len(), n, "one transform per bone");
        for (&b, &p) in self.above.iter().zip(&self.above_parent) {
            let local = &pose[b];
            let rotation = local.rotation.normalize();
            self.mats[b] =
                self.mats[p] * Affine3A::from_scale_rotation_translation(local.scale, rotation, local.translation);
            self.rots[b] = unit(self.rots[p] * rotation);
            self.heads[b] = self.mats[b].translation.into();
        }
    }

    /// Sways a whole clip of the target, as played and fixed up, stepped through its times: each chain bone's
    /// rotation is written to each pose.
    ///
    /// A clip that loops, the body at its end nearly as at its start but for where on the floor it stands, is gone
    /// round first, twice at least, till the chains start a round as they started the one before (to a hundredth of
    /// a radian; eight times at most), and the round after is kept. Chains swung hard never settle into a round, so
    /// what is left between how the round kept starts and how the next would is eased away over its last third of a
    /// second: the chains end as they start and the loop closes, though they may come a little into the body or out
    /// of their cones there. Any other clip starts with its chains at rest.
    ///
    /// # Errors
    /// If the clip doesn't pose the target.
    pub fn clip(&mut self, clip: &mut Clip) -> Result<(), Error> {
        let n = clip.poses.len();
        clip.check(self.mats.len() - 1)?;
        if n == 0 || self.chains.is_empty() {
            return Ok(());
        }
        let Clip { times, poses } = clip;
        let dt = |f: usize| times[f] - times[f - 1];
        // Where the body is at the end, against where it starts.
        self.reset();
        let mut pose = poses[n - 1].clone();
        self.pose(0.0, &mut pose);
        let end = (self.root_at.clone(), self.root_turn.clone(), self.was.clone());
        self.reset();
        pose.clone_from(&poses[0]);
        self.pose(0.0, &mut pose);
        let roots = self.root_at.len() as f32;
        let mut travel = (0..end.0.len()).map(|c| end.0[c] - self.root_at[c]).sum::<Vec3>() / roots;
        travel.y = 0.0;
        let off = (0..end.0.len())
            .map(|c| end.0[c] - travel - self.root_at[c])
            .chain((0..end.2.len()).flat_map(|k| [0, 1].map(|e| end.2[k][e] - travel - self.was[k][e])))
            .map(Vec3::length)
            .fold(0.0, f32::max);
        let turned = (0..end.1.len())
            .map(|c| 2.0 * end.1[c].dot(self.root_turn[c]).abs().min(1.0).acos())
            .fold(0.0, f32::max);
        let long = times[n - 1] - times[0];
        if !(n > 2 && long > 0.0 && off <= LOOP_NEAR * self.height && turned <= LOOP_TURN) {
            self.reset();
            for f in 0..n {
                self.pose(if f == 0 { 0.0 } else { dt(f) }, &mut poses[f]);
            }
            return Ok(());
        }
        // A last pose that is nearly the first again takes no time to get from: the chains jump there with the body.
        // Else the first comes as long after it as a pose does, the body traveling on through that pose as it does
        // through the others.
        let same = off <= LOOP_SAME * self.height && turned <= LOOP_SAME;
        let wrap = if same { 0.0 } else { long / (n - 1) as f32 };
        let back = -travel * (n as f32 / (n - 1) as f32);
        let again = |sway: &mut Self, first: &[Transform], pose: &mut Vec<Transform>| -> Vec<Quat> {
            pose.clear();
            pose.extend_from_slice(first);
            if same {
                sway.jump(pose);
            } else {
                sway.shift(back);
                sway.pose(wrap, pose);
            }
            sway.bones.iter().map(|&b| pose[b].rotation).collect()
        };
        let mut first: Vec<Quat> = Vec::new();
        for _ in 0..ROUNDS {
            for f in 1..n {
                pose.clone_from(&poses[f]);
                self.pose(dt(f), &mut pose);
            }
            let before = std::mem::replace(&mut first, again(self, &poses[0], &mut pose));
            let moved = before
                .iter()
                .zip(&first)
                .map(|(a, b)| apart(*a, *b))
                .fold(0.0, f32::max);
            if !before.is_empty() && moved <= LOOP_CLOSED {
                break;
            }
        }
        // The round kept: its first pose has its chains as the last round left them.
        for (&b, &turn) in self.bones.iter().zip(&first) {
            poses[0][b].rotation = turn;
        }
        for f in 1..n {
            self.pose(dt(f), &mut poses[f]);
        }
        // How the chains would start the round after, against how this one started: what is left between the two
        // is eased away over the last poses, so the loop closes however wildly the chains swing.
        let next = again(self, &poses[0], &mut pose);
        let ease = LOOP_EASE.min(long / 3.0);
        for f in 1..n {
            let left = (times[n - 1] + wrap - times[f]) / ease;
            if left >= 1.0 {
                continue;
            }
            let share = 1.0 - left.max(0.0);
            let share = share * share * (3.0 - 2.0 * share);
            for (i, &b) in self.bones.iter().enumerate() {
                let mut off = next[i].inverse() * first[i];
                if off.w < 0.0 {
                    off = -off;
                }
                poses[f][b].rotation = (poses[f][b].rotation * Quat::IDENTITY.slerp(off, share)).normalize();
            }
        }
        Ok(())
    }

    /// Sways the chains `dt` seconds on from the last pose. `pose` is the target's, each transform relative to its
    /// parent, as played and fixed up: the body is read from it, and each chain bone's rotation is written to it
    /// (whatever it was: a chain's goal is its rest pose under the bone above). The first pose and the first after
    /// [`Sway::reset`] put the chains at rest under the body, moving as the body does by the pose after: one already
    /// walking then doesn't whip them back.
    ///
    /// A later pose with no time passed (or a time that is no number, or less than none) holds the chains: each
    /// bone points at where its tail was, from where the body has it now, at its own length and within its cone,
    /// and how fast it moved is kept for when time goes on. So a pause shows the chains as they were, and a body
    /// moved while paused drags them after it. Nothing is kept out of the body or the floor then.
    ///
    /// The bones' scales are taken to be as at rest.
    ///
    /// # Panics
    /// If `pose` doesn't have one transform per bone.
    pub fn pose(&mut self, dt: f32, pose: &mut [Transform]) {
        self.place(pose);
        let moving = self.live && dt.is_finite() && dt > 0.0;
        let steps = if moving {
            (dt / STEP - 1e-3).ceil().clamp(1.0, STEPS)
        } else {
            1.0
        };
        let h = if moving { dt / steps } else { self.step };
        if moving && h != self.step {
            for (i, link) in self.links.iter_mut().enumerate() {
                link.carry = carry(h, self.stiff[i], self.damp[i]);
            }
            self.step = h;
        }
        let per_h = 1.0 / h;
        // The body's parts go from where they were to where they are, evenly through the steps, as the chains' roots
        // do.
        let rods = self.rods.len();
        let collide = moving && self.body && rods > 0;
        for (k, rod) in self.rods.iter().enumerate() {
            let ends = rod.ends.map(|(b, at)| self.mats[b].transform_point3(at));
            let from = std::mem::replace(&mut self.was[k], ends);
            if !collide {
                continue;
            }
            let (from, ends) = (from.map(Vec3A::from), ends.map(Vec3A::from));
            let (mut step, last) = (1.0, steps as usize - 1);
            for at in 0..last {
                let t = step / steps;
                self.now[at * rods + k] = Laid::new(from[0].lerp(ends[0], t), from[1].lerp(ends[1], t), rod.radius);
                step += 1.0;
            }
            self.now[last * rods + k] = Laid::new(ends[0], ends[1], rod.radius);
        }
        for c in 0..self.chains.len() {
            let hangs = self.chains[c].parent;
            let (turn, start) = (self.rots[hangs], self.heads[hangs] + self.rots[hangs] * self.offsets[c]);
            let range = self.starts[c]..self.starts[c + 1];
            if !moving {
                let (mut head, mut up) = (Vec3A::from(start), turn);
                for i in range {
                    let (b, link) = (self.bones[i], self.links[i]);
                    let rested = unit(up * link.rest);
                    let toward = up * link.toward;
                    let goal = head + link.length * toward;
                    let (turned, at) = if self.live {
                        // Held: toward where its tail was, at its length and within its cone.
                        let out = Vec3A::from(self.at[i]) - head;
                        let far = out.length_squared();
                        let dir = if far > 1e-12 * link.length * link.length {
                            out / far.sqrt()
                        } else {
                            toward
                        };
                        let dir = cone(dir, toward, link.cos, link.sin).map_or(dir, |(dir, _)| dir);
                        (unit(arc(toward, dir) * rested), head + link.length * dir)
                    } else {
                        self.speed[i] = Vec3::ZERO;
                        (rested, goal)
                    };
                    (self.at[i], self.goal[i]) = (at.into(), goal.into());
                    self.lean[i] = up * link.down;
                    pose[b].rotation = if self.live {
                        unit(up.conjugate() * turned)
                    } else {
                        link.rest
                    };
                    (self.rots[b], self.heads[b]) = (turned, head.into());
                    (head, up) = (at, turned);
                }
                (self.root_at[c], self.root_turn[c]) = (start, turn);
                continue;
            }
            let links = &self.links[range.clone()];
            if self.still {
                // The first time on from rest, each tail is taken to have been moving as its goal does now: a
                // body already walking when it is first seen doesn't whip its chains back.
                let (mut head, mut up) = (start, turn);
                for (i, link) in range.clone().zip(links) {
                    head += link.length * Vec3::from(up * link.toward);
                    up = unit(up * link.rest);
                    self.speed[i] = (head - self.goal[i]) / dt;
                }
            }
            // The bone the chain hangs from goes from where it was to where it is, evenly through the steps.
            let (from_at, from_turn) = (Vec3A::from(self.root_at[c]), self.root_turn[c]);
            let to_at = Vec3A::from(start);
            let to_turn = if from_turn.dot(turn) < 0.0 { -turn } else { turn };
            // No rod further from where the chain starts than the chain is long and the two are thick can touch it.
            let within = self.chains[c].length + THICK * self.height;
            let (mut k, mut root_was) = (1.0, from_at);
            while k <= steps {
                let last = k == steps;
                let (mut head, mut up) = if last {
                    (to_at, turn)
                } else {
                    let t = k / steps;
                    (from_at.lerp(to_at, t), from_turn.lerp(to_turn, t).normalize())
                };
                let placed = if collide {
                    let at = (k as usize - 1) * rods;
                    &self.now[at..at + rods]
                } else {
                    &[]
                };
                self.near.clear();
                for (r, rod) in placed.iter().enumerate() {
                    let reach = within + rod.radius;
                    if head.distance_squared(rod.nearest(head)) < reach * reach {
                        self.near.push(r);
                    }
                }
                let near = &self.near[..];
                // Where this bone's head was a step ago, and how fast it moves now.
                let mut head_was = std::mem::replace(&mut root_was, head);
                let mut drift = (head - head_was) * per_h;
                // Each bone's tail, its speed, its goal and its lean, as the step before left them.
                let kept = (self.at[range.clone()].iter_mut())
                    .zip(&mut self.speed[range.clone()])
                    .zip(&mut self.goal[range.clone()])
                    .zip(&mut self.lean[range.clone()]);
                for ((i, link), (((at_kept, speed_kept), goal_kept), lean_kept)) in range.clone().zip(links).zip(kept) {
                    // Where the tail would be with the bone at rest under the bone above, and how fast that moves.
                    let rested = up * link.rest;
                    let toward = up * link.toward;
                    let goal = head + link.length * toward;
                    let goal_was = Vec3A::from(*goal_kept);
                    let pace = (goal - goal_was) * per_h;
                    // Gravity, less what the rest pose carries, and the air, as how far they hold the tail off its
                    // goal. What the rest pose carries turns with the bone above: half way between where that was
                    // a step ago and is now, as taking it where it is now would be late by half a step.
                    let lean = up * link.down;
                    let carried = 0.5 * (lean + std::mem::replace(lean_kept, lean));
                    let pull = link.gravity * (Vec3A::NEG_Y - carried) - link.drag * pace;
                    let held = pull * link.loose;
                    // The spring, stepped as it is: how far the tail is off, and how fast that changes.
                    let was = Vec3A::from(*at_kept);
                    let (off, rate) = (was - goal_was - held, Vec3A::from(*speed_kept) - pace);
                    let [a, b, c, d] = link.carry;
                    let free = goal + (a * off + b * rate) + held;
                    let speed = c * off + d * rate + pace;
                    // Back at the bone's length from its head, within the cone about its rest.
                    let out = free - head;
                    let far = out.length_squared();
                    let mut dir = if far > 1e-12 * link.length * link.length {
                        out / far.sqrt()
                    } else {
                        toward
                    };
                    let mut edge = cone(dir, toward, link.cos, link.sin);
                    if let Some((at, _)) = edge {
                        dir = at;
                    }
                    // Out of the body and over the floor, though that take it out of the cone.
                    let reach = (
                        &self.clear[i * rods..(i + 1) * rods],
                        &self.ahead[i * rods..(i + 1) * rods],
                    );
                    let (kept, struck, floored) = keep_out(head, link, dir, reach, near, placed);
                    if struck.is_some() || floored {
                        (dir, edge) = (kept, None);
                    }
                    // The bone holds the tail in along its line, so about its head the tail keeps only its speed
                    // across the bone: across the bone as it was, carried round to the bone as it is, beside the
                    // head as it moves now. (Dropping what is along the bone as it is now would slow a swing by how
                    // far it turned in the step: more the longer the step, so by the frame rate.)
                    let before = (was - head_was) * link.short;
                    let across = speed - drift;
                    let across = across - across.dot(before) * before;
                    let mut speed =
                        across - (across.dot(dir) / (1.0 + before.dot(dir)).max(1e-6)) * (before + dir) + drift;
                    if let Some((_, side)) = edge {
                        // At the cone's edge it stops going outward, beside the goal.
                        let outward = link.cos * side - link.sin * toward;
                        speed -= (speed - pace).dot(outward).max(0.0) * outward;
                    }
                    if let Some(away) = struck {
                        // Against the body it stops going in, beside the goal, which moves as the body does.
                        speed -= (speed - pace).dot(away).min(0.0) * away;
                    }
                    if floored {
                        speed.y = speed.y.max(0.0);
                    }
                    let at = head + link.length * dir;
                    (*at_kept, *speed_kept, *goal_kept) = (at.into(), speed.into(), goal.into());
                    // The arc is a unit turn and so are the two the rest is made of: one step brings their product
                    // back to one, with no root, so however long the chain its turns stay unit ones.
                    let turned = unit(arc(toward, dir) * rested);
                    if last {
                        let bone = self.bones[i];
                        pose[bone].rotation = unit(up.conjugate() * turned);
                        (self.rots[bone], self.heads[bone]) = (turned, head.into());
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

/// All a step reads of a chain bone, side by side: one stretch of memory a bone, the bones of a chain one after
/// another.
#[derive(Clone, Copy, Debug, Default)]
struct Link {
    /// The bone's rest turn under its parent.
    rest: Quat,
    /// Which way its tail is from its head at rest, and which way down is, both in its parent's frame.
    toward: Vec3A,
    down: Vec3A,
    /// How long it is, and one over that.
    length: f32,
    short: f32,
    /// One over the spring's pull for each unit off the goal, the air's damping, and gravity.
    loose: f32,
    drag: f32,
    gravity: f32,
    /// The cosine and sine of how far it may bend.
    cos: f32,
    sin: f32,
    /// How low its tail may come.
    level: f32,
    /// How the spring's offset and speed are carried over one step of `step` seconds.
    carry: [f32; 4],
}

/// A part of the body a chain keeps out of: a rod with round ends from a point in one bone's frame to a point in
/// another's, or a ball if the two are one.
#[derive(Clone, Copy, Debug)]
struct Rod {
    ends: [(usize, Vec3); 2],
    radius: f32,
}

/// A rod where it is at one step: where it starts, how far and which way it goes on from there, one over the square
/// of how far, and how thick it is. Worked out once a step, so that the nearest point of it takes no division.
#[derive(Clone, Copy, Debug, Default)]
struct Laid {
    from: Vec3A,
    along: Vec3A,
    per: f32,
    radius: f32,
}

impl Laid {
    fn new(a: Vec3A, b: Vec3A, radius: f32) -> Self {
        let along = b - a;
        Self {
            from: a,
            along,
            per: 1.0 / along.length_squared().max(1e-20),
            radius,
        }
    }

    /// The point of the rod's line nearest `p`.
    fn nearest(&self, p: Vec3A) -> Vec3A {
        let t = ((p - self.from).dot(self.along) * self.per).clamp(0.0, 1.0);
        self.from + t * self.along
    }
}

/// The point of the line from `a` to `b` nearest `p`.
fn nearest(a: Vec3, b: Vec3, p: Vec3) -> Vec3 {
    let along = b - a;
    let t = ((p - a).dot(along) / along.length_squared().max(1e-20)).clamp(0.0, 1.0);
    a + t * along
}

/// A bone's direction kept within the cone about `toward`: nothing if it is already, else the direction at the
/// cone's edge and the way across from `toward` it is at.
fn cone(dir: Vec3A, toward: Vec3A, cos: f32, sin: f32) -> Option<(Vec3A, Vec3A)> {
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

/// A bone's direction with its tail out of the rods and over the floor: the bone starts at `head` and is `link`'s
/// length, `clear` is how near its tail may come to each rod's line, `near` the rods to look at and `link`'s level
/// how low it may come. The tail goes round the head, straight away from where it is too near, until it is clear;
/// the floor comes last, so it wins over a rod. Also the way out of the last rod it was in, and whether the floor
/// held it up.
///
/// A tail is asked no further out from a rod's line than its bone's head is now, but for `ahead`: how much further
/// out than the head it is at rest. So a bone that starts in a rod or on it (a skirt on a thigh, hair in the head)
/// is let lie as deep as it starts, and one whose head a rod swings through is let be, not turned straight out.
///
/// A rod is looked at tail first: how far the head is from it, which takes a root, only once the tail is too near.
fn keep_out(
    head: Vec3A,
    link: &Link,
    mut dir: Vec3A,
    (clear, ahead): (&[f32], &[f32]),
    near: &[usize],
    rods: &[Laid],
) -> (Vec3A, Option<Vec3A>, bool) {
    let length = link.length;
    let mut out = None;
    // A few times, as getting out of one may go into another, or further along the same.
    for _ in 0..8 {
        let mut any = false;
        for &k in near {
            let reach = clear[k];
            if reach <= 0.0 {
                continue;
            }
            let rod = &rods[k];
            let tail = head + length * dir;
            let from = rod.nearest(tail);
            let gap = tail.distance_squared(from);
            if gap >= reach * reach {
                continue;
            }
            let reach = reach.min(head.distance(rod.nearest(head)) + ahead[k]);
            if gap >= reach * reach {
                continue;
            }
            // Round the head, the tail is `reach` from there at one angle off the line to it.
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
            // With no way out, as far as it gets: straight away.
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
    let low = (link.level - head.y) * link.short;
    if dir.y >= low {
        return (dir, out, false);
    }
    if low >= 1.0 {
        return (Vec3A::Y, out, true);
    }
    let flat = Vec3A::new(dir.x, 0.0, dir.z);
    let wide = flat.length_squared();
    let flat = if wide > 1e-12 { flat / wide.sqrt() } else { Vec3A::X };
    (low * Vec3A::Y + (1.0 - low * low).sqrt() * flat, out, true)
}

/// The angle between two turns, nearly: right for small ones, never more than 2.
fn apart(a: Quat, b: Quat) -> f32 {
    2.0 * (a.inverse() * b).xyz().length()
}

/// The shortest turn taking one direction to another, with no library math. (glam's own gives no turn at all under
/// a twenty-fifth of a degree, which would hold a bone nearly at rest dead still.)
fn arc(from: Vec3A, to: Vec3A) -> Quat {
    let (axis, w) = (from.cross(to), 1.0 + from.dot(to));
    let long = w * w + axis.length_squared();
    if long > 1e-12 {
        let by = 1.0 / long.sqrt();
        Quat::from_vec4((axis * by).extend(w * by))
    } else {
        // Straight back: half a turn about any line across.
        Quat::from_vec4(from.any_orthonormal_vector().extend(0.0))
    }
}

/// A turn that is all but a unit one, made one: a step of Newton's method toward one over its length, so with no
/// root and no division. What is left is the square of how far off it was: nothing, for the product of unit turns.
fn unit(turn: Quat) -> Quat {
    turn * (1.5 - 0.5 * turn.length_squared())
}

/// How a damped spring's offset and speed are carried over `h` seconds: the exponential of `h` times
/// `[[0, 1], [-stiff, -damp]]`, by its (2, 2) Padé approximant, as rows. It is a ratio of two polynomials, so needs no
/// library math and gives the same bits everywhere, and it never grows whatever the step.
fn carry(h: f32, stiff: f32, damp: f32) -> [f32; 4] {
    // hM and its square over twelve.
    let m = [0.0, h, -h * stiff, -h * damp];
    let sq = [
        m[1] * m[2] / 12.0,
        m[1] * m[3] / 12.0,
        m[3] * m[2] / 12.0,
        (m[2] * m[1] + m[3] * m[3]) / 12.0,
    ];
    let top = [
        1.0 + sq[0] + m[0] / 2.0,
        sq[1] + m[1] / 2.0,
        sq[2] + m[2] / 2.0,
        1.0 + sq[3] + m[3] / 2.0,
    ];
    let low = [
        1.0 + sq[0] - m[0] / 2.0,
        sq[1] - m[1] / 2.0,
        sq[2] - m[2] / 2.0,
        1.0 + sq[3] - m[3] / 2.0,
    ];
    let det = low[0] * low[3] - low[1] * low[2];
    let inv = [low[3] / det, -low[1] / det, -low[2] / det, low[0] / det];
    [
        inv[0] * top[0] + inv[1] * top[2],
        inv[0] * top[1] + inv[1] * top[3],
        inv[2] * top[0] + inv[3] * top[2],
        inv[2] * top[1] + inv[3] * top[3],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bone;

    /// A skeleton of bones with no turn at rest, each this far from its parent.
    fn rig(bones: &[(Option<usize>, [f32; 3])]) -> Skeleton {
        let bones = bones
            .iter()
            .enumerate()
            .map(|(i, &(parent, at))| Bone {
                name: format!("bone{i}"),
                parent,
                rest: Transform::new(at.into(), Quat::IDENTITY),
            })
            .collect();
        Skeleton::new(bones).unwrap()
    }

    /// A chain of these bones under the first bone of the rig.
    fn chain(bones: &[usize]) -> Chain {
        Chain {
            bones: bones.to_vec(),
            parent: 0,
            length: 0.0,
        }
    }

    /// The spring's energy, bone by bone: of its speed past its goal's, and of how far it is off its goal.
    fn energy(sway: &Sway, was: &[Vec3], dt: f32) -> f32 {
        (0..sway.bones.len())
            .map(|i| {
                let pace = (sway.goal[i] - was[i]) / dt;
                let off = sway.at[i] - sway.goal[i];
                0.5 * (sway.speed[i] - pace).length_squared() + 0.5 * sway.stiff[i] * off.length_squared()
            })
            .sum()
    }

    #[test]
    fn a_step_carries_a_spring_as_the_exponential_does() {
        // Against the closed form for a spring that swings: over steps short and long, and never growing.
        for (stiff, damp) in [(49.0f32, 6.8f32), (900.0, 3.0), (8829.0, 131.0)] {
            let w = (stiff - damp * damp / 4.0).sqrt();
            for h in [1.0f32 / 240.0, 1.0 / 144.0, 1.0 / 60.0] {
                let (c, s, fall) = ((w * h).cos(), (w * h).sin(), (-damp * h / 2.0).exp());
                let exact = [
                    fall * (c + damp / (2.0 * w) * s),
                    fall * s / w,
                    -fall * stiff / w * s,
                    fall * (c - damp / (2.0 * w) * s),
                ];
                let got = carry(h, stiff, damp);
                let scale = [1.0, 1.0 / w, w, 1.0];
                let x = w * h;
                for k in 0..4 {
                    let off = (got[k] - exact[k]).abs() / scale[k];
                    assert!(
                        off < 0.02 * x.powi(5) + 1e-5,
                        "{stiff} {damp} {h}: {got:?} for {exact:?}"
                    );
                }
            }
            for h in [0.125f32, 0.5, 10.0, 1e4] {
                let [a, b, c, d] = carry(h, stiff, damp);
                // Its two rates of growth are no more than 1: the product and the sum say so (Jury's test).
                let (det, trace) = (a * d - b * c, a + d);
                assert!(
                    det.is_finite() && det <= 1.0 && trace.abs() <= 1.0 + det,
                    "{stiff} {damp} {h}"
                );
            }
        }
    }

    #[test]
    fn a_bone_let_go_swings_as_a_damped_spring_does() {
        // One bone hanging 0.2 m under a still body, let go a hundredth of its length to the side. With springs
        // that swing: as set, one comes to rest without swinging past.
        let sk = rig(&[(None, [0.0; 3]), (Some(0), [0.0, -0.2, 0.0])]);
        let springy = Spring {
            damping: [0.45, 0.45],
            drag: 0.5,
            ..Spring::per_meter(1.0)
        };
        let slack = Spring {
            damping: [0.05, 0.05],
            drag: 0.0,
            ..springy
        };
        let mut worst = 0.0f32;
        for spring in [springy, slack] {
            for dt in [1.0f32 / 20.0, 1.0 / 30.0, 1.0 / 60.0, 1.0 / 144.0] {
                let mut sway = Sway::with(&sk, &[chain(&[1])], spring);
                let mut pose = sk.rest();
                sway.pose(dt, &mut pose);
                assert!((sway.length[0] - 0.2).abs() < 1e-6 && (sway.stiff[0] - 9.0 * 9.81 / 0.2).abs() < 1e-2);
                let start = 0.002;
                sway.at[0].x += start;
                let (stiff, damp) = (sway.stiff[0], sway.damp[0]);
                let w = (stiff - damp * damp / 4.0).sqrt();
                let mut swung = 0.0f32;
                for step in 1..=(2.0 / dt) as usize {
                    sway.pose(dt, &mut pose);
                    let t = step as f32 * dt;
                    let exact = start * (-damp * t / 2.0).exp() * ((w * t).cos() + damp / (2.0 * w) * (w * t).sin());
                    let got = sway.at[0].x - sway.goal[0].x;
                    worst = worst.max((got - exact).abs() / start);
                    swung = swung.max(-got / start);
                    // The bone is turned to where its tail is.
                    let tail = pose[1].rotation * Vec3::new(0.0, -0.2, 0.0);
                    assert!((tail - (sway.at[0] - Vec3::new(0.0, -0.2, 0.0))).length() < 1e-6);
                }
                assert!(swung > 0.15, "it swung past its goal: {swung}");
            }
        }
        println!("spring: {worst} of the offset it started with off the closed form at most");
        assert!(worst < 0.02, "{worst} of the offset off");
    }

    #[test]
    fn a_bone_let_be_never_gains_energy_and_a_chain_dies_down() {
        // A tail of four bones hanging and an ear of two standing; then a tail and an ear of one bone each. The body
        // is shoved a third of a meter in one pose and then still.
        let long = rig(&[
            (None, [0.0, 1.0, 0.0]),
            (Some(0), [0.0, -0.1, -0.1]),
            (Some(1), [0.0, -0.12, -0.03]),
            (Some(2), [0.0, -0.12, 0.0]),
            (Some(3), [0.0, -0.1, 0.02]),
            (Some(0), [0.1, 0.6, 0.0]),
            (Some(5), [0.02, 0.08, 0.0]),
        ]);
        let short = rig(&[
            (None, [0.0, 1.0, 0.0]),
            (Some(0), [0.0, -0.3, -0.1]),
            (Some(0), [0.1, 0.6, 0.0]),
        ]);
        let rigs = [
            (&long, vec![chain(&[1, 2, 3, 4]), chain(&[5, 6])]),
            (&short, vec![chain(&[1]), chain(&[2])]),
        ];
        for (sk, chains) in &rigs {
            let single = chains[0].bones.len() == 1;
            for dt in [0.5f32, 1.0 / 30.0, 1.0 / 60.0, 1.0 / 144.0] {
                let mut sway = Sway::with(sk, chains, Spring::per_meter(1.0));
                let mut pose = sk.rest();
                // Still for a pose first: a body moving when first seen is taken to have been moving all along.
                sway.pose(dt, &mut pose);
                sway.pose(dt, &mut pose);
                pose[0].translation.x += 0.3;
                sway.pose(dt, &mut pose);
                let mut was = sway.goal.clone();
                // The most energy in each half second, and the most it grows by in a step, over itself.
                let (mut last, mut grew) = (f32::INFINITY, 0.0f32);
                let mut peaks = vec![0.0f32; 12];
                for step in 0..(6.0 / dt) as usize {
                    sway.pose(dt, &mut pose);
                    let now = energy(&sway, &was, dt);
                    was.clone_from(&sway.goal);
                    // (Down where rounding is all that is left, it wanders.)
                    if now > 1e-8 * peaks[0] {
                        grew = grew.max(now / last - 1.0);
                    }
                    last = now;
                    let window = ((step + 1) as f32 * dt * 2.0 - 1e-3) as usize;
                    peaks[window.min(11)] = peaks[window.min(11)].max(now);
                }
                println!(
                    "energy, bones alone {single}, steps of {dt}: {:e} then {:e} then {:e}, growing by {grew:e}",
                    peaks[0], peaks[1], peaks[11]
                );
                assert!(peaks[0] > 0.005 && peaks[11] < 1e-6 * peaks[0], "{peaks:?}");
                for pair in peaks.windows(2) {
                    assert!(pair[1] < 0.5 * pair[0] || pair[1] < 1e-9 * peaks[0], "{peaks:?}");
                }
                // A bone alone, its goal still, has no more energy after any step than before. Down a chain a bone
                // is swung by the one above, which never feels it, so its own may rise for a while.
                assert!(
                    !single || grew <= 1e-5,
                    "a bone alone gained {grew} of its energy in a step"
                );
            }
        }
    }

    /// A person in meters, 1.8 tall, with arms out, fingers, legs and toes, a tail of four bones under the hips and
    /// a ponytail of three under the head, and those two as chains.
    fn person() -> (Skeleton, Vec<Chain>) {
        let mut bones = vec![
            (None, [0.0, 1.0, 0.0]),
            (Some(0), [0.0, 0.15, 0.0]),
            (Some(1), [0.0, 0.15, 0.0]),
            (Some(2), [0.0, 0.2, 0.0]),
            (Some(3), [0.0, 0.1, 0.0]),
            (Some(4), [0.0, 0.2, 0.0]),
        ];
        for side in [1.0f32, -1.0] {
            let at = bones.len();
            bones.extend([
                (Some(2), [0.05 * side, 0.15, 0.0]),
                (Some(at), [0.12 * side, 0.0, 0.0]),
                (Some(at + 1), [0.28 * side, 0.0, 0.0]),
                (Some(at + 2), [0.26 * side, 0.0, 0.0]),
                (Some(at + 3), [0.08 * side, 0.0, 0.03]),
                (Some(at + 4), [0.03 * side, 0.0, 0.0]),
                (Some(at + 3), [0.08 * side, 0.0, -0.03]),
                (Some(at + 6), [0.03 * side, 0.0, 0.0]),
            ]);
        }
        for side in [1.0f32, -1.0] {
            let at = bones.len();
            bones.extend([
                (Some(0), [0.09 * side, -0.05, 0.0]),
                (Some(at), [0.0, -0.45, 0.0]),
                (Some(at + 1), [0.0, -0.42, 0.0]),
                (Some(at + 2), [0.0, -0.08, 0.12]),
            ]);
        }
        let tail = bones.len();
        bones.extend([
            (Some(0), [0.0, -0.02, -0.08]),
            (Some(tail), [0.0, -0.08, -0.08]),
            (Some(tail + 1), [0.0, -0.1, -0.06]),
            (Some(tail + 2), [0.0, -0.1, -0.04]),
            (Some(4), [0.0, 0.12, -0.06]),
            (Some(tail + 4), [0.0, -0.04, -0.02]),
            (Some(tail + 5), [0.0, -0.1, -0.02]),
        ]);
        let chains = vec![
            chain(&[tail, tail + 1, tail + 2, tail + 3]),
            Chain {
                parent: 4,
                ..chain(&[tail + 4, tail + 5, tail + 6])
            },
        ];
        (rig(&bones), chains)
    }

    /// How far inside the body or under the floor any tail is past what it may be, over the rig's height, whether
    /// or not it is kept out: the most of any, none if all are clear.
    fn depth(sway: &Sway) -> f32 {
        let (rods, room) = (sway.rods.len(), ROOM * sway.height);
        let mut worst = 0.0f32;
        for (i, &at) in sway.at.iter().enumerate() {
            let thick = (0.2 * sway.length[i]).min(THICK * sway.height);
            worst = worst.max((sway.floor + thick).min(sway.rest_y[i] - room) - at.y);
            let head = sway.heads[sway.bones[i]];
            for (k, &[a, b]) in sway.was.iter().enumerate() {
                let reach = (thick + sway.rods[k].radius).min(sway.apart[i * rods + k] - room);
                let reach = reach.min(head.distance(nearest(a, b, head)) + sway.ahead[i * rods + k]);
                worst = worst.max(reach - at.distance(nearest(a, b, at)));
            }
        }
        worst / sway.height
    }

    /// The person thrown about for three seconds: dropped and shoved to each side, the trunk bowing and the head
    /// nodding, so the tail is swung into the legs and the trunk and the ponytail into the head and the neck. With
    /// what `set` makes of the sway, the deepest any tail came into the body or the floor, and every chain bone's
    /// turn at every pose.
    fn thrown(set: impl Fn(&mut Sway)) -> (f32, Vec<u32>) {
        let (sk, chains) = person();
        let mut sway = Sway::with(&sk, &chains, Spring::per_meter(1.0));
        set(&mut sway);
        let (mut worst, mut bits) = (0.0f32, Vec::new());
        for f in 0..180 {
            let t = f as f32 / 60.0;
            let mut pose = sk.rest();
            pose[0].translation += Vec3::new(
                0.4 * (7.0 * t).sin(),
                -0.12 * (1.0 - (5.0 * t).cos()),
                0.3 * (4.0 * t).sin(),
            );
            pose[0].rotation = Quat::from_rotation_y(3.0 * t) * Quat::from_rotation_x(0.8 * (6.0 * t).sin());
            pose[1].rotation = Quat::from_rotation_x(0.6 * (9.0 * t).sin());
            pose[4].rotation = Quat::from_rotation_x(0.9 * (11.0 * t).sin());
            sway.pose(1.0 / 60.0, &mut pose);
            worst = worst.max(depth(&sway));
            bits.extend(
                sway.bones
                    .iter()
                    .flat_map(|&b| pose[b].rotation.to_array().map(f32::to_bits)),
            );
        }
        (worst, bits)
    }

    #[test]
    fn a_body_has_rods_and_rest_touches_none() {
        let (sk, chains) = person();
        let mut sway = Sway::with(&sk, &chains, Spring::per_meter(1.0));
        // Three along the spine and the neck, the head, and two for each arm and each leg.
        assert_eq!(sway.rods.len(), 12, "{:?}", sway.rods);
        // The ponytail starts inside the head, nearer it than the two are thick.
        let rods = sway.rods.len();
        let first = sway.starts[1];
        assert!(
            (0..rods).any(|k| sway.apart[first * rods + k] < sway.rods[k].radius),
            "the ponytail starts clear of the head"
        );
        for _ in 0..120 {
            let mut pose = sk.rest();
            sway.pose(1.0 / 60.0, &mut pose);
            assert_eq!(depth(&sway), 0.0);
            for (i, &b) in sway.bones.iter().enumerate() {
                let off = pose[b].rotation.angle_between(sway.rest[i]);
                assert!(off < 1e-4, "bone {b} is {off} radians off rest");
            }
        }
    }

    #[test]
    fn chains_thrown_about_stay_out_of_the_body_and_over_the_floor() {
        // With no cone to hold a bone back from getting out.
        let wide = |sway: &mut Sway| {
            for c in 0..2 {
                let spring = Spring {
                    limit: std::f32::consts::PI,
                    ..sway.spring(c)
                };
                sway.set_spring(c, spring);
            }
        };
        let (free, _) = thrown(|sway| {
            wide(sway);
            sway.collide(false, false);
        });
        let (kept, bits) = thrown(wide);
        let (coned, _) = thrown(|_| ());
        println!("thrown: {free} of the height into the body let be, {kept} kept out, {coned} with the cone");
        assert!(free > 0.03, "nothing was swung into the body: {free}");
        assert!(kept < 1e-4, "{kept} of the height inside");
        assert!(coned < 1e-4, "{coned} of the height inside with the cone");
        // And the same to the bit a second time.
        assert!(bits == thrown(wide).1);
    }

    #[test]
    fn a_skirt_whose_head_a_thigh_swings_through_isnt_turned_out() {
        // Two short bones hanging from the hips in front of a thigh, and two behind it. The thigh swings forward and
        // back past level, so its line goes through where the bones in front start.
        let (person, _) = person();
        let mut bones: Vec<(Option<usize>, [f32; 3])> = person
            .bones()
            .iter()
            .map(|b| (b.parent, b.rest.translation.to_array()))
            .collect();
        let skirt = bones.len();
        bones.extend([
            (Some(0), [0.09, -0.05, 0.06]),
            (Some(skirt), [0.0, -0.06, 0.0]),
            (Some(0), [0.09, -0.05, -0.06]),
            (Some(skirt + 2), [0.0, -0.06, 0.0]),
        ]);
        let sk = rig(&bones);
        let chains = [chain(&[skirt, skirt + 1]), chain(&[skirt + 2, skirt + 3])];
        let thigh = (0..sk.len())
            .find(|&b| sk.bones()[b].parent == Some(0) && sk.bones()[b].rest.translation.x > 0.05 && b < skirt)
            .unwrap();
        let limit = Spring::per_meter(1.0).limit;
        for swing in [1.2f32, 1.9] {
            let mut sway = Sway::with(&sk, &chains, Spring::per_meter(1.0));
            let mut most = 0.0f32;
            for f in 0..240 {
                let t = f as f32 / 60.0;
                let mut pose = sk.rest();
                pose[thigh].rotation = Quat::from_rotation_x(-swing * (3.0 * t).sin());
                sway.pose(1.0 / 60.0, &mut pose);
                for (i, &b) in sway.bones.iter().enumerate() {
                    most = most.max(pose[b].rotation.angle_between(sway.rest[i]));
                }
            }
            println!("skirt, thigh swung {swing} radians each way: bent {most} at most");
            // Laid along the thigh at most, and hardly past the cone. (Asked as far out as the thigh is thick, the
            // bones are bent 1.74 and 2.15 radians.)
            assert!(most > 0.5, "the thigh never reached the skirt: {most}");
            assert!(
                most < swing.min(limit) + 0.05,
                "bent {most} radians by a thigh swung {swing}"
            );
        }
    }

    /// A wave like a sine, a parabola for each half, and a turn about `axis` by about `angle` radians (right for
    /// small ones): with no library math, so the same to the bit on any machine.
    fn wave(x: f32) -> f32 {
        let t = x / std::f32::consts::PI;
        let t = t - 2.0 * (t / 2.0).floor();
        if t < 1.0 {
            4.0 * t * (1.0 - t)
        } else {
            -4.0 * (t - 1.0) * (2.0 - t)
        }
    }

    fn about(axis: Vec3, angle: f32) -> Quat {
        Quat::from_vec4((axis * (angle / 2.0)).extend(1.0)).normalize()
    }

    /// The person's pose `t` seconds into being thrown about much as in `thrown`, `hard` times as far.
    fn tossed(sk: &Skeleton, t: f32, hard: f32) -> Vec<Transform> {
        let mut pose = sk.rest();
        pose[0].translation += hard
            * Vec3::new(
                0.4 * wave(7.0 * t),
                -0.12 * (1.0 - wave(5.0 * t + 1.5)),
                0.3 * wave(4.0 * t),
            );
        pose[0].rotation = about(Vec3::Y, 1.5 * hard * wave(2.0 * t)) * about(Vec3::X, 0.8 * hard * wave(6.0 * t));
        pose[1].rotation = about(Vec3::X, 0.6 * hard * wave(9.0 * t));
        pose[4].rotation = about(Vec3::X, 0.9 * hard * wave(11.0 * t));
        pose
    }

    /// The most any chain bone's turn differs between two poses, in radians.
    fn differ(sway: &Sway, a: &[Transform], b: &[Transform]) -> f32 {
        let off = |&bone: &usize| {
            let off = a[bone].rotation.inverse() * b[bone].rotation;
            2.0 * off.xyz().length().atan2(off.w.abs())
        };
        sway.bones.iter().map(off).fold(0.0, f32::max)
    }

    #[test]
    fn the_step_sways_as_the_one_first_written_did() {
        // Three seconds of the person thrown about, gently and hard, at one step a pose, two, three and four, with
        // the body and the floor kept out of and not, and with no cone: the step against the one it was made from
        // (`before`), which it differs from by rounding alone. Every bone's turn at every pose is the same to a
        // hundred-thousandth of a radian, both after one pose from the same start and after all of them, each
        // going its own way (thrown hard, to three of those after all of them).
        //
        // But for a tail thrown into two parts of the body at once, the ponytail between the head and the neck.
        // There the first step is touchy by itself: a tail put out of one part stands right at its edge when that
        // part is looked at again, so rounding says which of the two it left last, and it is the last it stops
        // going into. The first step parts from itself as far, run again with its tails moved in their last bit:
        // counted here beside the poses this step parts from it.
        let (sk, chains) = person();
        let wide = Spring {
            limit: std::f32::consts::PI,
            ..Spring::per_meter(1.0)
        };
        let (mut worst, mut worst_through, mut parted_in_all) = (0.0f32, 0.0f32, [0, 0]);
        for hard in [0.25f32, 1.0] {
            for (collide, spring) in [
                (true, Spring::per_meter(1.0)),
                (false, Spring::per_meter(1.0)),
                (true, wide),
            ] {
                for rate in [60.0f32, 30.0, 24.0, 15.0] {
                    let mut sway = Sway::with(&sk, &chains, spring);
                    sway.collide(collide, collide);
                    let mut alone = sway.as_before();
                    let (mut each, mut through, mut bent) = (0.0f32, 0.0f32, 0.0f32);
                    let mut parted = [0, 0];
                    let poses = (3.0 * rate) as usize;
                    for f in 0..=poses {
                        // Held for a pose half way, as a pause is.
                        let dt = if f == 0 || f == poses / 2 { 0.0 } else { 1.0 / rate };
                        let posed = tossed(&sk, f as f32 / rate, hard);
                        // The first step from where this one stands, and again with every tail's last bit moved.
                        let (mut first, mut nudged) = (sway.as_before(), sway.as_before());
                        for (i, at) in nudged.at.iter_mut().enumerate() {
                            *at *= if (i + f) % 2 == 0 { 1.0 + 1.2e-7 } else { 1.0 - 1.2e-7 };
                        }
                        let (mut pose, mut was, mut other) = (posed.clone(), posed.clone(), posed.clone());
                        sway.pose(dt, &mut pose);
                        first.pose_before(dt, &mut was);
                        nudged.pose_before(dt, &mut other);
                        // Apart in a pose: a bone turned, a tail placed (in meters) or its speed.
                        let far = |a: &Sway, turned: f32| {
                            let at = (0..a.at.len()).any(|i| a.at[i].distance(first.at[i]) >= 2e-6);
                            let speed = (0..a.at.len()).any(|i| a.speed[i].distance(first.speed[i]) >= 2e-4 * rate);
                            turned >= 1e-5 || at || speed
                        };
                        let turned = differ(&sway, &pose, &was);
                        if far(&sway, turned) {
                            parted[0] += 1;
                        } else {
                            each = each.max(turned);
                        }
                        parted[1] += usize::from(far(&nudged, differ(&sway, &other, &was)));
                        for (i, &b) in sway.bones.iter().enumerate() {
                            let now = pose[b].rotation;
                            assert!((now.length() - 1.0).abs() < 1e-6, "bone {b} turned by {now:?}");
                            bent = bent.max(now.angle_between(sway.rest[i]));
                        }
                        alone.pose_before(dt, &mut was);
                        through = through.max(differ(&sway, &pose, &was));
                    }
                    println!(
                        "thrown {hard}, kept out {collide}, limit {:.2}, {rate} poses a second, bent {bent:.2}: \
                         {each:.1e} radians off the first step after a pose, {through:.1e} after all of them; \
                         parted in {} poses of {poses}, the first step from itself in {}",
                        spring.limit, parted[0], parted[1]
                    );
                    assert!(bent > 0.2, "nothing swung: {bent}");
                    if hard == 1.0 && collide {
                        assert!(
                            parted[0] <= poses / 4,
                            "parted from the first step in {} poses",
                            parted[0]
                        );
                        parted_in_all = [parted_in_all[0] + parted[0], parted_in_all[1] + parted[1]];
                    } else {
                        // Thrown hard, rounding has more to work on: three times as much is let by.
                        let near = if hard < 1.0 { 1e-5 } else { 3e-5 };
                        assert_eq!(parted[0], 0, "parted from the first step in a pose");
                        assert!(through < near, "{through} radians off the first step");
                        worst_through = worst_through.max(through);
                    }
                    worst = worst.max(each);
                }
            }
        }
        println!(
            "the step against the first: {worst:e} radians apart after a pose, {worst_through:e} after three seconds; \
             thrown hard into the body, parted in {} poses, the first step from itself in {}",
            parted_in_all[0], parted_in_all[1]
        );
    }

    #[test]
    #[should_panic(expected = "doesn't hang from")]
    fn a_chain_out_of_order_is_refused() {
        let sk = rig(&[
            (None, [0.0; 3]),
            (Some(0), [0.0, -0.2, 0.0]),
            (Some(1), [0.0, -0.2, 0.0]),
        ]);
        let _ = Sway::with(&sk, &[chain(&[2, 1])], Spring::per_meter(1.0));
    }
}
