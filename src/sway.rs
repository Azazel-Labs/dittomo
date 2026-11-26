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

use glam::{Affine3A, Quat, Vec3};

use crate::shape::Shape;
use crate::{Error, Mapping, Skeleton, Transform, contact, held, transfer};

/// The longest step the spring is advanced by, in seconds: a longer time between poses is split into equal steps.
const STEP: f32 = 1.0 / 60.0;
/// The most steps one pose is split into.
const STEPS: f32 = 4.0;
/// A chain shorter than this share of the rig's height is left out, and so is a bone this share of that nearer its
/// parent: the first is a helper or a marker, the second has no length to swing.
const SHORT: f32 = 0.01;
const NO_LENGTH: f32 = 1e-4;

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
    /// How fast a bone swings back to its rest pose, beside a pendulum as long as the chain is from that bone to its
    /// tip: 1 swings as fast as the pendulum, 2 twice as fast. First for a bone hanging straight down at rest, then
    /// for one standing straight up, which needs more to stand; a bone between the two takes its share of each.
    pub stiffness: [f32; 2],
    /// How soon a swing dies out: 0 swings on, 1 comes to rest without swinging past. For a bone hanging down, then
    /// for one standing up, as for `stiffness`.
    pub damping: [f32; 2],
    /// How much the still air slows a bone, a second: it lags a little more behind a body moving fast.
    pub drag: f32,
    /// How far a bone may bend from its rest pose under the bone above it, in radians.
    pub limit: f32,
}

impl Spring {
    /// The spring for a rig this many of its units to the meter: 1 for meters, 100 for centimeters.
    #[must_use]
    pub fn per_meter(units: f32) -> Self {
        Self {
            gravity: 9.81 * units,
            stiffness: [1.0, 3.0],
            damping: [0.45, 0.7],
            drag: 0.5,
            limit: 75f32.to_radians(),
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
    /// The bone's rest turn under its parent.
    rest: Vec<Quat>,
    /// Which way its tail is from its head, in its own frame, and how far.
    axis: Vec<Vec3>,
    length: Vec<f32>,
    /// How long the chain is from this bone's head to the tip, how far up the bone points at rest (0 straight down,
    /// 1 straight up), and which way down is in its parent's frame at rest: what a [`Spring`] is fitted to.
    left: Vec<f32>,
    lift: Vec<f32>,
    down: Vec<Vec3>,
    /// The spring's pull for each unit off the goal, and one over it.
    stiff: Vec<f32>,
    loose: Vec<f32>,
    /// The spring's damping and the air's together, and the air's alone.
    damp: Vec<f32>,
    drag: Vec<f32>,
    gravity: Vec<f32>,
    /// The cosine and sine of how far it may bend.
    cos: Vec<f32>,
    sin: Vec<f32>,
    /// How the spring's offset and speed are carried over one step of `step` seconds.
    carry: Vec<[f32; 4]>,
    /// Where the bone's tail is, how fast it moves, and where its goal was a step ago.
    at: Vec<Vec3>,
    speed: Vec<Vec3>,
    goal: Vec<Vec3>,
    /// Which way the rest pose's down was under the bone above, a step ago.
    lean: Vec<Vec3>,

    step: f32,
    /// Whether a pose has been taken since the start or the last reset.
    live: bool,
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
        Ok(Self::with(target, &chains(map, target), Spring::of(target)))
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
        let n = target.len();
        let mut chains = chains.to_vec();
        chains.sort_by_key(|c| c.bones.first().copied());
        let shape = Shape::light(target);
        let (pos, rot) = (|b: usize| shape.rest[b].position(), |b: usize| shape.rest[b].rotation);
        let parent = |b: usize| target.bones()[b].parent;
        let mut chained = vec![false; n];
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
            axis: Vec::new(),
            length: Vec::new(),
            left: Vec::new(),
            lift: Vec::new(),
            down: Vec::new(),
            stiff: Vec::new(),
            loose: Vec::new(),
            damp: Vec::new(),
            drag: Vec::new(),
            gravity: Vec::new(),
            cos: Vec::new(),
            sin: Vec::new(),
            carry: Vec::new(),
            at: Vec::new(),
            speed: Vec::new(),
            goal: Vec::new(),
            lean: Vec::new(),
            step: f32::NAN,
            live: false,
            chains: Vec::new(),
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
                sway.bones.push(b);
                sway.rest.push(target.bones()[b].rest.rotation.normalize());
                sway.axis.push(rot(b).inverse() * (along / length));
                sway.length.push(length);
                sway.lift.push((1.0 + along.y / length) / 2.0);
                sway.down.push(rot(up).inverse() * Vec3::NEG_Y);
                up = b;
            }
            let first = sway.left.len();
            sway.left.extend(&sway.length[first..]);
            for i in (first..sway.left.len() - 1).rev() {
                sway.left[i] += sway.left[i + 1];
            }
            chain.length = sway.left[first];
            sway.offsets
                .push(rot(chain.parent).inverse() * (pos(chain.bones[0]) - pos(chain.parent)));
            sway.starts.push(sway.bones.len());
        }
        // The bones above the chains that aren't in one, to place from each pose.
        let mut needed = vec![false; n];
        for chain in &chains {
            let mut up = Some(chain.parent);
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
        for list in [&mut sway.stiff, &mut sway.loose, &mut sway.damp, &mut sway.drag] {
            list.resize(all, 0.0);
        }
        for list in [&mut sway.gravity, &mut sway.cos, &mut sway.sin] {
            list.resize(all, 0.0);
        }
        sway.carry.resize(all, [0.0; 4]);
        for list in [&mut sway.at, &mut sway.speed, &mut sway.goal, &mut sway.lean] {
            list.resize(all, Vec3::ZERO);
        }
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
            let stiff = (quick * quick * spring.gravity / self.left[i]).max(1e-6);
            let drag = spring.drag.max(0.0);
            self.stiff[i] = stiff;
            self.loose[i] = 1.0 / stiff;
            self.damp[i] = 2.0 * blend(spring.damping).max(0.0) * stiff.sqrt() + drag;
            self.drag[i] = drag;
            self.gravity[i] = spring.gravity;
            (self.sin[i], self.cos[i]) = (sin, cos);
        }
        // The steps carried are worked out again at the next pose.
        self.step = f32::NAN;
    }

    /// Forgets where the chains are and how they move: the next pose puts them at rest under the body as it is then,
    /// as after a cut to another place or another clip.
    pub fn reset(&mut self) {
        self.live = false;
    }

    /// Sways the chains `dt` seconds on from the last pose. `pose` is the target's, each transform relative to its
    /// parent, as played and fixed up: the body is read from it, and each chain bone's rotation is written to it
    /// (whatever it was: a chain's goal is its rest pose under the bone above). The first pose, the first after
    /// [`Sway::reset`] and any with no time passed put the chains at rest under the body, still.
    ///
    /// The bones' scales are taken to be as at rest.
    ///
    /// # Panics
    /// If `pose` doesn't have one transform per bone.
    pub fn pose(&mut self, dt: f32, pose: &mut [Transform]) {
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
        let moving = self.live && dt.is_finite() && dt > 0.0;
        let steps = if moving {
            (dt / STEP - 1e-3).ceil().clamp(1.0, STEPS)
        } else {
            1.0
        };
        let h = if moving { dt / steps } else { self.step };
        if moving && h != self.step {
            for i in 0..self.bones.len() {
                self.carry[i] = carry(h, self.stiff[i], self.damp[i]);
            }
            self.step = h;
        }
        let per_h = 1.0 / h;
        for c in 0..self.chains.len() {
            let hangs = self.chains[c].parent;
            let (turn, start) = (self.rots[hangs], self.heads[hangs] + self.rots[hangs] * self.offsets[c]);
            let range = self.starts[c]..self.starts[c + 1];
            if !moving {
                let (mut head, mut up) = (start, turn);
                for i in range {
                    let b = self.bones[i];
                    let turned = (up * self.rest[i]).normalize();
                    let goal = head + self.length[i] * (turned * self.axis[i]);
                    (self.at[i], self.speed[i], self.goal[i]) = (goal, Vec3::ZERO, goal);
                    self.lean[i] = up * self.down[i];
                    pose[b].rotation = self.rest[i];
                    (self.rots[b], self.heads[b]) = (turned, head);
                    (head, up) = (goal, turned);
                }
                (self.root_at[c], self.root_turn[c]) = (start, turn);
                continue;
            }
            // The bone the chain hangs from goes from where it was to where it is, evenly through the steps.
            let (from_at, from_turn) = (self.root_at[c], self.root_turn[c]);
            let to_turn = if from_turn.dot(turn) < 0.0 { -turn } else { turn };
            let (mut k, mut root_was) = (1.0, from_at);
            while k <= steps {
                let last = k == steps;
                let (mut head, mut up) = if last {
                    (start, turn)
                } else {
                    let t = k / steps;
                    (from_at.lerp(start, t), from_turn.lerp(to_turn, t).normalize())
                };
                // Where this bone's head was a step ago, and how fast it moves now.
                let mut head_was = std::mem::replace(&mut root_was, head);
                let mut drift = (head - head_was) * per_h;
                for i in range.clone() {
                    // Where the tail would be with the bone at rest under the bone above, and how fast that moves.
                    let rested = up * self.rest[i];
                    let toward = rested * self.axis[i];
                    let goal = head + self.length[i] * toward;
                    let pace = (goal - self.goal[i]) * per_h;
                    // Gravity, less what the rest pose carries, and the air, as how far they hold the tail off its
                    // goal. What the rest pose carries turns with the bone above: half way between where that was
                    // a step ago and is now, as taking it where it is now would be late by half a step.
                    let lean = up * self.down[i];
                    let carried = 0.5 * (lean + std::mem::replace(&mut self.lean[i], lean));
                    let pull = self.gravity[i] * (Vec3::NEG_Y - carried) - self.drag[i] * pace;
                    let held = pull * self.loose[i];
                    // The spring, stepped as it is: how far the tail is off, and how fast that changes.
                    let (off, rate) = (self.at[i] - self.goal[i] - held, self.speed[i] - pace);
                    let [a, b, c, d] = self.carry[i];
                    let free = goal + (a * off + b * rate) + held;
                    let speed = c * off + d * rate + pace;
                    // Back at the bone's length from its head, within the cone about its rest.
                    let out = free - head;
                    let far = out.length_squared();
                    let mut dir = if far > 1e-12 * self.length[i] * self.length[i] {
                        out / far.sqrt()
                    } else {
                        toward
                    };
                    // The bone holds the tail in along its line, so about its head the tail keeps only its speed
                    // across the bone: across the bone as it was, carried round to the bone as it is, beside the
                    // head as it moves now. (Dropping what is along the bone as it is now would slow a swing by how
                    // far it turned in the step: more the longer the step, so by the frame rate.)
                    let was = self.at[i];
                    let before = (was - head_was) / self.length[i];
                    let across = speed - drift;
                    let across = across - across.dot(before) * before;
                    let round = |dir: Vec3| {
                        across - (across.dot(dir) / (1.0 + before.dot(dir)).max(1e-6)) * (before + dir) + drift
                    };
                    let bent = dir.dot(toward);
                    let speed = if bent < self.cos[i] {
                        let side = dir - bent * toward;
                        let wide = side.length_squared();
                        let side = if wide > 1e-12 {
                            side / wide.sqrt()
                        } else {
                            toward.any_orthonormal_vector()
                        };
                        dir = self.cos[i] * toward + self.sin[i] * side;
                        // At the cone's edge it stops going outward, beside the goal.
                        let (speed, outward) = (round(dir), self.cos[i] * side - self.sin[i] * toward);
                        speed - (speed - pace).dot(outward).max(0.0) * outward
                    } else {
                        round(dir)
                    };
                    let at = head + self.length[i] * dir;
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
        self.live = true;
    }
}

/// The shortest turn taking one direction to another, with no library math. (glam's own gives no turn at all under
/// a twenty-fifth of a degree, which would hold a bone nearly at rest dead still.)
fn arc(from: Vec3, to: Vec3) -> Quat {
    let (axis, w) = (from.cross(to), 1.0 + from.dot(to));
    let long = w * w + axis.length_squared();
    if long > 1e-12 {
        let by = 1.0 / long.sqrt();
        Quat::from_xyzw(axis.x * by, axis.y * by, axis.z * by, w * by)
    } else {
        // Straight back: half a turn about any line across.
        let across = from.any_orthonormal_vector();
        Quat::from_xyzw(across.x, across.y, across.z, 0.0)
    }
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
        // One bone hanging 0.2 m under a still body, let go a hundredth of its length to the side.
        let sk = rig(&[(None, [0.0; 3]), (Some(0), [0.0, -0.2, 0.0])]);
        let slack = Spring {
            damping: [0.05, 0.05],
            drag: 0.0,
            ..Spring::per_meter(1.0)
        };
        let mut worst = 0.0f32;
        for spring in [Spring::per_meter(1.0), slack] {
            for dt in [1.0f32 / 20.0, 1.0 / 30.0, 1.0 / 60.0, 1.0 / 144.0] {
                let mut sway = Sway::with(&sk, &[chain(&[1])], spring);
                let mut pose = sk.rest();
                sway.pose(dt, &mut pose);
                assert!((sway.length[0] - 0.2).abs() < 1e-6 && (sway.stiff[0] - 9.81 / 0.2).abs() < 1e-3);
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
                assert!(peaks[0] > 0.01 && peaks[11] < 1e-6 * peaks[0], "{peaks:?}");
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
