//! Playing a clip on the target through a mapping.
//!
//! A mapped bone takes its source bone's turn away from rest in the world, carried into the target's frame and
//! corrected for the two rest poses: at rest the source bone and its next mapped bones point one way, the target's
//! another, and the turn that best takes the one set of directions to the other (`fix`) is undone first. So the
//! target's bones point where the source's do, frame by frame, whatever each rig's rest pose and bone axes; the spine,
//! head, collarbones and feet, which every rig holds alike at rest, are matched rest to rest (`plan`). A bone between
//! two mapped bones takes its share of their turns by how far along it is (only of the twist, if the bone below is a
//! limb's joint), and one beside a mapped bone, starting where it starts
//! and pointing along it, turns with it; any other keeps its rest pose under its parent. The topmost mapped bones and
//! any whose source moves (not just turns) travel as their source does, scaled by how high each stands; the topmost
//! across the floor by how high the legs start, as far as they step.
//!
//! [`Player`] does the work that depends only on the two rigs and the mapping once, then poses frame after frame without
//! allocating. Rotations are chained as quaternions; whole transforms are chained only down the paths to bones that
//! travel, since only their positions are needed.

mod crowd;

use glam::{Affine3A, Mat3, Quat, Vec3};

use crate::contact;
use crate::held::{self, Held};
use crate::shape::Shape;
use crate::{Clip, Error, Mapping, Placed, Skeleton, Transform};

pub use crowd::Crowd;

#[derive(Clone, Copy, Debug, PartialEq)]
enum Plan {
    Rest,
    Mapped {
        source: usize,
        /// The source bone's rest turn, the bodies' turn and the rest poses' difference, undone in one.
        pre: Quat,
        /// How far the bone travels for each unit its source does, along each axis, if it travels.
        travel: Option<Vec3>,
    },
    Between {
        above: usize,
        below: usize,
        along: f32,
        /// Where the bone below is a limb's joint, the line from this bone to it at rest: the bone keeps the turn of
        /// the one above, taking only its share of the twist about this line.
        twist: Option<Vec3>,
    },
}

/// Plays poses of one skeleton on another through a [`Mapping`], a frame at a time, without allocating.
///
/// ```
/// # use dittomo::{Bone, Mapping, Player, Skeleton, Transform, Vec3};
/// # let bone = |name: &str, parent| Bone { name: name.into(), parent, rest: Transform::new(Vec3::Y, Default::default()) };
/// # let rig = Skeleton::new(vec![bone("hips", None), bone("spine", Some(0))]).unwrap();
/// let mapping = Mapping::new(&rig, &rig);
/// let mut player = Player::new(&mapping, &rig, &rig).unwrap();
/// let mut out = rig.rest();
/// player.pose(&rig.rest(), &mut out).unwrap();
/// ```
#[derive(Clone, Debug)]
pub struct Player {
    turn: Quat,
    plans: Vec<Plan>,
    // The source.
    s_parent: Vec<Option<usize>>,
    /// Source bones whose world rotation is needed.
    s_rot_need: Vec<bool>,
    /// Source bones whose whole world transform is needed: those that travel and the bones above them.
    s_mat_need: Vec<bool>,
    s_rest_pos: Vec<Vec3>,
    s_rest_translation: Vec<Vec3>,
    s_rest_rotation: Vec<Quat>,
    /// Source bones some target bone plays.
    s_played: Vec<bool>,
    /// For each source bone, the nearest bone above it some target bone plays, and where the bone stands from it at
    /// rest, in its frame.
    s_anchor: Vec<Option<(usize, Vec3)>>,
    s_height: f32,
    // The target.
    t_parent: Vec<Option<usize>>,
    t_rest: Vec<Transform>,
    t_rest_rot: Vec<Quat>,
    t_rest_pos: Vec<Vec3>,
    t_mat_need: Vec<bool>,
    /// For each source bone: whether it moves in the clip being played, so travels.
    moves: Vec<bool>,
    /// The mapped target bones that travel only if their source moves: (target, source, how far).
    moving: Vec<(usize, usize, f32)>,
    // Scratch, one entry per bone.
    /// Each source bone's turn in the world, turned into the target's frame.
    s_rot: Vec<Quat>,
    s_mat: Vec<Affine3A>,
    turned: Vec<Quat>,
    t_rot: Vec<Quat>,
    t_mat: Vec<Affine3A>,
    /// Feet held from the root in the source and in the target, played as the ends of their legs ([`held`]), and the
    /// source's and target's poses with their held feet hung.
    held: [Option<Held>; 2],
    hung: [Vec<Transform>; 2],
}

impl Player {
    /// A player for this mapping between these skeletons. Only the topmost mapped bones travel; a clip whose other
    /// bones move as well as turn (a prop bone, say) is told with [`Player::moving_in`].
    ///
    /// # Errors
    /// [`Error::NotTheirMapping`] if the mapping was made for skeletons of other sizes.
    pub fn new(map: &Mapping, source: &Skeleton, target: &Skeleton) -> Result<Self, Error> {
        if source.len() != map.sources() || target.len() != map.pairs().len() {
            return Err(Error::NotTheirMapping);
        }
        let held = [held::held(source), held::held(target)];
        let source = held[0].as_ref().map_or(source, |h| &h.skeleton);
        let target = held[1].as_ref().map_or(target, |h| &h.skeleton);
        let map = map.hung([held[0].as_ref(), held[1].as_ref()]);
        let mut player = Self::of_shapes(&map, source, target, &Shape::light(source), &Shape::light(target));
        player.held = held;
        Ok(player)
    }

    /// [`Player::new`], with each rig's shape already worked out, and the sizes already checked.
    pub(crate) fn of_shapes(map: &Mapping, source: &Skeleton, target: &Skeleton, s: &Shape, t: &Shape) -> Self {
        let (plans, moving) = plan(map, [(source, s), (target, t)]);
        let turn = map.turn();
        let back = turn.inverse();
        let (ns, nt) = (source.len(), target.len());
        let plans: Vec<Plan> = plans
            .into_iter()
            .map(|p| match p {
                Plan::Mapped {
                    source: a,
                    pre: unfix,
                    travel,
                } => Plan::Mapped {
                    source: a,
                    pre: (s.rest[a].rotation.inverse() * back * unfix).normalize(),
                    travel,
                },
                p => p,
            })
            .collect();
        let mut s_played = vec![false; ns];
        for p in &plans {
            if let Plan::Mapped { source: a, .. } = *p {
                s_played[a] = true;
            }
        }
        let s_parent: Vec<Option<usize>> = source.bones().iter().map(|b| b.parent).collect();
        let s_anchor = (0..ns)
            .map(|a| {
                let mut up = s_parent[a];
                while let Some(p) = up {
                    if s_played[p] {
                        let off = s.rest[p].matrix.inverse().transform_point3(s.rest[a].position());
                        return Some((p, off));
                    }
                    up = s_parent[p];
                }
                None
            })
            .collect();
        let mut player = Self {
            turn,
            plans,
            s_parent,
            s_rot_need: vec![false; ns],
            s_mat_need: vec![false; ns],
            s_rest_pos: s.rest.iter().map(Placed::position).collect(),
            s_rest_translation: source.bones().iter().map(|b| b.rest.translation).collect(),
            s_rest_rotation: source.bones().iter().map(|b| b.rest.rotation).collect(),
            s_played,
            s_anchor,
            s_height: s.height,
            t_parent: target.bones().iter().map(|b| b.parent).collect(),
            t_rest: target.rest(),
            t_rest_rot: t.rest.iter().map(|p| p.rotation.normalize()).collect(),
            t_rest_pos: t.rest.iter().map(Placed::position).collect(),
            t_mat_need: vec![false; nt],
            moves: vec![false; ns],
            moving,
            s_rot: vec![Quat::IDENTITY; ns],
            s_mat: vec![Affine3A::IDENTITY; ns],
            turned: vec![Quat::IDENTITY; nt],
            t_rot: vec![Quat::IDENTITY; nt],
            t_mat: vec![Affine3A::IDENTITY; nt],
            held: [None, None],
            hung: [Vec::new(), Vec::new()],
        };
        player.needs();
        player
    }

    /// Marks the source bones that move, not just turn, somewhere in this clip, so the bones playing them travel too.
    /// A bone moves when it does under its parent, or when a bone above it that no target bone plays moves or turns,
    /// carrying it: a body bone between the root and the hips (a four-legged rig's) rolling the animal onto its side
    /// lowers its hips, which the target's must follow. [`Mapping::apply`] does this for the clip it plays.
    ///
    /// # Errors
    /// If the clip doesn't pose the source.
    pub fn moving_in(&mut self, clip: &Clip) -> Result<&mut Self, Error> {
        clip.check(self.s_parent.len())?;
        let hung = self.held[0].as_ref().map(|h| h.hang_clip(clip));
        let clip = hung.as_ref().unwrap_or(clip);
        let ns = self.s_parent.len();
        let (mut shifts, mut turns) = (vec![false; ns], vec![false; ns]);
        for a in 0..ns {
            let (t, r) = (self.s_rest_translation[a], self.s_rest_rotation[a]);
            shifts[a] = clip
                .poses
                .iter()
                .any(|p| p[a].translation.distance(t) > 1e-4 * self.s_height);
            turns[a] = clip.poses.iter().any(|p| p[a].rotation.dot(r).abs() < 1.0 - 1e-7);
        }
        for a in 0..ns {
            let mut carried = shifts[a];
            let mut up = self.s_parent[a];
            while let Some(p) = up
                && !carried
                && !self.s_played[p]
            {
                carried = shifts[p] || turns[p];
                up = self.s_parent[p];
            }
            self.moves[a] = carried;
        }
        self.needs();
        Ok(self)
    }

    /// Works out which bones' world rotations and whole transforms each frame needs.
    fn needs(&mut self) {
        fn up(need: &mut [bool], parent: &[Option<usize>], mut b: usize) {
            while !std::mem::replace(&mut need[b], true) {
                match parent[b] {
                    Some(p) => b = p,
                    None => break,
                }
            }
        }
        self.s_rot_need.fill(false);
        self.s_mat_need.fill(false);
        self.t_mat_need.fill(false);
        for &(b, a, k) in &self.moving {
            if let Plan::Mapped { travel, .. } = &mut self.plans[b] {
                *travel = self.moves[a].then_some(Vec3::splat(k));
            }
        }
        for (b, p) in self.plans.iter().enumerate() {
            if let Plan::Mapped { source: a, travel, .. } = *p {
                up(&mut self.s_rot_need, &self.s_parent, a);
                if travel.is_some() {
                    up(&mut self.s_mat_need, &self.s_parent, a);
                    if let Some(q) = self.t_parent[b] {
                        up(&mut self.t_mat_need, &self.t_parent, q);
                    }
                }
            }
        }
    }

    /// Poses the target from a pose of the source, each transform relative to its parent.
    ///
    /// # Errors
    /// [`Error::PoseSize`] if either pose doesn't have one transform per bone.
    pub fn pose(&mut self, source: &[Transform], out: &mut [Transform]) -> Result<(), Error> {
        let (ns, nt) = (self.s_parent.len(), self.t_parent.len());
        for (got, expected) in [(source.len(), ns), (out.len(), nt)] {
            if got != expected {
                return Err(Error::PoseSize {
                    frame: 0,
                    bones: got,
                    expected,
                });
            }
        }
        let mut hung = std::mem::take(&mut self.hung);
        let [s_hung, t_hung] = &mut hung;
        let source = match &self.held[0] {
            Some(h) => {
                s_hung.clear();
                s_hung.extend_from_slice(source);
                h.hang(source, s_hung);
                &s_hung[..]
            }
            None => source,
        };
        if let Some(h) = self.held[1].take() {
            t_hung.clear();
            t_hung.extend_from_slice(out);
            h.hang(out, t_hung);
            self.pose_hung(source, t_hung);
            h.unhang(t_hung, out);
            self.held[1] = Some(h);
        } else {
            self.pose_hung(source, out);
        }
        self.hung = hung;
        Ok(())
    }

    /// [`Player::pose`], with any held feet hung from their legs in both poses.
    fn pose_hung(&mut self, source: &[Transform], out: &mut [Transform]) {
        let nt = self.t_parent.len();
        let ns = self.s_parent.len();
        // The source in the world, as far as needed, its turns in the target's frame. Each turn is made whole before
        // it is chained, not after, so the bones down a chain needn't wait on it.
        let turn = self.turn;
        for a in 0..ns {
            if !self.s_rot_need[a] {
                continue;
            }
            let local = &source[a];
            match self.s_parent[a] {
                Some(p) => {
                    self.s_rot[a] = self.s_rot[p] * local.rotation.normalize();
                    if self.s_mat_need[a] {
                        self.s_mat[a] = self.s_mat[p] * affine(local);
                    }
                }
                None => {
                    self.s_rot[a] = turn * local.rotation.normalize();
                    if self.s_mat_need[a] {
                        self.s_mat[a] = affine(local);
                    }
                }
            }
        }
        for (b, p) in self.plans.iter().enumerate() {
            if let Plan::Mapped { source: a, pre, .. } = *p {
                self.turned[b] = self.s_rot[a] * pre;
            }
        }
        for b in 0..nt {
            let (up_r, up_m) = match self.t_parent[b] {
                Some(q) => (self.t_rot[q], self.t_mat[q]),
                None => (Quat::IDENTITY, Affine3A::IDENTITY),
            };
            let rest = self.t_rest[b];
            // A bone's turn in the world is known before its parent's, but for a bone kept at rest.
            let (rotation, translation, world) = match self.plans[b] {
                Plan::Rest => (rest.rotation, rest.translation, up_r * rest.rotation),
                Plan::Mapped { source: a, travel, .. } => {
                    let w = self.turned[b] * self.t_rest_rot[b];
                    let at = travel.map_or(rest.translation, |k| {
                        let now = Vec3::from(self.s_mat[a].translation);
                        match self.s_anchor[a] {
                            // How far the source has come from where it stands at rest from the bone above it that
                            // is played, carried as that bone is turned now: a paw slipping off the end of its leg
                            // moves with the leg, wherever the leg has gone.
                            Some((p, off)) => {
                                let from = self.s_mat[p];
                                let moved =
                                    turn * from.transform_vector3(from.inverse().transform_point3(now) - off) * k;
                                rest.translation + up_m.inverse().transform_vector3(moved)
                            }
                            None => {
                                let moved = (turn * (now - self.s_rest_pos[a])) * k;
                                up_m.inverse().transform_point3(self.t_rest_pos[b] + moved)
                            }
                        }
                    });
                    (whole(up_r.inverse() * w), at, w)
                }
                Plan::Between {
                    above,
                    below,
                    along,
                    twist,
                } => {
                    let (a, z) = (self.turned[above], self.turned[below]);
                    let w =
                        twist.map_or_else(|| a.lerp(z, along), |axis| twisted(a, z, axis, along)) * self.t_rest_rot[b];
                    (whole(up_r.inverse() * w), rest.translation, w)
                }
            };
            let local = Transform {
                translation,
                rotation,
                scale: rest.scale,
            };
            self.t_rot[b] = world;
            if self.t_mat_need[b] {
                self.t_mat[b] = up_m * affine(&local);
            }
            out[b] = local;
        }
    }

    /// Feet held from the root, in the source and in the target.
    pub(crate) fn held(&self) -> &[Option<Held>; 2] {
        &self.held
    }
}

/// `above` turned on by `along` of the twist about `axis` that `below` has from it. A bone partway down to a limb's
/// joint lies along the bone above it in the source, so it turns with that one: blended toward the one below, a split
/// collarbone would take half the upper arm's raise and lift the shoulder, and a twist bone in a forearm half the
/// wrist's bend. A twist about the line to the joint leaves the joint where it is, and spreads a forearm's roll.
fn twisted(above: Quat, below: Quat, axis: Vec3, along: f32) -> Quat {
    let d = above.inverse() * below;
    let roll = Vec3::new(d.x, d.y, d.z).dot(axis) * axis;
    let t = Quat::from_xyzw(roll.x, roll.y, roll.z, d.w);
    let t = if t.length_squared() > 1e-12 {
        t.normalize()
    } else {
        Quat::IDENTITY
    };
    (above * Quat::IDENTITY.lerp(t, along)).normalize()
}

/// A turn that is whole but for rounding (a product of whole turns) made whole again, with a step of Newton's method
/// in place of a square root and a division.
fn whole(q: Quat) -> Quat {
    q * (1.5 - 0.5 * q.length_squared())
}

fn affine(t: &Transform) -> Affine3A {
    Affine3A::from_scale_rotation_translation(t.scale, t.rotation, t.translation)
}

pub(crate) fn play(map: &Mapping, source: &Skeleton, target: &Skeleton, clip: &Clip) -> Result<Clip, Error> {
    let mut player = Player::new(map, source, target)?;
    play_on(&mut player, target, clip)
}

/// Plays a clip with a player made for it.
pub(crate) fn play_on(player: &mut Player, target: &Skeleton, clip: &Clip) -> Result<Clip, Error> {
    player.moving_in(clip)?;
    let mut poses = Vec::with_capacity(clip.poses.len());
    let mut out = target.rest();
    for pose in &clip.poses {
        player.pose(pose, &mut out)?;
        poses.push(out.clone());
    }
    Ok(Clip {
        times: clip.times.clone(),
        poses,
    })
}

/// Each target bone's plan (a mapped bone's `pre` holding just its fix undone), and the mapped bones that aren't topmost
/// with how far each would travel if its source moved.
///
/// The neck and head are matched rest to rest, not by direction: where a head bone points to the top of the skull, or a
/// neck to the head, is anatomy (one rig's skull leans back, another's neck forward), but every rig looks ahead at
/// rest. Fitting their directions would tilt the target's gaze by the difference; matching rests keeps it looking where
/// the source looks. Feet and toes are matched rest to rest too, turned only about the vertical to point the same way:
/// every rig stands flat at rest, and how steeply the ankle comes down to the toes is anatomy (a high ankle over a
/// short foot), so fitting it would tip the target's toes into the floor or its heels off it. Limbs differ at rest by
/// pose (arms down or out), so they are matched by direction, but for the collarbones, matched rest to rest: how one
/// slopes up or back to the shoulder is anatomy. The spine, hips to chest, is matched rest to rest as
/// well: every rig stands upright at rest, and how far its chest sits ahead of its hips or behind, or where the legs
/// and arms hang from them, is anatomy too (one rig's back curves, another's hip joints sit behind its pelvis bone), so
/// fitting those would tip the pelvis or hunch the chest of a target standing as the source stands. So is any bone on
/// the body's middle in both that more than one limb hangs from, as a pelvis bone the spine doesn't reach: its children
/// fan out every way, and one rig's hips with the legs starting above them and another's with them below fit best with
/// a half turn, legs swapped and the hips twisted. A tail, on the middle but a limb of its own, goes the source's way.
fn plan(map: &Mapping, [(source, s), (target, t)]: [(&Skeleton, &Shape); 2]) -> (Vec<Plan>, Vec<(usize, usize, f32)>) {
    let n = t.children.len();
    let (sf, tf) = (contact::frame(source, s), contact::frame(target, t));
    let (s_spine, s_neck, s_collar) = (&sf.spine, &sf.above, &sf.collar);
    let (t_spine, t_neck, t_collar) = (&tf.spine, &tf.above, &tf.collar);
    let (s_feet, t_feet) = (feet(source, &sf.legs), feet(target, &tf.legs));
    let mut joint = vec![false; n];
    for l in tf.legs.iter().chain(&tf.arms) {
        for b in [l.upper, l.lower, l.end] {
            joint[b] = true;
        }
    }
    let turn = map.turn();
    let mapped = |b: usize| map.source_of(b);
    let left = map.left();
    // Whether a bone is on the body's middle, as far across it as the middle bone.
    let middle = |shape: &Shape, b: usize, left: Vec3| {
        (shape.rest[b].position() - shape.rest[shape.middle].position())
            .dot(left)
            .abs()
            < 0.01 * shape.height
    };
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

    let stands = |shape: &Shape, b: usize| shape.rest[b].position().y - shape.floor;
    let fallback = {
        let (hs, ht) = (stands(s, s.middle), stands(t, t.middle));
        if hs > 0.1 * s.height && ht > 0.1 * t.height {
            ht / hs
        } else {
            t.height / s.height
        }
    };

    // A step goes as far as the legs swing, so the body travels across the floor by how high each rig's legs start, if
    // both have legs, not its hips bone: a robot's hips bone sits below where its short legs start, and played on a
    // long-legged figure carried it further than its feet stepped, so they slid. Up and down it goes by how high the
    // bone stands.
    let stride = |shape: &Shape, legs: &[contact::Limb]| {
        let all: f32 = legs.iter().map(|l| stands(shape, l.upper)).sum();
        (!legs.is_empty() && all > 0.0).then(|| all / legs.len() as f32)
    };
    let across = stride(s, &sf.legs).zip(stride(t, &tf.legs)).map(|(a, b)| b / a);
    let mut fixes: Vec<Option<Quat>> = vec![None; n];
    let mut plans = vec![Plan::Rest; n];
    let mut moving = Vec::new();
    let mut s_mapped = vec![false; source.len()];
    for b in 0..n {
        if let Some(a) = mapped(b) {
            s_mapped[a] = true;
        }
    }
    let s_parent: Vec<Option<usize>> = source.bones().iter().map(|b| b.parent).collect();
    for b in 0..n {
        let Some(a) = mapped(b) else { continue };
        let mut thumb = None;
        let (pairs, below): (Vec<(Vec3, Vec3)>, Vec<usize>) = next[b]
            .iter()
            .filter_map(|&b2| {
                let a2 = mapped(b2)?;
                let u = turn * (s.rest[a2].position() - s.rest[a].position());
                let v = t.rest[b2].position() - t.rest[b].position();
                let long = u.length() > 1e-9 && v.length() > 1e-9;
                if long && map.thumbs(a2, b2) {
                    thumb = Some((u, v));
                }
                long.then_some(((u, v), a2))
            })
            .unzip();
        // The way a hand's fingers point is the way it points; its thumb, standing off them at an angle that differs
        // from rig to rig (a mitten's one finger, five spread), only turns it about that.
        let fingers = thumb.filter(|_| pairs.len() > 1).and_then(|(tu, tv)| {
            let sum = |f: fn(&(Vec3, Vec3)) -> Vec3| {
                pairs
                    .iter()
                    .filter(|p| **p != (tu, tv))
                    .map(|p| f(p).normalize())
                    .sum::<Vec3>()
            };
            let (u, v) = (sum(|p| p.0), sum(|p| p.1));
            let apart = |a: Vec3, b: Vec3| a.length() > 1e-6 && a.normalize().cross(b.normalize()).length() > 0.1;
            (apart(u, tu) && apart(v, tv)).then(|| align(u, tu, v, tv))
        });
        let d = driver(&s_parent, &s.children, &s_mapped, s, a, &below);
        let fix = match pairs.as_slice() {
            _ if s_neck[a] && t_neck[b] || s_spine.contains(&a) && t_spine.contains(&b) => Quat::IDENTITY,
            // A collarbone is matched rest to rest too: how it slopes up or back to the shoulder is anatomy, where
            // rigs differ at rest by pose in the upper arm (down or out), not there.
            _ if s_collar[a] && t_collar[b] => Quat::IDENTITY,
            _ if s_feet[a] && t_feet[b] => {
                let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
                pairs
                    .iter()
                    .map(|&(u, v)| (flat(u), flat(v)))
                    .find(|(u, v)| u.length() > 1e-9 && v.length() > 1e-9)
                    .map_or(Quat::IDENTITY, |(u, v)| {
                        Quat::from_rotation_y(u.cross(v).y.atan2(u.dot(v)))
                    })
            }
            [] => above(b).and_then(|p| fixes[p]).unwrap_or(Quat::IDENTITY),
            _ if fingers.is_some() => fingers.unwrap_or_default(),
            // As a fox's hips, its legs starting above them, onto a husky's, its legs starting below.
            _ if pairs.len() > 1 && middle(s, a, turn.inverse() * left) && middle(t, b, left) => Quat::IDENTITY,
            [(u, v)] => Quat::from_rotation_arc(u.normalize(), v.normalize()),
            [(u1, v1), rest @ ..] => {
                let second = rest.iter().find(|(u2, v2)| {
                    u1.normalize().cross(u2.normalize()).length() > 0.1
                        && v1.normalize().cross(v2.normalize()).length() > 0.1
                });
                let start = match second {
                    Some((u2, v2)) => align(*u1, *u2, *v1, *v2),
                    None => Quat::from_rotation_arc(u1.normalize(), v1.normalize()),
                };
                fit(&pairs, start)
            }
        };
        fixes[b] = Some(fix);
        let top = above(b).is_none();
        let k = {
            let (hs, ht) = (stands(s, a), stands(t, b));
            if hs > 0.1 * s.height && ht > 0.1 * t.height {
                ht / hs
            } else {
                fallback
            }
        };
        if !top {
            moving.push((b, a, k));
        }
        plans[b] = Plan::Mapped {
            source: d,
            pre: fix.inverse(),
            travel: top.then_some(Vec3::new(across.unwrap_or(k), k, across.unwrap_or(k))),
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
        let twist = joint[low].then(|| (t.rest[low].position() - t.rest[b].position()).normalize_or_zero());
        // Where the source's bone below hangs straight from its bone above, it bends there, at the joint of the one
        // below: a Biped's head turns on its chest, with no neck between. The target's bones between hold still
        // under the one above, its own joint below taking the bend, rather than each bending part way and leaning
        // the bone below off where the source's points.
        let direct = matches!((mapped(top), mapped(low)), (Some(sa), Some(sb)) if s_parent[sb] == Some(sa));
        plans[b] = Plan::Between {
            above: top,
            below: low,
            along: if direct && twist.is_none() { 0.0 } else { along },
            twist,
        };
    }
    // A bone with nothing mapped under it that starts where a mapped sibling starts and points along it, as VRoid's
    // sleeve and roll helpers do beside the upper arm, turns as that sibling does: kept at rest, a sleeve would stay
    // out in a T-pose as the arm comes down.
    for b in 0..n {
        if plans[b] != Plan::Rest || !next[b].is_empty() {
            continue;
        }
        let Some(p) = parent[b] else { continue };
        let at = t.rest[b].position();
        let reach = {
            let (mut far, mut stack) = (Vec3::ZERO, t.children[b].clone());
            while let Some(c) = stack.pop() {
                let d = t.rest[c].position() - at;
                if d.length() > far.length() {
                    far = d;
                }
                stack.extend(t.children[c].iter().copied());
            }
            far
        };
        let beside = t.children[p].iter().copied().find(|&l| {
            let Some(&low) = (l != b && mapped(l).is_some()).then(|| next[l].first()).flatten() else {
                return false;
            };
            let along = t.rest[low].position() - t.rest[l].position();
            let length = along.length();
            length > 1e-6
                && t.rest[l].position().distance(at) <= 0.1 * length
                && (reach.length() < 0.05 * length || reach.normalize().dot(along / length) > 0.87)
        });
        if let Some(l) = beside {
            plans[b] = Plan::Between {
                above: l,
                below: l,
                along: 0.0,
                twist: None,
            };
        }
    }
    (plans, moving)
}

/// The source bone whose turn places `a`'s mapped bones `below` it: `a` itself, or a bone under it that nothing plays,
/// standing where `a` does, that every one of them is under. A bone with no length places nothing, so the one under it
/// does: CMU's mocap turns its wrists' twist in the hand and their bend in a finger base starting at the wrist, with the
/// fingers under that; played from the hand alone, a target's hand kept its twist and lost its bend.
fn driver(
    parent: &[Option<usize>],
    children: &[Vec<usize>],
    mapped: &[bool],
    s: &Shape,
    a: usize,
    below: &[usize],
) -> usize {
    let at = s.rest[a].position();
    let mut d = a;
    while !below.is_empty() {
        let next = children[d].iter().copied().find(|&c| {
            !mapped[c]
                && s.rest[c].position().distance(at) <= 1e-4 * s.height
                && below.iter().all(|&a2| {
                    let mut up = Some(a2);
                    while let Some(x) = up {
                        if x == c {
                            return true;
                        }
                        up = parent[x];
                    }
                    false
                })
        });
        match next {
            Some(c) => d = c,
            None => break,
        }
    }
    d
}

/// Whether each bone is a foot (the end of a leg) or under one.
fn feet(skeleton: &Skeleton, legs: &[contact::Limb]) -> Vec<bool> {
    let mut feet = vec![false; skeleton.len()];
    for leg in legs {
        feet[leg.end] = true;
    }
    for (b, bone) in skeleton.bones().iter().enumerate() {
        if let Some(p) = bone.parent {
            feet[b] |= feet[p];
        }
    }
    feet
}

/// The rotation taking each `u` nearest its `v`'s direction, all at once (Wahba's problem), by Newton's method from
/// `start`, with the step of Müller, Bender, Chentanez and Macklin (2016) where Newton's isn't uphill. Proportions move
/// children differently on different rigs (one's collarbones beside its chest, another's above it), so no one child is
/// matched exactly; a turn about an axis every pair lies along is left as `start` has it.
fn fit(pairs: &[(Vec3, Vec3)], start: Quat) -> Quat {
    let mut q = start;
    for _ in 0..16 {
        // How much the directions agree is, to second order in a small turn w, f + g.w - w.A.w / 2.
        let (mut g, mut held, mut sym) = (Vec3::ZERO, 0.0, Mat3::ZERO);
        for (u, v) in pairs {
            let (r, v) = (q * u.normalize(), v.normalize());
            g += r.cross(v);
            held += r.dot(v);
            sym += Mat3::from_cols(r * v.x, r * v.y, r * v.z) + Mat3::from_cols(v * r.x, v * r.y, v * r.z);
        }
        let a = Mat3::from_diagonal(Vec3::splat(held * 1.001)) - sym * 0.5;
        let newton = (a.determinant().abs() > 1e-12)
            .then(|| a.inverse() * g)
            .filter(|w| w.dot(g) > 0.0);
        let w = newton.unwrap_or(g / (held.abs() + 1e-9));
        // Done once the step would gain next to nothing: about 0.002° short on an axis the pairs settle.
        if w.dot(g) < 1e-9 * pairs.len() as f32 {
            break;
        }
        let angle = w.length();
        q = (Quat::from_axis_angle(w / angle, angle.min(1.0)) * q).normalize();
    }
    q
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
