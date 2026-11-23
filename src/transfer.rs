//! Playing a clip on the target through a mapping.
//!
//! A mapped bone takes its source bone's turn away from rest in the world, carried into the target's frame and
//! corrected for the two rest poses: at rest the source bone and its next mapped bones point one way, the target's
//! another, and the turn that best takes the one set of directions to the other (`fix`) is undone first. So the target's bones point where the source's do, frame
//! by frame, whatever each rig's rest pose and bone axes. A bone between two mapped bones takes its share of their turns
//! by how far along it is; any other keeps its rest pose under its parent. The topmost mapped bones and any whose source
//! moves (not just turns) travel as their source does, scaled by how high each stands.
//!
//! [`Player`] does the work that depends only on the two rigs and the mapping once, then poses frame after frame without
//! allocating. Rotations are chained as quaternions; whole transforms are chained only down the paths to bones that
//! travel, since only their positions are needed.

use glam::{Affine3A, Mat3, Quat, Vec3};

use crate::contact;
use crate::shape::Shape;
use crate::{Clip, Error, Mapping, Placed, Skeleton, Transform};

#[derive(Clone, Copy, Debug, PartialEq)]
enum Plan {
    Rest,
    Mapped {
        source: usize,
        /// The source bone's rest turn, the bodies' turn and the rest poses' difference, undone in one.
        pre: Quat,
        /// How far the bone travels for each unit its source does, if it travels.
        travel: Option<f32>,
    },
    Between {
        above: usize,
        below: usize,
        along: f32,
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
    s_rot: Vec<Quat>,
    s_mat: Vec<Affine3A>,
    turned: Vec<Quat>,
    t_rot: Vec<Quat>,
    t_mat: Vec<Affine3A>,
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
        Ok(Self::of_shapes(
            map,
            source,
            target,
            &Shape::light(source),
            &Shape::light(target),
        ))
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
        let mut player = Self {
            turn,
            plans,
            s_parent: source.bones().iter().map(|b| b.parent).collect(),
            s_rot_need: vec![false; ns],
            s_mat_need: vec![false; ns],
            s_rest_pos: s.rest.iter().map(Placed::position).collect(),
            s_rest_translation: source.bones().iter().map(|b| b.rest.translation).collect(),
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
        };
        player.needs();
        player
    }

    /// Marks the source bones that move, not just turn, somewhere in this clip, so the bones playing them travel too.
    /// [`Mapping::apply`] does this for the clip it plays.
    ///
    /// # Errors
    /// If the clip doesn't pose the source.
    pub fn moving_in(&mut self, clip: &Clip) -> Result<&mut Self, Error> {
        clip.check(self.s_parent.len())?;
        for (a, m) in self.moves.iter_mut().enumerate() {
            let r = self.s_rest_translation[a];
            *m = clip
                .poses
                .iter()
                .any(|p| p[a].translation.distance(r) > 1e-4 * self.s_height);
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
                *travel = self.moves[a].then_some(k);
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
        // The source in the world, as far as needed.
        for a in 0..ns {
            if !self.s_rot_need[a] {
                continue;
            }
            let local = &source[a];
            match self.s_parent[a] {
                Some(p) => {
                    self.s_rot[a] = (self.s_rot[p] * local.rotation).normalize();
                    if self.s_mat_need[a] {
                        self.s_mat[a] = self.s_mat[p] * affine(local);
                    }
                }
                None => {
                    self.s_rot[a] = local.rotation.normalize();
                    if self.s_mat_need[a] {
                        self.s_mat[a] = affine(local);
                    }
                }
            }
        }
        let turn = self.turn;
        for (b, p) in self.plans.iter().enumerate() {
            if let Plan::Mapped { source: a, pre, .. } = *p {
                self.turned[b] = turn * self.s_rot[a] * pre;
            }
        }
        for b in 0..nt {
            let (up_r, up_m) = match self.t_parent[b] {
                Some(q) => (self.t_rot[q], self.t_mat[q]),
                None => (Quat::IDENTITY, Affine3A::IDENTITY),
            };
            let rest = self.t_rest[b];
            let (rotation, translation) = match self.plans[b] {
                Plan::Rest => (rest.rotation, rest.translation),
                Plan::Mapped { source: a, travel, .. } => {
                    let r = (up_r.inverse() * self.turned[b] * self.t_rest_rot[b]).normalize();
                    let at = travel.map_or(rest.translation, |k| {
                        let moved = turn * (Vec3::from(self.s_mat[a].translation) - self.s_rest_pos[a]) * k;
                        up_m.inverse().transform_point3(self.t_rest_pos[b] + moved)
                    });
                    (r, at)
                }
                Plan::Between { above, below, along } => {
                    let w = self.turned[above].lerp(self.turned[below], along) * self.t_rest_rot[b];
                    ((up_r.inverse() * w).normalize(), rest.translation)
                }
            };
            let local = Transform {
                translation,
                rotation,
                scale: rest.scale,
            };
            self.t_rot[b] = up_r * rotation;
            if self.t_mat_need[b] {
                self.t_mat[b] = up_m * affine(&local);
            }
            out[b] = local;
        }
        Ok(())
    }
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
/// neck to the head, is anatomy (one rig's skull leans back, another's neck forward), but every rig looks ahead at rest.
/// Fitting their directions would tilt the target's gaze by the difference; matching rests keeps it looking where the
/// source looks. Limbs differ at rest by pose (arms down or out), so they are matched by direction.
fn plan(map: &Mapping, [(source, s), (target, t)]: [(&Skeleton, &Shape); 2]) -> (Vec<Plan>, Vec<(usize, usize, f32)>) {
    let n = t.children.len();
    let (s_neck, t_neck) = (contact::above_chest(source, s), contact::above_chest(target, t));
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
    let mut moving = Vec::new();
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
            _ if s_neck[a] && t_neck[b] => Quat::IDENTITY,
            [] => above(b).and_then(|p| fixes[p]).unwrap_or(Quat::IDENTITY),
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
            source: a,
            pre: fix.inverse(),
            travel: top.then_some(k),
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
    (plans, moving)
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
