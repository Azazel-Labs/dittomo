//! Many characters played through one [`Player`] at once, side by side, so that the turns down each chain of bones are
//! worked out for several characters together rather than each waiting on the last.

use glam::{Affine3A, Quat, Vec3};

use super::{Plan, Player, affine};
use crate::{Error, Transform};

/// How many characters are posed side by side.
const LANES: usize = 8;

/// One turn for each of [`LANES`] characters, each part of them together.
#[derive(Clone, Copy, Debug)]
struct Turns {
    x: [f32; LANES],
    y: [f32; LANES],
    z: [f32; LANES],
    w: [f32; LANES],
}

impl Turns {
    const IDENTITY: Self = Self {
        x: [0.0; LANES],
        y: [0.0; LANES],
        z: [0.0; LANES],
        w: [1.0; LANES],
    };

    fn splat(q: Quat) -> Self {
        Self {
            x: [q.x; LANES],
            y: [q.y; LANES],
            z: [q.z; LANES],
            w: [q.w; LANES],
        }
    }

    fn each(f: impl Fn(usize) -> Quat) -> Self {
        let mut o = Self::IDENTITY;
        for i in 0..LANES {
            let q = f(i);
            (o.x[i], o.y[i], o.z[i], o.w[i]) = (q.x, q.y, q.z, q.w);
        }
        o
    }

    fn get(&self, i: usize) -> Quat {
        Quat::from_xyzw(self.x[i], self.y[i], self.z[i], self.w[i])
    }

    fn mul(&self, b: &Self) -> Self {
        let mut o = Self::IDENTITY;
        for i in 0..LANES {
            let (ax, ay, az, aw) = (self.x[i], self.y[i], self.z[i], self.w[i]);
            let (bx, by, bz, bw) = (b.x[i], b.y[i], b.z[i], b.w[i]);
            o.w[i] = aw * bw - ax * bx - ay * by - az * bz;
            o.x[i] = aw * bx + ax * bw + ay * bz - az * by;
            o.y[i] = aw * by - ax * bz + ay * bw + az * bx;
            o.z[i] = aw * bz + ax * by - ay * bx + az * bw;
        }
        o
    }

    /// Each turn undone, taking it to be whole.
    fn back(&self) -> Self {
        let mut o = *self;
        for i in 0..LANES {
            (o.x[i], o.y[i], o.z[i]) = (-o.x[i], -o.y[i], -o.z[i]);
        }
        o
    }

    fn scale(&mut self, k: impl Fn(usize) -> f32) {
        for i in 0..LANES {
            let k = k(i);
            (self.x[i], self.y[i], self.z[i], self.w[i]) = (self.x[i] * k, self.y[i] * k, self.z[i] * k, self.w[i] * k);
        }
    }

    fn length_squared(&self, i: usize) -> f32 {
        self.x[i] * self.x[i] + self.y[i] * self.y[i] + self.z[i] * self.z[i] + self.w[i] * self.w[i]
    }

    fn normalize(mut self) -> Self {
        let mut k = [0.0; LANES];
        for (i, k) in k.iter_mut().enumerate() {
            *k = 1.0 / self.length_squared(i).sqrt();
        }
        self.scale(|i| k[i]);
        self
    }

    /// As [`super::whole`].
    fn whole(mut self) -> Self {
        let mut k = [0.0; LANES];
        for (i, k) in k.iter_mut().enumerate() {
            *k = 1.5 - 0.5 * self.length_squared(i);
        }
        self.scale(|i| k[i]);
        self
    }

    /// As [`Quat::lerp`].
    fn lerp(&self, end: &Self, s: f32) -> Self {
        let mut o = *self;
        for i in 0..LANES {
            let dot = self.x[i] * end.x[i] + self.y[i] * end.y[i] + self.z[i] * end.z[i] + self.w[i] * end.w[i];
            let bias = if dot >= 0.0 { 1.0 } else { -1.0 };
            o.x[i] += (end.x[i] * bias - self.x[i]) * s;
            o.y[i] += (end.y[i] * bias - self.y[i]) * s;
            o.z[i] += (end.z[i] * bias - self.z[i]) * s;
            o.w[i] += (end.w[i] * bias - self.w[i]) * s;
        }
        o.normalize()
    }
}

/// Plays poses of one skeleton on another for many characters at once, through a [`Player`], without allocating.
/// Characters are posed eight at a time side by side, which is quicker than posing each in turn.
///
/// ```
/// # use dittomo::{Bone, Crowd, Mapping, Player, Skeleton, Transform, Vec3};
/// # let bone = |name: &str, parent| Bone { name: name.into(), parent, rest: Transform::new(Vec3::Y, Default::default()) };
/// # let rig = Skeleton::new(vec![bone("hips", None), bone("spine", Some(0))]).unwrap();
/// let mapping = Mapping::new(&rig, &rig);
/// let mut crowd = Crowd::new(Player::new(&mapping, &rig, &rig).unwrap());
/// // Three characters' poses, one after another.
/// let poses = [rig.rest(), rig.rest(), rig.rest()].concat();
/// let mut out = poses.clone();
/// crowd.pose(&poses, &mut out).unwrap();
/// ```
#[derive(Clone, Debug)]
pub struct Crowd {
    player: Player,
    s_rot: Vec<Turns>,
    s_mat: Vec<[Affine3A; LANES]>,
    turned: Vec<Turns>,
    t_rot: Vec<Turns>,
    t_mat: Vec<[Affine3A; LANES]>,
    /// The sources' and targets' poses with their held feet hung, if the rigs have any.
    hung: [Vec<Transform>; 2],
}

impl Crowd {
    /// Plays many characters through this player.
    pub fn new(player: Player) -> Self {
        let (ns, nt) = (player.s_parent.len(), player.t_parent.len());
        Self {
            s_rot: vec![Turns::IDENTITY; ns],
            s_mat: vec![[Affine3A::IDENTITY; LANES]; ns],
            turned: vec![Turns::IDENTITY; nt],
            t_rot: vec![Turns::IDENTITY; nt],
            t_mat: vec![[Affine3A::IDENTITY; LANES]; nt],
            hung: [Vec::new(), Vec::new()],
            player,
        }
    }

    /// The player each character is played through.
    pub fn player(&mut self) -> &mut Player {
        &mut self.player
    }

    /// Poses targets from poses of the source, one character's after another, as [`Player::pose`] would each.
    ///
    /// # Errors
    /// [`Error::PoseSize`] if the poses aren't one transform per bone for each character, the same characters in and
    /// out; `frame` is the first character whose pose isn't whole.
    pub fn pose(&mut self, source: &[Transform], out: &mut [Transform]) -> Result<(), Error> {
        let (ns, nt) = (self.player.s_parent.len(), self.player.t_parent.len());
        let n = source.len() / ns.max(1);
        for (got, expected) in [(source.len(), ns), (out.len(), nt)] {
            if got != n * expected {
                return Err(Error::PoseSize {
                    frame: n.min(got / expected.max(1)),
                    bones: got.saturating_sub(n.min(got / expected.max(1)) * expected),
                    expected,
                });
            }
        }
        let mut hung = std::mem::take(&mut self.hung);
        let [s_hung, t_hung] = &mut hung;
        let source = match &self.player.held()[0] {
            Some(h) => {
                s_hung.clear();
                s_hung.extend_from_slice(source);
                for (pose, to) in source.chunks(ns).zip(s_hung.chunks_mut(ns)) {
                    h.hang(pose, to);
                }
                &s_hung[..]
            }
            None => source,
        };
        let held = self.player.held()[1].is_some();
        if held {
            t_hung.clear();
            t_hung.extend_from_slice(out);
        }
        if let Some(h) = &self.player.held()[1] {
            for (pose, to) in out.chunks(nt).zip(t_hung.chunks_mut(nt)) {
                h.hang(pose, to);
            }
        }
        let target = if held { &mut t_hung[..] } else { &mut *out };
        for first in (0..n).step_by(LANES) {
            self.lanes(first, n, source, target);
        }
        if let Some(h) = &self.player.held()[1] {
            for (pose, to) in t_hung.chunks(nt).zip(out.chunks_mut(nt)) {
                h.unhang(pose, to);
            }
        }
        self.hung = hung;
        Ok(())
    }

    /// Poses characters `first` to `first + LANES`, those of them there are; lanes past the last character pose it
    /// again and are let go.
    fn lanes(&mut self, first: usize, n: usize, source: &[Transform], out: &mut [Transform]) {
        let p = &self.player;
        let (ns, nt) = (p.s_parent.len(), p.t_parent.len());
        let character = |i: usize| (first + i).min(n - 1);
        let turn = Turns::splat(p.turn);
        for a in 0..ns {
            if !p.s_rot_need[a] {
                continue;
            }
            let mut local = Turns::IDENTITY;
            for i in 0..LANES {
                let r = source[character(i) * ns + a].rotation;
                (local.x[i], local.y[i], local.z[i], local.w[i]) = (r.x, r.y, r.z, r.w);
            }
            let local = local.normalize();
            match p.s_parent[a] {
                Some(q) => {
                    self.s_rot[a] = self.s_rot[q].mul(&local);
                    if p.s_mat_need[a] {
                        for i in 0..LANES {
                            self.s_mat[a][i] = self.s_mat[q][i] * affine(&source[character(i) * ns + a]);
                        }
                    }
                }
                None => {
                    self.s_rot[a] = turn.mul(&local);
                    if p.s_mat_need[a] {
                        for i in 0..LANES {
                            self.s_mat[a][i] = affine(&source[character(i) * ns + a]);
                        }
                    }
                }
            }
        }
        for (b, plan) in p.plans.iter().enumerate() {
            if let Plan::Mapped { source: a, pre, .. } = *plan {
                self.turned[b] = self.s_rot[a].mul(&Turns::splat(pre));
            }
        }
        let lanes = LANES.min(n - first);
        for b in 0..nt {
            let up_r = match p.t_parent[b] {
                Some(q) => self.t_rot[q],
                None => Turns::IDENTITY,
            };
            let rest = p.t_rest[b];
            let rest_rot = Turns::splat(p.t_rest_rot[b]);
            let (rotation, world, travel) = match p.plans[b] {
                Plan::Rest => (
                    Turns::splat(rest.rotation),
                    up_r.mul(&Turns::splat(rest.rotation)),
                    None,
                ),
                Plan::Mapped { source: a, travel, .. } => {
                    let w = self.turned[b].mul(&rest_rot);
                    (up_r.back().mul(&w).whole(), w, travel.map(|k| (a, k)))
                }
                Plan::Between {
                    above,
                    below,
                    along,
                    twist: None,
                } => {
                    let w = self.turned[above].lerp(&self.turned[below], along).mul(&rest_rot);
                    (up_r.back().mul(&w).whole(), w, None)
                }
                Plan::Between {
                    above,
                    below,
                    along,
                    twist: Some(axis),
                } => {
                    let (a, z) = (&self.turned[above], &self.turned[below]);
                    let w = Turns::each(|i| super::twisted(a.get(i), z.get(i), axis, along)).mul(&rest_rot);
                    (up_r.back().mul(&w).whole(), w, None)
                }
            };
            self.t_rot[b] = world;
            let need = p.t_mat_need[b];
            for i in 0..LANES {
                let translation = travel.map_or(rest.translation, |(a, k)| {
                    let up_m = p.t_parent[b].map_or(Affine3A::IDENTITY, |q| self.t_mat[q][i]);
                    let moved = p.turn * (Vec3::from(self.s_mat[a][i].translation) - p.s_rest_pos[a]) * k;
                    up_m.inverse().transform_point3(p.t_rest_pos[b] + moved)
                });
                let local = Transform {
                    translation,
                    rotation: rotation.get(i),
                    scale: rest.scale,
                };
                if need {
                    let up_m = p.t_parent[b].map_or(Affine3A::IDENTITY, |q| self.t_mat[q][i]);
                    self.t_mat[b][i] = up_m * affine(&local);
                }
                if i < lanes {
                    out[(first + i) * nt + b] = local;
                }
            }
        }
    }
}
