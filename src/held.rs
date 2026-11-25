//! Limb ends held from elsewhere. Some rigs end a leg above the floor and put the foot or paw on a bone of its own,
//! parented to the rig's root and placed there by IK in the tool the rig was made in. Matched as it is, such a leg ends
//! short and its foot belongs to no limb: moved with the body, the foot stays where it was. Each such bone is hung from
//! the end of its limb instead, for the mapping and for playing, and put back under its own parent at the end. A bone
//! hung from a limb that comes after it in the rig comes after it in the hung skeleton too, the bones' order otherwise
//! kept.

use glam::{Affine3A, Vec3};

use crate::surface::Surface;
use crate::{Clip, Placed, Skeleton, Transform};

/// A skeleton with each held bone hung from the end of its limb, and the way between its poses and the rig's own.
#[derive(Clone, Debug)]
pub(crate) struct Held {
    /// The skeleton with each held bone hung from its limb.
    pub skeleton: Skeleton,
    /// Each held bone, its own parent, and the end of the limb it hangs from, as the rig numbers them.
    bones: Vec<(usize, Option<usize>, usize)>,
    /// Each bone's own parent.
    parents: Vec<Option<usize>>,
    /// The rig's bone each of the hung skeleton's is.
    order: Vec<usize>,
    /// The hung skeleton's bone each of the rig's is.
    index: Vec<usize>,
}

/// The skeleton's held bones, if it has any: a bone low down, in the lowest 15% of the body with bones under it
/// reaching the floor, hanging from the root or a bone holding half the skeleton or more, under a limb that hangs down
/// and stops short, between a twentieth and two fifths of the height up (half, hanging straight down over the bone)
/// with nothing under it or beside it reaching lower, straight above the bone (nearer it across than down), and every
/// other such limb at least twice as far across. A twist bone beside a leg that goes on to the floor ends no limb, nor
/// does a hand. Each limb takes the nearest such bone, and each bone the nearest limb.
pub(crate) fn held(skeleton: &Skeleton) -> Option<Held> {
    let bones = skeleton.bones();
    let n = bones.len();
    let parent = |b: usize| bones[b].parent;
    let rest = skeleton.world(&skeleton.rest());
    let pos: Vec<Vec3> = rest.iter().map(Placed::position).collect();
    let (lo, hi) = pos
        .iter()
        .fold((f32::MAX, f32::MIN), |(lo, hi), p| (lo.min(p.y), hi.max(p.y)));
    let height = hi - lo;
    if height <= 0.0 {
        return None;
    }
    let stands = |b: usize| (pos[b].y - lo) / height;
    let mut size = vec![1usize; n];
    let mut lowest: Vec<f32> = (0..n).map(stands).collect();
    let mut leaf = vec![true; n];
    for b in (0..n).rev() {
        if let Some(p) = parent(b) {
            size[p] += size[b];
            lowest[p] = lowest[p].min(lowest[b]);
            leaf[p] = false;
        }
    }
    // Whether `a` is above `b`, `b`'s parent or further up.
    let above = |a: usize, mut b: usize| loop {
        match parent(b) {
            Some(p) if p == a => break true,
            Some(p) => b = p,
            None => break false,
        }
    };
    // The bones that could end a limb, whatever bone they'd hold: tested once rather than with every bone.
    let ends: Vec<(usize, usize)> = (0..n)
        .filter_map(|l| {
            let q = parent(l)?;
            let can = leaf[l] && lowest[q] >= stands(l) && (0.05..0.5).contains(&stands(l)) && pos[q].y > pos[l].y;
            can.then_some((l, q))
        })
        .collect();
    let mut found: Vec<(f32, usize, usize, f32)> = Vec::new();
    for d in 0..n {
        if stands(d) >= 0.15 || lowest[d] > 0.05 || size[d] > 8 {
            continue;
        }
        for &(l, q) in &ends {
            let drop = pos[l].y - pos[d].y;
            let across = Vec3::new(pos[d].x - pos[l].x, 0.0, pos[d].z - pos[l].z).length();
            if l == d || across >= drop {
                continue;
            }
            // A long forearm hanging straight down from a short upper leg, under a head held high with tall ears (a
            // wolf's): the limb's last bone starts nearly half way up.
            let hangs = Vec3::new(pos[q].x - pos[l].x, 0.0, pos[q].z - pos[l].z).length() < 0.5 * (pos[q].y - pos[l].y);
            let high = if hangs && across < 0.5 * drop { 0.5 } else { 0.4 };
            let branches = || parent(d).is_none_or(|p| 2 * size[p] >= n && above(p, l));
            if stands(l) < high && !above(d, l) && branches() {
                found.push((pos[l].distance(pos[d]), d, l, across));
            }
        }
    }
    // A bone as near across to another limb as to its own stands under neither, as a mesh's node at the origin between
    // the feet of a rig whose legs end at the ankle: an IK bone stands under its own limb.
    let alone: Vec<bool> = found
        .iter()
        .map(|&(_, d, l, across)| {
            !found
                .iter()
                .any(|&(_, e, m, other)| e == d && m != l && other < 2.0 * across)
        })
        .collect();
    let mut found: Vec<(f32, usize, usize)> = found
        .iter()
        .zip(alone)
        .filter_map(|(&(dist, d, l, _), alone)| alone.then_some((dist, d, l)))
        .collect();
    found.sort_by(|x, y| x.0.total_cmp(&y.0));
    let (mut taken_d, mut taken_l) = (vec![false; n], vec![false; n]);
    let mut held = Vec::new();
    for (_, d, l) in found {
        if !taken_d[d] && !taken_l[l] {
            taken_d[d] = true;
            taken_l[l] = true;
            held.push((d, parent(d), l));
        }
    }
    // A limb under a held bone would be placed through it, its pose not yet put back.
    held.retain(|&(_, _, l)| !held_above(&taken_d, bones, l));
    if held.is_empty() {
        return None;
    }
    held.sort_unstable();
    let mut hung = bones.to_vec();
    for &(d, _, l) in &held {
        let local = Affine3A::from_mat4(rest[l].matrix).inverse() * Affine3A::from_mat4(rest[d].matrix);
        hung[d].parent = Some(l);
        hung[d].rest = transform(local);
    }
    // The rig's order, each bone put off until its parent is placed.
    let (mut order, mut placed, mut waiting) = (Vec::with_capacity(n), vec![false; n], vec![Vec::new(); n]);
    for b in 0..n {
        match hung[b].parent {
            Some(p) if !placed[p] => waiting[p].push(b),
            _ => {
                let mut next = vec![b];
                while let Some(x) = next.pop() {
                    order.push(x);
                    placed[x] = true;
                    next.extend(std::mem::take(&mut waiting[x]).into_iter().rev());
                }
            }
        }
    }
    let mut index = vec![0; n];
    for (v, &b) in order.iter().enumerate() {
        index[b] = v;
    }
    let skeleton = order
        .iter()
        .map(|&b| {
            let mut bone = hung[b].clone();
            bone.parent = bone.parent.map(|p| index[p]);
            bone
        })
        .collect();
    Some(Held {
        skeleton: Skeleton::new(skeleton).ok()?,
        bones: held,
        parents: bones.iter().map(|b| b.parent).collect(),
        order,
        index,
    })
}

fn held_above(taken: &[bool], bones: &[crate::Bone], mut b: usize) -> bool {
    while let Some(p) = bones[b].parent {
        if taken[p] {
            return true;
        }
        b = p;
    }
    false
}

impl Held {
    /// A pose of the rig as the hung skeleton has it, each held bone made relative to the end of its limb.
    pub(crate) fn hang(&self, pose: &[Transform], out: &mut [Transform]) {
        for (o, &b) in out.iter_mut().zip(&self.order) {
            *o = pose[b];
        }
        for &(d, p, l) in &self.bones {
            let to = self.world(pose, Some(l)).inverse() * self.world(pose, p) * affine(&pose[d]);
            out[self.index[d]] = transform(to);
        }
    }

    /// The other way: a pose of the hung skeleton as the rig has it, each held bone relative to its own parent again.
    pub(crate) fn unhang(&self, pose: &[Transform], out: &mut [Transform]) {
        for (p, &b) in pose.iter().zip(&self.order) {
            out[b] = *p;
        }
        // No held bone is above a limb's end or its own parent, so both are placed already.
        for &(d, p, l) in &self.bones {
            let to = self.world(out, p).inverse() * self.world(out, Some(l)) * affine(&pose[self.index[d]]);
            out[d] = transform(to);
        }
    }

    /// [`Held::hang`], each pose of a clip.
    pub(crate) fn hang_clip(&self, clip: &Clip) -> Clip {
        let poses = clip
            .poses
            .iter()
            .map(|pose| {
                let mut out = pose.clone();
                self.hang(pose, &mut out);
                out
            })
            .collect();
        Clip {
            times: clip.times.clone(),
            poses,
        }
    }

    /// [`Held::unhang`], each pose of a clip.
    pub(crate) fn unhang_clip(&self, clip: &mut Clip) {
        let mut out = Vec::new();
        for pose in &mut clip.poses {
            out.clone_from(pose);
            self.unhang(pose, &mut out);
            std::mem::swap(pose, &mut out);
        }
    }

    /// The rig's bone that is this bone of the hung skeleton.
    pub(crate) fn real(&self, hung: usize) -> usize {
        self.order[hung]
    }

    /// The hung skeleton's bone that is this bone of the rig.
    pub(crate) fn hung(&self, real: usize) -> usize {
        self.index[real]
    }

    /// Whether the hung skeleton has the bones in another order than the rig.
    pub(crate) fn reordered(&self) -> bool {
        self.order.iter().enumerate().any(|(v, &b)| v != b)
    }

    /// A skin of the rig as one of the hung skeleton: each held bone's rest is where it was, so only the bones' order
    /// changes.
    pub(crate) fn skin(&self, skin: &Surface) -> Surface {
        let mut out = skin.clone();
        for b in &mut out.bone {
            *b = self.index[*b];
        }
        out
    }

    /// Where a bone is in the world, as the rig places it; no held bone is above a limb's end, so it is the same in the
    /// rig and in the skeleton with the held bones hung.
    fn world(&self, pose: &[Transform], mut at: Option<usize>) -> Affine3A {
        let mut m = Affine3A::IDENTITY;
        while let Some(b) = at {
            m = affine(&pose[b]) * m;
            at = self.parents[b];
        }
        m
    }
}

fn affine(t: &Transform) -> Affine3A {
    Affine3A::from_scale_rotation_translation(t.scale, t.rotation, t.translation)
}

fn transform(m: Affine3A) -> Transform {
    let (scale, rotation, translation) = m.to_scale_rotation_translation();
    Transform {
        translation,
        rotation: rotation.normalize(),
        scale,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Bone;
    use glam::Quat;

    fn bone(name: &str, parent: Option<usize>, at: [f32; 3]) -> Bone {
        Bone {
            name: name.into(),
            parent,
            rest: Transform::new(Vec3::from_array(at), Quat::IDENTITY),
        }
    }

    /// A leg ending at the knee, its paw an IK bone under the root.
    fn rig() -> Skeleton {
        Skeleton::new(vec![
            bone("root", None, [0.0, 0.0, 0.0]),
            bone("body", Some(0), [0.0, 1.0, 0.0]),
            bone("head", Some(1), [0.0, 1.0, 0.3]),
            bone("thigh", Some(1), [0.2, 0.0, 0.0]),
            bone("shin", Some(3), [0.0, -0.5, 0.0]),
            bone("paw", Some(0), [0.2, 0.05, 0.0]),
            bone("toe", Some(5), [0.0, -0.05, 0.1]),
        ])
        .unwrap()
    }

    #[test]
    fn a_paw_under_the_root_hangs_from_its_leg() {
        let h = held(&rig()).expect("a held bone");
        assert_eq!(h.bones, vec![(5, Some(0), 4)]);
        assert_eq!(h.skeleton.bones()[5].parent, Some(4));
        let (a, b) = (rig().world(&rig().rest()), h.skeleton.world(&h.skeleton.rest()));
        assert!(a[5].position().distance(b[5].position()) < 1e-6);
    }

    #[test]
    fn a_leg_with_its_foot_holds_nothing() {
        let mut bones = rig().bones().to_vec();
        bones[5].parent = Some(4);
        bones[5].rest.translation = Vec3::new(0.0, -0.45, 0.0);
        assert!(held(&Skeleton::new(bones).unwrap()).is_none());
    }

    #[test]
    fn a_node_between_the_feet_ends_neither_leg() {
        // Legs ending at the ankle, and the mesh's node at the origin, a root of its own as some exports leave it: low
        // enough under either ankle to be its foot, but as near the one as the other.
        let mut bones = vec![
            bone("body", None, [0.0, 0.0, 0.0]),
            bone("hips", None, [0.0, 1.0, 0.0]),
            bone("head", Some(1), [0.0, 0.7, 0.0]),
        ];
        for x in [0.1, -0.1] {
            let thigh = bones.len();
            bones.push(bone("thigh", Some(1), [x, -0.05, 0.0]));
            bones.push(bone("calf", Some(thigh), [0.0, -0.4, 0.0]));
            bones.push(bone("foot", Some(thigh + 1), [0.0, -0.4, 0.0]));
        }
        assert!(held(&Skeleton::new(bones).unwrap()).is_none());
    }

    #[test]
    fn hanging_and_back_keeps_the_pose() {
        let s = rig();
        let h = held(&s).unwrap();
        let mut pose = s.rest();
        pose[0].translation = Vec3::new(0.3, 0.0, 0.5);
        pose[3].rotation = Quat::from_rotation_x(0.4);
        pose[5].translation = Vec3::new(0.25, 0.1, 0.2);
        pose[5].rotation = Quat::from_rotation_y(0.3);
        let mut hung = pose.clone();
        h.hang(&pose, &mut hung);
        // The paw is where the rig puts it, posed either way.
        let (a, b) = (s.world(&pose), h.skeleton.world(&hung));
        assert!(a[6].position().distance(b[6].position()) < 1e-5);
        let mut back = s.rest();
        h.unhang(&hung, &mut back);
        assert!(back[5].translation.distance(pose[5].translation) < 1e-5);
        assert!(back[5].rotation.dot(pose[5].rotation).abs() > 1.0 - 1e-5);
    }

    #[test]
    fn a_paw_before_its_leg_comes_after_it_hung() {
        // The paw and its toe come first, as some rigs list their IK bones, before the leg it hangs from.
        let s = Skeleton::new(vec![
            bone("root", None, [0.0, 0.0, 0.0]),
            bone("paw", Some(0), [0.2, 0.05, 0.0]),
            bone("toe", Some(1), [0.0, -0.05, 0.1]),
            bone("body", Some(0), [0.0, 1.0, 0.0]),
            bone("head", Some(3), [0.0, 1.0, 0.3]),
            bone("thigh", Some(3), [0.2, 0.0, 0.0]),
            bone("shin", Some(5), [0.0, -0.5, 0.0]),
        ])
        .unwrap();
        let h = held(&s).expect("a held bone");
        let names: Vec<&str> = h.skeleton.bones().iter().map(|b| b.name.as_str()).collect();
        assert_eq!(names, ["root", "body", "head", "thigh", "shin", "paw", "toe"]);
        assert_eq!((h.hung(1), h.real(5)), (5, 1));
        let mut pose = s.rest();
        pose[3].translation.z += 0.4;
        pose[5].rotation = Quat::from_rotation_x(0.3);
        let mut hung = s.rest();
        h.hang(&pose, &mut hung);
        let (a, b) = (s.world(&pose), h.skeleton.world(&hung));
        assert!(a[2].position().distance(b[6].position()) < 1e-5);
        let mut back = s.rest();
        h.unhang(&hung, &mut back);
        for (x, y) in back.iter().zip(&pose) {
            assert!(x.translation.distance(y.translation) < 1e-5);
        }
    }
}
