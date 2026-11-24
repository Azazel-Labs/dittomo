//! Retargeting as the poses come, with the contact fixes made frame by frame from the poses so far: a character driven
//! by motion capture streaming in, a networked player, another animation system's output.

use crate::contact::{self, Body, Feet, Hands, OnHead, Scratch, Travel};
use crate::shape::Shape;
use crate::{Error, Fixes, Mapping, Player, Skeleton, Transform};

/// Plays poses of one skeleton on another as they come, a frame at a time, without allocating, and fixes each up as
/// [`Mapping::apply_with`] fixes a clip, but from the poses so far, with nothing ahead to look at.
///
/// Arms are kept out of the body, hands brought together and kept on the head just as the whole clip has them, as
/// those fixes look only at the pose in hand. Planted feet and travel are worked out as the poses come: a foot plants
/// as soon as the source's is down and nearly still and lets go once it lifts, held where it was when it planted; a
/// clip walked in place moves on as far as the planted foot slides back. Keeping hands at the skin's distance
/// ([`Fixes::touch`]) is left out, as it needs skins and the whole clip's work.
///
/// ```
/// # use dittomo::{Bone, Fixes, Live, Mapping, Skeleton, Transform, Vec3};
/// # let bone = |name: &str, parent| Bone { name: name.into(), parent, rest: Transform::new(Vec3::Y, Default::default()) };
/// # let rig = Skeleton::new(vec![bone("hips", None), bone("spine", Some(0))]).unwrap();
/// let mapping = Mapping::new(&rig, &rig);
/// let mut live = Live::new(&mapping, &rig, &rig, Fixes::ALL).unwrap();
/// let mut out = rig.rest();
/// // Each frame, the source's pose and the seconds since the last one.
/// live.pose(1.0 / 60.0, &rig.rest(), &mut out).unwrap();
/// ```
pub struct Live {
    player: Player,
    source: Skeleton,
    target: Skeleton,
    travel: Option<Travel>,
    feet: Option<Feet>,
    /// The target's body, if arms are kept out of it or hands meet.
    body: Option<Body>,
    /// The bones to place for the body, if arms are kept out of it.
    push_out: Option<Vec<usize>>,
    hands: Option<Hands>,
    head: Option<OnHead>,
    /// The source's pose with its travel.
    walked: Vec<Transform>,
    scratch: Scratch,
}

impl Live {
    /// A live player for this mapping between these skeletons, making these fixes (all but touch).
    ///
    /// # Errors
    /// [`Error::NotTheirMapping`] if the mapping was made for skeletons of other sizes.
    pub fn new(map: &Mapping, source: &Skeleton, target: &Skeleton, fixes: Fixes) -> Result<Self, Error> {
        if source.len() != map.sources() || target.len() != map.pairs().len() {
            return Err(Error::NotTheirMapping);
        }
        let (s, t) = (Shape::light(source), Shape::light(target));
        let rigs = [(source, &s), (target, &t)];
        let player = Player::of_shapes(map, source, target, &s, &t);
        let both = fixes.hands || fixes.head;
        let t_body = (fixes.push_out || both).then(|| contact::body(target, &t)).flatten();
        let s_body = both.then(|| contact::body(source, &s)).flatten();
        let (mut hands, mut head) = (None, None);
        if let (Some(sb), Some(tb)) = (&s_body, &t_body) {
            hands = fixes.hands.then(|| Hands::new(map, rigs, [sb, tb])).flatten();
            head = fixes.head.then(|| OnHead::new(map, rigs, [sb, tb])).flatten();
        }
        let push_out = t_body.as_ref().filter(|_| fixes.push_out).map(|b| b.path(target));
        Ok(Self {
            player,
            travel: fixes.travel.then(|| Travel::new(source, &s)).flatten(),
            feet: fixes.feet.then(|| Feet::new(map, rigs)).flatten(),
            push_out,
            body: t_body,
            hands,
            head,
            walked: source.rest(),
            scratch: Scratch::new(source, target),
            source: source.clone(),
            target: target.clone(),
        })
    }

    /// Marks the source bones that move, not just turn, somewhere in this clip, as [`Player::moving_in`] does.
    ///
    /// # Errors
    /// If the clip doesn't pose the source.
    pub fn moving_in(&mut self, clip: &crate::Clip) -> Result<&mut Self, Error> {
        self.player.moving_in(clip)?;
        Ok(self)
    }

    /// Forgets the poses so far: where feet are planted, how far a clip walked in place has travelled, where hands
    /// met. For a cut, or a jump to another clip.
    pub fn reset(&mut self) {
        if let Some(t) = &mut self.travel {
            t.reset();
        }
        if let Some(f) = &mut self.feet {
            f.reset();
        }
        if let Some(h) = &mut self.hands {
            h.reset();
        }
    }

    /// Poses the target from a pose of the source `dt` seconds after the last, each transform relative to its parent,
    /// and fixes it up.
    ///
    /// # Errors
    /// [`Error::PoseSize`] if either pose doesn't have one transform per bone.
    pub fn pose(&mut self, dt: f32, source: &[Transform], out: &mut [Transform]) -> Result<(), Error> {
        let (ns, nt) = (self.source.len(), self.target.len());
        for (got, expected) in [(source.len(), ns), (out.len(), nt)] {
            if got != expected {
                return Err(Error::PoseSize {
                    frame: 0,
                    bones: got,
                    expected,
                });
            }
        }
        let rigs = [&self.source, &self.target];
        let from = match &mut self.travel {
            Some(t) => {
                t.pose(&self.source, dt, source, &mut self.walked, &mut self.scratch);
                &self.walked[..]
            }
            None => source,
        };
        self.player.pose(from, out)?;
        if let Some(f) = &mut self.feet {
            f.pose(rigs, dt, from, out, &mut self.scratch);
        }
        if let (Some(path), Some(body)) = (&self.push_out, &self.body) {
            contact::push_pose(&self.target, body, path, out, &mut self.scratch);
        }
        if let (Some(h), Some(body)) = (&mut self.hands, &self.body) {
            h.pose(rigs, body, from, out, &mut self.scratch);
        }
        if let Some(h) = &self.head {
            h.pose(rigs, from, out, &mut self.scratch);
        }
        Ok(())
    }
}
