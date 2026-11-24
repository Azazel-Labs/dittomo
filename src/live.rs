//! Retargeting as the poses come, with the contact fixes made frame by frame from the poses so far: a character driven
//! by motion capture streaming in, a networked player, another animation system's output.

use crate::contact::{self, Ahead, Body, Feet, Hands, OnHead, Scratch, Touch, Travel};
use crate::shape::Shape;
use crate::{Error, Fixes, Mapping, Player, Skeleton, Surface, Transform};

/// Plays poses of one skeleton on another as they come, a frame at a time, without allocating, and fixes each up as
/// [`Mapping::apply_with`] fixes a clip, but from the poses so far, with nothing ahead to look at.
///
/// Arms are kept out of the body, hands brought together, kept on the head and, given skins ([`Live::touching`]), kept
/// at the skin's distance just as the whole clip has them, as those fixes look only at the pose in hand. Planted feet
/// and travel are worked out as the poses come: a foot plants as soon as the source's is down and nearly still and lets
/// go once it lifts, held where it was when it planted; a clip walked in place moves on as far as the planted foot
/// slides back.
///
/// A foot can be held only once it has landed, where the whole clip eases it toward where it will land, so planted
/// feet slide more. [`Live::delayed`] plays each pose a few poses late, and plants feet with those few in sight as the
/// whole clip does.
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
    touch: Option<Touch>,
    /// The source's pose with its travel.
    walked: Vec<Transform>,
    /// The source bones placed once for every fix looking at the pose as it comes, and, if poses are played late,
    /// those placed for the fixes looking at the pose played.
    s_now: Vec<usize>,
    s_due: Vec<usize>,
    scratch: Scratch,
    /// If poses are played late, the feet planted with those ahead in sight.
    ahead: Option<Ahead>,
    /// If poses are played late, the last few: each the source's pose with its travel, and the target playing it.
    kept: Vec<(Vec<Transform>, Vec<Transform>)>,
    /// How many poses have come since the start or the last reset.
    seen: usize,
}

impl Live {
    /// A live player for this mapping between these skeletons, making these fixes (all but touch, which needs skins:
    /// see [`Live::touching`]).
    ///
    /// # Errors
    /// [`Error::NotTheirMapping`] if the mapping was made for skeletons of other sizes.
    pub fn new(map: &Mapping, source: &Skeleton, target: &Skeleton, fixes: Fixes) -> Result<Self, Error> {
        Self::touching(map, source, target, [&Surface::default(), &Surface::default()], fixes)
    }

    /// [`Live::new`], knowing each rig's skin (source's first), as [`Mapping::apply_touching`] does: a hand as near its
    /// skin as to touch it keeps the same gap to the target's. An empty [`Surface`] leaves that fix out.
    ///
    /// # Errors
    /// [`Error::NotTheirMapping`] if the mapping was made for skeletons of other sizes.
    pub fn touching(
        map: &Mapping,
        source: &Skeleton,
        target: &Skeleton,
        skins: [&Surface; 2],
        fixes: Fixes,
    ) -> Result<Self, Error> {
        if source.len() != map.sources() || target.len() != map.pairs().len() {
            return Err(Error::NotTheirMapping);
        }
        let (s, t) = (Shape::light(source), Shape::light(target));
        let rigs = [(source, &s), (target, &t)];
        let player = Player::of_shapes(map, source, target, &s, &t);
        let both = fixes.touch || fixes.hands || fixes.head;
        let mut t_body = (fixes.push_out || both).then(|| contact::body(target, &t)).flatten();
        let mut s_body = both.then(|| contact::body(source, &s)).flatten();
        // With skins, each head is as big as its skin shows, not as its bones suggest.
        for (body, shape, skin) in [(&mut s_body, &s, skins[0]), (&mut t_body, &t, skins[1])] {
            if let Some(b) = body {
                b.fit_head(shape, skin);
            }
        }
        let (mut hands, mut head, mut touch) = (None, None, None);
        if let (Some(sb), Some(tb)) = (&s_body, &t_body) {
            hands = fixes.hands.then(|| Hands::new(map, rigs, [sb, tb])).flatten();
            head = fixes.head.then(|| OnHead::new(map, rigs, [sb, tb])).flatten();
            let skinned = [(source, &s, skins[0]), (target, &t, skins[1])];
            touch = fixes.touch.then(|| Touch::new(map, skinned, [sb, tb])).flatten();
        }
        let push_out = t_body.as_ref().filter(|_| fixes.push_out).map(|b| b.path(target));
        let mut live = Self {
            player,
            travel: fixes.travel.then(|| Travel::new(source, &s)).flatten(),
            feet: fixes.feet.then(|| Feet::new(map, rigs)).flatten(),
            push_out,
            body: t_body,
            hands,
            head,
            touch,
            walked: source.rest(),
            s_now: Vec::new(),
            s_due: Vec::new(),
            scratch: Scratch::new(source, target),
            source: source.clone(),
            target: target.clone(),
            ahead: None,
            kept: Vec::new(),
            seen: 0,
        };
        live.paths();
        Ok(live)
    }

    /// Works out which source bones to place for which pose.
    fn paths(&mut self) {
        let empty: &[usize] = &[];
        let travel = self.travel.as_ref().map_or(empty, Travel::path);
        let feet = self.feet.as_ref().map_or(empty, Feet::s_path);
        let hands = self.hands.as_ref().map_or(empty, Hands::s_path);
        let head = self.head.as_ref().map_or(empty, OnHead::s_path);
        let touch = self.touch.as_ref().map_or(empty, Touch::s_path);
        if self.kept.is_empty() {
            self.s_now = contact::union(&[travel, feet, hands, head, touch]);
            self.s_due = Vec::new();
        } else {
            self.s_now = contact::union(&[travel, feet]);
            self.s_due = contact::union(&[hands, head, touch]);
        }
    }

    /// Plays each pose `poses` poses late, so that feet can be planted with those in sight: eased toward where they will
    /// land before they do, held at the average of where they are while down, as [`Mapping::apply_with`] has them. Until
    /// that many have come, the first pose is played. At 30 poses a second, 6 (a fifth of a second late) plant feet
    /// nearly as the whole clip does.
    #[must_use]
    pub fn delayed(mut self, poses: usize) -> Self {
        self.ahead = (poses > 0)
            .then(|| self.feet.as_ref().map(|f| Ahead::new(f, poses)))
            .flatten();
        self.kept = if poses > 0 {
            vec![(self.source.rest(), self.target.rest()); poses + 1]
        } else {
            Vec::new()
        };
        self.paths();
        self.reset();
        self
    }

    /// How many poses late each is played ([`Self::delayed`]).
    #[must_use]
    pub fn delay(&self) -> usize {
        self.kept.len().saturating_sub(1)
    }

    /// Marks the source bones that move, not just turn, somewhere in this clip, as [`Player::moving_in`] does.
    ///
    /// # Errors
    /// If the clip doesn't pose the source.
    pub fn moving_in(&mut self, clip: &crate::Clip) -> Result<&mut Self, Error> {
        self.player.moving_in(clip)?;
        Ok(self)
    }

    /// Forgets the poses so far: where feet are planted, how far a clip walked in place has traveled, where hands
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
        if let Some(a) = &mut self.ahead {
            a.reset();
        }
        self.seen = 0;
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
        if !self.kept.is_empty() {
            return self.pose_late(dt, source, out);
        }
        let from = match &mut self.travel {
            Some(t) => {
                t.pose(
                    &self.source,
                    dt,
                    source,
                    &mut self.walked,
                    &mut self.scratch,
                    Some(&self.s_now),
                );
                &self.walked[..]
            }
            None => {
                contact::place_source(&self.source, source, &self.s_now, &mut self.scratch);
                source
            }
        };
        self.player.pose(from, out)?;
        if let Some(f) = &mut self.feet {
            f.pose(&self.target, dt, out, &mut self.scratch);
        }
        if let (Some(path), Some(body)) = (&self.push_out, &self.body) {
            contact::push_pose(&self.target, body, path, out, &mut self.scratch);
        }
        if let (Some(h), Some(body)) = (&mut self.hands, &self.body) {
            h.placed(&self.target, body, out, &mut self.scratch);
        }
        if let Some(h) = &self.head {
            h.placed(&self.target, out, &mut self.scratch);
        }
        if let Some(t) = &mut self.touch {
            t.placed(&self.target, out, &mut self.scratch);
        }
        Ok(())
    }

    /// [`Self::pose`], played [`Self::delay`] poses late.
    fn pose_late(&mut self, dt: f32, source: &[Transform], out: &mut [Transform]) -> Result<(), Error> {
        let n = self.kept.len();
        let (walked, played) = &mut self.kept[self.seen % n];
        match &mut self.travel {
            Some(t) => t.pose(&self.source, dt, source, walked, &mut self.scratch, Some(&self.s_now)),
            None => {
                walked.copy_from_slice(source);
                contact::place_source(&self.source, source, &self.s_now, &mut self.scratch);
            }
        }
        self.player.pose(walked, played)?;
        if let Some(a) = &mut self.ahead {
            a.see(&self.target, dt, played, &mut self.scratch);
        }
        let due = self.seen.saturating_sub(n - 1);
        self.seen += 1;
        let (from, played) = &self.kept[due % n];
        out.copy_from_slice(played);
        if let Some(a) = &mut self.ahead {
            a.fix(&self.target, due, out, &mut self.scratch);
        }
        if let (Some(path), Some(body)) = (&self.push_out, &self.body) {
            contact::push_pose(&self.target, body, path, out, &mut self.scratch);
        }
        if !self.s_due.is_empty() {
            contact::place_source(&self.source, from, &self.s_due, &mut self.scratch);
        }
        if let (Some(h), Some(body)) = (&mut self.hands, &self.body) {
            h.placed(&self.target, body, out, &mut self.scratch);
        }
        if let Some(h) = &self.head {
            h.placed(&self.target, out, &mut self.scratch);
        }
        if let Some(t) = &mut self.touch {
            t.placed(&self.target, out, &mut self.scratch);
        }
        Ok(())
    }
}
