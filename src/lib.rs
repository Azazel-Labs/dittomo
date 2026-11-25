//! Retarget an animation from one skeleton onto any other.
//!
//! [`Mapping::new`] works out which bone of the target plays which bone of the source. It looks at bone names in the
//! many spellings rigs use (Mixamo, Unreal, Unity, Rigify, Character Creator, Daz, Blender's numbered copies), where each
//! bone sits on the body at rest, how far along the body it reaches and which way it points. It matches the two trees as
//! wholes, so extra bones on either side (a root above the hips, twist helpers, a longer spine, fingers, a tail) are
//! passed over rather than throwing the rest off. Nothing about humans is assumed: a quadruped maps onto a quadruped, a
//! dragon onto a bird, and two rigs that share little share little of the clip. [`Mapping::quality`] says how alike the
//! two were found.
//!
//! [`Mapping::apply`] then plays the clip: each mapped bone takes the source bone's turn away from its rest pose in the
//! world, corrected for the two rigs' rest poses differing (an A-pose against a T-pose, bones' axes pointing different
//! ways, rigs facing different ways, centimeters against meters). A target bone between two mapped ones takes its share
//! of the turn, so a three-bone spine bends a five-bone one smoothly. The hips travel across the floor by how high the
//! legs start, as far as they step, and up and down by how high the hips stand.
//!
//! A skeleton mapped onto itself plays its clip unchanged.
//!
//! [`Mapping::apply_with`] fixes the result up ([`contact`]): a clip walked in place gets the travel its feet imply,
//! feet stay still while planted (two-bone IK, so legs of other lengths don't skate) and elbows and hands are pushed
//! out of the body. [`Mapping::apply_touching`] also keeps hands as far from the target's skin as the source's are from
//! theirs ([`Surface`]), so a hand on a hip stays on the hip of a wider body.
//!
//! [`Player`] poses one frame at a time without allocating. [`Live`] does too, for poses that come one at a time with
//! nothing ahead to look at, and makes the same fixes from the poses so far. [`Crowd`] poses many characters through
//! one player at once, in about half the time each.
//!
//! With the `load` feature, [`load::Model`] reads skeletons and clips from glTF, GLB and FBX (and USD with `usd`).
//! [`load::Model::write_clip`] writes clips back into the target's own file, out as GLB or FBX, and, with no feature
//! needed, [`bvh::write`] a skeleton and a clip as BVH motion capture.
#![forbid(unsafe_code)]

mod assign;
pub mod bvh;
pub mod contact;
mod held;
mod live;
#[cfg(feature = "load")]
pub mod load;
mod mapping;
mod names;
mod shape;
pub mod surface;
mod transfer;

pub use contact::Fixes;
pub use glam::{Mat4, Quat, Vec3};
pub use live::Live;
pub use mapping::{Mapping, Pair};
pub use surface::Surface;
pub use transfer::{Crowd, Player};

/// A bone's place relative to its parent: translation, rotation and scale, applied scale first.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub translation: Vec3,
    pub rotation: Quat,
    pub scale: Vec3,
}

impl Transform {
    pub const IDENTITY: Self = Self {
        translation: Vec3::ZERO,
        rotation: Quat::IDENTITY,
        scale: Vec3::ONE,
    };

    /// A transform with no scale.
    #[must_use]
    pub fn new(translation: Vec3, rotation: Quat) -> Self {
        Self {
            translation,
            rotation,
            scale: Vec3::ONE,
        }
    }

    /// As a matrix.
    #[must_use]
    pub fn matrix(&self) -> Mat4 {
        Mat4::from_scale_rotation_translation(self.scale, self.rotation, self.translation)
    }
}

impl Default for Transform {
    fn default() -> Self {
        Self::IDENTITY
    }
}

/// One bone of a [`Skeleton`].
#[derive(Clone, Debug, PartialEq)]
pub struct Bone {
    pub name: String,
    /// The index of its parent, which comes before it; none for a root.
    pub parent: Option<usize>,
    /// Its rest pose, relative to its parent (to the world for a root).
    pub rest: Transform,
}

/// Bones, each after its parent.
#[derive(Clone, Debug, PartialEq)]
pub struct Skeleton {
    bones: Vec<Bone>,
}

/// What can be wrong with what's handed in.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// A bone names a parent that doesn't come before it.
    ParentNotBefore { bone: usize },
    /// A pose doesn't have one transform per bone.
    PoseSize {
        frame: usize,
        bones: usize,
        expected: usize,
    },
    /// A clip's times and poses differ in number.
    ClipSize { times: usize, poses: usize },
    /// A clip to write has no times, or times that don't rise.
    ClipTimes,
    /// The mapping was made for other skeletons.
    NotTheirMapping,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ParentNotBefore { bone } => write!(f, "bone {bone}'s parent doesn't come before it"),
            Self::PoseSize { frame, bones, expected } => {
                write!(f, "frame {frame} poses {bones} bones, the skeleton has {expected}")
            }
            Self::ClipSize { times, poses } => write!(f, "the clip has {times} times but {poses} poses"),
            Self::ClipTimes => write!(f, "the clip has no times, or times that don't rise"),
            Self::NotTheirMapping => write!(f, "the mapping was made for other skeletons"),
        }
    }
}

impl std::error::Error for Error {}

impl Skeleton {
    /// A skeleton of these bones, each of which must come after its parent.
    ///
    /// # Errors
    /// [`Error::ParentNotBefore`] if one doesn't.
    pub fn new(bones: Vec<Bone>) -> Result<Self, Error> {
        for (i, b) in bones.iter().enumerate() {
            if b.parent.is_some_and(|p| p >= i) {
                return Err(Error::ParentNotBefore { bone: i });
            }
        }
        Ok(Self { bones })
    }

    #[must_use]
    pub fn bones(&self) -> &[Bone] {
        &self.bones
    }

    #[must_use]
    pub fn len(&self) -> usize {
        self.bones.len()
    }

    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.bones.is_empty()
    }

    /// The first bone with this name.
    #[must_use]
    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.bones.iter().position(|b| b.name == name)
    }

    /// The way the body faces at rest, level, in the world. It's worked out as the mapping works it out (from mirrored
    /// limbs, bones' names and the feet), so a rig built facing away from glTF's +z says so.
    #[must_use]
    pub fn forward(&self) -> Vec3 {
        if self.bones.is_empty() {
            return Vec3::Z;
        }
        shape::Shape::of(self).frame * Vec3::Z
    }

    /// The rest pose, one transform per bone.
    #[must_use]
    pub fn rest(&self) -> Vec<Transform> {
        self.bones.iter().map(|b| b.rest).collect()
    }

    /// Where each bone is and how it's turned in the world, posed by these transforms relative to their parents.
    /// Rotations ignore scale, which is right as long as scales are positive and the same along every axis.
    ///
    /// # Panics
    /// If `pose` doesn't have one transform per bone.
    #[must_use]
    pub fn world(&self, pose: &[Transform]) -> Vec<Placed> {
        assert_eq!(pose.len(), self.bones.len(), "one transform per bone");
        let mut out: Vec<Placed> = Vec::with_capacity(pose.len());
        for (b, local) in self.bones.iter().zip(pose) {
            let placed = match b.parent {
                Some(p) => {
                    let up = out[p];
                    Placed {
                        matrix: up.matrix * local.matrix(),
                        rotation: (up.rotation * local.rotation).normalize(),
                    }
                }
                None => Placed {
                    matrix: local.matrix(),
                    rotation: local.rotation.normalize(),
                },
            };
            out.push(placed);
        }
        out
    }

    pub(crate) fn children(&self) -> Vec<Vec<usize>> {
        let mut out = vec![Vec::new(); self.bones.len()];
        for (i, b) in self.bones.iter().enumerate() {
            if let Some(p) = b.parent {
                out[p].push(i);
            }
        }
        out
    }
}

/// A bone placed in the world.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Placed {
    pub matrix: Mat4,
    pub rotation: Quat,
}

impl Placed {
    #[must_use]
    pub fn position(&self) -> Vec3 {
        self.matrix.w_axis.truncate()
    }
}

/// An animation: a pose for each time, each pose one transform per bone, relative to its parent.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Clip {
    pub times: Vec<f32>,
    pub poses: Vec<Vec<Transform>>,
}

impl Clip {
    pub(crate) fn check(&self, bones: usize) -> Result<(), Error> {
        if self.times.len() != self.poses.len() {
            return Err(Error::ClipSize {
                times: self.times.len(),
                poses: self.poses.len(),
            });
        }
        for (frame, p) in self.poses.iter().enumerate() {
            if p.len() != bones {
                return Err(Error::PoseSize {
                    frame,
                    bones: p.len(),
                    expected: bones,
                });
            }
        }
        Ok(())
    }
}
