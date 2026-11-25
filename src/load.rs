//! Skeletons and clips read from model files, through [polysmelt]: glTF and GLB (VRM avatars too), FBX (Mixamo's
//! downloads among them), BVH motion capture, and USD with polysmelt's `usd` feature.
//!
//! The skeleton is every skin's joints and the nodes above them (an armature node, say, which
//! may turn or scale the whole rig), each after its parent. A model without skins is taken
//! whole. Each animation becomes a [`Clip`] posed at every time any of its channels has a key.
//!
//! Clips go back out with [`Model::write_clip`], as glTF animations of the document the model was read from, which
//! polysmelt writes as a GLB, or as binary FBX with `Document::to_fbx`:
//!
//! ```no_run
//! # fn main() -> Result<(), Box<dyn std::error::Error>> {
//! use dittomo::load::{Model, document};
//! use dittomo::{Fixes, Mapping};
//! let source = Model::load("walk.fbx".as_ref())?;
//! let mut doc = document("avatar.vrm".as_ref())?;
//! let target = Model::from_document(&doc)?;
//! let mapping = Mapping::new(&source.skeleton, &target.skeleton);
//! let skins = [&source.skin, &target.skin];
//! let walk = mapping.apply_touching(&source.skeleton, &target.skeleton, skins, &source.clips[0].1, Fixes::ALL)?;
//! target.write_clip(&mut doc, Some("walk"), &walk)?;
//! std::fs::write("avatar_walk.glb", doc.to_glb()?)?;
//! # Ok(())
//! # }
//! ```
//!
//! [polysmelt]: https://github.com/Azazel-Labs/polysmelt

use polysmelt::document::{Document, Pose};
use polysmelt::mesh::Mesh;
use polysmelt::rig::{Rig, Trs};
use serde_json::{Value, json};

use crate::shape::Shape;
use crate::{Bone, Clip, Quat, Skeleton, Surface, Transform, Vec3};
use glam::{Mat4, Vec4};

/// A model's skeleton and its animations.
#[derive(Clone, Debug, PartialEq)]
pub struct Model {
    pub skeleton: Skeleton,
    /// Each animation's name (if it has one) and clip, in the file's order.
    pub clips: Vec<(Option<String>, Clip)>,
    /// The document node each bone was read from.
    pub nodes: Vec<usize>,
    /// The skin its meshes bind to the bones at rest, for [`crate::Mapping::apply_touching`]; empty when read from a
    /// [`Rig`] alone or when nothing is skinned.
    pub skin: Surface,
}

/// What can go wrong reading a model.
#[derive(Debug)]
pub enum Error {
    /// polysmelt couldn't read it.
    Read(polysmelt::Error),
    /// It has no nodes to make bones of.
    NoBones,
    /// A clip to write doesn't pose the skeleton, or has no times or times that don't rise.
    Clip(crate::Error),
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Read(e) => write!(f, "couldn't read the model: {e}"),
            Self::NoBones => write!(f, "the model has no bones"),
            Self::Clip(e) => write!(f, "couldn't write the clip: {e}"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Read(e) => Some(e),
            Self::NoBones => None,
            Self::Clip(e) => Some(e),
        }
    }
}

impl From<polysmelt::Error> for Error {
    fn from(e: polysmelt::Error) -> Self {
        Self::Read(e)
    }
}

impl Model {
    /// The skeleton and animations of a model file, its format from its extension.
    ///
    /// # Errors
    /// If the file can't be read or has no bones.
    #[cfg(feature = "fs")]
    pub fn load(path: &std::path::Path) -> Result<Self, Error> {
        Self::from_document(&document(path)?)
    }

    /// The skeleton and animations of a model in memory: `ext` names its format (`glb`, `fbx`,
    /// ...). A glTF that refers to other files needs [`Model::from_document`] with them.
    ///
    /// # Errors
    /// If it can't be read or has no bones.
    pub fn from_bytes(bytes: &[u8], ext: &str) -> Result<Self, Error> {
        Self::from_document(&document_from_bytes(bytes, ext)?)
    }

    /// The skeleton and animations of a document polysmelt has read.
    ///
    /// # Errors
    /// If an accessor can't be read or there are no bones.
    pub fn from_document(doc: &Document) -> Result<Self, Error> {
        let (mut model, bound) = Self::read(&doc.rig()?)?;
        // A mesh that can't be drawn leaves the skin empty; the skeleton and clips still stand. Where the bones rest as
        // the skins were bound rather than as the nodes stand, so does the skin.
        let pose = if bound.is_some() { Pose::Bind } else { Pose::Rest };
        if let Ok(Some(mesh)) = doc.flatten(pose) {
            model.skin = skin(&model, &mesh, bound.as_deref());
        }
        Ok(model)
    }

    /// The skeleton and animations of polysmelt's rig view.
    ///
    /// Each bone rests as its skin was bound, where that differs from the node's own transform: a file saved with its
    /// rig in a frame of some animation, legs bent mid-stride, still rests as its mesh was modeled. The bones above
    /// the skins, and the topmost joint of each, keep their nodes' transforms.
    ///
    /// # Errors
    /// [`Error::NoBones`] if there are none.
    pub fn from_rig(rig: &Rig) -> Result<Self, Error> {
        Self::read(rig).map(|(model, _)| model)
    }

    /// [`Model::from_rig`], and if the bones rest as the skins were bound rather than as the nodes stand, by node,
    /// the matrix that takes a vertex bound to that joint from where the file stores it to where the skeleton rests.
    fn read(rig: &Rig) -> Result<(Self, Option<Bound>), Error> {
        let nodes = bones_of(rig);
        if nodes.is_empty() {
            return Err(Error::NoBones);
        }
        let mut at = vec![None; rig.nodes.len()];
        for (b, &n) in nodes.iter().enumerate() {
            at[n] = Some(b);
        }
        let mut bones: Vec<Bone> = nodes
            .iter()
            .map(|&n| {
                let node = &rig.nodes[n];
                Bone {
                    name: node.name.clone().unwrap_or_else(|| format!("node{n}")),
                    parent: node.parent.and_then(|p| at[p]),
                    rest: transform(&node.rest),
                }
            })
            .collect();
        // The bones rest as bound only where the nodes stand lopsided and the bind pose doesn't, as a rig saved mid-stride:
        // a rest pose chosen on purpose, an A-pose for a T-posed mesh, say, stays.
        let posed = Skeleton::new(bones.clone()).expect("bones come parents first");
        let mut bound = bind(rig, &nodes, &mut bones);
        let mut skeleton = Skeleton::new(bones).expect("bones come parents first");
        if bound.is_some() && !(lopsided(&posed) > 0.01 && lopsided(&skeleton) < 0.5 * lopsided(&posed)) {
            (skeleton, bound) = (posed, None);
        }
        let clips = (0..rig.animations.len())
            .map(|a| {
                let anim = &rig.animations[a];
                let mut times = anim.key_times();
                if times.is_empty() {
                    times.push(0.0);
                }
                let mut all = Vec::with_capacity(rig.nodes.len());
                let mut poser = rig.poser(a);
                let poses = times
                    .iter()
                    .map(|&t| {
                        poser.pose_into(t, &mut all);
                        nodes.iter().map(|&n| transform(&all[n])).collect()
                    })
                    .collect();
                (anim.name.clone(), Clip { times, poses })
            })
            .collect();
        let model = Self {
            skeleton,
            clips,
            nodes,
            skin: Surface::default(),
        };
        Ok((model, bound))
    }

    /// The clip with this name.
    #[must_use]
    pub fn clip(&self, name: &str) -> Option<&Clip> {
        self.clips
            .iter()
            .find(|(n, _)| n.as_deref() == Some(name))
            .map(|(_, c)| c)
    }

    /// Adds a clip posing this model's skeleton to `doc`, the document it was read from, as a glTF animation named
    /// `name`, and returns the animation's index. Each bone's turn, place and size are played linearly between keys
    /// at the clip's times, those left out that playing between the rest gives within a thousandth of a degree, or a
    /// hundred-thousandth of the body's height, and left out altogether where they stay as at rest throughout. A keyed
    /// node placed by a matrix is placed by the translation, rotation and scale it stands at instead, as glTF animates
    /// only those. [`Document::to_glb`] or [`Document::to_fbx`] then gives the file.
    ///
    /// # Errors
    /// [`Error::Clip`] if the clip doesn't pose the skeleton, has no times, or its times don't rise.
    pub fn write_clip(&self, doc: &mut Document, name: Option<&str>, clip: &Clip) -> Result<usize, Error> {
        clip.check(self.skeleton.len()).map_err(Error::Clip)?;
        let n = clip.times.len();
        let rising = clip.times.iter().all(|t| t.is_finite()) && clip.times.windows(2).all(|w| w[0] < w[1]);
        if n == 0 || !rising {
            return Err(Error::Clip(crate::Error::ClipTimes));
        }
        let height = Shape::light(&self.skeleton).height;
        // Every accessor's floats, one after another, and each one's offset, keys, floats a key, and bounds (only times
        // have them).
        let mut bytes: Vec<u8> = Vec::new();
        let mut parts: Vec<(usize, usize, usize, Option<[f32; 2]>)> = Vec::new();
        let mut push = |floats: &mut dyn Iterator<Item = f32>, keys: usize, width: usize, bounds: Option<[f32; 2]>| {
            parts.push((bytes.len(), keys, width, bounds));
            for f in floats {
                bytes.extend_from_slice(&f.to_le_bytes());
            }
            parts.len() - 1
        };
        // The times written so far, by which of the clip's they are, to share between channels keyed alike.
        let mut timings: Vec<(Vec<usize>, usize)> = Vec::new();
        // Each channel's bone, path, times and keys.
        let mut channels: Vec<(usize, &str, usize, usize)> = Vec::new();
        // What a bone left unkeyed plays: its node's own transform, which a bone resting as its skin was bound needn't be.
        let rests: Vec<Transform> = self
            .skeleton
            .bones()
            .iter()
            .zip(&self.nodes)
            .map(|(bone, &n)| node_rest(doc.json().get("nodes").and_then(|l| l.get(n))).unwrap_or(bone.rest))
            .collect();
        for (b, &rest) in rests.iter().enumerate() {
            let poses = || clip.poses.iter().map(move |p| p[b]);
            // Each turn on the same side as the last, so playing between them takes the short way round.
            let mut last = rest.rotation;
            let turns: Vec<Vec4> = poses()
                .map(|t| {
                    let r = t.rotation.normalize();
                    last = if r.dot(last) < 0.0 { -r } else { r };
                    Vec4::from(last)
                })
                .collect();
            let near = |a: Vec3, r: Vec3| (a - r).length() <= 1e-6 * (1.0 + r.length());
            let mut keyed: Vec<(&str, Vec<Vec4>, f32)> = Vec::new();
            if turns
                .iter()
                .any(|&r| r.dot(Vec4::from(rest.rotation)).abs() < 1.0 - 1e-7)
            {
                keyed.push(("rotation", turns, TURN_TOLERANCE));
            }
            if poses().any(|t| !near(t.translation, rest.translation)) {
                let keys = poses().map(|t| t.translation.extend(0.0)).collect();
                keyed.push(("translation", keys, PLACE_TOLERANCE * height));
            }
            if poses().any(|t| !near(t.scale, rest.scale)) {
                let keys = poses().map(|t| t.scale.extend(0.0)).collect();
                keyed.push(("scale", keys, PLACE_TOLERANCE * rest.scale.length().max(1.0)));
            }
            for (path, keys, tolerance) in keyed {
                let turn = path == "rotation";
                let kept = thin(&clip.times, &keys, turn, tolerance);
                let input = match timings.iter().find(|t| t.0 == kept) {
                    Some(t) => t.1,
                    None => {
                        let bounds = [clip.times[kept[0]], clip.times[kept[kept.len() - 1]]];
                        let at = push(&mut kept.iter().map(|&k| clip.times[k]), kept.len(), 1, Some(bounds));
                        timings.push((kept.clone(), at));
                        at
                    }
                };
                let width = if turn { 4 } else { 3 };
                let output = push(
                    &mut kept.iter().flat_map(|&k| keys[k].to_array().into_iter().take(width)),
                    kept.len(),
                    width,
                    None,
                );
                channels.push((b, path, input, output));
            }
        }
        let buffer = doc.push_buffer(bytes);
        let json = doc.json_mut();
        let mut add = |key: &str, items: Vec<Value>| {
            let list = json.entry(key).or_insert_with(|| json!([]));
            let list = list.as_array_mut().expect("glTF lists are arrays");
            list.extend(items);
            list.len()
        };
        let views = parts
            .iter()
            .map(|&(at, keys, width, _)| json!({ "buffer": buffer, "byteOffset": at, "byteLength": 4 * width * keys }))
            .collect();
        let view0 = add("bufferViews", views) - parts.len();
        let accessors = parts
            .iter()
            .enumerate()
            .map(|(i, &(_, keys, width, bounds))| {
                let kind = ["SCALAR", "VEC2", "VEC3", "VEC4"][width - 1];
                let mut a = json!({ "bufferView": view0 + i, "componentType": 5126, "count": keys, "type": kind });
                if let Some([lo, hi]) = bounds {
                    (a["min"], a["max"]) = (json!([lo]), json!([hi]));
                }
                a
            })
            .collect();
        let accessor0 = add("accessors", accessors) - parts.len();
        let samplers = channels
            .iter()
            .map(|c| json!({ "input": accessor0 + c.2, "output": accessor0 + c.3, "interpolation": "LINEAR" }))
            .collect();
        let targets = channels
            .iter()
            .enumerate()
            .map(|(i, c)| json!({ "sampler": i, "target": { "node": self.nodes[c.0], "path": c.1 } }))
            .collect();
        let mut animation = json!({ "samplers": Value::Array(samplers), "channels": Value::Array(targets) });
        if let Some(name) = name {
            animation["name"] = json!(name);
        }
        let index = add("animations", vec![animation]) - 1;
        if let Some(nodes) = json.get_mut("nodes").and_then(Value::as_array_mut) {
            for &(b, ..) in &channels {
                let Some(node) = nodes.get_mut(self.nodes[b]).and_then(Value::as_object_mut) else {
                    continue;
                };
                if node.remove("matrix").is_some() {
                    let rest = rests[b];
                    node.insert("translation".into(), json!(rest.translation.to_array()));
                    node.insert("rotation".into(), json!(rest.rotation.to_array()));
                    node.insert("scale".into(), json!(rest.scale.to_array()));
                }
            }
        }
        Ok(index)
    }
}

/// How far from a turn left out of a written clip, as a quaternion, playing between those written may come: half of
/// about a thousandth of a degree.
const TURN_TOLERANCE: f32 = 1e-5;

/// How far from a place left out of a written clip playing between those written may come, over the body's height (or
/// for sizes, over the size at rest, or 1 if larger).
const PLACE_TOLERANCE: f32 = 1e-5;

/// Which of the keys at `times` to write, so that playing linearly between those written (`turns` along the sphere,
/// as quaternions) lands within `tolerance` of every key left out: the first and last, and those where playing
/// between would stray further. At most 127 keys in a row are left out, which bounds the work.
fn thin(times: &[f32], keys: &[Vec4], turns: bool, tolerance: f32) -> Vec<usize> {
    let n = keys.len();
    let between = |a: usize, b: usize, j: usize| {
        let s = (times[j] - times[a]) / (times[b] - times[a]);
        if turns {
            Vec4::from(Quat::from_vec4(keys[a]).slerp(Quat::from_vec4(keys[b]), s))
        } else {
            keys[a].lerp(keys[b], s)
        }
    };
    let mut kept = vec![0];
    let mut from = 0;
    for i in 1..n.saturating_sub(1) {
        let fits =
            i + 1 - from <= 128 && (from + 1..=i).all(|j| between(from, i + 1, j).distance(keys[j]) <= tolerance);
        if !fits {
            kept.push(i);
            from = i;
        }
    }
    if n > 1 {
        kept.push(n - 1);
    }
    kept
}

/// The document in a model file, its format from its extension, with all of it kept: [`Model::from_document`] reads
/// the model from it, and [`Model::write_clip`] adds clips to it to write back out.
///
/// # Errors
/// If the file can't be read.
#[cfg(feature = "fs")]
pub fn document(path: &std::path::Path) -> Result<Document, Error> {
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or_default();
    Ok(loader(ext).open(path)?)
}

/// [`document`] for a model in memory, `ext` naming its format (`glb`, `fbx`, ...). A glTF that refers to other files
/// needs polysmelt to read it with them.
///
/// # Errors
/// If it can't be read.
pub fn document_from_bytes(bytes: &[u8], ext: &str) -> Result<Document, Error> {
    Ok(loader(ext).read(bytes.to_vec(), &format!("model.{ext}"))?)
}

/// The skin of a mesh posed at rest, each vertex bound to the bone that weighs most on it, about one point every 1.25%
/// of the body's height. Vertex normals are worked out from the triangles where the mesh has none.
/// By node, the matrix that takes a vertex bound to that joint from where the file stores it to where the skeleton
/// rests.
type Bound = Vec<Option<Mat4>>;

/// How far a skeleton at rest is from its mirror image across the body's middle, over its height: each bone's distance
/// from the nearest mirrored bone, on average.
fn lopsided(skeleton: &Skeleton) -> f32 {
    let shape = Shape::of(skeleton);
    let left = shape.frame * Vec3::X;
    let middle = shape.rest[shape.middle].position();
    let at: Vec<Vec3> = shape.rest.iter().map(crate::Placed::position).collect();
    let off: f32 = at
        .iter()
        .map(|&p| {
            let mirrored = p - 2.0 * (p - middle).dot(left) * left;
            at.iter().map(|&q| q.distance(mirrored)).fold(f32::MAX, f32::min)
        })
        .sum();
    off / (at.len().max(1) as f32 * shape.height.max(f32::MIN_POSITIVE))
}

/// A node's own transform in a glTF document, by its matrix or its translation, rotation and scale.
fn node_rest(node: Option<&Value>) -> Option<Transform> {
    let node = node?.as_object()?;
    let floats = |key: &str| -> Option<Vec<f32>> {
        #[allow(clippy::cast_possible_truncation, reason = "glTF's numbers are floats")]
        node.get(key)?
            .as_array()?
            .iter()
            .map(|v| v.as_f64().map(|f| f as f32))
            .collect()
    };
    if let Some(m) = floats("matrix") {
        let (scale, rotation, translation) = Mat4::from_cols_slice(m.get(..16)?).to_scale_rotation_translation();
        return Some(Transform {
            translation,
            rotation,
            scale,
        });
    }
    let mut rest = Transform::IDENTITY;
    if let Some(t) = floats("translation") {
        rest.translation = Vec3::from_slice(t.get(..3)?);
    }
    if let Some(r) = floats("rotation") {
        rest.rotation = Quat::from_slice(r.get(..4)?).normalize();
    }
    if let Some(s) = floats("scale") {
        rest.scale = Vec3::from_slice(s.get(..3)?);
    }
    Some(rest)
}

/// Rests the bones as their skins were bound, where that differs from their nodes' transforms, and returns, by node,
/// each joint's matrix from where its vertices are stored to where the skeleton rests; `None` where nothing differs.
///
/// A joint's bind pose is its inverse bind matrix's inverse, from the first skin to list it. Those put the skeleton in
/// the skin's own space, which is where the node drawing the skinned mesh stands, as exporters write it. Unless that
/// space puts one of the skin's topmost joints near its node, the topmost joints keep their nodes' transforms instead
/// and the joints under them keep their place from them. Bones off the skin keep their own transforms.
fn bind(rig: &Rig, nodes: &[usize], bones: &mut [Bone]) -> Option<Bound> {
    let local = |t: &Transform| Mat4::from_scale_rotation_translation(t.scale, t.rotation, t.translation);
    let mut placed = vec![Mat4::IDENTITY; rig.nodes.len()];
    for n in rig.parents_first() {
        let up = rig.nodes[n].parent.map_or(Mat4::IDENTITY, |p| placed[p]);
        placed[n] = up * local(&transform(&rig.nodes[n].rest));
    }
    let size = nodes
        .iter()
        .map(|&n| placed[n].w_axis.truncate().length())
        .fold(0.0, f32::max)
        .max(f32::MIN_POSITIVE);
    // Each joint's inverse bind matrix and skin, and each skin's mesh's space.
    let mut inverse = vec![None; rig.nodes.len()];
    let mut meshes = vec![None; rig.skins.len()];
    for (k, skin) in rig.skins.iter().enumerate() {
        let ibms: Vec<Mat4> = skin.inverse_bind.iter().map(Mat4::from_cols_array).collect();
        if skin.joints.len() < 2 || ibms.iter().all(|m| *m == Mat4::IDENTITY) {
            continue;
        }
        meshes[k] = (0..rig.nodes.len())
            .find(|&n| rig.nodes[n].skin == Some(k))
            .map(|n| placed[n]);
        for (&j, m) in skin.joints.iter().zip(&ibms) {
            let ok = m.is_finite() && m.determinant().abs() > f32::EPSILON.powi(2) && m.inverse().is_finite();
            if ok && inverse.get(j).is_some_and(Option::is_none) {
                inverse[j] = Some((*m, k));
            }
        }
    }
    // Whether two spaces are near enough that one is the other with the body shifted a little, as by a bob or a sway.
    let near = |a: Mat4, b: Mat4| {
        let (sa, ra, ta) = a.to_scale_rotation_translation();
        let (sb, rb, tb) = b.to_scale_rotation_translation();
        ra.angle_between(rb) < 20f32.to_radians()
            && (ta - tb).length() < 0.2 * size
            && (sa - sb).abs().max_element() < 0.01 * sa.abs().max_element()
    };
    // The skins whose mesh's space puts a topmost joint, one with no joint above, near its node, as an IK target left
    // where it was bound while the body bobs and the legs swing.
    let mut under = vec![false; bones.len()];
    let mut fits = vec![false; rig.skins.len()];
    for (b, bone) in bones.iter().enumerate() {
        under[b] = bone.parent.is_some_and(|p| under[p] || inverse[nodes[p]].is_some());
        if let (Some((ibm, k)), false) = (inverse[nodes[b]], under[b]) {
            fits[k] |= meshes[k].is_some_and(|mesh| near(mesh, placed[nodes[b]] * ibm));
        }
    }
    // Each bone's world at rest as bound, and the skin's space it's under.
    let mut moved: Vec<Mat4> = Vec::with_capacity(bones.len());
    let mut space: Vec<Option<Mat4>> = Vec::with_capacity(bones.len());
    for (b, bone) in bones.iter().enumerate() {
        let parent = bone.parent;
        let above = parent.and_then(|p| space[p]);
        let (m, s) = match (inverse[nodes[b]], above) {
            (Some((ibm, _)), Some(s)) => (s * ibm.inverse(), Some(s)),
            (Some((ibm, k)), None) => match meshes[k].filter(|_| fits[k]) {
                Some(mesh) => (mesh * ibm.inverse(), Some(mesh)),
                None => (placed[nodes[b]], Some(placed[nodes[b]] * ibm)),
            },
            (None, _) => (parent.map_or(placed[nodes[b]], |p| moved[p] * local(&bone.rest)), above),
        };
        moved.push(m);
        space.push(s);
    }
    let mut changed = false;
    for b in 0..bones.len() {
        let parent = bones[b].parent;
        let m = parent.map_or(Mat4::IDENTITY, |p| moved[p]).inverse() * moved[b];
        let (scale, rotation, translation) = m.to_scale_rotation_translation();
        let rest = &mut bones[b].rest;
        let differs = rotation.dot(rest.rotation).abs() < 1.0 - 1e-6
            || (translation - rest.translation).length() > 1e-5 * size
            || (scale - rest.scale).abs().max_element() > 1e-4 * rest.scale.abs().max_element().max(1.0);
        let finite = translation.is_finite() && rotation.is_finite() && scale.is_finite();
        if differs && finite {
            *rest = Transform {
                translation,
                rotation: rotation.normalize(),
                scale,
            };
            changed = true;
        }
    }
    if !changed {
        return None;
    }
    // The matrices from the rests as they now stand, those left alone included.
    let mut at: Vec<Mat4> = Vec::with_capacity(bones.len());
    let mut out = vec![None; rig.nodes.len()];
    for (b, bone) in bones.iter().enumerate() {
        let w = bone.parent.map_or(Mat4::IDENTITY, |p| at[p]) * local(&bone.rest);
        out[nodes[b]] = inverse[nodes[b]].map(|(ibm, _)| w * ibm);
        at.push(w);
    }
    Some(out)
}

/// The skin of a mesh drawn at rest, or with `bound`, as stored, each vertex then moved by its joint's matrix there.
/// A face wider than the skin's spacing, all of it moving with one bone, is filled in with points across it, so a
/// low-poly body (a box for a head, say) has skin between its corners too. A face bending between two bones is left
/// to its corners, as which bone its middle moves with isn't known.
fn skin(model: &Model, mesh: &Mesh, bound: Option<&[Option<Mat4>]>) -> Surface {
    let mut bone_of = vec![None; model.nodes.iter().max().map_or(0, |&n| n + 1)];
    for (b, &n) in model.nodes.iter().enumerate() {
        bone_of[n] = Some(b);
    }
    let spacing = Shape::light(&model.skeleton).height / 80.0;
    let mut points = Vec::new();
    let mut corners: Vec<Option<(Vec3, Vec3, usize)>> = Vec::new();
    for p in &mesh.prims {
        if p.joints.len() != p.positions.len() {
            continue;
        }
        let normals: Vec<Vec3> = if p.normals.len() == p.positions.len() {
            p.normals.iter().map(|&n| Vec3::from_array(n)).collect()
        } else {
            let mut n = vec![Vec3::ZERO; p.positions.len()];
            for t in p.indices.as_chunks::<3>().0 {
                let [a, b, c] = t.map(|i| i as usize);
                let (Some(&pa), Some(&pb), Some(&pc)) = (p.positions.get(a), p.positions.get(b), p.positions.get(c))
                else {
                    continue;
                };
                let [pa, pb, pc] = [pa, pb, pc].map(Vec3::from_array);
                let f = (pb - pa).cross(pc - pa);
                for i in [a, b, c] {
                    n[i] += f;
                }
            }
            n
        };
        corners.clear();
        corners.extend(
            p.positions
                .iter()
                .zip(&p.joints)
                .zip(normals)
                .map(|((&at, &j), normal)| {
                    let b = (*bone_of.get(j as usize)?)?;
                    let (mut at, mut normal) = (Vec3::from_array(at), normal);
                    if let Some(m) = bound {
                        let m = (*m.get(j as usize)?)?;
                        at = m.transform_point3(at);
                        normal = m.inverse().transpose().transform_vector3(normal);
                    }
                    Some((at, normal.normalize_or_zero(), b))
                }),
        );
        points.extend(corners.iter().flatten().copied());
        let unindexed: Vec<u32>;
        let indices = if p.indices.is_empty() {
            unindexed = (0..p.positions.len() as u32).collect();
            &unindexed
        } else {
            &p.indices
        };
        for t in indices.as_chunks::<3>().0 {
            let corner = |i: u32| corners.get(i as usize).copied().flatten();
            if let (Some(a), Some(b), Some(c)) = (corner(t[0]), corner(t[1]), corner(t[2]))
                && a.2 == b.2
                && b.2 == c.2
            {
                fill([a.0, b.0, c.0], [a.1, b.1, c.1], a.2, spacing, &mut points);
            }
        }
    }
    Surface::new(&model.skeleton, points, spacing)
}

/// Points across a face with these corners and normals at them, at most `spacing` apart, the corners left out, each
/// with the normal blended from the corners'.
fn fill(at: [Vec3; 3], normals: [Vec3; 3], bone: usize, spacing: f32, out: &mut Vec<(Vec3, Vec3, usize)>) {
    let [a, b, c] = at;
    let longest = a.distance(b).max(b.distance(c)).max(c.distance(a));
    if longest.is_nan() || longest <= spacing {
        return;
    }
    // A face stretching far past the body (a ground plane, say) is filled no finer than this.
    let n = (longest / spacing).ceil().min(64.0) as usize;
    for i in 0..=n {
        for j in 0..=n - i {
            let k = n - i - j;
            if i == n || j == n || k == n {
                continue;
            }
            let [u, v, w] = [i, j, k].map(|x| x as f32 / n as f32);
            out.push((
                a * u + b * v + c * w,
                normals[0] * u + normals[1] * v + normals[2] * w,
                bone,
            ));
        }
    }
}

/// The format a file extension names, in lower case: VRM avatars are GLB.
/// A loader keeping all of a model, reading a VRM avatar as the glTF it is.
fn loader(ext: &str) -> polysmelt::Loader {
    let loader = polysmelt::Loader::new();
    if ext.eq_ignore_ascii_case("vrm") {
        loader.format(polysmelt::Format::Gltf)
    } else {
        loader
    }
}

/// The nodes that make the skeleton, each after its parent: every skin's joints and the nodes
/// above them, else every node.
fn bones_of(rig: &Rig) -> Vec<usize> {
    let order = rig.parents_first();
    let mut keep = vec![false; rig.nodes.len()];
    for skin in &rig.skins {
        for &j in &skin.joints {
            let mut n = Some(j);
            while let Some(i) = n {
                if std::mem::replace(&mut keep[i], true) {
                    break;
                }
                n = rig.nodes[i].parent;
            }
        }
    }
    if !keep.contains(&true) {
        return order;
    }
    order.into_iter().filter(|&n| keep[n]).collect()
}

fn transform(t: &Trs) -> Transform {
    Transform {
        translation: Vec3::from_array(t.translation),
        rotation: Quat::from_array(t.rotation).normalize(),
        scale: Vec3::from_array(t.scale),
    }
}
