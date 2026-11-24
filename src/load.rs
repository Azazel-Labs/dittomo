//! Skeletons and clips read from model files, through [polysmelt]: glTF and GLB (VRM avatars too), FBX (Mixamo's
//! downloads among them), BVH motion capture, and USD with polysmelt's `usd` feature.
//!
//! The skeleton is every skin's joints and the nodes above them (an armature node, say, which
//! may turn or scale the whole rig), each after its parent. A model without skins is taken
//! whole. Each animation becomes a [`Clip`] posed at every time any of its channels has a key.
//!
//! Clips go back out with [`Model::write_clip`], as glTF animations of the document the model was read from, which
//! polysmelt writes as a GLB:
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
use glam::Vec4;

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
    pub fn from_document(doc: &Document<'_>) -> Result<Self, Error> {
        let mut model = Self::from_rig(&doc.rig()?)?;
        // A mesh that can't be drawn leaves the skin empty; the skeleton and clips still stand.
        if let Ok(Some(mesh)) = doc.flatten(Pose::Rest) {
            model.skin = skin(&model, &mesh);
        }
        Ok(model)
    }

    /// The skeleton and animations of polysmelt's rig view.
    ///
    /// # Errors
    /// [`Error::NoBones`] if there are none.
    pub fn from_rig(rig: &Rig) -> Result<Self, Error> {
        let nodes = bones_of(rig);
        if nodes.is_empty() {
            return Err(Error::NoBones);
        }
        let mut at = vec![None; rig.nodes.len()];
        for (b, &n) in nodes.iter().enumerate() {
            at[n] = Some(b);
        }
        let bones: Vec<Bone> = nodes
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
        let skeleton = Skeleton::new(bones).expect("bones come parents first");
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
        Ok(Self {
            skeleton,
            clips,
            nodes,
            skin: Surface::default(),
        })
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
    /// only those. [`Document::to_glb`] then gives the file.
    ///
    /// # Errors
    /// [`Error::Clip`] if the clip doesn't pose the skeleton, has no times, or its times don't rise.
    pub fn write_clip(&self, doc: &mut Document<'_>, name: Option<&str>, clip: &Clip) -> Result<usize, Error> {
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
        for (b, bone) in self.skeleton.bones().iter().enumerate() {
            let rest = bone.rest;
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
        let json = &mut doc.json;
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
                    let rest = self.skeleton.bones()[b].rest;
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
pub fn document(path: &std::path::Path) -> Result<Document<'static>, Error> {
    use polysmelt::assets::Folder;
    use polysmelt::document::Keep;
    let ext = path.extension().and_then(|e| e.to_str()).unwrap_or_default();
    let ext = format_of(ext);
    let assets = Folder::new(path.parent().unwrap_or(std::path::Path::new(".")));
    Ok(polysmelt::formats::load(path, &ext, &assets, Keep::ALL)?)
}

/// [`document`] for a model in memory, `ext` naming its format (`glb`, `fbx`, ...). A glTF that refers to other files
/// needs polysmelt to read it with them.
///
/// # Errors
/// If it can't be read.
pub fn document_from_bytes(bytes: &[u8], ext: &str) -> Result<Document<'static>, Error> {
    use polysmelt::assets::Memory;
    use polysmelt::document::Keep;
    let ext = format_of(ext);
    let name = format!("model.{ext}");
    let assets = Memory::new();
    Ok(polysmelt::formats::load_bytes(bytes, &name, &ext, &assets, Keep::ALL)?.into_owned())
}

/// The skin of a mesh posed at rest, each vertex bound to the bone that weighs most on it, about one point every 1.25%
/// of the body's height. Vertex normals are worked out from the triangles where the mesh has none.
fn skin(model: &Model, mesh: &Mesh) -> Surface {
    let mut bone_of = vec![None; model.nodes.iter().max().map_or(0, |&n| n + 1)];
    for (b, &n) in model.nodes.iter().enumerate() {
        bone_of[n] = Some(b);
    }
    let mut points = Vec::new();
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
        for ((&at, &j), normal) in p.positions.iter().zip(&p.joints).zip(normals) {
            if let Some(&Some(b)) = bone_of.get(j as usize) {
                points.push((Vec3::from_array(at), normal, b));
            }
        }
    }
    let height = Shape::light(&model.skeleton).height;
    Surface::new(&model.skeleton, points, height / 80.0)
}

/// The format a file extension names, in lower case: VRM avatars are GLB.
fn format_of(ext: &str) -> String {
    let ext = ext.to_ascii_lowercase();
    if ext == "vrm" { "glb".to_string() } else { ext }
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
