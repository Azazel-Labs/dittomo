//! Skeletons and clips read from model files, through [polysmelt]: glTF and GLB (VRM avatars too), FBX (Mixamo's
//! downloads among them), and USD with polysmelt's `usd` feature.
//!
//! The skeleton is every skin's joints and the nodes above them (an armature node, say, which
//! may turn or scale the whole rig), each after its parent. A model without skins is taken
//! whole. Each animation becomes a [`Clip`] posed at every time any of its channels has a key.
//!
//! [polysmelt]: https://github.com/Azazel-Labs/polysmelt

use polysmelt::document::{Document, Pose};
use polysmelt::mesh::Mesh;
use polysmelt::rig::{Rig, Trs};

use crate::shape::Shape;
use crate::{Bone, Clip, Quat, Skeleton, Surface, Transform, Vec3};

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
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Read(e) => write!(f, "couldn't read the model: {e}"),
            Self::NoBones => write!(f, "the model has no bones"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Read(e) => Some(e),
            Self::NoBones => None,
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
        use polysmelt::assets::Folder;
        use polysmelt::document::Keep;
        let ext = path.extension().and_then(|e| e.to_str()).unwrap_or_default();
        let ext = format_of(ext);
        let assets = Folder::new(path.parent().unwrap_or(std::path::Path::new(".")));
        let doc = polysmelt::formats::load(path, &ext, &assets, Keep::ALL)?;
        Self::from_document(&doc)
    }

    /// The skeleton and animations of a model in memory: `ext` names its format (`glb`, `fbx`,
    /// ...). A glTF that refers to other files needs [`Model::from_document`] with them.
    ///
    /// # Errors
    /// If it can't be read or has no bones.
    pub fn from_bytes(bytes: &[u8], ext: &str) -> Result<Self, Error> {
        use polysmelt::assets::Memory;
        use polysmelt::document::Keep;
        let ext = format_of(ext);
        let name = format!("model.{ext}");
        let assets = Memory::new();
        let doc = polysmelt::formats::load_bytes(bytes, &name, &ext, &assets, Keep::ALL)?;
        Self::from_document(&doc)
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
