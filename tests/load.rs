//! Skeletons and clips read from a model file: Blender's FBX export of a column skinned to two
//! animated bones under an armature.
#![cfg(feature = "fs")]

use std::path::PathBuf;

use mummer::Mapping;
use mummer::load::Model;

fn path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("tests/data/armature_animated.fbx")
}

#[test]
fn reads_the_armature_and_its_clip() {
    let m = Model::load(&path()).unwrap();
    let names: Vec<&str> = m.skeleton.bones().iter().map(|b| b.name.as_str()).collect();
    let lower = names.iter().position(|n| n.contains("Lower")).expect("a Lower bone");
    let upper = names.iter().position(|n| n.contains("Upper")).expect("an Upper bone");
    assert_eq!(m.skeleton.bones()[upper].parent, Some(lower), "{names:?}");
    assert!(
        names.iter().any(|n| n.contains("Armature")),
        "the armature node is kept: {names:?}"
    );
    assert_eq!(m.nodes.len(), m.skeleton.len());

    assert_eq!(m.clips.len(), 1);
    let clip = &m.clips[0].1;
    assert!(clip.times.len() > 2);
    assert!(clip.times.windows(2).all(|w| w[0] < w[1]));
    assert!(clip.poses.iter().all(|p| p.len() == m.skeleton.len()));

    // Upper's top moves through the clip, and the rest pose stands it 2 units up the column.
    let rest = m.skeleton.world(&m.skeleton.rest());
    let first = m.skeleton.world(&clip.poses[0]);
    let last = m.skeleton.world(&clip.poses[clip.poses.len() - 1]);
    assert!(rest[upper].position().y > rest[lower].position().y);
    assert!(first[upper].rotation.angle_between(last[upper].rotation) > 0.1);
}

#[test]
fn reads_the_same_from_memory() {
    let bytes = std::fs::read(path()).unwrap();
    assert_eq!(Model::from_bytes(&bytes, "FBX").unwrap(), Model::load(&path()).unwrap());
}

#[test]
fn plays_onto_itself_unchanged() {
    let m = Model::load(&path()).unwrap();
    let map = Mapping::new(&m.skeleton, &m.skeleton);
    assert!(map.quality() > 0.99, "{}", map.quality());
    let clip = &m.clips[0].1;
    let out = map.apply(&m.skeleton, &m.skeleton, clip).unwrap();
    for (a, b) in clip.poses.iter().zip(&out.poses) {
        let (wa, wb) = (m.skeleton.world(a), m.skeleton.world(b));
        for (x, y) in wa.iter().zip(&wb) {
            assert!(x.position().distance(y.position()) < 1e-3 * 2.0);
            assert!(x.rotation.angle_between(y.rotation) < 1e-3);
        }
    }
}
