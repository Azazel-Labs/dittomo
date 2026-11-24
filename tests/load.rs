//! Skeletons and clips read from a model file: Blender's FBX export of a column skinned to two
//! animated bones under an armature.
#![cfg(feature = "fs")]

use std::path::PathBuf;

use dittomo::load::{self, Model};
use dittomo::{Clip, Mapping, Quat, Vec3};

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

#[test]
fn writes_a_clip_that_reads_back_the_same() {
    let mut doc = load::document(&path()).unwrap();
    let m = Model::from_document(&doc).unwrap();
    // The clip slowed to half speed, with the lower bone turned a little more each pose, so it differs from the file's.
    let clip = &m.clips[0].1;
    let lower = m
        .skeleton
        .bones()
        .iter()
        .position(|b| b.name.contains("Lower"))
        .unwrap();
    let mut slow = Clip {
        times: clip.times.iter().map(|t| 2.0 * t).collect(),
        poses: clip.poses.clone(),
    };
    for (f, p) in slow.poses.iter_mut().enumerate() {
        p[lower].rotation = Quat::from_rotation_z(0.01 * f as f32) * p[lower].rotation;
    }
    let at = m.write_clip(&mut doc, Some("slow"), &slow).unwrap();
    assert_eq!(at, 1);
    let glb = doc.to_glb().unwrap();
    let back = Model::from_bytes(&glb, "glb").unwrap();
    assert_eq!(back.skeleton, m.skeleton);
    assert_eq!(back.clips[0], m.clips[0], "the file's own clip is kept");
    assert!(back.clip("slow").is_some());
    // Played at each of the clip's times, whether keyed there or not.
    let rig = load::document_from_bytes(&glb, "glb").unwrap().rig().unwrap();
    for (&t, a) in slow.times.iter().zip(&slow.poses) {
        let all = rig.pose(Some(at), t);
        for (x, &node) in a.iter().zip(&m.nodes) {
            let y = &all[node];
            let (rotation, translation) = (Quat::from_array(y.rotation), Vec3::from_array(y.translation));
            assert!(x.rotation.dot(rotation).abs() > 1.0 - 1e-6, "{} {rotation}", x.rotation);
            assert!(x.translation.distance(translation) < 1e-4);
            assert!(x.scale.distance(Vec3::from_array(y.scale)) < 1e-4);
        }
    }
}

#[test]
fn writes_only_what_moves() {
    let mut doc = load::document(&path()).unwrap();
    let m = Model::from_document(&doc).unwrap();
    // The rest pose, held: nothing moves, so nothing is keyed.
    let still = Clip {
        times: vec![0.0, 1.0],
        poses: vec![m.skeleton.rest(); 2],
    };
    let a = m.write_clip(&mut doc, None, &still).unwrap();
    assert_eq!(doc.json["animations"][a]["channels"].as_array().unwrap().len(), 0);
    // One bone lifted: only its translation is keyed.
    let mut lifted = still.clone();
    lifted.poses[1][1].translation += Vec3::Y;
    let a = m.write_clip(&mut doc, None, &lifted).unwrap();
    let channels = doc.json["animations"][a]["channels"].as_array().unwrap();
    assert_eq!(channels.len(), 1);
    assert_eq!(channels[0]["target"]["node"], m.nodes[1]);
    assert_eq!(channels[0]["target"]["path"], "translation");
}

#[test]
fn refuses_clips_it_cannot_write() {
    let mut doc = load::document(&path()).unwrap();
    let m = Model::from_document(&doc).unwrap();
    let rest = m.skeleton.rest();
    let backwards = Clip {
        times: vec![1.0, 0.0],
        poses: vec![rest.clone(), rest.clone()],
    };
    let short = Clip {
        times: vec![0.0],
        poses: vec![rest[1..].to_vec()],
    };
    let empty = Clip {
        times: vec![],
        poses: vec![],
    };
    for clip in [backwards, short, empty] {
        assert!(matches!(m.write_clip(&mut doc, None, &clip), Err(load::Error::Clip(_))));
    }
    assert!(
        doc.json
            .get("animations")
            .and_then(|a| a.as_array())
            .is_none_or(|a| a.len() == 1)
    );
}
