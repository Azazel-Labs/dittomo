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
fn writes_bvh_that_reads_back_the_same() {
    let m = Model::load(&path()).unwrap();
    // The file's poses a thirtieth of a second apart, which the BVH keeps pose for pose.
    let clip = Clip {
        times: (0..m.clips[0].1.poses.len()).map(|k| k as f32 / 30.0).collect(),
        poses: m.clips[0].1.poses.clone(),
    };
    let text = dittomo::bvh::write(&m.skeleton, &clip, None).unwrap();
    let back = Model::from_bytes(text.as_bytes(), "bvh").unwrap();
    let played = &back.clips[0].1;
    assert_eq!(played.poses.len(), clip.poses.len());
    let ours: Vec<usize> = (m.skeleton.bones().iter())
        .map(|b| back.skeleton.index_of(&b.name.replace(' ', "_")).expect("every bone"))
        .collect();
    let size = 2.0;
    let near = |a: Vec3, b: Vec3| a.distance(b) < 1e-4 * size;
    // The rest pose is the skeleton's own, with every joint's axes the world's.
    let (rest, back_rest) = (
        m.skeleton.world(&m.skeleton.rest()),
        back.skeleton.world(&back.skeleton.rest()),
    );
    for (b, &o) in ours.iter().enumerate() {
        assert!(
            near(rest[b].position(), back_rest[o].position()),
            "{}",
            m.skeleton.bones()[b].name
        );
        assert!(back_rest[o].rotation.angle_between(Quat::IDENTITY) < 1e-6);
    }
    // Each pose puts each bone in the same place, turned the same way from rest.
    for (a, b) in clip.poses.iter().zip(&played.poses) {
        let (wa, wb) = (m.skeleton.world(a), back.skeleton.world(b));
        for (i, &o) in ours.iter().enumerate() {
            assert!(
                near(wa[i].position(), wb[o].position()),
                "{} {}",
                wa[i].position(),
                wb[o].position()
            );
            let turn = wa[i].rotation * rest[i].rotation.inverse();
            assert!(turn.angle_between(wb[o].rotation) < 1e-3, "{turn} {}", wb[o].rotation);
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

/// `bytes` in base64, for a glTF's data URI.
fn base64(bytes: &[u8]) -> String {
    let table = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut data = String::new();
    for c in bytes.chunks(3) {
        let n = c
            .iter()
            .enumerate()
            .fold(0u32, |n, (i, &b)| n | u32::from(b) << (16 - 8 * i));
        for i in 0..4 {
            let at = (n >> (18 - 6 * i)) as usize & 63;
            data.push(if i <= c.len() { char::from(table[at]) } else { '=' });
        }
    }
    data
}

/// A glTF of hips and two legs bound standing straight, saved with the left leg swung forward, as a rig saved in a
/// frame of its walk is.
fn mid_stride() -> String {
    let bound = [
        [0.0, 1.0, 0.0],
        [0.2, 1.0, 0.0],
        [0.2, 0.5, 0.0],
        [-0.2, 1.0, 0.0],
        [-0.2, 0.5, 0.0],
    ];
    let mut bytes = Vec::new();
    for p in bound {
        let m = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -p[0], -p[1], -p[2], 1.0f32,
        ];
        bytes.extend(m.iter().flat_map(|f| f.to_le_bytes()));
    }
    let data = base64(&bytes);
    let (s, c) = (15f32.to_radians().sin(), 15f32.to_radians().cos());
    format!(
        r#"{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[0,5]}}],
        "nodes":[{{"name":"Hips","translation":[0,1,0],"children":[1,3]}},
        {{"name":"LeftLeg","translation":[0.2,0,0],"rotation":[{s},0,0,{c}],"children":[2]}},
        {{"name":"LeftFoot","translation":[0,-0.5,0]}},
        {{"name":"RightLeg","translation":[-0.2,0,0],"children":[4]}},
        {{"name":"RightFoot","translation":[0,-0.5,0]}},
        {{"name":"Body","skin":0}}],
        "skins":[{{"joints":[0,1,2,3,4],"inverseBindMatrices":0}}],
        "accessors":[{{"bufferView":0,"componentType":5126,"count":5,"type":"MAT4"}}],
        "bufferViews":[{{"buffer":0,"byteLength":{}}}],
        "buffers":[{{"byteLength":{},"uri":"data:application/octet-stream;base64,{data}"}}]}}"#,
        bytes.len(),
        bytes.len()
    )
}

#[test]
fn a_rig_saved_mid_stride_rests_as_it_was_bound() {
    let text = mid_stride();
    let m = Model::from_bytes(text.as_bytes(), "gltf").unwrap();
    let world = m.skeleton.world(&m.skeleton.rest());
    let foot = world[2].matrix.w_axis.truncate();
    assert!(foot.distance(Vec3::new(0.2, 0.5, 0.0)) < 1e-5, "{foot}");
    assert!(m.skeleton.bones()[1].rest.rotation.angle_between(Quat::IDENTITY) < 1e-5);
    // Written back, the leg is keyed straight, as the file's own node still stands swung.
    let mut doc = load::document_from_bytes(text.as_bytes(), "gltf").unwrap();
    let still = Clip {
        times: vec![0.0, 1.0],
        poses: vec![m.skeleton.rest(); 2],
    };
    let a = m.write_clip(&mut doc, None, &still).unwrap();
    let channels = doc.json["animations"][a]["channels"].as_array().unwrap();
    assert_eq!(channels.len(), 1);
    assert_eq!(channels[0]["target"]["node"], 1);
    assert_eq!(channels[0]["target"]["path"], "rotation");
}

#[test]
fn a_rig_resting_as_chosen_keeps_its_rest() {
    // Both legs swung alike: no more lopsided than as bound, so the rest is as the nodes stand.
    let (s, c) = (15f32.to_radians().sin(), 15f32.to_radians().cos());
    let text = mid_stride().replace(
        r#""translation":[-0.2,0,0],"children""#,
        &format!(r#""translation":[-0.2,0,0],"rotation":[{s},0,0,{c}],"children""#),
    );
    let m = Model::from_bytes(text.as_bytes(), "gltf").unwrap();
    for b in [1, 3] {
        assert!(m.skeleton.bones()[b].rest.rotation.angle_between(Quat::IDENTITY) > 0.2);
    }
}

/// The corners and 12 triangles of a box, facing out: 24 corners, four to a side so each side has its own normals,
/// or the 8 shared, as a low-poly mesh is often drawn, its normals then pointing out from the corners.
fn cuboid(center: Vec3, half: Vec3, shared: bool) -> (Vec<Vec3>, Vec<u16>) {
    let (mut at, mut faces) = (Vec::new(), Vec::new());
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let n = Vec3::AXES[axis] * sign;
            let (mut u, mut v) = (Vec3::AXES[(axis + 1) % 3], Vec3::AXES[(axis + 2) % 3]);
            if sign < 0.0 {
                (u, v) = (v, u);
            }
            let mut corner = |p: Vec3| match at.iter().position(|&q| shared && q == p) {
                Some(i) => i as u16,
                None => {
                    at.push(p);
                    at.len() as u16 - 1
                }
            };
            let side = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
                .map(|(a, b)| corner(center + (n + u * a + v * b) * half));
            faces.extend([0, 1, 2, 0, 2, 3].map(|i| side[i]));
        }
    }
    (at, faces)
}

/// A glTF of a box for a chest, its sides far wider than the skin's spacing, its corners shared, and a box 2 cm wide
/// for a hand with its middle at `hand`.
fn chest_and_hand(hand: Vec3) -> String {
    let (mut at, mut faces) = cuboid(Vec3::new(0.0, 1.25, 0.0), Vec3::new(0.3, 0.35, 0.2), true);
    let (hand_at, hand_faces) = cuboid(hand, Vec3::splat(0.01), false);
    faces.extend(hand_faces.iter().map(|&i| i + at.len() as u16));
    let joints: Vec<u8> = (0..at.len() + hand_at.len())
        .flat_map(|i| [if i < at.len() { 1 } else { 2 }, 0, 0, 0])
        .collect();
    at.extend(hand_at);
    let mut bytes = Vec::new();
    for p in [[0.0, 1.0, 0.0], [0.0, 1.25, 0.0], hand.to_array(), [0.0, 0.0, 0.0]] {
        let m = [
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -p[0], -p[1], -p[2], 1.0f32,
        ];
        bytes.extend(m.iter().flat_map(|f| f.to_le_bytes()));
    }
    let views = [bytes.len(), at.len() * 12, joints.len(), at.len() * 16, faces.len() * 2];
    bytes.extend(at.iter().flat_map(Vec3::to_array).flat_map(f32::to_le_bytes));
    bytes.extend(&joints);
    bytes.extend(
        (0..at.len())
            .flat_map(|_| [1.0f32, 0.0, 0.0, 0.0])
            .flat_map(f32::to_le_bytes),
    );
    bytes.extend(faces.iter().flat_map(|i| i.to_le_bytes()));
    let mut offset = 0;
    let views: Vec<String> = views
        .iter()
        .map(|&len| {
            let view = format!(r#"{{"buffer":0,"byteOffset":{offset},"byteLength":{len}}}"#);
            offset += len;
            view
        })
        .collect();
    let (lo, hi) = at
        .iter()
        .fold((Vec3::MAX, Vec3::MIN), |(lo, hi), &p| (lo.min(p), hi.max(p)));
    format!(
        r#"{{"asset":{{"version":"2.0"}},"scene":0,"scenes":[{{"nodes":[0,4]}}],
        "nodes":[{{"name":"Hips","translation":[0,1,0],"children":[1,2,3]}},
        {{"name":"Chest","translation":[0,0.25,0]}},
        {{"name":"Hand","translation":{hand_rel:?}}},
        {{"name":"Foot","translation":[0,-1,0]}},
        {{"name":"Body","skin":0,"mesh":0}}],
        "skins":[{{"joints":[0,1,2,3],"inverseBindMatrices":0}}],
        "meshes":[{{"primitives":[{{"attributes":{{"POSITION":1,"JOINTS_0":2,"WEIGHTS_0":3}},"indices":4}}]}}],
        "accessors":[{{"bufferView":0,"componentType":5126,"count":4,"type":"MAT4"}},
        {{"bufferView":1,"componentType":5126,"count":{n},"type":"VEC3","min":{lo:?},"max":{hi:?}}},
        {{"bufferView":2,"componentType":5121,"count":{n},"type":"VEC4"}},
        {{"bufferView":3,"componentType":5126,"count":{n},"type":"VEC4"}},
        {{"bufferView":4,"componentType":5123,"count":{f},"type":"SCALAR"}}],
        "bufferViews":[{views}],
        "buffers":[{{"byteLength":{len},"uri":"data:application/octet-stream;base64,{data}"}}]}}"#,
        n = at.len(),
        hand_rel = (hand - Vec3::Y).to_array(),
        f = faces.len(),
        lo = lo.to_array(),
        hi = hi.to_array(),
        views = views.join(","),
        len = bytes.len(),
        data = base64(&bytes),
    )
}

#[test]
fn a_low_poly_body_has_skin_across_its_faces() {
    // 5 cm in front of the chest, off to the side, where its nearest corners face out every which way; and 2 cm into
    // its side. The gap is from the hand's middle to the skin, less the hand's half-width.
    for (hand, want) in [(Vec3::new(0.2, 1.45, 0.25), 0.04), (Vec3::new(0.28, 1.2, 0.0), -0.03)] {
        let m = Model::from_bytes(chest_and_hand(hand).as_bytes(), "gltf").unwrap();
        let hand = m.skeleton.index_of("Hand").unwrap();
        let world = m.skeleton.world(&m.skeleton.rest());
        let gap = m.skin.gap(&m.skeleton, &world, hand, &[hand]).unwrap();
        assert!((gap - want).abs() < 0.01, "{gap} for {want}");
    }
}
