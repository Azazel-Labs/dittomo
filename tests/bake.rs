//! Clips baked into a library and read back: what a shader would skin with against the clip itself, loops, the clip
//! table, bad and cut-off bytes, sizes, and the WGSL through naga.

#[path = "../benches/common/mod.rs"]
#[allow(dead_code)]
mod common;

use std::f32::consts::TAU;

use dittomo::bake::{self, Baker, Encoding, Entry, Library};
use dittomo::{Bone, Clip, Fixes, Mapping, Mat4, Quat, Skeleton, Transform, Vec3};

/// Mixamo's rig with every bone swinging, walking off 4.2 m or kept in place, played on the mannequin with every fix:
/// 90 poses at 30 a second.
fn retargeted(travels: bool) -> (Skeleton, Clip) {
    let (source, target) = (common::mixamo(), common::mannequin());
    let mut walk = common::walk(&source, 90);
    if !travels {
        for pose in &mut walk.poses {
            for (tr, b) in pose.iter_mut().zip(source.bones()) {
                tr.translation.z = b.rest.translation.z;
            }
        }
    }
    let map = Mapping::new(&source, &target);
    let fixes = if travels { Fixes::ALL } else { Fixes::default() };
    let clip = map.apply_with(&source, &target, &walk, fixes).expect("a retarget");
    (target, clip)
}

/// How tall the rig stands at rest.
fn height(sk: &Skeleton) -> f32 {
    let ys = sk.world(&sk.rest()).into_iter().map(|p| p.position().y);
    ys.clone().fold(f32::MIN, f32::max) - ys.fold(f32::MAX, f32::min)
}

/// The clip's pose at `t`, between its keys, each bone under its parent: how an engine plays it.
fn pose_at(clip: &Clip, t: f32) -> Vec<Transform> {
    let i = clip.times.partition_point(|&k| k <= t).clamp(1, clip.times.len() - 1);
    let f = ((t - clip.times[i - 1]) / (clip.times[i] - clip.times[i - 1])).clamp(0.0, 1.0);
    (clip.poses[i - 1].iter().zip(&clip.poses[i]))
        .map(|(a, b)| Transform {
            translation: a.translation.lerp(b.translation, f),
            rotation: a.rotation.slerp(b.rotation, f),
            scale: a.scale.lerp(b.scale, f),
        })
        .collect()
}

/// Each bone's skinning matrix at a pose: its world matrix times its inverse bind, the bind being the rest pose.
fn skinning(sk: &Skeleton, pose: &[Transform]) -> Vec<Mat4> {
    let rest = sk.world(&sk.rest());
    (sk.world(pose).iter().zip(&rest))
        .map(|(now, bind)| now.matrix * bind.matrix.inverse())
        .collect()
}

/// The leaves under a hand, each with its three bones above: a fingertip's four influences.
fn fingers(sk: &Skeleton) -> Vec<[usize; 4]> {
    let bones = sk.bones();
    let up = |b: usize| bones[b].parent.expect("a finger has bones above it");
    let leaf = |b: usize| bones.iter().all(|c| c.parent != Some(b));
    let under_hand = |mut b: usize| {
        while let Some(p) = bones[b].parent {
            if bones[p].name.starts_with("hand") {
                return true;
            }
            b = p;
        }
        false
    };
    (0..bones.len())
        .filter(|&b| leaf(b) && under_hand(b))
        .map(|b| [b, up(b), up(up(b)), up(up(up(b)))])
        .collect()
}

const WEIGHTS: [f32; 4] = [0.5, 0.3, 0.15, 0.05];

/// How far the library puts points from where the clip does at these times, at worst, over the rig's height: every
/// joint and points a twentieth of the height off it moved by its bone alone, then each fingertip (its last bone
/// carried on by its own length) moved by its last bone alone and by its last four.
fn errors(sk: &Skeleton, clip: &Clip, library: &Library, times: &[f32]) -> (f32, f32) {
    let tall = height(sk);
    let rest = sk.world(&sk.rest());
    let at = |b: usize| rest[b].position();
    let tips = fingers(sk);
    assert_eq!(tips.len(), 10, "five fingertips a hand");
    let (mut all, mut tip) = (0f32, 0f32);
    for &t in times {
        let truth = skinning(sk, &pose_at(clip, clip.times[0] + t));
        let baked: Vec<Transform> = (0..sk.len()).map(|b| library.bone(0, t, b as u32)).collect();
        for b in 0..sk.len() {
            for off in [
                Vec3::ZERO,
                Vec3::X,
                Vec3::NEG_X,
                Vec3::Y,
                Vec3::NEG_Y,
                Vec3::Z,
                Vec3::NEG_Z,
            ] {
                let v = at(b) + off * 0.05 * tall;
                let got = baked[b].rotation * v + baked[b].translation;
                all = all.max(got.distance(truth[b].transform_point3(v)));
            }
        }
        for chain in &tips {
            let v = at(chain[0]) * 2.0 - at(chain[1]);
            let alone = baked[chain[0]].rotation * v + baked[chain[0]].translation;
            tip = tip.max(alone.distance(truth[chain[0]].transform_point3(v)));
            let blended = bake::skin(v, &chain.map(|b| baked[b]), WEIGHTS);
            let want: Vec3 = (chain.iter().zip(WEIGHTS))
                .map(|(&b, w)| truth[b].transform_point3(v) * w)
                .sum();
            tip = tip.max(blended.distance(want));
        }
    }
    (all / tall, tip / tall)
}

fn baked(sk: &Skeleton, clip: &Clip, encoding: Encoding, looping: bool) -> Vec<u8> {
    let mut baker = Baker::new(sk, encoding);
    assert_eq!(baker.add(clip, 30.0, looping), Ok(0));
    baker.finish()
}

/// The farthest any joint gets from the origin along an axis over the clip.
fn reach(sk: &Skeleton, clip: &Clip) -> f32 {
    let far = |pose: &Vec<Transform>| {
        sk.world(pose)
            .iter()
            .map(|p| p.position().abs().max_element())
            .fold(0.0, f32::max)
    };
    clip.poses.iter().map(far).fold(0.0, f32::max)
}

#[test]
fn a_retargeted_clip_comes_back_at_its_frames() {
    let on_frames: Vec<f32> = (0..90).map(|k| k as f32 / 30.0).collect();
    // Past either end the clip holds its first and last pose.
    let outside = [-1.0, 89.0 / 30.0 + 0.5];
    for travels in [false, true] {
        let (sk, clip) = retargeted(travels);
        let tall = height(&sk);
        let reach = reach(&sk, &clip) / tall;
        // Floats are as good as the arithmetic. A half has 11 bits, so it places a joint within a 2048th of how far
        // out it stands along each axis, and turns a bone within about a thousandth of a radian.
        let half = reach * 3f32.sqrt() / 2048.0 + 1e-4;
        for (encoding, bound) in [(Encoding::F32, 2e-6 * reach.max(1.0)), (Encoding::Half, half)] {
            let bytes = baked(&sk, &clip, encoding, false);
            let library = Library::read(&bytes).expect("a library");
            assert_eq!(
                (library.encoding(), library.bones(), library.clips()),
                (encoding, sk.len(), 1)
            );
            let entry = library.entry(0).expect("its clip");
            assert_eq!((entry.frames, entry.loops()), (90, false));
            assert!((entry.rate - 30.0).abs() < 1e-3 && (entry.seconds() - 89.0 / 30.0).abs() < 1e-4);
            let (all, tip) = errors(&sk, &clip, &library, &on_frames);
            println!(
                "{encoding:?} at the frames, {reach:.1} heights out: {all:.2e} of the height, fingertips {tip:.2e}"
            );
            assert!(
                all < bound && tip < bound,
                "{encoding:?}: {all} and {tip} of the height, over {bound}"
            );
            let (all, tip) = errors(&sk, &clip, &library, &outside);
            assert!(
                all < bound && tip < bound,
                "{encoding:?} held at its ends: {all} and {tip}"
            );
        }
    }
}

#[test]
fn between_frames_it_stays_near_the_clip() {
    // The clip turns each bone under its parent between keys; the library moves each joint straight and turns each
    // bone in the world. With every bone swinging 23 degrees each way once a second, 30 frames a second keep the two
    // within half a hundredth of the height, in either encoding.
    let (sk, clip) = retargeted(false);
    let between: Vec<f32> = (0..89 * 4).map(|k| k as f32 / 120.0).collect();
    for encoding in [Encoding::F32, Encoding::Half] {
        let bytes = baked(&sk, &clip, encoding, false);
        let library = Library::read(&bytes).expect("a library");
        let (all, tip) = errors(&sk, &clip, &library, &between);
        println!("{encoding:?} between frames: {all:.2e} of the height at worst, {tip:.2e} at the fingertips");
        assert!(all < 5e-3 && tip < 5e-3, "{encoding:?}: {all} and {tip} of the height");
    }
}

#[test]
fn joints_blend_closer_than_skinning_translations_would() {
    // Why a record holds where the joint stands, not the skinning translation: half way between two frames, blending
    // the translations pulls every joint toward the rig's origin.
    let (sk, clip) = retargeted(true);
    let bytes = baked(&sk, &clip, Encoding::F32, false);
    let library = Library::read(&bytes).expect("a library");
    let rest = sk.world(&sk.rest());
    let (mut joints, mut translations) = (0f32, 0f32);
    for k in 0..89 {
        let truth = sk.world(&pose_at(&clip, (k as f32 + 0.5) / 30.0));
        for b in 0..sk.len() {
            let pivot = rest[b].position();
            let (a, z) = (library.stored(0, k, b).unwrap(), library.stored(0, k + 1, b).unwrap());
            let turn = a.rotation.lerp(z.rotation, 0.5);
            let by_joint = a.translation.lerp(z.translation, 0.5);
            let by_translation =
                turn * pivot + (a.translation - a.rotation * pivot).lerp(z.translation - z.rotation * pivot, 0.5);
            joints = joints.max(by_joint.distance(truth[b].position()));
            translations = translations.max(by_translation.distance(truth[b].position()));
        }
    }
    let tall = height(&sk);
    println!(
        "half way between frames a joint is off by {:.2e} of the height at worst, {:.2e} were translations stored",
        joints / tall,
        translations / tall
    );
    assert!(joints * 2.0 < translations, "{joints} against {translations}");
}

/// A second of Mixamo's rig turning once round on the spot with every bone swinging, the last pose the first again.
fn spin() -> (Skeleton, Clip) {
    let sk = common::mixamo();
    let mut clip = Clip::default();
    for f in 0..=30 {
        let t = f as f32 / 30.0;
        let mut pose = sk.rest();
        for (i, (tr, b)) in pose.iter_mut().zip(sk.bones()).enumerate() {
            let k = i as f32;
            let axis = Vec3::new((k * 1.3).sin(), (k * 0.7).cos(), (k * 2.1).sin() + 0.3).normalize();
            tr.rotation = b.rest.rotation * Quat::from_axis_angle(axis, 0.4 * (TAU * t + k).sin());
            if b.name == "mixamorig:Hips" {
                tr.rotation = Quat::from_rotation_y(TAU * t) * tr.rotation;
                tr.translation.y += 3.0 * (2.0 * TAU * t).sin();
            }
        }
        clip.times.push(t);
        clip.poses.push(pose);
    }
    (sk, clip)
}

#[test]
fn a_looping_clip_wraps_without_a_pop() {
    let (sk, clip) = spin();
    let hand = sk.index_of("mixamorig:LeftHandIndex4").expect("a fingertip") as u32;
    let tip = sk.world(&sk.rest())[hand as usize].position();
    for encoding in [Encoding::F32, Encoding::Half] {
        let bytes = baked(&sk, &clip, encoding, true);
        let library = Library::read(&bytes).expect("a library");
        let entry = library.entry(0).expect("its clip");
        assert_eq!(
            (entry.frames, entry.loops()),
            (30, true),
            "the last pose is the first, and isn't stored"
        );
        assert!((entry.seconds() - 1.0).abs() < 1e-5);

        // The body has turned once round, so its last frame and its first lie in opposite hemispheres: the wrap is
        // the one place a plain blend would go the long way.
        let hips = sk.index_of("mixamorig:Hips").unwrap();
        let (first, last) = (
            library.stored(0, 0, hips).unwrap(),
            library.stored(0, 29, hips).unwrap(),
        );
        assert!(first.rotation.dot(last.rotation) < -0.9);

        let at = |t: f32| {
            let b = library.bone(0, t, hand);
            b.rotation * tip + b.translation
        };
        // Time wraps both ways.
        for t in [0.0, 0.26, 0.5, 0.99] {
            for turns in [-2.0, -1.0, 1.0, 3.0] {
                assert!(at(t + turns).distance(at(t)) < 0.02, "{t} and {turns} turns on");
            }
        }
        // Stepping finely through two turns, the fingertip moves no further in a step across the wrap than it does
        // in the fastest step elsewhere.
        let step = 1.0 / 480.0;
        let moved = |k: i32| at(k as f32 * step).distance(at((k + 1) as f32 * step));
        let elsewhere = (40..440).map(moved).fold(0.0, f32::max);
        let across = (440..520).map(moved).fold(0.0, f32::max);
        assert!(elsewhere > 0.1, "it moves");
        assert!(
            across <= elsewhere * 1.05,
            "{across} a step across the wrap, {elsewhere} elsewhere"
        );
    }
}

#[test]
fn rotations_keep_to_one_hemisphere_along_a_clip() {
    // Two turns round: the hips' rotation as posed changes sign on the way.
    let (sk, mut clip) = spin();
    let again: Vec<_> = clip.poses[1..].to_vec();
    clip.times.extend((31..=60).map(|f| f as f32 / 30.0));
    clip.poses.extend(again);
    for encoding in [Encoding::F32, Encoding::Half] {
        let bytes = baked(&sk, &clip, encoding, false);
        let library = Library::read(&bytes).expect("a library");
        assert_eq!(library.entry(0).unwrap().frames, 61);
        for b in 0..sk.len() {
            assert!(library.stored(0, 0, b).unwrap().rotation.w >= 0.0);
            for k in 0..60 {
                let (a, z) = (library.stored(0, k, b).unwrap(), library.stored(0, k + 1, b).unwrap());
                assert!(a.rotation.dot(z.rotation) > 0.9, "bone {b} from frame {k} to the next");
            }
        }
    }
}

#[test]
fn the_clip_table_finds_each_of_several_clips() {
    let (sk, walk) = retargeted(true);
    let short = Clip {
        times: walk.times[..31].to_vec(),
        poses: walk.poses[10..41].to_vec(),
    };
    let still = Clip {
        times: vec![2.0],
        poses: vec![walk.poses[50].clone()],
    };
    // Each clip, how it's baked, and the frames that makes: 1 s looping at 24, 89/30 s at 60, one pose, 1 s at 30.
    let clips = [
        (&short, 24.0, true, 24),
        (&walk, 60.0, false, 179),
        (&still, 30.0, true, 1),
        (&short, 30.0, false, 31),
    ];
    for encoding in [Encoding::F32, Encoding::Half] {
        let mut baker = Baker::new(&sk, encoding);
        for (i, &(clip, rate, looping, _)) in clips.iter().enumerate() {
            assert_eq!(baker.add(clip, rate, looping), Ok(i));
        }
        let bytes = baker.finish();
        let library = Library::read(&bytes).expect("a library");
        assert_eq!(library.clips(), 4);
        assert_eq!(library.entry(4), None);

        let header = library.header();
        assert_eq!((header.magic, header.version), (bake::MAGIC, bake::VERSION));
        assert_eq!((header.table, header.pivots), (8, 8 + 4 * 4));
        assert_eq!(header.words as usize * 4, bytes.len());
        let mut first = header.pivots as usize + 3 * sk.len();
        for (i, &(clip, rate, looping, frames)) in clips.iter().enumerate() {
            let entry = library.entry(i).expect("the clip");
            assert_eq!(
                (entry.first as usize, entry.frames, entry.loops()),
                (first, frames, looping),
                "clip {i}"
            );
            let seconds = clip.times[clip.times.len() - 1] - clip.times[0];
            assert_eq!(bake::frames(seconds, rate, looping), frames as usize);
            first += frames as usize * sk.len() * encoding.record();

            // Each plays as it does baked alone.
            let mut alone = Baker::new(&sk, encoding);
            alone.add(clip, rate, looping).unwrap();
            let alone = alone.finish();
            let alone = Library::read(&alone).unwrap();
            for t in [0.0, 0.31, 0.77, 2.5] {
                for b in [0, 7, sk.len() as u32 - 1] {
                    assert_eq!(
                        library.bone(i as u32, t, b),
                        alone.bone(0, t, b),
                        "clip {i} at {t}, bone {b}"
                    );
                }
            }
        }
        assert_eq!(first * 4, bytes.len(), "the last clip ends the library");
        for (b, placed) in sk.world(&sk.rest()).iter().enumerate() {
            assert_eq!(library.pivot_of(b), Some(placed.position()));
        }
    }
}

#[test]
fn a_cross_fade_goes_from_one_clip_to_the_other() {
    let (sk, walk) = retargeted(true);
    let mut baker = Baker::new(&sk, Encoding::F32);
    baker.add(&walk, 30.0, false).unwrap();
    let late = Clip {
        times: walk.times[..60].to_vec(),
        poses: walk.poses[30..].to_vec(),
    };
    baker.add(&late, 30.0, false).unwrap();
    let bytes = baker.finish();
    let library = Library::read(&bytes).unwrap();
    for b in 0..sk.len() as u32 {
        let (a, z) = (library.bone(0, 0.4, b), library.bone(1, 1.1, b));
        let near = |x: Transform, y: Transform| {
            x.translation.distance(y.translation) < 1e-3 && x.rotation.dot(y.rotation).abs() > 1.0 - 1e-6
        };
        assert!(near(library.blend(0, 0.4, 1, 1.1, 0.0, b), a));
        assert!(near(library.blend(0, 0.4, 1, 1.1, 1.0, b), z));
        // Half way, the joint stands half way between where the two clips have it.
        let (ja, jz) = (library.joint(0, 0.4, b), library.joint(1, 1.1, b));
        let pivot = library.pivot_of(b as usize).unwrap();
        let mid = library.blend(0, 0.4, 1, 1.1, 0.5, b);
        let stands = mid.rotation * pivot + mid.translation;
        assert!(stands.distance(ja.translation.lerp(jz.translation, 0.5)) < 1e-3);
    }
}

#[test]
fn inverse_binds_other_than_the_rest_pose_are_taken() {
    // The mesh bound with the whole rig turned a quarter round and moved: the rest pose then skins to that turn undone.
    let (sk, clip) = retargeted(true);
    let moved = Mat4::from_rotation_translation(Quat::from_rotation_y(TAU / 4.0), Vec3::new(10.0, 0.0, -5.0));
    let binds: Vec<Mat4> = sk
        .world(&sk.rest())
        .iter()
        .map(|p| (moved * p.matrix).inverse())
        .collect();
    let mut baker = Baker::new(&sk, Encoding::F32).bound(&binds).expect("one a bone");
    baker.add(&clip, 30.0, false).unwrap();
    let bytes = baker.finish();
    let library = Library::read(&bytes).unwrap();
    let truth = sk.world(&clip.poses[40]);
    let v = Vec3::new(30.0, 120.0, 8.0);
    for b in 0..sk.len() {
        let got = library.bone(0, 40.0 / 30.0, b as u32);
        let want = (truth[b].matrix * binds[b]).transform_point3(v);
        assert!((got.rotation * v + got.translation).distance(want) < 1e-2, "bone {b}");
    }
    assert_eq!(
        Baker::new(&sk, Encoding::F32).bound(&binds[1..]).err(),
        Some(bake::Error::Binds {
            binds: sk.len() - 1,
            expected: sk.len()
        })
    );
}

/// A rig of three bones and a library of two short clips on it.
fn small(encoding: Encoding) -> Vec<u8> {
    let bone = |name: &str, parent| Bone {
        name: name.into(),
        parent,
        rest: Transform::new(Vec3::Y, Quat::IDENTITY),
    };
    let sk = Skeleton::new(vec![bone("hips", None), bone("spine", Some(0)), bone("head", Some(1))]).unwrap();
    let clip = common::walk(&sk, 5);
    let mut baker = Baker::new(&sk, encoding);
    baker.add(&clip, 30.0, true).unwrap();
    baker.add(&clip, 30.0, false).unwrap();
    baker.finish()
}

/// Plays everything a library says it holds, and some it doesn't: nothing may panic.
fn play(library: &Library) {
    for clip in 0..library.clips() as u32 + 2 {
        for bone in 0..library.bones() as u32 + 2 {
            for time in [
                -1e9,
                -0.3,
                0.0,
                0.05,
                0.5,
                1e9,
                f32::NAN,
                f32::INFINITY,
                f32::NEG_INFINITY,
            ] {
                let _ = library.bone(clip, time, bone);
                let _ = library.joint(clip, time, bone);
                let _ = library.blend(clip, time, clip + 1, 0.1, 0.5, bone);
            }
            let _ = library.pivot_of(bone as usize);
            let _ = library.stored(clip as usize, 0, bone as usize);
            let _ = library.stored(clip as usize, usize::MAX, bone as usize);
        }
        if let Some(entry) = library.entry(clip as usize) {
            let _ = entry.seconds();
        }
    }
}

#[test]
fn a_wrong_magic_or_version_is_turned_away() {
    let bytes = small(Encoding::Half);
    assert!(Library::read(&bytes).is_ok());
    let patched = |word: usize, value: u32| {
        let mut b = bytes.clone();
        b[word * 4..word * 4 + 4].copy_from_slice(&value.to_le_bytes());
        b
    };
    assert_eq!(Library::read(&patched(0, 0)).err(), Some(bake::Error::NotALibrary));
    assert_eq!(
        Library::read(&patched(0, bake::MAGIC.swap_bytes())).err(),
        Some(bake::Error::NotALibrary)
    );
    assert_eq!(
        Library::read(b"glTF\x02\0\0\0 and then some more bytes of another file").err(),
        Some(bake::Error::NotALibrary)
    );
    assert_eq!(Library::read(&patched(1, 0)).err(), Some(bake::Error::Version(0)));
    assert_eq!(
        Library::read(&patched(1, bake::VERSION + 1)).err(),
        Some(bake::Error::Version(bake::VERSION + 1))
    );
    assert_eq!(Library::read(&patched(2, 2)).err(), Some(bake::Error::Malformed));
}

#[test]
fn cut_short_is_an_error_and_never_a_panic() {
    for encoding in [Encoding::F32, Encoding::Half] {
        let bytes = small(encoding);
        for len in 0..bytes.len() {
            assert_eq!(
                Library::read(&bytes[..len]).err(),
                Some(bake::Error::Truncated),
                "{len} bytes of {}",
                bytes.len()
            );
        }
        // Bytes past its end are another's, in a pool.
        let mut pooled = bytes.clone();
        pooled.extend([0xAB; 7]);
        let library = Library::read(&pooled).expect("a library with more after it");
        assert_eq!(library.header().words as usize * 4, bytes.len());
        play(&library);
    }
}

#[test]
fn a_header_or_table_that_lies_is_an_error_or_harmless() {
    for encoding in [Encoding::F32, Encoding::Half] {
        let bytes = small(encoding);
        let words = (bytes.len() / 4) as u32;
        let values = [
            0,
            1,
            2,
            3,
            7,
            8,
            9,
            16,
            words - 1,
            words,
            words + 1,
            0x7FFF_FFFF,
            0x8000_0000,
            u32::MAX,
        ]
        .into_iter()
        .chain([f32::NAN, f32::INFINITY, -30.0, 0.0, 1e-30, 1e30].map(f32::to_bits));
        let (mut read, mut refused) = (0, 0);
        for value in values {
            // The header's eight words and the two clips' four each.
            for word in 0..16 {
                let mut b = bytes.clone();
                b[word * 4..word * 4 + 4].copy_from_slice(&value.to_le_bytes());
                match Library::read(&b) {
                    Ok(library) => {
                        play(&library);
                        read += 1;
                    }
                    Err(_) => refused += 1,
                }
            }
        }
        assert!(read > 20 && refused > 100, "{read} read, {refused} refused");
    }
}

#[test]
fn what_cant_be_baked_is_an_error() {
    let (sk, clip) = retargeted(true);
    let mut baker = Baker::new(&sk, Encoding::Half);
    for rate in [0.0, -30.0, f32::NAN, f32::INFINITY] {
        assert_eq!(baker.add(&clip, rate, false), Err(bake::Error::Rate));
    }
    assert_eq!(
        baker.add(&Clip::default(), 30.0, false),
        Err(bake::Error::Clip(dittomo::Error::ClipTimes))
    );
    let other = common::walk(&common::mixamo(), 3);
    assert!(matches!(
        baker.add(&other, 30.0, false),
        Err(bake::Error::Clip(dittomo::Error::PoseSize { .. }))
    ));
    // Nothing was added: an empty library is still one, and plays as the bind pose.
    let bytes = baker.finish();
    let library = Library::read(&bytes).expect("an empty library");
    assert_eq!(
        (library.clips(), bytes.len()),
        (0, bake::size(Encoding::Half, sk.len(), 0, 0))
    );
    assert_eq!(library.bone(0, 0.0, 3), Transform::IDENTITY);
}

#[test]
fn a_library_is_as_big_as_the_formula_says() {
    let (sk, walk) = retargeted(true);
    for encoding in [Encoding::F32, Encoding::Half] {
        let mut baker = Baker::new(&sk, encoding);
        let mut frames = 0;
        for (clips, (rate, looping)) in [(24.0, true), (30.0, false), (50.0, false)].into_iter().enumerate() {
            baker.add(&walk, rate, looping).unwrap();
            frames += bake::frames(89.0 / 30.0, rate, looping);
            assert_eq!(baker.finish().len(), bake::size(encoding, sk.len(), clips + 1, frames));
        }
        assert_eq!(frames, 71 + 90 + 150);
    }
    // The numbers in the module's notes: fifty clips of 3 s at 30 Hz on 80 bones.
    let (looped, once) = (50 * bake::frames(3.0, 30.0, true), 50 * bake::frames(3.0, 30.0, false));
    assert_eq!((looped, once), (50 * 90, 50 * 91));
    assert_eq!(bake::size(Encoding::Half, 80, 50, looped), 5_761_792);
    assert_eq!(bake::size(Encoding::F32, 80, 50, looped), 10_081_792);
    assert_eq!(bake::size(Encoding::Half, 80, 50, once), 5_825_792);
    assert_eq!(bake::size(Encoding::F32, 80, 50, once), 10_193_792);
}

#[test]
fn the_wgsl_parses_and_validates() {
    use naga::valid::{Capabilities, ValidationFlags, Validator};

    // The sampler, and a vertex shader that uses all of it as a host would.
    let source = bake::wgsl(1, 3)
        + r"
struct Instance {
    @location(4) clips: vec2<u32>,
    @location(5) times: vec2<f32>,
    @location(6) weight: f32,
}

@vertex
fn vertex(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) bones: vec4<u32>,
    @location(3) weights: vec4<f32>,
    instance: Instance,
) -> @builtin(position) vec4<f32> {
    let a = blend(instance.clips.x, instance.times.x, instance.clips.y, instance.times.y, instance.weight, bones.x);
    let b = blend(instance.clips.x, instance.times.x, instance.clips.y, instance.times.y, instance.weight, bones.y);
    let c = bone(instance.clips.x, instance.times.x, bones.z);
    let d = bone(instance.clips.x, instance.times.x, bones.w);
    let n = skin_normal(normal, a, b, c, d, weights);
    return vec4<f32>(skin(position, a, b, c, d, weights) + n * 0.0, 1.0);
}
";
    let module = naga::front::wgsl::parse_str(&source).unwrap_or_else(|e| panic!("{}", e.emit_to_string(&source)));
    let mut validator = Validator::new(ValidationFlags::all(), Capabilities::empty());
    if let Err(e) = validator.validate(&module) {
        panic!("{}", e.emit_to_string(&source));
    }

    // The library is bound where asked, read-only, as words.
    let (_, library) = (module.global_variables.iter())
        .find(|(_, g)| g.name.as_deref() == Some("library"))
        .expect("the library");
    let binding = library.binding.as_ref().expect("a binding");
    assert_eq!((binding.group, binding.binding), (1, 3));
    assert_eq!(
        library.space,
        naga::AddressSpace::Storage {
            access: naga::StorageAccess::LOAD
        }
    );

    // Its structs' fields are the reader's, in the layout's order.
    let fields = |name: &str| -> Vec<String> {
        let (_, ty) = (module.types.iter())
            .find(|(_, t)| t.name.as_deref() == Some(name))
            .expect("the struct");
        let naga::TypeInner::Struct { members, .. } = &ty.inner else {
            panic!("{name} isn't a struct")
        };
        members.iter().map(|m| m.name.clone().unwrap()).collect()
    };
    assert_eq!(
        fields("Header"),
        [
            "magic", "version", "encoding", "bones", "clips", "words", "table", "pivots"
        ]
    );
    assert_eq!(fields("Entry"), ["first", "frames", "rate", "flags"]);
    // And the reader's are these: a field added to one and not the other stops here.
    let bytes = small(Encoding::Half);
    let library = Library::read(&bytes).unwrap();
    let bake::Header {
        magic,
        version,
        encoding,
        bones,
        clips,
        words,
        table,
        pivots,
    } = library.header();
    assert_eq!(
        [magic, version, encoding, bones, clips, table, pivots],
        [bake::MAGIC, 1, 1, 3, 2, 8, 16]
    );
    let Entry {
        first,
        frames,
        rate,
        flags,
    } = library.entry(0).unwrap();
    assert_eq!((first, frames, flags, words), (25, 4, 1, 25 + 4 * 3 * (4 + 5)));
    assert!((rate - 30.0).abs() < 1e-4);
}
