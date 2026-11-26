//! Chains found on the bones a clip doesn't drive, and swayed: at rest, across frame rates, under violent motion.

#[path = "../benches/common/mod.rs"]
#[allow(dead_code)]
mod common;

use std::f32::consts::TAU;

use dittomo::sway::{self, Chain, Spring};
use dittomo::{Bone, Mapping, Quat, Skeleton, Sway, Transform, Vec3};

/// The skeleton with a run of bones hung from the bone named `under`, each the child of the one before, at these
/// places in the world, y along each bone.
fn hung(sk: &Skeleton, under: &str, name: &str, at: &[[f32; 3]]) -> Skeleton {
    let mut bones = sk.bones().to_vec();
    let mut up = sk.index_of(under).expect("the bone to hang from");
    let world = sk.world(&sk.rest());
    let (mut up_at, mut up_turn) = (world[up].position(), world[up].rotation);
    for (k, &p) in at.iter().enumerate() {
        let p = Vec3::from(p);
        let along = at.get(k + 1).map_or(p - up_at, |&q| Vec3::from(q) - p);
        let turn = Quat::from_rotation_arc(Vec3::Y, along.normalize());
        bones.push(Bone {
            name: format!("{name}{}", k + 1),
            parent: Some(up),
            rest: Transform::new(up_turn.inverse() * (p - up_at), up_turn.inverse() * turn),
        });
        (up, up_at, up_turn) = (bones.len() - 1, p, turn);
    }
    Skeleton::new(bones).expect("parents first")
}

const TAIL: [[f32; 3]; 4] = [
    [0.0, 98.0, -8.0],
    [0.0, 90.0, -16.0],
    [0.0, 80.0, -22.0],
    [0.0, 70.0, -26.0],
];
const PONYTAIL: [[f32; 3]; 3] = [[0.0, 172.0, -8.0], [0.0, 165.0, -12.0], [0.0, 155.0, -14.0]];

/// Mixamo's rig with a tail of four bones under its hips and a ponytail of three under its head.
fn tailed() -> Skeleton {
    let sk = hung(&common::mixamo(), "mixamorig:Hips", "Tail", &TAIL);
    hung(&sk, "mixamorig:Head", "Ponytail", &PONYTAIL)
}

fn named(sk: &Skeleton, chain: &Chain) -> Vec<String> {
    chain.bones.iter().map(|&b| sk.bones()[b].name.clone()).collect()
}

#[test]
fn finds_a_tail_and_a_ponytail_and_nothing_else() {
    let (source, target) = (common::mixamo(), tailed());
    let map = Mapping::new(&source, &target);
    let found = sway::chains(&map, &target);
    let names: Vec<Vec<String>> = found.iter().map(|c| named(&target, c)).collect();
    assert_eq!(
        names,
        [
            vec!["Tail1", "Tail2", "Tail3", "Tail4"],
            vec!["Ponytail1", "Ponytail2", "Ponytail3"]
        ]
    );
    assert_eq!(found[0].parent, target.index_of("mixamorig:Hips").unwrap());
    assert_eq!(found[1].parent, target.index_of("mixamorig:Head").unwrap());
    // Each is as long as its bones, the last as long again as it is far from the one before.
    let along = |at: &[[f32; 3]]| {
        let steps: Vec<f32> = at
            .windows(2)
            .map(|w| Vec3::from(w[0]).distance(Vec3::from(w[1])))
            .collect();
        steps.iter().sum::<f32>() + steps[steps.len() - 1]
    };
    assert!((found[0].length - along(&TAIL)).abs() < 1e-3, "{}", found[0].length);
    assert!((found[1].length - along(&PONYTAIL)).abs() < 1e-3, "{}", found[1].length);
}

#[test]
fn fingers_toes_and_twist_bones_the_source_lacks_are_no_chains() {
    // A source with no fingers, toes' ends or twist bones: Mixamo's rig cut off at the hands and the toes' bases.
    let whole = common::mixamo();
    let cut: Vec<Bone> = whole
        .bones()
        .iter()
        .filter(|b| {
            !["Thumb", "Index", "Middle", "Ring", "Pinky", "Toe_End"]
                .iter()
                .any(|f| b.name.contains(f))
        })
        .cloned()
        .collect();
    let index = |name: &str| cut.iter().position(|b| b.name == name);
    let source = Skeleton::new(
        cut.iter()
            .map(|b| Bone {
                parent: b.parent.and_then(|p| index(&whole.bones()[p].name)),
                ..b.clone()
            })
            .collect(),
    )
    .unwrap();
    for target in [tailed(), common::mannequin()] {
        let map = Mapping::new(&source, &target);
        let free = (0..target.len()).filter(|&b| map.source_of(b).is_none()).count();
        assert!(
            free > 20,
            "the target's fingers have nothing to play: {free} bones free"
        );
        for chain in sway::chains(&map, &target) {
            let names = named(&target, &chain);
            assert!(
                names.iter().all(|n| n.starts_with("Tail") || n.starts_with("Ponytail")),
                "{names:?} is no chain"
            );
        }
    }
}

#[test]
fn hair_that_branches_goes_on_down_its_longest_lock() {
    // A ponytail that splits after its first bone: a long lock of two bones more and a short one of two.
    let sk = hung(&common::mixamo(), "mixamorig:Head", "Ponytail", &PONYTAIL);
    let sk = hung(&sk, "Ponytail1", "Wisp", &[[3.0, 168.0, -11.0], [5.0, 164.0, -12.0]]);
    let map = Mapping::new(&common::mixamo(), &sk);
    let found = sway::chains(&map, &sk);
    let names: Vec<Vec<String>> = found.iter().map(|c| named(&sk, c)).collect();
    assert_eq!(
        names,
        [vec!["Ponytail1", "Ponytail2", "Ponytail3"], vec!["Wisp1", "Wisp2"]]
    );
    assert_eq!(found[1].parent, sk.index_of("Ponytail1").unwrap());
}

#[test]
fn helpers_at_one_spot_and_controls_under_a_root_are_no_chains() {
    let m = common::mannequin();
    let mut bones = m.bones().to_vec();
    let at = |name: &str| m.index_of(name).unwrap();
    let mut add = |name: &str, parent: usize, offset: Vec3| {
        bones.push(Bone {
            name: name.into(),
            parent: Some(parent),
            rest: Transform::new(offset, Quat::IDENTITY),
        });
        bones.len() - 1
    };
    // A helper right at the head with two bones a millimeter long under it, an eye, and a chain of controls under the root.
    let helper = add("helper", at("head"), Vec3::ZERO);
    let marker = add("marker", helper, Vec3::new(0.1, 0.0, 0.0));
    add("marker_end", marker, Vec3::new(0.1, 0.0, 0.0));
    add("eye", at("head"), Vec3::new(3.0, 6.0, 8.0));
    let ik = add("ik_foot_root", at("root"), Vec3::new(0.0, 0.0, 20.0));
    add("ik_foot_l", ik, Vec3::new(10.0, 0.0, 0.0));
    let target = Skeleton::new(bones).unwrap();
    let map = Mapping::new(&common::mixamo(), &target);
    let found: Vec<Vec<String>> = sway::chains(&map, &target).iter().map(|c| named(&target, c)).collect();
    assert!(found.is_empty(), "{found:?}");
}

/// Prints the chains found on VRoid rigs played from a Mixamo rig, scored against the rigs' own `J_Sec_` names.
/// The names only score: nothing finds a chain by them. Run with
/// `cargo test --features fs --test sway -- --ignored --nocapture`.
#[cfg(feature = "fs")]
#[test]
#[ignore = "prints chains for a person to look over; needs bench/rigs/data"]
fn prints_the_chains_of_vroid_rigs() {
    let dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("bench/rigs/data");
    let Ok(source) = dittomo::load::Model::load(&dir.join("xbot.glb")) else {
        println!("no rigs in {}", dir.display());
        return;
    };
    for file in ["seed-san.vrm", "vrm-twist.vrm"] {
        let Ok(target) = dittomo::load::Model::load(&dir.join(file)) else {
            println!("{file}: not there");
            continue;
        };
        let sk = &target.skeleton;
        let map = Mapping::new(&source.skeleton, sk);
        let found = sway::chains(&map, sk);
        let mut chained = vec![false; sk.len()];
        println!("{file}: {} bones, {} chains", sk.len(), found.len());
        for chain in &found {
            for &b in &chain.bones {
                chained[b] = true;
            }
            println!(
                "  {:8.4} under {:24} {}",
                chain.length,
                sk.bones()[chain.parent].name,
                named(sk, chain).join(" ")
            );
        }
        let sec = |b: usize| sk.bones()[b].name.contains("J_Sec_");
        let (all, both) = (
            (0..sk.len()).filter(|&b| sec(b)).count(),
            (0..sk.len()).filter(|&b| sec(b) && chained[b]).count(),
        );
        let any = chained.iter().filter(|&&c| c).count();
        if all == 0 {
            println!("  no J_Sec_ bones to score by");
            continue;
        }
        println!(
            "  {both} of {any} chained bones are J_Sec_ (precision), {both} of {all} J_Sec_ bones chained (recall)"
        );
        for b in 0..sk.len() {
            let name = &sk.bones()[b].name;
            if sec(b) != chained[b] {
                let how = if chained[b] {
                    "chained, not J_Sec_"
                } else {
                    "J_Sec_, not chained"
                };
                let driven = map.source_of(b).map(|a| source.skeleton.bones()[a].name.as_str());
                println!("    {how}: {name} (plays {driven:?})");
            }
        }
    }
}

/// The tailed rig and its sway, played from the plain rig.
fn swayed() -> (Skeleton, Sway) {
    let target = tailed();
    let map = Mapping::new(&common::mixamo(), &target);
    let sway = Sway::new(&map, &target).unwrap();
    assert_eq!(sway.chains().len(), 2);
    (target, sway)
}

/// The rig's pose `t` seconds into a dance in place: the hips swing a tenth of a meter to each side, as often as the
/// tail would swing by itself, and bob, the body turns, the chest bows and the head shakes. `wild` makes it a fit:
/// meters of travel, spins and whipping about, several times a second.
fn danced(sk: &Skeleton, t: f32, wild: bool) -> Vec<Transform> {
    let mut pose = sk.rest();
    let at = |name: &str| sk.index_of(name).unwrap();
    let (hips, chest, head) = (at("mixamorig:Hips"), at("mixamorig:Spine1"), at("mixamorig:Head"));
    if wild {
        pose[hips].translation += Vec3::new(200.0 * (TAU * 3.0 * t).sin(), 80.0 * (TAU * 4.3 * t).sin(), 150.0 * t);
        pose[hips].rotation *= Quat::from_rotation_y(15.0 * t) * Quat::from_rotation_x(2.0 * (9.0 * t).sin());
        pose[chest].rotation *= Quat::from_rotation_z(1.5 * (23.0 * t).sin());
        pose[head].rotation *= Quat::from_rotation_x(1.2 * (31.0 * t).cos());
    } else {
        pose[hips].translation += Vec3::new(10.0 * (TAU * 0.8 * t).sin(), 1.5 * (TAU * 1.6 * t).sin(), 0.0);
        pose[hips].rotation *= Quat::from_rotation_y(0.25 * (TAU * 0.5 * t).sin());
        pose[chest].rotation *= Quat::from_rotation_x(0.15 * (TAU * 0.7 * t).sin());
        pose[head].rotation *= Quat::from_rotation_y(0.3 * (TAU * 0.9 * t).sin());
    }
    pose
}

/// Where a chain's tip is in the world: the end of its last bone, as far past it as that bone is from the one before.
fn tip(sk: &Skeleton, chain: &Chain, pose: &[Transform]) -> Vec3 {
    let last = chain.bones[chain.bones.len() - 1];
    let reach = sk.bones()[last].rest.translation.length();
    let world = sk.world(pose);
    world[last].position() + world[last].rotation * (reach * Vec3::Y)
}

/// The angle between two directions, told apart even when it is small.
fn angle(a: Vec3, b: Vec3) -> f32 {
    a.cross(b).length().atan2(a.dot(b))
}

/// The angle between two turns, told apart even when it is small.
fn apart(a: Quat, b: Quat) -> f32 {
    let d = a.inverse() * b;
    2.0 * d.xyz().length().atan2(d.w.abs())
}

/// How far each chain bone is bent from its rest pose under its parent, in radians: the most of any.
fn bend(sk: &Skeleton, sway: &Sway, pose: &[Transform]) -> f32 {
    sway.chains()
        .iter()
        .flat_map(|c| &c.bones)
        .map(|&b| angle(pose[b].rotation * Vec3::Y, sk.bones()[b].rest.rotation * Vec3::Y))
        .fold(0.0, f32::max)
}

/// Steps of about a sixtieth of a second, some half that and some twice, that add up to each sixth of a second.
fn jittered(seconds: f32) -> Vec<f32> {
    let mut seed = 0x2545_f491u32;
    let mut out = Vec::new();
    for _ in 0..(seconds * 6.0).round() as usize {
        let mut left = 1.0f32 / 6.0;
        while left > 1e-6 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let dt = (1.0 / 120.0 + (seed >> 8) as f32 / (1u32 << 24) as f32 * (1.0 / 30.0 - 1.0 / 120.0)).min(left);
            out.push(dt);
            left -= dt;
        }
    }
    out
}

/// Plays the dance through these steps, and gives each chain's tip at every sixth of a second.
fn tips(dts: &[f32], wild: bool) -> Vec<[Vec3; 2]> {
    let (sk, mut sway) = swayed();
    let (mut t, mut next, mut out) = (0.0f32, 1.0f32 / 6.0, Vec::new());
    let mut pose = danced(&sk, 0.0, wild);
    sway.pose(0.0, &mut pose);
    for &dt in dts {
        t += dt;
        pose = danced(&sk, t, wild);
        sway.pose(dt, &mut pose);
        if t > next - 1e-4 {
            out.push([0, 1].map(|c| tip(&sk, &sway.chains()[c], &pose)));
            next += 1.0 / 6.0;
        }
    }
    out
}

#[test]
fn a_body_at_rest_keeps_its_chains_at_rest() {
    let (sk, mut sway) = swayed();
    let mut worst = 0.0f32;
    for _ in 0..300 {
        let mut pose = sk.rest();
        sway.pose(1.0 / 60.0, &mut pose);
        for chain in sway.chains() {
            for &b in &chain.bones {
                worst = worst.max(apart(pose[b].rotation, sk.bones()[b].rest.rotation));
            }
        }
    }
    println!("rest: {worst:e} radians off at most");
    assert!(worst < 1e-4, "{worst} radians off rest");
}

#[test]
fn the_same_dance_sways_the_same_at_any_frame_rate() {
    let seconds = 4.0;
    let even = |hz: f32| vec![1.0 / hz; (seconds * hz).round() as usize];
    let runs = [even(30.0), even(60.0), even(144.0), jittered(seconds)].map(|dts| tips(&dts, false));
    let (sk, sway) = swayed();
    let lengths = [0, 1].map(|c| sway.chains()[c].length);
    // The chains do move: their tips stray from where rest would have them.
    let stiff: Vec<[Vec3; 2]> = (1..=runs[1].len())
        .map(|k| {
            let pose = danced(&sk, k as f32 / 6.0, false);
            [0, 1].map(|c| tip(&sk, &sway.chains()[c], &pose))
        })
        .collect();
    let mut swung = [0.0f32; 2];
    let mut worst = [0.0f32; 2];
    for run in &runs {
        assert_eq!(run.len(), runs[1].len());
        for (k, (a, b)) in run.iter().zip(&runs[1]).enumerate() {
            for c in 0..2 {
                worst[c] = worst[c].max(a[c].distance(b[c]) / lengths[c]);
                swung[c] = swung[c].max(b[c].distance(stiff[k][c]) / lengths[c]);
            }
        }
    }
    println!("frame rates: tips {worst:?} of their chains apart at most, swung {swung:?} off rest");
    assert!(swung.iter().all(|&s| s > 0.1), "the chains hardly sway: {swung:?}");
    assert!(worst.iter().all(|&w| w < 0.02), "tips {worst:?} of their chains apart");
}

#[test]
fn a_fit_at_any_step_never_breaks_a_chain() {
    let (sk, sway) = swayed();
    let rest_world = sk.world(&sk.rest());
    let limit = sway.spring(0).limit;
    let mut most = 0.0f32;
    for dts in [
        vec![0.5; 20],
        vec![1.0 / 60.0; 600],
        jittered(10.0),
        vec![1.0 / 7.0; 70],
    ] {
        let (_, mut sway) = swayed();
        let mut t = 0.0;
        for &dt in &dts {
            t += dt;
            let mut pose = danced(&sk, t, true);
            let played = pose.clone();
            sway.pose(dt, &mut pose);
            let world = sk.world(&pose);
            for (b, (now, was)) in pose.iter().zip(&played).enumerate() {
                assert!(
                    now.rotation.is_finite() && now.rotation.is_normalized(),
                    "bone {b} at {t}"
                );
                assert_eq!((now.translation, now.scale), (was.translation, was.scale));
                let chained = sway.chains().iter().any(|c| c.bones.contains(&b));
                assert!(chained || now.rotation == was.rotation, "bone {b} is in no chain");
                if let Some(p) = sk.bones()[b].parent {
                    let (long, rest) = (
                        world[b].position().distance(world[p].position()),
                        rest_world[b].position().distance(rest_world[p].position()),
                    );
                    assert!(
                        (long - rest).abs() <= 1e-4 * rest.max(1.0),
                        "bone {b}: {long} long, {rest} at rest"
                    );
                }
            }
            let bent = bend(&sk, &sway, &pose);
            most = most.max(bent);
            assert!(bent <= limit + 1e-3, "bent {bent} radians at {t}");
        }
    }
    println!("fit: bent {most} radians at most, of {limit} allowed");
    assert!(most > 0.9 * limit, "the fit never reached the limit: {most}");
}

#[test]
fn two_runs_sway_the_same_to_the_bit() {
    let run = || {
        let (sk, mut sway) = swayed();
        let mut t = 0.0;
        let mut bits = Vec::new();
        for dt in jittered(3.0) {
            t += dt;
            let mut pose = danced(&sk, t, t > 2.0);
            sway.pose(dt, &mut pose);
            bits.extend(pose.iter().flat_map(|p| p.rotation.to_array().map(f32::to_bits)));
        }
        bits
    };
    assert_eq!(run(), run());
}

#[test]
fn a_tail_lags_a_body_shoved_sideways_then_comes_back_to_rest() {
    let (sk, mut sway) = swayed();
    let chains = sway.chains().to_vec();
    let posed = |t: f32| {
        let mut pose = sk.rest();
        // Half a meter to the side, in a tenth of a second.
        let s = ((t - 1.0) / 0.1).clamp(0.0, 1.0);
        pose[0].translation.x += 50.0 * s * s * (3.0 - 2.0 * s);
        pose
    };
    // Where each tip is from where rest would have it, over its chain's length.
    let mut lag = |t: f32| {
        let (mut pose, stiff) = (posed(t), posed(t));
        sway.pose(1.0 / 60.0, &mut pose);
        [0, 1].map(|c| (tip(&sk, &chains[c], &pose) - tip(&sk, &chains[c], &stiff)) / chains[c].length)
    };
    let (mut most, mut last, mut settled) = ([0.0f32; 2], [0.0f32; 2], [0.0f32; 2]);
    for f in 0..=300 {
        let t = f as f32 / 60.0;
        let off = lag(t);
        for c in 0..2 {
            last[c] = off[c].length();
            most[c] = most[c].max(last[c]);
            if last[c] > 0.01 {
                settled[c] = t - 1.1;
            }
            assert!(
                t > 1.0 || last[c] < 1e-4,
                "chain {c} moved before the shove: {}",
                last[c]
            );
            // Mid-shove, the tips trail the body: they are off toward where it came from.
            assert!(f != 64 || off[c].x < -0.02, "chain {c} doesn't trail: {:?}", off[c]);
        }
    }
    println!(
        "shove: tips lagged {most:?} of their chains at most, within a hundredth {settled:?} s after, {last:?} at the end"
    );
    assert!(
        settled.iter().all(|&s| s < 3.0),
        "still swinging {settled:?} seconds after the shove"
    );
    assert!(most.iter().all(|&m| m > 0.1), "the chains hardly lagged: {most:?}");
    assert!(last.iter().all(|&l| l < 1e-3), "not back at rest: {last:?}");
}

#[test]
fn reset_puts_the_chains_back_at_rest() {
    let (sk, mut sway) = swayed();
    let mut pose = sk.rest();
    for f in 0..90 {
        pose = danced(&sk, f as f32 / 60.0, true);
        sway.pose(1.0 / 60.0, &mut pose);
    }
    assert!(bend(&sk, &sway, &pose) > 0.1, "the fit bent nothing");
    sway.reset();
    let mut pose = danced(&sk, 1.5, true);
    sway.pose(1.0 / 60.0, &mut pose);
    for chain in sway.chains() {
        for &b in &chain.bones {
            assert_eq!(pose[b].rotation, sk.bones()[b].rest.rotation.normalize(), "bone {b}");
        }
    }
    // And they start from there, still: reset again with the body upright somewhere else and then held, they stay.
    sway.reset();
    for _ in 0..60 {
        pose = sk.rest();
        pose[0].translation += Vec3::new(300.0, 0.0, -200.0);
        sway.pose(1.0 / 60.0, &mut pose);
    }
    let bent = bend(&sk, &sway, &pose);
    assert!(bent < 1e-4, "{bent}");
}

#[test]
fn a_spring_set_stiffer_swings_less() {
    let run = |stiffness: [f32; 2]| {
        let (sk, mut sway) = swayed();
        for c in 0..2 {
            let spring = Spring {
                stiffness,
                ..sway.spring(c)
            };
            sway.set_spring(c, spring);
            assert_eq!(sway.spring(c), spring);
        }
        let mut most = 0.0f32;
        for f in 0..240 {
            let mut pose = danced(&sk, f as f32 / 60.0, false);
            sway.pose(1.0 / 60.0, &mut pose);
            most = most.max(bend(&sk, &sway, &pose));
        }
        most
    };
    let (loose, stiff) = (run([1.0, 3.0]), run([4.0, 12.0]));
    println!("stiffness: bent {loose} radians as set, {stiff} four times as stiff");
    assert!(stiff < 0.5 * loose, "{stiff} against {loose}");
}

#[test]
fn chains_given_by_hand_sway_with_a_spring_for_their_units() {
    // The same rig in meters, its chains named by hand, sways the same angles as the one in centimeters.
    let (cm, mut by_map) = swayed();
    let m = Skeleton::new(
        cm.bones()
            .iter()
            .map(|b| Bone {
                rest: Transform::new(b.rest.translation / 100.0, b.rest.rotation),
                ..b.clone()
            })
            .collect(),
    )
    .unwrap();
    let chains: Vec<Chain> = by_map
        .chains()
        .iter()
        .rev()
        .map(|c| Chain {
            length: 0.0,
            ..c.clone()
        })
        .collect();
    let mut by_hand = Sway::with(&m, &chains, Spring::per_meter(1.0));
    assert!((Spring::of(&cm).gravity / (9.81 * 180.0 / 1.7) - 1.0).abs() < 1e-3);
    for (a, b) in by_hand.chains().iter().zip(by_map.chains()) {
        assert_eq!(a.bones, b.bones);
        assert!((a.length * 100.0 - b.length).abs() < 1e-2);
    }
    // The rig stands 1.8 m by its bones, so guessed at 1.7 m its gravity is that much stronger: matched here.
    for c in 0..2 {
        let spring = Spring {
            gravity: 981.0,
            ..by_map.spring(c)
        };
        by_map.set_spring(c, spring);
    }
    let mut off = 0.0f32;
    for f in 0..240 {
        let mut pose = danced(&cm, f as f32 / 60.0, false);
        let mut small = pose.clone();
        for p in &mut small {
            p.translation /= 100.0;
        }
        by_map.pose(1.0 / 60.0, &mut pose);
        by_hand.pose(1.0 / 60.0, &mut small);
        for chain in by_map.chains() {
            for &b in &chain.bones {
                off = off.max(apart(pose[b].rotation, small[b].rotation));
            }
        }
    }
    println!("units: {off} radians apart at most");
    assert!(off < 1e-2, "{off} radians apart");
}
