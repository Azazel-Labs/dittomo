//! Chains side by side, tied: found on a made-up skirt, a cape and pigtails, held at rest, kept out of the legs that
//! swing through them, and measured on real rigs.

use std::f32::consts::TAU;

use super::tests::{person, rig};
use super::*;

/// The person with a skirt of eight chains of three bones round the hips, flaring a little, and those as chains
/// after the ponytail. Its two thighs.
fn skirted() -> (Skeleton, Vec<Chain>, [usize; 2]) {
    let (person, mut chains) = person();
    // The tail hangs inside the skirt, and would be taken for one of its folds: it is left stiff here.
    chains.remove(0);
    let mut bones: Vec<(Option<usize>, [f32; 3])> = person
        .bones()
        .iter()
        .map(|b| (b.parent, b.rest.translation.to_array()))
        .collect();
    let thigh = |side: f32| {
        (0..bones.len())
            .find(|&b| bones[b].0 == Some(0) && bones[b].1[0] * side > 0.05 && bones[b].1[1] < 0.0)
            .unwrap()
    };
    let thighs = [thigh(1.0), thigh(-1.0)];
    for fold in 0..8 {
        let (sin, cos) = ((fold as f32 + 0.5) * TAU / 8.0).sin_cos();
        let at = bones.len();
        bones.push((Some(0), [0.15 * sin, -0.02, 0.15 * cos]));
        bones.push((Some(at), [0.02 * sin, -0.13, 0.02 * cos]));
        bones.push((Some(at + 1), [0.02 * sin, -0.13, 0.02 * cos]));
        chains.push(Chain {
            bones: vec![at, at + 1, at + 2],
            parent: 0,
            length: 0.0,
        });
    }
    (rig(&bones), chains, thighs)
}

/// The skirted person `t` seconds into a walk in place, or a run: each thigh swung `swing` radians to the front and
/// half that back, the knees bent as the thigh comes forward, the hips bobbing and turning with the stride.
fn strode(sk: &Skeleton, thighs: [usize; 2], t: f32, swing: f32, strides: f32) -> Vec<Transform> {
    let mut pose = sk.rest();
    let turn = TAU * strides * t;
    for (side, &thigh) in thighs.iter().enumerate() {
        let phase = (turn + side as f32 * TAU / 2.0).sin();
        pose[thigh].rotation = Quat::from_rotation_x(-swing * (0.25 + 0.75 * phase));
        pose[thigh + 1].rotation = Quat::from_rotation_x(0.8 * swing * (0.5 + 0.5 * phase));
    }
    pose[0].translation.y += 0.03 * (2.0 * turn).cos();
    pose[0].rotation = Quat::from_rotation_y(0.15 * turn.sin());
    pose
}

/// What a run of poses did to the chains side by side: of the tails of chains with a neighbor, and of the points
/// half way between two tied tails, how many of each were in a leg (nearer its line than it is thick, and than
/// at rest, by a thousandth of the height); and of the tied pairs, how many were nearer than a quarter of how far
/// apart they are at rest, or further than half as far again.
#[derive(Default, Debug)]
struct Tally {
    tails: [usize; 2],
    mids: [usize; 2],
    pairs: [usize; 3],
}

impl Tally {
    /// Counts the pose the sway has just taken. `legs` says which rods are a leg's.
    fn count(&mut self, sway: &Sway, legs: &[bool]) {
        let (rods, room) = (sway.rods.len(), 1e-3 * sway.height);
        let inside = |p: Vec3, apart: &[f32]| {
            (0..rods).any(|k| {
                let [a, b] = sway.was[k];
                legs[k] && p.distance(nearest(a, b, p)) < sway.rods[k].radius.min(apart[k]) - room
            })
        };
        for i in 0..sway.sides.len() {
            if sway.spans[i] == [0.0; 2] {
                continue;
            }
            self.tails[0] += usize::from(inside(sway.at[i], &sway.apart[i * rods..(i + 1) * rods]));
            self.tails[1] += 1;
            for s in 0..2 {
                let (to, span) = (sway.sides[i][s] as usize, sway.spans[i][s]);
                // Each pair once.
                if span == 0.0 || to < i && sway.sides[to].contains(&(i as u32)) {
                    continue;
                }
                let at = (2 * i + s) * rods;
                let mid = (sway.at[i] + sway.at[to]) / 2.0;
                self.mids[0] += usize::from(inside(mid, &sway.mid_apart[at..at + rods]));
                self.mids[1] += 1;
                let far = sway.at[i].distance(sway.at[to]);
                self.pairs[0] += usize::from(far < 0.25 * span);
                self.pairs[1] += usize::from(far > 1.5 * span);
                self.pairs[2] += 1;
            }
        }
    }

    fn shares(&self) -> [f32; 4] {
        let share = |a: usize, of: usize| 100.0 * a as f32 / of.max(1) as f32;
        [
            share(self.tails[0], self.tails[1]),
            share(self.mids[0], self.mids[1]),
            share(self.pairs[0], self.pairs[2]),
            share(self.pairs[1], self.pairs[2]),
        ]
    }
}

/// Which of the sway's rods are along a leg.
fn leg_rods(sk: &Skeleton, sway: &Sway) -> Vec<bool> {
    let legs = contact::legs(sk);
    let of_leg = |b: usize| legs.iter().any(|l| [l.upper, l.lower, l.end].contains(&b));
    sway.rods
        .iter()
        .map(|r| r.ends[0].0 != r.ends[1].0 && r.ends.iter().all(|&(b, _)| of_leg(b)))
        .collect()
}

#[test]
fn a_skirt_is_tied_round_and_a_tail_to_nothing() {
    let (sk, chains, _) = skirted();
    let sway = Sway::with(&sk, &chains, Spring::per_meter(1.0));
    // The ponytail hangs alone; each fold of the skirt is tied to the one before and the one after.
    assert_eq!(sway.beside()[0], [None; 2]);
    for fold in 0..8 {
        let mut beside: Vec<usize> = sway.beside()[1 + fold].iter().map(|b| b.unwrap() - 1).collect();
        beside.sort_unstable();
        let mut round = vec![(fold + 1) % 8, (fold + 7) % 8];
        round.sort_unstable();
        assert_eq!(beside, round, "fold {fold}");
    }
    // Each tail to the tail at its own height in the next fold.
    for i in sway.starts[1]..sway.bones.len() {
        for s in 0..2 {
            let to = sway.sides[i][s] as usize;
            assert_eq!((to - sway.starts[1]) % 3, (i - sway.starts[1]) % 3, "tail {i} to {to}");
            assert!(sway.spans[i][s] > 0.1 && sway.spans[i][s] < 0.17);
        }
    }
    assert!(sway.spans[..sway.starts[1]].iter().all(|s| *s == [0.0; 2]));
}

#[test]
fn a_cape_is_tied_in_a_row_and_pigtails_and_unlike_chains_are_not() {
    // Under a post: a cape of three strips of three bones, 8 cm apart; two pigtails of two bones, 16 cm apart and
    // 20 cm long; and beside the cape's last strip, as near, one a third as long.
    let mut bones = vec![(None, [0.0, 1.5, 0.0]), (Some(0), [0.0, -1.5, 0.0])];
    let mut chains = Vec::new();
    let mut hang = |x: f32, z: f32, count: usize, long: f32| {
        let at = bones.len();
        bones.push((Some(0), [x, 0.0, z]));
        for k in 1..count {
            bones.push((Some(at + k - 1), [0.0, -long, 0.0]));
        }
        chains.push(Chain {
            bones: (at..at + count).collect(),
            parent: 0,
            length: 0.0,
        });
    };
    for strip in 0..3 {
        hang(0.08 * (strip as f32 - 1.0), -0.1, 4, 0.15);
    }
    hang(-0.08, 0.1, 2, 0.1);
    hang(0.08, 0.1, 2, 0.1);
    hang(0.16, -0.1, 3, 0.07);
    let sk = rig(&bones);
    let sway = Sway::with(&sk, &chains, Spring::per_meter(1.0));
    let beside: Vec<Vec<usize>> = sway
        .beside()
        .iter()
        .map(|b| b.iter().flatten().copied().collect())
        .collect();
    assert_eq!(beside[0], [1]);
    assert!(beside[1] == [0, 2] || beside[1] == [2, 0], "{:?}", beside[1]);
    assert_eq!(beside[2], [1]);
    assert!(beside[3..].iter().all(Vec::is_empty), "{beside:?}");
}

#[test]
fn a_skirt_at_rest_stays_at_rest_and_sways_the_same_twice() {
    let (sk, chains, thighs) = skirted();
    let mut sway = Sway::with(&sk, &chains, Spring::per_meter(1.0));
    for _ in 0..120 {
        let mut pose = sk.rest();
        sway.pose(1.0 / 60.0, &mut pose);
        for (i, &b) in sway.bones.iter().enumerate() {
            let off = pose[b].rotation.angle_between(sway.rest[i]);
            assert!(off < 1e-4, "bone {b} is {off} radians off rest");
        }
    }
    let run = || {
        let mut sway = Sway::with(&sk, &chains, Spring::per_meter(1.0));
        let mut bits = Vec::new();
        for f in 0..120 {
            let mut pose = strode(&sk, thighs, f as f32 / 30.0, 1.2, 1.5);
            sway.pose(1.0 / 30.0, &mut pose);
            bits.extend(pose.iter().flat_map(|p| p.rotation.to_array().map(f32::to_bits)));
        }
        bits
    };
    assert!(run() == run(), "two runs differ");
}

#[test]
fn a_skirt_tied_keeps_its_folds_in_order_and_the_legs_in() {
    // A walk and a run in place, thighs swung 0.7 and 1.3 radians forward, at 30 and 60 poses a second: untied,
    // then tied.
    let (sk, chains, thighs) = skirted();
    for (swing, strides) in [(0.7f32, 1.0f32), (1.3, 1.6)] {
        for rate in [30.0f32, 60.0] {
            let run = |tied: bool| {
                let mut sway = Sway::with(&sk, &chains, Spring::per_meter(1.0));
                sway.tie(tied);
                let legs = leg_rods(&sk, &sway);
                assert_eq!(legs.iter().filter(|&&l| l).count(), 4);
                let (mut tally, mut bent) = (Tally::default(), 0.0f32);
                for f in 0..(4.0 * rate) as usize {
                    let mut pose = strode(&sk, thighs, f as f32 / rate, swing, strides);
                    sway.pose(1.0 / rate, &mut pose);
                    tally.count(&sway, &legs);
                    for (i, &b) in sway.bones.iter().enumerate() {
                        let now = pose[b].rotation;
                        assert!(now.is_finite() && (now.length() - 1.0).abs() < 1e-5);
                        bent = bent.max(now.angle_between(sway.rest[i]));
                    }
                }
                (tally.shares(), bent)
            };
            let ((loose, loose_bent), (tied, tied_bent)) = (run(false), run(true));
            println!(
                "skirt, thighs swung {swing} at {rate} poses a second, untied then tied: tails in a leg {:.2}% {:.2}%, \
                 half way between two {:.2}% {:.2}%, pairs under a quarter apart {:.2}% {:.2}%, over half as far \
                 again {:.2}% {:.2}%, bent {loose_bent:.2} {tied_bent:.2} at most",
                loose[0], tied[0], loose[1], tied[1], loose[2], tied[2], loose[3], tied[3]
            );
            // Untied, a thigh comes out between two folds; tied it lifts the cloth between them.
            assert!(loose[1] > 2.0, "no thigh came between two folds: {}%", loose[1]);
            assert!(tied[1] < 0.25 * loose[1], "half way points in a leg: {}%", tied[1]);
            assert!(tied[0] <= loose[0] + 0.1, "tails in a leg: {}%", tied[0]);
            assert!(
                tied[2] < 0.5 && tied[3] < 8.0,
                "pairs too near {}%, too far {}%",
                tied[2],
                tied[3]
            );
        }
    }
}

/// Prints how often the chains side by side on real rigs are in the legs and how their pairs keep, untied and
/// tied, played from real clips. Run with
/// `cargo test --features fs --lib sway::tied -- --ignored --nocapture`.
#[cfg(feature = "fs")]
#[test]
#[ignore = "prints numbers for a person to look over; needs bench/rigs/data"]
fn prints_how_tied_chains_keep_on_real_rigs() {
    use crate::Fixes;
    use crate::load::Model;
    let dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("bench/rigs/data");
    for file in ["seed-san.vrm", "vrm-twist.vrm"] {
        let Ok(target) = Model::load(&dir.join(file)) else {
            println!("{file}: not there");
            continue;
        };
        let sk = &target.skeleton;
        for from in ["xbot.glb", "michelle.glb", "soldier.glb"] {
            let Ok(source) = Model::load(&dir.join(from)) else {
                continue;
            };
            let map = Mapping::new(&source.skeleton, sk);
            let sway = Sway::new(&map, sk).unwrap();
            if from == "xbot.glb" {
                let tied = sway.beside().iter().filter(|b| **b != [None; 2]).count();
                println!("{file}: {tied} of {} chains beside another", sway.chains().len());
                for (c, beside) in sway.beside().iter().enumerate() {
                    let name = |c: usize| sk.bones()[sway.chains()[c].bones[0]].name.as_str();
                    let others: Vec<&str> = beside.iter().flatten().map(|&d| name(d)).collect();
                    if !others.is_empty() {
                        println!("  {} beside {}", name(c), others.join(" and "));
                    }
                }
            }
            let legs = leg_rods(sk, &sway);
            for (name, clip) in &source.clips {
                if clip.poses.len() < 10 {
                    continue;
                }
                let played = map.apply_with(&source.skeleton, sk, clip, Fixes::ALL).unwrap();
                let run = |tied: bool| {
                    let mut sway = sway.clone();
                    sway.tie(tied);
                    let mut tally = Tally::default();
                    // Twice through, the second counted.
                    for round in 0..2 {
                        for (f, pose) in played.poses.iter().enumerate() {
                            let dt = if f == 0 {
                                0.0
                            } else {
                                played.times[f] - played.times[f - 1]
                            };
                            let mut pose = pose.clone();
                            if f == 0 && round == 1 {
                                sway.jump(&mut pose);
                            } else {
                                sway.pose(dt, &mut pose);
                            }
                            if round == 1 {
                                tally.count(&sway, &legs);
                            }
                        }
                    }
                    tally.shares()
                };
                let (loose, tied) = (run(false), run(true));
                println!(
                    "  {from} {:?}, untied then tied: tails in a leg {:.2}% {:.2}%, half way between two {:.2}% \
                     {:.2}%, pairs under a quarter apart {:.2}% {:.2}%, over half as far again {:.2}% {:.2}%",
                    name.as_deref().unwrap_or("clip"),
                    loose[0],
                    tied[0],
                    loose[1],
                    tied[1],
                    loose[2],
                    tied[2],
                    loose[3],
                    tied[3]
                );
            }
        }
    }
}
