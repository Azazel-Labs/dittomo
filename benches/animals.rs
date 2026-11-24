//! How well mapping and playing hold up between four-legged animals of different builds: a fox, dogs, deer, horses, a
//! cow and a long-necked alpaca.
//!
//! ```text
//! sh bench/animals/fetch.sh                        # download the animals (once)
//! cargo bench --features fs --bench animals        # print the table
//! cargo bench --features fs --bench animals -- --pairs   # print each pair's misses
//! cargo bench --features fs --bench animals -- --min 100 --max-posture 1 --max-floor 13   # fail past these (CI)
//! ```
//!
//! `bench/animals/labels.txt` names, for each rig, the bones playing 17 joints of a four-legged body. Each ordered pair
//! of animals is mapped, and a joint is found when the target's bone plays the source's. Then:
//!
//! - posture: the source standing at rest played on the target, how far the target's hips, chest and head turn from
//!   its own rest, in degrees. Each animal carries its body as it is built (an alpaca its neck upright, a fox its head
//!   low), so standing as the source stands is standing as it does itself.
//! - floor: every clip of the source played on the target with every fix, how much further the target's feet sink
//!   below its floor at worst than the source's own do below theirs, over the height, in percent.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

use dittomo::load::Model;
use dittomo::{Clip, Fixes, Mapping};

const JOINTS: [&str; 17] = [
    "hips",
    "chest",
    "head",
    "l_hip",
    "l_knee",
    "l_hock",
    "l_hindfoot",
    "r_hip",
    "r_knee",
    "r_hock",
    "r_hindfoot",
    "l_shoulder",
    "l_elbow",
    "l_forefoot",
    "r_shoulder",
    "r_elbow",
    "r_forefoot",
];
const AXIS: [&str; 3] = ["hips", "chest", "head"];
const FEET: [&str; 4] = ["l_hindfoot", "r_hindfoot", "l_forefoot", "r_forefoot"];

/// The most frames of a clip measured; longer clips are sampled evenly.
const FRAMES: usize = 40;

struct Animal {
    name: String,
    model: Model,
    /// Each labeled joint's bones.
    labels: BTreeMap<String, Vec<usize>>,
}

fn dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("bench/animals")
}

/// The labels file: `[rig files]` then `joint = bone | bone` lines; `#` starts a comment.
fn labels(path: &Path) -> BTreeMap<String, BTreeMap<String, Vec<String>>> {
    let mut out: BTreeMap<String, BTreeMap<String, Vec<String>>> = BTreeMap::new();
    let text = std::fs::read_to_string(path).expect("bench/animals/labels.txt");
    let mut rigs: Vec<String> = Vec::new();
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        if let Some(names) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            rigs = names.split_whitespace().map(str::to_owned).collect();
        } else if let Some((joint, bones)) = line.split_once('=') {
            let bones: Vec<String> = bones.split('|').map(|b| b.trim().to_owned()).collect();
            for rig in &rigs {
                out.entry(rig.clone())
                    .or_default()
                    .insert(joint.trim().to_owned(), bones.clone());
            }
        }
    }
    out
}

fn load() -> Vec<Animal> {
    let labeled = labels(&dir().join("labels.txt"));
    let mut out = Vec::new();
    for (file, joints) in labeled {
        let path = dir().join("data").join(&file);
        let model = match Model::load(&path) {
            Ok(m) => m,
            Err(e) => {
                eprintln!("{file}: {e} (sh bench/animals/fetch.sh)");
                continue;
            }
        };
        let mut labels = BTreeMap::new();
        for (joint, names) in joints {
            let bones: Vec<usize> = names.iter().filter_map(|n| model.skeleton.index_of(n)).collect();
            assert!(!bones.is_empty(), "{file}: no bone {names:?} for {joint}");
            labels.insert(joint, bones);
        }
        let name = file.split('.').next().unwrap_or(&file).to_owned();
        out.push(Animal { name, model, labels });
    }
    out
}

/// At most [`FRAMES`] of a clip, evenly spread.
fn sampled(clip: &Clip) -> Clip {
    let n = clip.poses.len();
    let step = n.div_ceil(FRAMES).max(1);
    Clip {
        times: clip.times.iter().step_by(step).copied().collect(),
        poses: clip.poses.iter().step_by(step).cloned().collect(),
    }
}

/// How far the animal's feet sink below its floor (where they stand at rest) at worst over a clip, over its height,
/// in percent.
fn sunk(animal: &Animal, clip: &Clip) -> f32 {
    let skeleton = &animal.model.skeleton;
    let rest = skeleton.world(&skeleton.rest());
    let ys = || rest.iter().map(|w| w.position().y);
    let height = ys().fold(f32::MIN, f32::max) - ys().fold(f32::MAX, f32::min);
    let feet: Vec<usize> = FEET
        .iter()
        .filter_map(|j| animal.labels.get(*j))
        .flatten()
        .copied()
        .collect();
    let floor = feet.iter().map(|&b| rest[b].position().y).fold(f32::MAX, f32::min);
    let mut sink = 0.0f32;
    for pose in &clip.poses {
        let w = skeleton.world(pose);
        for &b in &feet {
            sink = sink.max((floor - w[b].position().y) / height * 100.0);
        }
    }
    sink
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let has = |f: &str| args.iter().any(|a| a == f);
    let value = |f: &str| {
        args.iter()
            .position(|a| a == f)
            .and_then(|i| args.get(i + 1))
            .and_then(|v| v.parse::<f32>().ok())
    };
    let animals = load();
    if animals.len() < 2 {
        eprintln!("fewer than two animals: sh bench/animals/fetch.sh");
        std::process::exit(if value("--min").is_some() { 1 } else { 0 });
    }
    let started = Instant::now();
    let own: BTreeMap<String, f32> = animals
        .iter()
        .map(|a| {
            let worst = a
                .model
                .clips
                .iter()
                .map(|(_, c)| sunk(a, &sampled(c)))
                .fold(0.0, f32::max);
            (a.name.clone(), worst)
        })
        .collect();
    let (mut found, mut labeled) = (0usize, 0usize);
    let (mut posture_sum, mut posture_n, mut posture_worst) = (0.0f32, 0usize, (0.0f32, String::new()));
    let mut floor_worst = (0.0f32, String::new());
    let mut floor_sum = (0.0f32, 0usize);
    let mut rows = Vec::new();
    for s in &animals {
        for t in &animals {
            if std::ptr::eq(s, t) {
                continue;
            }
            let (ss, ts) = (&s.model.skeleton, &t.model.skeleton);
            let map = Mapping::new(ss, ts);
            let mut hits = 0;
            let mut misses = Vec::new();
            for joint in JOINTS {
                let (Some(want), Some(bones)) = (s.labels.get(joint), t.labels.get(joint)) else {
                    continue;
                };
                labeled += 1;
                if bones
                    .iter()
                    .any(|&b| map.source_of(b).is_some_and(|a| want.contains(&a)))
                {
                    hits += 1;
                } else {
                    let got = bones
                        .iter()
                        .map(|&b| map.source_of(b).map_or("-".to_owned(), |a| ss.bones()[a].name.clone()))
                        .collect::<Vec<_>>()
                        .join("|");
                    misses.push(format!("{joint} <- {got}"));
                }
            }
            found += hits;

            // Posture: the source at rest, played.
            let rest = Clip {
                times: vec![0.0],
                poses: vec![ss.rest()],
            };
            let played = map.apply(ss, ts, &rest).expect("plays");
            let (t_rest, t_now) = (ts.world(&ts.rest()), ts.world(&played.poses[0]));
            let mut posture = 0.0f32;
            for joint in AXIS {
                let Some(&b) = t.labels.get(joint).and_then(|b| b.first()) else {
                    continue;
                };
                let turn = (t_now[b].rotation * t_rest[b].rotation.inverse()).angle_between(glam::Quat::IDENTITY);
                posture = posture.max(turn.to_degrees());
            }
            posture_sum += posture;
            posture_n += 1;
            if posture > posture_worst.0 {
                posture_worst = (posture, format!("{} on {}", s.name, t.name));
            }

            // Floor: every clip with every fix, past how far the source's own feet sink.
            let mut sink = 0.0f32;
            for (_, clip) in &s.model.clips {
                let out = map.apply_with(ss, ts, &sampled(clip), Fixes::ALL).expect("plays");
                sink = sink.max(sunk(t, &out));
            }
            let sink = (sink - own[&s.name]).max(0.0);
            floor_sum.0 += sink;
            floor_sum.1 += 1;
            if sink > floor_worst.0 {
                floor_worst = (sink, format!("{} on {}", s.name, t.name));
            }
            rows.push((s.name.clone(), t.name.clone(), hits, misses, posture, sink));
        }
    }
    let joints = 100.0 * found as f32 / labeled.max(1) as f32;
    let posture = posture_sum / posture_n.max(1) as f32;
    let floor = floor_sum.0 / floor_sum.1.max(1) as f32;
    println!(
        "## Four-legged animals\n\n{} animals, {} pairs, in {:.1} s.\n",
        animals.len(),
        rows.len(),
        started.elapsed().as_secs_f32()
    );
    println!("| | Mean | Worst |\n| --- | ---: | ---: |");
    println!("| Joints found | {joints:.1}% | |");
    println!(
        "| Posture at rest (axis turned from its own rest) | {posture:.2}° | {:.2}° ({}) |",
        posture_worst.0, posture_worst.1
    );
    println!(
        "| Feet below the floor past the source's (% of height) | {floor:.2}% | {:.2}% ({}) |",
        floor_worst.0, floor_worst.1
    );
    if has("--pairs") {
        println!();
        for (s, t, hits, misses, posture, sink) in &rows {
            println!("{s} -> {t}: {hits} found, posture {posture:.1}°, floor {sink:.2}%");
            for m in misses {
                println!("    {m}");
            }
        }
    }
    let mut failed = false;
    if let Some(min) = value("--min")
        && joints < min
    {
        eprintln!("joints found {joints:.1}% < {min}%");
        failed = true;
    }
    if let Some(max) = value("--max-posture")
        && posture_worst.0 > max
    {
        eprintln!("posture {:.2}° > {max}° ({})", posture_worst.0, posture_worst.1);
        failed = true;
    }
    if let Some(max) = value("--max-floor")
        && floor_worst.0 > max
    {
        eprintln!(
            "feet {:.2}% below the floor > {max}% ({})",
            floor_worst.0, floor_worst.1
        );
        failed = true;
    }
    if failed {
        std::process::exit(1);
    }
}
