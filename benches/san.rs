//! dittomo on the Mixamo test of Skeleton-Aware Networks (Aberman et al. 2020), scored as that paper's code scores
//! its own network, so the numbers sit beside its published table.
//!
//! ```text
//! sh bench/san/fetch.sh                       # download SAN's test set (once, 84 MB)
//! cargo bench --bench san                     # print the table
//! cargo bench --bench san -- --per-character  # and each target's row
//! cargo bench --bench san -- --data DIR       # SAN's Mixamo folder somewhere else
//! ```
//!
//! The test set is SAN's: the 106 clips in `test_list.txt`, as Mixamo itself put them on each of eight characters.
//! The four test characters (Mousey, Goblin, Mremireh, Vampire) share a skeleton. BigVegas has another, rooted
//! at a pelvis below the hips, with hair, a cape, sleeves and a jowl. Each clip is played from one character onto
//! another and compared with Mixamo's own version of it on the target, the ground truth SAN compares with:
//!
//! - intra-structural: each test character onto each of the other three (12 pairs);
//! - cross-structural: BigVegas onto each test character (4 pairs), as SAN's `get_error.py` does.
//!
//! The error is SAN's `get_error.py`: for each clip the mean, over frames, the target's 28 scored joints and x, y and z,
//! of the squared difference between the played and the true joint positions in the world, divided by the square of
//! the target's height (toe tip to root plus head top to root, along its bones). The clips are each averaged per
//! target character, then the characters are averaged, and the result is multiplied by 1000, as the paper's table
//! is. Clips are read at 30 frames a second (every other frame) and cut to a multiple of four frames, as SAN's
//! loader does. The mean distance as a share of height is printed too, being easier to picture.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

use dittomo::{Bone, Clip, Fixes, Mapping, Quat, Skeleton, Transform, Vec3};

const TEST: [&str; 4] = ["Mousey_m", "Goblin_m", "Mremireh_m", "Vampire_m"];
const CROSS: &str = "BigVegas";

/// The joints SAN scores on its test characters' skeleton (`corps_name_mixamo2_m` in its `bvh_parser.py`).
const SCORED: [&str; 28] = [
    "Hips",
    "LeftUpLeg",
    "LeftLeg",
    "LeftFoot",
    "LeftToeBase",
    "LeftToe_End",
    "RightUpLeg",
    "RightLeg",
    "RightFoot",
    "RightToeBase",
    "RightToe_End",
    "Spine",
    "Spine1",
    "Spine1_split",
    "Spine2",
    "Neck",
    "Head",
    "HeadTop_End",
    "LeftShoulder",
    "LeftShoulder_split",
    "LeftArm",
    "LeftForeArm",
    "LeftHand",
    "RightShoulder",
    "RightShoulder_split",
    "RightArm",
    "RightForeArm",
    "RightHand",
];

/// A BVH file: its skeleton (the rest pose being its offsets, unturned) and its motion.
struct Bvh {
    skeleton: Skeleton,
    clip: Clip,
}

#[derive(Clone, Copy)]
enum Channel {
    Position(usize),
    Rotation(usize),
}

fn read_bvh(path: &Path) -> Result<Bvh, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut words = text.split_ascii_whitespace();
    let mut bones: Vec<Bone> = Vec::new();
    let mut channels: Vec<Vec<Channel>> = Vec::new();
    // The bone each open brace belongs to; None for an end site, which isn't a bone.
    let mut open: Vec<Option<usize>> = Vec::new();
    let mut pending: Option<(String, bool)> = None;
    let float = |w: Option<&str>| -> Result<f32, String> {
        w.ok_or("file ends early")?.parse::<f32>().map_err(|e| format!("{e}"))
    };
    loop {
        let w = words.next().ok_or("no MOTION section")?;
        match w {
            "HIERARCHY" => {}
            "ROOT" | "JOINT" => {
                let name = words.next().ok_or("joint without a name")?.to_string();
                pending = Some((name, true));
            }
            "End" => {
                words.next(); // "Site"
                pending = Some((String::new(), false));
            }
            "{" => match pending.take() {
                Some((name, true)) => {
                    let parent = open.iter().rev().find_map(|b| *b);
                    bones.push(Bone {
                        name,
                        parent,
                        rest: Transform::IDENTITY,
                    });
                    channels.push(Vec::new());
                    open.push(Some(bones.len() - 1));
                }
                Some((_, false)) => open.push(None),
                None => return Err("brace without a joint".into()),
            },
            "}" => {
                open.pop();
            }
            "OFFSET" => {
                let v = Vec3::new(float(words.next())?, float(words.next())?, float(words.next())?);
                if let Some(Some(b)) = open.last() {
                    bones[*b].rest.translation = v;
                }
            }
            "CHANNELS" => {
                let n: usize = words
                    .next()
                    .ok_or("CHANNELS without a count")?
                    .parse()
                    .map_err(|e| format!("{e}"))?;
                let b = open.last().copied().flatten().ok_or("channels on an end site")?;
                for _ in 0..n {
                    let c = words.next().ok_or("too few channels")?;
                    let axis = match c.as_bytes().first() {
                        Some(b'X') => 0,
                        Some(b'Y') => 1,
                        Some(b'Z') => 2,
                        _ => return Err(format!("unknown channel {c}")),
                    };
                    channels[b].push(if c.ends_with("position") {
                        Channel::Position(axis)
                    } else {
                        Channel::Rotation(axis)
                    });
                }
            }
            "MOTION" => break,
            other => return Err(format!("unexpected {other}")),
        }
    }
    let _ = words.next(); // "Frames:"
    let frames: usize = words
        .next()
        .ok_or("no frame count")?
        .parse()
        .map_err(|e| format!("{e}"))?;
    let _ = (words.next(), words.next()); // "Frame" "Time:"
    let dt = float(words.next())?;
    let skeleton = Skeleton::new(bones).map_err(|e| format!("{e:?}"))?;
    let rest = skeleton.rest();
    let mut clip = Clip::default();
    for f in 0..frames {
        let mut pose = rest.clone();
        for (b, list) in channels.iter().enumerate() {
            let mut position = pose[b].translation;
            let mut rotation = Quat::IDENTITY;
            for c in list {
                let v = float(words.next())?;
                match *c {
                    Channel::Position(a) => position[a] = v,
                    Channel::Rotation(a) => {
                        let r = v.to_radians();
                        // Each rotation channel turns about its axis after the ones before it (intrinsic order).
                        rotation *= match a {
                            0 => Quat::from_rotation_x(r),
                            1 => Quat::from_rotation_y(r),
                            _ => Quat::from_rotation_z(r),
                        };
                    }
                }
            }
            pose[b] = Transform::new(position, rotation.normalize());
        }
        clip.times.push(f as f32 * dt);
        clip.poses.push(pose);
    }
    Ok(Bvh { skeleton, clip })
}

/// SAN's loader: every other frame, cut to a multiple of four.
fn as_san_reads(clip: &Clip) -> Clip {
    let mut out = Clip::default();
    for (t, p) in clip.times.iter().zip(&clip.poses).step_by(2) {
        out.times.push(*t);
        out.poses.push(p.clone());
    }
    let n = out.times.len() / 4 * 4;
    out.times.truncate(n);
    out.poses.truncate(n);
    out
}

fn bare(name: &str) -> &str {
    name.rsplit(':').next().unwrap_or(name)
}

/// SAN's height: along the bones from the left toe tip up to the root, and from the head top to the root.
fn height(s: &Skeleton) -> f32 {
    let chain = |end: &str| {
        let mut b = s.bones().iter().position(|b| bare(&b.name) == end).expect("end joint");
        let mut sum = 0.0;
        while let Some(p) = s.bones()[b].parent {
            sum += s.bones()[b].rest.translation.length();
            b = p;
        }
        sum
    };
    chain("LeftToe_End") + chain("HeadTop_End")
}

/// The clip copied onto the target bone for bone by name, each bone's local rotation and the root's position as they
/// are: the "copy rotations" row of SAN's table.
fn copy_rotations(source: &Skeleton, target: &Skeleton, clip: &Clip, scale: f32) -> Clip {
    let from: Vec<Option<usize>> = target
        .bones()
        .iter()
        .map(|t| source.bones().iter().position(|s| bare(&s.name) == bare(&t.name)))
        .collect();
    let rest = target.rest();
    Clip {
        times: clip.times.clone(),
        poses: clip
            .poses
            .iter()
            .map(|p| {
                rest.iter()
                    .enumerate()
                    .map(|(t, r)| match from[t] {
                        Some(s) if target.bones()[t].parent.is_none() => {
                            Transform::new(p[s].translation * scale, p[s].rotation)
                        }
                        Some(s) => Transform::new(r.translation, p[s].rotation),
                        None => *r,
                    })
                    .collect()
            })
            .collect(),
    }
}

/// One clip's scores, before the ×1000.
#[derive(Default, Clone, Copy)]
struct Score {
    /// SAN's error: squared, per coordinate, over height squared.
    squared: f64,
    /// The mean distance over height.
    distance: f64,
    /// SAN's error with each frame's root put where the truth's is (R2ET's "local MSE").
    local: f64,
}

fn error(target: &Skeleton, played: &Clip, truth: &Clip, h: f32) -> Score {
    let scored: Vec<usize> = SCORED
        .iter()
        .map(|n| {
            target
                .bones()
                .iter()
                .position(|b| bare(&b.name) == *n)
                .expect("scored joint")
        })
        .collect();
    let root = scored[0];
    let frames = played.poses.len().min(truth.poses.len());
    let mut s = Score::default();
    for f in 0..frames {
        let (a, b) = (target.world(&played.poses[f]), target.world(&truth.poses[f]));
        let shift = b[root].position() - a[root].position();
        for &j in &scored {
            let d = a[j].position() - b[j].position();
            s.squared += f64::from(d.length_squared());
            s.distance += f64::from(d.length());
            s.local += f64::from((d + shift).length_squared());
        }
    }
    let n = (frames * scored.len()) as f64;
    let h = f64::from(h);
    Score {
        squared: s.squared / (n * 3.0) / (h * h),
        distance: s.distance / n / h,
        local: s.local / (n * 3.0) / (h * h),
    }
}

/// Each scored joint's root-aligned squared distance over height squared, averaged over frames.
fn by_joint(target: &Skeleton, played: &Clip, truth: &Clip, h: f32) -> [f64; SCORED.len()] {
    let scored: Vec<usize> = SCORED
        .iter()
        .map(|n| {
            target
                .bones()
                .iter()
                .position(|b| bare(&b.name) == *n)
                .expect("scored joint")
        })
        .collect();
    let frames = played.poses.len().min(truth.poses.len());
    let mut out = [0.0; SCORED.len()];
    for f in 0..frames {
        let (a, b) = (target.world(&played.poses[f]), target.world(&truth.poses[f]));
        let shift = b[scored[0]].position() - a[scored[0]].position();
        for (k, &j) in scored.iter().enumerate() {
            out[k] += f64::from((a[j].position() + shift - b[j].position()).length_squared());
        }
    }
    let h = f64::from(h);
    out.map(|e| e / frames.max(1) as f64 / (h * h))
}

#[derive(Default, Clone)]
struct Tally {
    sum: Score,
    clips: usize,
}

impl Tally {
    fn add(&mut self, s: Score) {
        self.sum.squared += s.squared;
        self.sum.distance += s.distance;
        self.sum.local += s.local;
        self.clips += 1;
    }

    fn mean(&self) -> Score {
        let n = self.clips.max(1) as f64;
        Score {
            squared: self.sum.squared / n,
            distance: self.sum.distance / n,
            local: self.sum.local / n,
        }
    }
}

#[derive(Clone, Copy)]
enum Method {
    /// Each bone's local rotation and the root's position, copied by name.
    Copy,
    /// The same, the root's position scaled by the ratio of the hips' heights.
    CopyScaled,
    /// dittomo, with these fixes.
    Dittomo(Fixes, &'static str),
}

impl Method {
    fn name(self) -> &'static str {
        match self {
            Self::Copy => "Copy rotations",
            Self::CopyScaled => "Copy, root scaled",
            Self::Dittomo(_, name) => name,
        }
    }

    fn play(self, source: &Skeleton, target: &Skeleton, clip: &Clip) -> Clip {
        match self {
            Self::Copy => copy_rotations(source, target, clip, 1.0),
            Self::CopyScaled => copy_rotations(source, target, clip, hips(target) / hips(source)),
            Self::Dittomo(fixes, _) => {
                let mapping = Mapping::new(source, target);
                mapping.apply_with(source, target, clip, fixes).expect("plays")
            }
        }
    }
}

const fn only(travel: bool, feet: bool, push_out: bool) -> Fixes {
    Fixes {
        travel,
        feet,
        push_out,
        floor: false,
        touch: false,
        hands: false,
        head: false,
        place: false,
    }
}

const METHODS: [Method; 7] = [
    Method::Copy,
    Method::CopyScaled,
    Method::Dittomo(Fixes::NONE, "dittomo"),
    Method::Dittomo(only(true, false, false), "dittomo + travel"),
    Method::Dittomo(only(false, true, false), "dittomo + feet"),
    Method::Dittomo(only(false, false, true), "dittomo + push out"),
    Method::Dittomo(Fixes::ALL, "dittomo + all fixes"),
];

/// How high the hips are at rest.
fn hips(s: &Skeleton) -> f32 {
    let i = s.bones().iter().position(|b| bare(&b.name) == "Hips").expect("hips");
    s.world(&s.rest())[i].position().y
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let has = |f: &str| args.iter().any(|a| a == f);
    let data = args
        .iter()
        .position(|a| a == "--data")
        .and_then(|i| args.get(i + 1))
        .map_or_else(
            || Path::new(env!("CARGO_MANIFEST_DIR")).join("bench/san/data/Mixamo"),
            PathBuf::from,
        );
    let list = match std::fs::read_to_string(data.join("test_list.txt")) {
        Ok(l) => l,
        Err(_) => {
            eprintln!("no SAN test set in {}: run sh bench/san/fetch.sh", data.display());
            return;
        }
    };
    if let Some(i) = args.iter().position(|a| a == "--map") {
        let (source, target) = (&args[i + 1], &args[i + 2]);
        let clip = list.lines().next().unwrap_or_default().trim();
        let s = read_bvh(&data.join(source).join(clip)).expect("source");
        let t = read_bvh(&data.join(target).join(clip)).expect("target");
        let mapping = Mapping::new(&s.skeleton, &t.skeleton);
        let sw = s.skeleton.world(&s.skeleton.rest());
        let tw = t.skeleton.world(&t.skeleton.rest());
        for (ti, pair) in mapping.pairs().iter().enumerate() {
            let tb = &t.skeleton.bones()[ti];
            let depth = std::iter::successors(tb.parent, |p| t.skeleton.bones()[*p].parent).count();
            let what = pair.as_ref().map_or_else(
                || "-".to_string(),
                |p| {
                    format!(
                        "{} ({:.2}) src at {:?}",
                        s.skeleton.bones()[p.source].name,
                        p.score,
                        sw[p.source].position()
                    )
                },
            );
            println!("{}{} at {:?} <- {what}", "  ".repeat(depth), tb.name, tw[ti].position());
        }
        println!("source tree:");
        for (si, b) in s.skeleton.bones().iter().enumerate() {
            let depth = std::iter::successors(b.parent, |p| s.skeleton.bones()[*p].parent).count();
            println!("{}{} at {:?}", "  ".repeat(depth), b.name, sw[si].position());
        }
        return;
    }
    let clips: Vec<&str> = list.lines().map(str::trim).filter(|l| !l.is_empty()).collect();
    let mut cache: BTreeMap<(String, String), Bvh> = BTreeMap::new();
    let mut load = |character: &str, clip: &str| -> Option<(Skeleton, Clip)> {
        let key = (character.to_string(), clip.to_string());
        if !cache.contains_key(&key) {
            match read_bvh(&data.join(character).join(clip)) {
                Ok(b) => {
                    cache.insert(key.clone(), b);
                }
                Err(e) => {
                    eprintln!("{character}/{clip}: {e}");
                    return None;
                }
            }
        }
        let b = &cache[&key];
        Some((b.skeleton.clone(), as_san_reads(&b.clip)))
    };
    let heights: BTreeMap<&str, f32> = TEST
        .iter()
        .map(|c| {
            let s = read_bvh(&data.join("std_bvhs").join(format!("{c}.bvh"))).expect("standard skeleton");
            (*c, height(&s.skeleton))
        })
        .collect();

    // [split][method][target]
    let mut tallies = vec![vec![vec![Tally::default(); TEST.len()]; METHODS.len()]; 2];
    let mut timing = [0.0f64; METHODS.len()];
    let mut frames_played = 0usize;
    let mut worst: Vec<Vec<(f64, String)>> = vec![Vec::new(); METHODS.len()];
    // [split][method][scored joint]: root-aligned squared distance over height squared, summed over clips.
    let mut joints = vec![vec![[0.0f64; SCORED.len()]; METHODS.len()]; 2];
    for (split, sources) in [(0, TEST.to_vec()), (1, vec![CROSS])] {
        for (ti, target) in TEST.iter().enumerate() {
            for source in &sources {
                if source == target {
                    continue;
                }
                for clip in &clips {
                    let (Some((ss, sc)), Some((ts, truth))) = (load(source, clip), load(target, clip)) else {
                        continue;
                    };
                    let h = heights[target];
                    frames_played += sc.poses.len();
                    for (m, method) in METHODS.iter().enumerate() {
                        let t0 = Instant::now();
                        let played = method.play(&ss, &ts, &sc);
                        timing[m] += t0.elapsed().as_secs_f64();
                        let score = error(&ts, &played, &truth, h);
                        if has("--joints") {
                            for (j, e) in by_joint(&ts, &played, &truth, h).iter().enumerate() {
                                joints[split][m][j] += e;
                            }
                        }
                        tallies[split][m][ti].add(score);
                        worst[m].push((score.squared, format!("{source} -> {target}: {clip}")));
                    }
                }
            }
        }
    }

    // Each target's mean, then the mean of the targets, as SAN averages.
    let summary = |split: usize, m: usize| -> (Score, usize) {
        let rows: Vec<Score> = tallies[split][m]
            .iter()
            .filter(|t| t.clips > 0)
            .map(Tally::mean)
            .collect();
        let n = rows.len().max(1) as f64;
        let mean = Score {
            squared: rows.iter().map(|r| r.squared).sum::<f64>() / n * 1000.0,
            distance: rows.iter().map(|r| r.distance).sum::<f64>() / n * 100.0,
            local: rows.iter().map(|r| r.local).sum::<f64>() / n * 1000.0,
        };
        (mean, tallies[split][m].iter().map(|t| t.clips).sum())
    };
    println!(
        "SAN's Mixamo test: {} clips, error ×10³ (SAN's get_error), mean distance as % of height",
        clips.len()
    );
    println!();
    println!("{:<20} {:>26}   {:>26}", "", "intra-structural", "cross-structural");
    println!(
        "{:<20} {:>8} {:>8} {:>8}   {:>8} {:>8} {:>8}",
        "method", "error", "local", "dist %", "error", "local", "dist %"
    );
    for (m, method) in METHODS.iter().enumerate() {
        let (i, ic) = summary(0, m);
        let (c, cc) = summary(1, m);
        println!(
            "{:<20} {:>8.3} {:>8.3} {:>8.2}   {:>8.3} {:>8.3} {:>8.2}   ({ic} + {cc} clip pairs)",
            method.name(),
            i.squared,
            i.local,
            i.distance,
            c.squared,
            c.local,
            c.distance
        );
    }
    println!();
    println!("time per frame played, mapping included ({frames_played} frames each):");
    for (m, method) in METHODS.iter().enumerate() {
        println!(
            "  {:<20} {:>7.2} µs",
            method.name(),
            timing[m] / frames_played as f64 * 1e6
        );
    }
    if has("--per-character") {
        for (split, label) in [(0, "intra"), (1, "cross")] {
            println!();
            println!("{label}: error ×10³ by target");
            for (ti, target) in TEST.iter().enumerate() {
                let row: Vec<String> = (0..METHODS.len())
                    .map(|m| format!("{:>9.3}", tallies[split][m][ti].mean().squared * 1000.0))
                    .collect();
                println!("{target:<12} h={:>6.1} {}", heights[target], row.join(" "));
            }
        }
    }
    if has("--worst") {
        for (m, method) in METHODS.iter().enumerate() {
            let w = &mut worst[m];
            w.sort_by(|a, b| b.0.total_cmp(&a.0));
            println!();
            println!("{}: worst clips, error ×10³", method.name());
            for (e, what) in w.iter().take(6) {
                println!("  {:>9.3}  {what}", e * 1000.0);
            }
        }
    }
    if has("--joints") {
        for (split, label) in [(0, "intra"), (1, "cross")] {
            let clips = tallies[split][0].iter().map(|t| t.clips).sum::<usize>().max(1) as f64;
            println!();
            println!("{label}: root-aligned squared error by joint, ×10³ (per clip, all coordinates)");
            let names: Vec<&str> = METHODS.iter().map(|m| m.name()).collect();
            println!(
                "{:<20} {}",
                "joint",
                names
                    .iter()
                    .map(|n| format!("{:>10.10}", n))
                    .collect::<Vec<_>>()
                    .join("")
            );
            for (j, joint) in SCORED.iter().enumerate() {
                let row: String = (0..METHODS.len())
                    .map(|m| format!("{:>10.3}", joints[split][m][j] / clips * 1000.0))
                    .collect();
                println!("{joint:<20} {row}");
            }
        }
    }
}
