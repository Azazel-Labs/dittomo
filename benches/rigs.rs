//! How well mapping finds the right bones on real rigs, against matching by name.
//!
//! ```text
//! sh bench/rigs/fetch.sh                      # download the rigs (once)
//! cargo bench --features fs --bench rigs      # print the table
//! cargo bench --features fs --bench rigs -- --dump     # print each rig's tree
//! cargo bench --features fs --bench rigs -- --pairs    # print every pair's misses
//! cargo bench --features fs --bench rigs -- --show xbot cesium   # the target's tree, with what each bone plays
//! cargo bench --features fs --bench rigs -- --min 95   # fail below 95% of joints found (CI)
//! cargo bench --features fs --bench rigs -- --max-degrees 3   # fail if limbs stray more than 3° on average (CI)
//! ```
//!
//! `bench/rigs/labels.txt` names, for each rig, the bone playing each of 25 joints of the body (hips, spine, chest,
//! neck, head, and on each side the clavicle, upper arm, forearm, hand, index finger, thumb, thigh, shin, foot and toe),
//! labeled by hand; a rig without one of them leaves it out.
//! Each ordered pair of rigs is mapped, and a joint is found when the target's joint bone plays the source's.
//!
//! Then every clip of every rig is played on every other rig, and each labeled limb segment's direction (upper arm
//! from shoulder to elbow, and so on) is compared with the source's, each in its own body's frame, against two ways
//! of copying a clip onto the right bones without dittomo: copying each bone's local rotation, as an engine's
//! animation mixer does when a clip is played on another rig by bone name, and copying each bone's turn from rest in
//! the world.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use dittomo::load::Model;
use dittomo::{Clip, Fixes, Live, Mapping, Quat, Skeleton, Surface, Transform, Vec3};
use glam::Mat3;

#[path = "rigs/godot.rs"]
mod godot;

const JOINTS: [&str; 25] = [
    "hips",
    "spine",
    "chest",
    "neck",
    "head",
    "l_clavicle",
    "l_upperarm",
    "l_forearm",
    "l_hand",
    "r_clavicle",
    "r_upperarm",
    "r_forearm",
    "r_hand",
    "l_thigh",
    "l_shin",
    "l_foot",
    "l_toe",
    "r_thigh",
    "r_shin",
    "r_foot",
    "r_toe",
    "l_index",
    "r_index",
    "l_thumb",
    "r_thumb",
];

struct Rig {
    name: String,
    model: Model,
    /// Each labeled joint's bone.
    labels: BTreeMap<String, usize>,
}

fn dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("bench/rigs")
}

/// The labels file: `[rig file]` then `joint = bone name` lines; `#` starts a comment.
fn labels(path: &Path) -> BTreeMap<String, BTreeMap<String, String>> {
    let mut out: BTreeMap<String, BTreeMap<String, String>> = BTreeMap::new();
    let Ok(text) = std::fs::read_to_string(path) else {
        return out;
    };
    let mut rig = String::new();
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        if let Some(name) = line.strip_prefix('[').and_then(|l| l.strip_suffix(']')) {
            rig = name.to_string();
        } else if let Some((joint, bone)) = line.split_once('=') {
            out.entry(rig.clone())
                .or_default()
                .insert(joint.trim().to_string(), bone.trim().to_string());
        }
    }
    out
}

fn dump(name: &str, s: &Skeleton) {
    println!("[{name}] {} bones", s.len());
    let world = s.world(&s.rest());
    let depth = |mut b: usize| {
        let mut d = 0;
        while let Some(p) = s.bones()[b].parent {
            d += 1;
            b = p;
        }
        d
    };
    for (i, b) in s.bones().iter().enumerate() {
        let p = world[i].position();
        println!(
            "{:4} {}{}  ({:.2}, {:.2}, {:.2})",
            i,
            "  ".repeat(depth(i)),
            b.name,
            p.x,
            p.y,
            p.z
        );
    }
}

/// The target bone of the same name as each source bone, ignoring case and a namespace before `:`: how three.js's
/// `SkeletonUtils.retarget` and Babylon match by default.
fn by_name(source: &Skeleton, target: &Skeleton) -> Vec<Option<usize>> {
    let key = |n: &str| n.rsplit(':').next().unwrap_or(n).to_ascii_lowercase();
    target
        .bones()
        .iter()
        .map(|t| source.bones().iter().position(|s| key(&s.name) == key(&t.name)))
        .collect()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let has = |f: &str| args.iter().any(|a| a == f);
    let data = dir().join("data");
    let mut files: Vec<PathBuf> = std::fs::read_dir(&data)
        .map(|d| d.filter_map(|e| e.ok().map(|e| e.path())).collect())
        .unwrap_or_default();
    files.sort();
    if files.is_empty() {
        eprintln!("no rigs in {}: run sh bench/rigs/fetch.sh", data.display());
        return;
    }
    let all_labels = labels(&dir().join("labels.txt"));
    let mut rigs = Vec::new();
    let mut picked = Vec::new();
    for f in &files {
        let name = f.file_name().unwrap().to_string_lossy().to_string();
        let model = match Model::load(f) {
            Ok(m) => m,
            Err(e) => {
                eprintln!("{name}: {e}");
                continue;
            }
        };
        if has("--dump") {
            dump(&name, &model.skeleton);
            continue;
        }
        let mut labels = BTreeMap::new();
        // Motion capture picked for its moves has the capture rig's bones, and is kept apart from the rigs.
        let extra = name.starts_with("cmu-") && name != "cmu-mocap.bvh";
        let key = if extra { "cmu-mocap.bvh" } else { name.as_str() };
        for (joint, bone) in all_labels.get(key).into_iter().flatten() {
            match model.skeleton.index_of(bone) {
                Some(b) => {
                    labels.insert(joint.clone(), b);
                }
                None => eprintln!("{name}: no bone {bone} for {joint}"),
            }
        }
        if labels.is_empty() {
            eprintln!("{name}: no labels, left out");
            continue;
        }
        if extra {
            picked.push(Rig { name, model, labels });
        } else {
            rigs.push(Rig { name, model, labels });
        }
    }
    if has("--dump") {
        return;
    }
    // --atlas: only the contact atlas.
    if has("--atlas") {
        atlas(&rigs);
        meeting(&rigs);
        return;
    }
    // --moves: only the picked motion capture.
    if has("--meeting") {
        meeting(&rigs);
        return;
    }
    if has("--moves") {
        moves(&picked, &rigs);
        return;
    }
    // --low-hands: how often each clip has a wrist near the floor.
    if has("--low-hands") {
        for r in &rigs {
            let (sk, h) = (&r.model.skeleton, height(&r.model.skeleton));
            let floor = sk
                .world(&sk.rest())
                .iter()
                .map(|p| p.position().y)
                .fold(f32::MAX, f32::min);
            for (name, clip) in &r.model.clips {
                let low = clip
                    .poses
                    .iter()
                    .filter(|p| {
                        let w = sk.world(p);
                        ["l_hand", "r_hand"]
                            .iter()
                            .filter_map(|j| r.labels.get(*j))
                            .any(|&b| w[b].position().y - floor < 0.08 * h)
                    })
                    .count();
                println!("{} {name:?}: {low} of {}", r.name, clip.poses.len());
            }
        }
        return;
    }
    // --contact: only the fixes on the clips.
    if has("--contact") {
        contact(&rigs);
        return;
    }

    // --show source target: the target's tree, each bone with what it plays.
    if let Some(i) = args.iter().position(|a| a == "--show") {
        let find = |n: &str| rigs.iter().find(|r| r.name.starts_with(n)).expect("a rig of that name");
        let (s, t) = (find(&args[i + 1]), find(&args[i + 2]));
        let (ss, ts) = (&s.model.skeleton, &t.model.skeleton);
        let map = Mapping::new(ss, ts);
        println!("{} → {}: quality {:.2}", s.name, t.name, map.quality());
        for (b, bone) in ts.bones().iter().enumerate() {
            let mut d = 0;
            let mut x = b;
            while let Some(p) = ts.bones()[x].parent {
                d += 1;
                x = p;
            }
            let played = map.pairs()[b].map_or("-".to_string(), |p| {
                format!("{} ({:.2})", ss.bones()[p.source].name, p.score)
            });
            println!("{:4} {}{}  ←  {played}", b, "  ".repeat(d), bone.name);
        }
        return;
    }

    let show = has("--pairs");
    let (mut found, mut named, mut godot, mut asked) = (0usize, 0usize, 0usize, 0usize);
    let mut per: BTreeMap<&str, (usize, usize, usize, usize)> = BTreeMap::new();
    // Godot's profile name for each bone of each rig: a pair maps both rigs to the profile, as a Godot `BoneMap` does.
    let profiled: Vec<Vec<Option<&str>>> = rigs.iter().map(|r| godot::auto_map(&r.model.skeleton)).collect();
    let mut worst: Vec<(f32, String)> = Vec::new();
    for (si, s) in rigs.iter().enumerate() {
        for (ti, t) in rigs.iter().enumerate() {
            if std::ptr::eq(s, t) {
                continue;
            }
            let (ss, ts) = (&s.model.skeleton, &t.model.skeleton);
            let map = Mapping::new(ss, ts);
            let names = by_name(ss, ts);
            let (gs, gt) = (&profiled[si], &profiled[ti]);
            let (mut f, mut nm, mut gd, mut a) = (0, 0, 0, 0);
            let mut misses = Vec::new();
            for joint in JOINTS {
                let (Some(&sb), Some(&tb)) = (s.labels.get(joint), t.labels.get(joint)) else {
                    continue;
                };
                a += 1;
                let e = per.entry(joint).or_default();
                e.3 += 1;
                if map.source_of(tb) == Some(sb) {
                    f += 1;
                    e.0 += 1;
                } else {
                    misses.push(format!(
                        "{joint}: {} plays {} (want {})",
                        ts.bones()[tb].name,
                        map.source_of(tb).map_or("nothing", |b| ss.bones()[b].name.as_str()),
                        ss.bones()[sb].name
                    ));
                }
                if names[tb] == Some(sb) {
                    nm += 1;
                    e.1 += 1;
                }
                if gt[tb].is_some() && gs.iter().position(|p| *p == gt[tb]) == Some(sb) {
                    gd += 1;
                    e.2 += 1;
                }
            }
            found += f;
            named += nm;
            godot += gd;
            asked += a;
            let share = f as f32 / a.max(1) as f32;
            worst.push((share, format!("{} → {}", s.name, t.name)));
            if show && !misses.is_empty() {
                println!("{} → {}: {f}/{a}, quality {:.2}", s.name, t.name, map.quality());
                for m in &misses {
                    println!("    {m}");
                }
            }
        }
    }
    let pct = |a: usize, b: usize| 100.0 * a as f64 / b.max(1) as f64;
    println!(
        "\n{} rigs, {} ordered pairs, {asked} labeled joints",
        rigs.len(),
        rigs.len() * (rigs.len() - 1)
    );
    println!("| Matcher | joints found |\n| --- | ---: |");
    println!("| dittomo | {:.1}% |", pct(found, asked));
    println!("| exact name (three.js, Babylon default) | {:.1}% |", pct(named, asked));
    println!("| Godot humanoid auto mapping | {:.1}% |", pct(godot, asked));
    println!("\n| Joint | dittomo | by name | Godot |\n| --- | ---: | ---: | ---: |");
    for joint in JOINTS {
        if let Some(&(f, n, g, a)) = per.get(joint) {
            println!(
                "| {joint} | {:.0}% | {:.0}% | {:.0}% |",
                pct(f, a),
                pct(n, a),
                pct(g, a)
            );
        }
    }
    worst.sort_by(|a, b| a.0.total_cmp(&b.0));
    println!("\nWorst pairs:\n");
    for (share, name) in worst.iter().take(10) {
        println!("- {:.0}% {name}", share * 100.0);
    }
    let mean = motion(&rigs, show);
    contact(&rigs);
    touching(&rigs);
    atlas(&rigs);
    meeting(&rigs);
    moves(&picked, &rigs);
    let flag = |f: &str| {
        args.iter()
            .position(|a| a == f)
            .and_then(|i| args.get(i + 1))
            .and_then(|m| m.parse::<f64>().ok())
    };
    let mut failed = false;
    if let Some(min) = flag("--min")
        && pct(found, asked) < min
    {
        eprintln!("found {:.1}% of joints, under {min}%", pct(found, asked));
        failed = true;
    }
    if let Some(max) = flag("--max-degrees")
        && f64::from(mean) > max
    {
        eprintln!("limbs point {mean:.1}° from the source's on average, over {max}°");
        failed = true;
    }
    if failed {
        std::process::exit(1);
    }
}

/// Limb segments, from one labeled joint to the next, whose directions a retarget should copy.
const SEGMENTS: [(&str, &str); 18] = [
    ("hips", "chest"),
    ("chest", "head"),
    ("l_upperarm", "l_forearm"),
    ("l_forearm", "l_hand"),
    ("l_hand", "l_index"),
    ("l_hand", "l_thumb"),
    ("r_upperarm", "r_forearm"),
    ("r_forearm", "r_hand"),
    ("r_hand", "r_index"),
    ("r_hand", "r_thumb"),
    ("l_thigh", "l_shin"),
    ("l_shin", "l_foot"),
    ("l_foot", "l_toe"),
    ("r_thigh", "r_shin"),
    ("r_shin", "r_foot"),
    ("r_foot", "r_toe"),
    ("l_clavicle", "l_upperarm"),
    ("r_clavicle", "r_upperarm"),
];

/// The most frames of a clip measured; longer clips are sampled evenly.
const FRAMES: usize = 40;

/// The body's frame at rest from the hand labels and the world's up, as rows (left, up, forward): a world direction times it is the
/// direction in the body's own terms, whichever way the rig faces and whatever its units.
fn body(rig: &Rig) -> Option<[Vec3; 3]> {
    let world = rig.model.skeleton.world(&rig.model.skeleton.rest());
    let at = |j: &str| rig.labels.get(j).map(|&b| world[b].position());
    let hips = at("hips").or_else(|| at("spine"))?;
    let top = at("head").or_else(|| at("neck")).or_else(|| at("chest"))?;
    // The world axis nearest hips to head, so a rig standing with its head forward isn't measured tilted.
    let rise = top - hips;
    let up = [Vec3::X, Vec3::Y, Vec3::Z, Vec3::NEG_X, Vec3::NEG_Y, Vec3::NEG_Z]
        .into_iter()
        .max_by(|a, b| a.dot(rise).total_cmp(&b.dot(rise)))
        .unwrap_or(Vec3::Y);
    let side = match (at("l_thigh"), at("r_thigh")) {
        (Some(l), Some(r)) => l - r,
        _ => at("l_upperarm")? - at("r_upperarm")?,
    };
    let left = (side - up * side.dot(up)).normalize_or_zero();
    (up != Vec3::ZERO && left != Vec3::ZERO).then(|| [left, up, left.cross(up)])
}

/// Each segment's direction in the body's frame. A foot's is where it points, the way it points at rest across the
/// floor turned as the foot has turned from rest: every rig stands flat at rest, and how steeply its ankle comes down
/// to its toes is anatomy, so matching that slope would push one rig's toes into the floor; a foot should turn as the
/// source's does. The trunk's (hips to chest, chest to head) is how it leans: straight up, tilted as the segment has
/// tilted from rest from the way the hips face. Every rig stands upright at rest, and how far its chest sits ahead of its hips or behind, or its
/// head ahead of its chest, is anatomy too. So is how a collarbone slopes up or back to the shoulder: its is straight
/// out to its side, tilted as it has tilted from rest from its parent. Rigs differ at rest in their upper arms (down or out), not there.
fn angles(rig: &Rig, frame: &[Vec3; 3], pose: &[Transform]) -> Vec<Option<Vec3>> {
    let s = &rig.model.skeleton;
    let (world, rest) = (s.world(pose), s.world(&s.rest()));
    SEGMENTS
        .iter()
        .map(|(ja, jb)| {
            let (a, b) = (*rig.labels.get(*ja)?, *rig.labels.get(*jb)?);
            let d = if ja.ends_with("_foot") {
                let d = rest[b].position() - rest[a].position();
                world[a].rotation * rest[a].rotation.inverse() * (d - frame[1] * frame[1].dot(d))
            } else if let Some(built) = match *ja {
                "hips" | "chest" => Some(frame[1]),
                "l_clavicle" => Some(frame[0]),
                "r_clavicle" => Some(-frame[0]),
                _ => None,
            } {
                // A collarbone's tilt is from its parent, the chest, and the trunk's from the way the hips face, so a
                // body that only turns about doesn't shrug or lean.
                let from = |p: usize| world[p].rotation * rest[p].rotation.inverse();
                let carry = match (ja.ends_with("_clavicle"), s.bones()[a].parent, rig.labels.get("hips")) {
                    (true, Some(p), _) => from(p),
                    (false, _, Some(&h)) => {
                        let q = from(h);
                        let about = q.xyz().dot(frame[1]) * frame[1];
                        let f = Quat::from_xyzw(about.x, about.y, about.z, q.w);
                        if f.length_squared() > 1e-6 {
                            f.normalize()
                        } else {
                            Quat::IDENTITY
                        }
                    }
                    _ => Quat::IDENTITY,
                };
                let (was, now) = (
                    rest[b].position() - rest[a].position(),
                    carry.inverse() * (world[b].position() - world[a].position()),
                );
                carry * Quat::from_rotation_arc(was.normalize(), now.normalize()) * built
            } else {
                world[b].position() - world[a].position()
            };
            let d = Vec3::new(frame[0].dot(d), frame[1].dot(d), frame[2].dot(d)).normalize_or_zero();
            (d != Vec3::ZERO).then_some(d)
        })
        .collect()
}

/// Where the head looks, in the body's frame: the way the body faces at rest, turned as the head has turned from rest.
/// Every rig looks ahead at rest, so this is the gaze whatever the head bone's axes or the skull's shape.
fn looks(rig: &Rig, frame: &[Vec3; 3], pose: &[Transform]) -> Option<Vec3> {
    let head = *rig.labels.get("head")?;
    let s = &rig.model.skeleton;
    let (now, rest) = (s.world(pose)[head].rotation, s.world(&s.rest())[head].rotation);
    let d = now * rest.inverse() * frame[2];
    Some(Vec3::new(frame[0].dot(d), frame[1].dot(d), frame[2].dot(d)).normalize())
}

/// Every clip of every rig that has one played on every other rig: how far, in degrees, each labeled limb segment
/// points from where the source's does, measured in each body's own frame. Gives dittomo's mean.
fn motion(rigs: &[Rig], show: bool) -> f32 {
    let methods = [
        "dittomo",
        "dittomo, labeled bones set by hand",
        "world turns from rest copied onto the labeled bones",
        "local rotations copied onto the labeled bones",
    ];
    let mut errs: [Vec<f32>; 4] = Default::default();
    let mut per: BTreeMap<(usize, usize), [Vec<f32>; 4]> = BTreeMap::new();
    let mut by_segment: Vec<[Vec<f32>; 4]> = vec![Default::default(); SEGMENTS.len()];
    let mut gaze: [Vec<f32>; 4] = Default::default();
    let mut clips = 0usize;
    for s in rigs {
        let Some(fs) = body(s) else { continue };
        if s.model.clips.is_empty() {
            continue;
        }
        for t in rigs {
            if std::ptr::eq(s, t) {
                continue;
            }
            let Some(ft) = body(t) else { continue };
            let (ss, ts) = (&s.model.skeleton, &t.model.skeleton);
            let map = Mapping::new(ss, ts);
            let mut fixed = map.clone();
            for (joint, &tb) in &t.labels {
                if let Some(&sb) = s.labels.get(joint) {
                    // Whatever else plays the labeled source bone by now, not as first matched: that would undo a
                    // label set before this one.
                    let others: Vec<usize> = (0..ts.len())
                        .filter(|&b| b != tb && fixed.source_of(b) == Some(sb))
                        .collect();
                    for b in others {
                        fixed.set(b, None);
                    }
                    fixed.set(tb, Some(sb));
                }
            }
            // The labeled pairs, and the turn from the source body's frame to the target's.
            let pairs: Vec<(usize, usize)> = t
                .labels
                .iter()
                .filter_map(|(j, &tb)| s.labels.get(j).map(|&sb| (tb, sb)))
                .collect();
            let turn = Quat::from_mat3(
                &(Mat3::from_cols(ft[0], ft[1], ft[2]) * Mat3::from_cols(fs[0], fs[1], fs[2]).transpose()),
            );
            let (src_rest, tgt_rest) = (ss.world(&ss.rest()), ts.world(&ts.rest()));
            let mut pair: [Vec<f32>; 4] = Default::default();
            for (_, clip) in &s.model.clips {
                let step = clip.poses.len().div_ceil(FRAMES).max(1);
                let poses: Vec<Vec<Transform>> = clip.poses.iter().step_by(step).cloned().collect();
                let sub = Clip {
                    times: clip.times.iter().step_by(step).copied().collect(),
                    poses,
                };
                let played = [
                    map.apply(ss, ts, &sub).expect("plays"),
                    fixed.apply(ss, ts, &sub).expect("plays"),
                ];
                clips += 1;
                for (f, pose) in sub.poses.iter().enumerate() {
                    let want = angles(s, &fs, pose);
                    let mut copied = ts.rest();
                    for &(tb, sb) in &pairs {
                        copied[tb].rotation = pose[sb].rotation;
                    }
                    // Each labeled bone turned in the world as its source is turned from rest.
                    let src = ss.world(pose);
                    let mut turned = ts.rest();
                    let mut world: Vec<Quat> = Vec::with_capacity(ts.len());
                    for (b, bone) in ts.bones().iter().enumerate() {
                        let up = bone.parent.map_or(Quat::IDENTITY, |p| world[p]);
                        let w = match pairs.iter().find(|p| p.0 == b) {
                            Some(&(_, sb)) => {
                                let delta = src[sb].rotation * src_rest[sb].rotation.inverse();
                                let w = (turn * delta * turn.inverse() * tgt_rest[b].rotation).normalize();
                                turned[b].rotation = (up.inverse() * w).normalize();
                                w
                            }
                            None => up * bone.rest.rotation,
                        };
                        world.push(w);
                    }
                    let got = [
                        angles(t, &ft, &played[0].poses[f]),
                        angles(t, &ft, &played[1].poses[f]),
                        angles(t, &ft, &turned),
                        angles(t, &ft, &copied),
                    ];
                    let poses = [&played[0].poses[f], &played[1].poses[f], &turned, &copied];
                    if let Some(w) = looks(s, &fs, pose) {
                        for (m, p) in poses.iter().enumerate() {
                            if let Some(g) = looks(t, &ft, p) {
                                gaze[m].push(w.dot(g).clamp(-1.0, 1.0).acos().to_degrees());
                            }
                        }
                    }
                    for (m, g) in got.iter().enumerate() {
                        for (k, (w, g)) in want.iter().zip(g).enumerate() {
                            if let (Some(w), Some(g)) = (w, g) {
                                let a = w.dot(*g).clamp(-1.0, 1.0).acos().to_degrees();
                                pair[m].push(a);
                                by_segment[k][m].push(a);
                            }
                        }
                    }
                }
            }
            for m in 0..4 {
                errs[m].extend(&pair[m]);
            }
            per.insert(
                (
                    rigs.iter().position(|r| std::ptr::eq(r, s)).unwrap(),
                    rigs.iter().position(|r| std::ptr::eq(r, t)).unwrap(),
                ),
                pair,
            );
        }
    }
    let stats = |v: &[f32]| {
        let mut v = v.to_vec();
        v.sort_by(f32::total_cmp);
        let mean = v.iter().sum::<f32>() / v.len().max(1) as f32;
        let q = |p: f32| v.get(((v.len() as f32 - 1.0) * p) as usize).copied().unwrap_or(0.0);
        let within = 100.0 * v.iter().filter(|&&a| a <= 15.0).count() as f32 / v.len().max(1) as f32;
        (mean, q(0.5), q(0.9), within)
    };
    println!(
        "
{clips} clips played onto other rigs, {} limb directions compared",
        errs[0].len()
    );
    println!(
        "| Retarget | mean | median | 90th percentile | within 15° |
| --- | ---: | ---: | ---: | ---: |"
    );
    for (m, name) in methods.iter().enumerate() {
        let (mean, med, p90, within) = stats(&errs[m]);
        println!("| {name} | {mean:.1}° | {med:.1}° | {p90:.1}° | {within:.0}% |");
    }
    println!(
        "
| Segment | dittomo | labeled | world turns | local copied |
| --- | ---: | ---: | ---: | ---: |"
    );
    for (k, (a, b)) in SEGMENTS.iter().enumerate() {
        let m: Vec<String> = by_segment[k].iter().map(|v| format!("{:.1}°", stats(v).0)).collect();
        println!("| {a} → {b} | {} |", m.join(" | "));
    }
    let g: Vec<String> = gaze.iter().map(|v| format!("{:.1}°", stats(v).0)).collect();
    println!("| gaze (head's forward) | {} |", g.join(" | "));
    if show {
        println!(
            "
Per pair, mean degrees (dittomo / labeled / world turns / local copied):"
        );
        for ((s, t), pair) in &per {
            let m: Vec<String> = pair.iter().map(|v| format!("{:.1}", stats(v).0)).collect();
            println!("- {} → {}: {}", rigs[*s].name, rigs[*t].name, m.join(" / "));
        }
    }
    stats(&errs[0]).0
}

/// The most frames of a clip played for foot contact, from its start, at its own rate.
const CONTACT_FRAMES: usize = 300;

/// A rig's height at rest, top bone to bottom.
fn height(s: &Skeleton) -> f32 {
    let ys: Vec<f32> = s.world(&s.rest()).iter().map(|p| p.position().y).collect();
    ys.iter().copied().fold(f32::MIN, f32::max) - ys.iter().copied().fold(f32::MAX, f32::min)
}

/// Every clip played on every other rig with each fix alone, all of them and none. While a source foot stays put
/// (within 5% of its height of the lowest it gets, moving under a tenth of its height a second, once a walk in place
/// is given the travel its feet imply by `contact::travel`): how far the target's foot slides each frame, and how far
/// its sole is from the height it should be at, both over the target's height. The sole is whichever of the foot and
/// toe joints is lowest against where the source's are, each counted from where it stands at rest and the source's
/// scaled: a foot flat stands on its ankle's height, one rolled onto its toes on its toes', and ankles stand higher
/// over the toes on some rigs than on others. And what each fix costs in limb directions, and how often it makes a
/// joint pop: of the frames of each joint both rigs label, the share where the joint, about the hips and over height,
/// is further from where it would be moving steadily from the frame before to the frame after than the source's by over
/// 1% of the height, as a fix taking hold or letting go in a frame jerks the joints it moves.
fn contact(rigs: &[Rig]) {
    const RUNS: [(&str, Fixes); 6] = [
        ("none (`apply`)", Fixes::NONE),
        (
            "travel",
            Fixes {
                travel: true,
                ..Fixes::NONE
            },
        ),
        (
            "planted feet",
            Fixes {
                feet: true,
                ..Fixes::NONE
            },
        ),
        (
            "push-out",
            Fixes {
                push_out: true,
                ..Fixes::NONE
            },
        ),
        ("all (`apply_with`)", Fixes::ALL),
        (
            "all and place",
            Fixes {
                place: true,
                ..Fixes::ALL
            },
        ),
    ];
    // And every fix made frame by frame, from the poses so far, then with a few poses ahead in sight.
    const LATE: [usize; 2] = [0, 6];
    const ROWS: usize = RUNS.len() + LATE.len() + 1;
    let late = ["", ", 6 poses late"].map(|l| format!("all, live{l} (`Live`)"));
    let names: Vec<&str> = RUNS
        .iter()
        .map(|r| r.0)
        .chain(late.iter().map(String::as_str))
        .chain(["all and place, live"])
        .collect();
    let feet = ["l_foot", "r_foot"];
    let (mut slid, mut off, mut planted) = ([0.0f64; ROWS], [0.0f64; ROWS], 0usize);
    let mut errs: [Vec<f32>; ROWS] = Default::default();
    let (mut pops, mut joint_frames) = ([0usize; ROWS], 0usize);
    for s in rigs {
        let (Some(fs), true) = (body(s), feet.iter().all(|f| s.labels.contains_key(*f))) else {
            continue;
        };
        let (ss, hs) = (&s.model.skeleton, height(&s.model.skeleton));
        let s_rest = ss.world(&ss.rest());
        for t in rigs {
            if std::ptr::eq(s, t) || !feet.iter().all(|f| t.labels.contains_key(*f)) {
                continue;
            }
            let Some(ft) = body(t) else { continue };
            let (ts, ht) = (&t.model.skeleton, height(&t.model.skeleton));
            let t_rest = ts.world(&ts.rest());
            let map = Mapping::new(ss, ts);
            for (_, clip) in &s.model.clips {
                let n = clip.poses.len().min(CONTACT_FRAMES);
                if n < 3 {
                    continue;
                }
                let clip = Clip {
                    times: clip.times[..n].to_vec(),
                    poses: clip.poses[..n].to_vec(),
                };
                // A walk in place slides its planted feet back; given its travel, they stay put, as they should.
                let src: Vec<_> = dittomo::contact::travel(ss, &clip)
                    .poses
                    .iter()
                    .map(|p| ss.world(p))
                    .collect();
                let mut played: Vec<Clip> = RUNS
                    .iter()
                    .map(|(_, fixes)| map.apply_with(ss, ts, &clip, *fixes).expect("plays"))
                    .collect();
                played.extend(LATE.map(|d| live(&map, ss, ts, &clip, d)));
                let (none, placing) = (Surface::default(), RUNS[RUNS.len() - 1].1);
                played.push(live_touching(&map, [ss, ts], [&none, &none], &clip, 0, placing));
                let tgt: Vec<Vec<_>> = played
                    .iter()
                    .map(|c| c.poses.iter().map(|p| ts.world(p)).collect())
                    .collect();
                for foot in feet {
                    let (a, b) = (s.labels[foot], t.labels[foot]);
                    let toe = foot.replace("foot", "toe");
                    let sole: Vec<(usize, usize)> = std::iter::once((a, b))
                        .chain(s.labels.get(&toe).zip(t.labels.get(&toe)).map(|(&a, &b)| (a, b)))
                        .collect();
                    // A clip whose foot never comes within a tenth of the height of where it stands at rest has it off the
                    // ground throughout, as in a fall, held still at the lowest it gets: nothing is planted.
                    let ground = src.iter().map(|w| w[a].position().y).fold(f32::MAX, f32::min);
                    let aloft = ground > s_rest[a].position().y + 0.1 * hs;
                    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
                    let still = |f: usize| {
                        let dt = clip.times[f + 1] - clip.times[f];
                        let p = src[f][a].position();
                        dt > 0.0
                            && !aloft
                            && p.y - ground < 0.05 * hs
                            && flat(src[f + 1][a].position() - p).length() / dt < 0.1 * hs
                    };
                    for f in (0..n - 1).filter(|&f| still(f)) {
                        planted += 1;
                        for m in 0..ROWS {
                            let d = flat(tgt[m][f + 1][b].position() - tgt[m][f][b].position()).length();
                            slid[m] += f64::from(d / ht);
                            let over = sole
                                .iter()
                                .map(|&(a, b)| {
                                    let want = (src[f][a].position().y - s_rest[a].position().y) * ht / hs;
                                    tgt[m][f][b].position().y - t_rest[b].position().y - want
                                })
                                .fold(f32::MAX, f32::min);
                            off[m] += f64::from(over.abs() / ht);
                        }
                    }
                }
                if let (Some(&sh), Some(&th)) = (s.labels.get("hips"), t.labels.get("hips")) {
                    let joints: Vec<(usize, usize)> = s
                        .labels
                        .iter()
                        .filter_map(|(k, &a)| Some((a, *t.labels.get(k)?)))
                        .collect();
                    let raw: Vec<_> = clip.poses[..n].iter().map(|p| ss.world(p)).collect();
                    let times = &clip.times;
                    let kink = |w: &[Vec<dittomo::Placed>], f: usize, j: usize, hip: usize, h: f32| {
                        let at = |f: usize| (w[f][j].position() - w[f][hip].position()) / h;
                        let (d0, d1) = (times[f] - times[f - 1], times[f + 1] - times[f]);
                        at(f).distance(at(f - 1).lerp(at(f + 1), d0 / (d0 + d1)))
                    };
                    for f in (1..n - 1).filter(|&f| times[f - 1] < times[f] && times[f] < times[f + 1]) {
                        for &(a, b) in &joints {
                            let want = kink(&raw, f, a, sh, hs);
                            joint_frames += 1;
                            for (m, w) in tgt.iter().enumerate() {
                                let pop = kink(w, f, b, th, ht) - want > 0.01;
                                pops[m] += usize::from(pop);
                            }
                        }
                    }
                }
                for (f, pose) in clip.poses.iter().enumerate() {
                    let want = angles(s, &fs, pose);
                    for (m, c) in played.iter().enumerate() {
                        let got = angles(t, &ft, &c.poses[f]);
                        for (w, g) in want.iter().zip(&got) {
                            if let (Some(w), Some(g)) = (w, g) {
                                errs[m].push(w.dot(*g).clamp(-1.0, 1.0).acos().to_degrees());
                            }
                        }
                    }
                }
            }
        }
    }
    let mean = |v: &[f32]| v.iter().sum::<f32>() / v.len().max(1) as f32;
    println!(
        "
{planted} frames with a source foot planted, at each clip's own rate (its first {CONTACT_FRAMES} frames); slide and
height off are per planted frame, over the target's height"
    );
    println!(
        "| Fixes | Foot slide | Sole height off | Limb directions, mean | Pops |
| --- | ---: | ---: | ---: | ---: |"
    );
    for (m, name) in names.iter().enumerate() {
        println!(
            "| {name} | {:.5} | {:.4} | {:.1}° | {:.2}% |",
            slid[m] / planted.max(1) as f64,
            off[m] / planted.max(1) as f64,
            mean(&errs[m]),
            100.0 * pops[m] as f64 / joint_frames.max(1) as f64
        );
    }
}

/// The clip played a pose at a time by a [`Live`] making every fix it makes, `delay` poses late. The last pose is played
/// again until every pose has come out.
fn live(map: &Mapping, source: &Skeleton, target: &Skeleton, clip: &Clip, delay: usize) -> Clip {
    let empty = Surface::default();
    live_touching(map, [source, target], [&empty, &empty], clip, delay, Fixes::ALL)
}

/// [`live`], knowing each rig's skin.
fn live_touching(
    map: &Mapping,
    [source, target]: [&Skeleton; 2],
    skins: [&Surface; 2],
    clip: &Clip,
    delay: usize,
    fixes: Fixes,
) -> Clip {
    let mut live = Live::touching(map, source, target, skins, fixes)
        .expect("fits")
        .delayed(delay);
    live.moving_in(clip).expect("poses the source");
    let mut out = target.rest();
    let n = clip.poses.len();
    let dt = |f: usize| {
        if f == 0 {
            0.0
        } else {
            clip.times[f.min(n - 1)] - clip.times[f.min(n - 1) - 1]
        }
    };
    let poses = (0..n + delay)
        .map(|f| {
            let dt = if f < n { dt(f) } else { dt(n - 1) };
            live.pose(dt, &clip.poses[f.min(n - 1)], &mut out).expect("plays");
            out.clone()
        })
        .skip(delay)
        .collect();
    Clip {
        times: clip.times.clone(),
        poses,
    }
}

/// Every clip played on every other rig with a skin, with all fixes but touch and with all of them, the whole clip at
/// once and live. While a source hand
/// is near its body (its gap to the skin under 3% of its height): how far the target's gap is from the source's, both
/// over their heights. And over every frame: how often a target hand is in its body by more than 1% of its height when
/// the source's isn't.
fn touching(rigs: &[Rig]) {
    const RUNS: [(&str, Fixes); 3] = [
        (
            "all but touch",
            Fixes {
                touch: false,
                ..Fixes::ALL
            },
        ),
        ("all (`apply_touching`)", Fixes::ALL),
        ("all, live (`Live::touching`)", Fixes::ALL),
    ];
    let joints = ["l_hand", "r_hand", "l_upperarm", "r_upperarm"];
    let ready = |r: &Rig| !r.model.skin.is_empty() && joints.iter().all(|j| r.labels.contains_key(*j));
    let (mut near, mut frames, mut pairs) = (0usize, 0usize, 0usize);
    let mut off: [Vec<f32>; RUNS.len()] = Default::default();
    let mut inside = [0usize; RUNS.len()];
    for s in rigs.iter().filter(|r| ready(r) && !r.model.clips.is_empty()) {
        let (ss, hs) = (&s.model.skeleton, height(&s.model.skeleton));
        let s_away = [s.labels["l_upperarm"], s.labels["r_upperarm"]];
        for t in rigs.iter().filter(|r| ready(r)) {
            if std::ptr::eq(s, t) {
                continue;
            }
            pairs += 1;
            let (ts, ht) = (&t.model.skeleton, height(&t.model.skeleton));
            let t_away = [t.labels["l_upperarm"], t.labels["r_upperarm"]];
            let map = Mapping::new(ss, ts);
            for (_, clip) in &s.model.clips {
                let step = clip.poses.len().div_ceil(FRAMES).max(1);
                let clip = Clip {
                    times: clip.times.iter().step_by(step).copied().collect(),
                    poses: clip.poses.iter().step_by(step).cloned().collect(),
                };
                let skins = [&s.model.skin, &t.model.skin];
                let mut played: Vec<Clip> = RUNS[..2]
                    .iter()
                    .map(|(_, fixes)| map.apply_touching(ss, ts, skins, &clip, *fixes).expect("plays"))
                    .collect();
                played.push(live_touching(&map, [ss, ts], skins, &clip, 0, Fixes::ALL));
                for (f, pose) in clip.poses.iter().enumerate() {
                    let sw = ss.world(pose);
                    for hand in ["l_hand", "r_hand"] {
                        let Some(gs) = s.model.skin.gap(ss, &sw, s.labels[hand], &s_away, s.labels["hips"]) else {
                            continue;
                        };
                        let gs = gs / hs;
                        frames += 1;
                        let close = gs < 0.03;
                        near += usize::from(close);
                        for (m, c) in played.iter().enumerate() {
                            let tw = ts.world(&c.poses[f]);
                            let Some(gt) = t.model.skin.gap(ts, &tw, t.labels[hand], &t_away, t.labels["hips"]) else {
                                continue;
                            };
                            let gt = gt / ht;
                            if close {
                                off[m].push((gt - gs).abs());
                            }
                            inside[m] += usize::from(gt < -0.01 && gs >= -0.01);
                        }
                    }
                }
            }
        }
    }
    println!(
        "
{pairs} pairs with skins; {frames} hand frames, {near} with the source hand within 3% of its height of its body; gaps
over each one's height"
    );
    println!(
        "| Fixes | Hand gap off, source hand near (mean / p90) | Target hand in its body, source's not |
| --- | ---: | ---: |"
    );
    for (m, (name, _)) in RUNS.iter().enumerate() {
        let mut v = off[m].clone();
        v.sort_by(f32::total_cmp);
        let mean = v.iter().sum::<f32>() / v.len().max(1) as f32;
        let p90 = v.get(v.len() * 9 / 10).copied().unwrap_or(0.0);
        println!(
            "| {name} | {mean:.4} / {p90:.4} | {:.1}% |",
            100.0 * inside[m] as f64 / frames.max(1) as f64
        );
    }
}

/// The labeled joints whose skin a hand can be put on, and the group each counts in. A point of skin belongs to the
/// nearest of these at or above its bone.
const PARTS: [(&str, usize); 17] = [
    ("hips", 0),
    ("spine", 0),
    ("chest", 0),
    ("l_clavicle", 0),
    ("r_clavicle", 0),
    ("neck", 1),
    ("head", 1),
    ("l_thigh", 2),
    ("r_thigh", 2),
    ("l_shin", 3),
    ("r_shin", 3),
    ("l_foot", 3),
    ("r_foot", 3),
    ("l_toe", 3),
    ("r_toe", 3),
    ("l_upperarm", 4),
    ("r_upperarm", 4),
];
const GROUPS: [&str; 4] = ["trunk", "head and neck", "thighs", "shins"];

/// The labeled bones a hand's place is told by, each from one labeled joint to the next, and the group it counts in.
const LINES: [(&str, &str, usize); 8] = [
    ("hips", "spine", 0),
    ("spine", "chest", 0),
    ("chest", "neck", 0),
    ("neck", "head", 1),
    ("l_thigh", "l_shin", 2),
    ("r_thigh", "r_shin", 2),
    ("l_shin", "l_foot", 3),
    ("r_shin", "r_foot", 3),
];

/// A contact atlas: on each rig with a skin, standing at rest, each hand is put by two-bone IK on points all over the
/// skin of its body (not its arms), touching and a little off, wherever it reaches; each such pose is then played on
/// every other rig with a skin, bones only (`apply`, `apply_with`) and knowing the skins (`apply_touching`). Over the
/// poses with the source's hand within 5% of its height of its skin: how often the target's wrist is nearest skin of
/// the same part of the body, how far its hand's gap to its skin is from the source's (over each one's height), and
/// how often it is in its body by over 1% of its height.
fn atlas(rigs: &[Rig]) {
    const RUNS: [&str; 5] = [
        "none (`apply`)",
        "all, bones only (`apply_with`)",
        "all (`apply_touching`)",
        "all and place, bones only",
        "all and place",
    ];
    const SPOTS: usize = 60;
    let joints = ["l_hand", "r_hand", "l_upperarm", "r_upperarm", "hips"];
    let ready = |r: &Rig| {
        !r.model.skin.is_empty()
            && joints.iter().all(|j| r.labels.contains_key(*j))
            && LINES
                .iter()
                .all(|(a, b, _)| r.labels.contains_key(*a) && r.labels.contains_key(*b))
    };
    // How far along a labeled bone a point is (0 at its first joint, 1 at its second), and how far from it.
    let along = |w: &[dittomo::Placed], r: &Rig, p: Vec3, k: usize| {
        let (a, b) = (w[r.labels[LINES[k].0]].position(), w[r.labels[LINES[k].1]].position());
        let f = (p - a).dot(b - a) / (b - a).length_squared().max(1e-12);
        (f, p.distance(a + (b - a) * f.clamp(0.0, 1.0)))
    };
    let line_at = |w: &[dittomo::Placed], r: &Rig, p: Vec3| {
        (0..LINES.len())
            .map(|k| (k, along(w, r, p, k)))
            .min_by(|x, y| x.1.1.total_cmp(&y.1.1))
            .map(|(k, (f, _))| (k, f))
            .expect("lines")
    };
    // Each bone's part: the labeled joint at or above it, if any.
    let parts = |r: &Rig| -> Vec<Option<usize>> {
        let sk = &r.model.skeleton;
        (0..sk.len())
            .map(|b| {
                let mut at = Some(b);
                while let Some(x) = at {
                    if let Some(k) = PARTS.iter().position(|(name, _)| r.labels.get(*name) == Some(&x)) {
                        return Some(k);
                    }
                    at = sk.bones()[x].parent;
                }
                None
            })
            .collect()
    };
    let placing = Fixes {
        place: true,
        ..Fixes::ALL
    };
    let (mut pairs, mut poses) = (0usize, [0usize; GROUPS.len()]);
    let mut same = [[0usize; GROUPS.len()]; RUNS.len()];
    let mut inside = [[0usize; GROUPS.len()]; RUNS.len()];
    let mut slid = [[0.0f32; GROUPS.len()]; RUNS.len()];
    let mut off: [[Vec<f32>; GROUPS.len()]; RUNS.len()] = Default::default();
    for s in rigs.iter().filter(|r| ready(r)) {
        let (ss, hs) = (&s.model.skeleton, height(&s.model.skeleton));
        let s_away = [s.labels["l_upperarm"], s.labels["r_upperarm"]];
        let (rest, s_part) = (ss.rest(), parts(s));
        let s_points = s.model.skin.points(&ss.world(&rest));
        let spots: Vec<(Vec3, Vec3)> = s_points
            .iter()
            .filter(|&&(_, _, b)| s_part[b].is_some_and(|k| PARTS[k].1 < GROUPS.len()))
            .map(|&(p, n, _)| (p, n))
            .collect();
        let step = spots.len().div_ceil(SPOTS).max(1);
        // Each pose with a hand by the body: the pose, the hand, its gap and the part it is by.
        let mut samples = Vec::new();
        for limb in dittomo::contact::arms(ss) {
            let Some(hand) = ["l_hand", "r_hand"].into_iter().find(|h| s.labels[*h] == limb.end) else {
                continue;
            };
            for &(p, n) in spots.iter().step_by(step) {
                for out in [0.03, 0.06] {
                    let mut pose = rest.clone();
                    dittomo::contact::reach(ss, &mut pose, &limb, p + n * out * hs);
                    let sw = ss.world(&pose);
                    let Some(gap) = s.model.skin.gap(ss, &sw, limb.end, &s_away, s.labels["hips"]) else {
                        continue;
                    };
                    let part = line_at(&sw, s, sw[limb.end].position());
                    if (-0.01..0.05).contains(&(gap / hs)) {
                        samples.push((pose, hand, gap / hs, part));
                    }
                }
            }
        }
        for t in rigs.iter().filter(|r| ready(r)) {
            if std::ptr::eq(s, t) != std::env::var_os("ATLAS_SELF").is_some() {
                continue;
            }
            if let Ok(only) = std::env::var("ATLAS_ONLY")
                && only != format!("{},{}", s.name, t.name)
            {
                continue;
            }
            pairs += 1;
            let (ts, ht) = (&t.model.skeleton, height(&t.model.skeleton));
            let t_away = [t.labels["l_upperarm"], t.labels["r_upperarm"]];
            let map = Mapping::new(ss, ts);
            let skins = [&s.model.skin, &t.model.skin];
            let mut tally = [(0usize, 0.0f32, 0usize); RUNS.len()];
            for (pose, hand, gs, part) in &samples {
                let clip = Clip {
                    times: vec![0.0],
                    poses: vec![pose.clone()],
                };
                let played = [
                    map.apply(ss, ts, &clip).expect("plays"),
                    map.apply_with(ss, ts, &clip, Fixes::ALL).expect("plays"),
                    map.apply_touching(ss, ts, skins, &clip, Fixes::ALL).expect("plays"),
                    map.apply_with(ss, ts, &clip, placing).expect("plays"),
                    map.apply_touching(ss, ts, skins, &clip, placing).expect("plays"),
                ];
                let g = LINES[part.0].2;
                poses[g] += 1;
                for (m, c) in played.iter().enumerate() {
                    let tw = ts.world(&c.poses[0]);
                    let Some(gt) = t.model.skin.gap(ts, &tw, t.labels[*hand], &t_away, t.labels["hips"]) else {
                        continue;
                    };
                    let gt = gt / ht;
                    off[m][g].push((gt - gs).abs());
                    inside[m][g] += usize::from(gt < -0.01);
                    let wrist = tw[t.labels[*hand]].position();
                    let at = line_at(&tw, t, wrist).0 == part.0;
                    same[m][g] += usize::from(at);
                    tally[m].0 += usize::from(at);
                    slid[m][g] += (along(&tw, t, wrist, part.0).0 - part.1).abs();
                    tally[m].1 += (gt - gs).abs();
                    tally[m].2 += usize::from(gt < -0.01);
                }
            }
            if std::env::var_os("ATLAS_PAIRS").is_some() {
                let n = samples.len().max(1) as f32;
                let cell = |m: usize| {
                    format!(
                        "{:.0}% {:.4} {:.0}%",
                        100.0 * tally[m].0 as f32 / n,
                        tally[m].1 / n,
                        100.0 * tally[m].2 as f32 / n
                    )
                };
                eprintln!(
                    "{} -> {} ({}): bones {} | place {} || touch {} | place {}",
                    s.name,
                    t.name,
                    samples.len(),
                    cell(1),
                    cell(3),
                    cell(2),
                    cell(4)
                );
            }
        }
    }
    println!(
        "
A contact atlas: {pairs} pairs with skins, a hand put by IK on the skin of the source standing at rest. Poses with the
hand by the {}: {}. Each cell: the target's wrist nearest the same labeled bone, and how far along that bone it is off
(as a share of the bone); its hand's gap to its skin off the source's (mean / p90, over each one's height); in its body",
        GROUPS.join(", "),
        poses.map(|n| n.to_string()).join(", ")
    );
    println!(
        "| Fixes | {} |
| --- | {}",
        GROUPS.join(" | "),
        "---: | ".repeat(GROUPS.len())
    );
    for (m, name) in RUNS.iter().enumerate() {
        let cells: Vec<String> = (0..GROUPS.len())
            .map(|g| {
                let mut v = off[m][g].clone();
                v.sort_by(f32::total_cmp);
                let mean = v.iter().sum::<f32>() / v.len().max(1) as f32;
                let n = poses[g].max(1) as f64;
                format!(
                    "{:.0}%, {:.2}; {mean:.4} / {:.4}; {:.1}%",
                    100.0 * same[m][g] as f64 / n,
                    f64::from(slid[m][g]) / n,
                    v.get(v.len() * 9 / 10).copied().unwrap_or(0.0),
                    100.0 * inside[m][g] as f64 / n
                )
            })
            .collect();
        println!("| {name} | {} |", cells.join(" | "));
    }
}

/// The atlas for hands meeting: on each rig with a skin, one hand is held before the chest by two-bone IK, and the
/// other put on points all over the skin of that forearm and hand, touching and a little off; each such pose is then
/// played on every other rig with a skin. Over the poses with the source's hand within 5% of its height of skin not its
/// own arm's: how far along the held forearm (from its elbow to its wrist, past 1 on the hand) the target's wrist is
/// off, how far its hand's gap to that skin is from the source's, how often it is in it by over 1% of its height, and
/// how far the wrists' distance is off (the last three over each one's height).
fn meeting(rigs: &[Rig]) {
    const RUNS: [&str; 5] = [
        "none (`apply`)",
        "all, bones only (`apply_with`)",
        "all (`apply_touching`)",
        "all and place, bones only",
        "all and place",
    ];
    const SPOTS: usize = 40;
    const SIDES: [[&str; 3]; 2] = [
        ["l_upperarm", "l_forearm", "l_hand"],
        ["r_upperarm", "r_forearm", "r_hand"],
    ];
    let ready = |r: &Rig| {
        !r.model.skin.is_empty()
            && r.labels.contains_key("hips")
            && SIDES.iter().flatten().all(|j| r.labels.contains_key(*j))
            && body(r).is_some()
    };
    // How far along the held forearm the other wrist is, and how far apart the wrists are.
    let seen = |w: &[dittomo::Placed], r: &Rig, held: usize| {
        let at = |j: &str| w[r.labels[j]].position();
        let (a, b, p) = (at(SIDES[held][1]), at(SIDES[held][2]), at(SIDES[1 - held][2]));
        ((p - a).dot(b - a) / (b - a).length_squared().max(1e-12), p.distance(b))
    };
    let placing = Fixes {
        place: true,
        ..Fixes::ALL
    };
    let (mut pairs, mut poses) = (0usize, 0usize);
    let mut inside = [0usize; RUNS.len()];
    let mut sums = [[0.0f64; 3]; RUNS.len()];
    let mut off: [Vec<f32>; RUNS.len()] = Default::default();
    for s in rigs.iter().filter(|r| ready(r)) {
        let (ss, hs) = (&s.model.skeleton, height(&s.model.skeleton));
        let Some([_, up, forward]) = body(s) else { continue };
        let rest = ss.rest();
        let at_rest = ss.world(&rest);
        let limbs = dittomo::contact::arms(ss);
        let limb = |side: usize| limbs.iter().find(|l| l.end == s.labels[SIDES[side][2]]);
        // Each pose with a hand by the other arm: the pose, which arm is held, its gap, and what is seen of it.
        let mut samples = Vec::new();
        for held in 0..2 {
            let (Some(hold), Some(mover)) = (limb(held), limb(1 - held)) else {
                continue;
            };
            let pos = |b: usize| at_rest[b].position();
            let long = pos(hold.upper).distance(pos(hold.lower)) + pos(hold.lower).distance(pos(hold.end));
            let chest = (pos(s.labels[SIDES[0][0]]) + pos(s.labels[SIDES[1][0]])) / 2.0;
            let mut base = rest.clone();
            dittomo::contact::reach(ss, &mut base, hold, chest + (forward * 0.45 - up * 0.25) * long);
            let based = ss.world(&base);
            let fore = s.labels[SIDES[held][1]];
            let on_fore = |mut b: usize| {
                loop {
                    if b == fore {
                        return true;
                    }
                    match ss.bones()[b].parent {
                        Some(p) => b = p,
                        None => return false,
                    }
                }
            };
            let spots: Vec<(Vec3, Vec3)> = s
                .model
                .skin
                .points(&based)
                .into_iter()
                .filter(|&(_, _, b)| on_fore(b))
                .map(|(p, n, _)| (p, n))
                .collect();
            let step = spots.len().div_ceil(SPOTS).max(1);
            let away = [s.labels[SIDES[1 - held][0]]];
            for &(p, n) in spots.iter().step_by(step) {
                for out in [0.03, 0.06] {
                    let mut pose = base.clone();
                    dittomo::contact::reach(ss, &mut pose, mover, p + n * out * hs);
                    let sw = ss.world(&pose);
                    let Some(gap) = s.model.skin.gap(ss, &sw, mover.end, &away, s.labels["hips"]) else {
                        continue;
                    };
                    if (-0.01..0.05).contains(&(gap / hs)) {
                        let (along, apart) = seen(&sw, s, held);
                        samples.push((pose, held, gap / hs, along, apart / hs));
                    }
                }
            }
        }
        for t in rigs.iter().filter(|r| ready(r)) {
            if std::ptr::eq(s, t) != std::env::var_os("ATLAS_SELF").is_some() {
                continue;
            }
            if let Ok(only) = std::env::var("ATLAS_ONLY")
                && only != format!("{},{}", s.name, t.name)
            {
                continue;
            }
            pairs += 1;
            let (ts, ht) = (&t.model.skeleton, height(&t.model.skeleton));
            let map = Mapping::new(ss, ts);
            let skins = [&s.model.skin, &t.model.skin];
            let mut tally = [(0.0f32, 0usize, 0.0f32); RUNS.len()];
            for (pose, held, gs, along, apart) in &samples {
                let clip = Clip {
                    times: vec![0.0],
                    poses: vec![pose.clone()],
                };
                let played = [
                    map.apply(ss, ts, &clip).expect("plays"),
                    map.apply_with(ss, ts, &clip, Fixes::ALL).expect("plays"),
                    map.apply_touching(ss, ts, skins, &clip, Fixes::ALL).expect("plays"),
                    map.apply_with(ss, ts, &clip, placing).expect("plays"),
                    map.apply_touching(ss, ts, skins, &clip, placing).expect("plays"),
                ];
                poses += 1;
                let away = [t.labels[SIDES[1 - held][0]]];
                for (m, c) in played.iter().enumerate() {
                    let tw = ts.world(&c.poses[0]);
                    let hand = t.labels[SIDES[1 - held][2]];
                    let Some(gt) = t.model.skin.gap(ts, &tw, hand, &away, t.labels["hips"]) else {
                        continue;
                    };
                    let gt = gt / ht;
                    let (a, d) = seen(&tw, t, *held);
                    off[m].push((gt - gs).abs());
                    inside[m] += usize::from(gt < -0.01);
                    sums[m][0] += f64::from((a - along).abs());
                    sums[m][1] += f64::from((d / ht - apart).abs());
                    sums[m][2] += f64::from((gt - gs).abs());
                    tally[m].0 += (gt - gs).abs();
                    tally[m].1 += usize::from(gt < -0.01);
                    tally[m].2 += (d / ht - apart).abs();
                }
            }
            if std::env::var_os("ATLAS_PAIRS").is_some() {
                let n = samples.len().max(1) as f32;
                let cell = |m: usize| {
                    format!(
                        "{:.4} {:.0}% {:.4}",
                        tally[m].0 / n,
                        100.0 * tally[m].1 as f32 / n,
                        tally[m].2 / n
                    )
                };
                eprintln!(
                    "meeting {} -> {} ({}): bones {} | place {} || touch {} | place {}",
                    s.name,
                    t.name,
                    samples.len(),
                    cell(1),
                    cell(3),
                    cell(2),
                    cell(4)
                );
            }
        }
    }
    println!(
        "
Hands meeting: {pairs} pairs with skins, one hand held before the chest and the other put by IK on the skin of that
forearm and hand: {poses} poses. Each cell: how far along the held forearm the target's wrist is off (as a share of the
forearm); its hand's gap to skin not its own arm's off the source's (mean / p90); in that skin; the wrists' distance
off (the last three over each one's height)
| Fixes | hand on the other arm |
| --- | ---: |"
    );
    for (m, name) in RUNS.iter().enumerate() {
        let mut v = off[m].clone();
        v.sort_by(f32::total_cmp);
        let n = poses.max(1) as f64;
        println!(
            "| {name} | {:.2}; {:.4} / {:.4}; {:.1}%; {:.4} |",
            sums[m][0] / n,
            sums[m][2] / n,
            v.get(v.len() * 9 / 10).copied().unwrap_or(0.0),
            100.0 * inside[m] as f64 / n,
            sums[m][1] / n
        );
    }
}

/// Motion capture picked for what the hands do (crawling, a cartwheel, getting up off the floor, jumping jacks, combing
/// hair, washing, a chin on a fist, being cold), played on every rig, at 30 poses a second. Three things, each over the
/// frames where the source does it, as shares of each one's height: a wrist low and still (within 6% of the height of
/// the floor, moving under a fifth of the height a second), how far the target's is off that height over its own floor
/// and how far it slides a frame; a wrist by the head (within 20% of the height of the head's joint), how far the
/// target's is off that distance from its own; the wrists together (within 10%), how far the target's are off that.
fn moves(moves: &[Rig], rigs: &[Rig]) {
    let runs: [(&str, Option<Fixes>); 4] = [
        ("all (`apply_with`)", Some(Fixes::ALL)),
        (
            "all and place",
            Some(Fixes {
                place: true,
                ..Fixes::ALL
            }),
        ),
        ("all, live (`Live`)", None),
        ("none (`apply`)", Some(Fixes::NONE)),
    ];
    let joints = ["l_hand", "r_hand", "l_foot", "r_foot", "head", "hips"];
    let ready = |r: &Rig| joints.iter().all(|j| r.labels.contains_key(*j));
    // The floor a clip is played over: the lowest a hand or a foot gets in it.
    let floor = |r: &Rig, w: &[Vec<dittomo::Placed>]| {
        w.iter()
            .flat_map(|w| joints[..4].iter().map(|j| w[r.labels[*j]].position().y))
            .fold(f32::MAX, f32::min)
    };
    println!(
        "
Picked motion capture on every rig. Each cell: a low, still wrist's height off and slide a frame; a wrist by the head,
its distance off; the wrists together, their distance off (all over the height); then the frames each is over"
    );
    println!(
        "| Motion | {} | frames |
| --- | {}---: |",
        runs.map(|r| r.0).join(" | "),
        "---: | ".repeat(runs.len())
    );
    for s in moves.iter().filter(|r| ready(r)) {
        let (ss, hs) = (&s.model.skeleton, height(&s.model.skeleton));
        let Some((_, clip)) = s.model.clips.first() else {
            continue;
        };
        let step = clip
            .times
            .windows(2)
            .map(|w| w[1] - w[0])
            .find(|d| *d > 0.0)
            .map_or(1, |dt| ((1.0 / 30.0) / dt).round().max(1.0) as usize);
        let clip = Clip {
            times: clip.times.iter().step_by(step).copied().collect(),
            poses: clip.poses.iter().step_by(step).cloned().collect(),
        };
        let n = clip.poses.len();
        let sw: Vec<_> = clip.poses.iter().map(|p| ss.world(p)).collect();
        let fs = floor(s, &sw);
        // Height off, slide, by the head, together: sums and counts.
        let mut sums = [[0.0f64; 4]; 4];
        let mut counts = [0usize; 3];
        for t in rigs.iter().filter(|r| ready(r)) {
            let (ts, ht) = (&t.model.skeleton, height(&t.model.skeleton));
            let map = Mapping::new(ss, ts);
            let played: Vec<Vec<Vec<dittomo::Placed>>> = runs
                .iter()
                .map(|(_, fixes)| {
                    let c = match fixes {
                        Some(f) => map.apply_with(ss, ts, &clip, *f).expect("plays"),
                        None => live(&map, ss, ts, &clip, 0),
                    };
                    c.poses.iter().map(|p| ts.world(p)).collect()
                })
                .collect();
            let floors: Vec<f32> = played.iter().map(|w| floor(t, w)).collect();
            for f in 1..n {
                let dt = clip.times[f] - clip.times[f - 1];
                if dt <= 0.0 {
                    continue;
                }
                let at = |w: &[dittomo::Placed], r: &Rig, j: &str| w[r.labels[j]].position();
                for hand in ["l_hand", "r_hand"] {
                    let (p, q) = (at(&sw[f], s, hand), at(&sw[f - 1], s, hand));
                    let low = (p.y - fs) / hs;
                    if low < 0.06 && p.distance(q) / dt / hs < 0.2 {
                        counts[0] += 1;
                        for (m, w) in played.iter().enumerate() {
                            let (a, b) = (at(&w[f], t, hand), at(&w[f - 1], t, hand));
                            sums[m][0] += f64::from(((a.y - floors[m]) / ht - low).abs());
                            sums[m][1] += f64::from(Vec3::new(a.x - b.x, 0.0, a.z - b.z).length() / ht);
                        }
                    }
                    let by = p.distance(at(&sw[f], s, "head")) / hs;
                    if by < 0.2 {
                        counts[1] += 1;
                        for (m, w) in played.iter().enumerate() {
                            let got = at(&w[f], t, hand).distance(at(&w[f], t, "head")) / ht;
                            sums[m][2] += f64::from((got - by).abs());
                        }
                    }
                }
                let gap = at(&sw[f], s, "l_hand").distance(at(&sw[f], s, "r_hand")) / hs;
                if gap < 0.10 {
                    counts[2] += 1;
                    for (m, w) in played.iter().enumerate() {
                        let got = at(&w[f], t, "l_hand").distance(at(&w[f], t, "r_hand")) / ht;
                        sums[m][3] += f64::from((got - gap).abs());
                    }
                }
            }
        }
        let cells: Vec<String> = (0..runs.len())
            .map(|m| {
                let per = |i: usize, c: usize| sums[m][i] / counts[c].max(1) as f64;
                format!("{:.4}, {:.5}; {:.4}; {:.4}", per(0, 0), per(1, 0), per(2, 1), per(3, 2))
            })
            .collect();
        println!(
            "| {} | {} | {}, {}, {} |",
            s.name.trim_start_matches("cmu-").trim_end_matches(".bvh"),
            cells.join(" | "),
            counts[0],
            counts[1],
            counts[2]
        );
    }
}
