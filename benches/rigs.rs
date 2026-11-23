//! How well mapping finds the right bones on real rigs, against matching by name.
//!
//! ```text
//! sh bench/rigs/fetch.sh                      # download the rigs (once)
//! cargo bench --features fs --bench rigs      # print the table
//! cargo bench --features fs --bench rigs -- --dump     # print each rig's tree
//! cargo bench --features fs --bench rigs -- --pairs    # print every pair's misses
//! cargo bench --features fs --bench rigs -- --show xbot cesium   # the target's tree, with what each bone plays
//! cargo bench --features fs --bench rigs -- --min 90   # fail below 90% of joints found (CI)
//! cargo bench --features fs --bench rigs -- --max-degrees 4   # fail if limbs stray more than 4° on average (CI)
//! ```
//!
//! `bench/rigs/labels.txt` names, for each rig, the bone playing each of 23 joints of the body (hips, spine, chest,
//! neck, head, and on each side the clavicle, upper arm, forearm, hand, index finger, thigh, shin, foot and toe),
//! labelled by hand; a rig without one of them leaves it out.
//! Each ordered pair of rigs is mapped, and a joint is found when the target's joint bone plays the source's.
//!
//! Then every clip of every rig is played on every other rig, and each labelled limb segment's direction (upper arm
//! from shoulder to elbow, and so on) is compared with the source's, each in its own body's frame, against two ways
//! of copying a clip onto the right bones without mummer: copying each bone's local rotation, as an engine's
//! animation mixer does when a clip is played on another rig by bone name, and copying each bone's turn from rest in
//! the world.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use glam::Mat3;
use mummer::load::Model;
use mummer::{Clip, Fixes, Mapping, Quat, Skeleton, Transform, Vec3};

const JOINTS: [&str; 23] = [
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
];

struct Rig {
    name: String,
    model: Model,
    /// Each labelled joint's bone.
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
        for (joint, bone) in all_labels.get(&name).into_iter().flatten() {
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
        rigs.push(Rig { name, model, labels });
    }
    if has("--dump") {
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
    let (mut found, mut named, mut asked) = (0usize, 0usize, 0usize);
    let mut per: BTreeMap<&str, (usize, usize, usize)> = BTreeMap::new();
    let mut worst: Vec<(f32, String)> = Vec::new();
    for s in &rigs {
        for t in &rigs {
            if std::ptr::eq(s, t) {
                continue;
            }
            let (ss, ts) = (&s.model.skeleton, &t.model.skeleton);
            let map = Mapping::new(ss, ts);
            let names = by_name(ss, ts);
            let (mut f, mut nm, mut a) = (0, 0, 0);
            let mut misses = Vec::new();
            for joint in JOINTS {
                let (Some(&sb), Some(&tb)) = (s.labels.get(joint), t.labels.get(joint)) else {
                    continue;
                };
                a += 1;
                let e = per.entry(joint).or_default();
                e.2 += 1;
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
            }
            found += f;
            named += nm;
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
        "\n{} rigs, {} ordered pairs, {asked} labelled joints",
        rigs.len(),
        rigs.len() * (rigs.len() - 1)
    );
    println!("| Matcher | joints found |\n| --- | ---: |");
    println!("| mummer | {:.1}% |", pct(found, asked));
    println!("| exact name (three.js, Babylon default) | {:.1}% |", pct(named, asked));
    println!("\n| Joint | mummer | by name |\n| --- | ---: | ---: |");
    for joint in JOINTS {
        if let Some(&(f, n, a)) = per.get(joint) {
            println!("| {joint} | {:.0}% | {:.0}% |", pct(f, a), pct(n, a));
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

/// Limb segments, from one labelled joint to the next, whose directions a retarget should copy.
const SEGMENTS: [(&str, &str); 16] = [
    ("hips", "chest"),
    ("chest", "head"),
    ("l_upperarm", "l_forearm"),
    ("l_forearm", "l_hand"),
    ("l_hand", "l_index"),
    ("r_upperarm", "r_forearm"),
    ("r_forearm", "r_hand"),
    ("r_hand", "r_index"),
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

fn angles(rig: &Rig, frame: &[Vec3; 3], pose: &[Transform]) -> Vec<Option<Vec3>> {
    let world = rig.model.skeleton.world(pose);
    SEGMENTS
        .iter()
        .map(|(a, b)| {
            let (a, b) = (*rig.labels.get(*a)?, *rig.labels.get(*b)?);
            let d = world[b].position() - world[a].position();
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

/// Every clip of every rig that has one played on every other rig: how far, in degrees, each labelled limb segment
/// points from where the source's does, measured in each body's own frame. Gives mummer's mean.
fn motion(rigs: &[Rig], show: bool) -> f32 {
    let methods = [
        "mummer",
        "mummer, labelled bones set by hand",
        "world turns from rest copied onto the labelled bones",
        "local rotations copied onto the labelled bones",
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
                    for (b, p) in map.pairs().iter().enumerate() {
                        if p.is_some_and(|p| p.source == sb) && b != tb {
                            fixed.set(b, None);
                        }
                    }
                    fixed.set(tb, Some(sb));
                }
            }
            // The labelled pairs, and the turn from the source body's frame to the target's.
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
                    // Each labelled bone turned in the world as its source is turned from rest.
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
| Segment | mummer | labelled | world turns | local copied |
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
Per pair, mean degrees (mummer / labelled / world turns / local copied):"
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
/// is given the travel its feet imply by `contact::travel`): how far the target's
/// foot slides each frame, and how far it is from the height it should be at (as far above its rest height as the
/// source's is above its own, scaled), both over the target's height. And what each costs in limb directions.
fn contact(rigs: &[Rig]) {
    const RUNS: [(&str, Fixes); 5] = [
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
    ];
    let feet = ["l_foot", "r_foot"];
    let (mut slid, mut off, mut planted) = ([0.0f64; RUNS.len()], [0.0f64; RUNS.len()], 0usize);
    let mut errs: [Vec<f32>; RUNS.len()] = Default::default();
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
                let src: Vec<_> = mummer::contact::travel(ss, &clip)
                    .poses
                    .iter()
                    .map(|p| ss.world(p))
                    .collect();
                let played: Vec<Clip> = RUNS
                    .iter()
                    .map(|(_, fixes)| map.apply_with(ss, ts, &clip, *fixes).expect("plays"))
                    .collect();
                let tgt: Vec<Vec<_>> = played
                    .iter()
                    .map(|c| c.poses.iter().map(|p| ts.world(p)).collect())
                    .collect();
                for foot in feet {
                    let (a, b) = (s.labels[foot], t.labels[foot]);
                    let ground = src.iter().map(|w| w[a].position().y).fold(f32::MAX, f32::min);
                    let flat = |v: Vec3| Vec3::new(v.x, 0.0, v.z);
                    let still = |f: usize| {
                        let dt = clip.times[f + 1] - clip.times[f];
                        let p = src[f][a].position();
                        dt > 0.0
                            && p.y - ground < 0.05 * hs
                            && flat(src[f + 1][a].position() - p).length() / dt < 0.1 * hs
                    };
                    for f in (0..n - 1).filter(|&f| still(f)) {
                        planted += 1;
                        let want = t_rest[b].position().y + (src[f][a].position().y - s_rest[a].position().y) * ht / hs;
                        for m in 0..RUNS.len() {
                            let d = flat(tgt[m][f + 1][b].position() - tgt[m][f][b].position()).length();
                            slid[m] += f64::from(d / ht);
                            off[m] += f64::from((tgt[m][f][b].position().y - want).abs() / ht);
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
        "| Fixes | Foot slide | Foot height off | Limb directions, mean |
| --- | ---: | ---: | ---: |"
    );
    for (m, (name, _)) in RUNS.iter().enumerate() {
        println!(
            "| {name} | {:.5} | {:.4} | {:.1}° |",
            slid[m] / planted.max(1) as f64,
            off[m] / planted.max(1) as f64,
            mean(&errs[m])
        );
    }
}

/// Every clip played on every other rig with a skin, with all fixes but touch and with all of them. While a source hand
/// is near its body (its gap to the skin under 3% of its height): how far the target's gap is from the source's, both
/// over their heights. And over every frame: how often a target hand is in its body by more than 1% of its height when
/// the source's isn't.
fn touching(rigs: &[Rig]) {
    const RUNS: [(&str, Fixes); 2] = [
        (
            "all but touch",
            Fixes {
                touch: false,
                ..Fixes::ALL
            },
        ),
        ("all (`apply_touching`)", Fixes::ALL),
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
                let played: Vec<Clip> = RUNS
                    .iter()
                    .map(|(_, fixes)| {
                        map.apply_touching(ss, ts, [&s.model.skin, &t.model.skin], &clip, *fixes)
                            .expect("plays")
                    })
                    .collect();
                for (f, pose) in clip.poses.iter().enumerate() {
                    let sw = ss.world(pose);
                    for hand in ["l_hand", "r_hand"] {
                        let Some(gs) = s.model.skin.gap(ss, &sw, s.labels[hand], &s_away) else {
                            continue;
                        };
                        let gs = gs / hs;
                        frames += 1;
                        let close = gs < 0.03;
                        near += usize::from(close);
                        for (m, c) in played.iter().enumerate() {
                            let tw = ts.world(&c.poses[f]);
                            let Some(gt) = t.model.skin.gap(ts, &tw, t.labels[hand], &t_away) else {
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
