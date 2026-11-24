//! How long mapping and playing take on production-sized rigs.
//!
//! ```text
//! cargo bench --bench retarget                       # print the table
//! cargo bench --bench retarget -- --json out.json    # and write the numbers
//! cargo bench --bench retarget -- --against base.json --slack 0.25
//! ```
//!
//! `--against` compares with numbers an earlier run wrote (on the same machine: CI runs the
//! base and the change in one job) and fails when a case got slower by more than `--slack`
//! (a fraction, 0.25 by default).

mod common;

use std::hint::black_box;
use std::time::{Duration, Instant};

use dittomo::{Clip, Crowd, Fixes, Live, Mapping, Player, Skeleton, Surface, Transform, Vec3};

struct Case {
    name: &'static str,
    /// Microseconds for one run, the median of the timed runs.
    micros: f64,
    /// What the time is per: one frame, one mapping.
    per: &'static str,
}

/// The median time of `f`, run as often as fits `budget` (at least five times).
fn time(budget: Duration, mut f: impl FnMut()) -> f64 {
    f();
    let mut samples = Vec::new();
    let start = Instant::now();
    while samples.len() < 5 || start.elapsed() < budget {
        let t = Instant::now();
        f();
        samples.push(t.elapsed().as_secs_f64() * 1e6);
    }
    samples.sort_by(f64::total_cmp);
    samples[samples.len() / 2]
}

fn play(budget: Duration, name: &'static str, source: &Skeleton, target: &Skeleton, clip: &Clip) -> [Case; 4] {
    let map = time(budget, || {
        black_box(Mapping::new(black_box(source), black_box(target)));
    });
    let mapping = Mapping::new(source, target);
    let frames = clip.poses.len() as f64;
    let apply = time(budget, || {
        black_box(mapping.apply(source, target, black_box(clip)).expect("plays"));
    }) / frames;
    let mut player = Player::new(&mapping, source, target).expect("fits");
    player.moving_in(clip).expect("poses the source");
    let mut out = target.rest();
    let pose = time(budget, || {
        for p in &clip.poses {
            player.pose(black_box(p), &mut out).expect("plays");
        }
        black_box(&out);
    }) / frames;
    // A crowd of characters, each at its own frame of the clip.
    const CROWD: usize = 64;
    let mut crowd = Crowd::new(player);
    let n = clip.poses.len();
    let frames_in: Vec<Vec<Transform>> = (0..n)
        .map(|f| {
            (0..CROWD)
                .flat_map(|c| clip.poses[(f + 7 * c) % n].iter().copied())
                .collect()
        })
        .collect();
    let mut outs = vec![Transform::IDENTITY; CROWD * target.len()];
    let crowd_pose = time(budget, || {
        for p in &frames_in {
            crowd.pose(black_box(p), &mut outs).expect("plays");
        }
        black_box(&outs);
    }) / (frames * CROWD as f64);
    [
        Case {
            name: Box::leak(format!("{name}: map").into_boxed_str()),
            micros: map,
            per: "mapping",
        },
        Case {
            name: Box::leak(format!("{name}: play").into_boxed_str()),
            micros: apply,
            per: "frame",
        },
        Case {
            name: Box::leak(format!("{name}: pose").into_boxed_str()),
            micros: pose,
            per: "frame, reused player",
        },
        Case {
            name: Box::leak(format!("{name}: pose a crowd of {CROWD}").into_boxed_str()),
            micros: crowd_pose,
            per: "frame each, reused crowd",
        },
    ]
}

/// The rigs, the clip and the mapping as JSON, for `bench/three` to play the same motion through three.js: each bone's
/// name, parent and rest, each frame's local transforms, and for each target bone its source's name.
fn dump(path: &str, source: &Skeleton, target: &Skeleton, clip: &Clip, mapping: &Mapping) {
    let trs = |t: &Transform| {
        let (p, q, s) = (t.translation, t.rotation, t.scale);
        format!(
            "[{},{},{},{},{},{},{},{},{},{}]",
            p.x, p.y, p.z, q.x, q.y, q.z, q.w, s.x, s.y, s.z
        )
    };
    let bones = |sk: &Skeleton| {
        let list: Vec<String> = sk
            .bones()
            .iter()
            .map(|b| {
                let parent = b.parent.map_or("null".to_string(), |p| p.to_string());
                format!(
                    "{{\"name\":\"{}\",\"parent\":{parent},\"rest\":{}}}",
                    b.name,
                    trs(&b.rest)
                )
            })
            .collect();
        format!("[{}]", list.join(","))
    };
    let frames: Vec<String> = clip
        .poses
        .iter()
        .map(|p| format!("[{}]", p.iter().map(trs).collect::<Vec<_>>().join(",")))
        .collect();
    let times: Vec<String> = clip.times.iter().map(f32::to_string).collect();
    let names: Vec<String> = (0..target.len())
        .filter_map(|b| {
            let a = mapping.source_of(b)?;
            Some(format!("\"{}\":\"{}\"", target.bones()[b].name, source.bones()[a].name))
        })
        .collect();
    let body = format!(
        "{{\"source\":{},\"target\":{},\"times\":[{}],\"frames\":[{}],\"names\":{{{}}}}}
",
        bones(source),
        bones(target),
        times.join(","),
        frames.join(","),
        names.join(",")
    );
    std::fs::write(path, body).expect("writes the rigs");
}

/// A rough skin for a rig: a tube round each bone, from it to each child, a twentieth of the rig's height thick,
/// with a point about every hundredth of the height.
fn skin(sk: &Skeleton) -> Surface {
    let rest = sk.world(&sk.rest());
    let ys = rest.iter().map(|p| p.position().y);
    let height = ys.clone().fold(f32::MIN, f32::max) - ys.fold(f32::MAX, f32::min);
    let (r, spacing) = (height / 40.0, height / 100.0);
    let mut points = Vec::new();
    for (c, bone) in sk.bones().iter().enumerate() {
        let Some(b) = bone.parent else { continue };
        let (a, z) = (rest[b].position(), rest[c].position());
        let Some(along) = (z - a).try_normalize() else { continue };
        let side = along.any_orthonormal_vector();
        let up = along.cross(side);
        let steps = ((z - a).length() / spacing).ceil().max(1.0) as usize;
        let round = (std::f32::consts::TAU * r / spacing).ceil() as usize;
        for i in 0..=steps {
            let at = a.lerp(z, i as f32 / steps as f32);
            for k in 0..round {
                let t = k as f32 / round as f32 * std::f32::consts::TAU;
                let n: Vec3 = side * t.cos() + up * t.sin();
                points.push((at + n * r, n, b));
            }
        }
    }
    Surface::new(sk, points, spacing)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let flag = |f: &str| args.iter().position(|a| a == f).and_then(|i| args.get(i + 1)).cloned();
    let budget = Duration::from_secs_f64(flag("--budget").and_then(|s| s.parse().ok()).unwrap_or(1.0));
    let slack: f64 = flag("--slack").and_then(|s| s.parse().ok()).unwrap_or(0.25);

    let mixamo = common::mixamo();
    let mannequin = common::mannequin();
    let walk = common::walk(&mixamo, 60);
    if let Some(path) = flag("--dump") {
        dump(&path, &mixamo, &mannequin, &walk, &Mapping::new(&mixamo, &mannequin));
        return;
    }
    let mut cases = Vec::new();
    cases.extend(play(budget, "mixamo 65 → itself", &mixamo, &mixamo, &walk));
    cases.extend(play(budget, "mixamo 65 → mannequin 76", &mixamo, &mannequin, &walk));
    let gait = common::gait(&mixamo, 90, false);
    let in_place = common::gait(&mixamo, 90, true);
    let mapping = Mapping::new(&mixamo, &mannequin);
    for (name, clip, fixes) in [("walk", &gait, Fixes::ALL), ("walk in place", &in_place, Fixes::ALL)] {
        let micros = time(budget, || {
            black_box(
                mapping
                    .apply_with(&mixamo, &mannequin, black_box(clip), fixes)
                    .expect("plays"),
            );
        }) / clip.poses.len() as f64;
        cases.push(Case {
            name: Box::leak(format!("mixamo 65 → mannequin 76: {name}, all fixes").into_boxed_str()),
            micros,
            per: "frame",
        });
    }
    // The same walks a pose at a time as they come, every fix a live player makes on, then played six poses late.
    for (name, clip, late) in [
        ("walk", &gait, 0),
        ("walk in place", &in_place, 0),
        ("walk, 6 poses late", &gait, 6),
    ] {
        let mut live = Live::new(&mapping, &mixamo, &mannequin, Fixes::ALL)
            .expect("fits")
            .delayed(late);
        let mut out = mannequin.rest();
        let micros = time(budget, || {
            live.reset();
            for p in &clip.poses {
                live.pose(1.0 / 30.0, black_box(p), &mut out).expect("plays");
            }
            black_box(&out);
        }) / clip.poses.len() as f64;
        cases.push(Case {
            name: Box::leak(format!("mixamo 65 → mannequin 76: {name}, live, all fixes").into_boxed_str()),
            micros,
            per: "frame, reused live player",
        });
    }

    // And with skins, so hands keep the skin's distance too.
    let skins = [skin(&mixamo), skin(&mannequin)];
    let mut live = Live::touching(&mapping, &mixamo, &mannequin, [&skins[0], &skins[1]], Fixes::ALL).expect("fits");
    let mut out = mannequin.rest();
    let micros = time(budget, || {
        live.reset();
        for p in &gait.poses {
            live.pose(1.0 / 30.0, black_box(p), &mut out).expect("plays");
        }
        black_box(&out);
    }) / gait.poses.len() as f64;
    cases.push(Case {
        name: Box::leak(
            format!(
                "mixamo 65 → mannequin 76: walk, live, all fixes with skins ({} and {} points)",
                skins[0].len(),
                skins[1].len()
            )
            .into_boxed_str(),
        ),
        micros,
        per: "frame, reused live player",
    });

    cases.push(Case {
        name: "mixamo 65 → mannequin 76: player setup",
        micros: time(budget, || {
            black_box(Player::new(&mapping, &mixamo, black_box(&mannequin)).expect("same rigs"));
        }),
        per: "rig pair",
    });
    cases.push(Case {
        name: "mixamo 65 → mannequin 76: live player setup",
        micros: time(budget, || {
            black_box(Live::new(&mapping, &mixamo, black_box(&mannequin), Fixes::ALL).expect("same rigs"));
        }),
        per: "rig pair",
    });
    println!("| Case | µs | per |\n| --- | ---: | --- |");
    for c in &cases {
        println!("| {} | {:.2} | {} |", c.name, c.micros, c.per);
    }

    if let Some(path) = flag("--json") {
        let body: Vec<String> = cases
            .iter()
            .map(|c| format!("  {{\"name\": \"{}\", \"micros\": {}}}", c.name, c.micros))
            .collect();
        std::fs::write(&path, format!("[\n{}\n]\n", body.join(",\n"))).expect("writes the numbers");
    }

    if let Some(path) = flag("--against") {
        let base = std::fs::read_to_string(&path).expect("reads the base numbers");
        let mut slower = Vec::new();
        for c in &cases {
            // One case per line: {"name": "...", "micros": ...}
            let Some(line) = base.lines().find(|l| l.contains(&format!("\"{}\"", c.name))) else {
                continue;
            };
            let Some(was) = line
                .rsplit(':')
                .next()
                .and_then(|s| s.trim().trim_end_matches([',', '}']).trim().parse::<f64>().ok())
            else {
                continue;
            };
            let change = c.micros / was - 1.0;
            println!("{}: {was:.2} → {:.2} µs ({:+.0}%)", c.name, c.micros, change * 100.0);
            if change > slack {
                slower.push(c.name);
            }
        }
        if !slower.is_empty() {
            eprintln!("slower by more than {:.0}%: {}", slack * 100.0, slower.join(", "));
            std::process::exit(1);
        }
    }
}
