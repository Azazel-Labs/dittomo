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

use mummer::{Clip, Fixes, Mapping, Player, Skeleton, Transform};

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

fn play(budget: Duration, name: &'static str, source: &Skeleton, target: &Skeleton, clip: &Clip) -> [Case; 3] {
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

    cases.push(Case {
        name: "mixamo 65 → mannequin 76: player setup",
        micros: time(budget, || {
            black_box(Player::new(&mapping, &mixamo, black_box(&mannequin)).expect("same rigs"));
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
