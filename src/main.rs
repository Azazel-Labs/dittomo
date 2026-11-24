//! Retargets a model file's clips onto another model and writes that model, with them, as a GLB, or the clips as BVH.
//!
//! ```text
//! dittomo SOURCE TARGET OUT.glb|OUT.vrm|OUT.bvh [--clip N|NAME]... [--fixes all|none] [--keep] [--fps N] [--pairs]
//! ```
//!
//! Every clip of SOURCE is played on TARGET, or those named by `--clip` (by name, or by number from 0), each named as
//! in SOURCE. TARGET's own clips are left out unless `--keep` is given. `--fixes none` plays the clips plainly, without
//! feet kept planted, hands kept out of the body, and the rest; `--fixes all` also gives a clip walked in place the
//! travel its feet imply; `--pairs` prints which bone of SOURCE each bone of TARGET follows.
//!
//! An OUT ending in `.bvh` gets TARGET's skeleton and a clip as BVH motion capture, `--fps` frames a second (by default
//! the clip's own rate, up to 120); with more than one clip, each goes to a file of its own, named OUT-CLIP.bvh.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use dittomo::load::{self, Model};
use dittomo::{Fixes, Mapping};
use polysmelt::document::Keep;

const USAGE: &str = concat!(
    "usage: dittomo SOURCE TARGET OUT.glb|OUT.vrm|OUT.bvh",
    " [--clip N|NAME]... [--fixes all|none] [--keep] [--fps N] [--pairs]"
);

fn main() -> ExitCode {
    match run(std::env::args().skip(1).collect()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("dittomo: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run(args: Vec<String>) -> Result<(), String> {
    let mut files: Vec<PathBuf> = Vec::new();
    let (mut picks, mut fixes, mut keep, mut pairs, mut fps) = (Vec::new(), Fixes::default(), false, false, None);
    let mut it = args.into_iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--clip" => picks.push(it.next().ok_or(USAGE)?),
            "--fixes" => {
                fixes = match it.next().as_deref() {
                    Some("all") => Fixes::ALL,
                    Some("none") => Fixes::NONE,
                    _ => return Err(USAGE.into()),
                }
            }
            "--keep" => keep = true,
            "--fps" => {
                let n: f32 = it.next().and_then(|n| n.parse().ok()).ok_or(USAGE)?;
                if !(n.is_finite() && n > 0.0) {
                    return Err(format!(
                        "--fps needs a positive number of frames a second
{USAGE}"
                    ));
                }
                fps = Some(n);
            }
            "--pairs" => pairs = true,
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(());
            }
            _ if a.starts_with("--") => return Err(format!("unknown option {a}\n{USAGE}")),
            _ => files.push(a.into()),
        }
    }
    let [source, target, out] = <[PathBuf; 3]>::try_from(files).map_err(|_| USAGE.to_string())?;
    let is = |ext: &str| out.extension().is_some_and(|e| e.eq_ignore_ascii_case(ext));
    let bvh = is("bvh");
    if is("gltf") {
        return Err("glTF goes out as GLB: name the output .glb (or .vrm, or .bvh for the clips alone)".into());
    }
    if fps.is_some() && !bvh {
        return Err("--fps is for BVH output, whose frames are evenly spaced; GLB keeps the clip's own times".into());
    }
    let read = |p: &Path, e: load::Error| format!("{}: {e}", p.display());
    let s = Model::load(&source).map_err(|e| read(&source, e))?;
    let mut doc = load::document(&target).map_err(|e| read(&target, e))?;
    let t = Model::from_document(&doc).map_err(|e| read(&target, e))?;

    let clips: Vec<usize> = if picks.is_empty() {
        (0..s.clips.len()).collect()
    } else {
        let find = |p: &String| {
            (s.clips.iter().position(|(n, _)| n.as_deref() == Some(p.as_str())))
                .or_else(|| p.parse().ok().filter(|&i| i < s.clips.len()))
                .ok_or_else(|| format!("{} has no clip {p}", source.display()))
        };
        picks.iter().map(find).collect::<Result<_, _>>()?
    };
    if clips.is_empty() {
        return Err(format!("{} has no clips", source.display()));
    }

    let mapping = Mapping::new(&s.skeleton, &t.skeleton);
    let paired = mapping.pairs().iter().flatten().count();
    eprintln!(
        "{paired} of {}'s {} bones follow {}'s; quality {:.0}% (bones played and how alike, by length)",
        target.display(),
        t.skeleton.len(),
        source.display(),
        100.0 * mapping.quality()
    );
    if pairs {
        for (b, bone) in t.skeleton.bones().iter().enumerate() {
            let from = mapping
                .source_of(b)
                .map_or("-", |a| s.skeleton.bones()[a].name.as_str());
            println!("{} <- {from}", bone.name);
        }
    }

    if bvh {
        let stem = out.with_extension("");
        for &c in &clips {
            let (name, clip) = &s.clips[c];
            let name = name.clone().unwrap_or_else(|| format!("clip{c}"));
            let played = mapping
                .apply_touching(&s.skeleton, &t.skeleton, [&s.skin, &t.skin], clip, fixes)
                .map_err(|e| format!("clip {name}: {e}"))?;
            let text = dittomo::bvh::write(&t.skeleton, &played, fps.map(|n| 1.0 / n))
                .map_err(|e| format!("clip {name}: {e}"))?;
            let file = if clips.len() == 1 {
                out.clone()
            } else {
                let safe: String = name
                    .chars()
                    .map(|c| {
                        if c.is_alphanumeric() || "-_.".contains(c) {
                            c
                        } else {
                            '_'
                        }
                    })
                    .collect();
                PathBuf::from(format!("{}-{safe}.bvh", stem.display()))
            };
            std::fs::write(&file, text).map_err(|e| format!("{}: {e}", file.display()))?;
            eprintln!("played {name}: wrote {}", file.display());
        }
        return Ok(());
    }
    if !keep {
        doc.prune(Keep {
            animations: false,
            ..Keep::ALL
        });
    }
    for &c in &clips {
        let (name, clip) = &s.clips[c];
        let name = name.clone().unwrap_or_else(|| format!("clip{c}"));
        let played = mapping
            .apply_touching(&s.skeleton, &t.skeleton, [&s.skin, &t.skin], clip, fixes)
            .map_err(|e| format!("clip {name}: {e}"))?;
        t.write_clip(&mut doc, Some(&name), &played)
            .map_err(|e| format!("clip {name}: {e}"))?;
        let seconds = clip.times.last().copied().unwrap_or(0.0) - clip.times[0];
        eprintln!("played {name}: {} poses over {seconds:.2} s", clip.times.len());
    }
    let glb = doc.to_glb().map_err(|e| format!("couldn't write the GLB: {e}"))?;
    std::fs::write(&out, glb).map_err(|e| format!("{}: {e}", out.display()))?;
    eprintln!("wrote {}", out.display());
    Ok(())
}
