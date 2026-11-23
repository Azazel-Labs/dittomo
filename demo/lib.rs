//! mummer in the browser, through a handful of plain wasm exports (no bindings generator).
//!
//! The page asks for a buffer, writes a model file's extension then its bytes into it, and calls [`load`] for the
//! source (slot 0) or the target (slot 1). [`map`] matches the two and leaves a JSON report as the text; [`play`] plays
//! a clip and leaves every bone's place in the world, frame by frame, for both skeletons. Results are read straight out
//! of the module's memory, at the pointers the exports give.

use std::cell::RefCell;

use mummer::load::Model;
use mummer::{Clip, Fixes, Mapping, Skeleton, contact};

#[path = "../benches/common/mod.rs"]
#[allow(dead_code)]
mod common;

#[derive(Default)]
struct State {
    buffer: Vec<u8>,
    models: [Option<Model>; 2],
    mapping: Option<Mapping>,
    text: String,
    times: Vec<f32>,
    places: [Vec<f32>; 2],
}

thread_local! {
    static STATE: RefCell<State> = RefCell::new(State::default());
}

fn with<T>(f: impl FnOnce(&mut State) -> T) -> T {
    STATE.with(|s| f(&mut s.borrow_mut()))
}

/// A buffer of `len` bytes for the page to write a model into: its extension, then its bytes.
#[unsafe(no_mangle)]
pub extern "C" fn buffer(len: usize) -> *mut u8 {
    with(|s| {
        s.buffer.clear();
        s.buffer.resize(len, 0);
        s.buffer.as_mut_ptr()
    })
}

/// Reads the model in the buffer (its first `ext_len` bytes the extension) into slot 0, the source, or 1, the target.
/// 1 if it read, else 0 with the error as the text.
#[unsafe(no_mangle)]
pub extern "C" fn load(slot: u32, ext_len: usize) -> u32 {
    with(|s| {
        let (ext, bytes) = s.buffer.split_at(ext_len.min(s.buffer.len()));
        let ext = String::from_utf8_lossy(ext).into_owned();
        match Model::from_bytes(bytes, &ext) {
            Ok(model) => {
                s.models[slot as usize & 1] = Some(model);
                s.mapping = None;
                s.buffer = Vec::new();
                1
            }
            Err(e) => {
                s.text = e.to_string();
                0
            }
        }
    })
}

/// Loads the built-in sample: a 65-bone rig laid out as Mixamo's, walking, and a 76-bone game-engine mannequin with
/// a five-bone spine, twist bones, an A-pose and other bone axes.
#[unsafe(no_mangle)]
pub extern "C" fn sample() {
    let model = |skeleton: Skeleton, clips: Vec<(Option<String>, Clip)>| Model {
        nodes: (0..skeleton.len()).collect(),
        skeleton,
        clips,
    };
    let source = common::mixamo();
    let walk = common::gait(&source, 180, false);
    let in_place = common::gait(&source, 180, true);
    let swing = common::walk(&source, 120);
    let clips = vec![
        (Some("walk".to_owned()), walk),
        (Some("walk in place".to_owned()), in_place),
        (Some("every bone swinging".to_owned()), swing),
    ];
    with(|s| {
        s.models = [Some(model(source, clips)), Some(model(common::mannequin(), Vec::new()))];
        s.mapping = None;
    });
}

/// Matches the source to the target, leaving a JSON report as the text: both skeletons, each target bone's source
/// and score, the quality, the source's clips and the limbs found. 1 if both are loaded.
#[unsafe(no_mangle)]
pub extern "C" fn map() -> u32 {
    with(|s| {
        let [Some(source), Some(target)] = &s.models else {
            s.text = "load a source and a target first".to_owned();
            return 0;
        };
        let mapping = Mapping::new(&source.skeleton, &target.skeleton);
        s.text = report(source, target, &mapping);
        s.mapping = Some(mapping);
        1
    })
}

/// Plays the source's clip `clip` on the target with the fixes in `fixes` (1 travel, 2 feet, 4 push-out), leaving
/// each frame's time and every bone's place in the world, for both. The number of frames, or 0 with the error as the
/// text.
#[unsafe(no_mangle)]
pub extern "C" fn play(clip: u32, fixes: u32) -> u32 {
    with(|s| {
        let ([Some(source), Some(target)], Some(mapping)) = (&s.models, &s.mapping) else {
            s.text = "map first".to_owned();
            return 0;
        };
        let rest;
        let from = match source.clips.get(clip as usize) {
            Some((_, c)) => c,
            None => {
                rest = Clip {
                    times: vec![0.0],
                    poses: vec![source.skeleton.rest()],
                };
                &rest
            }
        };
        let fixes = Fixes {
            travel: fixes & 1 != 0,
            feet: fixes & 2 != 0,
            push_out: fixes & 4 != 0,
        };
        let shown = if fixes.travel {
            contact::travel(&source.skeleton, from)
        } else {
            from.clone()
        };
        let played = match mapping.apply_with(&source.skeleton, &target.skeleton, from, fixes) {
            Ok(c) => c,
            Err(e) => {
                s.text = e.to_string();
                return 0;
            }
        };
        s.times.clone_from(&from.times);
        s.places = [places(&source.skeleton, &shown), places(&target.skeleton, &played)];
        from.poses.len() as u32
    })
}

fn places(skeleton: &Skeleton, clip: &Clip) -> Vec<f32> {
    clip.poses
        .iter()
        .flat_map(|p| skeleton.world(p))
        .flat_map(|w| w.position().to_array())
        .collect()
}

#[unsafe(no_mangle)]
pub extern "C" fn text() -> *const u8 {
    with(|s| s.text.as_ptr())
}

#[unsafe(no_mangle)]
pub extern "C" fn text_len() -> usize {
    with(|s| s.text.len())
}

#[unsafe(no_mangle)]
pub extern "C" fn times() -> *const f32 {
    with(|s| s.times.as_ptr())
}

/// Every bone's place, frame after frame, three floats each: 0 for the source, 1 for the target.
#[unsafe(no_mangle)]
pub extern "C" fn places_of(which: u32) -> *const f32 {
    with(|s| s.places[which as usize & 1].as_ptr())
}

fn report(source: &Model, target: &Model, mapping: &Mapping) -> String {
    use std::fmt::Write;
    let quote = |s: &str| {
        let mut out = String::from("\"");
        for c in s.chars() {
            match c {
                '"' => out.push_str("\\\""),
                '\\' => out.push_str("\\\\"),
                c if (c as u32) < 0x20 => {
                    let _ = write!(out, "\\u{:04x}", c as u32);
                }
                c => out.push(c),
            }
        }
        out.push('"');
        out
    };
    let skeleton = |sk: &Skeleton| {
        let bones: Vec<String> = sk
            .bones()
            .iter()
            .map(|b| {
                let parent = b.parent.map_or("null".to_owned(), |p| p.to_string());
                format!("{{\"name\":{},\"parent\":{parent}}}", quote(&b.name))
            })
            .collect();
        let limbs = |l: Vec<contact::Limb>| {
            let v: Vec<String> = l
                .iter()
                .map(|l| format!("[{},{},{}]", l.upper, l.lower, l.end))
                .collect();
            format!("[{}]", v.join(","))
        };
        format!(
            "{{\"bones\":[{}],\"legs\":{},\"arms\":{}}}",
            bones.join(","),
            limbs(contact::legs(sk)),
            limbs(contact::arms(sk))
        )
    };
    let pairs: Vec<String> = mapping
        .pairs()
        .iter()
        .map(|p| match p {
            Some(p) => format!("{{\"source\":{},\"score\":{:.3}}}", p.source, p.score),
            None => "null".to_owned(),
        })
        .collect();
    let clips: Vec<String> = source
        .clips
        .iter()
        .enumerate()
        .map(|(i, (name, c))| {
            let name = name.clone().unwrap_or_else(|| format!("clip {}", i + 1));
            let length = c.times.last().copied().unwrap_or(0.0) - c.times.first().copied().unwrap_or(0.0);
            format!(
                "{{\"name\":{},\"frames\":{},\"seconds\":{length:.3}}}",
                quote(&name),
                c.poses.len()
            )
        })
        .collect();
    format!(
        "{{\"quality\":{:.4},\"source\":{},\"target\":{},\"pairs\":[{}],\"clips\":[{}]}}",
        mapping.quality(),
        skeleton(&source.skeleton),
        skeleton(&target.skeleton),
        pairs.join(","),
        clips.join(",")
    )
}
