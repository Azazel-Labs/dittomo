//! `dittomo::bake`'s WGSL run on a real adapter and held to the CPU reader.
//!
//! Bakes four clips on the bench mannequin in each encoding, uploads the bytes as they are to a storage buffer, and
//! runs compute shaders made of `bake::wgsl(0, 0)` and a few lines that call `bone`, `blend`, `skin` and
//! `skin_normal` for a list of queries. What comes back is compared with `Library::bone`, `Library::blend` and
//! `bake::skin` number for number, on every adapter the machine has (`WGPU_BACKEND` and `WGPU_ADAPTER_NAME` narrow
//! them down, as anywhere in wgpu).
//!
//! `cargo run --release` or `cargo test`. It ends with 1 if any adapter disagrees with the CPU, and with 2 if there
//! was no adapter to run on, unless `GPU_CHECK_OPTIONAL=1` says that is all right: then it says it was skipped and
//! ends with 0.

#[path = "../../benches/common/mod.rs"]
#[allow(dead_code)]
mod common;

use std::f32::consts::TAU;
use std::process::ExitCode;

use dittomo::bake::{self, Baker, Encoding, Library};
use dittomo::{Clip, Fixes, Mapping, Quat, Skeleton, Transform, Vec3};
use wgpu::util::DeviceExt;

const ENCODINGS: [Encoding; 3] = [Encoding::F32, Encoding::Half, Encoding::Fixed];

/// The most the shader's numbers may differ from the CPU's: a rotation's, and a place over the rig's height. Both
/// sides do the same sums in 32-bit floats, in whatever order the driver likes.
const BOUND: f32 = 1e-5;

/// Words in a query of each pass, and floats in what any of them answers.
const BONE_QUERY: usize = 4;
const BLEND_QUERY: usize = 8;
const SKIN_QUERY: usize = 24;
const ANSWER: usize = 8;

/// The shader's own part, after `bake::wgsl(0, 0)`: each pass reads its queries as words and writes eight floats a
/// query.
const MAIN: &str = r"
@group(1) @binding(0) var<storage, read> queries: array<u32>;
@group(1) @binding(1) var<storage, read_write> answers: array<f32>;

fn number(at: u32) -> f32 {
    return bitcast<f32>(queries[at]);
}

fn numbers(at: u32) -> vec3<f32> {
    return vec3<f32>(number(at), number(at + 1u), number(at + 2u));
}

fn put(at: u32, a: vec4<f32>, b: vec3<f32>) {
    answers[at] = a.x;
    answers[at + 1u] = a.y;
    answers[at + 2u] = a.z;
    answers[at + 3u] = a.w;
    answers[at + 4u] = b.x;
    answers[at + 5u] = b.y;
    answers[at + 6u] = b.z;
    answers[at + 7u] = 0.0;
}

// A query: clip, time, bone, nothing.
@compute @workgroup_size(64)
fn bones(@builtin(global_invocation_id) id: vec3<u32>) {
    let q = id.x * 4u;
    if q + 4u > arrayLength(&queries) {
        return;
    }
    let got = bone(queries[q], number(q + 1u), queries[q + 2u]);
    put(id.x * 8u, got.rotation, got.translation);
}

// A query: clip a, time a, clip b, time b, weight, bone, nothing twice.
@compute @workgroup_size(64)
fn blends(@builtin(global_invocation_id) id: vec3<u32>) {
    let q = id.x * 8u;
    if q + 8u > arrayLength(&queries) {
        return;
    }
    let got = blend(queries[q], number(q + 1u), queries[q + 2u], number(q + 3u), number(q + 4u), queries[q + 5u]);
    put(id.x * 8u, got.rotation, got.translation);
}

// A query: clip a, time a, clip b, time b, weight, nothing thrice; a point and nothing; a normal and nothing; four
// bones; four weights.
@compute @workgroup_size(64)
fn skins(@builtin(global_invocation_id) id: vec3<u32>) {
    let q = id.x * 24u;
    if q + 24u > arrayLength(&queries) {
        return;
    }
    let weights = vec4<f32>(number(q + 20u), number(q + 21u), number(q + 22u), number(q + 23u));
    let a = blend(queries[q], number(q + 1u), queries[q + 2u], number(q + 3u), number(q + 4u), queries[q + 16u]);
    let b = blend(queries[q], number(q + 1u), queries[q + 2u], number(q + 3u), number(q + 4u), queries[q + 17u]);
    let c = blend(queries[q], number(q + 1u), queries[q + 2u], number(q + 3u), number(q + 4u), queries[q + 18u]);
    let d = blend(queries[q], number(q + 1u), queries[q + 2u], number(q + 3u), number(q + 4u), queries[q + 19u]);
    let point = skin(numbers(q + 8u), a, b, c, d, weights);
    let normal = skin_normal(numbers(q + 12u), a, b, c, d, weights);
    put(id.x * 8u, vec4<f32>(point, 0.0), normal);
}
";

/// How tall the rig stands at rest.
fn height(sk: &Skeleton) -> f32 {
    let ys = sk.world(&sk.rest()).into_iter().map(|p| p.position().y);
    ys.clone().fold(f32::MIN, f32::max) - ys.fold(f32::MAX, f32::min)
}

/// A second of Mixamo's rig turning once round on the spot with every bone swinging, the last pose the first again.
fn spin(sk: &Skeleton) -> Clip {
    let mut clip = Clip::default();
    for f in 0..=30 {
        let t = f as f32 / 30.0;
        let mut pose = sk.rest();
        for (i, (tr, b)) in pose.iter_mut().zip(sk.bones()).enumerate() {
            let k = i as f32;
            let axis = Vec3::new((k * 1.3).sin(), (k * 0.7).cos(), (k * 2.1).sin() + 0.3).normalize();
            tr.rotation = b.rest.rotation * Quat::from_axis_angle(axis, 0.4 * (TAU * t + k).sin());
            if b.name == "mixamorig:Hips" {
                tr.rotation = Quat::from_rotation_y(TAU * t) * tr.rotation;
                tr.translation.y += 3.0 * (2.0 * TAU * t).sin();
            }
        }
        clip.times.push(t);
        clip.poses.push(pose);
    }
    clip
}

/// The mannequin, and four clips retargeted onto it from Mixamo's rig, each with the rate it is baked at and whether
/// it loops: a walk 4.2 m off played once, a second of it kept in place and looped at another rate than it was keyed
/// at, a turn once round on the spot that loops (so its last frame and its first lie in opposite hemispheres), and
/// one pose held.
fn clips() -> (Skeleton, Vec<(Clip, f32, bool)>) {
    let (source, target) = (common::mixamo(), common::mannequin());
    let map = Mapping::new(&source, &target);
    let play = |clip: &Clip, fixes: Fixes| map.apply_with(&source, &target, clip, fixes).expect("a retarget");
    let walk = common::walk(&source, 90);
    let mut in_place = common::walk(&source, 31);
    for pose in &mut in_place.poses {
        for (tr, b) in pose.iter_mut().zip(source.bones()) {
            tr.translation.z = b.rest.translation.z;
        }
    }
    let walk = play(&walk, Fixes::ALL);
    let still = Clip {
        times: vec![2.0],
        poses: vec![walk.poses[50].clone()],
    };
    let clips = vec![
        (walk, 30.0, false),
        (play(&in_place, Fixes::default()), 24.0, true),
        (play(&spin(&source), Fixes::default()), 30.0, true),
        (still, 30.0, true),
    ];
    (target, clips)
}

/// The same numbers every run, spread from 0 to 1.
struct Dice(u32);

impl Dice {
    fn roll(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (self.0 >> 8) as f32 / 16_777_216.0
    }

    fn pick(&mut self, n: usize) -> usize {
        ((self.roll() * n as f32) as usize).min(n - 1)
    }
}

/// What each pass is asked, as the words the shader reads.
struct Queries {
    bone: Vec<u32>,
    blend: Vec<u32>,
    skin: Vec<u32>,
}

/// The times a clip is asked at: each frame and the one past the last, places between each two, before the clip
/// starts, past its end, a hair either side of its end (where a loop wraps), and many times round.
fn times(entry: bake::Entry) -> Vec<f32> {
    let (rate, seconds) = (entry.rate, entry.seconds());
    let mut out = Vec::new();
    for k in 0..=entry.frames {
        let k = k as f32;
        out.extend([
            k / rate,
            (k + 0.5) / rate,
            (k + [0.37, 0.93, 0.001, 0.75][k as usize % 4]) / rate,
        ]);
    }
    out.extend([
        -1e-4,
        -0.01,
        -0.5 / rate,
        -0.5 * seconds,
        -seconds,
        -2.3 * seconds,
        -3.0 * seconds,
        -7.77,
    ]);
    out.extend([
        seconds + 1e-4,
        seconds + 0.5 / rate,
        1.5 * seconds,
        2.0 * seconds,
        3.3 * seconds,
        7.0 * seconds,
        61.7,
    ]);
    out.extend([seconds, seconds - 1e-4, seconds - 0.25 / rate, seconds - 0.75 / rate]);
    out
}

fn queries(sk: &Skeleton, library: &Library) -> Queries {
    let (bones, clips) = (library.bones() as u32, library.clips() as u32);
    let entries: Vec<bake::Entry> = (0..library.clips())
        .map(|c| library.entry(c).expect("the clip"))
        .collect();
    let mut dice = Dice(7);

    // Every bone of every clip at each of its times, then clips and bones the library doesn't hold: both sides take
    // the last it does.
    let mut bone = Vec::new();
    for (clip, &entry) in entries.iter().enumerate() {
        for time in times(entry) {
            for b in 0..bones {
                bone.extend([clip as u32, time.to_bits(), b, 0]);
            }
        }
    }
    for (clip, b) in [(clips, 3), (clips + 6, bones), (1, bones + 9), (u32::MAX, u32::MAX)] {
        for time in [-0.3f32, 0.0, 0.41, 5.0] {
            bone.extend([clip, time.to_bits(), b, 0]);
        }
    }

    // A time anywhere from half a clip before it starts to twice round, and a weight that is 0 or 1 now and then.
    let time = |dice: &mut Dice, clip: usize| (dice.roll() * 3.0 - 0.5) * entries[clip].seconds().max(0.5);
    let weight = |dice: &mut Dice| match dice.pick(8) {
        0 => 0.0,
        1 => 1.0,
        _ => dice.roll(),
    };

    // Every bone between every two clips, a clip and itself too, eight times over.
    let mut blend = Vec::new();
    for a in 0..entries.len() {
        for b in 0..entries.len() {
            for _ in 0..8 {
                let (time_a, time_b, w) = (time(&mut dice, a), time(&mut dice, b), weight(&mut dice));
                for index in 0..bones {
                    blend.extend([a as u32, time_a.to_bits(), b as u32, time_b.to_bits()]);
                    blend.extend([w.to_bits(), index, 0, 0]);
                }
            }
        }
    }

    // Points near each bone that has three above it, bound to the four of them, forty times over.
    let rest = sk.world(&sk.rest());
    let tall = height(sk);
    let up = |b: usize| sk.bones()[b].parent;
    let chains: Vec<[usize; 4]> = (0..sk.len())
        .filter_map(|b| {
            let (p, pp) = (up(b)?, up(up(b)?)?);
            Some([b, p, pp, up(pp)?])
        })
        .collect();
    let mut skin = Vec::new();
    for chain in &chains {
        for _ in 0..40 {
            let (a, b) = (dice.pick(entries.len()), dice.pick(entries.len()));
            let (time_a, time_b, w) = (time(&mut dice, a), time(&mut dice, b), weight(&mut dice));
            let mut around = || Vec3::new(dice.roll() - 0.5, dice.roll() - 0.5, dice.roll() - 0.5);
            let point = rest[chain[0]].position() + around() * 0.1 * tall;
            let normal = (around() + Vec3::splat(1e-3)).normalize();
            let mut weights = [dice.roll() + 0.05, dice.roll(), dice.roll(), dice.roll()];
            let sum: f32 = weights.iter().sum();
            weights = weights.map(|x| x / sum);
            skin.extend([a as u32, time_a.to_bits(), b as u32, time_b.to_bits()]);
            skin.extend([w.to_bits(), 0, 0, 0]);
            skin.extend([point.x, point.y, point.z, 0.0].map(f32::to_bits));
            skin.extend([normal.x, normal.y, normal.z, 0.0].map(f32::to_bits));
            skin.extend(chain.map(|b| b as u32));
            skin.extend(weights.map(f32::to_bits));
        }
    }
    Queries { bone, blend, skin }
}

/// The worst the shader was off the CPU in each number it gives, and the first few queries past [`BOUND`].
#[derive(Default)]
struct Worst {
    /// `bone`'s rotation and translation, `blend`'s, then `skin`'s point and `skin_normal`'s normal: rotations and
    /// the normal as the largest difference in any one number, the others as a distance over the rig's height.
    off: [f32; 6],
    queries: [usize; 3],
    over: Vec<String>,
    not_numbers: usize,
}

const NAMES: [&str; 6] = [
    "bone rotation",
    "bone translation",
    "blend rotation",
    "blend translation",
    "skin point",
    "skin_normal",
];

impl Worst {
    fn take(&mut self, which: usize, off: f32, what: impl Fn() -> String) {
        // A difference that is no number is over any bound.
        let off = if off.is_nan() {
            self.not_numbers += 1;
            f32::INFINITY
        } else {
            off
        };
        if off > BOUND && self.over.len() < 12 {
            self.over.push(format!("{} off by {off:.3e}: {}", NAMES[which], what()));
        }
        self.off[which] = self.off[which].max(off);
    }

    fn ok(&self) -> bool {
        self.off.iter().all(|&off| off <= BOUND)
    }
}

fn float(word: u32) -> f32 {
    f32::from_bits(word)
}

/// The largest difference between two rotations' numbers.
fn apart(a: Quat, b: [f32; 4]) -> f32 {
    let d = (a.x - b[0]).abs().max((a.y - b[1]).abs());
    d.max((a.z - b[2]).abs()).max((a.w - b[3]).abs()) + if a.is_nan() { f32::NAN } else { 0.0 }
}

/// The CPU's answers to the queries against the shader's.
fn compare(library: &Library, tall: f32, queries: &Queries, answers: [&[f32]; 3]) -> Worst {
    let mut worst = Worst::default();
    let vec3 = |f: &[f32]| Vec3::new(f[0], f[1], f[2]);

    let pairs = queries
        .bone
        .as_chunks::<BONE_QUERY>()
        .0
        .iter()
        .zip(answers[0].as_chunks::<ANSWER>().0);
    for (q, got) in pairs {
        let (clip, time, bone) = (q[0], float(q[1]), q[2]);
        let want = library.bone(clip, time, bone);
        let said = || format!("clip {clip} at {time} s, bone {bone}: CPU {want:?}, shader {got:?}");
        worst.take(0, apart(want.rotation, [got[0], got[1], got[2], got[3]]), said);
        worst.take(1, want.translation.distance(vec3(&got[4..7])) / tall, said);
    }
    worst.queries[0] = queries.bone.len() / BONE_QUERY;

    let pairs = queries
        .blend
        .as_chunks::<BLEND_QUERY>()
        .0
        .iter()
        .zip(answers[1].as_chunks::<ANSWER>().0);
    for (q, got) in pairs {
        let (a, time_a, b, time_b, weight, bone) = (q[0], float(q[1]), q[2], float(q[3]), float(q[4]), q[5]);
        let want = library.blend(a, time_a, b, time_b, weight, bone);
        let said = || {
            format!(
                "clip {a} at {time_a} s to clip {b} at {time_b} s by {weight}, bone {bone}: CPU {want:?}, shader \
                 {got:?}"
            )
        };
        worst.take(2, apart(want.rotation, [got[0], got[1], got[2], got[3]]), said);
        worst.take(3, want.translation.distance(vec3(&got[4..7])) / tall, said);
    }
    worst.queries[1] = queries.blend.len() / BLEND_QUERY;

    let pairs = queries
        .skin
        .as_chunks::<SKIN_QUERY>()
        .0
        .iter()
        .zip(answers[2].as_chunks::<ANSWER>().0);
    for (q, got) in pairs {
        let (a, time_a, b, time_b, weight) = (q[0], float(q[1]), q[2], float(q[3]), float(q[4]));
        let numbers = |at: usize| Vec3::new(float(q[at]), float(q[at + 1]), float(q[at + 2]));
        let (point, normal) = (numbers(8), numbers(12));
        let chain = [q[16], q[17], q[18], q[19]];
        let weights = [float(q[20]), float(q[21]), float(q[22]), float(q[23])];
        let bones: [Transform; 4] = chain.map(|bone| library.blend(a, time_a, b, time_b, weight, bone));
        let want = bake::skin(point, &bones, weights);
        // `skin_normal` has no twin in the crate: this is the shader's, line for line.
        let facing = (bones.iter().zip(weights))
            .map(|(bone, w)| bone.rotation * normal * w)
            .sum::<Vec3>()
            .normalize();
        let said = || {
            format!(
                "clip {a} at {time_a} s to clip {b} at {time_b} s by {weight}, bones {chain:?} by {weights:?}: CPU \
                 {want:?} facing {facing:?}, shader {got:?}"
            )
        };
        worst.take(4, want.distance(vec3(&got[0..3])) / tall, said);
        worst.take(5, (facing - vec3(&got[4..7])).abs().max_element(), said);
    }
    worst.queries[2] = queries.skin.len() / SKIN_QUERY;
    worst
}

/// What `work` gives, or what wgpu or the driver's own shader compiler said against it.
fn guarded<T>(device: &wgpu::Device, work: impl FnOnce() -> T) -> Result<T, String> {
    let internal = device.push_error_scope(wgpu::ErrorFilter::Internal);
    let validation = device.push_error_scope(wgpu::ErrorFilter::Validation);
    let out = work();
    let errors = [pollster::block_on(validation.pop()), pollster::block_on(internal.pop())];
    match errors.into_iter().flatten().next() {
        Some(e) => Err(e.to_string()),
        None => Ok(out),
    }
}

/// A device with the three passes built on it.
struct Gpu {
    device: wgpu::Device,
    queue: wgpu::Queue,
    library: wgpu::BindGroupLayout,
    asked: wgpu::BindGroupLayout,
    /// `bones`, `blends` and `skins`, each with how many words a query of its has.
    passes: [(wgpu::ComputePipeline, usize); 3],
}

impl Gpu {
    fn new(adapter: &wgpu::Adapter) -> Result<Self, String> {
        // The least a device may have and still run compute shaders: enough here, and no adapter is asked for more
        // than it has.
        let ask = wgpu::DeviceDescriptor {
            label: Some("gpu-check"),
            required_limits: wgpu::Limits::downlevel_defaults(),
            ..Default::default()
        };
        let (device, queue) = pollster::block_on(adapter.request_device(&ask)).map_err(|e| e.to_string())?;
        let storage = |binding: u32, read_only: bool| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::COMPUTE,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Storage { read_only },
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        };
        let library = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("library"),
            entries: &[storage(0, true)],
        });
        let asked = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("queries and answers"),
            entries: &[storage(0, true), storage(1, false)],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("gpu-check"),
            bind_group_layouts: &[Some(&library), Some(&asked)],
            immediate_size: 0,
        });
        // The sampler as the crate hands it out, bound where a host would bind it, and the passes after it.
        let source = bake::wgsl(0, 0) + MAIN;
        // No colons in the label: DXC reads it as the shader's file name.
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("the sampler and the passes"),
            source: wgpu::ShaderSource::Wgsl(source.into()),
        });
        let pass = |entry: &str, words: usize| {
            let pipeline = device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
                label: Some(entry),
                layout: Some(&layout),
                module: &module,
                entry_point: Some(entry),
                compilation_options: wgpu::PipelineCompilationOptions::default(),
                cache: None,
            });
            (pipeline, words)
        };
        let built = guarded(&device, || {
            [
                pass("bones", BONE_QUERY),
                pass("blends", BLEND_QUERY),
                pass("skins", SKIN_QUERY),
            ]
        });
        let passes = built.map_err(|e| format!("the shader was turned down: {e}"))?;
        Ok(Self {
            device,
            queue,
            library,
            asked,
            passes,
        })
    }

    /// The library's bytes in a storage buffer, as they are.
    fn upload(&self, bytes: &[u8]) -> wgpu::BindGroup {
        let buffer = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("library"),
            contents: bytes,
            usage: wgpu::BufferUsages::STORAGE,
        });
        self.device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("library"),
            layout: &self.library,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: buffer.as_entire_binding(),
            }],
        })
    }

    /// One pass over its queries: eight floats back for each, as many queries at a time as the device dispatches.
    fn run(&self, library: &wgpu::BindGroup, pass: usize, queries: &[u32]) -> Result<Vec<f32>, String> {
        let (pipeline, words) = &self.passes[pass];
        let most = self.device.limits().max_compute_workgroups_per_dimension as usize * 64;
        let mut out = Vec::with_capacity(queries.len() / words * ANSWER);
        for batch in queries.chunks(most * words) {
            let count = batch.len() / words;
            let bytes: Vec<u8> = batch.iter().flat_map(|w| w.to_le_bytes()).collect();
            let size = (count * ANSWER * 4) as u64;
            let asked = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("queries"),
                contents: &bytes,
                usage: wgpu::BufferUsages::STORAGE,
            });
            let answers = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("answers"),
                size,
                usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
                mapped_at_creation: false,
            });
            let read = self.device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("answers read back"),
                size,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            let group = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("queries and answers"),
                layout: &self.asked,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: asked.as_entire_binding(),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: answers.as_entire_binding(),
                    },
                ],
            });
            let sent = guarded(&self.device, || {
                let mut encoder = self
                    .device
                    .create_command_encoder(&wgpu::CommandEncoderDescriptor::default());
                {
                    let mut compute = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor::default());
                    compute.set_pipeline(pipeline);
                    compute.set_bind_group(0, library, &[]);
                    compute.set_bind_group(1, &group, &[]);
                    compute.dispatch_workgroups(count.div_ceil(64) as u32, 1, 1);
                }
                encoder.copy_buffer_to_buffer(&answers, 0, &read, 0, size);
                self.queue.submit([encoder.finish()]);
            });
            sent.map_err(|e| format!("the pass was turned down: {e}"))?;
            let (sender, receiver) = std::sync::mpsc::channel();
            read.slice(..).map_async(wgpu::MapMode::Read, move |mapped| {
                let _ = sender.send(mapped);
            });
            (self.device.poll(wgpu::PollType::wait_indefinitely())).map_err(|e| e.to_string())?;
            (receiver.recv().map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
            let view = read.slice(..).get_mapped_range().map_err(|e| e.to_string())?;
            out.extend(
                view.as_chunks::<4>()
                    .0
                    .iter()
                    .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])),
            );
        }
        Ok(out)
    }
}

/// Every encoding's library through the three passes on one adapter. Whether the shader agreed with the CPU.
fn check(adapter: &wgpu::Adapter, sk: &Skeleton, libraries: &[(Encoding, Vec<u8>)]) -> Result<bool, String> {
    let gpu = Gpu::new(adapter)?;
    let tall = height(sk);
    let mut agreed = true;
    for (encoding, bytes) in libraries {
        let library = Library::read(bytes).map_err(|e| e.to_string())?;
        let queries = queries(sk, &library);
        let bound = gpu.upload(bytes);
        let bone = gpu.run(&bound, 0, &queries.bone)?;
        let blend = gpu.run(&bound, 1, &queries.blend)?;
        let skin = gpu.run(&bound, 2, &queries.skin)?;
        let worst = compare(&library, tall, &queries, [&bone, &blend, &skin]);
        let [bones, blends, skins] = worst.queries;
        println!(
            "  {encoding:?}, {} bytes, {bones} bone, {blends} blend and {skins} skin queries:",
            bytes.len()
        );
        for (name, off) in NAMES.iter().zip(worst.off) {
            println!("    {name:<18} {off:.2e}");
        }
        for line in &worst.over {
            println!("    OVER {BOUND:e}: {line}");
        }
        if worst.not_numbers > 0 {
            println!("    {} answers were no numbers", worst.not_numbers);
        }
        agreed &= worst.ok();
    }
    Ok(agreed)
}

/// What the run came to.
#[derive(Debug, PartialEq, Eq)]
enum Outcome {
    /// The shader agreed with the CPU on every adapter, of which there were this many.
    Agreed(usize),
    /// Some adapter's shader didn't, or couldn't be run.
    Disagreed,
    /// There was no adapter with compute shaders to run on.
    NoAdapter,
}

fn run() -> Outcome {
    let (sk, clips) = clips();
    let libraries: Vec<(Encoding, Vec<u8>)> = ENCODINGS
        .iter()
        .map(|&encoding| {
            let mut baker = Baker::new(&sk, encoding);
            for (clip, rate, looping) in &clips {
                baker.add(clip, *rate, *looping).expect("a clip baked");
            }
            (encoding, baker.finish())
        })
        .collect();
    {
        // The turn's last frame and its first lie in opposite hemispheres somewhere: the wrap the shader must take
        // the short way round.
        let library = Library::read(&libraries[0].1).expect("a library");
        let last = library.entry(2).expect("the turn").frames as usize - 1;
        let flips = (0..sk.len()).any(|b| {
            let (a, z) = (library.stored(2, 0, b).unwrap(), library.stored(2, last, b).unwrap());
            a.rotation.dot(z.rotation) < 0.0
        });
        assert!(flips, "the looping turn no longer wraps across hemispheres");
    }
    println!(
        "{} bones, {} clips, the rig {:.1} tall; rotations and normals by their numbers, places over the rig's height",
        sk.len(),
        clips.len(),
        height(&sk)
    );

    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
    let named = std::env::var("WGPU_ADAPTER_NAME").ok().map(|n| n.to_lowercase());
    let adapters = pollster::block_on(instance.enumerate_adapters(wgpu::Backends::all()));
    let (mut ran, mut failed) = (0, false);
    for adapter in &adapters {
        let info = adapter.get_info();
        if named.as_ref().is_some_and(|n| !info.name.to_lowercase().contains(n)) {
            continue;
        }
        let compute = (adapter.get_downlevel_capabilities().flags).contains(wgpu::DownlevelFlags::COMPUTE_SHADERS);
        println!(
            "adapter: {} ({:?}, {:?}; driver {} {})",
            info.name, info.backend, info.device_type, info.driver, info.driver_info
        );
        if !compute {
            println!("  no compute shaders: passed over");
            continue;
        }
        match check(adapter, &sk, &libraries) {
            Ok(true) => ran += 1,
            Ok(false) => {
                println!("  MISMATCH: the shader and the CPU reader disagree past {BOUND:e}");
                failed = true;
            }
            Err(e) => {
                println!("  FAILED to run: {e}");
                failed = true;
            }
        }
    }
    if failed {
        Outcome::Disagreed
    } else if ran == 0 {
        Outcome::NoAdapter
    } else {
        Outcome::Agreed(ran)
    }
}

/// Whether having no adapter is all right: `GPU_CHECK_OPTIONAL=1`.
fn optional() -> bool {
    std::env::var("GPU_CHECK_OPTIONAL").is_ok_and(|v| v == "1")
}

fn main() -> ExitCode {
    match run() {
        Outcome::Agreed(adapters) => {
            println!(
                "gpu-check: OK, the shader gives what the CPU reader gives within {BOUND:e} on {adapters} adapter(s)"
            );
            ExitCode::SUCCESS
        }
        Outcome::Disagreed => {
            println!("gpu-check: FAILED");
            ExitCode::from(1)
        }
        Outcome::NoAdapter if optional() => {
            println!("gpu-check: SKIPPED, no adapter with compute shaders and GPU_CHECK_OPTIONAL=1");
            ExitCode::SUCCESS
        }
        Outcome::NoAdapter => {
            println!(
                "gpu-check: FAILED, no adapter with compute shaders to run on (set GPU_CHECK_OPTIONAL=1 to let that \
                 pass)"
            );
            ExitCode::from(2)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The same run as `cargo run`: `cargo test -- --nocapture` shows the adapters and the numbers.
    #[test]
    fn the_shader_gives_what_the_cpu_reader_gives() {
        match run() {
            Outcome::Agreed(_) => {}
            Outcome::NoAdapter if optional() => {
                println!("gpu-check: SKIPPED, no adapter with compute shaders and GPU_CHECK_OPTIONAL=1");
            }
            other => panic!("gpu-check: {other:?}"),
        }
    }
}
