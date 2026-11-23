//! Renders a clip retargeted from one model onto another as a video, side by side: the source as its file plays it,
//! the target played by this crate, and the target with each paired bone's local rotation copied as it is, which is
//! what playing a clip on another rig by bone name gives. The meshes are skinned and drawn on the CPU, over a checkered
//! floor that stays put, so feet that slide show it; frames are piped to `ffmpeg`, which must be on the path.
//!
//! ```text
//! cargo run --release --features fs --example video -- SOURCE TARGET OUT.mp4 [--clip N|NAME] [--seconds S]
//!     [--still FRAME OUT.png] [--fixes none|notouch] [--bare]
//! ```
//!
//! `--bare` leaves the labels off, for frames to lay out elsewhere.

use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};

use mummer::load::Model;
use mummer::{Clip, Fixes, Mapping, Transform, Vec3};
use polysmelt::assets::Folder;
use polysmelt::document::{Document, Keep, Pose};
use polysmelt::mesh::Mesh;
use polysmelt::rig::{Rig, Trs};

/// One panel's size in pixels, and how many samples a side each pixel takes.
const W: usize = 480;
const H: usize = 720;
const SS: usize = 2;
const FPS: f32 = 30.0;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut free = Vec::new();
    let (mut clip_arg, mut seconds, mut still, mut fixes) = (None, None, None, Fixes::ALL);
    let mut bare = false;
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--clip" => clip_arg = it.next().cloned(),
            "--seconds" => seconds = it.next().and_then(|s| s.parse::<f32>().ok()),
            "--still" => {
                let f: usize = it.next().and_then(|s| s.parse().ok()).expect("--still FRAME OUT.png");
                still = Some((f, PathBuf::from(it.next().expect("--still FRAME OUT.png"))));
            }
            "--bare" => bare = true,
            "--fixes" => match it.next().map(String::as_str) {
                Some("none") => fixes = Fixes::NONE,
                Some("notouch") => fixes.touch = false,
                _ => {}
            },
            _ => free.push(a.clone()),
        }
    }
    let [source, target, out] = free.as_slice() else {
        eprintln!("usage: video SOURCE TARGET OUT.mp4 [--clip N|NAME] [--seconds S] [--still FRAME OUT.png] [--bare]");
        std::process::exit(2);
    };
    let (src, tgt) = (Character::load(Path::new(source)), Character::load(Path::new(target)));
    let a = match &clip_arg {
        None => 0,
        Some(c) => c.parse().unwrap_or_else(|_| {
            src.model
                .clips
                .iter()
                .position(|(n, _)| n.as_deref() == Some(c.as_str()))
                .expect("no clip of that name")
        }),
    };
    let (clip_name, clip) = &src.model.clips[a];
    let (ss, ts) = (&src.model.skeleton, &tgt.model.skeleton);
    let map = Mapping::new(ss, ts);
    let played = map
        .apply_touching(ss, ts, [&src.model.skin, &tgt.model.skin], clip, fixes)
        .expect("plays");
    eprintln!("skin points: {} {}", src.model.skin.len(), tgt.model.skin.len());
    let mapped = map.pairs().iter().filter(|p| p.is_some()).count();
    eprintln!(
        "{} → {}: {mapped} of {} target bones paired, clip {:?} of {:.2} s",
        src.name,
        tgt.name,
        ts.len(),
        clip_name,
        clip.times.last().copied().unwrap_or(0.0)
    );

    let start = clip.times.first().copied().unwrap_or(0.0);
    let length = clip.times.last().copied().unwrap_or(0.0) - start;
    // Short clips loop until there's something to watch.
    let total = seconds.unwrap_or_else(|| {
        if length < 4.0 {
            (4.0 / length.max(0.1)).ceil() * length
        } else {
            length
        }
    });
    let frames = ((total * FPS).round() as usize).max(1);

    let labels = [
        (
            "SOURCE".to_string(),
            format!("{} {}", src.name, clip_name.as_deref().unwrap_or("")),
        ),
        (
            "DITTOMO".to_string(),
            format!("{}  {mapped}/{} BONES", tgt.name, ts.len()),
        ),
        ("ROTATIONS COPIED".to_string(), format!("{}  BY BONE PAIR", tgt.name)),
    ];
    let mut panels = [Panel::new(&src), Panel::new(&tgt), Panel::new(&tgt)];
    let mut ffmpeg = still.is_none().then(|| {
        Command::new("ffmpeg")
            .args(["-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s"])
            .arg(format!("{}x{H}", 3 * W))
            .args([
                "-r",
                &FPS.to_string(),
                "-i",
                "-",
                "-c:v",
                "libx264",
                "-preset",
                "veryfast",
                "-crf",
                "20",
            ])
            .args(["-pix_fmt", "yuv420p", "-movflags", "+faststart"])
            .arg(out)
            .stdin(Stdio::piped())
            .spawn()
            .expect("ffmpeg runs")
    });
    // Frames go to ffmpeg from a thread of their own, so the next is drawn while it takes one.
    let writer = ffmpeg.as_mut().map(|ff| {
        let mut stdin = ff.stdin.take().expect("ffmpeg's input");
        let (tx, rx) = std::sync::mpsc::sync_channel::<Vec<u8>>(4);
        let done = std::thread::spawn(move || {
            for frame in rx {
                stdin.write_all(&frame).expect("ffmpeg takes the frame");
            }
        });
        (tx, done)
    });
    let mut frame = vec![0u8; 3 * W * 3 * H];
    let range = match &still {
        Some((f, _)) => 0..=*f,
        None => 0..=frames - 1,
    };
    for f in range {
        let t = start + (f as f32 / FPS) % length.max(f32::EPSILON);
        let s_pose = sample(clip, t);
        let mummer_pose = sample(&played, t);
        // The same travel as mummer's, so only the rotations differ.
        let mut copied = mummer_pose.clone();
        for (b, bone) in ts.bones().iter().enumerate() {
            copied[b].rotation = map.source_of(b).map_or(bone.rest.rotation, |sb| s_pose[sb].rotation);
        }
        let draw = still.as_ref().is_none_or(|(s, _)| *s == f);
        // Each panel skinned and drawn on its own thread.
        let drawn: Vec<bool> = std::thread::scope(|sc| {
            let jobs: Vec<_> = panels
                .iter_mut()
                .enumerate()
                .map(|(p, panel)| {
                    let (src, tgt, labels) = (&src, &tgt, &labels);
                    let (mummer_pose, copied) = (&mummer_pose, &copied);
                    sc.spawn(move || {
                        let mesh = match p {
                            0 => src.doc.flatten(Pose::Animation { index: a, time: t }).expect("draws"),
                            1 => tgt.draw(mummer_pose),
                            _ => tgt.draw(copied),
                        };
                        let Some(mesh) = mesh else { return false };
                        panel.follow(&mesh, f == 0);
                        if draw {
                            panel.render(&mesh, (!bare).then_some(&labels[p]));
                        }
                        draw
                    })
                })
                .collect();
            jobs.into_iter().map(|j| j.join().expect("a panel draws")).collect()
        });
        for (p, panel) in panels.iter().enumerate() {
            if drawn[p] {
                panel.blit(&mut frame, p);
            }
        }
        if let Some((tx, _)) = &writer {
            tx.send(frame.clone()).expect("the writer runs");
        }
        if f % 30 == 0 {
            eprint!("\r{f}/{frames}");
        }
    }
    eprintln!();
    if let Some((_, path)) = &still {
        image::save_buffer(path, &frame, (3 * W) as u32, H as u32, image::ColorType::Rgb8).expect("writes the still");
    }
    if let Some((tx, done)) = writer {
        drop(tx);
        done.join().expect("the writer ends");
    }
    if let Some(mut ff) = ffmpeg {
        assert!(ff.wait().expect("ffmpeg ends").success(), "ffmpeg failed");
        eprintln!("wrote {out}");
    }
}

/// A model: its document twice (with textures for the first frame, without for the rest), rig and skeleton.
struct Character {
    name: String,
    doc: Document<'static>,
    textured: Document<'static>,
    rig: Rig,
    model: Model,
}

impl Character {
    fn load(path: &Path) -> Self {
        let ext = path
            .extension()
            .and_then(|e| e.to_str())
            .unwrap_or_default()
            .to_ascii_lowercase();
        let ext = if ext == "vrm" { "glb".to_string() } else { ext };
        let assets = Folder::new(path.parent().unwrap_or(Path::new(".")));
        let read = |keep| polysmelt::formats::load(path, &ext, &assets, keep).expect("reads the model");
        let doc = read(Keep {
            image_bytes: false,
            ..Keep::ALL
        });
        let textured = read(Keep::ALL);
        let rig = doc.rig().expect("has a rig");
        let model = Model::from_document(&doc).expect("has bones");
        let name = path
            .file_stem()
            .unwrap()
            .to_string_lossy()
            .to_uppercase()
            .replace(['-', '_'], " ");
        Self {
            name,
            doc,
            textured,
            rig,
            model,
        }
    }

    /// The mesh with the skeleton in `pose`.
    fn draw(&self, pose: &[Transform]) -> Option<Mesh> {
        let mut nodes: Vec<Trs> = self.rig.nodes.iter().map(|n| n.rest).collect();
        for (b, &n) in self.model.nodes.iter().enumerate() {
            let t = &pose[b];
            nodes[n] = Trs {
                translation: t.translation.to_array(),
                rotation: t.rotation.to_array(),
                scale: t.scale.to_array(),
            };
        }
        self.doc.flatten_nodes(&nodes).expect("draws")
    }
}

/// The clip's pose at `t`, between its keys.
fn sample(clip: &Clip, t: f32) -> Vec<Transform> {
    let k = clip.times.partition_point(|&x| x <= t);
    if k == 0 {
        return clip.poses[0].clone();
    }
    if k == clip.times.len() {
        return clip.poses[k - 1].clone();
    }
    let (a, b) = (k - 1, k);
    let s = ((t - clip.times[a]) / (clip.times[b] - clip.times[a]).max(f32::EPSILON)).clamp(0.0, 1.0);
    clip.poses[a]
        .iter()
        .zip(&clip.poses[b])
        .map(|(x, y)| Transform {
            translation: x.translation.lerp(y.translation, s),
            rotation: x.rotation.slerp(y.rotation, s),
            scale: x.scale.lerp(y.scale, s),
        })
        .collect()
}

/// A texture as linear RGB.
struct Texture {
    w: usize,
    h: usize,
    texels: Vec<[f32; 3]>,
}

impl Texture {
    fn at(&self, uv: [f32; 2]) -> [f32; 3] {
        let x = ((uv[0].rem_euclid(1.0) * self.w as f32) as usize).min(self.w - 1);
        let y = ((uv[1].rem_euclid(1.0) * self.h as f32) as usize).min(self.h - 1);
        self.texels[y * self.w + x]
    }
}

/// One character's view: its camera, and the buffers it's drawn into.
struct Panel {
    /// The way the character faces at rest, for the camera to stand in front of it.
    forward: Vec3,
    floor: f32,
    height: f32,
    center: Vec3,
    /// Each material's color texture, decoded once.
    textures: Vec<Option<Texture>>,
    color: Vec<[f32; 3]>,
    depth: Vec<f32>,
    srgb: Vec<u8>,
}

/// The camera: looking down a little, from the character's front left.
fn axes(forward: Vec3) -> (Vec3, Vec3, Vec3) {
    let yaw = forward.x.atan2(forward.z) + 25f32.to_radians();
    let pitch = 12f32.to_radians();
    let toward = Vec3::new(pitch.cos() * yaw.sin(), pitch.sin(), pitch.cos() * yaw.cos());
    let f = -toward;
    let r = f.cross(Vec3::Y).normalize();
    let u = r.cross(f);
    (f, r, u)
}

fn srgb_to_linear(c: u8) -> f32 {
    let c = f32::from(c) / 255.0;
    if c <= 0.04045 {
        c / 12.92
    } else {
        ((c + 0.055) / 1.055).powf(2.4)
    }
}

fn linear_to_srgb(c: f32) -> f32 {
    let c = c.clamp(0.0, 1.0);
    if c <= 0.003_130_8 {
        c * 12.92
    } else {
        1.055 * c.powf(1.0 / 2.4) - 0.055
    }
}

impl Panel {
    fn new(c: &Character) -> Self {
        let rest = c.textured.flatten(Pose::Rest).expect("draws").expect("has triangles");
        let (mut lo, mut hi) = (f32::MAX, f32::MIN);
        for p in rest.prims.iter().flat_map(|p| &p.positions) {
            lo = lo.min(p[1]);
            hi = hi.max(p[1]);
        }
        let lut: Vec<f32> = (0..=255u8).map(srgb_to_linear).collect();
        let textures = rest
            .materials
            .iter()
            .map(|m| {
                let img = image::load_from_memory(&rest.images.get(m.color_image?)?.bytes)
                    .ok()?
                    .to_rgb8();
                let texels = img
                    .pixels()
                    .map(|p| [lut[p[0] as usize], lut[p[1] as usize], lut[p[2] as usize]])
                    .collect();
                Some(Texture {
                    w: img.width() as usize,
                    h: img.height() as usize,
                    texels,
                })
            })
            .collect();
        Self {
            forward: c.model.skeleton.forward(),
            floor: lo,
            height: (hi - lo).max(1e-3),
            center: Vec3::ZERO,
            textures,
            color: vec![[0.0; 3]; W * SS * H * SS],
            depth: vec![f32::MAX; W * SS * H * SS],
            srgb: vec![0; W * H * 3],
        }
    }

    /// Moves the camera after the character across the floor, slowly enough not to shake with it.
    fn follow(&mut self, mesh: &Mesh, first: bool) {
        let (mut sum, mut n) = (Vec3::ZERO, 0.0);
        for p in mesh.prims.iter().flat_map(|p| &p.positions) {
            sum += Vec3::from_array(*p);
            n += 1.0;
        }
        let c = sum / f32::max(n, 1.0);
        let c = Vec3::new(c.x, self.floor + 0.5 * self.height, c.z);
        self.center = if first { c } else { self.center.lerp(c, 0.12) };
    }

    fn render(&mut self, mesh: &Mesh, label: Option<&(String, String)>) {
        let (w, h) = (W * SS, H * SS);
        let (f, r, u) = axes(self.forward);
        let k = h as f32 / (self.height * 1.35);
        let (cx, cy) = (w as f32 / 2.0, h as f32 * 0.5);
        let c = self.center;
        let screens: Vec<Vec<[f32; 3]>> = mesh
            .prims
            .iter()
            .map(|prim| {
                prim.positions
                    .iter()
                    .map(|p| {
                        let d = Vec3::from_array(*p) - c;
                        [cx + d.dot(r) * k, cy - d.dot(u) * k, d.dot(f)]
                    })
                    .collect()
            })
            .collect();
        let scene = Scene {
            mesh,
            screens: &screens,
            textures: &self.textures,
            axes: (f, r, u),
            k,
            middle: (cx, cy),
            center: c,
            floor: self.floor,
            height: self.height,
        };
        // Bands of rows, each drawn on its own thread.
        let rows = h.div_ceil(BANDS);
        std::thread::scope(|sc| {
            for (n, (color, depth)) in self
                .color
                .chunks_mut(rows * w)
                .zip(self.depth.chunks_mut(rows * w))
                .enumerate()
            {
                let scene = &scene;
                sc.spawn(move || scene.band(n * rows, color, depth));
            }
        });
        // Down to the panel's size, then the labels on top.
        for y in 0..H {
            for x in 0..W {
                let mut sum = [0.0; 3];
                for sy in 0..SS {
                    for sx in 0..SS {
                        let c = self.color[(y * SS + sy) * w + x * SS + sx];
                        for ch in 0..3 {
                            sum[ch] += c[ch];
                        }
                    }
                }
                for ch in 0..3 {
                    self.srgb[(y * W + x) * 3 + ch] = (linear_to_srgb(sum[ch] / (SS * SS) as f32) * 255.0 + 0.5) as u8;
                }
            }
        }
        if let Some((title, line)) = label {
            text(&mut self.srgb, 18, 18, 4, title, [255, 255, 255]);
            text(&mut self.srgb, 18, 56, 2, line, [170, 175, 185]);
        }
    }

    /// Copies the panel into place `p` of the frame.
    fn blit(&self, frame: &mut [u8], p: usize) {
        for y in 0..H {
            let row = &self.srgb[y * W * 3..(y + 1) * W * 3];
            let at = (y * 3 * W + p * W) * 3;
            frame[at..at + W * 3].copy_from_slice(row);
        }
        // A thin line between panels.
        if p > 0 {
            for y in 0..H {
                let at = (y * 3 * W + p * W) * 3;
                frame[at..at + 3].copy_from_slice(&[60, 62, 70]);
            }
        }
    }
}

/// How many bands of rows a panel is drawn in at once.
const BANDS: usize = 6;

/// What drawing a panel needs, shared by the threads drawing its bands.
struct Scene<'a> {
    mesh: &'a Mesh,
    /// Each primitive's vertices on screen: x, y, and depth.
    screens: &'a [Vec<[f32; 3]>],
    textures: &'a [Option<Texture>],
    axes: (Vec3, Vec3, Vec3),
    k: f32,
    middle: (f32, f32),
    center: Vec3,
    floor: f32,
    height: f32,
}

impl Scene<'_> {
    /// Draws rows from `y0` into `color` and `depth`, which hold those rows only.
    fn band(&self, y0: usize, color: &mut [[f32; 3]], depth: &mut [f32]) {
        let (w, h) = (W * SS, H * SS);
        let (f, r, u) = self.axes;
        let k = self.k;
        let (cx, cy) = self.middle;
        let c = self.center;
        let cell = self.height / 3.0;
        // The background and the floor, found by a ray through each pixel.
        for y in y0..y0 + depth.len() / w {
            for x in 0..w {
                let (a, b) = ((x as f32 + 0.5 - cx) / k, (cy - y as f32 - 0.5) / k);
                let q = c + a * r + b * u;
                let i = (y - y0) * w + x;
                let sky = 0.04 + 0.05 * (y as f32 / h as f32);
                let t = (self.floor - q.y) / f.y;
                if f.y < 0.0 && t > -10.0 * self.height {
                    let p = q + t * f;
                    let check = ((p.x / cell).floor() + (p.z / cell).floor()) as i64 & 1;
                    let base = if check == 0 { 0.30 } else { 0.22 };
                    let fade = (-(Vec3::new(p.x - c.x, 0.0, p.z - c.z).length() / (2.5 * self.height)).powi(2)).exp();
                    let v = sky + (base - sky) * fade;
                    color[i] = [v, v * 1.01, v * 1.04];
                    depth[i] = t;
                } else {
                    color[i] = [sky, sky, sky * 1.2];
                    depth[i] = f32::MAX;
                }
            }
        }
        // A soft shadow under the character.
        let shadow_r = 0.28 * self.height;
        for y in y0..y0 + depth.len() / w {
            for x in 0..w {
                let i = (y - y0) * w + x;
                if depth[i] == f32::MAX {
                    continue;
                }
                let (a, b) = ((x as f32 + 0.5 - cx) / k, (cy - y as f32 - 0.5) / k);
                let p = c + a * r + b * u + depth[i] * f;
                let d = Vec3::new(p.x - c.x, 0.0, p.z - c.z).length() / shadow_r;
                let s = 1.0 - 0.55 * (-d * d * 2.0).exp();
                for ch in &mut color[i] {
                    *ch *= s;
                }
            }
        }
        let light = (-0.45 * r + 0.75 * u - 0.5 * f).normalize();
        let view = -f;
        for (pi, prim) in self.mesh.prims.iter().enumerate() {
            let m = self.mesh.materials.get(prim.material);
            let base = m.map_or([0.7; 4], |m| m.color);
            let tex = self.textures.get(prim.material).and_then(Option::as_ref);
            let screen = &self.screens[pi];
            for tri in prim.indices.as_chunks::<3>().0 {
                let [i0, i1, i2] = tri.map(|i| i as usize);
                let (p0, p1, p2) = (screen[i0], screen[i1], screen[i2]);
                let area = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
                if area.abs() < 1e-9 {
                    continue;
                }
                let face = {
                    let (a, b, c) = (
                        Vec3::from_array(prim.positions[i0]),
                        Vec3::from_array(prim.positions[i1]),
                        Vec3::from_array(prim.positions[i2]),
                    );
                    (b - a).cross(c - a).normalize_or_zero()
                };
                let x0 = p0[0].min(p1[0]).min(p2[0]).floor().max(0.0) as usize;
                let x1 = (p0[0].max(p1[0]).max(p2[0]).ceil() as usize).min(w);
                let ya = (p0[1].min(p1[1]).min(p2[1]).floor().max(0.0) as usize).max(y0);
                let yb = (p0[1].max(p1[1]).max(p2[1]).ceil() as usize).min(y0 + depth.len() / w);
                for y in ya..yb {
                    for x in x0..x1 {
                        let (px, py) = (x as f32 + 0.5, y as f32 + 0.5);
                        let w0 = ((p1[0] - px) * (p2[1] - py) - (p1[1] - py) * (p2[0] - px)) / area;
                        let w1 = ((p2[0] - px) * (p0[1] - py) - (p2[1] - py) * (p0[0] - px)) / area;
                        let w2 = 1.0 - w0 - w1;
                        if w0 < 0.0 || w1 < 0.0 || w2 < 0.0 {
                            continue;
                        }
                        let z = w0 * p0[2] + w1 * p1[2] + w2 * p2[2];
                        let i = (y - y0) * w + x;
                        if z >= depth[i] {
                            continue;
                        }
                        depth[i] = z;
                        let mut n = if prim.normals.is_empty() {
                            face
                        } else {
                            let (a, b, c) = (prim.normals[i0], prim.normals[i1], prim.normals[i2]);
                            Vec3::new(
                                w0 * a[0] + w1 * b[0] + w2 * c[0],
                                w0 * a[1] + w1 * b[1] + w2 * c[1],
                                w0 * a[2] + w1 * b[2] + w2 * c[2],
                            )
                            .normalize_or_zero()
                        };
                        if n.dot(view) < 0.0 {
                            n = -n;
                        }
                        let mut albedo = [base[0], base[1], base[2]];
                        if let (Some(t), false) = (tex, prim.uvs.is_empty()) {
                            let (a, b, c) = (prim.uvs[i0], prim.uvs[i1], prim.uvs[i2]);
                            let s = t.at([w0 * a[0] + w1 * b[0] + w2 * c[0], w0 * a[1] + w1 * b[1] + w2 * c[1]]);
                            for ch in 0..3 {
                                albedo[ch] *= s[ch];
                            }
                        }
                        if !prim.colors.is_empty() {
                            let (a, b, c) = (prim.colors[i0], prim.colors[i1], prim.colors[i2]);
                            for ch in 0..3 {
                                albedo[ch] *= w0 * a[ch] + w1 * b[ch] + w2 * c[ch];
                            }
                        }
                        let diffuse = n.dot(light).max(0.0);
                        let rim = (1.0 - n.dot(view).clamp(0.0, 1.0)).powi(3) * 0.18;
                        let lit = 0.22 + 0.08 * n.y + 0.85 * diffuse;
                        color[i] = [
                            albedo[0] * lit + rim,
                            albedo[1] * lit + rim,
                            albedo[2] * lit + rim * 1.1,
                        ];
                    }
                }
            }
        }
    }
}

/// Writes `s` in a 5×7 pixel font, each pixel `scale` wide, its top left at (`x`, `y`), clipped to the panel.
fn text(buf: &mut [u8], x: usize, y: usize, scale: usize, s: &str, rgb: [u8; 3]) {
    let mut at = x;
    for ch in s.chars() {
        let rows = glyph(ch.to_ascii_uppercase());
        for (ry, row) in rows.iter().enumerate() {
            for rx in 0..5 {
                if row >> (4 - rx) & 1 == 0 {
                    continue;
                }
                for dy in 0..scale {
                    for dx in 0..scale {
                        let (px, py) = (at + rx * scale + dx, y + ry * scale + dy);
                        if px < W && py < H {
                            buf[(py * W + px) * 3..(py * W + px) * 3 + 3].copy_from_slice(&rgb);
                        }
                    }
                }
            }
        }
        at += 6 * scale;
    }
}

fn glyph(c: char) -> [u8; 7] {
    match c {
        'A' => [0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11],
        'B' => [0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E],
        'C' => [0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E],
        'D' => [0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E],
        'E' => [0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F],
        'F' => [0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10],
        'G' => [0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F],
        'H' => [0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11],
        'I' => [0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E],
        'J' => [0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C],
        'K' => [0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11],
        'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F],
        'M' => [0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11],
        'N' => [0x11, 0x11, 0x19, 0x15, 0x13, 0x11, 0x11],
        'O' => [0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'P' => [0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10],
        'Q' => [0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D],
        'R' => [0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11],
        'S' => [0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E],
        'T' => [0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        'U' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'V' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04],
        'W' => [0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A],
        'X' => [0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11],
        'Y' => [0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04],
        'Z' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F],
        '0' => [0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E],
        '1' => [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
        '2' => [0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F],
        '3' => [0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E],
        '4' => [0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02],
        '5' => [0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E],
        '6' => [0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E],
        '7' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
        '8' => [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
        '9' => [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C],
        '-' => [0, 0, 0, 0x1F, 0, 0, 0],
        '_' => [0, 0, 0, 0, 0, 0, 0x1F],
        '.' => [0, 0, 0, 0, 0, 0x0C, 0x0C],
        ':' => [0, 0x0C, 0x0C, 0, 0x0C, 0x0C, 0],
        '/' => [0, 0x01, 0x02, 0x04, 0x08, 0x10, 0],
        '|' => [0x04; 7],
        '(' => [0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02],
        ')' => [0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08],
        '%' => [0x18, 0x19, 0x02, 0x04, 0x08, 0x13, 0x03],
        '>' | '→' => [0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08],
        _ => [0; 7],
    }
}
