//! Clips baked into one flat library a shader plays: bake once on the CPU, play on the GPU.
//!
//! A [`Baker`] takes clips already on one skeleton (retargeted ones, every fix kept) and writes one library for that
//! rig: for every clip, every frame and every bone, the bone's turn from its bind pose in the world and where its
//! joint stands. That is all skinning needs, so the shader walks no hierarchy. [`Library`] reads the bytes back and
//! samples them on the CPU, and [`wgsl`] is the same sampler as WGSL, function for function, for WebGPU.
//!
//! # Layout
//!
//! Little-endian 32-bit words. Every offset is a count of words from the library's first word, so a library is
//! found by where it starts alone: in a storage buffer of its own today, in a pool of many addressed by buffer
//! device address later. With `C` clips, `B` bones and `F` frames in all:
//!
//! | Part | Words | Holds |
//! | --- | ---: | --- |
//! | Header | 8 | [`MAGIC`], [`VERSION`], encoding (0 floats, 1 halves), `B`, `C`, the library's words, the clip table's offset, the pivots' offset |
//! | Clip table | 4 × `C` | per clip: its first record's offset, its frames, frames a second (a float), flags (bit 0: it loops) |
//! | Pivots | 3 × `B` | per bone: where its joint stands at bind (x, y, z as floats, in either encoding) |
//! | Records | `R` × `B` × `F` | per clip, per frame, per bone: rotation x, y, z, w, then the joint's place x, y, z |
//!
//! A record is `R` = 7 words as floats ([`Encoding::F32`]) and 4 as halves ([`Encoding::Half`]): eight 16-bit halves,
//! the low half of each word first, the eighth zero. A library is `4 × (8 + 4C + 3B + R·B·F)` bytes ([`size`]). Fifty
//! clips of 3 s at 30 Hz on 80 bones, looping (90 frames each), are 5,761,792 bytes as halves and 10,081,792 as
//! floats; played once through (91 frames each), 5,825,792 and 10,193,792.
//!
//! # What a record means
//!
//! A vertex `v` bound to a bone goes to `rotation × (v − pivot) + place`: the bone's world transform times its
//! inverse bind, kept as a turn about its own joint. [`Library::bone`] and the shader's `bone` hand that back as a
//! rotation and a translation, `place − rotation × pivot`, ready for linear-blend skinning. The joint's place is
//! stored rather than that translation because it is what blends well: between two frames, or two clips, a joint
//! moves straight from one place to the other, where the translation is the small difference of two long arms and
//! bends the body toward the rig's origin when blended (two to five times further off half way between frames, on
//! made-up clips). It rounds better too: a half's error in the turn moves a fingertip by a finger's length of it, not
//! by its distance from the origin.
//!
//! Floats give back a retargeted clip's frames to a millionth of the rig's height. A half has 11 bits, so it places
//! a joint within a 2048th of how far it stands from the origin along each axis: 0.0006 of the height at the
//! fingertips on a clip played in place, 0.0009 on one that walks three heights away. Bake clips that travel far as
//! floats, or in place with the travel left to the host. Between frames the sampler moves each joint straight and
//! turns each bone in the world, where the clip turns it under its parent: at 30 frames a second that is under a
//! thousandth of the height on a walk, and up to four on a clip with every bone swinging hard.
//!
//! No scale is stored. A rig's scales at bind cancel out of the product, so they cost nothing, as long as they are
//! positive and the same along every axis under any bone that turns. Scale a clip animates is dropped: each bone
//! keeps its bind size and turns about its joint, which still stands where the clip puts it, so bones below a
//! scaled one are placed right but the skin on the scaled bone itself is not stretched. Scales that differ by axis
//! shear what's below them once it turns; the shear is dropped the same way. Retargeted clips keep the target's own
//! scales, so none of this touches them.
//!
//! Each bone's rotations are kept in one hemisphere from frame to frame (the first frame's nearest no turn at all), so
//! a host may blend neighboring frames linearly with no check of its own, in a texture filter say. The sampler here
//! checks anyway, as one clip's last frame and first, or two clips, may still lie in opposite ones.
//!
//! # Time
//!
//! A clip is sampled again at a steady rate: the nearest to the rate asked for that fits its length in a whole
//! number of frames, so the last frame is the clip's last pose. A looping clip's last pose is taken to be its first
//! again, as loops are keyed, and isn't stored: frame `F` is frame 0. Time is in seconds from the clip's first pose;
//! a looping clip wraps it and any other holds its first pose before and its last after.
//!
//! # On the GPU
//!
//! Upload the bytes as they are to a buffer with storage usage and bind it read-only; [`wgsl`] declares it as
//! `var<storage, read> library: array<u32>`. One binding serves both encodings: halves are unpacked by a small
//! function, since WGSL's `f16` is an extension. Each instance then needs only a clip, a time, and for a cross-fade a
//! second clip and time and a weight: call `bone` or `blend` per influence in a vertex shader, or once per bone in a
//! compute pass that writes a pose buffer, and `skin` to place the vertex. WebGPU's compatibility mode allows no
//! storage buffers in the vertex stage, so there it is the compute pass.
//!
//! The WGSL is parsed and validated by naga in this crate's tests. It has not been run on a GPU here: that it gives
//! what [`Library`] gives rests on the two being written side by side.

use crate::{Clip, Mat4, Quat, Skeleton, Transform, Vec3};

/// A library's first word: `DTMO` as bytes.
pub const MAGIC: u32 = u32::from_le_bytes(*b"DTMO");

/// The layout's version, a library's second word.
pub const VERSION: u32 = 1;

/// Words in the header, in a clip's row of the clip table, and in a bone's pivot.
const HEADER: usize = 8;
const ENTRY: usize = 4;
const PIVOT: usize = 3;

/// The clip table's flag for a clip that loops.
const LOOPS: u32 = 1;

/// The least a half above zero holds, 2⁻²⁴.
const TINY: f32 = 5.960_464_5e-8;

/// How a library keeps its numbers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Encoding {
    /// 32-bit floats: seven words a record.
    F32,
    /// 16-bit half floats: four words a record. A joint is placed within a 2048th of how far it stands from the origin.
    Half,
}

impl Encoding {
    /// Words in one bone's record for one frame.
    #[must_use]
    pub const fn record(self) -> usize {
        match self {
            Self::F32 => 7,
            Self::Half => 4,
        }
    }

    /// As the header's third word.
    const fn code(self) -> u32 {
        match self {
            Self::F32 => 0,
            Self::Half => 1,
        }
    }
}

/// What can go wrong baking a library or reading one.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    /// A clip doesn't pose the skeleton, or has no times or times that don't rise.
    Clip(crate::Error),
    /// The frames a second asked for aren't a positive number.
    Rate,
    /// There isn't one inverse bind matrix per bone.
    Binds { binds: usize, expected: usize },
    /// The library would pass what 32-bit offsets reach.
    TooBig,
    /// The bytes don't start with [`MAGIC`].
    NotALibrary,
    /// The library is of a version this code doesn't read.
    Version(u32),
    /// The bytes stop short of what the header or a clip says is there.
    Truncated,
    /// The header or the clip table makes no sense: an encoding that isn't one, a part outside the library, a clip
    /// of no frames or a rate that isn't a positive number.
    Malformed,
}

impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Clip(e) => write!(f, "couldn't bake the clip: {e}"),
            Self::Rate => write!(f, "the frames a second aren't a positive number"),
            Self::Binds { binds, expected } => {
                write!(f, "{binds} inverse bind matrices, the skeleton has {expected} bones")
            }
            Self::TooBig => write!(f, "the library would be over 4 billion words"),
            Self::NotALibrary => write!(f, "not a baked clip library"),
            Self::Version(v) => write!(f, "a library of version {v}, this reads version {VERSION}"),
            Self::Truncated => write!(f, "the library is cut short"),
            Self::Malformed => write!(f, "the library's header or clip table makes no sense"),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Clip(e) => Some(e),
            _ => None,
        }
    }
}

/// How many frames a clip `seconds` long is stored as, sampled about `rate` times a second: one at each end and those
/// between, or for a looping clip one fewer, the last being the first again. Never none.
#[must_use]
pub fn frames(seconds: f32, rate: f32, looping: bool) -> usize {
    let steps = seconds * rate;
    if steps.is_nan() || steps <= 0.0 {
        return 1;
    }
    if looping {
        (steps.round() as usize).max(1)
    } else {
        (steps - 1e-3).ceil().max(0.0) as usize + 1
    }
}

/// A library's size in bytes: `4 × (8 + 4 × clips + 3 × bones + record × bones × frames)`, `frames` being every
/// clip's frames together ([`frames`]).
#[must_use]
pub const fn size(encoding: Encoding, bones: usize, clips: usize, frames: usize) -> usize {
    4 * (HEADER + ENTRY * clips + PIVOT * bones + encoding.record() * bones * frames)
}

/// Bakes clips on one skeleton into a library.
///
/// ```
/// # use dittomo::{Bone, Clip, Skeleton, Transform, Vec3};
/// # use dittomo::bake::{Baker, Encoding, Library};
/// # let bone = |name: &str, parent| Bone { name: name.into(), parent, rest: Transform::new(Vec3::Y, Default::default()) };
/// # let rig = Skeleton::new(vec![bone("hips", None), bone("spine", Some(0))]).unwrap();
/// # let walk = Clip { times: vec![0.0, 1.0], poses: vec![rig.rest(), rig.rest()] };
/// let mut baker = Baker::new(&rig, Encoding::Half);
/// let clip = baker.add(&walk, 30.0, true).unwrap();
/// let bytes = baker.finish();
/// // Upload `bytes` to a storage buffer; on the CPU:
/// let library = Library::read(&bytes).unwrap();
/// let spine = library.bone(clip as u32, 0.5, 1);
/// ```
#[derive(Clone, Debug)]
pub struct Baker {
    skeleton: Skeleton,
    encoding: Encoding,
    /// Each bone's turn at bind, undone.
    unbind: Vec<Quat>,
    /// Where each bone's joint stands at bind.
    pivots: Vec<Vec3>,
    /// Each clip's row of the clip table, its first record counted from where the records begin.
    table: Vec<[u32; ENTRY]>,
    records: Vec<u32>,
}

impl Baker {
    /// A baker for clips on `skeleton`, bound at its rest pose.
    #[must_use]
    pub fn new(skeleton: &Skeleton, encoding: Encoding) -> Self {
        let rest = skeleton.world(&skeleton.rest());
        Self {
            skeleton: skeleton.clone(),
            encoding,
            unbind: rest.iter().map(|p| p.rotation.inverse()).collect(),
            pivots: rest.iter().map(crate::Placed::position).collect(),
            table: Vec::new(),
            records: Vec::new(),
        }
    }

    /// Bound by a mesh's inverse bind matrices instead, one per bone, where they differ from the rest pose. Call it
    /// before adding clips.
    ///
    /// # Errors
    /// [`Error::Binds`] if there isn't one per bone.
    pub fn bound(mut self, inverse_binds: &[Mat4]) -> Result<Self, Error> {
        if inverse_binds.len() != self.skeleton.len() {
            return Err(Error::Binds {
                binds: inverse_binds.len(),
                expected: self.skeleton.len(),
            });
        }
        for (b, inverse) in inverse_binds.iter().enumerate() {
            let (_, turn, at) = inverse.inverse().to_scale_rotation_translation();
            self.unbind[b] = turn.normalize().inverse();
            self.pivots[b] = at;
        }
        Ok(self)
    }

    /// Adds `clip`, sampled about `rate` times a second ([`frames`] says how many frames that makes), and says which
    /// clip of the library it is. A `looping` clip's last pose is taken to be its first again.
    ///
    /// # Errors
    /// [`Error::Clip`] if the clip doesn't pose the skeleton or its times are none or don't rise, [`Error::Rate`] if
    /// `rate` isn't a positive number, [`Error::TooBig`] if the library would pass 32-bit offsets.
    pub fn add(&mut self, clip: &Clip, rate: f32, looping: bool) -> Result<usize, Error> {
        clip.check(self.skeleton.len()).map_err(Error::Clip)?;
        let times = &clip.times;
        if times.is_empty() || times.windows(2).any(|w| w[1] <= w[0]) || !times.iter().all(|t| t.is_finite()) {
            return Err(Error::Clip(crate::Error::ClipTimes));
        }
        if !(rate.is_finite() && rate > 0.0) {
            return Err(Error::Rate);
        }
        let seconds = times[times.len() - 1] - times[0];
        let count = frames(seconds, rate, looping);
        // The rate that fits: the last frame on the last pose, or for a loop the frame after it on the first.
        let rate = match (count, looping) {
            (1, _) => rate,
            (_, true) => count as f32 / seconds,
            (_, false) => (count - 1) as f32 / seconds,
        };
        let bones = self.skeleton.len();
        let words = (count.checked_mul(bones))
            .and_then(|n| n.checked_mul(self.encoding.record()))
            .and_then(|n| n.checked_add(self.records.len()))
            .and_then(|n| n.checked_add(HEADER + ENTRY * (self.table.len() + 1) + PIVOT * bones))
            .ok_or(Error::TooBig)?;
        if u32::try_from(words).is_err() {
            return Err(Error::TooBig);
        }

        let first = self.records.len() as u32;
        let mut last = vec![Quat::IDENTITY; bones];
        for k in 0..count {
            let pose = crate::bvh::sample(clip, times[0] + k as f32 / rate);
            let world = self.skeleton.world(&pose);
            for (b, placed) in world.iter().enumerate() {
                let mut turn = (placed.rotation * self.unbind[b]).normalize();
                if turn.dot(last[b]) < 0.0 {
                    turn = -turn;
                }
                last[b] = turn;
                self.write(turn, placed.position());
            }
        }
        self.table
            .push([first, count as u32, rate.to_bits(), if looping { LOOPS } else { 0 }]);
        Ok(self.table.len() - 1)
    }

    /// One record: a bone's rotation and its joint's place.
    fn write(&mut self, turn: Quat, at: Vec3) {
        match self.encoding {
            Encoding::F32 => {
                let numbers = [turn.x, turn.y, turn.z, turn.w, at.x, at.y, at.z];
                self.records.extend(numbers.map(f32::to_bits));
            }
            Encoding::Half => {
                let pairs = [[turn.x, turn.y], [turn.z, turn.w], [at.x, at.y], [at.z, 0.0]];
                let word = |[low, high]: [f32; 2]| u32::from(to_half(low)) | u32::from(to_half(high)) << 16;
                self.records.extend(pairs.map(word));
            }
        }
    }

    /// The library's bytes, with every clip added so far.
    #[must_use]
    pub fn finish(&self) -> Vec<u8> {
        let (bones, clips) = (self.skeleton.len(), self.table.len());
        let table = HEADER;
        let pivots = table + ENTRY * clips;
        let records = pivots + PIVOT * bones;
        let words = records + self.records.len();
        let mut out: Vec<u32> = Vec::with_capacity(words);
        out.extend([MAGIC, VERSION, self.encoding.code()]);
        out.extend([bones, clips, words, table, pivots].map(|n| n as u32));
        for row in &self.table {
            out.extend([row[0] + records as u32, row[1], row[2], row[3]]);
        }
        out.extend(self.pivots.iter().flat_map(Vec3::to_array).map(f32::to_bits));
        out.extend(&self.records);
        out.iter().flat_map(|w| w.to_le_bytes()).collect()
    }
}

/// `v` as the nearest half float's bits, ties to even. What a half can't hold becomes the biggest it can, and what
/// isn't a number zero, so every half written is finite.
fn to_half(v: f32) -> u16 {
    if v.is_nan() {
        return 0;
    }
    let bits = v.to_bits();
    let sign = ((bits >> 16) & 0x8000) as u16;
    // 65520 is half way from the biggest half to what would be the next, and rounds up to it.
    if v.abs() >= 65520.0 {
        return sign | 0x7BFF;
    }
    let exponent = ((bits >> 23) & 0xFF) as i32 - 127;
    let mantissa = bits & 0x007F_FFFF;
    if exponent < -25 {
        return sign;
    }
    // What's kept, what's cut off, and half of the last place kept.
    let (kept, cut, half) = if exponent < -14 {
        // Below the smallest normal half: counted in steps of 2⁻²⁴.
        let shift = (-1 - exponent) as u32;
        let whole = mantissa | 0x0080_0000;
        (whole >> shift, whole & ((1 << shift) - 1), 1 << (shift - 1))
    } else {
        (
            ((exponent + 15) as u32) << 10 | mantissa >> 13,
            mantissa & 0x1FFF,
            0x1000,
        )
    };
    // Rounding up may carry into the exponent, which is then the right one.
    let up = cut > half || (cut == half && kept & 1 == 1);
    sign | (kept + u32::from(up)) as u16
}

/// A half float's value from its 16 bits. As the shader's `half`: what the baker never writes, infinities and
/// not-a-numbers, come out as numbers past the biggest half.
fn half(h: u32) -> f32 {
    let e = (h >> 10) & 31;
    let m = h & 1023;
    let v = if e == 0 {
        m as f32 * TINY
    } else {
        f32::from_bits((e + 112) << 23 | m << 13)
    };
    if h & 0x8000 != 0 { -v } else { v }
}

/// From `a` toward `b` by `s`, the short way round, as a rotation. As the shader's `nlerp`.
fn nlerp(a: Quat, b: Quat, s: f32) -> Quat {
    let toward = if a.dot(b) < 0.0 { -b } else { b };
    (a + (toward - a) * s).normalize()
}

/// A library's header, word for word.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    pub magic: u32,
    pub version: u32,
    /// 0 for floats, 1 for halves.
    pub encoding: u32,
    pub bones: u32,
    pub clips: u32,
    /// The whole library's length in words.
    pub words: u32,
    /// The offset of the clip table.
    pub table: u32,
    /// The offset of the pivots.
    pub pivots: u32,
}

/// A clip's row of the clip table, word for word.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Entry {
    /// The offset of its first frame's first record.
    pub first: u32,
    pub frames: u32,
    /// Frames a second.
    pub rate: f32,
    /// Bit 0: it loops.
    pub flags: u32,
}

impl Entry {
    #[must_use]
    pub fn loops(&self) -> bool {
        self.flags & LOOPS != 0
    }

    /// How long the clip plays: once round for one that loops.
    #[must_use]
    pub fn seconds(&self) -> f32 {
        let steps = if self.loops() {
            self.frames
        } else {
            self.frames.saturating_sub(1)
        };
        steps as f32 / self.rate
    }
}

/// A library read back: the reference the shader must match. Every index is clamped to what the library holds, as a
/// shader's is, so no call panics.
#[derive(Clone, Copy, Debug)]
pub struct Library<'a> {
    bytes: &'a [u8],
    header: Header,
}

impl<'a> Library<'a> {
    /// The library these bytes start with, checked through: every part and every clip's records lie inside it. Bytes
    /// past its end are let be, so a library may be read out of a pool.
    ///
    /// # Errors
    /// [`Error::Truncated`] if the bytes stop short, [`Error::NotALibrary`] or [`Error::Version`] if they start with
    /// another magic or version, [`Error::Malformed`] if the header or clip table makes no sense.
    pub fn read(bytes: &'a [u8]) -> Result<Self, Error> {
        if bytes.len() < 4 * HEADER {
            return Err(Error::Truncated);
        }
        let mut library = Self {
            bytes,
            header: Header {
                magic: 0,
                version: 0,
                encoding: 0,
                bones: 0,
                clips: 0,
                words: HEADER as u32,
                table: 0,
                pivots: 0,
            },
        };
        let h = library.read_header();
        if h.magic != MAGIC {
            return Err(Error::NotALibrary);
        }
        if h.version != VERSION {
            return Err(Error::Version(h.version));
        }
        if bytes.len() / 4 < h.words as usize {
            return Err(Error::Truncated);
        }
        let (words, bones) = (u64::from(h.words), u64::from(h.bones));
        let within = |at: u32, len: u64| u64::from(at) >= HEADER as u64 && u64::from(at) + len <= words;
        let parts = within(h.table, ENTRY as u64 * u64::from(h.clips)) && within(h.pivots, PIVOT as u64 * bones);
        if h.encoding > 1 || !parts {
            return Err(Error::Malformed);
        }
        library.header = h;
        let record = if h.encoding == 1 { 4 } else { 7 };
        for clip in 0..h.clips {
            let c = library.clip_entry(h.table, clip);
            // No overflow: three 32-bit counts and a 3-bit one.
            let len = u128::from(c.frames) * u128::from(h.bones) * record;
            let fits = u128::from(c.first) >= HEADER as u128 && u128::from(c.first) + len <= u128::from(h.words);
            if c.frames == 0 || !(c.rate.is_finite() && c.rate > 0.0) || !fits {
                return Err(Error::Malformed);
            }
        }
        Ok(library)
    }

    #[must_use]
    pub fn header(&self) -> Header {
        self.header
    }

    #[must_use]
    pub fn encoding(&self) -> Encoding {
        if self.header.encoding == 1 {
            Encoding::Half
        } else {
            Encoding::F32
        }
    }

    /// How many bones each frame poses.
    #[must_use]
    pub fn bones(&self) -> usize {
        self.header.bones as usize
    }

    /// How many clips it holds.
    #[must_use]
    pub fn clips(&self) -> usize {
        self.header.clips as usize
    }

    /// A clip's row of the clip table, if it holds that clip.
    #[must_use]
    pub fn entry(&self, clip: usize) -> Option<Entry> {
        (clip < self.clips()).then(|| self.clip_entry(self.header.table, clip as u32))
    }

    /// Where a bone's joint stands at bind, if it has that bone.
    #[must_use]
    pub fn pivot_of(&self, bone: usize) -> Option<Vec3> {
        (bone < self.bones()).then(|| self.pivot(self.header.pivots, bone as u32))
    }

    /// A record as stored: a bone's rotation and where its joint stands (as the translation) at one frame of a clip.
    /// For a host that lays the records out its own way, as a texture.
    #[must_use]
    pub fn stored(&self, clip: usize, frame: usize, bone: usize) -> Option<Transform> {
        let c = self.entry(clip)?;
        let h = self.header;
        (frame < c.frames as usize && bone < self.bones())
            .then(|| self.record(c.first, frame as u32 * h.bones + bone as u32, h.encoding))
    }

    // The sampler. Each function below is the shader's of the same name, line for line: change both or neither.

    /// A word of the library, zero past its end. As the shader's `library[at]`.
    fn word(&self, at: u32) -> u32 {
        let at = at as usize * 4;
        if at / 4 >= self.header.words as usize {
            return 0;
        }
        (self.bytes.get(at..at + 4)).map_or(0, |b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    fn read_header(&self) -> Header {
        Header {
            magic: self.word(0),
            version: self.word(1),
            encoding: self.word(2),
            bones: self.word(3),
            clips: self.word(4),
            words: self.word(5),
            table: self.word(6),
            pivots: self.word(7),
        }
    }

    fn clip_entry(&self, table: u32, clip: u32) -> Entry {
        let at = table + 4 * clip;
        Entry {
            first: self.word(at),
            frames: self.word(at + 1),
            rate: f32::from_bits(self.word(at + 2)),
            flags: self.word(at + 3),
        }
    }

    fn pivot(&self, pivots: u32, bone: u32) -> Vec3 {
        let at = pivots + 3 * bone;
        Vec3::new(
            f32::from_bits(self.word(at)),
            f32::from_bits(self.word(at + 1)),
            f32::from_bits(self.word(at + 2)),
        )
    }

    /// The `index`th record from `first`: a rotation and a joint's place.
    fn record(&self, first: u32, index: u32, encoding: u32) -> Transform {
        if encoding == 1 {
            let at = first + 4 * index;
            let (a, b) = (self.word(at), self.word(at + 1));
            let (c, d) = (self.word(at + 2), self.word(at + 3));
            return Transform::new(
                Vec3::new(half(c & 0xFFFF), half(c >> 16), half(d & 0xFFFF)),
                Quat::from_xyzw(half(a & 0xFFFF), half(a >> 16), half(b & 0xFFFF), half(b >> 16)),
            );
        }
        let at = first + 7 * index;
        Transform::new(
            Vec3::new(
                f32::from_bits(self.word(at + 4)),
                f32::from_bits(self.word(at + 5)),
                f32::from_bits(self.word(at + 6)),
            ),
            Quat::from_xyzw(
                f32::from_bits(self.word(at)),
                f32::from_bits(self.word(at + 1)),
                f32::from_bits(self.word(at + 2)),
                f32::from_bits(self.word(at + 3)),
            ),
        )
    }

    /// A bone's rotation from bind and where its joint stands (as the translation), `time` seconds into a clip.
    #[must_use]
    pub fn joint(&self, clip: u32, time: f32, bone: u32) -> Transform {
        let h = self.header;
        if h.bones == 0 {
            return Transform::IDENTITY;
        }
        let b = bone.min(h.bones - 1);
        if h.clips == 0 {
            return Transform::new(self.pivot(h.pivots, b), Quat::IDENTITY);
        }
        let c = self.clip_entry(h.table, clip.min(h.clips - 1));
        let n = c.frames as f32;
        let last = c.frames - 1;
        let mut x = time * c.rate;
        let mut next = 1;
        if c.flags & 1 != 0 {
            x = (x - (x / n).floor() * n).max(0.0);
            next = c.frames;
        } else {
            x = x.clamp(0.0, last as f32);
        }
        let i = (x as u32).min(last);
        let j = if next == 1 { (i + 1).min(last) } else { (i + 1) % next };
        let s = x - i as f32;
        let here = self.record(c.first, i * h.bones + b, h.encoding);
        let there = self.record(c.first, j * h.bones + b, h.encoding);
        Transform::new(
            here.translation + (there.translation - here.translation) * s,
            nlerp(here.rotation, there.rotation, s),
        )
    }

    /// A joint as what skinning takes: its rotation, and the translation that keeps the joint where it stands.
    fn bound(&self, joint: Transform, bone: u32) -> Transform {
        let h = self.header;
        if h.bones == 0 {
            return joint;
        }
        let pivot = self.pivot(h.pivots, bone.min(h.bones - 1));
        Transform::new(joint.translation - joint.rotation * pivot, joint.rotation)
    }

    /// A bone's skinning transform `time` seconds into a clip: its world transform times its inverse bind, as a
    /// rotation and a translation. A point `v` of the bind pose goes to `rotation × v + translation`.
    #[must_use]
    pub fn bone(&self, clip: u32, time: f32, bone: u32) -> Transform {
        self.bound(self.joint(clip, time, bone), bone)
    }

    /// A bone's skinning transform part way from one clip to another: all the first at a `weight` of 0, all the
    /// second at 1. Each joint goes straight from where one clip has it to where the other does, so bones between
    /// poses far apart come out a little short: it is for fading between poses much alike, over a short time.
    #[must_use]
    pub fn blend(&self, clip_a: u32, time_a: f32, clip_b: u32, time_b: f32, weight: f32, bone: u32) -> Transform {
        let a = self.joint(clip_a, time_a, bone);
        let b = self.joint(clip_b, time_b, bone);
        let joint = Transform::new(
            a.translation + (b.translation - a.translation) * weight,
            nlerp(a.rotation, b.rotation, weight),
        );
        self.bound(joint, bone)
    }
}

/// Where a point of the bind pose goes, moved by up to four bones' skinning transforms ([`Library::bone`]) by these
/// weights: linear-blend skinning. As the shader's `skin`.
#[must_use]
pub fn skin(point: Vec3, bones: &[Transform; 4], weights: [f32; 4]) -> Vec3 {
    let mut out = Vec3::ZERO;
    for (bone, weight) in bones.iter().zip(weights) {
        out += (bone.rotation * point + bone.translation) * weight;
    }
    out
}

/// The sampler as WGSL, to go before a host's own shader: the structs, `bone`, `blend`, `skin` and `skin_normal`,
/// and the library declared as `@group(group) @binding(binding) var<storage, read> library: array<u32>`. [`WGSL`] is
/// the same without that line, for a host that declares `library` itself.
#[must_use]
pub fn wgsl(group: u32, binding: u32) -> String {
    format!("@group({group}) @binding({binding}) var<storage, read> library: array<u32>;\n{WGSL}")
}

/// The sampler as WGSL, less the library's declaration ([`wgsl`] adds it). Each function is [`Library`]'s of the same
/// name, line for line.
pub const WGSL: &str = r"
// dittomo's baked clip library, version 1, read from `library: array<u32>`. Offsets count words from its start.

struct Header {
    magic: u32,
    version: u32,
    // 0 for floats, 1 for halves.
    encoding: u32,
    bones: u32,
    clips: u32,
    // The whole library's length in words.
    words: u32,
    // The offsets of the clip table and of the pivots.
    table: u32,
    pivots: u32,
}

struct Entry {
    // The offset of the clip's first frame's first record.
    first: u32,
    frames: u32,
    // Frames a second.
    rate: f32,
    // Bit 0: it loops.
    flags: u32,
}

// A rotation (a quaternion, x y z w) and a translation.
struct Bone {
    rotation: vec4<f32>,
    translation: vec3<f32>,
}

// A half float's value from its 16 bits.
fn half(h: u32) -> f32 {
    let e = (h >> 10u) & 31u;
    let m = h & 1023u;
    var v = f32(m) * 5.9604644775390625e-8;
    if e != 0u {
        v = bitcast<f32>(((e + 112u) << 23u) | (m << 13u));
    }
    return select(v, -v, (h & 0x8000u) != 0u);
}

// From `a` toward `b` by `s`, the short way round, as a rotation.
fn nlerp(a: vec4<f32>, b: vec4<f32>, s: f32) -> vec4<f32> {
    let toward = select(b, -b, dot(a, b) < 0.0);
    return normalize(a + (toward - a) * s);
}

// `v` turned by the rotation `q`.
fn turn(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

fn read_header() -> Header {
    return Header(
        library[0],
        library[1],
        library[2],
        library[3],
        library[4],
        library[5],
        library[6],
        library[7],
    );
}

fn clip_entry(table: u32, clip: u32) -> Entry {
    let at = table + 4u * clip;
    return Entry(library[at], library[at + 1u], bitcast<f32>(library[at + 2u]), library[at + 3u]);
}

fn pivot(pivots: u32, bone: u32) -> vec3<f32> {
    let at = pivots + 3u * bone;
    return vec3<f32>(bitcast<f32>(library[at]), bitcast<f32>(library[at + 1u]), bitcast<f32>(library[at + 2u]));
}

// The `index`th record from `first`: a rotation and a joint's place.
fn record(first: u32, index: u32, encoding: u32) -> Bone {
    if encoding == 1u {
        let at = first + 4u * index;
        let a = library[at];
        let b = library[at + 1u];
        let c = library[at + 2u];
        let d = library[at + 3u];
        return Bone(
            vec4<f32>(half(a & 0xFFFFu), half(a >> 16u), half(b & 0xFFFFu), half(b >> 16u)),
            vec3<f32>(half(c & 0xFFFFu), half(c >> 16u), half(d & 0xFFFFu)),
        );
    }
    let at = first + 7u * index;
    return Bone(
        vec4<f32>(
            bitcast<f32>(library[at]),
            bitcast<f32>(library[at + 1u]),
            bitcast<f32>(library[at + 2u]),
            bitcast<f32>(library[at + 3u]),
        ),
        vec3<f32>(bitcast<f32>(library[at + 4u]), bitcast<f32>(library[at + 5u]), bitcast<f32>(library[at + 6u])),
    );
}

// A bone's rotation from bind and where its joint stands (as the translation), `time` seconds into a clip.
fn joint(clip: u32, time: f32, bone: u32) -> Bone {
    let h = read_header();
    if h.bones == 0u {
        return Bone(vec4<f32>(0.0, 0.0, 0.0, 1.0), vec3<f32>(0.0));
    }
    let b = min(bone, h.bones - 1u);
    if h.clips == 0u {
        return Bone(vec4<f32>(0.0, 0.0, 0.0, 1.0), pivot(h.pivots, b));
    }
    let c = clip_entry(h.table, min(clip, h.clips - 1u));
    let n = f32(c.frames);
    let last = c.frames - 1u;
    var x = time * c.rate;
    var next = 1u;
    if (c.flags & 1u) != 0u {
        x = max(x - floor(x / n) * n, 0.0);
        next = c.frames;
    } else {
        x = clamp(x, 0.0, f32(last));
    }
    let i = min(u32(x), last);
    var j = min(i + 1u, last);
    if next != 1u {
        j = (i + 1u) % next;
    }
    let s = x - f32(i);
    let here = record(c.first, i * h.bones + b, h.encoding);
    let there = record(c.first, j * h.bones + b, h.encoding);
    return Bone(nlerp(here.rotation, there.rotation, s), here.translation + (there.translation - here.translation) * s);
}

// A joint as what skinning takes: its rotation, and the translation that keeps the joint where it stands.
fn bound(at: Bone, bone: u32) -> Bone {
    let h = read_header();
    if h.bones == 0u {
        return at;
    }
    let p = pivot(h.pivots, min(bone, h.bones - 1u));
    return Bone(at.rotation, at.translation - turn(at.rotation, p));
}

// A bone's skinning transform `time` seconds into a clip: its world transform times its inverse bind. A point `v` of
// the bind pose goes to `turn(rotation, v) + translation`.
fn bone(clip: u32, time: f32, index: u32) -> Bone {
    return bound(joint(clip, time, index), index);
}

// A bone's skinning transform part way from one clip to another: all the first at `weight` 0, all the second at 1.
fn blend(clip_a: u32, time_a: f32, clip_b: u32, time_b: f32, weight: f32, index: u32) -> Bone {
    let a = joint(clip_a, time_a, index);
    let b = joint(clip_b, time_b, index);
    let at = Bone(nlerp(a.rotation, b.rotation, weight), a.translation + (b.translation - a.translation) * weight);
    return bound(at, index);
}

// Where a point of the bind pose goes, moved by four bones' skinning transforms by these weights.
fn skin(point: vec3<f32>, a: Bone, b: Bone, c: Bone, d: Bone, weights: vec4<f32>) -> vec3<f32> {
    return (turn(a.rotation, point) + a.translation) * weights.x
        + (turn(b.rotation, point) + b.translation) * weights.y
        + (turn(c.rotation, point) + c.translation) * weights.z
        + (turn(d.rotation, point) + d.translation) * weights.w;
}

// Which way a normal of the bind pose faces, turned by the same four bones.
fn skin_normal(normal: vec3<f32>, a: Bone, b: Bone, c: Bone, d: Bone, weights: vec4<f32>) -> vec3<f32> {
    return normalize(
        turn(a.rotation, normal) * weights.x
            + turn(b.rotation, normal) * weights.y
            + turn(c.rotation, normal) * weights.z
            + turn(d.rotation, normal) * weights.w,
    );
}
";

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn halves_of_known_values() {
        assert_eq!(TINY, 2f32.powi(-24));
        assert!(
            WGSL.contains("5.9604644775390625e-8"),
            "the shader's is the same, written out in full"
        );
        for (v, h) in [
            (0.0, 0x0000),
            (-0.0, 0x8000),
            (1.0, 0x3C00),
            (-2.0, 0xC000),
            (0.5, 0x3800),
            (0.1, 0x2E66),
            (1.0 / 3.0, 0x3555),
            (65504.0, 0x7BFF),
            (1024.0 * TINY, 0x0400), // the smallest normal half, 2⁻¹⁴
            (1023.0 * TINY, 0x03FF), // the biggest below it
            (TINY, 0x0001),
            (-TINY, 0x8001),
        ] {
            assert_eq!(to_half(v), h, "{v} as a half");
            // A tenth and a third aren't halves: they come back as the half nearest them.
            let back = half(u32::from(h));
            assert!((back - v).abs() <= v.abs() * 2f32.powi(-11), "{h:#06x} as a float");
            assert!(
                back.to_bits() == v.to_bits() || h == 0x2E66 || h == 0x3555,
                "{h:#06x} as a float"
            );
        }
    }

    #[test]
    fn halves_round_to_the_nearest_and_ties_to_even() {
        // Half way between two halves goes to the even one; a hair either side goes to the nearer.
        let step = 2f32.powi(-10);
        assert_eq!(to_half(1.0 + step * 0.5), 0x3C00);
        assert_eq!(to_half(1.0 + step * 1.5), 0x3C02);
        assert_eq!(to_half(1.0 + step * 0.5001), 0x3C01);
        assert_eq!(to_half(1.0 + step * 0.4999), 0x3C00);
        // Rounding up carries into the exponent.
        assert_eq!(to_half(2.0 - step * 0.25), 0x4000);
        // Below the smallest normal half the same holds in steps of 2⁻²⁴, down to nothing.
        assert_eq!(to_half(TINY * 1.5), 0x0002);
        assert_eq!(to_half(TINY * 2.5), 0x0002);
        assert_eq!(to_half(TINY * 0.5), 0x0000);
        assert_eq!(to_half(TINY * 0.51), 0x0001);
        assert_eq!(to_half(TINY * 1023.75), 0x0400);
        assert_eq!(to_half(1e-30), 0x0000);
        assert_eq!(to_half(-1e-30), 0x8000);
    }

    #[test]
    fn halves_stay_finite() {
        assert_eq!(to_half(65519.0), 0x7BFF);
        assert_eq!(to_half(65520.0), 0x7BFF);
        assert_eq!(to_half(1e9), 0x7BFF);
        assert_eq!(to_half(f32::INFINITY), 0x7BFF);
        assert_eq!(to_half(f32::NEG_INFINITY), 0xFBFF);
        assert_eq!(to_half(f32::NAN), 0x0000);
    }

    #[test]
    fn every_finite_half_comes_back() {
        for h in 0..=u16::MAX {
            if (h >> 10) & 31 != 31 {
                assert_eq!(to_half(half(u32::from(h))), h);
            }
        }
    }
}
