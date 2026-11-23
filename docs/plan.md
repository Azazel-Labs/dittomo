# Plan: from mummer to a learned runtime

mummer is the first piece of a larger design, **Stridefold**. Stridefold does retargeting and motion matching in one
learned pass. An expensive contact-aware retargeter runs at cook time as a teacher. Small learned motion-matching networks
are trained on its output. Each character's body enters the decoder as a constant bias computed once at spawn. At runtime
there is no search and no solver, only a few batched int8 matrix multiplies plus closed-form IK. The estimate is 15 to 25
microseconds per hero character, with no animation clips in memory.

mummer supplies three things that design needs, and that no open-source project offers today:

1. **The import step.** It maps any rig onto the canonical skeleton automatically, with no name table and no humanoid
   template, and says how well it did (`Mapping::quality`).
2. **The cheap base retarget.** The learned decoder only has to predict a residual on top of mummer's output, which is
   easier to learn and to budget than a whole pose ([R2ET][r2et] showed this).
3. **Most of the body descriptor.** The rest-shape features mummer already measures for matching (bone directions,
   heights, lengths over body height, side) are most of the β vector the network is conditioned on.

All numbers below are design targets or back-of-envelope estimates, not measurements, unless they cite a source.

## Goal

**mummer retargets better and faster than any other tool, open or closed, and a public benchmark shows it.** No claim
counts until the benchmark measures it against the alternatives on the same rigs and clips. The targets are proposals
until Phase 1 measures where mummer stands today.

| Measure | How it's measured | Target | Best known today |
|---|---|---|---|
| Mapping accuracy | Share of bones paired as a person would pair them, over a hand-labeled set of real rigs from different tools | Highest of any tool; every bone right on Mixamo, Unreal, Unity, Rigify, Character Creator and Daz rigs | No tool publishes one. Engine tools need name tables or hand fixes |
| Rigs needing no manual fix | Same set, plus unnamed bones, extra roots, twist bones, other spine counts, quadrupeds | Highest of any tool | Engine tools are humanoid-only; topology-free research methods need training or example clips |
| Motion fidelity | Joint position error against Mixamo's own version of the same clip on the target character; limb direction error | Lower than [R2ET][r2et-paper] and [SAN][san-page] on their Mixamo test split | Both publish Mixamo results |
| Contacts | Foot skate (cm per planted frame), self-penetration depth | Lower than every engine tool; close to contact-aware research methods | [ReConForM][reconform] and [MeshRet][meshret-paper] report these, at 30 to 67 FPS on a GPU |
| Per-frame cost | Microseconds per character per frame, 65-bone humanoid, one CPU core, planted feet on | Under 5 µs | [Ultrafast online retargeting][ultrafast]: 22,431 FPS in C++, about 45 µs, though it does more contact work per frame |
| Mapping time | Milliseconds to map two 100-bone rigs | Under 10 ms, so mapping runs live when a rig is dropped in | No one publishes this |

## Where things stand

mummer 0.1 (about 2,700 lines of library, 1,200 of tests, 700 of benchmarks):

- `Mapping::new` matches two skeletons as whole trees on names and rest shape. It is the constrained tree mapping of
  [Zhang (1996)][zhang], with subtrees paired by the Hungarian method. Any bone on either side can be skipped.
- Names from Mixamo, Unreal, Unity, Rigify, Character Creator, Daz, 3ds Max Biped and Blender are normalized.
- `Mapping::apply` carries each bone's world turn across, corrected for both rest poses. Bones between two mapped ones
  share their turns. The topmost bones travel, scaled by height.
- Tests: 300 made-up humanoid pairs (different names, roots, twist bones, spine counts, A-pose and T-pose, axis
  conventions, facing, units, leg lengths) all map with every limb within a degree. A quadruped maps onto an unnamed,
  longer one. A snake mapped from a human scores low.

Phase 1 so far (November 2026):

- `load` reads skeletons and clips from glTF, GLB and FBX through polysmelt's new `rig` view, and builds for wasm.
  The twelve benchmark rigs and their 35 clips load in about 0.14 s, two thirds of it one Mixamo FBX's compressed
  arrays. Loading them found and fixed, in polysmelt: FBX vertices with no skin weights given a joint past the skin's
  end, FBX loading a third faster, a GLB on disk read one system call per accessor (12 times faster for a rig with
  1,407 channels), and baking a clip sorting every key of every channel (55 times faster); `Rig::poser` steps through
  each channel's keys rather than searching them.
- `Player` poses a frame in about 1.6 µs (65 → 76 bones) without allocating, normalizing only what comes in and what
  goes out. Playing a clip, setup included, went from about 30 µs per frame to 2.5 µs.
- `apply_with` adds travel for in-place clips, planted feet with two-bone IK, and sphere push-out, at about 3.3 µs per
  frame with all three on, travel worked out from the feet or not. Planted-foot skate on a
  longer-legged target fell from 0.00048 to 0.00002 of its height per frame. Legs and arms are found without names.
- Mapping takes 1.1 ms for a 65-bone rig onto itself and 2.3 ms onto a 76-bone mannequin.
- **Bone-finding on twelve real rigs** (Mixamo, Ready Player Me, VRM, Khronos samples, three.js and Babylon
  characters; 132 ordered pairs, 2558 hand-labelled joints): mummer finds 92.7% of joints; matching by exact name, as
  three.js and Babylon do, finds 30.3%. Testing on them found and fixed: VRM's `J_Bip_C_` names, a bare `Arm`/`Leg`
  against numbered `arm_joint_2`, rigs that stop at the base of the head (height now also comes from the shoulders),
  a palm bone holding two fingers, mirrored limbs with uneven bone counts, and bones left out though pinned by the pairs
  around them. polysmelt's skin reader had an accessor-offset bug that the VRMs exposed.
- **Limb directions on real clips**: the rigs' own 35 clips played on every other rig (385 pairs). Limbs point 3.4°
  from the source's on average (median 0.2°, 90th percentile 7.7°). Copying world turns from rest onto the right bones
  gives 22°, and copying local rotations, as an animation mixer does with a clip from another rig, 75°. Measuring this
  found the chest tipped by up to 50° on rigs whose collarbones sit beside the chest instead of above it: the rest
  correction lined up only the child with most below it, and now fits all of them at once (Wahba's problem, a few
  Newton steps at setup), which halved the 90th percentile.
- **Foot contact on real clips**: the same pairs over 23,170 frames with a source foot planted. With every fix on, the
  target's feet slide 57 times less (0.00007 of its height per frame against 0.00402) and sit 8.5 times closer to the
  height they should (0.0011 against 0.0094), for 1.1° more limb error as legs bend to keep feet down. Measuring this
  found the arm finder taking a forearm twist bone for the elbow (pushing a seed-san avatar's arms out by up to 60°)
  and a cartoon robot's fingers for feet. Limbs are now picked along each chain by distance from the spine and the
  end, legs must reach the floor, and arms are the chains off the one spine bone whose chains end in the most bones.
- **Gaze**: the head's facing, against the source's in each body's frame, was off by 11.8°, as the neck and head took
  their source's turns as they are and tipped the face of rigs whose head bone isn't built facing forward. They now keep
  the target's rest look and take only the turn away from it: 1.3°, for chest → head going from 4.6° to 6.2°.
- **Hands on the body** (`apply_touching`, with `Surface` skins read from the skinned meshes): on the 63 pairs with
  skins, where the source's hand is within 3% of its height of its body (4,671 frames), the target's gap to its own
  skin is now off by 0.0108 of its height on average against 0.0294 (90th percentile 0.0213 against 0.0654), and its
  hand ends up inside its body while the source's isn't in 1.4% of frames against 8.0%.
- `examples/video.rs` renders a retarget as video (source, dittomo, rotations copied by name), and the brand is now
  **dittomo** (`assets/brand/`); the crate keeps the name `mummer` for now.
- The worst pairs are RobotExpressive (a cartoon with legs a fifth of its height apart and feet hung from the root for
  IK) against unnamed rigs, and BrainStem (all bones `nodeN`, one arm without a hand). Neither was tuned for.
- CI runs fmt, clippy, tests on Linux and Windows, builds and drives the wasm demo, runs the real-rig benchmark (failing
  below 90% of joints found or above 4° of limb error), and benchmarks each pull request against its base.
- A browser demo (`demo/`) loads two files, shows the match and plays the clip with each fix toggled.

Playback against three.js, measured back to back on the same rigs and clip (`bench/three`, run by CI): mummer plays a
frame in 3.6 µs, setup included, against `SkeletonUtils.retargetClip`'s 221 µs, and poses one with a reused player in
2.5 µs against `retarget`'s 186 µs. Babylon's `AnimatorAvatar` (`bench/babylon`, also run by CI) is the closer one:
on a busy machine, best of five runs each, mummer played a frame in 7.0 µs against its 50 µs (7×), or 143 µs with the
hips' path scaled (20×). Babylon corrects for rests that differ but only bakes whole clips.

Missing: engine humanoid profiles (Unity, Godot, Unreal) in the rig benchmark, and the structure-of-arrays path for many
characters.

## Open-source landscape

Checked 2026-11-22. Stars and last push are from the GitHub API on that day.

**No open-source library maps an arbitrary skeleton onto another without a name table or a fixed humanoid template.**
Engine tools are humanoid-only and name-based. Research code needs training, or example clips on the target rig. Rust
has nothing: Bevy's animation crates, `ozz-animation-rs` and `anymotion` don't retarget, and the only retargeting
module found is in the abandoned Ambient engine ([ambient_animation][ambient]).

### Retargeting

| Project | License | Activity | Mapping | Gap mummer fills |
|---|---|---|---|---|
| [Godot 4 retargeting][godot] | MIT | Engine built-in | Bone-name patterns onto a fixed humanoid profile; fix by hand | Humanoid only, names only, overwrites rest poses; Godot only |
| [GodotAnimationRetargeting][godot-mod] | MIT | 136 stars, archived | Presets plus a name dictionary | Archived; Godot only |
| [three.js SkeletonUtils][three] | MIT | Engine built-in | Name map supplied by the user | Frequent reports of broken Mixamo results ([thread][three-bug]) |
| [Babylon.js AnimatorAvatar][babylon] | Apache-2.0 | New in v9 | Name-based, rest-pose and root-scale corrections | Closest web competitor; humanoid-oriented |
| [ReNim][renim] (Blender) | GPL-3.0 | 145 stars, last push 2024-04 | Node graph, presets, JSON maps | Artist tool, Blender only |
| [Expy Kit][expy] / [Retarget][blender-retarget] (Blender) | none stated | 442 stars, last push 2025-12 | Rig presets (Mixamo, Unreal, Daz…) | Artist tool, Blender only |
| [GMR][gmr] | MIT | 2,841 stars, last push 2026-10 | Hand-written config per robot | Human to humanoid robots ([paper][gmr-paper]); not games |
| [Skeleton-Aware Networks][san] | BSD-2-Clause | 1,735 stars, last push 2024-07 | Learned; same basic tree shape only ([project][san-page]) | Needs training; no arbitrary topology |
| [R2ET][r2et] | MIT | 232 stars | Learned residual on copied rotations | Needs training and a mapping |
| [MeshRet][meshret] | MIT | 70 stars | Learned dense mesh-interaction field ([paper][meshret-paper]) | Needs training; GPU, about 30 FPS |
| [Motion2Motion][m2m] | Project page | SIGGRAPH Asia 2025 | Sparse bone pairs plus one or a few example clips on the target | Needs example motion on the target |
| [PALUM][palum] | Paper | 2026 | Learned, part-based, any skeleton | Needs training |

Source-available but not open: Unreal's IK Retargeter with auto-mapping. Closed: Unity's Humanoid Avatar. Both are
humanoid-only and engine-bound.

Prior art closest to mummer's matcher: a USC ICT paper combining name keywords with hierarchy and symmetry
([PDF][usc]). It is a method, not a library.

### Motion matching

| Project | License | Activity | What it is |
|---|---|---|---|
| [orangeduck/Motion-Matching][oduck] | MIT | 945 stars, last push 2025-02 | Holden's reference code, including learned motion matching |
| [JLPM22/MotionMatching][jlpm] | MIT | 612 stars, last push 2026-04 | Classic motion matching for Unity |
| [AI4Animation][ai4a] | none stated | 8,933 stars | Starke's neural character animation research code |
| [godot_motion_matching][vsekai] | MIT | 4 stars, last push 2024-02 | Small Godot experiment |
| Unreal Pose Search | Source-available | Engine built-in | Classic motion matching with PCA and a kd-tree |

Nobody open-source combines retargeting with learned motion matching. That combination is Stridefold's core.

### What this means for mummer

- **To be better:** the automatic, any-topology matcher is something nobody else has. But the engines already ship
  planted feet and root handling, so mummer loses on contacts until it has both.
- **To be faster:** mummer is closed form per bone, with no solver and no network. The learned research methods run at
  30 to 268 FPS on a GPU, and the fastest published online method takes about 45 µs per frame.
- **To prove both:** a public benchmark on real rigs, since no competitor publishes one.
- A Rust crate is uncontested; a wasm demo reaches the three.js and Babylon audiences directly.

## Design goals and budgets

The runtime does no optimization and no retargeting solve. It reads precomputed data, runs fixed-size small networks,
and applies closed-form IK. Everything expensive moves to editor or cook time.

| Budget | Target | Why |
|---|---|---|
| Hero character, full pipeline | under 0.05 ms per character per frame, one core | 100+ characters inside a 5 ms animation budget |
| Crowd character (LOD 2+) | under 0.005 ms | Thousands of agents |
| Runtime memory per motion set | 2 to 20 MB, independent of body count | Data must not multiply with characters |
| Hot-path allocations | zero | Fixed buffers, pooled per worker |
| Determinism | bit-identical on one platform | Replays, networking, QA repro |
| Editor bake for a new character | under 10 minutes on a workstation GPU | Try a new body in the same session |
| Full retrain | overnight | Only when the clip library changes |

Runtime work, in order of preference: a table lookup; a tiny network as batched SIMD matrix multiplies; a closed-form
fixup (two-bone IK, foot lock, inertialization). Anything iterative moves to cook time.

## State of the art that informs the design

The fastest proven runtime pieces are learned motion matching (constant cost, megabytes of weights) and solver-free
online retargeting. Mesh-aware and neural retargeters give better contacts but run at tens of frames per second, so they
belong at cook time.

| Method | Reported cost | Role here |
|---|---|---|
| [Learned Motion Matching][lmm] (Holden et al., SIGGRAPH 2020; [paper][lmm-pdf]) | 590 MB controller to about 17 MB of weights | Runtime core: Projector, Stepper, Decompressor |
| [Lipschitz + sparse mixture-of-experts LMM][lipschitz] (Kleanthous and Martini, MIG 2023) | 8.5x lower CPU cost than LMM at 80% of its memory | Architecture for Stepper and Decompressor |
| [Ultrafast and Controllable Online Retargeting][ultrafast] (ACM TOG, Dec 2025) | 22,431 FPS in C++; 0.13 ms per frame in Unity vs 3.92 ms for R2ET | Model for the runtime contact fixup (sphere push-out) |
| [ReConForM][reconform] (Kinetix, Eurographics 2025) | 67 FPS on an RTX 3060 | Cook-time teacher with contacts |
| [R2ET][r2et-paper] (CVPR 2023) | 268 FPS (PyTorch) | Template for the shape-conditioned residual |
| [MeshRet][meshret-paper] (NeurIPS 2024) | 30 FPS (GPU) | Cook-time teacher for hard contact cases |
| [PALUM][palum], [Motion2Motion][m2m-paper] | Offline | Cross-family transfer at cook time |
| mummer | 2.4 µs per frame posing, 4.4 µs with travel, planted feet and push-out, one CPU core (65 → 76 bones); 1 to 2.3 ms to map; 92.7% of joints found on 12 real rigs (exact names: 30.3%); limbs within 3.3° of the source's on their 35 clips | Import mapping, base retarget, β features |

Two lessons carry into the architecture. Residual corrections on a cheap base retarget are easier to learn and budget
than full pose prediction. And the expensive contact-aware solvers are good enough to be offline teachers, because their
output is computed once per clip and body.

## Architecture

Two pipelines share one artifact. The cook pipeline turns any source clips into one packed motion-set file. The runtime
pipeline only reads that file.

```
cook:    source clips ─► mummer map + base retarget ─► teacher (contacts, mesh) ─► train networks ─► motion set
                         onto canonical skeleton        per body, cached by hash

runtime: query ─► Projector (every 6-10 frames) ─► Stepper ─► Decompressor + body bias ─► foot lock, 2-bone IK ─► pose
```

### 1. Canonical skeleton and body descriptor

- **Canonical skeleton.** One project-wide skeleton per morphology family (30 to 60 joints for humanoids). Source clips
  are brought onto it once by **mummer**, replacing manual cleanup in an engine's retargeter. Clips whose
  `Mapping::quality` falls below a threshold are flagged for a person to correct with `Mapping::set`.
- **Body descriptor β.** 32 to 64 floats per character, computed at import in seconds: bone lengths over leg length,
  rest-pose offsets from the canonical skeleton, radii of 12 to 20 collision spheres fitted to the skinned mesh, and
  style flags (such as squash and stretch). mummer's shape features supply the first two.

### 2. Cook-time teacher

- Sample bodies: every shipped character plus a few hundred variations across the character-creator range.
- For each body, retarget the canonical library with mummer as the starting pose, then refine with a contact-aware
  optimizer ([ReConForM][reconform]-style first, [MeshRet][meshret]-style for hard cases): ground contacts,
  self-contacts, penetration.
- Output per frame: target-space local pose, root motion, foot and hand contact labels, prop attach transforms.
- Runs on a build farm, cached by a hash of clip, body and solver version. Never ships.

### 3. Body-normalized matching features

Trajectory, foot positions and velocities, and hip velocity are measured in body-relative units (positions over leg
length, speeds over leg length per second). One Projector and one Stepper then serve every body. Stride differences are
handled by the decoder and a playback-rate scale from β.

### 4. Learned motion matching core

| Network | Input | Output | Runs | Starting size |
|---|---|---|---|---|
| Projector | Normalized query features | Matched feature + latent | Every 6 to 10 frames, or on a big query change | 4 × 256, sparse experts |
| Stepper | Feature + latent | Next feature + latent | Every frame | 3 × 256 |
| Decompressor | Feature + latent, plus β | Residual over mummer's base pose, root delta, contact probabilities, props | Every frame | 3 × 512 |

### 5. Body conditioning for free (bias fold)

The Decompressor's first layer computes W_x·x + W_β·β + b. β is constant per character, so b_char = W_β·β + b is
computed once at spawn. Every frame the layer is just W_x·x + b_char, the same cost as an unconditioned network.
Characters with different bodies still batch together: one shared matrix multiply, then a per-row bias. FiLM-style scale
and shift on hidden layers fold the same way.

### 6. Runtime contact and IK fixup (closed form)

- **Foot lock:** pin the foot when contact probability crosses a threshold (with hysteresis); blend out on lift-off.
- **Terrain and props:** analytic two-bone IK, pole from the decoded pose.
- **Penetration (LOD 0):** a few fixed sphere pairs, one push-out step, after the [ultrafast online method][ultrafast].
- **Transitions:** inertialization when the Projector jumps; no blend buffers.

### 7. Editor workflow for a new character

1. Import the mesh. mummer maps it and β is computed in seconds; it plays immediately on the shared networks.
2. A background job runs the teacher on about 50 reference clips and compares contact error, foot skate and penetration
   against the student (minutes).
3. Above threshold, fine-tune a small low-rank adapter on the Decompressor for that body, merged at cook time.
4. Full retraining only when the clip library changes.

### 8. Cooked data format

One motion-set file per morphology family: header (canonical skeleton, β schema, normalization stats, version hash);
int8 per-channel weights laid out for the target SIMD width; per-character β plus merged adapter deltas (biases folded
at load); contact channels and IK chains. No clips at runtime.

## Handling variation

| Case | Strategy | Runtime cost |
|---|---|---|
| Fixed cast (under 20) | Shared networks; adapters for characters that fail validation | Same as base |
| Character creator | Train across the slider range; recompute β and refold the bias on change | One small matrix-vector multiply per change |
| Hero character, exact quality | Cook a classic motion-matching database from teacher output for that body | Search cost, memory linear in data |
| Body outside the trained range | Editor warns by distance to nearest training bodies; add it or fine-tune an adapter | None |
| Non-humanoid families | Own canonical skeleton per family. mummer already maps across topologies; [PALUM][palum] or [Motion2Motion][m2m-paper] at cook time where shapes differ too much | Same as humanoid |
| Extra limbs, props | Extra decoder outputs and IK chains | Linear in outputs |
| Squash and stretch | Translation channels in the output, flagged in β | Slightly larger output layer |

## Performance estimate

At the starting sizes a hero character is about 640,000 multiply-adds per frame: Decompressor about 500,000, Stepper
about 115,000, Projector about 25,000 averaged, closed-form fixups under 5,000. That is roughly 15 to 25 microseconds on
one core with batched int8, inside the 0.05 ms target. Weights are about 1 MB int8 per motion set before experts. To be
verified with a prototype.

How to hit it: batch characters on the same motion set into structure-of-arrays blocks; int8 with per-channel scales
(AVX-512 VNNI or AVX2 on PC, NEON dot-product on ARM), last layer 16-bit if precision suffers; integer kernels for
determinism; threads per chunk of 16 to 64 characters; all buffers sized at load.

| LOD | What runs | Rough cost |
|---|---|---|
| 0 (hero) | Everything, including sphere push-out and hand IK | 100% |
| 1 (mid) | No push-out or hand IK; Projector every 12 frames | about 90% |
| 2 (far) | Decompressor every other frame, interpolated; foot lock only | about 50% |
| 3 (crowd) | Shared feature stream per group, decoded once and instanced, or baked vertex animation | under 5% |

## Roadmap

### Phase 1: the best and fastest retargeter, proven

- [x] **Load rigs and clips through [polysmelt][polysmelt]**, behind a `load` feature so the core crate keeps glam as
      its only dependency. polysmelt reads glTF, FBX and USD into one glTF document, with FBX skin clusters and
      animation stacks already converted into glTF skins and sampled channels. Mixamo's FBX downloads load directly,
      with no Blender round trip. `Document::decode` turns quantized and compressed data into plain floats first. The
      benchmark needs this.
- [x] **Typed skin and animation views in polysmelt**, rather than a private reader in mummer. These are already on
      polysmelt's planned list ("typed views over the document's meshes, accessors, nodes, skins and animations"), and
      mummer is their first user. Build with `default-features = false, features = ["gltf", "fbx"]`, plus `usd` when
      needed (it raises the minimum Rust to 1.96).
- [ ] **Benchmark harness.** A hand-labeled set of real rigs, plus Mixamo clips on several characters. It runs mummer
      and, headless, three.js SkeletonUtils, Babylon AnimatorAvatar, Godot's importer, R2ET and SAN. Unreal's IK
      Retargeter joins through editor Python where its license allows. Mixamo's terms likely forbid redistributing its
      files, so publish the scripts and have users download the data. Done for bone-finding: `bench/rigs/fetch.sh`
      downloads twelve rigs at pinned commits with checksums, `bench/rigs/labels.txt` labels them, and
      `cargo bench --features fs --bench rigs` scores mummer against exact-name matching, then plays every rig's clips
      on every other and scores limb directions against copying rotations and foot contact with each fix. The
      other tools are still to come.
- [ ] **Measure where mummer stands** on every row of the Goal table. Mapping accuracy is measured on real rigs;
      per-frame cost and mapping time on made-up rigs (see Where things stand); limb directions and foot contact on real
      clips; three.js's and Babylon's playback on the same rigs.
- [x] **Root motion** for in-place clips, from planted feet.
- [x] **Planted feet** with two-bone leg IK, so feet don't slide when leg lengths differ.
- [x] **Self-penetration cleanup:** one closed-form sphere push-out pass, after the [ultrafast online method][ultrafast].
- [ ] **Speed:** no allocation in `apply` (done: `Player`), a structure-of-arrays path that retargets many characters
      per call (not started), and a per-frame cost benchmark in CI that fails on regressions (done).
- [ ] **Publish the results table** in the README, regenerated by CI. The numbers are in the README; CI writes the
      real-rig table and the speed table to each run's summary and fails on a drop, but doesn't commit them back.
- [x] **Browser demo** (wasm): drop in two glTF or FBX files, see the mapping and its quality, play the clip.
      polysmelt builds for wasm without its `fs` feature and loads from memory (`formats::load_bytes`).

### Phase 2: teacher

- [ ] Choose the canonical humanoid skeleton (see open questions).
- [ ] β descriptor from mummer's shape features plus fitted collision spheres.
- [ ] First teacher: ReConForM-style contact optimizer starting from mummer's pose; contact, skate and penetration
      metrics on its output before anything trains on it.

### Phase 3: learned core

- [ ] Train Projector, Stepper and a residual Decompressor on teacher output, with the bias fold.
- [ ] Validation gate and low-rank adapters for bodies that fail it.

### Phase 4: runtime kernels

- [ ] int8 batched kernels per platform, closed-form fixups, LOD tiers, cooked file format.
- [ ] Measure against the budgets above.

## Risks

The biggest risk is quality, not speed: learned decoders soften poses and slide feet.

| Risk | Mitigation |
|---|---|
| Decoder smooths away detail (fingers, sharp hits) | Per-joint loss weights, 16-bit last layer, contact and velocity loss; hero fallback to a cooked database |
| Foot sliding from flickering contacts | Hysteresis plus runtime foot lock |
| Bodies outside the trained range | Distance warning; adapter or add to training set |
| Teacher artifacts learned as truth | Version the teacher; metrics on its output before training |
| A wrong mummer mapping poisons a whole clip set | Quality threshold at import; `Mapping::set` corrections stored with the asset |
| Animators lose control | Debug view of the nearest real clip per Projector output; style tags as query features |
| Determinism across platforms | Integer kernels; one reference implementation tested everywhere |
| Mocap and generative-model licensing | Track every clip's license; weights inherit the most restrictive |

## Open questions

- Canonical skeleton: in-house, Unreal's Manny, or NVIDIA SOMA?
- First platform: PC only, or consoles too? This decides the int8 kernel set.
- Does the target game need a character creator? That decides how much slider space training must cover.
- First teacher: ReConForM-style optimizer or MeshRet-style dense mesh method?

## Name

**Stridefold**: "stride" for locomotion and motion matching, "fold" for bodies folded into the network as a constant
bias and the expensive retarget folded into cook time. A web search on 2026-11-22 found no product, company or
repository by that name. A web search is not trademark clearance: check USPTO and EUIPO (classes 9 and 42), the .com
and .dev domains, and GitHub, Fab and the Unity Asset Store first. Note that Stride is also an open-source game engine.

## References

- Zhang, [A constrained edit distance between unordered labeled trees][zhang], Algorithmica 15(3), 1996.
- Holden, Kanoun, Perepichka, Popa, [Learned Motion Matching][lmm-pdf], SIGGRAPH 2020 ([Ubisoft La Forge][lmm]).
- Kleanthous, Martini, [Learning Robust and Scalable Motion Matching with Lipschitz Continuity and Sparse Mixture of
  Experts][lipschitz], MIG 2023.
- Guo, Chen et al., [Ultrafast and Controllable Online Motion Retargeting for Game Scenarios][ultrafast], ACM TOG 44(6),
  2025.
- Cheynel et al., [ReConForM: Real-time Contact-aware Motion Retargeting][reconform], Eurographics 2025.
- Zhang et al., [Skinned Motion Retargeting with Residual Perception of Motion Semantics & Geometry (R2ET)][r2et-paper],
  CVPR 2023 ([code][r2et]).
- Ye et al., [Skinned Motion Retargeting with Dense Geometric Interaction Perception (MeshRet)][meshret-paper],
  NeurIPS 2024 ([code][meshret]).
- Aberman et al., [Skeleton-Aware Networks for Deep Motion Retargeting][san-page], SIGGRAPH 2020 ([code][san]).
- Chen et al., [Motion2Motion: Cross-topology Motion Transfer with Sparse Correspondence][m2m-paper], SIGGRAPH Asia 2025
  ([project][m2m]).
- [PALUM: Part-based Attention Learning for Unified Motion Retargeting][palum], 2026.
- Ze et al., [Retargeting Matters: General Motion Retargeting for Humanoid Motion Tracking][gmr-paper], 2025
  ([code][gmr]).
- Feng et al., [Automating the Transfer of a Generic Set of Behaviors onto a Virtual Character][usc], USC ICT.

[polysmelt]: https://github.com/Azazel-Labs/polysmelt
[zhang]: https://unpaywall.org/10.1007%2FBF01975866
[lmm]: https://www.ubisoft.com/en-us/studio/laforge/news/3NWqwcMU9lMfumgfPAEfka/learned-motion-matching
[lmm-pdf]: https://www.theorangeduck.com/media/uploads/other_stuff/Learned_Motion_Matching.pdf
[lipschitz]: https://dblp.org/rec/conf/mig/KleanthousM23.html
[ultrafast]: https://discovery.ucl.ac.uk/id/eprint/10219287
[reconform]: https://arxiv.org/abs/2502.21207
[r2et]: https://github.com/Kebii/R2ET
[r2et-paper]: https://arxiv.org/abs/2303.08658
[meshret]: https://github.com/abcyzj/MeshRet
[meshret-paper]: https://arxiv.org/abs/2410.20986
[san]: https://github.com/DeepMotionEditing/deep-motion-editing
[san-page]: https://deepmotionediting.github.io/retargeting
[m2m]: https://lhchen.top/Motion2Motion
[m2m-paper]: https://arxiv.org/abs/2508.13139
[palum]: https://arxiv.org/html/2601.07272
[gmr]: https://github.com/YanjieZe/GMR
[gmr-paper]: https://arxiv.org/abs/2510.02252
[usc]: https://ict.usc.edu/pubs/Automating%20the%20Transfer%20of%20a%20Generic%20Set%20of%20Behaviors%20onto%20a%20Virtual%20Character.pdf
[godot]: https://docs.godotengine.org/en/4.7/tutorials/assets_pipeline/retargeting_3d_skeletons.html
[godot-mod]: https://github.com/smix8/GodotAnimationRetargeting
[three]: https://threejs.org/docs/pages/module-SkeletonUtils
[three-bug]: https://discourse.threejs.org/t/skeletonutils-retarget-doesnt-work-with-mixamorig-skeleton-inverted-feet-backward-hands/54892
[babylon]: https://forum.babylonjs.com/t/introducing-animation-retargeting/62547
[renim]: https://github.com/anasrar/ReNim
[expy]: https://github.com/pKrime/Expy-Kit
[blender-retarget]: https://extensions.blender.org/add-ons/retarget/
[ambient]: https://docs.rs/crate/ambient_animation/0.2.0/source/src/retargeting.rs
[oduck]: https://github.com/orangeduck/Motion-Matching
[jlpm]: https://github.com/JLPM22/MotionMatching
[ai4a]: https://github.com/sebastianstarke/AI4Animation
[vsekai]: https://github.com/V-Sekai/godot_motion_matching
