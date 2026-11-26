# Beyond retargeting: secondary motion, a runtime, the GPU (research digest, 2026-11-26)

Two surveys were run on whether retargeting alone is enough, what Unreal's AnimDynamics and its kind do, how much of
a runtime animation system dittomo needs, and what can move to the GPU. Nothing here is built; costs are estimates.

## Scope: what to build, what to leave

**Build a layer that works on a pose after retargeting, not an animation system.**

- **Secondary motion for the bones a clip doesn't drive** (hair, tails, capes, ears, skirts). Today they keep their
  rest pose: on the README's VRoid rig, 155 bones with 54 paired, all hair and clothes are rigid. The retargeter is
  the only thing that knows which bones got no motion, so it can find the chains and set them up with no hand work,
  as it maps bones. No tool found does this; VRM is the nearest (its animations carry the body only, spring bones do
  the rest).
- **Look-at** (neck and head are already found on any rig), **reach goals** on the limbs it already knows, **feet on
  uneven ground** (inside the feet fix, or two IK passes fight), **an inertializer** for cuts (it shares the spring's
  math), and exposing what it already derives: root travel per frame, foot contacts for footsteps.
- **Three headless helpers** for hosts with no animation system (custom engines, Bevy, the web, servers): sample a
  clip at a time, blend poses, write skinning matrices.

**Leave to engines:** animation graphs, state machines, blend spaces as assets, sync groups, compression (ACL),
ragdolls, full-body iterative IK, mesh cloth and strand hair. ozz-animation is the model: jobs, no graph.

**Not AnimDynamics itself.** It is a rigid-box solver with limits and no collision, called a legacy tool by Epic
staff next to their Rigid Body node. Everything widely used (KawaiiPhysics, VRM spring bones, DynamicBone, Godot's
spring bones) is a chain of particles: one per bone, pulled toward the animated pose, its length restored in one
pass from root to tip.

## Secondary motion on the CPU ("sway")

- One particle at each chain bone's tail, solved root to tip, one way: the body never feels the chain, and only
  undriven bones are written.
- An exact damped spring toward a goal that moves through the step (a rational approximation of the exponential:
  stable at any step, no libm, the same bits on every machine), so frame rate doesn't change the motion; VRM's and
  KawaiiPhysics' updates depend on it.
- Then: length restored, a cone limit, pushed out of the body's spheres (limb capsules are new work) and the floor.
- Gravity less what the rest pose already carries, so the rest pose is a fixed point: no sag, no warm-up.
- Reset and teleport carry the particles; substeps of at most 1/60 s, four at most.
- Set up by itself: stiffness from each bone's own length (three times as fast as a pendulum that long), damping
  0.6 and most of the slowing left to the air, radius from bone length or the skin. Fingers, faces, IK controls and
  props left out.
  VRoid's `J_Sec_` bones and VRM spring lists are ground truth for finding chains.
- Last, after every contact fix. Over a whole clip: stepped through its times, twice round a loop.
- About 20 to 30 ns a bone a substep by estimate: 1.5 µs for a VRoid avatar's 60 spring bones.
- VRM and KawaiiPhysics settings map onto it by matching poles at 60 poses a second.

**Built so far** (`src/sway.rs`): finding chains, one-way chains, the body and the floor kept out of, and playing
(`Fixes::sway`, off by default, in `Live` and whole clips; `Sway::clip` by itself). On the VRoid bench rig all 83
`J_Sec_` bones are found and nothing else; on the other VRM all 19 of its own spring joints. What differed from the
plan above:

- Speed isn't taken from how far the tail moved in the step (that damps a swing by how far it turned, so by frame
  rate): what is across the bone is kept, carried round with the bone. Gravity's rest share is taken half a step
  back. With both, 144 against 60 poses a second differ by 0.5 to 0.7% of a chain's length on a moderate dance,
  from 4 to 7%. Whipped twice as hard they differ by 7 to 10%: a chain of several bones is touchy itself.
- A lone bone with no children is never a chain (eyes, jaws, markers), so single-bone ears stay stiff.
- The defaults are set against springs set up by hand. Both VRM bench rigs carry `VRMC_springBone` settings; a
  test steps the format's own update at 60 poses a second beside `Sway` under the same body (an idle, a walk and a
  run from another rig) on the bones both swing, 17 and 57 of them. Bent from rest, in radians, on average and for
  95 in a hundred, on the two rigs:

  | | by hand | sway before | sway now |
  |---|---|---|---|
  | idle | 0.003 0.009, 0.041 0.152 | 0.356 1.254, 0.587 1.309 | 0.005 0.009, 0.011 0.061 |
  | walk | 0.135 0.274, 0.360 0.835 | 0.541 1.253, 0.728 1.309 | 0.203 0.367, 0.299 0.485 |
  | run | 0.344 0.791, 0.616 1.422 | 0.849 1.309, 0.946 1.309 | 0.349 0.773, 0.482 0.967 |

  The two ways' bones are 0.02 to 0.76 radians apart on average now, from 0.24 to 1.66. What it took: stiffness by
  a bone's own length, not the length left to the tip (9 g a bone's length for each radian off, from 1 to 9 g a
  chain's length: a tilted head bent a long chain's first bones several times as far as it tilted, which was most
  of the idle's bend); and air drag of 25 a second, from 0.5, as nearly all of a hand-set spring's slowing is drag
  in the world, so its chains trail a body that travels. The cone stays at 75 degrees: narrower fits worse, as
  hand-set springs have none. Stiffer near the root, and a length between the bone's and the chain's, fit no
  better. Through `apply_with`, 0 to 1.2% of chain bones' poses are at the cone in a run, from 23 to 31%.
- A pose with no time passed holds the chains (each bone points at where its tail was); only the first puts them
  at rest. They start moving as the body does by the second pose, so a clip that starts mid-stride doesn't whip
  them back.
- The body is the one arms keep out of, as rods with round ends: spine, neck, arms and legs, and the head's ball
  (fitted to the skin where `Live` and `apply_touching` know it). No rig-made colliders, no skin-fitted limbs, and
  nothing for a body with no arms found (animals): only the floor. A particle slides round its bone's head to get
  out, so lengths hold; no pair comes nearer than at rest, so rest stays a fixed point. Getting out wins over the
  cone: a rope let down lies flat.
- A tail is asked no further out from a rod than its bone's head is then, but for how much further out it is at
  rest. A two-bone skirt starting on a thigh, the thigh swung 1.2 and 1.9 radians through where it starts, is bent
  1.20 and 1.33 radians, from 1.74 and 2.15: laid along the thigh, not turned out of it. On the bench rigs that
  was the lesser part: of the 5 to 8% of poses past the cone in a run (3.09 radians at worst), nearly all were
  long hair against the back and arms, swung there by the floppy springs. With the old springs this rule takes
  7.61% to 7.34%; the new springs alone take it to 0.27% at most (1.34 at worst in a run, 1.75 in a samba, hair
  an arm lifts), and the rule changes nothing more there.
- A knock comes when a step ends inside, so frame rates tell more where chains touch the body: 3.8% of a chain's
  length between 30, 60 and 144 poses a second where a ponytail knocks on the back, against 0.7% swinging free.
- Twice round a loop isn't enough: what is left of the start takes nearly 2 s to halve. A loop is gone round till a round
  starts as the one before did (eight times at most), and what is left is eased away over its last 0.3 s, so the
  ends meet whatever the chains do. A loop is told from its ends alone: a clip that only starts and ends in the
  same stance is taken for one.
- `Sway::jump` carries the chains with a body set down elsewhere; `Live` has no such call yet, only `reset`.

Stages: find chains; one chain's integrator against the analytic spring and across frame rates; colliders and
limits; `Live`, whole clips, baking; defaults; VRM and KawaiiPhysics import; `Crowd` lanes; links between sibling
chains (skirts), telling baked jiggle from intent, resampling a 3-bone chain onto 8.

## The GPU

- **The transfer needs no hierarchy on the target's side** already: with the source's turns kept in the world, a
  target bone's skinning turn is one or two quaternion products. Positions need a sum up the ancestors only.
- **The win is moving data, not arithmetic**: `Crowd` is 6.2 ms for 10,000 characters on one core, but their poses
  are 23 MB uploaded a frame, against 32 to 48 bytes an instance if the GPU does the work.

In order, after the CPU versions:

1. **Bake once, play on the GPU.** A retargeted clip library per target rig (a quaternion and a translation per
   bone per frame, half floats: 5.8 MB for 50 clips of 3 s at 30 Hz and 80 bones), sampled in the vertex shader,
   two clips cross-faded. CPU code only, every fix kept. **WebGPU is the target** (decided 2026-11-26), not
   WebGL2: the library is a storage buffer read by index, not a texture, so the same buffer feeds the compute
   passes of steps 3 to 5 and nothing is laid out twice. A texture for WebGL2 can still be written from the same
   records by a host that wants one.
2. **World-space source clips and flat mapping tables**, used by `Crowd` itself: the groundwork for the rest.
3. **Retarget on the GPU per instance**: one source library, about 8 KB per target rig, one compute pass with a
   thread per character. For many body types, big libraries, and live or generated sources.
4. **Spring chains in compute**: a thread per chain, no coloring needed, reading and writing the pose buffer.
5. **Two-bone IK, floor and push-out on the GPU**, from contacts baked into the clip. Hands together, hands on the
   head and touch stay on the CPU, for heroes.
6. Batched inference for the learned runtime in the plan, last.

**Two tiers, one set of records.** WebGPU (WGSL, storage buffers, compute) is the floor: it runs in browsers and,
through wgpu, natively. Later, a tier for what current Vulkan and D3D12 allow and WebGPU doesn't: buffer device
addresses (pointers in shaders, so one pool holds every rig's library and an instance carries a pointer to its
clip, its mapping table and its chain state, with no bind groups or per-rig batching), mesh shaders, and
multi-draw indirect with culling in compute. The flat records are written so they hold offsets in the first tier
and addresses in the second. What dittomo hands a host is then the data and the generated shader source for its
tier, not a renderer.

Left to hosts: skinning, instancing, culling, LOD, per-vertex animation textures, terrain queries.

**To shape now on the CPU** so the port is mechanical: the per-bone plan as one flat record (no enum, no `Option`,
flags not `Vec<bool>`), a world-space source input and a world-space output for `Crowd`, lean and twist in lanes,
travel without matrices, contacts as channels of a clip, and one versioned export of the tables.

Numbers others publish: 9,547 instanced characters at 34 fps in 2007 (GPU Gems 3); 10 to 15 thousand skinned meshes
over 150 fps on a GTX 1060 (gpuinstance); Alan Wake 2's GPU skinning aimed at a million bones.

### Built: step 1, bake once and play (`dittomo::bake`)

CPU code only, and no GPU has run it yet. `Baker` takes clips already on one rig and writes one library; `Library`
reads it back and samples it on the CPU; `bake::wgsl` is the same sampler as WGSL, written beside it function for
function. naga parses and validates the WGSL in the tests, with a vertex shader that calls all of it. That the two
give the same numbers rests on reading them side by side until a host runs both.

Little-endian 32-bit words, every offset a count of words from the library's start (`C` clips, `B` bones, `F`
frames in all, `R` words a record: 7 as floats, 4 as halves):

| Part | Words | Holds |
| --- | ---: | --- |
| Header | 8 | magic `DTMO`, version 1, encoding, `B`, `C`, the library's words, the table's and the pivots' offsets |
| Clip table | 4 × `C` | per clip: first record's offset, frames, frames a second, flags (bit 0: loops) |
| Pivots | 3 × `B` | per bone: where its joint stands at bind, as floats |
| Records | `R` × `B` × `F` | per clip, frame and bone: rotation from bind (x, y, z, w), the joint's place (x, y, z) |

`4 × (8 + 4C + 3B + R·B·F)` bytes. Fifty clips of 3 s at 30 Hz on 80 bones: 5,761,792 bytes as halves and
10,081,792 as floats looping (90 frames each), 5,825,792 and 10,193,792 played once through (91 each).

What changed from the plan above:

- **A record holds where the joint stands, not the skinning translation.** The translation (the world transform
  times the inverse bind) is the small difference of two long arms, and blending it between frames or clips pulls
  joints toward the rig's origin: two to five times further off half way between frames on the test clips. The
  sampler blends the joint's place, then takes the pivot off (`place − rotation × pivot`), which costs three words a
  bone once and one more read a sample. What `bone` hands back is still the skinning rotation and translation.
- **No scale.** Scales at bind cancel. Scale a clip animates is dropped: the bone keeps its bind size about its
  joint, which still stands where the clip puts it. Retargeted clips keep the target's scales.
- **Halves are weak for places, fine for rotations.** A half places a joint within a 2048th of how far out it
  stands: 0.0006 of the rig's height at the fingertips in place, 0.0009 three heights from the origin, against a
  millionth as floats. Sixteen bits spread evenly over each clip's own bounds would be tens of times closer for
  the same bytes; not done, as halves were the plan. Until then, bake clips that travel far in place or as floats.
- **Between frames** the sampler turns each bone in the world, the clip under its parent: under a thousandth of
  the height on a walk at 30 Hz, up to four on a clip with every bone swinging hard. A higher rate buys that back.
- **A looping clip's last pose is taken to be its first** and isn't stored, and each clip's rate is nudged so its
  frames fit its length exactly.

What a host does: upload the bytes as they are to a storage buffer and bind it read-only where it told `wgsl` to
declare `library: array<u32>` (one binding for either encoding; halves are unpacked in WGSL, as `f16` is an
extension). Give each instance a clip and a time, and for a cross-fade a second clip and time and a weight. Call
`bone(clip, time, index)` or `blend(clip_a, time_a, clip_b, time_b, weight, index)` for each of a vertex's four
bones and `skin(position, a, b, c, d, weights)`, or call them once a bone in a compute pass that writes a pose
buffer, which is the only way in WebGPU's compatibility mode (no storage buffers in the vertex stage) and the
cheaper one for dense meshes. The WGSL's names are plain (`bone`, `blend`, `skin`, `Bone`, `Header`, `Entry`,
`library`): a host whose own shader uses one must rename.
