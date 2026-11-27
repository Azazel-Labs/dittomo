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
- A lone bone with no children is never a chain (eyes, jaws, markers), so single-bone ears stay stiff. (Since:
  `sway::danglers` lists them for a caller to pick from; see 2026-11-27 below.)
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
  (Since: `Live::jump`.)

**Cost, from CI (d9da460, release, a 136-bone target with 60 bones of hair, 30 poses a second so two steps a
pose):** sway alone 17.1 µs a pose, 11.0 with nothing to keep out of: about 90 ns a bone a step for the spring and
turning the bones, and half as much again for the body and floor. Three to four times the estimate above. Not
profiled; the spring's own cost is the larger part, not the colliders.

Stages: find chains; one chain's integrator against the analytic spring and across frame rates; colliders and
limits; `Live`, whole clips, baking; defaults; VRM and KawaiiPhysics import; `Crowd` lanes; links between sibling
chains (skirts), telling baked jiggle from intent, resampling a 3-bone chain onto 8.

### Sway, 2026-11-27: chains tied side by side, bones alone, a jump in `Live`, the demo

**Chains side by side are tied** (`Sway::tie`, on by default; `Sway::beside` says which). Two chains are side by
side if they hang from one bone of the body, the longer is at most 1.5 times the shorter, and their middles are no
further apart than 0.6 of the shorter's length. A chain is tied to the nearest such chain and to the nearest that
isn't the same way off, where each picks it back: a ring closes, a row of strips has two ends, and nothing is
looked up by name. Each tail is tied to the nearest tail of the chain beside it and keeps between 0.5 and 1.25 of
how far from it it is at rest, each of the two going half the way. The point half way between two tied tails is
kept out of the legs, where it is clear of them at rest. Chains with none beside them are stepped as before, to
the bit.

On a made-up skirt (eight folds of three bones round the hips of a 1.8 m person; a walk with thighs swung 0.7
radians and a run with 1.3, at 30 and 60 poses a second), untied then tied:

| | walk | run |
|---|---|---|
| a tail in a leg | 0% → 0% | 0% → 0% |
| the point half way between two folds in a leg | 5.2 to 5.5% → 0% | 6.6% → 0.00 to 0.07% |
| a tied pair nearer than a quarter of its rest distance | 0% → 0% | 0% → 0% |
| a tied pair over half as far again | 1.4% → 0% | 1.5 to 1.6% → 4.2 to 4.3% |
| most a bone is bent, radians | 0.38 → 0.38 | 0.37 → 1.5 to 1.6 |

Untied, no tail ever touches a leg: the thigh comes out between two folds. Tied, it lifts both (the 1.5 radians),
and the folds next to those are left behind for a pose or two (the 4%).

On the VRM bench rigs there is no skirt to measure. One has no chains side by side (two locks pick each other,
but the head lies between them). On the other, ten of 24 chains are tied, all long locks of hair; its coat's three
flaps a thigh are 0.85 to 0.88 of their length apart, so not side by side. No chain's tail is in a leg in any clip,
tied or not (the rods already did that). With 0.9 for 0.6 the flaps tie, and the point half way between two of
them is in the other leg in 0.52% of a samba's poses untied, none tied; but pigtails as far apart as they are long
would tie too. Of the tied locks' pairs, those over half as far again apart, untied then tied: 10.9% → 2.6% in a
run, 7.3% → 0% in a walk, 9.0% → 1.2% in a samba.

The yardsticks on that rig (the other's are the same to the last digit): against springs set by hand, the two
ways' bones are 0.381 radians apart in the walk, from 0.402, and 0.623 in the run, from 0.640; the idle is as it
was. Through `apply_with`, more poses are past the cone, pulled into the body by a tie: 0.20% in the walk from
none, 0.38% in the run from 0.27%, 0.07% in the samba from 0.04%; the most a bone bends in the walk is 1.36
radians, from 0.95.

What was tried and dropped:

- *Neighbors read where they were as the pose began, gone on as they were going*, so chains could be stepped one
  after another as before. A 9 cm bone swings at 31 radians a second, so a thirtieth of a second of going straight
  on is off by half its swing: in a run 86 to 90% of tied pairs were over half as far again apart, from 15%. Tied
  chains are stepped together now, a step at a time: a neighbor already stepped is where it is, one yet to be is
  where it was a step ago, gone on as it was going.
- *The point half way kept out of every part, as near as at rest.* Half way between two tails lying on the head is
  deeper in it the further apart they swing, so it lifted hair off the head standing still: bent 0.10 radians on
  average in an idle, from 0.005. Out of the legs only now, and only one it is clear of at rest.
- *Ties through the body, and between tails in a bunch.* Locks either side of the neck were tied through it, and
  tails 1.4 cm apart on 9 cm bones were held within a band a small bend leaves: an idle bent 0.47 radians for 95
  in a hundred, from 0.22. Two tails aren't tied with a part of the body half way between them at rest, nor nearer
  each other than half the longer of their bones (0.23 then).
- *Each of two tied tails going all the way, not half.* Fewer pairs stretched in the run (3.3% for 5.5%), but the
  points half way were in a leg more often (1.7% for 0.7%).

Not done: a tail hung inside a skirt is taken for one of its folds; ties between chains hanging from different
bones (a skirt hung half from each thigh); a cape against the back (the point half way keeps out of legs only).

**Cost: not measured.** The speed bench's rig (fifteen locks of four bones) has no chains side by side (its tails
are 2 cm apart on 6 cm bones), so its numbers show only whether sharing one bone's step between the two loops
costs anything; they should be as they were. Nothing in CI times tied chains. A tied bone reads two neighbors and
measures to each every step, and where a leg is near, measures the two points half way to it.

**A bone hanging alone** (an ear, an antenna, a tail of one bone) is still not swung by itself. `sway::danglers`
lists the bones `chains` leaves out for that alone, each as a chain of one bone as long as it is far from its
parent, to pick from and give to `Sway::with`. No rule tells them from eyes and jaws. On the bench rigs played from
a rig with no face bones, by where the bone stands and where the middle of its skin does, in head radii from the
head's middle (or the trunk's, for the flaps):

| bone | stands at | its skin's middle at | that far from the bone, of the height |
|---|---|---|---|
| a coat flap in front, one behind (to swing) | 0.97 | 1.34 | 0.05 |
| a fringe (to swing) | 1.14 | 1.12 | 0.045 |
| a jaw | 0.96 | 1.25 | 0.023 |
| eyes, three rigs | 0.73 to 0.92 | 0.77 to 0.97 | 0.002 to 0.005 |
| an eye, the VRM | 0.51 | 0.98 | 0.042 |
| eyes with no skin, guessed head | 0.69 to 1.13 | | |

A jaw's skin stands further out than a fringe's and one eye's as far off its bone as a flap's. Also listed on
those rigs, with nothing to tell them by: a robot's vent, control bones under a root, pole targets, a prop.

**`Live::jump`** says the body is set down elsewhere at the next pose: as `reset` for feet, travel and hands, but
the chains go along (`Sway::jump`). On a walk set down three meters off and turned, the chains are within 0.02
radians of a body that stayed; untold they whip 1.36 radians.

**The demo** bundles no file. Its sample target now wears a ponytail and a skirt of eight folds the source has no
bones for (27 bones in nine chains, eight tied), drawn in their own color, with a row saying how many chains are
left loose. The bench VRMs are 10 MB each under their own licenses and are not bundled; dropped in as the target
they sway the same way.

**VRM spring settings** are not read. `tests/sway.rs` reads `VRMC_springBone` from the GLB's JSON by hand for the
yardstick; a `load` reader needs the same from `src/load.rs` (which already has the JSON) and the mapping of
stiffness, drag and gravity onto `Spring` by matching poles at 60 poses a second, which is not worked out.

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

`Baker` takes clips already on one rig and writes one library; `Library` reads it back and samples it on the CPU;
`bake::wgsl` is the same sampler as WGSL, written beside it function for function. naga parses and validates the
WGSL in the tests, with a vertex shader that calls all of it.

The WGSL now runs on real adapters against the CPU reader: `gpu-check/`, a crate of its own on wgpu, bakes four
retargeted clips on the 76-bone mannequin in each encoding (a walk 4.2 m off played once, a second in place looped,
a turn once round looped, one pose held), uploads the bytes as they are and has compute shaders call `bone`,
`blend`, `skin` and `skin_normal` for 39,764, 9,728 and 2,840 queries: every bone at every frame and between
frames, before 0, past the end, on and either side of the loop's wrap, clips and bones the library doesn't hold.
Locally on an Intel Arc integrated GPU (driver 32.0.101.9150) through Vulkan, DX12 and OpenGL, and on Microsoft's
software DX12 adapter, the worst the shader is off the CPU, over all of them (a rotation's or normal's largest
difference in one number; a place as a share of the rig's height):

| Encoding | `bone` rotation | translation | `blend` rotation | translation | `skin` point | `skin_normal` |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Floats | 1.2e-7 | 4.5e-7 | 1.8e-7 | 5.0e-7 | 6.0e-7 | 3.1e-7 |
| Halves | 1.8e-7 | 4.6e-7 | 1.8e-7 | 4.4e-7 | 5.9e-7 | 4.0e-7 |
| Fixed point | 1.2e-7 | 7.9e-7 | 1.8e-7 | 6.0e-7 | 5.9e-7 | 7.0e-7 |

The check fails past 1e-5, and a CI job is set to run it on Mesa's lavapipe (Vulkan on the CPU). The first run found
one disagreement: the Intel driver divides by multiplying by a reciprocal, so `30.0 / 30.0` is 0.99999994 there, and
a looping clip asked for at exactly its length (or any whole number of them) was not wrapped. It still got the
first frame's pose, as the last frame blended all the way to the first, but where those two lie in opposite
hemispheres the rotation came back with every sign turned. Both samplers now take a whole turn left over as the
start again, and agree.

Little-endian 32-bit words, every offset a count of words from the library's start (`C` clips, `B` bones, `F`
frames in all, `R` words a record: 7 as floats, 4 as halves, 4 in fixed point):

| Part | Words | Holds |
| --- | ---: | --- |
| Header | 8 | magic `DTMO`, version 2, encoding (0 floats, 1 halves, 2 fixed point), `B`, `C`, the library's words, the table's and the pivots' offsets |
| Clip table | 4 × `C` | per clip: first record's offset, frames, frames a second, flags (bit 0: loops) |
| Pivots | 3 × `B` | per bone: where its joint stands at bind, as floats |
| Boxes | 6 × `B` × `C` | fixed point only, each clip's right ahead of its records; per bone: the least corner and the size of the box its joint keeps to through the clip, as floats |
| Records | `R` × `B` × `F` | per clip, frame and bone: rotation from bind (x, y, z, w), the joint's place (x, y, z) |

`4 × (8 + 4C + 3B + R·B·F)` bytes, and `4 × 6·B·C` more in fixed point. Fifty clips of 3 s at 30 Hz on 80 bones:
5,761,792 bytes as halves, 5,857,792 in fixed point and 10,081,792 as floats looping (90 frames each); 5,825,792,
5,921,792 and 10,193,792 played once through (91 each). Version 2 added fixed point and nothing else: version 1
libraries are still read, and are what version 2 is with floats or halves.

How far a skinned point is put from where the clip has it, at the frames, as a share of the rig's height, on the
test clips (every bone swinging, 90 frames, 76 bones):

| Encoding | Bytes | In place: any bone | fingertips | Walking 4.2 m off: any bone | fingertips |
| --- | ---: | ---: | ---: | ---: | ---: |
| Floats | 192,480 | 6.6e-7 | 6.0e-7 | 8.0e-7 | 8.0e-7 |
| Halves | 110,400 | 5.8e-4 | 5.7e-4 | 9.0e-4 | 8.8e-4 |
| Fixed point | 112,224 | 1.0e-5 | 8.7e-6 | 2.4e-5 | 2.2e-5 |
| Fixed point, one box a clip (not kept) | 110,424 | 1.8e-5 | 1.7e-5 | 2.6e-5 | 2.5e-5 |

What changed from the plan above:

- **A record holds where the joint stands, not the skinning translation.** The translation (the world transform
  times the inverse bind) is the small difference of two long arms, and blending it between frames or clips pulls
  joints toward the rig's origin: two to five times further off half way between frames on the test clips. The
  sampler blends the joint's place, then takes the pivot off (`place − rotation × pivot`), which costs three words a
  bone once and one more read a sample. What `bone` hands back is still the skinning rotation and translation.
- **No scale.** Scales at bind cancel. Scale a clip animates is dropped: the bone keeps its bind size about its
  joint, which still stands where the clip puts it. Retargeted clips keep the target's scales.
- **Halves are weak for places, so there is fixed point.** A half places a joint within a 2048th of how far out it
  stands: 0.0006 of the rig's height at the fingertips in place, 0.0009 three heights from the origin, against a
  millionth as floats. The third encoding spreads sixteen bits evenly over the box each bone's joint keeps to
  through each clip (its least corner and its size, six floats a bone a clip, ahead of the clip's records), and the
  rotation's four numbers evenly from -1 to 1. It is 40 to 65 times closer than halves for 1.7% more bytes on a
  3 s clip, and no worse far from the origin than near it: what is left is half a 65535th of how far the joint
  moves, along each axis.
  - *A box a bone, not a box a clip.* One box for the whole rig saves those 1.7% and puts points 1.8 times further
    off in place, 1.1 times on the walk, where every box is as long as the walk either way. Error times bytes is
    lower a box a bone on both. On a clip of a frame or two the boxes cost more than the records (six words a bone
    against four a frame), and are still the nearer way to place a joint far from the origin.
  - *Rotations as signed 16-bit counts, not halves.* A half keeps 11 bits of each number, so a stored rotation is
    up to 0.0007 of a radian off, which alone moves a point a twentieth of the height from its joint by 3.5e-5 of
    the height: more than fixed-point places leave. A count of a 32767th is up to 0.00006 of a radian off, costs
    the same 16 bits and is cheaper to unpack. Smallest-three was not needed.
  - The eighth 16 bits of a record are still zero. After the rotation a record has 64 bits left, room for a place
    of three 21-bit counts, 32 times finer; nothing asks for it yet.
- **Between frames** the sampler turns each bone in the world, the clip under its parent: under a thousandth of
  the height on a walk at 30 Hz, up to four on a clip with every bone swinging hard. A higher rate buys that back.
- **A looping clip's last pose is taken to be its first** and isn't stored, and each clip's rate is nudged so its
  frames fit its length exactly.

What a host does: upload the bytes as they are to a storage buffer and bind it read-only where it told `wgsl` to
declare `library: array<u32>` (one binding for any encoding; halves are unpacked in WGSL, as `f16` is an
extension, and so is fixed point). Give each instance a clip and a time, and for a cross-fade a second clip and time and a weight. Call
`bone(clip, time, index)` or `blend(clip_a, time_a, clip_b, time_b, weight, index)` for each of a vertex's four
bones and `skin(position, a, b, c, d, weights)`, or call them once a bone in a compute pass that writes a pose
buffer, which is the only way in WebGPU's compatibility mode (no storage buffers in the vertex stage) and the
cheaper one for dense meshes. The WGSL's names are plain (`bone`, `blend`, `skin`, `Bone`, `Header`, `Entry`,
`library`): a host whose own shader uses one must rename.
