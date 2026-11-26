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
- Set up by itself: stiffness from the length left to the tip (a pendulum's, stiffer for chains standing against
  gravity), damping 0.45 to 0.7, radius from bone length or the skin. Fingers, faces, IK controls and props left out.
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
- The defaults are floppy: a vigorous dance drives long chains to the cone, and real walks and runs swing a VRoid
  rig's chains so hard they never settle into a round. To tune at the defaults stage, before the fix is on by
  default.
- A pose with no time passed holds the chains (each bone points at where its tail was); only the first puts them
  at rest. They start moving as the body does by the second pose, so a clip that starts mid-stride doesn't whip
  them back.
- The body is the one arms keep out of, as rods with round ends: spine, neck, arms and legs, and the head's ball
  (fitted to the skin where `Live` and `apply_touching` know it). No rig-made colliders, no skin-fitted limbs, and
  nothing for a body with no arms found (animals): only the floor. A particle slides round its bone's head to get
  out, so lengths hold; no pair comes nearer than at rest, so rest stays a fixed point. Getting out wins over the
  cone: a rope let down lies flat, and a short bone whose head a thigh swings through is turned straight out of
  it (5 to 7% of chain bones' poses past their cones in a run on the VRoid rigs).
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
