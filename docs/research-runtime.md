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

Stages: find chains; one chain's integrator against the analytic spring and across frame rates; colliders and
limits; `Live`, whole clips, baking; defaults; VRM and KawaiiPhysics import; `Crowd` lanes; links between sibling
chains (skirts), telling baked jiggle from intent, resampling a 3-bone chain onto 8.

## The GPU

- **The transfer needs no hierarchy on the target's side** already: with the source's turns kept in the world, a
  target bone's skinning turn is one or two quaternion products. Positions need a sum up the ancestors only.
- **The win is moving data, not arithmetic**: `Crowd` is 6.2 ms for 10,000 characters on one core, but their poses
  are 23 MB uploaded a frame, against 32 to 48 bytes an instance if the GPU does the work.

In order, after the CPU versions:

1. **Bake once, play on the GPU.** A retargeted clip library as a texture per target rig (a quaternion and a
   translation per bone per frame, half floats: 5.8 MB for 50 clips of 3 s at 30 Hz and 80 bones), sampled in the
   vertex shader, two clips cross-faded. CPU code only, every fix kept, runs on WebGL2. Babylon's baked layout can
   be written as it is.
2. **World-space source clips and flat mapping tables**, used by `Crowd` itself: the groundwork for the rest.
3. **Retarget on the GPU per instance**: one source library, about 8 KB per target rig, one compute pass with a
   thread per character. For many body types, big libraries, and live or generated sources.
4. **Spring chains in compute**: a thread per chain, no coloring needed, reading and writing the pose buffer.
5. **Two-bone IK, floor and push-out on the GPU**, from contacts baked into the clip. Hands together, hands on the
   head and touch stay on the CPU, for heroes.
6. Batched inference for the learned runtime in the plan, last.

Left to hosts: skinning, instancing, culling, LOD, per-vertex animation textures, terrain queries.

**To shape now on the CPU** so the port is mechanical: the per-bone plan as one flat record (no enum, no `Option`,
flags not `Vec<bool>`), a world-space source input and a world-space output for `Crowd`, lean and twist in lanes,
travel without matrices, contacts as channels of a clip, and one versioned export of the tables.

Numbers others publish: 9,547 instanced characters at 34 fps in 2007 (GPU Gems 3); 10 to 15 thousand skinned meshes
over 150 fps on a GTX 1060 (gpuinstance); Alan Wake 2's GPU skinning aimed at a million bones.
