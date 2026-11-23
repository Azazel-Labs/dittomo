// Plays the retarget benchmark's clip through Babylon.js's AnimatorAvatar, to compare with dittomo on the same rigs, clip
// and bone pairs. Write the rigs with `cargo bench --bench retarget -- --dump target/rigs.json`, then
// `npm ci --ignore-scripts && node retarget.mjs ../../target/rigs.json` here.
//
// Babylon is given dittomo's mapping as its node names, so only retargeting is timed. `retargetAnimationGroup` rewrites
// every key of a clip once (a bake, like three.js's `retargetClip`); it doesn't play a pose at a time. Each key moves
// the source bone's change from rest into the target bone's rest frame, by four 4×4 matrix products and a decompose.

import { readFileSync } from 'node:fs';
import { performance } from 'node:perf_hooks';

import { NullEngine } from '@babylonjs/core/Engines/nullEngine.js';
import { Scene } from '@babylonjs/core/scene.js';
import { TransformNode } from '@babylonjs/core/Meshes/transformNode.js';
import { Mesh } from '@babylonjs/core/Meshes/mesh.js';
import { VertexData } from '@babylonjs/core/Meshes/mesh.vertexData.js';
import { Skeleton } from '@babylonjs/core/Bones/skeleton.js';
import { Bone } from '@babylonjs/core/Bones/bone.js';
import { Animation } from '@babylonjs/core/Animations/animation.js';
import { AnimationGroup } from '@babylonjs/core/Animations/animationGroup.js';
import { AnimatorAvatar } from '@babylonjs/core/Animations/animatorAvatar.js';
import { Matrix, Quaternion, Vector3 } from '@babylonjs/core/Maths/math.vector.js';
import { Logger } from '@babylonjs/core/Misc/logger.js';
import '@babylonjs/core/Animations/animatable.js';

const path = process.argv[2];
if (!path) {
  console.error('usage: node retarget.mjs <rigs.json>');
  process.exit(2);
}
const rigs = JSON.parse(readFileSync(path, 'utf8'));
Logger.LogLevels = Logger.ErrorLogLevel;

const engine = new NullEngine();
const scene = new Scene(engine);

/** Transform nodes for the dumped bones at rest, under a root node. */
function nodes(list, prefix) {
  const root = new TransformNode(`${prefix}root`, scene);
  const made = list.map((b) => {
    const n = new TransformNode(b.name, scene);
    const t = b.rest;
    n.position = new Vector3(t[0], t[1], t[2]);
    n.rotationQuaternion = new Quaternion(t[3], t[4], t[5], t[6]);
    n.scaling = new Vector3(t[7], t[8], t[9]);
    return n;
  });
  list.forEach((b, i) => (made[i].parent = b.parent === null ? root : made[b.parent]));
  return { root, made };
}

// The source is plain transform nodes, as a glTF file loads; the target a skinned mesh whose bones those nodes drive,
// which is what an avatar looks for.
const source = nodes(rigs.source, 's:');
const target = nodes(rigs.target, 't:');
const skeleton = new Skeleton('target', 'target', scene);
const bones = [];
rigs.target.forEach((b, i) => {
  const n = target.made[i];
  const local = Matrix.Compose(n.scaling, n.rotationQuaternion, n.position);
  const bone = new Bone(b.name, skeleton, b.parent === null ? null : bones[b.parent], local, local.clone());
  bone.linkTransformNode(n);
  bones.push(bone);
});
const mesh = new Mesh('body', scene);
const box = new VertexData();
box.positions = [0, 0, 0, 1, 0, 0, 0, 1, 0];
box.indices = [0, 1, 2];
box.applyToMesh(mesh);
mesh.skeleton = skeleton;
mesh.parent = target.root;
const avatar = new AnimatorAvatar('target', target.root, false, false);
avatar.showWarnings = false;

// The clip as Babylon keeps one: a rotation animation per source bone and the hips' position, keyed at 60 frames a
// second.
const fps = 60;
const frames = rigs.frames.length;
const group = new AnimationGroup('walk', scene);
rigs.source.forEach((b, i) => {
  const rot = new Animation(`${b.name}.rot`, 'rotationQuaternion', fps, Animation.ANIMATIONTYPE_QUATERNION);
  rot.setKeys(
    rigs.frames.map((f, k) => ({ frame: rigs.times[k] * fps, value: new Quaternion(f[i][3], f[i][4], f[i][5], f[i][6]) })),
  );
  group.addTargetedAnimation(rot, source.made[i]);
});
const pos = new Animation(`${rigs.source[0].name}.pos`, 'position', fps, Animation.ANIMATIONTYPE_VECTOR3);
pos.setKeys(rigs.frames.map((f, k) => ({ frame: rigs.times[k] * fps, value: new Vector3(f[0][0], f[0][1], f[0][2]) })));
group.addTargetedAnimation(pos, source.made[0]);

// Babylon looks target bones up by the source node's name after this map: source name → target name.
const mapNodeNames = new Map(Object.entries(rigs.names).map(([t, s]) => [s, t]));

/** The median time of `f` in microseconds, run as often as fits `budget` seconds (at least five times). */
function time(budget, f) {
  for (let i = 0; i < 20; i++) f(); // warm the JIT up
  const samples = [];
  const start = performance.now();
  while (samples.length < 5 || performance.now() - start < budget * 1000) {
    const t = performance.now();
    f();
    samples.push((performance.now() - t) * 1000);
  }
  samples.sort((a, b) => a - b);
  return samples[samples.length >> 1];
}

const budget = Number(process.argv[3] ?? 1);
let played;
const play =
  time(budget, () => {
    played?.dispose();
    played = avatar.retargetAnimationGroup(group, { mapNodeNames, fixRootPosition: false });
  }) / frames;

// The same with the hips' path scaled to the target's size, as dittomo does: Babylon plays both clips key by key for it.
const hips = rigs.source[0].name;
const foot = rigs.names[Object.keys(rigs.names).find((t) => /foot/i.test(t))];
let scaled;
const scale =
  time(budget, () => {
    scaled?.dispose();
    scaled = avatar.retargetAnimationGroup(group, {
      mapNodeNames,
      rootNodeName: hips,
      groundReferenceNodeName: foot,
    });
  }) / frames;

const keys = played.targetedAnimations.flatMap((ta) => ta.animation.getKeys());
const finite = keys.every((k) => (k.value.asArray?.() ?? []).every(Number.isFinite));
if (!finite || played.targetedAnimations.length === 0) {
  console.error('Babylon.js retargeted nothing usable');
  process.exit(1);
}
console.log(`Babylon.js ${NullEngine.Version}, Node ${process.versions.node}`);
console.log(`| Case | µs | per |\n| --- | ---: | --- |`);
console.log(`| ${rigs.source.length} → ${rigs.target.length}: retargetAnimationGroup | ${play.toFixed(2)} | frame |`);
console.log(`| ${rigs.source.length} → ${rigs.target.length}: retargetAnimationGroup, hips scaled | ${scale.toFixed(2)} | frame |`);
