// Plays the retarget benchmark's clip through three.js SkeletonUtils, to compare with dittomo on the same rigs, clip and
// bone pairs. Write the rigs with `cargo bench --bench retarget -- --dump target/rigs.json`, then
// `npm ci --ignore-scripts && node retarget.mjs ../../target/rigs.json` here.
//
// three.js is given dittomo's mapping as its bone names, so only playing is timed. Its `retarget` copies each mapped
// bone's world rotation, with no correction for rests that differ, so it does less than dittomo per bone.

import { readFileSync } from 'node:fs';
import { performance } from 'node:perf_hooks';

import * as THREE from 'three';
import * as SkeletonUtils from 'three/addons/utils/SkeletonUtils.js';

const path = process.argv[2];
if (!path) {
  console.error('usage: node retarget.mjs <rigs.json>');
  process.exit(2);
}
const rigs = JSON.parse(readFileSync(path, 'utf8'));
// Track names can't hold `:` (three.js's PropertyBinding reserves it), so Mixamo's prefix is written with `_`.
const clean = (name) => THREE.PropertyBinding.sanitizeNodeName(name);

/** A three.js object holding a skeleton built from the dumped bones at rest. */
function build(list) {
  const bones = list.map((b) => {
    const bone = new THREE.Bone();
    bone.name = clean(b.name);
    set(bone, b.rest);
    return bone;
  });
  const root = new THREE.Group();
  list.forEach((b, i) => (b.parent === null ? root : bones[b.parent]).add(bones[i]));
  root.updateMatrixWorld(true);
  root.skeleton = new THREE.Skeleton(bones);
  return root;
}

function set(bone, t) {
  bone.position.set(t[0], t[1], t[2]);
  bone.quaternion.set(t[3], t[4], t[5], t[6]);
  bone.scale.set(t[7], t[8], t[9]);
}

const source = build(rigs.source);
const target = build(rigs.target);
const names = Object.fromEntries(Object.entries(rigs.names).map(([t, s]) => [clean(t), clean(s)]));
const hip = clean(rigs.source[0].name);
const frames = rigs.frames.length;

// The clip as three.js keeps one: a rotation track per source bone and the hips' position.
const tracks = rigs.source.map((b, i) => {
  const values = rigs.frames.flatMap((f) => f[i].slice(3, 7));
  return new THREE.QuaternionKeyframeTrack(`${clean(b.name)}.quaternion`, rigs.times, values);
});
tracks.push(
  new THREE.VectorKeyframeTrack(`${hip}.position`, rigs.times, rigs.frames.flatMap((f) => f[0].slice(0, 3))),
);
const clip = new THREE.AnimationClip('walk', -1, tracks);

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
const options = () => ({ names, hip, fps: frames / clip.duration });

let played;
const play =
  time(budget, () => {
    played = SkeletonUtils.retargetClip(target, source.skeleton, clip, options());
  }) / played.tracks[0].times.length;

const opts = options();
const pose =
  time(budget, () => {
    for (const f of rigs.frames) {
      source.skeleton.bones.forEach((bone, i) => set(bone, f[i]));
      source.updateMatrixWorld(true);
      SkeletonUtils.retarget(target, source, opts);
    }
  }) / frames;

const finite = target.skeleton.bones.every((b) => b.quaternion.toArray().every(Number.isFinite));
if (!finite || played.tracks.length === 0) {
  console.error('three.js played nothing usable');
  process.exit(1);
}
console.log(`three.js ${THREE.REVISION}, Node ${process.versions.node}`);
console.log(`| Case | µs | per |\n| --- | ---: | --- |`);
console.log(`| ${rigs.source.length} → ${rigs.target.length}: retargetClip | ${play.toFixed(2)} | frame |`);
console.log(`| ${rigs.source.length} → ${rigs.target.length}: retarget | ${pose.toFixed(2)} | frame, rigs reused |`);
