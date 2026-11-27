// Drives the built demo the way the page does, without a browser: the sample, the match, a clip with every fix,
// the sample's clip as BVH read back, then each model under ../tests/data through the buffer, and a clip played on one
// of them downloaded and read back.
// Run after build.sh: node check.mjs
import { readFileSync, readdirSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join, extname } from "node:path";

const here = dirname(fileURLToPath(import.meta.url));
const { instance: { exports: w } } = await WebAssembly.instantiate(readFileSync(join(here, "www/dittomo_demo.wasm")), {});
const text = () => new TextDecoder().decode(new Uint8Array(w.memory.buffer, w.text(), w.text_len()));
const check = (ok, what) => { if (!ok) { console.error(`failed: ${what}: ${text()}`); process.exit(1); } };

w.sample();
check(w.map(), "map the sample");
const report = JSON.parse(text());
// A quarter of the target's bones are a ponytail and a skirt the source has nothing for, which count against it.
check(report.quality > 0.45, `quality ${report.quality}`);
check(report.target.legs.length === 2 && report.target.arms.length === 2, "two legs and two arms");
for (const [clip, fixes] of [[0, 0], [1, 15]]) {
  const t = performance.now();
  const frames = w.play(clip, fixes);
  check(frames > 0, `play clip ${clip}`);
  const n = report.target.bones.length;
  const at = new Float32Array(w.memory.buffer, w.places_of(1), frames * n * 3);
  const z = (f) => { let s = 0; for (let b = 0; b < n; b++) s += at[(f * n + b) * 3 + 2] / n; return s; };
  check(Number.isFinite(z(frames - 1)), "finite places");
  const went = z(frames - 1) - z(0);
  check(went > 100, `clip ${clip} with fixes ${fixes} travels (${went.toFixed(1)})`);
  console.log(`clip ${clip}, fixes ${fixes}: ${frames} frames in ${(performance.now() - t).toFixed(1)} ms, traveled ${went.toFixed(0)}`);
}

// The sample's target wears a ponytail and a skirt of eight folds that the source has no bones for: found
// as chains, the skirt's tied side by side, stiff without the sway fix and swung with it.
// (27 of its bones or 28: the ponytail's first is taken for the end of the source's head, or not.)
check(report.chains.length === 9 && [27, 28].includes(report.chains.flat().length) && report.tied === 8,
  `9 chains of 27 or 28 bones, 8 side by side (${report.chains.length}, ${report.chains.flat().length}, ${report.tied})`);
{
  const n = report.target.bones.length, loose = report.chains.flat();
  const places = (fixes) => {
    const frames = w.play(0, fixes);
    check(frames > 0, `play with fixes ${fixes}`);
    return [frames, new Float32Array(w.memory.buffer, w.places_of(1), frames * n * 3).slice()];
  };
  const [frames, stiff] = places(15), [, swung] = places(15 | 256);
  let most = 0, others = 0;
  for (let f = 0; f < frames; f++) {
    for (let b = 0; b < n; b++) {
      const o = (f * n + b) * 3;
      const d = Math.hypot(swung[o] - stiff[o], swung[o + 1] - stiff[o + 1], swung[o + 2] - stiff[o + 2]);
      check(Number.isFinite(d), "finite places with sway");
      // A chain's first bone starts where its parent has it either way; the rest go where it swings.
      if (loose.includes(b)) most = Math.max(most, d); else others = Math.max(others, d);
    }
  }
  check(most > 2 && most < 60, `the loose bones swing (${most.toFixed(1)} cm at most)`);
  check(others === 0, `only the loose bones move (${others})`);
  console.log(`sway: ${report.chains.length} chains of ${loose.length} bones, ${report.tied} side by side, moved ` +
    `${most.toFixed(1)} cm at most from where they are stiff`);
  check(w.play(1, 15) > 0, "play clip 1 again");
}

check(!w.export(0), "no download of the sample, which has no file");

const load = (slot, extension, body) => {
  const ext = new TextEncoder().encode(extension);
  const at = w.buffer(ext.length + body.length); // first, as it may grow the memory
  const into = new Uint8Array(w.memory.buffer, at, ext.length + body.length);
  into.set(ext);
  into.set(body, ext.length);
  return w.load(slot, ext.length);
};
// The sample's clip as BVH, which needs no file, read back as the source and mapped onto the sample it came from.
let t = performance.now();
const bvhSize = w.export(1);
check(bvhSize > 0, "BVH of the sample's clip");
const bvh = new Uint8Array(w.memory.buffer, w.out(), bvhSize).slice();
const bvhTook = performance.now() - t;
check(new TextDecoder().decode(bvh.subarray(0, 9)) === "HIERARCHY", "a BVH");
check(load(0, "bvh", bvh) && w.map(), "read the BVH back");
const fromBvh = JSON.parse(text());
check(fromBvh.quality > 0.9, `the BVH maps onto the sample it came from (${fromBvh.quality})`);
console.log(`BVH of the sample's clip: ${bvhSize} bytes in ${bvhTook.toFixed(1)} ms, quality back ` +
  `${fromBvh.quality.toFixed(2)}`);

const data = join(here, "../tests/data");
const models = readdirSync(data).filter((f) => [".fbx", ".glb", ".gltf"].includes(extname(f)));
for (const name of models) {
  check(load(0, extname(name).slice(1), readFileSync(join(data, name))), `load ${name}`);
  console.log(`read ${name}`);
}

// The first model's clip played on itself, downloaded, and read back with that clip alone.
const name = models[0], body = readFileSync(join(data, name));
check(load(0, extname(name).slice(1), body) && load(1, extname(name).slice(1), body), `load ${name} twice`);
check(w.map(), `map ${name} to itself`);
const frames = w.play(0, 0);
check(frames > 0, `play ${name}`);
t = performance.now();
const size = w.export(0);
check(size > 0, `download ${name}`);
const glb = new Uint8Array(w.memory.buffer, w.out(), size).slice();
check(new TextDecoder().decode(glb.subarray(0, 4)) === "glTF", "a GLB");
const took = performance.now() - t;
check(load(0, "glb", glb), "read the download back");
check(w.map(), "map the download");
const back = JSON.parse(text());
check(back.clips.length === 1 && back.clips[0].frames <= frames, `one clip back (${JSON.stringify(back.clips)})`);
console.log(`downloaded ${name} with its clip: ${size} bytes in ${took.toFixed(1)} ms, ${back.clips[0].frames} of ` +
  `${frames} poses keyed`);

// The same as FBX, read back with its clip.
check(load(0, extname(name).slice(1), body) && load(1, extname(name).slice(1), body) && w.map(), `map ${name} again`);
check(w.play(0, 0) === frames, `play ${name} again`);
t = performance.now();
const fbxSize = w.export(2);
check(fbxSize > 0, `download ${name} as FBX`);
const fbx = new Uint8Array(w.memory.buffer, w.out(), fbxSize).slice();
const fbxTook = performance.now() - t;
check(new TextDecoder().decode(fbx.subarray(0, 18)) === "Kaydara FBX Binary", "an FBX");
check(load(0, "fbx", fbx), "read the FBX back");
check(w.map(), "map the FBX");
const fbxBack = JSON.parse(text());
check(fbxBack.clips.length === 1, `one clip back from the FBX (${JSON.stringify(fbxBack.clips)})`);
console.log(`downloaded ${name} with its clip as FBX: ${fbxSize} bytes in ${fbxTook.toFixed(1)} ms`);
console.log("ok");
