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
check(report.quality > 0.5, `quality ${report.quality}`);
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
  console.log(`clip ${clip}, fixes ${fixes}: ${frames} frames in ${(performance.now() - t).toFixed(1)} ms, travelled ${went.toFixed(0)}`);
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
console.log("ok");
