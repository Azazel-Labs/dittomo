// Drives the built demo the way the page does, without a browser: the sample, the match, a clip with every fix,
// then each model under ../tests/data through the buffer. Run after build.sh: node check.mjs
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

const data = join(here, "../tests/data");
for (const name of readdirSync(data).filter((f) => [".fbx", ".glb", ".gltf"].includes(extname(f)))) {
  const ext = new TextEncoder().encode(extname(name).slice(1));
  const body = readFileSync(join(data, name));
  const into = new Uint8Array(w.memory.buffer, w.buffer(ext.length + body.length), ext.length + body.length);
  into.set(ext);
  into.set(body, ext.length);
  check(w.load(0, ext.length), `load ${name}`);
  console.log(`read ${name}`);
}
console.log("ok");
