// The page for dittomo's wasm demo. The module has no imports: files go in through a buffer it owns, results come out
// of its memory at the pointers its exports give.

const $ = (id) => document.getElementById(id);
const ui = {
  err: $("err"), view: $("view"), playing: $("playing"), scrub: $("scrub"), clip: $("clip"), save: $("save"),
  travel: $("travel"), feet: $("feet"), push: $("push"), touch: $("touch"), hands: $("hands"), head: $("head"), sample: $("sample"), stats: $("stats"), pairs: $("pairs"),
};

let wasm;
let report = null;       // what map() said
let frames = 0, times = null, places = [null, null];
let frame = 0, playing = true, clock = 0, last = 0;
let yaw = -0.6, pitch = 0.18, hovered = -1;
const names = ["", ""];
let targetFile = null;   // the target's file name, when it came from one

const text = () => new TextDecoder().decode(new Uint8Array(wasm.memory.buffer, wasm.text(), wasm.text_len()));
const fail = (m) => { ui.err.textContent = m; };

async function start() {
  const bytes = await (await fetch("dittomo_demo.wasm")).arrayBuffer();
  ({ instance: { exports: wasm } } = await WebAssembly.instantiate(bytes, {}));
  sample();
}

function sample() {
  wasm.sample();
  names[0] = "sample: 65 bones laid out as Mixamo's, walking";
  names[1] = "sample: 76-bone mannequin, A-pose, twist bones";
  targetFile = null;
  remap();
}

async function load(slot, file) {
  const ext = new TextEncoder().encode(file.name.split(".").pop().toLowerCase());
  const body = new Uint8Array(await file.arrayBuffer());
  const at = wasm.buffer(ext.length + body.length);
  const into = new Uint8Array(wasm.memory.buffer, at, ext.length + body.length);
  into.set(ext); into.set(body, ext.length);
  if (!wasm.load(slot, ext.length)) return fail(`${file.name}: ${text()}`);
  names[slot] = `${file.name}, ${(file.size / 1e6).toFixed(1)} MB`;
  if (slot === 1) targetFile = file.name;
  remap();
}

function remap() {
  fail("");
  const t0 = performance.now();
  if (!wasm.map()) { report = null; draw(); return fail(text()); }
  const took = performance.now() - t0;
  report = JSON.parse(text());
  report.took = took;
  ui.clip.replaceChildren(...report.clips.map((c, i) =>
    new Option(`${c.name} (${c.frames} frames, ${c.seconds.toFixed(1)} s)`, i)));
  if (!report.clips.length) ui.clip.append(new Option("rest pose (no clips)", 0));
  table();
  play();
}

function play() {
  if (!report) return;
  const fixes = (ui.travel.checked ? 1 : 0) | (ui.feet.checked ? 2 : 0) | (ui.push.checked ? 4 : 0) | (ui.touch.checked ? 8 : 0) | (ui.hands.checked ? 16 : 0) | (ui.head.checked ? 32 : 0);
  const t0 = performance.now();
  frames = wasm.play(+ui.clip.value || 0, fixes);
  report.played = performance.now() - t0;
  if (!frames) return fail(text());
  const mem = wasm.memory.buffer;
  times = new Float32Array(mem, wasm.times(), frames).slice();
  places = [0, 1].map((w) =>
    new Float32Array(mem, wasm.places_of(w), frames * bones(w).length * 3).slice());
  frame = Math.min(frame, frames - 1);
  clock = times[frame] - times[0];
  ui.scrub.max = frames - 1;
  ui.save.disabled = !targetFile;
  ui.save.title = targetFile ? "" : "Load a target model from a file to download it with the clip";
  stats();
}

// The target's file with the clip just played in place of its own, named after both; a VRM stays a .vrm.
function save() {
  const n = wasm.export();
  if (!n) return fail(text());
  const glb = new Uint8Array(wasm.memory.buffer, wasm.glb(), n).slice();
  const [, base, ext] = targetFile.match(/^(.*?)(?:\.([^.]*))?$/);
  const clip = report.clips[+ui.clip.value]?.name ?? "rest";
  const a = document.createElement("a");
  a.href = URL.createObjectURL(new Blob([glb], { type: "model/gltf-binary" }));
  a.download = `${base}-${clip}`.replace(/[^\w.-]+/g, "-") + (ext?.toLowerCase() === "vrm" ? ".vrm" : ".glb");
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 10000);
}

const bones = (w) => report[w ? "target" : "source"].bones;

function stats() {
  const mapped = report.pairs.filter((p) => p).length;
  const rows = [
    ["Source", names[0]], ["Target", names[1]],
    ["Bones", `${report.source.bones.length} → ${report.target.bones.length}, ${mapped} matched`],
    ["Quality", `${(report.quality * 100).toFixed(1)}%`],
    ["Limbs found", `${report.target.legs.length} legs, ${report.target.arms.length} arms`],
    ["Match took", `${report.took.toFixed(1)} ms`],
    ["Play took", `${report.played.toFixed(1)} ms for ${frames} frames, ${(report.played * 1000 / frames).toFixed(0)} µs each`],
  ];
  ui.stats.replaceChildren(...rows.flatMap(([k, v]) => {
    const dt = document.createElement("dt"), dd = document.createElement("dd");
    dt.textContent = k; dd.textContent = v; return [dt, dd];
  }));
}

function table() {
  const src = report.source.bones, tgt = report.target.bones;
  ui.pairs.replaceChildren(...tgt.map((b, i) => {
    const p = report.pairs[i];
    const tr = document.createElement("tr");
    const cell = (t, cls) => { const td = document.createElement("td"); td.textContent = t; if (cls) td.className = cls; tr.append(td); return td; };
    cell(b.name);
    cell(p ? src[p.source].name : "—");
    const s = cell(p ? p.score.toFixed(2) : "", "score");
    if (p) s.style.color = `hsl(${Math.round(120 * Math.max(0, Math.min(1, p.score)))} 60% 45%)`;
    tr.onmouseenter = () => { hovered = i; };
    tr.onmouseleave = () => { hovered = -1; };
    return tr;
  }));
}

// Drawing: each skeleton in its own half, followed so it stays in view, over a floor that slides as it travels.

function at(w, f, b) {
  const o = (f * bones(w).length + b) * 3, p = places[w];
  return [p[o], p[o + 1], p[o + 2]];
}

function centre(w, f) {
  const n = bones(w).length, c = [0, 0, 0];
  for (let b = 0; b < n; b++) { const p = at(w, f, b); c[0] += p[0] / n; c[2] += p[2] / n; }
  return c;
}

function height(w) {
  let lo = Infinity, hi = -Infinity;
  for (let b = 0; b < bones(w).length; b++) { const y = at(w, 0, b)[1]; lo = Math.min(lo, y); hi = Math.max(hi, y); }
  return Math.max(hi - lo, 1e-3);
}

function draw() {
  const c = ui.view, dpr = devicePixelRatio || 1;
  const W = c.clientWidth, H = c.clientHeight;
  if (c.width !== W * dpr || c.height !== H * dpr) { c.width = W * dpr; c.height = H * dpr; }
  const g = c.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, W, H);
  if (!report || !frames) return;
  const css = getComputedStyle(document.documentElement);
  const color = [css.getPropertyValue("--source"), css.getPropertyValue("--target")];
  const line = css.getPropertyValue("--line"), muted = css.getPropertyValue("--muted");
  const highlight = hovered < 0 ? [-1, -1] : [report.pairs[hovered]?.source ?? -1, hovered];
  const [cy, sy, cp, sp] = [Math.cos(yaw), Math.sin(yaw), Math.cos(pitch), Math.sin(pitch)];
  const narrow = W < 520;

  for (const w of [0, 1]) {
    const h = height(w), n = bones(w).length;
    const root = centre(w, frame);
    const half = narrow ? { x: 0, y: w * H / 2, w: W, h: H / 2 } : { x: w * W / 2, y: 0, w: W / 2, h: H };
    const s = (half.h * 0.72) / h;
    const ox = half.x + half.w / 2, oy = half.y + half.h * 0.86;
    const ground = Math.min(...Array.from({ length: n }, (_, b) => at(w, 0, b)[1]));
    const project = ([x, y, z]) => {
      x -= root[0]; z -= root[2]; y -= ground;
      const rx = cy * x + sy * z, rz = -sy * x + cy * z;
      return [ox + s * rx, oy - s * (cp * y - sp * rz)];
    };
    // Floor
    g.strokeStyle = line; g.lineWidth = 1;
    const step = h / 4, span = 5;
    for (let k = -span; k <= span; k++) {
      for (const along of [0, 2]) {
        const off = root[along] % step;
        const a = [0, ground, 0], b = [0, ground, 0];
        const across = 2 - along;
        a[along] = b[along] = root[along] + k * step - off;
        a[across] = root[across] - span * step; b[across] = root[across] + span * step;
        g.beginPath(); g.moveTo(...project(a)); g.lineTo(...project(b)); g.stroke();
      }
    }
    // Bones
    const limbs = new Set([...report[w ? "target" : "source"].legs, ...report[w ? "target" : "source"].arms].flat());
    for (let b = 0; b < n; b++) {
      const p = bones(w)[b].parent;
      if (p == null) continue;
      const on = b === highlight[w] || p === highlight[w];
      g.strokeStyle = on ? css.getPropertyValue("--accent") : color[w];
      g.globalAlpha = on || limbs.has(b) ? 1 : 0.6;
      g.lineWidth = on ? 4 : limbs.has(b) ? 2.5 : 1.5;
      g.beginPath(); g.moveTo(...project(at(w, frame, p))); g.lineTo(...project(at(w, frame, b))); g.stroke();
    }
    g.globalAlpha = 1;
    if (highlight[w] >= 0) {
      const [x, y] = project(at(w, frame, highlight[w]));
      g.fillStyle = css.getPropertyValue("--accent"); g.beginPath(); g.arc(x, y, 5, 0, 7); g.fill();
    }
    g.fillStyle = muted; g.font = "12px system-ui, sans-serif";
    g.fillText(w ? "target" : "source", half.x + 12, half.y + 20);
  }
}

function tick(now) {
  const dt = Math.min((now - last) / 1000, 0.1); last = now;
  if (playing && frames > 1) {
    const length = times[frames - 1] - times[0];
    clock = (clock + dt) % Math.max(length, 1e-3);
    while (frame < frames - 1 && times[frame + 1] - times[0] <= clock) frame++;
    if (times[frame] - times[0] > clock) { frame = 0; while (frame < frames - 1 && times[frame + 1] - times[0] <= clock) frame++; }
    ui.scrub.value = frame;
  }
  draw();
  requestAnimationFrame(tick);
}

// Wiring

for (const drop of document.querySelectorAll(".drop")) {
  const slot = +drop.dataset.slot, input = drop.querySelector("input");
  input.onchange = () => input.files[0] && load(slot, input.files[0]);
  drop.ondragover = (e) => { e.preventDefault(); drop.classList.add("over"); };
  drop.ondragleave = () => drop.classList.remove("over");
  drop.ondrop = (e) => { e.preventDefault(); drop.classList.remove("over"); e.dataTransfer.files[0] && load(slot, e.dataTransfer.files[0]); };
}
ui.playing.onclick = () => { playing = !playing; ui.playing.textContent = playing ? "Pause" : "Play"; };
ui.scrub.oninput = () => { frame = +ui.scrub.value; clock = times ? times[frame] - times[0] : 0; };
for (const el of [ui.clip, ui.travel, ui.feet, ui.push, ui.touch, ui.hands, ui.head]) el.onchange = play;
ui.sample.onclick = sample;
ui.save.onclick = save;

let drag = null;
ui.view.onpointerdown = (e) => { drag = [e.clientX, e.clientY, yaw, pitch]; ui.view.setPointerCapture(e.pointerId); };
ui.view.onpointermove = (e) => {
  if (!drag) return;
  yaw = drag[2] + (e.clientX - drag[0]) * 0.01;
  pitch = Math.max(-1.2, Math.min(1.2, drag[3] + (e.clientY - drag[1]) * 0.01));
};
ui.view.onpointerup = () => { drag = null; };

start().then(() => requestAnimationFrame((t) => { last = t; tick(t); })).catch((e) => fail(String(e)));
