// FPV video link (view/link.js): signal quality, break-ups, analog versus digital, and the picture delay.
import test from 'node:test';
import assert from 'node:assert/strict';
import { LINK, LINK_TYPES, panelsBetween, linkQuality, createLink, stepLink, createPoseDelay } from '../src/view/link.js';
import { buildCourse, insideTower } from '../src/course/geometry.js';
import { sphereBox } from '../src/physics/collide.js';
import { DEFAULT_LAYOUT, FIELD } from '../src/config/course.js';
import { createSim, step, DT } from '../src/sim.js';
import { createAutopilot, demoScript } from '../src/demo/autopilot.js';
import { mulberry32, qFromAxisAngle, qRot, qMul } from '../src/physics/math.js';

const course = buildCourse(DEFAULT_LAYOUT), tw = course.tower, boxes = course.boxes;
const PILOT = FIELD.pilotEye, UP = [0, 1, 0];
const near = (a, b, tol, msg) => assert.ok(Math.abs(a - b) <= tol, `${msg}: ${a} vs ${b}`);
const LOW = 0.3, HIGH = 0.9; // heights in the middle of the tower's lower and upper levels

// ---------------------------------------------------------------- what is in the way

test('the brief layout is the one these tests assume', () => {
  assert.equal(boxes.filter((b) => b.kind === 'panel').length, 6, 'three walls on each level');
  assert.equal(boxes.filter((b) => b.kind === 'roof').length, 1);
  assert.ok(Math.abs(tw.x) < 1e-9 && tw.yaw === 0 && PILOT[2] > tw.z + 3, 'tower square to the field, pilot in front of it');
});

test('panels between the drone and the pilot: none through the entrance, one or two behind the walls', () => {
  const n = (p, to = PILOT) => panelsBetween(p, to, boxes);
  assert.equal(n([0, 1, 1]), 0, 'open field');
  assert.equal(n([0, 1.5, tw.z]), 0, 'hovering over the roof');
  assert.equal(n([0, LOW, tw.z]), 0, 'lower level: the pilot sees in through the entrance');
  assert.equal(n([0, HIGH, tw.z]), 1, 'upper level: the upper front panel is in the way');
  assert.equal(n([0, LOW, tw.z - 0.65]), 1, 'behind the tower, low: back panel, then out through the entrance');
  assert.equal(n([0, HIGH, tw.z - 0.65]), 2, 'behind the tower, high: back and front panels');
  assert.equal(n([0, HIGH, tw.z], [0, 2, tw.z]), 1, 'straight up from inside: the roof');
  // each side of each level, from the centre outwards
  const out = (y, dx, dz) => n([0, y, tw.z], [dx * 1.5, y, tw.z + dz * 1.5]);
  assert.deepEqual([out(LOW, 0, 1), out(LOW, 0, -1), out(LOW, -1, 0), out(LOW, 1, 0)], [0, 1, 1, 1], 'lower level: only the front is open');
  assert.deepEqual([out(HIGH, 0, 1), out(HIGH, 0, -1), out(HIGH, -1, 0), out(HIGH, 1, 0)], [1, 1, 1, 0], 'upper level: only the right is open');
  assert.equal(n([-1.5, LOW, tw.z], [1.5, LOW, tw.z]), 2, 'right through the lower level');
  assert.equal(n([-1.5, HIGH, tw.z], [1.5, HIGH, tw.z]), 1, 'through the upper level and out of the exit');
});

test('panelsBetween: direction does not matter, a line that stops short counts nothing, other boxes are ignored', () => {
  const a = [0, HIGH, tw.z - 0.65];
  assert.equal(panelsBetween(a, PILOT, boxes), panelsBetween(PILOT, a, boxes));
  const back = tw.z - tw.half;
  assert.equal(panelsBetween([0, LOW, back - 0.5], [0, LOW, back - 0.01], boxes), 0, 'stops 1 cm short of the back panel');
  assert.equal(panelsBetween([0, LOW, back - 0.5], [0, LOW, back + 0.01], boxes), 1, 'reaches 1 cm past it');
  assert.equal(panelsBetween([0, LOW, 1], [0, LOW, 1], boxes), 0, 'a zero-length line in the open');
  const crate = { c: [0, 0.5, 2], h: [0.3, 0.5, 0.05], yaw: 0, kind: 'crate' };
  assert.equal(panelsBetween([0, 0.5, 1], [0, 0.5, 3], [crate]), 0, 'only tower panels and the roof block the signal');
  assert.equal(panelsBetween([0, 0.5, 1], [0, 0.5, 3], [{ ...crate, kind: 'panel' }]), 1);
  assert.equal(panelsBetween([0, 0.5, 1], [0, 0.5, 3], []), 0);
});

test('panelsBetween agrees with stepping along the line half a millimetre at a time, tower straight or turned', () => {
  const turned = buildCourse({ ...DEFAULT_LAYOUT, items: { ...DEFAULT_LAYOUT.items, tower: { ...DEFAULT_LAYOUT.items.tower, x: 0.4, rot: 30 } } });
  assert.ok(Math.abs(turned.tower.yaw) > 0.5, 'the second tower really is turned');
  for (const co of [course, turned]) {
    const rng = mulberry32(co === course ? 11 : 12), t = co.tower;
    const pick = () => [t.x + (rng() - 0.5) * 2, 0.02 + rng() * 1.6, t.z + (rng() - 0.5) * 2];
    const panels = co.boxes.filter((b) => b.kind === 'panel' || b.kind === 'roof');
    let crossings = 0;
    for (let k = 0; k < 120; k++) {
      const a = pick(), b = pick(), L = Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]), steps = Math.ceil(L / 0.0005);
      let slow = 0;
      for (const box of panels) {
        const solid = { c: box.c, h: box.h, yaw: box.yaw };
        for (let i = 0; i <= steps; i++) {
          const f = i / steps, p = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f];
          if (sphereBox(p, 1e-7, solid)) { slow++; break; }
        }
      }
      assert.equal(panelsBetween(a, b, co.boxes), slow, `line ${k}: ${a.map((v) => v.toFixed(3))} to ${b.map((v) => v.toFixed(3))}`);
      crossings += slow;
    }
    assert.ok(crossings > 60, `the random lines do cross panels (${crossings} crossings)`);
  }
});

// ---------------------------------------------------------------- signal quality

test('signal quality: near perfect in the open, each panel costs the same, never outside 0 to 1', () => {
  assert.ok(linkQuality([0, 1, 0], UP, PILOT, boxes) > 0.98, 'mid-field');
  const home = linkQuality([0, 0.13, 2.62], UP, PILOT, boxes);
  assert.ok(home > 0.95 && home < 1, `sitting on Home Base, almost under the pilot's nose (${home.toFixed(3)})`);
  for (const p of [[0, HIGH, tw.z], [0, LOW, tw.z - 0.65], [0, HIGH, tw.z - 0.65]]) {
    const n = panelsBetween(p, PILOT, boxes);
    near(linkQuality(p, UP, PILOT, []) - linkQuality(p, UP, PILOT, boxes), LINK.wallLoss * n, 1e-12, `${n} panel(s) in the way`);
  }
  const lower = linkQuality([0, LOW, tw.z], UP, PILOT, boxes), upper = linkQuality([0, HIGH, tw.z], UP, PILOT, boxes);
  const behind = linkQuality([0, HIGH, tw.z - 0.65], UP, PILOT, boxes);
  assert.ok(lower > upper && upper > behind && behind > 0.7, `lower ${lower.toFixed(2)} > upper ${upper.toFixed(2)} > behind ${behind.toFixed(2)}`);
  const rng = mulberry32(3);
  for (let k = 0; k < 500; k++) {
    const p = [(rng() - 0.5) * 200, rng() * 5, (rng() - 0.5) * 200];
    const q = linkQuality(p, qRot(qFromAxisAngle([rng() - 0.5, rng() - 0.5, rng() - 0.5], rng() * 6), UP), PILOT, boxes);
    assert.ok(q >= 0 && q <= 1 && Number.isFinite(q));
  }
  const at = linkQuality(PILOT, UP, PILOT, boxes);
  assert.ok(Number.isFinite(at) && at > 0.99, 'no trouble when the drone is exactly at the receiver');
});

test('signal quality: falls with distance and is gone at the range limit', () => {
  const far = (d) => linkQuality([PILOT[0], PILOT[1], PILOT[2] - d], UP, PILOT, []);
  near(far(LINK.range / 2), 0.75, 1e-9, 'half range');
  assert.equal(far(LINK.range), 0);
  assert.equal(far(LINK.range * 3), 0);
  assert.ok(far(1) > far(5) && far(5) > far(20));
  assert.ok(far(6.5) > 0.985, 'the far end of the 20 ft field costs almost nothing');
});

test('signal quality: the blind spot above and below the drone', () => {
  const p = [PILOT[0], 0.5, PILOT[2]]; // straight below the receiver
  near(linkQuality(p, UP, PILOT, []), 1 - LINK.nullLoss, 1e-3, 'level, receiver straight overhead');
  near(linkQuality(p, [0, -1, 0], PILOT, []), 1 - LINK.nullLoss, 1e-3, 'upside down is the same');
  assert.ok(linkQuality(p, [1, 0, 0], PILOT, []) > 0.999, 'on its side: the receiver is square on to the antenna');
  // level and far away: the receiver is near the horizon, so no loss
  assert.ok(linkQuality([0, 1.4, -3], UP, PILOT, []) > 0.98);
  // banked so the top of the drone points at the pilot
  const at = [0, 1.6, 0], toPilot = [0, 0, 1];
  near(linkQuality(at, toPilot, PILOT, []), 1 - LINK.nullLoss, 0.01, 'top pointing at the pilot');
  const tilt = (deg) => linkQuality(at, qRot(qFromAxisAngle([1, 0, 0], (deg * Math.PI) / 180), UP), PILOT, []);
  assert.ok(tilt(0) > tilt(30) && tilt(30) > tilt(60) && tilt(60) > tilt(90), 'the more the top tips towards the pilot, the worse');
  assert.ok(tilt(30) > 0.99, 'a normal 30 degree bank costs under 1 %');
});

// ---------------------------------------------------------------- break-ups

// Run a link for `secs` at `hz` and report what happened.
function watch({ type = 'analog', quality = 1, enclosed = false, secs = 600, hz = 60, seed = 1 } = {}) {
  const link = createLink(mulberry32(seed)), dt = 1 / hz, out = { glitches: 0, torn: 0, frozen: 0, noiseMin: 1, noiseMax: 0, delayMin: 9, delayMax: 0, runs: [], strengths: [] };
  let run = 0;
  for (let i = 0; i < secs * hz; i++) {
    stepLink(link, dt, { quality: typeof quality === 'function' ? quality(i * dt) : quality, enclosed, type });
    for (const k of ['q', 'noise', 'tear', 'delay']) assert.ok(Number.isFinite(link[k]) && link[k] >= 0 && link[k] <= 1, `${k} in range`);
    if (link.tear > 0) { out.torn++; run += dt; if (run === dt) out.strengths.push(link.tear); } else if (run) { out.runs.push(run); run = 0; }
    if (link.frozen) out.frozen++;
    out.noiseMin = Math.min(out.noiseMin, link.noise); out.noiseMax = Math.max(out.noiseMax, link.noise);
    out.delayMin = Math.min(out.delayMin, link.delay); out.delayMax = Math.max(out.delayMax, link.delay);
  }
  out.glitches = link.glitches; out.link = link; out.tornShare = out.torn / (secs * hz); out.frozenShare = out.frozen / (secs * hz);
  return out;
}
const G = LINK.glitch, meanLen = (G.min + G.max) / 2;
// Break-ups per second expected for a given start rate (none can start while one is running).
const expected = (rate) => rate / (1 + rate * meanLen);

test('link types', () => assert.deepEqual(LINK_TYPES, ['analog', 'digital']));

test('break-ups are rare in the open, common in the tower and commoner still on a weak signal', () => {
  const open = watch(), tower = watch({ enclosed: true }), weak = watch({ quality: 0.5 }), both = watch({ quality: 0.5, enclosed: true });
  const rate = (w) => w.glitches / 600;
  const check = (w, r, name) => assert.ok(Math.abs(rate(w) - expected(r)) < expected(r) * 0.35 + 0.01, `${name}: ${rate(w).toFixed(3)} per second, expected about ${expected(r).toFixed(3)}`);
  check(open, G.open, 'open field'); check(tower, G.open + G.tower, 'inside the tower');
  check(weak, G.open + 0.5 * G.weak, 'half signal'); check(both, G.open + G.tower + 0.5 * G.weak, 'half signal inside the tower');
  assert.ok(rate(open) < rate(tower) && rate(tower) < rate(weak) && rate(weak) < rate(both));
  assert.ok(open.tornShare < 0.02, `in the open the picture is clean over 98 % of the time (${(100 * open.tornShare).toFixed(1)} % broken)`);
  assert.ok(tower.tornShare > 0.03 && tower.tornShare < 0.15, `in the tower ${(100 * tower.tornShare).toFixed(1)} % broken`);
});

test('each break-up lasts between the shortest and longest set length, with a strength from 0.35 to 1', () => {
  for (const hz of [60, 144]) {
    const w = watch({ enclosed: true, hz, seed: 7 }), dt = 1 / hz;
    assert.ok(w.runs.length > 100);
    assert.ok(Math.min(...w.runs) >= G.min - dt - 1e-9 && Math.max(...w.runs) <= G.max + dt + 1e-9, `${hz} Hz: ${Math.min(...w.runs).toFixed(3)} to ${Math.max(...w.runs).toFixed(3)} s`);
    const mean = w.runs.reduce((a, b) => a + b, 0) / w.runs.length;
    near(mean, meanLen, 0.03, `${hz} Hz mean length`);
    assert.ok(Math.min(...w.strengths) >= 0.35 && Math.max(...w.strengths) <= 1);
    assert.ok(Math.max(...w.strengths) > 0.9 && Math.min(...w.strengths) < 0.45, 'weak and strong ones both happen');
  }
});

test('the break-up rate does not depend on the frame rate', () => {
  const at = (hz) => watch({ enclosed: true, hz, seed: 21, secs: 900 }).glitches / 900;
  const r30 = at(30), r60 = at(60), r144 = at(144), want = expected(G.open + G.tower);
  for (const r of [r30, r60, r144]) assert.ok(Math.abs(r - want) < want * 0.2, `${r.toFixed(3)} per second against ${want.toFixed(3)}`);
});

test('the same seed gives the same break-ups, a different seed gives others, and no time passing changes nothing', () => {
  const trace = (seed) => { const l = createLink(mulberry32(seed)), t = []; for (let i = 0; i < 6000; i++) { stepLink(l, 1 / 60, { enclosed: true }); t.push(l.tear); } return t; };
  assert.deepEqual(trace(5), trace(5));
  assert.notDeepEqual(trace(5), trace(6));
  const l = createLink(mulberry32(9));
  for (let i = 0; i < 300; i++) stepLink(l, 1 / 60, { quality: 0.6, enclosed: true });
  const before = { ...l, rng: null };
  for (let i = 0; i < 1000; i++) stepLink(l, 0, { quality: 0.6, enclosed: true });
  assert.deepEqual({ ...l, rng: null }, before, 'a paused link stays exactly as it was');
  assert.ok(Number.isFinite(createLink().q) && stepLink(createLink(), 1 / 60).q === 1, 'works with its defaults');
});

test('the picture follows a change in signal smoothly, at the same pace at any frame rate', () => {
  for (const hz of [20, 60, 240]) {
    const l = createLink(mulberry32(1));
    for (let i = 0; i < Math.round(LINK.smooth * hz); i++) stepLink(l, 1 / hz, { quality: 0 });
    near(l.q, Math.exp(-1), 0.02, `${hz} Hz: after one time constant`);
    for (let i = 0; i < 5 * hz; i++) stepLink(l, 1 / hz, { quality: 0 });
    assert.ok(l.q < 1e-6);
    for (let i = 0; i < 5 * hz; i++) stepLink(l, 1 / hz, { quality: 1 });
    assert.ok(l.q > 1 - 1e-6);
  }
  const l = createLink(mulberry32(1));
  stepLink(l, 1 / 60, { quality: 7 }); assert.equal(l.target, 1);
  stepLink(l, 1 / 60, { quality: -3 }); assert.equal(l.target, 0);
});

// ---------------------------------------------------------------- analog versus digital

test('analog: a fixed short delay, always a little grain, more static as the signal weakens, never a frozen picture', () => {
  const good = watch({ type: 'analog', secs: 120 }), poor = watch({ type: 'analog', quality: 0.5, secs: 120 }), dead = watch({ type: 'analog', quality: 0, secs: 120 });
  for (const w of [good, poor, dead]) {
    assert.equal(w.frozen, 0, 'analog video never freezes');
    assert.equal(w.delayMin, LINK.analog.delay); assert.equal(w.delayMax, LINK.analog.delay);
    assert.ok(w.noiseMin >= LINK.analog.grain - 1e-12, 'always some grain');
  }
  near(good.noiseMin, LINK.analog.grain, 1e-9, 'clean signal: only the grain');
  const settled = (quality) => { const l = createLink(() => 1); for (let i = 0; i < 600; i++) stepLink(l, 1 / 60, { quality }); return l.noise; };
  assert.ok(settled(1) < settled(0.8) && settled(0.8) < settled(0.5) && settled(0.5) < settled(0.2), 'weaker signal, more static');
  assert.ok(settled(0.2) > 0.5 && settled(0.9) < 0.15);
  assert.ok(watch({ type: 'analog', enclosed: true, secs: 120 }).noiseMax > LINK.analog.grain + 0.15, 'a break-up adds a burst of static');
  assert.ok(LINK.analog.delay >= 0.015 && LINK.analog.delay <= 0.035, 'analog delay in the usual 15 to 35 ms band');
});

test('digital: clean and steady on a good signal, then blocky and slower, then frozen', () => {
  const quiet = (quality, type = 'digital') => { const l = createLink(() => 1); for (let i = 0; i < 600; i++) stepLink(l, 1 / 60, { quality, type }); return l; };
  const D = LINK.digital;
  const full = quiet(1), edge = quiet(D.clean), half = quiet(0.5), gone = quiet(0.05);
  assert.equal(full.noise, 0); assert.equal(full.frozen, false); near(full.delay, D.delay, 1e-12, 'full signal');
  assert.ok(edge.noise < 1e-6 && !edge.frozen, 'still clean at the edge of the clean band');
  assert.ok(half.noise > 0.3 && !half.frozen, `blocky at half signal (${half.noise.toFixed(2)})`);
  assert.ok(half.delay > D.delay + 0.005 && half.delay <= D.delay + D.weakDelay, 'and slower');
  assert.ok(gone.frozen && gone.delay > half.delay, 'no usable signal: frozen');
  assert.ok(D.delay > LINK.analog.delay, 'digital is slower than analog to begin with');
  assert.ok(quiet(1, 'analog').noise > full.noise, 'on a good signal digital is the cleaner picture');
  // break-ups: some freeze the picture, weaker ones only smear it
  const w = watch({ type: 'digital', enclosed: true, seed: 4 });
  assert.ok(w.frozen > 0 && w.frozen < w.torn, `frozen ${w.frozen} frames of ${w.torn} broken ones`);
  assert.ok(w.frozenShare < 0.08, `in the tower the picture is frozen ${(100 * w.frozenShare).toFixed(1)} % of the time`);
  const l = createLink(mulberry32(4)); let sawBlocky = false, sawFrozen = false;
  for (let i = 0; i < 60 * 600; i++) {
    stepLink(l, 1 / 60, { type: 'digital', enclosed: true });
    assert.equal(l.frozen, l.tear >= D.freezeAt, 'on a good signal only a strong break-up freezes');
    if (l.tear > 0 && !l.frozen) { sawBlocky = true; assert.ok(l.noise > 0.2); }
    if (l.frozen) sawFrozen = true;
    if (l.tear === 0) assert.equal(l.noise, 0);
  }
  assert.ok(sawBlocky && sawFrozen);
  assert.equal(watch({ type: 'digital', secs: 60 }).delayMin, D.delay);
});

test('an unknown link type behaves as analog', () => {
  const a = createLink(mulberry32(2)), b = createLink(mulberry32(2));
  for (let i = 0; i < 2000; i++) { stepLink(a, 1 / 60, { quality: 0.7, type: 'analog' }); stepLink(b, 1 / 60, { quality: 0.7, type: 'smoke signals' }); }
  assert.deepEqual({ ...a, rng: null }, { ...b, rng: null });
});

// ---------------------------------------------------------------- picture delay

test('pose delay: empty, one pose, and exact playback of steady motion', () => {
  const d = createPoseDelay();
  assert.equal(d.sample(1), null); assert.equal(d.length, 0);
  d.push(10, [1, 2, 3], [0, 0, 0, 1]);
  for (const t of [0, 10, 99]) assert.deepEqual(d.sample(t), { p: [1, 2, 3], q: [0, 0, 0, 1] });
  d.clear(); assert.equal(d.sample(10), null);
  const at = (t) => [t, 2 * t, -t];
  for (let i = 0; i <= 60; i++) d.push(5 + i / 60, at(5 + i / 60), [0, 0, 0, 1]);
  for (const delay of [LINK.analog.delay, LINK.digital.delay, LINK.digital.delay + LINK.digital.weakDelay, 0.0001, 0.5]) {
    const t = 6 - delay, s = d.sample(t);
    for (let k = 0; k < 3; k++) near(s.p[k], at(t)[k], 1e-9, `delay ${delay}`);
  }
  assert.deepEqual(d.sample(6).p, at(6), 'no delay: the newest pose');
  assert.deepEqual(d.sample(60).p, at(6), 'the future: still the newest pose');
  assert.deepEqual(d.sample(1).p, at(5), 'before the first pose: the oldest');
});

test('pose delay: uneven frames, a full buffer, repeated and backward time stamps', () => {
  const d = createPoseDelay(96), rng = mulberry32(8);
  let t = 0; const times = [];
  for (let i = 0; i < 500; i++) { t += 0.004 + rng() * 0.03; times.push(t); d.push(t, [3 * t, 0, t * t], [0, 0, 0, 1]); }
  assert.equal(d.length, 96, 'keeps only the newest 96');
  for (const back of [0.005, 0.022, 0.042, 0.072, 0.3]) {
    const s = d.sample(t - back);
    near(s.p[0], 3 * (t - back), 1e-9, `${back} s back`);
    near(s.p[2], (t - back) ** 2, 0.0004, `${back} s back (curved path, straight-line blend between frames)`);
  }
  near(d.sample(0).p[0], 3 * times[500 - 96], 1e-12, 'older than the buffer: the oldest pose kept');
  // the same time stamp again replaces the newest pose and does not use up a slot
  const d2 = createPoseDelay(8);
  d2.push(1, [1, 0, 0], [0, 0, 0, 1]); d2.push(1, [2, 0, 0], [0, 0, 0, 1]);
  assert.equal(d2.length, 1); assert.equal(d2.sample(1).p[0], 2);
  d2.push(2, [4, 0, 0], [0, 0, 0, 1]); near(d2.sample(1.5).p[0], 3, 1e-12, 'blend');
  // time going backwards (a new page clock) starts the history again
  d2.push(0.5, [9, 0, 0], [0, 0, 0, 1]);
  assert.equal(d2.length, 1); assert.equal(d2.sample(2).p[0], 9); assert.equal(d2.sample(0).p[0], 9);
  // the pose handed back is a copy: changing it does not change the history
  const s = d2.sample(0.5); s.p[0] = -100; s.q[3] = 55;
  assert.equal(d2.sample(0.5).p[0], 9); assert.equal(d2.sample(0.5).q[3], 1);
});

test('pose delay: rotations blend the short way round and stay unit length', () => {
  const d = createPoseDelay(), q0 = [0, 0, 0, 1], q90 = qFromAxisAngle([0, 1, 0], Math.PI / 2);
  d.push(0, [0, 0, 0], q0); d.push(1, [0, 0, 0], q90);
  const mid = d.sample(0.5).q.slice(), want = qFromAxisAngle([0, 1, 0], Math.PI / 4);
  for (let k = 0; k < 4; k++) near(mid[k], want[k], 1e-9, 'half way is 45 degrees');
  for (const f of [0.1, 0.37, 0.8]) near(Math.hypot(...d.sample(f).q), 1, 1e-12, 'unit length');
  // the same rotation written with the opposite sign must not send the camera the long way round
  const e = createPoseDelay(), flipped = q90.map((v) => -v);
  e.push(0, [0, 0, 0], q90); e.push(1, [0, 0, 0], flipped);
  for (const f of [0.25, 0.5, 0.75]) {
    const fwd = qRot(e.sample(f).q, [0, 0, -1]), ref = qRot(q90, [0, 0, -1]);
    for (let k = 0; k < 3; k++) near(fwd[k], ref[k], 1e-9, 'same view all the way');
  }
  // a fast spin: the delayed view lags by exactly the delay
  const g = createPoseDelay(), rate = 6; // rad/s yaw
  for (let i = 0; i <= 120; i++) g.push(i / 120, [0, 0, 0], qFromAxisAngle([0, 1, 0], (rate * i) / 120));
  const fwd = qRot(g.sample(1 - 0.042).q, [0, 0, -1]), ref = qRot(qFromAxisAngle([0, 1, 0], rate * (1 - 0.042)), [0, 0, -1]);
  for (let k = 0; k < 3; k++) near(fwd[k], ref[k], 1e-4, 'spinning');
  assert.ok(Math.abs(qMul(q0, q90)[3] - q90[3]) < 1e-12);
});

// ---------------------------------------------------------------- over a whole flight

test('over the whole demo flight the picture stays flyable on both systems, and is worst behind and inside the tower', () => {
  const sim = createSim(DEFAULT_LAYOUT), ap = createAutopilot(sim, demoScript(sim));
  const links = { analog: createLink(mulberry32(31)), digital: createLink(mulberry32(31)) };
  const stat = { analog: { n: 0, torn: 0, frozen: 0, noise: 0 }, digital: { n: 0, torn: 0, frozen: 0, noise: 0 } };
  let qMin = 1, qSum = 0, worstAt = null, inTower = 0, inTowerQ = 0, outQ = 0, i = 0;
  const every = Math.round(1 / 60 / DT);
  while (!ap.done && sim.t < 590) {
    step(sim, ap.sticks(DT));
    if (++i % every) continue;
    const s = sim.quad, inside = insideTower(sim.course.tower, s.p);
    const q = linkQuality(s.p, qRot(s.q, UP), PILOT, sim.course.boxes);
    if (q < qMin) { qMin = q; worstAt = s.p.slice(); }
    qSum += q; if (inside) { inTower++; inTowerQ += q; } else outQ += q;
    for (const type of LINK_TYPES) {
      const l = stepLink(links[type], every * DT, { quality: q, enclosed: inside, type }), st = stat[type];
      st.n++; st.noise += l.noise; if (l.tear > 0) st.torn++; if (l.frozen) st.frozen++;
    }
  }
  const n = stat.analog.n;
  assert.ok(ap.done && !sim.crashed, 'the demo finished');
  assert.ok(qSum / n > 0.95, `average signal ${(100 * qSum / n).toFixed(1)} %`);
  assert.ok(qMin > 0.6, `worst signal ${(100 * qMin).toFixed(0)} % at ${worstAt.map((v) => v.toFixed(2))}`);
  // The weakest picture of the run is at the beacon, which sits behind the tower as the pilot sees it.
  assert.ok(panelsBetween(worstAt, PILOT, sim.course.boxes) >= 1, 'at the worst spot the tower is between the drone and the pilot');
  assert.ok(inTower > 300 && inTowerQ / inTower < outQ / (n - inTower), 'signal is lower inside the tower than outside');
  assert.ok(stat.analog.torn / n < 0.06, `analog broken up ${(100 * stat.analog.torn / n).toFixed(1)} % of the flight`);
  assert.equal(stat.analog.frozen, 0);
  assert.ok(stat.analog.noise / n < 0.12, `analog static averages ${(stat.analog.noise / n).toFixed(3)}`);
  assert.ok(stat.digital.frozen / n < 0.03, `digital frozen ${(100 * stat.digital.frozen / n).toFixed(1)} % of the flight`);
  assert.ok(stat.digital.noise / n < 0.05, `digital blockiness averages ${(stat.digital.noise / n).toFixed(3)}`);
  assert.ok(links.analog.glitches > 3, 'and yet there were some break-ups to fly through');
});
