import test from 'node:test';
import assert from 'node:assert/strict';
import { createSim, step, DT, setYaw } from '../src/sim.js';
import { DEFAULT_LAYOUT, GATE } from '../src/config/course.js';
import { DRONE } from '../src/config/drone.js';
import { IN } from '../src/physics/math.js';

const idle = { throttle: 0, pitch: 0, roll: 0, yaw: 0 };
const run = (sim, secs, sticks = idle) => { for (let i = 0; i < secs / DT; i++) step(sim, typeof sticks === 'function' ? sticks(sim) : sticks); };
const avg = (a) => a.reduce((x, y) => x + y, 0) / a.length;

test('rests on its legs without drifting or sinking', () => {
  const sim = createSim(DEFAULT_LAYOUT);
  const start = sim.quad.p.slice();
  run(sim, 3);
  assert.ok(sim.grounded);
  assert.ok(Math.abs(sim.quad.p[0] - start[0]) < 0.005 && Math.abs(sim.quad.p[2] - start[2]) < 0.005, 'no drift');
  assert.ok(Math.abs(sim.quad.p[1] + DRONE.footZ) < 0.004, `feet on the floor, y=${sim.quad.p[1]}`);
});

test('hover throttle matches weight / max thrust', () => {
  const sim = createSim(DEFAULT_LAYOUT);
  run(sim, 2, { ...idle, throttle: 1 });
  run(sim, 3);
  assert.ok(sim.quad.p[1] > 1, 'climbed');
  assert.ok(Math.abs(sim.quad.v[1]) < 0.03, 'holding altitude');
  const expected = (DRONE.mass * 9.81) / (4 * DRONE.maxThrust * sim.quad.sag);
  assert.ok(Math.abs(avg(sim.quad.motors) - expected) < 0.01, `hover ${avg(sim.quad.motors)} vs ${expected}`);
});

test('angle mode: forward stick moves forward and levels out when released', () => {
  const sim = createSim(DEFAULT_LAYOUT);
  run(sim, 1.5, { ...idle, throttle: 1 });
  const z0 = sim.quad.p[2];
  run(sim, 1, { ...idle, pitch: 0.5 });
  assert.ok(sim.quad.p[2] < z0 - 0.3, 'moved away from the pilot');
  run(sim, 1, { ...idle, yaw: 1 });
  run(sim, 2);
  const up = 1 - 2 * (sim.quad.q[0] ** 2 + sim.quad.q[2] ** 2);
  assert.ok(up > 0.999, 'level again');
});

// Fly straight through the High Risk (12 in) slot of the gate at a given height offset from its centre.
function flySlot(offset) {
  const layout = { ...DEFAULT_LAYOUT, items: { ...DEFAULT_LAYOUT.items, gate: { x: 0, z: 1.0, rot: 0 } } };
  const sim = createSim(layout);
  const o = GATE.openings[2], mid = (o.y0 + o.y1) / 2 + offset;
  // body centre height so the drone (feet .. top) is centred on `mid`
  const y = mid - (DRONE.footZ + DRONE.topZ) / 2;
  sim.quad.p = [0, y, 1.6]; sim.prev = sim.quad.p.slice();
  let maxHits = 0;
  for (let i = 0; i < 4 / DT; i++) {
    const s = sim.quad;
    step(sim, { throttle: Math.max(-1, Math.min(1, (y - s.p[1]) * 6)), pitch: s.p[2] > 0.4 ? 0.12 : 0, roll: Math.max(-1, Math.min(1, -s.p[0] * 2)), yaw: 0 });
    if (Math.abs(s.p[2] - 1.0) < 0.25) maxHits = Math.max(maxHits, sim.hits);
  }
  return { sim, maxHits };
}

test('clears the 12 in High Risk slot when centred', () => {
  const { sim, maxHits } = flySlot(0);
  assert.equal(maxHits, 0, 'no contact');
  assert.ok(sim.quad.p[2] < 0.6, 'went through');
  assert.ok(12 * IN - 0.0213 - (DRONE.topZ - DRONE.footZ) > 0.07, 'about 74 mm total spare height');
});

test('hits the frame when 40 mm too high in the 12 in slot', () => {
  const { maxHits } = flySlot(0.04);
  assert.ok(maxHits > 0, 'contact with the pipe');
});
