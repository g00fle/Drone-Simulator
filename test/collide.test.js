// Nothing solid may pass through anything solid: throw the drone at the course and watch every collision sphere.
import test from 'node:test';
import assert from 'node:assert/strict';
import { createSim, step, DT } from '../src/sim.js';
import { DEFAULT_LAYOUT } from '../src/config/course.js';
import { DRONE } from '../src/config/drone.js';
import { sphereFloor, sphereCapsule, sphereBox, sphereCylinder, near } from '../src/physics/collide.js';
import { qRot, add, qFromAxisAngle } from '../src/physics/math.js';

const idle = { throttle: 0, pitch: 0, roll: 0, yaw: 0 };

// Deepest any collision sphere sits inside the floor, a pipe, a panel or the beacon, as a fraction of
// what it would take for that sphere's centre to reach the surface (1 = the centre is at the surface).
function deepest(sim) {
  const s = sim.quad, all = sim.course, R = DRONE.boundRadius; let worst = 0;
  const co = { capsules: all.capsules.filter((p) => near(s.p, R, p)), boxes: all.boxes.filter((p) => near(s.p, R, p)), cylinders: all.cylinders.filter((p) => near(s.p, R, p)) };
  for (const sp of DRONE.spheres) {
    const c = add(s.p, qRot(s.q, sp.p));
    const see = (hit) => { if (hit) worst = Math.max(worst, hit.d / sp.r); };
    see(sphereFloor(c, sp.r));
    for (const p of co.capsules) see(sphereCapsule(c, sp.r, p));
    for (const p of co.boxes) see(sphereBox(c, sp.r, p));
    for (const p of co.cylinders) see(sphereCylinder(c, sp.r, p));
  }
  return worst;
}

function launch(p, v, { w = [0, 0, 0], yaw = 0, secs = 1.2, sticks = idle } = {}) {
  const sim = createSim(DEFAULT_LAYOUT);
  const q = sim.quad; q.p = p.slice(); q.v = v.slice(); q.w = w.slice(); q.q = qFromAxisAngle([0, 1, 0], yaw); sim.prev = p.slice();
  let worst = 0;
  for (let i = 0; i < secs / DT; i++) { step(sim, sticks); worst = Math.max(worst, deepest(sim)); }
  return { sim, worst };
}

test('no part of the drone gets its centre into a pipe, panel or the floor at any speed up to 9 m/s', () => {
  const co = createSim(DEFAULT_LAYOUT).course, tw = co.tower, g = DEFAULT_LAYOUT.items.gate, c = DEFAULT_LAYOUT.items.corridor;
  let worst = 0, runs = 0;
  for (const speed of [1, 3, 6, 9]) {
    const shots = [
      [[tw.x - 1, 0.35, tw.z], [speed, 0, 0]], // tower left wall
      [[tw.x, 0.9, tw.z - 1], [0, 0, speed]], // tower back wall, upper
      [[tw.x + 0.1, tw.roofTop + 0.8, tw.z], [0, -speed, 0]], // down on to the roof
      [[tw.x - 0.3, 0.6, tw.z + 1], [0.2 * speed, 0, -speed]], // front corner post
      [[g.x, 0.77, g.z + 0.8], [0, 0, -speed]], // gate cross bars
      [[g.x + 0.3, 0.5, g.z + 0.8], [0, 0, -speed]], // gate upright
      [[g.x + 0.31, 1.2, g.z - 0.8], [0, -0.3 * speed, speed]],
      [[c.x, 0.36, c.z + 0.8], [0, 0, -speed]], // corridor middle bar
      [[c.x + 0.25, 0.7, c.z + 0.8], [0.1 * speed, 0, -speed]], // corridor upright
      [[0, 1.5, 0], [0, -speed, 0]], // straight into the floor
      [[0.5, 0.5, 1], [0.6 * speed, -0.8 * speed, 0]],
    ];
    for (const [p, v] of shots) for (const spin of [[0, 0, 0], [6, 2, -5]]) for (const yaw of [0, 0.6]) {
      const r = launch(p, v, { w: spin, yaw }); runs++;
      worst = Math.max(worst, r.worst);
      assert.ok(r.worst < 1, `sphere centre reached a surface: start ${p}, velocity ${v}, spin ${spin}, yaw ${yaw}, depth ${r.worst.toFixed(2)} radii`);
    }
  }
  assert.ok(runs > 100);
});

test('a fast hit on a tower wall stops outside it, and a dive stays above the floor', () => {
  const tw = createSim(DEFAULT_LAYOUT).course.tower;
  for (const speed of [3, 6, 9]) {
    const sim = createSim(DEFAULT_LAYOUT), q = sim.quad;
    q.p = [tw.x - 1, 0.35, tw.z]; q.v = [speed, 0, 0]; sim.prev = q.p.slice();
    let maxX = -Infinity, minY = Infinity;
    for (let i = 0; i < 1.5 / DT; i++) { step(sim, idle); maxX = Math.max(maxX, q.p[0]); minY = Math.min(minY, q.p[1]); }
    assert.ok(maxX < tw.x - 0.3048, `stayed outside the wall at ${speed} m/s (got to ${(maxX - tw.x).toFixed(3)} m from the tower centre)`);
    assert.ok(minY > 0, 'centre stayed above the floor');
  }
});

test('a medical pod dropped from 2 m lands on the tower roof and stays there; a ball thrown at a wall stays outside', () => {
  const sim = createSim(DEFAULT_LAYOUT), tw = sim.course.tower;
  const pod = sim.objects.find((o) => o.type === 'pod'), ball = sim.objects.find((o) => o.type === 'ball');
  pod.p = [tw.x, tw.roofTop + 2, tw.z]; pod.v = [0, 0, 0]; pod.asleep = false;
  ball.p = [tw.x - 0.8, 0.3, tw.z]; ball.v = [8, 0, 0]; ball.asleep = false;
  let ballMaxX = -Infinity;
  for (let i = 0; i < 2.5 / DT; i++) { step(sim, idle); ballMaxX = Math.max(ballMaxX, ball.p[0]); }
  assert.ok(Math.abs(pod.p[1] - tw.roofTop) < 0.01, `pod rests on the roof (y=${pod.p[1].toFixed(3)}, roof ${tw.roofTop.toFixed(3)})`);
  assert.ok(ballMaxX < tw.x - 0.3048, 'ball bounced off the wall');
});

// Fly with given sticks and report what happened after the first touch.
function fly(p, v, { yaw = 0, secs = 4, sticks = idle } = {}) {
  const sim = createSim(DEFAULT_LAYOUT), q = sim.quad;
  q.p = p.slice(); q.v = v.slice(); q.q = qFromAxisAngle([0, 1, 0], yaw); sim.prev = p.slice();
  let hit = false, vAfter = 0, upMin = 1, yMax = -Infinity;
  for (let i = 0; i < secs / DT; i++) {
    step(sim, sticks);
    hit ||= sim.hits > 0 || sim.inNet;
    if (hit) { vAfter = Math.max(vAfter, Math.hypot(...q.v)); upMin = Math.min(upMin, qRot(q.q, [0, 1, 0])[1]); yMax = Math.max(yMax, q.p[1]); }
  }
  return { sim, q, hit, vAfter, upMin, yMax, up: qRot(q.q, [0, 1, 0])[1], speed: Math.hypot(...q.v) };
}

test('hitting a pole with the sticks centred never makes the drone speed up or climb away', () => {
  const g = DEFAULT_LAYOUT.items.gate;
  for (const s of [2, 3, 4]) for (const yaw of [0, 1.2]) for (const off of [0.3, 0.36]) for (const y of [0.3, 0.77]) {
    const r = fly([g.x + off + Math.sin(yaw) * 0.9, y, g.z + Math.cos(yaw) * 0.9], [-Math.sin(yaw) * s, 0, -Math.cos(yaw) * s], { yaw, secs: 6 });
    const tag = `${s} m/s, yaw ${yaw}, offset ${off}, height ${y}`;
    assert.ok(r.hit, `${tag}: should hit the gate`);
    assert.ok(r.vAfter <= s * 1.15 + 0.3, `${tag}: reached ${r.vAfter.toFixed(1)} m/s after the hit`);
    assert.ok(r.yMax < y + 0.35, `${tag}: climbed to ${r.yMax.toFixed(2)} m`);
    if (!r.sim.crashed) assert.ok(r.up > 0.9 && r.speed < s, `${tag}: should settle level and slower (up ${r.up.toFixed(2)}, ${r.speed.toFixed(1)} m/s)`);
  }
});

test('the safety net catches a fast drone without a crash and holds a pushing drone level', () => {
  const B = createSim(DEFAULT_LAYOUT).course.bounds;
  const fast = fly([1.2, 1, 0], [6, 0, 0]);
  assert.ok(fast.hit && !fast.sim.crashed, 'a 6 m/s run into the side net is not a crash');
  assert.ok(fast.q.p[0] < B.x1 && fast.upMin > 0.95, 'pushed back inside, still level');
  assert.ok(fast.sim.rules.log.some((l) => /safety net/.test(l.text)), 'the log says what happened');
  for (const [name, p, v, sticks, ax, lim] of [
    ['side', [1.2, 1, 0], [3, 0, 0], { ...idle, roll: 1 }, 0, B.x1],
    ['far end', [0, 1, -2.6], [0, 0, -3], { ...idle, pitch: 1 }, 2, -B.z0],
    ['ceiling', [0, 2, 0], [0, 3, 0], { ...idle, throttle: 1 }, 1, B.y1],
  ]) {
    const r = fly(p, v, { sticks, secs: 6 });
    assert.ok(!r.sim.crashed, `${name}: no crash`);
    assert.ok(Math.abs(r.q.p[ax]) < lim + 0.05, `${name}: held at the net (${r.q.p[ax].toFixed(2)})`);
    assert.ok(r.speed < 0.2, `${name}: comes to rest against the net (${r.speed.toFixed(2)} m/s)`);
    if (ax !== 1) assert.ok(Math.abs(r.q.p[1] - 1) < 0.1, `${name}: keeps its height (${r.q.p[1].toFixed(2)})`);
    else assert.ok(r.up > 0.97, 'ceiling: stays level');
  }
});
