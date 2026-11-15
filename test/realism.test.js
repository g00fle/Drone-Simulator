// Optional realism (physics/realism.js): ground and ceiling effect, prop wash, tower buffeting and the battery.
// Every part is checked switched on, switched off, and switched over in mid-flight.
import test from 'node:test';
import assert from 'node:assert/strict';
import { createSim, step, DT, resetDrone, Rules } from '../src/sim.js';
import { DEFAULT_LAYOUT } from '../src/config/course.js';
import { DRONE, PAYLOADS } from '../src/config/drone.js';
import { AERO, groundGain, ceilingGain, restVolts, packOhms, washGrip } from '../src/physics/realism.js';
import { qRot, qFromAxisAngle, clamp } from '../src/physics/math.js';
import { createAutopilot, demoScript } from '../src/demo/autopilot.js';

const idle = { throttle: 0, pitch: 0, roll: 0, yaw: 0 };
const ALL = { ground: true, wash: true, battery: true };
const HOVER = (DRONE.mass * 9.81) / (4 * DRONE.maxThrust);
const OPEN = [1.1, 0.6, -0.4]; // a clear patch of floor, a metre from everything
const avg = (a) => a.reduce((x, y) => x + y, 0) / a.length;

// A drone already hovering at p (yaw 0), with the chosen realism switches.
function hoverAt(p, realism = {}, opts = {}) {
  const sim = createSim(DEFAULT_LAYOUT, { realism, ...opts }), q = sim.quad;
  q.p = p.slice(); q.motors = [HOVER, HOVER, HOVER, HOVER]; sim.prev = p.slice();
  return sim;
}
// Sticks that hold the drone at a point (yaw 0), the way a steady pilot would.
const holdAt = (t) => (sim) => {
  const s = sim.quad;
  return {
    throttle: clamp((t[1] - s.p[1]) * 3, -1, 1), yaw: 0,
    pitch: clamp((s.p[2] - t[2]) * 1.2 + s.v[2] * 0.9, -1, 1), roll: clamp((t[0] - s.p[0]) * 1.2 - s.v[0] * 0.9, -1, 1),
  };
};
function run(sim, secs, sticks = idle, each) {
  for (let i = 0; i < Math.round(secs / DT); i++) { step(sim, typeof sticks === 'function' ? sticks(sim) : sticks); if (each) each(sim); }
}
const ball = (sim, i = 0) => sim.objects.filter((o) => o.type === 'ball')[i];
const place = (o, p) => { o.p = p.slice(); o.v = [0, 0, 0]; o.asleep = true; o.grounded = true; o.still = 0; return o; };
const moved = (o, from) => Math.hypot(o.p[0] - from[0], o.p[2] - from[2]);
const logged = (sim, re) => sim.rules.log.filter((l) => re.test(l.text)).length;

// ---------------------------------------------------------------- ground and ceiling effect

test('ground effect formula: strongest at landing height, gone by 2 m, never more than the cap', () => {
  const landed = groundGain(0.16), low = groundGain(0.3), mid = groundGain(0.6), high = groundGain(1.5);
  assert.ok(landed > 0.05 && landed < 0.12, `about 5 to 12 % extra lift sitting on the legs (${(landed * 100).toFixed(1)} %)`);
  assert.ok(low > 0.015 && low < landed && mid < low && high < mid, 'falls away with height');
  assert.ok(high < 0.003, `almost nothing at 1.5 m (${(high * 100).toFixed(2)} %)`);
  assert.equal(groundGain(2), 0); assert.equal(groundGain(5), 0);
  for (let z = 0; z < 2.2; z += 0.01) {
    const g = groundGain(z);
    assert.ok(g >= 0 && g <= AERO.ground.max && Number.isFinite(g), `in range at ${z.toFixed(2)} m`);
    assert.ok(g >= groundGain(z + 0.01) - 1e-12, `never rises with height (${z.toFixed(2)} m)`);
  }
  assert.ok(ceilingGain(0.03) > ceilingGain(0.08) && ceilingGain(0.08) > ceilingGain(0.3), 'ceiling effect grows as the props near the roof');
  assert.ok(ceilingGain(0) <= AERO.ceiling.max && ceilingGain(1) === 0 && ceilingGain(0.3) < 0.01);
});

test('ground effect off: every rotor gain stays exactly 1; on: a low hover needs less motor effort', () => {
  const effort = (y, realism) => {
    const sim = hoverAt([OPEN[0], y, OPEN[2]], realism); let n = 0, m = 0, g = 0;
    run(sim, 1.5); run(sim, 1, idle, () => { n++; m += avg(sim.quad.motors); g += avg(sim.quad.ge); });
    assert.ok(Math.abs(sim.quad.v[1]) < 0.02 && !sim.hits, 'steady hover, touching nothing');
    return { motors: m / n, ge: g / n };
  };
  const offLow = effort(0.26, {}), offHigh = effort(1.4, {});
  assert.equal(offLow.ge, 1); assert.equal(offHigh.ge, 1);
  assert.ok(Math.abs(offLow.motors / offHigh.motors - 1) < 0.003, 'switched off, height makes no difference');
  const onLow = effort(0.26, { ground: true }), onHigh = effort(1.4, { ground: true });
  assert.ok(onLow.ge > 1.02 && onLow.ge < 1.07, `3 to 6 % extra lift at a 10 cm skid height (${onLow.ge.toFixed(3)})`);
  assert.ok(onHigh.ge < 1.004, `under half a percent at 1.4 m (${onHigh.ge.toFixed(4)})`);
  const saving = 1 - onLow.motors / onHigh.motors;
  assert.ok(saving > 0.015 && saving < 0.07, `the low hover needs ${(saving * 100).toFixed(1)} % less motor effort`);
  assert.ok(Math.abs(onHigh.motors / offHigh.motors - 1) < 0.005, 'high up, on and off agree');
});

test('ground effect: the tower roof counts as ground, the roof edge splits the rotors, and the roof gives ceiling effect from below', () => {
  const tw = createSim(DEFAULT_LAYOUT).course.tower, y = tw.roofTop + 0.25;
  const over = hoverAt([tw.x, y, tw.z], { ground: true }); run(over, 1);
  assert.ok(Math.min(...over.quad.ge) > 1.025, `over the roof: ${over.quad.ge.map((g) => g.toFixed(3))}`);
  const beside = hoverAt([OPEN[0], y, OPEN[2]], { ground: true }); run(beside, 1);
  assert.ok(Math.max(...beside.quad.ge) < 1.004, `same height beside the tower: ${beside.quad.ge.map((g) => g.toFixed(4))}`);

  // Centred on the right-hand roof edge: the two left rotors are over the panel, the two right ones are not.
  const pe = [tw.x + tw.roofHalf + 0.03, y, tw.z], edge = hoverAt(pe, { ground: true }); let upMin = 1;
  run(edge, 2, holdAt(pe), () => { upMin = Math.min(upMin, qRot(edge.quad.q, [0, 1, 0])[1]); });
  const g = edge.quad.ge; // motor order: front-right, front-left, rear-left, rear-right
  assert.ok(g[1] > g[0] + 0.02 && g[2] > g[3] + 0.02, `left rotors lift more: ${g.map((v) => v.toFixed(3))}`);
  assert.ok(g[0] < 1.004 && g[3] < 1.004, 'the right rotors, off the edge, get nothing');
  assert.ok(upMin > 0.999 && !edge.crashed, `the flight controller keeps it level (${upMin.toFixed(5)})`);
  assert.ok(edge.quad.motors[1] < edge.quad.motors[0] - 0.002 && edge.quad.motors[2] < edge.quad.motors[3] - 0.002, 'by easing off the two motors over the roof');
  // Hands off, the uneven lift tips the drone gently away from the roof, as a real one does at a ledge.
  const loose = hoverAt(pe, { ground: true }), plain = hoverAt(pe, {}); run(loose, 2); run(plain, 2);
  const slid = loose.quad.p[0] - plain.quad.p[0];
  assert.ok(slid > 0.02 && slid < 0.15 && !loose.crashed, `hands off it slides ${(slid * 1000).toFixed(0)} mm off the edge in 2 s`);

  const p = [tw.x, tw.roofTop - 0.13, tw.z]; // upper level, props about 12 cm under the roof
  const under = hoverAt(p, { ground: true }); let yMin = p[1], yMax = p[1];
  run(under, 2, holdAt(p), () => { yMin = Math.min(yMin, under.quad.p[1]); yMax = Math.max(yMax, under.quad.p[1]); });
  assert.ok(Math.min(...under.quad.ge) > 1.015 && Math.max(...under.quad.ge) < 1.08, `under the roof: ${under.quad.ge.map((v) => v.toFixed(3))}`);
  assert.ok(yMax - yMin < 0.03 && !under.hits && !under.crashed, `altitude hold copes, no contact (moved ${((yMax - yMin) * 1000).toFixed(0)} mm)`);
  const off = hoverAt(p, {}); run(off, 0.5, holdAt(p));
  assert.deepEqual(off.quad.ge, [1, 1, 1, 1]);
});

test('ground effect: take-off, a slow landing and a knocked-over drone all behave', () => {
  const sim = createSim(DEFAULT_LAYOUT, { realism: { ground: true } }), start = sim.quad.p.slice();
  run(sim, 2);
  assert.ok(sim.grounded && Math.hypot(sim.quad.p[0] - start[0], sim.quad.p[2] - start[2]) < 0.005, 'sits still on its legs');
  assert.ok(Math.abs(sim.quad.p[1] + DRONE.footZ) < 0.004, 'feet on the floor');
  run(sim, 1.5, { ...idle, throttle: 1 });
  assert.ok(sim.quad.p[1] > 1, 'climbs away');
  let impact = 0; run(sim, 6, { ...idle, throttle: -0.5 }, () => { if (sim.grounded) impact = Math.max(impact, -sim.quad.v[1]); });
  assert.ok(sim.grounded && !sim.crashed, 'lands');
  assert.ok(Math.abs(sim.quad.p[1] + DRONE.footZ) < 0.006, `settles on the floor, no hovering on the cushion (y=${sim.quad.p[1].toFixed(3)})`);
  // upside down there is no cushion at all
  const flip = hoverAt([OPEN[0], 0.4, OPEN[2]], { ground: true });
  flip.quad.q = qFromAxisAngle([0, 0, 1], Math.PI); step(flip, idle);
  assert.deepEqual(flip.quad.ge, [1, 1, 1, 1]);
});

// ---------------------------------------------------------------- prop wash

test('prop wash off: a hover next to a foam ball leaves every loose object exactly where it was', () => {
  const sim = hoverAt(OPEN, {}), b = place(ball(sim), [OPEN[0] + 0.3, 0, OPEN[2]]);
  const before = sim.objects.map((o) => o.p.slice());
  run(sim, 3, holdAt(OPEN));
  sim.objects.forEach((o, i) => assert.deepEqual(o.p, before[i], `${o.id} did not move`));
  assert.ok(b.asleep && b.wash === null);
});

test('prop wash on: a hover blows a foam ball away along the floor, but a pod and the rescue figure stay put', () => {
  const sim = hoverAt(OPEN, { wash: true });
  const from = [OPEN[0] + 0.35, 0, OPEN[2]], b = place(ball(sim), from);
  const pod = place(sim.objects.find((o) => o.type === 'pod'), [OPEN[0], 0, OPEN[2] + 0.35]);
  const fig = place(sim.objects.find((o) => o.type === 'figure'), [OPEN[0] - 0.35, 0, OPEN[2]]);
  let vMax = 0; run(sim, 3, holdAt(OPEN), () => { vMax = Math.max(vMax, Math.hypot(b.v[0], b.v[2])); });
  assert.ok(b.p[0] - from[0] > 0.3, `ball rolled away from the drone (${(b.p[0] - from[0]).toFixed(2)} m)`);
  assert.ok(Math.abs(b.p[2] - from[2]) < 0.05, 'straight out, not sideways');
  assert.ok(vMax < 4, `at a believable speed (${vMax.toFixed(2)} m/s peak)`);
  assert.ok(Math.abs(b.p[1]) < 0.005, 'it stays on the floor');
  assert.ok(moved(pod, [OPEN[0], 0, OPEN[2] + 0.35]) < 0.001, 'the 225 g pod does not move');
  assert.ok(moved(fig, [OPEN[0] - 0.35, 0, OPEN[2]]) < 0.001, 'the 45 g figure does not move');
});

test('prop wash: the rescue figure and a pod hold still even under the lowest, closest hover', () => {
  for (const type of ['figure', 'pod']) for (const h of [0.2, 0.3, 0.5]) for (const r of [0.12, 0.17, 0.25, 0.4]) {
    const sim = hoverAt([OPEN[0], 0, OPEN[2]], { wash: true }), o = sim.objects.find((x) => x.type === type);
    place(o, [OPEN[0] + r, 0, OPEN[2]]);
    sim.quad.p[1] = h + (type === 'pod' ? 0.2 : 0.15); // clear of the object
    const p = sim.quad.p.slice(); run(sim, 1, holdAt(p));
    assert.ok(moved(o, [OPEN[0] + r, 0, OPEN[2]]) < 0.001, `${type} ${r} m out under a hover at ${sim.quad.p[1].toFixed(2)} m`);
  }
});

test('prop wash gets weaker with height and with distance, and needs the motors running', () => {
  const force = (y, r, prep) => {
    const sim = hoverAt([OPEN[0], y, OPEN[2]], { wash: true }), b = place(ball(sim), [OPEN[0] + r, 0, OPEN[2]]);
    if (prep) prep(sim);
    run(sim, 0.05);
    return b.wash ? Math.hypot(b.wash[0], b.wash[1]) : 0;
  };
  const near = force(0.4, 0.3), high = force(1.6, 0.3), far = force(0.4, 2.5);
  assert.ok(near > 2 * washGrip({ rollMu: 0.08 }) * PAYLOADS.ball.mass * 9.81, `plenty to move a ball close by (${near.toFixed(4)} N)`);
  assert.ok(high < near / 2, `much weaker from 1.6 m up (${high.toFixed(4)} N)`);
  assert.equal(far, 0, 'nothing 2.5 m away');
  assert.equal(force(0.4, 0.3, (sim) => { sim.fc.armed = false; sim.quad.motors = [0, 0, 0, 0]; }), 0, 'no wash with the motors stopped');
  assert.equal(force(0.4, 4.5), 0, 'nothing beyond its reach');
});

test('prop wash is stopped by the tower roof and by a wall panel', () => {
  const tw = createSim(DEFAULT_LAYOUT).course.tower;
  // Over the roof: a ball on the floor beside the tower is sheltered, a ball on the roof is not.
  const p = [tw.x + 0.1, tw.roofTop + 0.35, tw.z], sim = hoverAt(p, { wash: true });
  const floorBall = place(ball(sim, 0), [tw.x + 0.55, 0, tw.z]), roofBall = place(ball(sim, 1), [tw.x - 0.15, tw.roofTop, tw.z]);
  const pod = place(sim.objects.find((o) => o.type === 'pod'), [tw.x - 0.1, tw.roofTop, tw.z + 0.12]);
  run(sim, 2.5, holdAt(p));
  assert.ok(floorBall.asleep && moved(floorBall, [tw.x + 0.55, 0, tw.z]) === 0, 'ball on the floor is under the roof\'s shadow');
  assert.ok(moved(roofBall, [tw.x - 0.15, tw.roofTop, tw.z]) > 0.1, 'ball on the roof is blown along (and off) it');
  assert.ok(moved(pod, [tw.x - 0.1, tw.roofTop, tw.z + 0.12]) < 0.001 && Math.abs(pod.p[1] - tw.roofTop) < 0.005, 'a delivered pod stays on the roof');

  // Beside a wall: a ball inside the tower is sheltered by the panel. The same distance through the open entrance is not.
  const inside = [tw.x, 0, tw.z];
  const wall = [tw.x - 0.6, 0.5, tw.z], a = hoverAt(wall, { wash: true }), ba = place(ball(a), inside);
  run(a, 2, holdAt(wall));
  assert.ok(moved(ba, inside) === 0 && ba.asleep, 'the side panel blocks the wash');
  const door = [tw.x, 0.5, tw.z + 0.6], b = hoverAt(door, { wash: true }), bb = place(ball(b), inside);
  run(b, 2, holdAt(door));
  assert.ok(moved(bb, inside) > 0.03, `through the open entrance the ball is pushed (${moved(bb, inside).toFixed(2)} m)`);
});

test('prop wash can be switched on and off in flight; the same switch lets the foam balls roll freely', () => {
  const sim = hoverAt(OPEN, {}), from = [OPEN[0] + 0.3, 0, OPEN[2]], b = place(ball(sim), from);
  run(sim, 1, holdAt(OPEN)); assert.equal(moved(b, from), 0, 'off: nothing');
  sim.realism.wash = true; run(sim, 1, holdAt(OPEN));
  assert.ok(moved(b, from) > 0.05 && b.wash, 'on: the ball starts rolling');
  sim.realism.wash = false; run(sim, 0.01, holdAt(OPEN));
  assert.ok(sim.objects.every((o) => o.wash === null), 'off again: every wash force is cleared');
  run(sim, 4, holdAt(OPEN));
  assert.ok(b.asleep && Math.hypot(...b.v) === 0, 'and the ball coasts to a stop');
  // A ball shoved at 1 m/s travels further when it rolls (wash switch on) than when it is treated as sliding.
  const roll = (wash) => {
    const s = createSim(DEFAULT_LAYOUT, { realism: { wash } }), o = place(ball(s), [1.2, 0, 0.2]);
    o.asleep = false; o.v = [0, 0, -1]; run(s, 4);
    return moved(o, [1.2, 0, 0.2]);
  };
  const sliding = roll(false), rolling = roll(true);
  assert.ok(sliding > 0.1 && rolling > sliding * 1.5 && rolling < 1.2, `sliding ${sliding.toFixed(2)} m, rolling ${rolling.toFixed(2)} m`);
});

// ---------------------------------------------------------------- tower buffeting

test('tower buffeting: only inside the tower, only with prop wash on, small, and repeatable by seed', () => {
  const tw = createSim(DEFAULT_LAYOUT).course.tower, inside = [tw.x, 0.3, tw.z];
  // Hands off for 2 s. gust = the strongest gust seen, inTower = how far the buffeting was switched in.
  const wander = (p, realism, seed) => {
    const sim = hoverAt(p, realism, { seed }); let gust = 0, inTower = 0;
    run(sim, 2, idle, () => { for (const g of sim.fx.gust) gust = Math.max(gust, Math.abs(g)); inTower = Math.max(inTower, sim.fx.inTower); });
    return { sim, p: sim.quad.p, gust, inTower };
  };
  const apart = (a, b) => Math.hypot(a.p[0] - b.p[0], a.p[1] - b.p[1], a.p[2] - b.p[2]);
  const calm = wander(inside, {}, 1);
  assert.ok(calm.gust === 0 && calm.inTower === 0, 'wash off: no gusts inside the tower');
  assert.deepEqual(calm.p, wander(inside, {}, 99).p, 'and the seed changes nothing');
  const open = wander(OPEN, { wash: true }, 1), openOff = wander(OPEN, {}, 1);
  assert.ok(open.gust === 0 && open.inTower === 0, 'wash on, out in the open: no gusts');
  assert.deepEqual(open.p, openOff.p, 'so the flight is identical to wash off');
  const a = wander(inside, { wash: true }, 7), again = wander(inside, { wash: true }, 7), other = wander(inside, { wash: true }, 8);
  assert.ok(a.inTower > 0.99 && a.gust > 1, 'wash on inside: the gusts are running');
  assert.ok(apart(a, calm) > 0.01 && apart(a, calm) < 0.3, `they push a hands-off drone ${(apart(a, calm) * 1000).toFixed(0)} mm off its calm path in 2 s`);
  assert.ok(!a.sim.crashed && a.sim.hits === 0, 'without reaching a wall');
  assert.deepEqual(a.p, again.p, 'the same seed repeats exactly');
  assert.ok(apart(a, other) > 0.005, 'a different seed gives different gusts');
  // Long run with a pilot holding position: gusts stay bounded and the drone stays off the walls.
  const sim = hoverAt(inside, { wash: true }, { seed: 3 }); let gust = 0, far = 0, hits = 0;
  run(sim, 15, holdAt(inside), () => {
    for (const g of sim.fx.gust) gust = Math.max(gust, Math.abs(g));
    far = Math.max(far, Math.hypot(sim.quad.p[0] - inside[0], sim.quad.p[2] - inside[2])); hits += sim.hits;
  });
  assert.ok(gust < 5, `gusts stay within 5 standard deviations (${gust.toFixed(2)})`);
  assert.ok(AERO.buffet.side * 5 < 0.1, 'so the strongest shove is under a tenth of the thrust');
  assert.ok(gust > 2, 'and the long run really was gusty');
  assert.ok(far > 0.005 && far < 0.12 && hits === 0 && !sim.crashed, `held within ${(far * 1000).toFixed(0)} mm of the middle, never touching a wall`);
  // Leaving the tower, the buffeting dies away.
  sim.quad.p = OPEN.slice(); sim.quad.v = [0, 0, 0]; sim.prev = OPEN.slice(); run(sim, 2.5, holdAt(OPEN));
  assert.ok(sim.fx.inTower < 0.01, 'faded out a couple of seconds after leaving');
});

// ---------------------------------------------------------------- battery

test('battery tables: 16.8 V full, about 13 V empty, and a bigger pack has less resistance', () => {
  const b = DRONE.battery;
  assert.ok(Math.abs(restVolts(b, 1) - 16.8) < 1e-9 && Math.abs(restVolts(b, 0) - 13.08) < 1e-9);
  assert.equal(restVolts(b, 1.4), restVolts(b, 1)); assert.equal(restVolts(b, -0.2), restVolts(b, 0));
  for (let s = 0; s < 1; s += 0.01) assert.ok(restVolts(b, s + 0.01) >= restVolts(b, s), 'voltage never rises as the pack empties');
  assert.ok(Math.abs(packOhms(b, 1800) - 0.032) < 1e-9, `1800 mAh pack and wiring: ${packOhms(b, 1800)} ohm`);
  assert.ok(packOhms(b, 2200) < packOhms(b, 1800) && packOhms(b, 1800) < packOhms(b, 1500));
  assert.ok(b.packs.includes(b.packmAh));
});

test('battery off: the pack never runs down, whatever the flying', () => {
  const sim = hoverAt([OPEN[0], 1, OPEN[2]], {});
  run(sim, 20, (s) => ({ ...idle, throttle: Math.sin(s.t * 3) }));
  const q = sim.quad;
  assert.equal(q.usedAh, 0); assert.equal(q.usedWh, 0); assert.equal(q.soc, 1); assert.equal(q.power, 1); assert.equal(sim.battLevel, 0);
  assert.ok(q.sag > 0.91 && q.sag <= 1, `only the small fixed dip under load (${q.sag.toFixed(3)})`);
  assert.equal(logged(sim, /Battery/), 0);
});

test('battery on: a hover draws about 10 A, which is about 10 minutes on the 1800 mAh pack', () => {
  const sim = hoverAt([OPEN[0], 1, OPEN[2]], { battery: true }), q = sim.quad, b = DRONE.battery;
  run(sim, 5); const ah0 = q.usedAh, wh0 = q.usedWh;
  run(sim, 30);
  const amps = ((q.usedAh - ah0) * 3600) / 30, watts = ((q.usedWh - wh0) * 3600) / 30;
  assert.ok(Math.abs(watts - (b.hoverWatts + b.avionicsWatts)) < 4, `hover power ${watts.toFixed(1)} W`);
  assert.ok(amps > 9 && amps < 10.8, `hover current ${amps.toFixed(2)} A`);
  assert.ok(Math.abs(q.amps - amps) < 0.4 && Math.abs(q.watts - watts) < 4, 'the live readings agree with the totals');
  const minutes = (1.8 / amps) * 60;
  assert.ok(minutes > 9.5 && minutes < 12, `hover endurance ${minutes.toFixed(1)} min`);
  // (the charge reading is one 2 ms step behind the running total)
  assert.ok(Math.abs(q.soc - (1 - q.usedAh / 1.8)) < 1e-5 && q.soc < 0.96 && q.soc > 0.93, `charge left after 35 s: ${(q.soc * 100).toFixed(1)} %`);
  assert.ok(q.volts < restVolts(b, q.soc) - 0.2 && q.volts > 15.5, `voltage sits below resting under load (${q.volts.toFixed(2)} V)`);
  assert.ok(Math.abs(q.sag - (q.volts / 16.8) ** 2) < 1e-9 && q.sag > 0.85 && q.sag < 1, `thrust available ${(q.sag * 100).toFixed(0)} %`);
  const expected = HOVER / q.sag;
  assert.ok(Math.abs(avg(q.motors) - expected) < 0.01, 'the flight controller makes up for the sag');
  assert.equal(sim.battLevel, 0);
});

test('battery: carrying a pod and running the magnet cost power; a hard climb sags the voltage; a bigger pack lasts longer', () => {
  // Eight seconds held at a point, long enough for altitude hold to settle with whatever is being carried.
  const steady = (prep, mAh = 1800, y = 2) => {
    const p = [OPEN[0], y, OPEN[2]], sim = hoverAt(p, { battery: true, packmAh: mAh }); if (prep) prep(sim);
    run(sim, 8, holdAt(p));
    assert.ok(Math.abs(sim.quad.v[1]) < 0.02 && Math.abs(sim.quad.p[1] - y) < 0.03 && !sim.crashed, 'settled in a hover');
    return sim;
  };
  const bare = steady(), q = bare.quad, b = DRONE.battery;
  const pod = steady((s) => { s.quad.payload = { mass: PAYLOADS.pod.mass, r: [0, -0.1, 0] }; });
  const ratio = (pod.quad.watts - b.avionicsWatts) / (q.watts - b.avionicsWatts), ideal = ((DRONE.mass + PAYLOADS.pod.mass) / DRONE.mass) ** 1.5;
  assert.ok(Math.abs(ratio / ideal - 1) < 0.03, `a 225 g pod raises flying power by ${((ratio - 1) * 100).toFixed(0)} % (weight to the power 1.5)`);
  const podMinutes = (1.8 / pod.quad.amps) * 60;
  assert.ok(podMinutes > 6 && podMinutes < 9, `hovering with a pod would flatten the 1800 mAh pack in ${podMinutes.toFixed(1)} min`);
  const magnet = steady((s) => { s.magnetOn = true; });
  assert.ok(Math.abs(magnet.quad.watts - q.watts - b.magnetWatts) < 0.5, 'the magnet adds its 4 W');
  // A short full-throttle punch from a low hover.
  const punch = steady(null, 1800, 0.5), pq = punch.quad, vHover = pq.volts, sagHover = pq.sag;
  let vMin = 99, aMax = 0, sagMin = 1, yMax = 0, touched = false;
  const watch = () => {
    vMin = Math.min(vMin, pq.volts); aMax = Math.max(aMax, pq.amps); sagMin = Math.min(sagMin, pq.sag);
    yMax = Math.max(yMax, pq.p[1]); touched ||= punch.hits > 0 || punch.inNet;
  };
  punch.fc.altHold = false; run(punch, 0.15, { ...idle, throttle: 1 }, watch);
  assert.ok(pq.v[1] > 3, `it leaps upward (${pq.v[1].toFixed(1)} m/s after 0.15 s)`);
  assert.ok(aMax > 60 && aMax < 120, `full throttle draws ${aMax.toFixed(0)} A`);
  assert.ok(vMin < vHover - 1.5 && vMin > 12, `and pulls the pack down to ${vMin.toFixed(1)} V`);
  assert.ok(sagMin < sagHover - 0.15 && sagMin > 0.5, `leaving ${(sagMin * 100).toFixed(0)} % of full thrust`);
  punch.fc.altHold = true; run(punch, 4, holdAt([OPEN[0], 0.5, OPEN[2]]), watch);
  assert.ok(pq.volts > vHover - 0.1, `the voltage recovers once the load eases (${pq.volts.toFixed(2)} V)`);
  assert.ok(!touched && !punch.crashed && yMax < 3, `nothing was hit on the way (peak ${yMax.toFixed(2)} m)`);
  // pack size
  const small = steady(null, 1500), big = steady(null, 2200);
  assert.ok(small.quad.soc < q.soc - 0.002 && big.quad.soc > q.soc + 0.002, 'same flight, less of a bigger pack used');
  assert.ok(small.quad.volts < big.quad.volts, 'and the bigger pack sags less');
});

test('battery: low and critical warnings come once each, then an empty pack stops the motors', () => {
  const sim = hoverAt([OPEN[0], 0.8, OPEN[2]], { battery: true }), q = sim.quad, Ah = 1.8;
  q.usedAh = 0.74 * Ah; // skip the first seven minutes
  run(sim, 2); assert.equal(sim.battLevel, 0, '26 % left: no warning yet');
  let t = 0; while (sim.battLevel < 1 && t < 60) { run(sim, 0.5); t += 0.5; }
  assert.ok(sim.battLevel === 1 && Math.abs(q.soc - 0.25) < 0.01, `low at ${(q.soc * 100).toFixed(1)} %`);
  q.usedAh = 0.895 * Ah; t = 0; while (sim.battLevel < 2 && t < 60) { run(sim, 0.5); t += 0.5; }
  assert.ok(sim.battLevel === 2 && Math.abs(q.soc - 0.10) < 0.01, `critical at ${(q.soc * 100).toFixed(1)} %`);
  assert.ok(Math.abs(q.v[1]) < 0.05 && q.p[1] > 0.6, 'still able to hover at 10 %');
  assert.ok(q.sag > 0.6 && q.sag < 0.85, `with less thrust in hand (${(q.sag * 100).toFixed(0)} %)`);
  q.usedAh = 0.998 * Ah; t = 0; while (sim.battLevel < 3 && t < 30) { run(sim, 0.25); t += 0.25; }
  assert.equal(sim.battLevel, 3, 'empty');
  const tEmpty = sim.t; run(sim, DRONE.battery.fadeSeconds + 0.3);
  assert.equal(q.power, 0); assert.ok(Math.max(...q.motors) < 0.01, `motors stopped ${(sim.t - tEmpty).toFixed(1)} s after the pack ran out`);
  run(sim, 2);
  assert.ok(sim.onFloor && q.p[1] < 0.3, 'the drone has come down');
  assert.equal(logged(sim, /Battery low/), 1); assert.equal(logged(sim, /Battery critical/), 1); assert.equal(logged(sim, /Battery empty/), 1);
  // Full stick does nothing now, and Reset does not recharge. Only a new run brings a fresh pack.
  resetDrone(sim); run(sim, 2, { ...idle, throttle: 1 });
  assert.ok(sim.grounded && q.p[1] < 0.2 && q.soc === 0, 'a flat pack stays flat after Reset');
  assert.equal(createSim(DEFAULT_LAYOUT, { realism: { battery: true } }).quad.soc, 1);
});

test('battery can be switched off and on in flight without a jolt, and keeps its charge used so far', () => {
  const sim = hoverAt([OPEN[0], 1, OPEN[2]], { battery: true }), q = sim.quad;
  run(sim, 10); const used = q.usedAh, soc = q.soc; let yMin = q.p[1], yMax = q.p[1];
  const watch = () => { yMin = Math.min(yMin, q.p[1]); yMax = Math.max(yMax, q.p[1]); };
  sim.realism.battery = false; run(sim, 5, idle, watch);
  assert.ok(q.usedAh === used && q.soc === 1 && q.power === 1 && sim.battLevel === 0, 'off: nothing more is used');
  sim.realism.battery = true; run(sim, 5, idle, watch);
  assert.ok(q.usedAh > used && Math.abs(q.soc - soc) < 0.03 && q.soc < soc, 'on again: carries on from where it was');
  assert.ok(yMax - yMin < 0.05 && !sim.crashed, `height changed by ${((yMax - yMin) * 1000).toFixed(0)} mm across both switches`);
  // An empty pack that has shut the motors down comes back when the battery simulation is switched off.
  q.usedAh = 5; run(sim, 4);
  assert.equal(q.power, 0);
  sim.realism.battery = false; resetDrone(sim); run(sim, 2, { ...idle, throttle: 1 });
  assert.ok(q.p[1] > 0.8, 'flies again');
});

// ---------------------------------------------------------------- everything together

test('all realism on in mid-flight: no jolt when the switches flip', () => {
  const sim = hoverAt([OPEN[0], 0.4, OPEN[2]], {}), q = sim.quad; let yMin = 9, yMax = -9, upMin = 1;
  const watch = () => { yMin = Math.min(yMin, q.p[1]); yMax = Math.max(yMax, q.p[1]); upMin = Math.min(upMin, qRot(q.q, [0, 1, 0])[1]); };
  run(sim, 2, holdAt([OPEN[0], 0.4, OPEN[2]]), watch);
  Object.assign(sim.realism, ALL); run(sim, 3, holdAt([OPEN[0], 0.4, OPEN[2]]), watch);
  Object.assign(sim.realism, { ground: false, wash: false, battery: false }); run(sim, 3, holdAt([OPEN[0], 0.4, OPEN[2]]), watch);
  assert.ok(yMax - yMin < 0.06 && upMin > 0.995 && !sim.crashed && !sim.hits, `height band ${((yMax - yMin) * 1000).toFixed(0)} mm, stayed level`);
});

// Same checks as the collision tests, with every realism switch on.
function fly(p, v, { yaw = 0, secs = 4, sticks = idle } = {}) {
  const sim = createSim(DEFAULT_LAYOUT, { realism: ALL }), q = sim.quad;
  q.p = p.slice(); q.v = v.slice(); q.q = qFromAxisAngle([0, 1, 0], yaw); sim.prev = p.slice();
  let hit = false, vAfter = 0, upMin = 1, yMax = -Infinity;
  for (let i = 0; i < secs / DT; i++) {
    step(sim, sticks);
    hit ||= sim.hits > 0 || sim.inNet;
    if (hit) { vAfter = Math.max(vAfter, Math.hypot(...q.v)); upMin = Math.min(upMin, qRot(q.q, [0, 1, 0])[1]); yMax = Math.max(yMax, q.p[1]); }
  }
  return { sim, q, hit, vAfter, upMin, yMax, up: qRot(q.q, [0, 1, 0])[1], speed: Math.hypot(...q.v) };
}

test('with all realism on, a pole hit still never makes the drone speed up or climb away', () => {
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

test('with all realism on, the safety net still catches and holds the drone', () => {
  const B = createSim(DEFAULT_LAYOUT).course.bounds;
  const fast = fly([1.2, 1, 0], [6, 0, 0]);
  assert.ok(fast.hit && !fast.sim.crashed && fast.q.p[0] < B.x1 && fast.upMin > 0.95, 'a 6 m/s run into the side net: no crash, pushed back, level');
  for (const [name, p, v, sticks, ax, lim] of [
    ['side', [1.2, 1, 0], [3, 0, 0], { ...idle, roll: 1 }, 0, B.x1],
    ['far end', [0, 1, -2.6], [0, 0, -3], { ...idle, pitch: 1 }, 2, -B.z0],
    ['ceiling', [0, 2, 0], [0, 3, 0], { ...idle, throttle: 1 }, 1, B.y1],
  ]) {
    const r = fly(p, v, { sticks, secs: 6 });
    assert.ok(!r.sim.crashed, `${name}: no crash`);
    assert.ok(Math.abs(r.q.p[ax]) < lim + 0.05, `${name}: held at the net (${r.q.p[ax].toFixed(2)})`);
    assert.ok(r.speed < 0.2, `${name}: comes to rest against the net (${r.speed.toFixed(2)} m/s)`);
  }
});

test('the demo flight still scores 120 with every realism switch on, using under half the battery', () => {
  const sim = createSim(DEFAULT_LAYOUT, { realism: ALL });
  const ap = createAutopilot(sim, demoScript(sim));
  while (!ap.done && sim.t < 590) step(sim, ap.sticks(DT));
  const r = sim.rules;
  assert.ok(!sim.crashed, 'no crash');
  assert.deepEqual(r.score, { m1A: 5, m1B: 5, m1C: 10, m1Exit: 5, m2Lift: 10, m2Place: 10, m3Roof: 15, m3Treat: 15, m3Bonus: 5, m4Path: 10, m4Pods: 0, m5Path: 10, m5Beacon: 10 });
  assert.equal(Rules.total(r), 120);
  assert.ok(r.finishTime < 600);
  assert.ok(sim.quad.soc > 0.5 && sim.quad.soc < 0.85, `charge left ${(sim.quad.soc * 100).toFixed(0)} %`);
  assert.equal(sim.battLevel, 0);
});

// The app starts the demo with whatever the pilot has switched on, so every combination it offers must fly.
const DEMO_SCORE = { m1A: 5, m1B: 5, m1C: 10, m1Exit: 5, m2Lift: 10, m2Place: 10, m3Roof: 15, m3Treat: 15, m3Bonus: 5, m4Path: 10, m4Pods: 0, m5Path: 10, m5Beacon: 10 };
function demoRun(realism) {
  const sim = createSim(DEFAULT_LAYOUT, { realism });
  const ap = createAutopilot(sim, demoScript(sim));
  while (!ap.done && sim.t < 590) step(sim, ap.sticks(DT));
  return sim;
}

test('the demo flight scores 120 with each realism switch on by itself', () => {
  for (const key of ['ground', 'wash', 'battery']) {
    const sim = demoRun({ [key]: true });
    assert.ok(!sim.crashed, `${key}: no crash`);
    assert.deepEqual(sim.rules.score, DEMO_SCORE, `${key}: score sheet`);
    assert.equal(Rules.total(sim.rules), 120, `${key}: total`);
  }
});

test('the demo flight scores 120 on every battery pack the app offers, and a smaller pack ends lower', () => {
  const left = [];
  for (const packmAh of DRONE.battery.packs) {
    const sim = demoRun({ ...ALL, packmAh });
    assert.ok(!sim.crashed, `${packmAh} mAh: no crash`);
    assert.deepEqual(sim.rules.score, DEMO_SCORE, `${packmAh} mAh: score sheet`);
    assert.equal(Rules.total(sim.rules), 120, `${packmAh} mAh: total`);
    assert.equal(sim.battLevel, 0, `${packmAh} mAh: no battery warning (${(sim.quad.soc * 100).toFixed(0)} % left)`);
    left.push(sim.quad.soc);
  }
  for (let i = 1; i < left.length; i++) assert.ok(left[i] > left[i - 1] + 0.02, `packs in rising size leave more charge: ${left.map((s) => (s * 100).toFixed(0)).join(', ')} %`);
});
