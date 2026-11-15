// The headless world: drone + course + loose objects + pickups + sensors + rules.
// No rendering here, so it runs the same in the browser, in tests and in the demo autopilot.
import { sub, add, dot, len, cross, qRot, qRotInv, qFromAxisAngle, yawRot } from './physics/math.js';
import { createQuad, stepQuad, pointVelocity } from './physics/quad.js';
import { sphereFloor, sphereCapsule, sphereBox, sphereCylinder, sphereSphere, contactForce, addBounds, near, segmentBlocked } from './physics/collide.js';
import { createObjects, stepObjects, wake, atRest, figureRings } from './physics/objects.js';
import { createFx, stepAero, stepBattery } from './physics/realism.js';
import { createFC, fcUpdate } from './control/fc.js';
import { buildCourse, crossed, inZone, insideTower } from './course/geometry.js';
import * as Rules from './missions/rules.js';
import { DRONE, PAYLOADS } from './config/drone.js';
import { BEACON } from './config/course.js';

export const DT = 1 / 500;
const K = 5000, C = 25, MU = 0.5, CT = 30; // drone contact spring, damper, friction
const NET = { reach: 0.16, k: 420, c: 28, max: 0.45 }; // safety net: drone half-width, spring, damper, most it stretches

// opts.realism: { ground, wash, battery, packmAh } switches the optional realism on (physics/realism.js).
// All of it is off unless asked for. opts.seed fixes the random tower buffeting.
export function createSim(layout, opts = {}) {
  const cfg = opts.drone || DRONE;
  const sim = {
    cfg, layout, opts,
    realism: { ground: false, wash: false, battery: false, packmAh: cfg.battery.packmAh, ...(opts.realism || {}) },
    fx: createFx(opts.seed), battLevel: 0, // battLevel: 0 fine, 1 low, 2 critical, 3 empty
    course: addBounds(buildCourse(layout)),
    quad: createQuad(), fc: createFC(), rules: Rules.createRules(opts),
    objects: null, carried: null, magnetOn: false, gripper: !!opts.gripper,
    grounded: false, onFloor: false, crashed: false, crashReason: '', flipTime: 0, impact: 0,
    beaconLit: false, beaconForce: 0, atHome: false,
    markerHold: {}, markerSeen: null, sensorClock: 0, t: 0, sway: [0, 0],
    payloadSpheres: [],
  };
  sim.objects = createObjects(sim.course);
  resetDrone(sim);
  return sim;
}

export function resetDrone(sim) {
  const q = sim.quad, h = sim.course.spawns.home;
  if (sim.carried) drop(sim);
  q.p = [h[0], -sim.cfg.footZ + 0.002, h[2]]; q.v = [0, 0, 0]; q.w = [0, 0, 0]; q.q = [0, 0, 0, 1]; q.motors = [0, 0, 0, 0];
  sim.fc.altI = 0; sim.fc.armed = true; sim.crashed = false; sim.crashReason = ''; sim.flipTime = 0; sim.prev = q.p.slice();
}

export function setYaw(sim, yaw) { sim.quad.q = qFromAxisAngle([0, 1, 0], yaw); }

export function toggleMagnet(sim, on = !sim.magnetOn) {
  sim.magnetOn = on;
  if (!on && sim.carried && sim.carried.type !== 'figure') drop(sim);
}

function attach(sim, o) {
  sim.carried = o; o.carried = true; o.asleep = false; o.grounded = false;
  if (o.type === 'figure') { sim.quad.payload = { mass: o.mass, r: sim.cfg.hookPoint }; sim.payloadSpheres = []; return; }
  const m = sim.cfg.magnetFace;
  sim.quad.payload = { mass: o.mass, r: [m[0], m[1] - o.top / 2, m[2]] };
  sim.payloadSpheres = o.spheres.map((s) => ({ p: [m[0], m[1] - o.top + s.y, m[2]], r: s.r, part: 'payload' }));
}

function drop(sim) {
  const o = sim.carried; if (!o) return;
  o.carried = false; o.v = sim.quad.v.slice(); wake(o);
  sim.carried = null; sim.quad.payload = null; sim.payloadSpheres = [];
}

// ---- drone contacts ----
function droneContacts(sim) {
  const s = sim.quad, cfg = sim.cfg, co = sim.course;
  const F = [0, 0, 0], T = [0, 0, 0];
  let grounded = false, onFloor = false, beacon = 0, hits = 0;
  const nearCaps = co.capsules.filter((p) => near(s.p, cfg.boundRadius, p));
  const nearBoxes = co.boxes.filter((p) => near(s.p, cfg.boundRadius, p));
  const nearCyl = co.cylinders.filter((p) => near(s.p, cfg.boundRadius, p));
  const nearObjs = sim.objects.filter((o) => !o.carried && Math.abs(o.p[0] - s.p[0]) < 0.4 && Math.abs(o.p[2] - s.p[2]) < 0.4 && Math.abs(o.p[1] - s.p[1]) < 0.5);
  const B = co.bounds;
  const spheres = sim.payloadSpheres.length ? cfg.spheres.concat(sim.payloadSpheres) : cfg.spheres;
  for (const sp of spheres) {
    const rw = qRot(s.q, sp.p), c = add(s.p, rw);
    let vel = null;
    const apply = (hit, k = K, cdamp = C, other = null) => {
      if (!hit) return null;
      vel = vel || pointVelocity(s, rw);
      const vrel = other ? sub(vel, other.v) : vel;
      const vn = -dot(vrel, hit.n); if (vn > sim.impact) sim.impact = vn;
      const f = contactForce(hit, vrel, k, cdamp, MU, CT);
      F[0] += f[0]; F[1] += f[1]; F[2] += f[2];
      const t = cross(rw, f); T[0] += t[0]; T[1] += t[1]; T[2] += t[2];
      if (hit.n[1] > 0.7) grounded = true;
      hits++;
      return f;
    };
    if (apply(sphereFloor(c, sp.r))) onFloor = true;
    for (const p of nearCaps) apply(sphereCapsule(c, sp.r, p));
    for (const p of nearBoxes) apply(sphereBox(c, sp.r, p));
    for (const p of nearCyl) {
      const hit = sphereCylinder(c, sp.r, p), f = apply(hit);
      if (f && p.ob === 'beacon' && hit.n[1] > 0.7) beacon += f[1];
    }
    for (const o of nearObjs) {
      const hit = o.cyl ? sphereCylinder(c, sp.r, { c: o.p, r: o.cyl.r, h: o.cyl.h }) : sphereSphere(c, sp.r, [o.p[0], o.p[1] + o.spheres[0].y, o.p[2]], o.spheres[0].r);
      const ko = Math.min(K, o.mass * 22500);
      const f = apply(hit, ko, Math.min(C, o.mass * 150), o);
      if (f) { o.f[0] -= f[0]; o.f[1] -= f[1]; o.f[2] -= f[2]; wake(o); }
    }
  }
  // Safety net and ceiling: an invisible, springy net around the flying space. It pushes on the drone as a
  // whole (no twisting, no crash), firmly enough to stop a full-speed run within about 0.3 m.
  const R = NET.reach, lim = [[0, B.x0 + R, 1], [0, B.x1 - R, -1], [2, B.z0 + R, 1], [2, B.z1 - R, -1], [1, B.y1 - R * 0.6, -1]];
  let net = false, top = false;
  for (const [ax, edge, dir] of lim) {
    const pen = (edge - s.p[ax]) * dir; if (pen <= 0) continue;
    net = true; if (ax === 1) top = true;
    const vin = -s.v[ax] * dir; // speed into the net
    F[ax] += dir * Math.max(0, NET.k * pen + NET.c * vin);
    if (pen > NET.max) { s.p[ax] = edge - dir * NET.max; if (vin > 0) s.v[ax] = 0; } // the net only stretches so far
  }
  if (net && !sim.inNet) sim.rules.log.push({ t: sim.rules.time, text: 'Out of bounds: the safety net pushed the drone back.', kind: 'warn' });
  sim.inNet = net; sim.atCeiling = top;
  sim.grounded = grounded; sim.onFloor = onFloor; sim.beaconForce = beacon; sim.hits = hits;
  return { f: F, t: T };
}

// ---- pickups ----
function pickups(sim) {
  const s = sim.quad, cfg = sim.cfg;
  const up = qRot(s.q, [0, 1, 0]);
  const fig = sim.objects.find((o) => o.type === 'figure');
  const hook = add(s.p, qRot(s.q, cfg.hookPoint));
  if (sim.carried) {
    const o = sim.carried;
    if (o.type === 'figure') {
      const hang = PAYLOADS.figure.ringCenterY + PAYLOADS.figure.ringID / 2 - 0.003;
      const y = hook[1] - hang;
      const down = y <= 0;
      o.p = [hook[0], Math.max(0, y), hook[2]]; o.v = s.v.slice();
      sim.quad.payload = down ? null : { mass: o.mass, r: cfg.hookPoint };
      if (!down && y > 0.05 && !o.lifted) { o.lifted = true; Rules.event(sim.rules, 'figure_lift'); }
      const nose = qRot(s.q, [0, 0, -1]);
      if (down && dot(s.v, nose) < -0.04) { // set down and the hook backs out
        o.p[1] = 0; o.v = [0, 0, 0]; o.carried = false; o.hookClear = false; wake(o);
        sim.carried = null; sim.quad.payload = null;
        if (o.lifted) { o.lifted = false; Rules.event(sim.rules, 'figure_release', { inSafe: inZone(sim.course.zones.safe, o.p[0], o.p[2], 0.045) }); }
      }
    } else {
      const m = add(s.p, qRot(s.q, cfg.magnetFace));
      o.p = add(m, qRot(s.q, [0, -o.top, 0])); o.v = s.v.slice(); o.q = s.q;
    }
    return;
  }
  // hook through a harness ring
  if (fig && !fig.carried) {
    const rings = figureRings(fig);
    const dist = len(sub(hook, rings[0].c));
    if (!fig.hookClear) { if (dist > 0.05) fig.hookClear = true; }
    else if (dist < 0.03 && up[1] > 0.8) {
      for (const ring of rings) {
        const rel = sub(hook, ring.c), along = dot(rel, ring.a);
        const radial = len([rel[0] - ring.a[0] * along, rel[1] - ring.a[1] * along, rel[2] - ring.a[2] * along]);
        if (Math.abs(along) < 0.007 && radial < PAYLOADS.figure.ringID / 2 - 0.003) { attach(sim, fig); return; }
      }
    }
  }
  // magnet
  if (sim.magnetOn && up[1] > 0.85) {
    const m = add(s.p, qRot(s.q, cfg.magnetFace));
    for (const o of sim.objects) {
      if (o.type === 'figure' || (o.type === 'ball' && !sim.gripper)) continue;
      const dy = m[1] - (o.p[1] + o.top), dh = Math.hypot(m[0] - o.p[0], m[2] - o.p[2]);
      const reach = o.type === 'ball' ? 0.03 : 0.012;
      if (dh < (o.type === 'ball' ? 0.035 : 0.025) && dy > -0.01 && dy < reach) { attach(sim, o); return; }
    }
  }
}

// ---- sensors -> rule events ----
function sensors(sim, dt) {
  const s = sim.quad, co = sim.course, r = sim.rules, p = s.p, prev = sim.prev;
  for (const sn of co.sensors) {
    const dir = crossed(sn, prev, p); if (!dir) continue;
    if (sn.ob === 'tower') {
      if (sn.id === 'entrance') Rules.event(r, dir < 0 ? 'tower_enter' : 'tower_leave_low');
      else if (dir > 0) Rules.event(r, 'tower_exit');
    } else if (sn.ob === 'gate') Rules.event(r, 'gate', { id: sn.id, points: sn.points, label: sn.label, dir });
    else if (sn.ob === 'corridor') Rules.event(r, 'corridor_cross', { level: sn.level, bay: sn.bay, dir });
  }
  sim.prev = p.slice();

  if (sim.onFloor) {
    const q = yawRot([p[0] - co.corridor.x, 0, p[2] - co.corridor.z], -co.corridor.yaw);
    if (Math.abs(q[0]) < co.corridor.half && Math.abs(q[2]) < co.corridor.depth) Rules.event(r, 'corridor_ground');
  }
  const pressed = sim.beaconForce > BEACON.pressForce;
  if (pressed && !sim.beaconPressed) { sim.beaconLit = true; Rules.event(r, 'beacon'); }
  sim.beaconPressed = pressed;

  sim.sensorClock += dt;
  if (sim.sensorClock < 0.05) return;
  const step = sim.sensorClock; sim.sensorClock = 0;

  const speed = len(s.v);
  sim.atHome = sim.onFloor && speed < 0.25 && inZone(co.zones.home, p[0], p[2]);

  // Marker inspection: centred in the FPV view, close, facing, unobstructed, held 1 s.
  sim.markerSeen = null;
  if (r.active === 1 && r.m1.entered && insideTower(co.tower, p)) {
    const cam = sim.cfg.camera, tilt = (cam.tiltUpDeg * Math.PI) / 180;
    const eye = add(p, qRot(s.q, cam.pos)), axis = qRot(s.q, [0, Math.sin(tilt), -Math.cos(tilt)]);
    for (const m of co.markers) {
      if (r.score[`m1${m.id}`]) continue;
      const to = sub(m.p, eye), d = len(to);
      const facing = -dot(to, m.n) / d, centred = dot(to, axis) / d;
      const ok = d < 0.75 && facing > 0.6 && centred > Math.cos((16 * Math.PI) / 180) && !segmentBlocked(eye, m.p, co.boxes);
      if (ok) {
        sim.markerHold[m.id] = (sim.markerHold[m.id] || 0) + step;
        sim.markerSeen = { id: m.id, hold: Math.min(1, sim.markerHold[m.id]) };
        if (sim.markerHold[m.id] >= 1) Rules.event(r, 'marker', { id: m.id, points: m.points });
      } else sim.markerHold[m.id] = 0;
    }
  }

  // Pod deliveries and contained hazmat pods
  if (r.active === 3) {
    const tw = co.tower;
    for (const o of sim.objects) {
      if (o.type !== 'pod' || !atRest(o)) continue;
      const q = yawRot([o.p[0] - tw.x, 0, o.p[2] - tw.z], -tw.yaw);
      if (Math.abs(o.p[1] - tw.roofTop) < 0.03 && Math.abs(q[0]) < tw.roofHalf && Math.abs(q[2]) < tw.roofHalf) Rules.event(r, 'pod_rest', { where: 'roof', id: o.id });
      else if (o.p[1] < 0.02 && inZone(co.zones.treat, o.p[0], o.p[2])) Rules.event(r, 'pod_rest', { where: 'treat', id: o.id });
    }
  }
  if (r.active === 4) {
    let n = 0;
    for (const o of sim.objects) if (o.type === 'ball' && !o.carried && o.p[1] < 0.05 && inZone(co.zones.contain, o.p[0], o.p[2])) n++;
    Rules.event(r, 'balls', { count: n });
  }
}

// Hard stop behind the springs: if a collision sphere is still deeper than SLOP inside something solid,
// move the drone straight back out and cancel its speed into the surface. The springs alone are soft
// enough that a fast hit could sink past a pipe or panel and come out the far side.
const SLOP = 0.003;
function depenetrate(sim) {
  const s = sim.quad, cfg = sim.cfg, co = sim.course;
  const caps = co.capsules.filter((p) => near(s.p, cfg.boundRadius, p));
  const boxes = co.boxes.filter((p) => near(s.p, cfg.boundRadius, p));
  const cyls = co.cylinders.filter((p) => near(s.p, cfg.boundRadius, p));
  const spheres = sim.payloadSpheres.length ? cfg.spheres.concat(sim.payloadSpheres) : cfg.spheres;
  const local = spheres.map((sp) => ({ rw: qRot(s.q, sp.p), r: sp.r }));
  for (let pass = 0; pass < 6; pass++) {
    let worst = null, arm = null, cur = null;
    const see = (hit) => { if (hit && hit.d > SLOP && (!worst || hit.d > worst.d)) { worst = hit; arm = cur; } };
    for (const sp of local) {
      const c = add(s.p, sp.rw); cur = sp.rw;
      see(sphereFloor(c, sp.r));
      for (const p of caps) see(sphereCapsule(c, sp.r, p));
      for (const p of boxes) see(sphereBox(c, sp.r, p));
      for (const p of cyls) see(sphereCylinder(c, sp.r, p));
    }
    if (!worst) return;
    const n = worst.n, push = worst.d - SLOP;
    s.p = [s.p[0] + n[0] * push, s.p[1] + n[1] * push, s.p[2] + n[2] * push];
    // Stop the touching point itself (a dead-stop impulse on the rigid body), so the drone can neither
    // slide nor swing further into the obstacle.
    const rb = qRotInv(s.q, arm), nb = qRotInv(s.q, n), rxn = cross(rb, nb), I = cfg.inertia;
    const vn = dot(add(s.v, qRot(s.q, cross(s.w, rb))), n);
    if (vn < 0) {
      const j = -vn / (1 / cfg.mass + (rxn[0] * rxn[0]) / I[0] + (rxn[1] * rxn[1]) / I[1] + (rxn[2] * rxn[2]) / I[2]);
      s.v = [s.v[0] + (n[0] * j) / cfg.mass, s.v[1] + (n[1] * j) / cfg.mass, s.v[2] + (n[2] * j) / cfg.mass];
      s.w = [s.w[0] + (rxn[0] * j) / I[0], s.w[1] + (rxn[1] * j) / I[1], s.w[2] + (rxn[2] * j) / I[2]];
    }
  }
}

function crashCheck(sim, dt) {
  const up = qRot(sim.quad.q, [0, 1, 0]);
  if (sim.hits && up[1] < 0.2) sim.flipTime += dt; else sim.flipTime = 0;
  if (!sim.crashed && sim.flipTime > 0.6) { sim.crashed = true; sim.crashReason = 'Flipped over'; }
  if (!sim.crashed && sim.impact > 4.5) { sim.crashed = true; sim.crashReason = 'Hard impact'; }
  sim.impact = 0;
  if (sim.crashed) sim.fc.armed = false;
}

// One fixed physics step. sticks: { throttle, pitch, roll, yaw } each -1..1.
// A fast drone is advanced in smaller slices so no part of it moves more than about 4 mm between
// collision checks (the thinnest collision sphere is 4 mm in radius).
export function step(sim, sticks, dt = DT) {
  const q = sim.quad, n = Math.max(1, Math.min(8, Math.ceil(((len(q.v) + len(q.w) * 0.2) * dt) / 0.004)));
  for (let i = 0; i < n; i++) stepOnce(sim, sticks, dt / n);
}

// Battery warnings for the mission log, one each time the charge drops into a lower band.
const BATT_TEXT = ['', 'Battery low: head back to Home Base soon.', 'Battery critical: land now.', 'Battery empty: the motors are stopping.'];
function batteryWatch(sim) {
  const b = sim.cfg.battery, soc = sim.quad.soc;
  const level = !sim.realism.battery ? 0 : soc <= 0 ? 3 : soc <= b.criticalAt ? 2 : soc <= b.lowAt ? 1 : 0;
  if (level > sim.battLevel) sim.rules.log.push({ t: sim.rules.time, text: BATT_TEXT[level], kind: 'warn' });
  sim.battLevel = level;
}

function stepOnce(sim, sticks, dt) {
  const ext = droneContacts(sim);
  crashCheck(sim, dt);
  stepAero(sim, dt, ext);
  const cmd = fcUpdate(sim.fc, sim.quad, sim.cfg, sticks, dt, sim.grounded, sim.hits > 0, sim.atCeiling);
  sim.lastCmd = cmd;
  stepQuad(sim.quad, sim.cfg, cmd, dt, ext);
  stepBattery(sim.quad, sim.cfg, sim.realism.battery, sim.realism.packmAh, sim.magnetOn, dt);
  batteryWatch(sim);
  if (sim.hits) depenetrate(sim);
  stepObjects(sim.objects, sim.course, dt, sim.realism.wash);
  pickups(sim);
  sensors(sim, dt);
  Rules.tick(sim.rules, dt);
  sim.t += dt;
}

export { Rules };
