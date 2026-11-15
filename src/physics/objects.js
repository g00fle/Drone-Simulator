// Loose mission objects: medical pods, rescue figure, foam-ball hazmat pods.
// They translate only (no tipping), which is enough for pushing, carrying and scoring.
import { sub, len, yawRot } from './math.js';
import { sphereFloor, sphereCapsule, sphereBox, sphereCylinder, sphereSphere, contactForce, near } from './collide.js';
import { PAYLOADS } from '../config/drone.js';
import { G } from './quad.js';

const W2 = 150 * 150; // contact stiffness per kg

export function createObjects(course) {
  const objs = [];
  const P = PAYLOADS;
  course.spawns.pods.forEach((p, i) => objs.push({
    type: 'pod', id: `pod${i + 1}`, p: p.slice(), v: [0, 0, 0], yaw: 0, mass: P.pod.mass, mu: 0.6,
    spheres: [{ y: P.pod.radius, r: P.pod.radius }, { y: P.pod.height - P.pod.radius, r: P.pod.radius }],
    cyl: { r: P.pod.radius, h: P.pod.height }, top: P.pod.height,
  }));
  objs.push({
    type: 'figure', id: 'figure', p: course.spawns.figure.p.slice(), v: [0, 0, 0], yaw: course.spawns.figure.yaw, mass: P.figure.mass, mu: 0.6,
    spheres: [{ y: 0.045, r: 0.045 }], cyl: { r: 0.05, h: 0.0635 }, top: P.figure.size[1], hookClear: true, lifted: false,
  });
  course.spawns.balls.forEach((p, i) => objs.push({
    // mu = grip when treated as sliding (the default). rollMu = rolling resistance, used when prop wash is simulated.
    type: 'ball', id: `ball${i + 1}`, p: [p[0], 0, p[2]], v: [0, 0, 0], yaw: 0, mass: P.ball.mass, mu: 0.25, rollMu: 0.08,
    spheres: [{ y: P.ball.radius, r: P.ball.radius }], cyl: null, top: 2 * P.ball.radius, roll: [0, 0, 0],
  }));
  for (const o of objs) { o.f = [0, 0, 0]; o.carried = false; o.asleep = true; o.still = 0; o.grounded = false; o.wash = null; }
  return objs;
}

export function wake(o) { o.asleep = false; o.still = 0; }

// rolling = true lets the foam balls roll freely (rollMu) instead of gripping the floor as if sliding.
export function stepObjects(objs, course, dt, rolling = false) {
  // object-object contacts
  for (let i = 0; i < objs.length; i++) {
    const a = objs[i]; if (a.carried) continue;
    for (let j = i + 1; j < objs.length; j++) {
      const b = objs[j]; if (b.carried || (a.asleep && b.asleep)) continue;
      if (Math.abs(a.p[0] - b.p[0]) > 0.2 || Math.abs(a.p[2] - b.p[2]) > 0.2) continue;
      for (const sa of a.spheres) for (const sb of b.spheres) {
        const hit = sphereSphere([a.p[0], a.p[1] + sa.y, a.p[2]], sa.r, [b.p[0], b.p[1] + sb.y, b.p[2]], sb.r);
        if (!hit) continue;
        const m = Math.min(a.mass, b.mass);
        const f = contactForce(hit, sub(a.v, b.v), m * W2, m * 150, 0.2, m * 40);
        for (let k = 0; k < 3; k++) { a.f[k] += f[k]; b.f[k] -= f[k]; }
        wake(a); wake(b);
      }
    }
  }
  for (const o of objs) {
    if (o.carried || o.asleep) { o.f[0] = o.f[1] = o.f[2] = 0; continue; }
    const k = o.mass * W2, c = o.mass * 150, ct = o.mass * 60;
    const mu = rolling && o.rollMu !== undefined ? o.rollMu : o.mu;
    const F = [o.f[0], o.f[1] - o.mass * G, o.f[2]];
    let supported = false;
    for (const s of o.spheres) {
      const cpos = [o.p[0], o.p[1] + s.y, o.p[2]];
      const apply = (hit) => {
        if (!hit) return;
        const f = contactForce(hit, o.v, k, c, mu, ct);
        F[0] += f[0]; F[1] += f[1]; F[2] += f[2];
        if (hit.n[1] > 0.7) supported = true;
      };
      apply(sphereFloor(cpos, s.r));
      for (const cap of course.capsules) if (near(cpos, s.r, cap)) apply(sphereCapsule(cpos, s.r, cap));
      for (const b of course.boxes) if (near(cpos, s.r, b)) apply(sphereBox(cpos, s.r, b));
      for (const cy of course.cylinders) if (near(cpos, s.r, cy)) apply(sphereCylinder(cpos, s.r, cy));
    }
    // keep objects inside the safety net
    const B = course.bounds;
    if (o.p[0] < B.x0) F[0] += k * (B.x0 - o.p[0]); if (o.p[0] > B.x1) F[0] -= k * (o.p[0] - B.x1);
    if (o.p[2] < B.z0) F[2] += k * (B.z0 - o.p[2]); if (o.p[2] > B.z1) F[2] -= k * (o.p[2] - B.z1);
    const damp = supported ? (o.type === 'ball' ? 0.9 : 6) : 0.3; // rolling resistance / air
    for (let i = 0; i < 3; i++) {
      o.v[i] += (F[i] / o.mass - damp * o.v[i]) * dt;
      o.p[i] += o.v[i] * dt;
    }
    // Hard stop behind the springs, so a dropped or shoved object cannot sink through a pipe, panel or the floor.
    for (const s of o.spheres) {
      const cpos = [o.p[0], o.p[1] + s.y, o.p[2]];
      const out = (hit) => {
        if (!hit || hit.d <= 0.004) return;
        const push = hit.d - 0.004, vn = o.v[0] * hit.n[0] + o.v[1] * hit.n[1] + o.v[2] * hit.n[2];
        for (let i = 0; i < 3; i++) { o.p[i] += hit.n[i] * push; cpos[i] += hit.n[i] * push; if (vn < 0) o.v[i] -= hit.n[i] * vn; }
      };
      out(sphereFloor(cpos, s.r));
      for (const cap of course.capsules) if (near(cpos, s.r, cap)) out(sphereCapsule(cpos, s.r, cap));
      for (const b of course.boxes) if (near(cpos, s.r, b)) out(sphereBox(cpos, s.r, b));
      for (const cy of course.cylinders) if (near(cpos, s.r, cy)) out(sphereCylinder(cpos, s.r, cy));
    }
    if (o.roll) { o.roll[0] += (o.v[2] / o.spheres[0].r) * dt; o.roll[2] -= (o.v[0] / o.spheres[0].r) * dt; }
    o.grounded = supported;
    o.f[0] = o.f[1] = o.f[2] = 0;
    if (supported && len(o.v) < 0.02) { o.still += dt; if (o.still > 0.4) { o.asleep = true; o.v = [0, 0, 0]; } } else o.still = 0;
  }
}

export const atRest = (o) => !o.carried && o.grounded && len(o.v) < 0.05;

// Ring centres/axes of the rescue figure's two harness rings (world frame).
export function figureRings(fig) {
  const c = [fig.p[0], fig.p[1] + PAYLOADS.figure.ringCenterY, fig.p[2]];
  return [{ c, a: yawRot([1, 0, 0], fig.yaw) }, { c, a: yawRot([0, 0, 1], fig.yaw) }];
}
