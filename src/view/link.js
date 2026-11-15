// FPV video link: the delay, static and break-up a pilot sees through goggles.
// Pure logic (no DOM, no three.js), so it runs in the tests. view/fpvfx.js draws the result.
// The numbers are ESTIMATES for typical hobby video gear, not measurements of this drone's camera and
// transmitter (the brief does not name a video system): tune them once the real one has flown.
import { sub, len, dot, clamp, yawRot } from '../physics/math.js';

export const LINK = {
  // Glass-to-glass delay, seconds. Digital also slows down as the signal weakens.
  analog: { delay: 0.022, grain: 0.06 },
  digital: { delay: 0.042, weakDelay: 0.03, clean: 0.9, freezeAt: 0.7 },
  range: 60, // m: distance at which the picture is gone (the 20 ft field is far inside this)
  wallLoss: 0.12, // each tower panel between the drone and the pilot
  nullLoss: 0.3, // the antenna's blind spot (straight above and below the drone) pointing at the pilot
  nullPower: 8, // how narrow that blind spot is: half the loss is gone 23 degrees away from it
  smooth: 0.15, // s: how quickly the picture follows a change in signal
  // Break-ups: how often (per second) and how long (seconds).
  glitch: { open: 0.04, tower: 0.35, weak: 1.5, min: 0.08, max: 0.35 },
};

export const LINK_TYPES = ['analog', 'digital'];

// Does the straight line a-b pass through the box? Exact, in the box's own frame.
function crosses(a, b, box) {
  const p = yawRot(sub(a, box.c), -box.yaw), d = yawRot(sub(b, a), -box.yaw);
  let t0 = 0, t1 = 1;
  for (let i = 0; i < 3; i++) {
    const h = box.h[i];
    if (Math.abs(d[i]) < 1e-12) { if (Math.abs(p[i]) > h) return false; continue; }
    let lo = (-h - p[i]) / d[i], hi = (h - p[i]) / d[i];
    if (lo > hi) [lo, hi] = [hi, lo];
    t0 = Math.max(t0, lo); t1 = Math.min(t1, hi);
    if (t0 > t1) return false;
  }
  return true;
}

// How many tower panels (walls or roof) lie on the straight line between two points.
export function panelsBetween(a, b, boxes) {
  let n = 0;
  for (const box of boxes) if ((box.kind === 'panel' || box.kind === 'roof') && crosses(a, b, box)) n++;
  return n;
}

// Signal quality, 1 = perfect, 0 = no picture. pos = the drone, up = its up direction (unit vector),
// pilot = where the receiver is, boxes = the course panels.
export function linkQuality(pos, up, pilot, boxes) {
  const d = sub(pilot, pos), dist = len(d);
  const axis = dist > 1e-6 ? Math.abs(dot(up, d)) / dist : 0;
  return clamp(1 - (dist / LINK.range) ** 2 - LINK.wallLoss * panelsBetween(pos, pilot, boxes) - LINK.nullLoss * axis ** LINK.nullPower, 0, 1);
}

export function createLink(rng = Math.random) {
  return { rng, q: 1, target: 1, noise: 0, tear: 0, frozen: false, delay: 0, left: 0, strength: 0, glitches: 0 };
}

// Advance the link by dt seconds.
//   quality  - from linkQuality
//   enclosed - the drone is inside the tower, where reflections off the panels cause extra break-ups
//   type     - 'analog' or 'digital'
// Sets: q (smoothed quality), delay (s), noise (0..1 static or blockiness), tear (0 = none, up to 1 = the
// strength of the break-up happening now) and frozen (digital only: the picture is stuck on an old frame).
export function stepLink(link, dt, { quality = 1, enclosed = false, type = 'analog' } = {}) {
  const G = LINK.glitch, digital = type === 'digital';
  link.target = clamp(quality, 0, 1);
  link.q += (link.target - link.q) * (1 - Math.exp(-dt / LINK.smooth));
  const weak = 1 - link.q;
  if (link.left > 0) link.left = Math.max(0, link.left - dt);
  else {
    const rate = G.open + (enclosed ? G.tower : 0) + weak * G.weak;
    if (link.rng() < 1 - Math.exp(-rate * dt)) {
      link.left = G.min + link.rng() * (G.max - G.min);
      link.strength = clamp(0.35 + 0.65 * link.rng() + 0.5 * weak, 0, 1);
      link.glitches++;
    }
  }
  link.tear = link.left > 0 ? link.strength : 0;
  if (digital) {
    const D = LINK.digital;
    // Digital video hides a weak signal until it cannot: clean, then blocky, then frozen.
    link.noise = clamp(Math.max(0, D.clean - link.q) * 1.2 + link.tear * 0.6, 0, 1);
    link.frozen = link.tear >= D.freezeAt || link.q < 0.12;
    link.delay = D.delay + D.weakDelay * clamp((D.clean - link.q) / D.clean, 0, 1);
  } else {
    // Analog video shows every bit of a weak signal as static, but never stops.
    link.noise = clamp(LINK.analog.grain + weak * 0.7 + link.tear * 0.5, 0, 1);
    link.frozen = false;
    link.delay = LINK.analog.delay;
  }
  return link;
}

// Remembers recent drone poses so the camera can be drawn where the drone was `delay` seconds ago.
export function createPoseDelay(size = 96) {
  const buf = Array.from({ length: size }, () => ({ t: 0, p: [0, 0, 0], q: [0, 0, 0, 1] }));
  let head = 0, count = 0; // head = index of the newest entry
  const at = (i) => buf[(head - i + size * 2) % size]; // i = 0 is the newest
  const out = { p: [0, 0, 0], q: [0, 0, 0, 1] };
  return {
    get length() { return count; },
    clear() { count = 0; },
    // t must not go backwards (a jump back clears the history).
    push(t, p, q) {
      if (count && t < at(0).t) count = 0;
      if (!count || t > at(0).t) { head = (head + 1) % size; count = Math.min(size, count + 1); }
      const e = at(0);
      e.t = t; e.p[0] = p[0]; e.p[1] = p[1]; e.p[2] = p[2]; e.q[0] = q[0]; e.q[1] = q[1]; e.q[2] = q[2]; e.q[3] = q[3];
    },
    // The pose at time t, blended between the two nearest remembered poses. null if nothing is remembered.
    sample(t) {
      if (!count) return null;
      let i = 0;
      while (i < count - 1 && at(i).t > t) i++; // at(i) = the newest entry not after t (or the oldest)
      const a = at(i), b = i > 0 ? at(i - 1) : a;
      const f = b.t > a.t ? clamp((t - a.t) / (b.t - a.t), 0, 1) : 0;
      for (let k = 0; k < 3; k++) out.p[k] = a.p[k] + (b.p[k] - a.p[k]) * f;
      // shortest-way blend of the two rotations
      const s = a.q[0] * b.q[0] + a.q[1] * b.q[1] + a.q[2] * b.q[2] + a.q[3] * b.q[3] < 0 ? -1 : 1;
      let n = 0;
      for (let k = 0; k < 4; k++) { out.q[k] = a.q[k] + (s * b.q[k] - a.q[k]) * f; n += out.q[k] * out.q[k]; }
      n = Math.sqrt(n) || 1;
      for (let k = 0; k < 4; k++) out.q[k] /= n;
      return out;
    },
  };
}
