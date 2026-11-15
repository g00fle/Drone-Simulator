// Optional realism, each part with its own switch in sim.realism:
//   ground  - ground effect over the floor and the tower roof, ceiling effect under the roof
//   wash    - prop wash blowing on loose objects, and buffeting inside the tower
//   battery - a pack that runs down, sags under load and finally gives out
// Headless (no rendering). The strengths below are ESTIMATES from published models, not measurements
// of this drone: tune them once the real one has flown.
import { qRot, yawRot, clamp, mulberry32 } from './math.js';
import { G } from './quad.js';
import { segmentBlocked } from './collide.js';
import { wake } from './objects.js';
import { insideTower } from '../course/geometry.js';
import { PAYLOADS } from '../config/drone.js';

export const AERO = {
  rho: 1.225, // air density, kg/m^3
  propR: 0.0635, // 5 in props
  // Ground effect for a quadrotor (Sanchez-Cuevas, Heredia and Ollero, 2017): each rotor's own image
  // below the floor, its neighbours' images, and the "fountain" of air trapped under the body.
  ground: { bodyK: 2, max: 0.25 },
  // Ceiling effect (Hsiao and Chirarattananon, 2018 form): thrust rises as the props close in on a roof.
  ceiling: { k1: 6.924, k2: 0.02, max: 0.2 },
  wash: {
    hz: 50, // how often the forces on the objects are refreshed
    jet: 0.8, // share of the ideal downwash speed that gets past the guards and frame
    decay: 0.76, // m: the downwash has lost half its speed this far below the props
    core: 0.14, spread: 0.12, // m: the outflow along the floor is fastest at radius core + spread x height
    layer0: 0.02, layerGrow: 0.1, // m: depth of the fast layer hugging the floor = layer0 + layerGrow x radius
    profile: 0.6, // average pressure through that layer, compared with its peak
    reach: 4, // m: no effect further away than this
    // Drag coefficient x side-on area, m^2.
    drag: {
      ball: 0.47 * Math.PI * PAYLOADS.ball.radius ** 2,
      figure: 1.1 * 0.5 * (PAYLOADS.figure.size[0] + PAYLOADS.figure.size[2]) * PAYLOADS.figure.size[1],
      pod: 0.9 * 2 * PAYLOADS.pod.radius * PAYLOADS.pod.height,
    },
  },
  // Inside the tower the drone's own wash comes back off the walls and shakes it.
  // side, lift: random force as a share of the thrust. twist: random torque in N m per newton of thrust.
  buffet: { ramp: 0.3, tau: 0.25, side: 0.012, lift: 0.01, twist: 0.0006, yaw: 0.3 },
};

// Close to a normal distribution with unit spread (four uniform numbers added up).
const gauss = (rng) => (rng() + rng() + rng() + rng() - 2) * Math.sqrt(3);

export const FX_SEED = 20270; // the tower air used when no seed is given (tests, the demo flight)
export function createFx(seed = FX_SEED) {
  return { rng: mulberry32(seed), washClock: 1, washOn: false, gust: [0, 0, 0, 0, 0, 0], inTower: 0 };
}

const geomCache = new WeakMap();
function rotorGeom(cfg) {
  let g = geomCache.get(cfg);
  if (!g) {
    const m = cfg.motorPos, dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    g = { d: dist(m[0], m[1]), b: dist(m[0], m[2]) }; // neighbouring rotors, opposite rotors
    geomCache.set(cfg, g);
  }
  return g;
}

// Extra thrust (0.08 = 8 % more) for one rotor of a quadrotor, z metres above a flat surface.
// d = distance between neighbouring rotors, b = between opposite rotors.
export function groundGain(z, d = 0.159, b = 0.225) {
  const R = AERO.propR, K = AERO.ground;
  if (!(z < 2)) return 0; // too high to matter (under 0.1 %)
  z = Math.max(z, 0.5 * R);
  const z4 = 4 * z * z, R2z = R * R * z;
  const den = 1 - (R / (4 * z)) ** 2 - R2z / Math.sqrt((d * d + z4) ** 3) - (0.5 * R2z) / Math.sqrt((2 * d * d + z4) ** 3)
    - (2 * R2z * K.bodyK) / Math.sqrt((b * b + z4) ** 3);
  return clamp(1 / Math.max(den, 0.5) - 1, 0, K.max);
}

// Extra thrust for a rotor z metres below a flat roof.
export function ceilingGain(z) {
  const R = AERO.propR, K = AERO.ceiling;
  if (!(z < 1)) return 0;
  const x = R / (Math.max(z, 0) + K.k2);
  return clamp(1 / Math.max(1 - (x * x) / K.k1, 0.5) - 1, 0, K.max);
}

// Sets the per-rotor thrust gain (s.ge), pushes loose objects with the prop wash and adds tower buffeting
// to ext (the outside force and torque on the drone for this step).
export function stepAero(sim, dt, ext) {
  const s = sim.quad, cfg = sim.cfg, r = sim.realism, fx = sim.fx, tw = sim.course.tower;
  const up = qRot(s.q, [0, 1, 0]);

  if (r.ground) {
    const R = AERO.propR, geo = rotorGeom(cfg), level = clamp(up[1], 0, 1) ** 2;
    for (let i = 0; i < 4; i++) {
      const rw = qRot(s.q, cfg.motorPos[i]), x = s.p[0] + rw[0], y = s.p[1] + rw[1], z = s.p[2] + rw[2];
      let gain = groundGain(y, geo.d, geo.b);
      if (tw) {
        const q = yawRot([x - tw.x, 0, z - tw.z], -tw.yaw);
        // 1 = the whole prop disc is over (or under) the roof panel, 0 = none of it
        const cover = clamp((tw.roofHalf - Math.max(Math.abs(q[0]), Math.abs(q[2]))) / (2 * R) + 0.5, 0, 1);
        if (cover > 0) {
          const under = tw.roofTop - 0.005; // underside of the roof panel
          if (y > tw.roofTop) gain = cover * groundGain(y - tw.roofTop, geo.d, geo.b) + (1 - cover) * gain;
          else if (y < under) gain += cover * ceilingGain(under - y);
        }
      }
      s.ge[i] = 1 + level * gain;
    }
  } else s.ge[0] = s.ge[1] = s.ge[2] = s.ge[3] = 1;

  if (r.wash) {
    fx.washOn = true;
    fx.washClock += dt;
    if (fx.washClock >= 1 / AERO.wash.hz) { fx.washClock = 0; washForces(sim, up); }
    for (const o of sim.objects) {
      if (!o.wash || o.carried) continue;
      o.f[0] += o.wash[0]; o.f[2] += o.wash[1];
      if (o.asleep) wake(o);
    }
  } else if (fx.washOn) {
    fx.washOn = false; fx.washClock = 1;
    for (const o of sim.objects) o.wash = null;
  }

  const B = AERO.buffet, inside = r.wash && tw && insideTower(tw, s.p) ? 1 : 0;
  fx.inTower += (inside - fx.inTower) * (1 - Math.exp(-dt / B.ramp));
  if (fx.inTower > 0.01 && s.thrust > 0.5) {
    const a = dt / B.tau, k = Math.sqrt(2 * a), g = fx.gust;
    for (let i = 0; i < 6; i++) g[i] += -g[i] * a + k * gauss(fx.rng);
    const T = s.thrust * fx.inTower;
    ext.f[0] += B.side * T * g[0]; ext.f[1] += B.lift * T * g[1]; ext.f[2] += B.side * T * g[2];
    ext.t[0] += B.twist * T * g[3]; ext.t[1] += B.twist * B.yaw * T * g[4]; ext.t[2] += B.twist * T * g[5];
  }
}

// Friction that has to be beaten before the wash can start an object moving.
export const washGrip = (o) => (o.rollMu !== undefined ? o.rollMu : o.mu);

// Downwash model: a jet that slows and widens on the way down, then turns outward along the surface the
// object stands on. Speed along that surface is zero right under the drone, peaks one "core radius" out
// and fades beyond. Sets o.wash = [fx, fz] (newtons, world frame) or null for every loose object.
function washForces(sim, up) {
  const s = sim.quad, W = AERO.wash, tw = sim.course.tower, T = s.thrust;
  const active = T > 0.5 && up[1] > 0.3;
  let overRoof = false;
  if (active && tw && s.p[1] > tw.roofTop) {
    const q = yawRot([s.p[0] - tw.x, 0, s.p[2] - tw.z], -tw.yaw);
    overRoof = Math.abs(q[0]) < tw.roofHalf && Math.abs(q[2]) < tw.roofHalf;
  }
  // Ideal fully-developed downwash speed for the four props together, times the share that survives.
  const U0 = active ? W.jet * Math.sqrt(T / (AERO.rho * 2 * Math.PI * AERO.propR ** 2)) : 0;
  for (const o of sim.objects) {
    o.wash = null;
    if (!active || o.carried || !(o.asleep || o.grounded)) continue;
    const h = s.p[1] - o.p[1];
    if (h < 0.05) continue; // the object is level with the drone or above it
    if (overRoof && o.p[1] < tw.roofTop - 0.05) continue; // the roof catches the wash
    const slant = h / up[1];
    if (slant > W.reach) continue;
    const dx = o.p[0] - (s.p[0] - up[0] * slant), dz = o.p[2] - (s.p[2] - up[2] * slant), rr = Math.hypot(dx, dz);
    if (rr < 1e-4 || rr > W.reach) continue;
    const U = U0 / (1 + slant / W.decay), x = rr / (W.core + W.spread * slant);
    const nx = dx / rr, nz = dz / rr;
    const rel = (U * 2 * x) / (1 + x * x) - (o.v[0] * nx + o.v[2] * nz); // air speed past the object
    if (rel <= 0) continue;
    const depth = Math.min(1, (W.layer0 + W.layerGrow * rr) / o.top); // how much of the object is in the fast layer
    const F = 0.5 * AERO.rho * W.drag[o.type] * depth * W.profile * rel * rel;
    const moving = !o.asleep && Math.hypot(o.v[0], o.v[2]) > 0.03;
    if (!moving && F < washGrip(o) * o.mass * G) continue; // not enough to break it loose
    if (segmentBlocked(s.p, [o.p[0], o.p[1] + o.top / 2, o.p[2]], sim.course.boxes)) continue; // a panel is in the way
    o.wash = [F * nx, F * nz];
  }
}

const lerpTable = (t, x) => {
  const f = clamp(x, 0, 1) * (t.length - 1), i = Math.min(t.length - 2, Math.floor(f));
  return t[i] + (t[i + 1] - t[i]) * (f - i);
};

// Resting (no load) pack voltage at a state of charge 0..1.
export const restVolts = (b, soc) => b.cells * lerpTable(b.ocv, soc);
// Internal resistance of the pack plus wiring, ohms.
export const packOhms = (b, packmAh) => b.rWiring + (b.cells * b.rCellAh) / (packmAh / 1000);

// Electrical power, pack voltage, charge used and how much thrust the pack can still deliver (s.sag).
// on = false: the pack never runs down, and thrust only dips a little at high throttle.
export function stepBattery(s, cfg, on, packmAh, magnetOn, dt) {
  const b = cfg.battery, hoverF = (cfg.mass * G) / 4;
  let p = 0, load = 0;
  for (let i = 0; i < 4; i++) {
    p += Math.pow((s.motors[i] * cfg.maxThrust * s.sag) / hoverF, 1.5); // power grows faster than thrust
    load += s.motors[i];
  }
  s.watts = (b.hoverWatts * p) / 4 + b.avionicsWatts + (magnetOn ? b.magnetWatts : 0);
  if (!on) {
    s.sag = 1 - (b.sagLoad * load) / 4; s.power = 1; s.soc = 1; s.volts = 0; s.amps = 0;
    return;
  }
  const Ah = packmAh / 1000;
  s.soc = clamp(1 - s.usedAh / Ah, 0, 1);
  const rest = restVolts(b, s.soc);
  if (!(s.volts > 0)) s.volts = rest;
  s.amps = s.watts / Math.max(s.volts, 0.5 * rest);
  s.volts += (rest - s.amps * packOhms(b, packmAh) - s.volts) * (1 - Math.exp(-dt / 0.08));
  s.usedAh += (s.amps * dt) / 3600;
  s.usedWh += (s.watts * dt) / 3600;
  s.sag = clamp((s.volts / b.vRef) ** 2, 0.3, 1); // thrust goes with voltage squared
  if (s.soc <= 0) s.power = Math.max(0, s.power - dt / b.fadeSeconds); // empty: the motors fade out
  else s.power = 1;
}
