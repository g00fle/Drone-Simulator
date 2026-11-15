// Scripted autopilot: flies a full run on the default layout. Used by ?demo=1 and by the tests.
import { qRot, clamp, len, IN } from '../physics/math.js';
import { Rules, toggleMagnet } from '../sim.js';
import { GATE, CORRIDOR, TOWER } from '../config/course.js';
import { PAYLOADS } from '../config/drone.js';

const wrap = (a) => Math.atan2(Math.sin(a), Math.cos(a));
export const yawOf = (q) => { const n = qRot(q, [0, 0, -1]); return Math.atan2(-n[0], -n[2]); };

// Sticks that fly toward a point, holding a heading.
export function flyTo(sim, tgt, yawT = 0, speed = 0.8) {
  const s = sim.quad, k = sim.cfg.fc, yaw = yawOf(s.q);
  const ex = tgt[0] - s.p[0], ez = tgt[2] - s.p[2], d = Math.hypot(ex, ez);
  const vmax = Math.min(speed, d * 1.6), vx = d > 1e-6 ? (ex / d) * vmax : 0, vz = d > 1e-6 ? (ez / d) * vmax : 0;
  const ax = (vx - s.v[0]) * 2.2, az = (vz - s.v[2]) * 2.2;
  const f = [-Math.sin(yaw), -Math.cos(yaw)], r = [Math.cos(yaw), -Math.sin(yaw)];
  const maxA = (k.maxAngleDeg * Math.PI) / 180;
  const stick = (a) => clamp(Math.atan(a / 9.81) / maxA, -0.6, 0.6);
  return {
    pitch: stick(ax * f[0] + az * f[1]), roll: stick(ax * r[0] + az * r[1]),
    throttle: clamp(((tgt[1] - s.p[1]) * 2.5) / k.climbRate, -0.7, 0.9),
    yaw: clamp(-wrap(yawT - yaw) * 2.5, -1, 1),
  };
}

export function createAutopilot(sim, script) {
  const ap = { i: 0, t: 0, done: false, label: '', last: [0, 0, 0], yaw: 0 };
  ap.sticks = (dt) => {
    while (ap.i < script.length) {
      const st = script[ap.i];
      if (st.label) ap.label = st.label;
      if (st.do) { st.do(sim); ap.i++; ap.t = 0; continue; }
      ap.t += dt;
      const next = () => { ap.i++; ap.t = 0; };
      if (st.to) {
        ap.last = st.to; if (st.yaw !== undefined) ap.yaw = st.yaw;
        const e = [st.to[0] - sim.quad.p[0], st.to[1] - sim.quad.p[1], st.to[2] - sim.quad.p[2]];
        const arrived = len(e) < (st.tol || 0.04) && len(sim.quad.v) < (st.vtol || 0.08) && Math.abs(wrap(ap.yaw - yawOf(sim.quad.q))) < 0.06;
        if ((st.until && st.until(sim)) || (!st.until && arrived && (ap.hold = (ap.hold || 0) + dt) >= (st.hold || 0)) || ap.t > (st.timeout || 25)) { ap.hold = 0; next(); continue; }
        if (!arrived) ap.hold = 0;
        return flyTo(sim, st.to, ap.yaw, st.speed);
      }
      if (st.land) {
        if ((sim.onFloor && ap.t > 0.5 && len(sim.quad.v) < 0.05) || ap.t > 8) { next(); continue; }
        return { ...flyTo(sim, [ap.last[0], 0, ap.last[2]], ap.yaw, 0.3), throttle: -0.45 };
      }
      if (st.wait) { if (ap.t >= st.wait) { next(); continue; } return { throttle: st.throttle ?? 0, pitch: 0, roll: 0, yaw: 0 }; }
      next();
    }
    ap.done = true;
    return { throttle: -0.5, pitch: 0, roll: 0, yaw: 0 };
  };
  return ap;
}

// Full run for a layout with every obstacle at rot 0 (the default). Expected score: 120.
export function demoScript(sim) {
  const L = sim.layout.items, cfg = sim.cfg, H = 1.7;
  const com = -cfg.footZ; // CoM height when standing
  const mid = (cfg.footZ + cfg.topZ) / 2; // body centre offset for threading openings
  const hook = cfg.hookPoint, mag = cfg.magnetFace;
  const home = [L.home.x, H, L.home.z];
  const declare = (n) => ({ do: (s) => Rules.declare(s.rules, n, s.atHome), label: `Mission ${n}` });
  const goHome = [{ to: home, label: 'Return to Home Base' }, { to: [home[0], com + 0.15, home[2]], speed: 0.5 }, { land: true }, { wait: 0.4, throttle: -0.5 }];
  const s = [{ wait: 0.5, throttle: -0.5 }];

  // Mission 1: in by the ground entrance, markers A, B, C, out by the upper opening.
  // The camera sits above the centre of mass and looks 10 degrees up, so the drone hovers a little below
  // each marker to hold it in the middle of the picture. Each step waits for the marker to be scored.
  const T = L.tower, c = TOWER.cube, my = (level) => (level + 0.5) * c - 0.055;
  const seen = (id) => (m) => m.rules.score[`m1${id}`] > 0;
  s.push(declare(1), { to: [home[0], H, home[2]] }, { to: [T.x, H, T.z + 1.0] }, { to: [T.x, my(0), T.z + 0.9], speed: 0.5 },
    { to: [T.x, my(0), T.z], speed: 0.35, until: seen('A'), timeout: 20, label: 'Marker A' },
    { to: [T.x, my(0), T.z], yaw: Math.PI / 2, until: seen('B'), timeout: 20, label: 'Marker B' },
    { to: [T.x, my(1), T.z], yaw: 0, speed: 0.3, until: seen('C'), timeout: 20, label: 'Marker C' },
    { to: [T.x + 0.95, my(1), T.z], speed: 0.35, label: 'Upper exit' }, { to: [T.x + 0.95, H, T.z] }, ...goHome);

  // Mission 2: hook a harness ring, carry the figure to the Safe Zone, set it down, back out.
  const F = L.figure, ringY = PAYLOADS.figure.ringCenterY, yHook = ringY - hook[1];
  s.push(declare(2), { to: [home[0], H, home[2]], yaw: 0 }, { to: [F.x, H, F.z + 0.9] },
    { to: [F.x, yHook, F.z - hook[2] + 0.2], speed: 0.4, tol: 0.012, vtol: 0.04, label: 'Line up on the ring' },
    { to: [F.x, yHook, F.z - hook[2] - 0.05], speed: 0.06, until: (m) => !!m.carried, timeout: 12 },
    { to: [F.x, H, F.z], speed: 0.5 }, { to: [L.safe.x, H, L.safe.z - hook[2]] },
    { to: [L.safe.x, 0.19, L.safe.z - hook[2]], speed: 0.4, tol: 0.03, label: 'Set down in the Safe Zone' },
    { to: [L.safe.x, 0.19, L.safe.z - hook[2] + 0.3], speed: 0.25, until: (m) => !m.carried, timeout: 6 },
    { to: [L.safe.x, H, L.safe.z + 0.3] }, ...goHome);

  // Mission 3: pods by magnet to the tower roof, then the floor treatment location.
  const podTop = PAYLOADS.pod.height, carryY = podTop - mag[1];
  const grab = (P) => [{ do: (m) => toggleMagnet(m, true) }, { to: [P.x, 0.6, P.z] },
    { to: [P.x, carryY - 0.01, P.z], speed: 0.3, until: (m) => !!m.carried, timeout: 10, label: 'Magnet pickup' }];
  const roofY = c * 2 + 0.016 + carryY + 0.02;
  s.push(declare(3), { to: [home[0], H, home[2]], yaw: 0 }, ...grab(L.pod1), { to: [L.pod1.x, roofY + 0.2, L.pod1.z] }, { to: [T.x, roofY + 0.2, T.z] },
    { to: [T.x, roofY, T.z], speed: 0.3, tol: 0.03, label: 'Roof delivery' }, { do: (m) => toggleMagnet(m, false) }, { wait: 0.3 },
    { to: [T.x, roofY + 0.3, T.z], hold: 1 }, { to: [L.pod2.x, roofY + 0.3, L.pod2.z] }, ...grab(L.pod2),
    { to: [L.pod2.x, H, L.pod2.z] }, { to: [1.25, H, 0.2] }, { to: [L.treat.x, H, L.treat.z] },
    { to: [L.treat.x, carryY + 0.03, L.treat.z], speed: 0.4, tol: 0.03, label: 'Treatment location' }, { do: (m) => toggleMagnet(m, false) }, { wait: 0.3 },
    { to: [L.treat.x, H, L.treat.z], hold: 0.8 }, ...goHome);

  // Mission 4: High Risk passage outbound, then call the mission complete.
  const hi = GATE.openings[2], gy = (hi.y0 + hi.y1) / 2 - mid, G = L.gate;
  s.push(declare(4), { to: [home[0], gy, home[2]], yaw: 0 }, { to: [G.x, gy, G.z + 0.8], tol: 0.02, hold: 0.3, label: 'High Risk passage' },
    { to: [G.x, gy, G.z - 0.6], speed: 0.4 }, { to: [G.x, H + 0.1, G.z - 0.6] }, ...goHome, { do: (m) => Rules.completeMission(m.rules) });

  // Mission 5: weave the Lower Pathway, then press the beacon with a foot.
  const C = L.corridor, bw = CORRIDOR.bayWidth, cy = CORRIDOR.lower / 2 - mid + 0.005, B = L.beacon;
  const bay = (i) => C.x + (i - 1) * bw, foot = 0.192 * Math.SQRT1_2;
  s.push(declare(5), { to: [home[0], H, home[2]], yaw: 0 }, { to: [bay(0), H, C.z + 0.75] },
    { to: [bay(0), cy, C.z + 0.75], speed: 0.4, tol: 0.02, label: 'Lower Pathway' }, { to: [bay(0), cy, C.z - 0.55], speed: 0.4, tol: 0.03 },
    { to: [bay(1), cy, C.z - 0.55], speed: 0.3, tol: 0.03 }, { to: [bay(1), cy, C.z + 0.55], speed: 0.4, tol: 0.03 },
    { to: [bay(2), cy, C.z + 0.55], speed: 0.3, tol: 0.03 }, { to: [bay(2), cy, C.z - 0.55], speed: 0.4, tol: 0.03 },
    { to: [bay(2), 1.3, C.z - 0.55] }, { to: [1.25, 1.3, B.z + 0.6] }, { to: [B.x - foot, 0.5, B.z + foot], tol: 0.02, label: 'Beacon' },
    { land: true }, { wait: 0.5, throttle: -1 }, { to: [B.x - foot, H, B.z + foot + 0.3] }, ...goHome,
    { do: (m) => Rules.flightComplete(m.rules, m.atHome), label: 'Flight Complete' });
  return s;
}

void IN;
