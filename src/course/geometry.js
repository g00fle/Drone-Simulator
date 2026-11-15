// Turns a layout into collision primitives, sensors and markers. Pure data, no three.js,
// so physics, scoring and tests all share it.
import { IN, yawRot } from '../physics/math.js';
import { PIPE_R, FIELD, TOWER, GATE, FLOOR_FRAME, CORRIDOR, BEACON, ZONES } from '../config/course.js';
import { PAYLOADS } from '../config/drone.js';

const rad = (d) => (d * Math.PI) / 180;

function placer(item) {
  const yaw = rad(item.rot || 0);
  return {
    yaw,
    pt: (p) => { const q = yawRot(p, yaw); return [q[0] + item.x, q[1], q[2] + item.z]; },
    dir: (d) => yawRot(d, yaw),
  };
}

export function buildCourse(layout) {
  const it = layout.items;
  const c = { capsules: [], boxes: [], cylinders: [], sensors: [], markers: [], zones: {}, spawns: {}, tower: null, corridor: null, beacon: null };
  const pipe = (P, ob, a, b) => c.capsules.push({ a: P.pt(a), b: P.pt(b), r: PIPE_R, ob });
  const sensor = (P, ob, id, center, normal, halfW, y0, y1, extra = {}) =>
    c.sensors.push({ ob, id, o: P.pt(center), n: P.dir(normal), t: P.dir([normal[2], 0, -normal[0]]), halfW, y0, y1, ...extra });

  // ---- Medical Tower ----
  {
    const P = placer(it.tower), h = TOWER.cube / 2, H = TOWER.cube * TOWER.levels;
    for (const sx of [-1, 1]) for (const sz of [-1, 1]) pipe(P, 'tower', [sx * h, 0, sz * h], [sx * h, H, sz * h]);
    for (let l = 0; l <= TOWER.levels; l++) {
      const y = l === 0 ? PIPE_R : l * TOWER.cube;
      pipe(P, 'tower', [-h, y, -h], [h, y, -h]); pipe(P, 'tower', [-h, y, h], [h, y, h]);
      pipe(P, 'tower', [-h, y, -h], [-h, y, h]); pipe(P, 'tower', [h, y, -h], [h, y, h]);
    }
    const walls = {
      front: { c: [0, 0, h], hx: TOWER.panel / 2, hz: TOWER.panelT / 2, n: [0, 0, 1] },
      back: { c: [0, 0, -h], hx: TOWER.panel / 2, hz: TOWER.panelT / 2, n: [0, 0, -1] },
      left: { c: [-h, 0, 0], hx: TOWER.panelT / 2, hz: TOWER.panel / 2, n: [-1, 0, 0] },
      right: { c: [h, 0, 0], hx: TOWER.panelT / 2, hz: TOWER.panel / 2, n: [1, 0, 0] },
    };
    const open = [TOWER.lowerOpen, TOWER.upperOpen];
    for (let l = 0; l < TOWER.levels; l++) {
      const yc = (l + 0.5) * TOWER.cube;
      for (const [name, w] of Object.entries(walls)) {
        if (name === open[l]) {
          sensor(P, 'tower', l === 0 ? 'entrance' : 'exit', [w.c[0], 0, w.c[2]], w.n, h, l * TOWER.cube, (l + 1) * TOWER.cube);
          continue;
        }
        { const cc = P.pt([w.c[0], yc, w.c[2]]); // collision slab is thicker than the 5 mm board so fast hits cannot tunnel
          c.boxes.push({ c: cc, h: [w.hx, TOWER.panel / 2, w.hz], yaw: P.yaw, ob: 'tower', kind: 'panel', col: { c: cc, h: [Math.max(w.hx, 0.012), TOWER.panel / 2, Math.max(w.hz, 0.012)] } }); }
      }
    }
    const roofY = H + PIPE_R + TOWER.panelT / 2;
    c.boxes.push({ c: P.pt([0, roofY, 0]), h: [TOWER.panel / 2, TOWER.panelT / 2, TOWER.panel / 2], yaw: P.yaw, ob: 'tower', kind: 'roof',
      col: { c: P.pt([0, roofY + TOWER.panelT / 2 - 0.012, 0]), h: [TOWER.panel / 2, 0.012, TOWER.panel / 2] } });
    for (const m of TOWER.markers) {
      const w = walls[m.wall];
      const inward = [-w.n[0], 0, -w.n[2]];
      const pos = [w.c[0] + inward[0] * 0.006, (m.level + 0.5) * TOWER.cube, w.c[2] + inward[2] * 0.006];
      c.markers.push({ id: m.id, points: m.points, p: P.pt(pos), n: P.dir(inward), size: 0.2 });
    }
    c.tower = { x: it.tower.x, z: it.tower.z, yaw: P.yaw, half: h, height: H, roofTop: roofY + TOWER.panelT / 2, roofHalf: TOWER.panel / 2 };
  }

  // ---- Hazard Passage gate ----
  {
    const P = placer(it.gate), h = GATE.width / 2, top = GATE.openings[GATE.openings.length - 1].y1;
    for (const sx of [-1, 1]) {
      pipe(P, 'gate', [sx * h, 0, 0], [sx * h, top, 0]);
      pipe(P, 'gate', [sx * h, PIPE_R, -GATE.foot], [sx * h, PIPE_R, GATE.foot]);
    }
    for (const o of GATE.openings) {
      pipe(P, 'gate', [-h, o.y1, 0], [h, o.y1, 0]);
      sensor(P, 'gate', o.id, [0, 0, 0], [0, 0, -1], h, o.y0, o.y1, { points: o.points, label: o.label });
    }
  }

  // ---- Hazmat floor frame + foam balls ----
  {
    const P = placer(it.floorFrame), L = FLOOR_FRAME.length / 2, W = FLOOR_FRAME.width / 2, y = PIPE_R;
    pipe(P, 'floorFrame', [-L, y, -W], [L, y, -W]); pipe(P, 'floorFrame', [-L, y, W], [L, y, W]);
    pipe(P, 'floorFrame', [-L, y, -W], [-L, y, W]); pipe(P, 'floorFrame', [L, y, -W], [L, y, W]);
    // ASSUMPTION: the two 12 in stubs point inward from opposite sides (40 + 18 split, mirrored).
    const sx = -L + FLOOR_FRAME.stubAt;
    pipe(P, 'floorFrame', [sx, y, -W], [sx, y, -W + FLOOR_FRAME.stub]);
    pipe(P, 'floorFrame', [-sx, y, W], [-sx, y, W - FLOOR_FRAME.stub]);
    c.spawns.balls = [];
    const r = PAYLOADS.ball.radius;
    for (const end of [-1, 1]) {
      const offs = [[0, -0.12], [0, 0], [0, 0.12], [0.1, -0.06], [0.1, 0.06]];
      for (const [dx, dz] of offs) c.spawns.balls.push(P.pt([end * (L - 0.12 - dx), r, dz]));
    }
  }

  // ---- Communication Corridor ----
  {
    const P = placer(it.corridor), n = CORRIDOR.bays, bw = CORRIDOR.bayWidth, half = (n * bw) / 2;
    for (let i = 0; i <= n; i++) {
      const x = -half + i * bw;
      pipe(P, 'corridor', [x, 0, 0], [x, CORRIDOR.height, 0]);
      pipe(P, 'corridor', [x, PIPE_R, -CORRIDOR.foot], [x, PIPE_R, CORRIDOR.foot]);
    }
    pipe(P, 'corridor', [-half, CORRIDOR.lower, 0], [half, CORRIDOR.lower, 0]);
    pipe(P, 'corridor', [-half, CORRIDOR.height, 0], [half, CORRIDOR.height, 0]);
    for (let i = 0; i < n; i++) {
      const x = -half + (i + 0.5) * bw;
      sensor(P, 'corridor', `lower${i}`, [x, 0, 0], [0, 0, -1], bw / 2, 0, CORRIDOR.lower, { level: 'lower', bay: i });
      sensor(P, 'corridor', `upper${i}`, [x, 0, 0], [0, 0, -1], bw / 2, CORRIDOR.lower, CORRIDOR.height, { level: 'upper', bay: i });
    }
    c.corridor = { x: it.corridor.x, z: it.corridor.z, yaw: P.yaw, half, depth: 0.45 };
  }

  // ---- Beacon (tap light) ----
  c.beacon = { c: [it.beacon.x, 0, it.beacon.z], r: BEACON.radius, h: BEACON.height };
  c.cylinders.push({ ...c.beacon, ob: 'beacon' });

  // ---- Zones and loose-object spawns ----
  for (const id of Object.keys(ZONES)) c.zones[id] = { id, ...ZONES[id], x: it[id].x, z: it[id].z, yaw: rad(it[id].rot || 0) };
  c.spawns.pods = [[it.pod1.x, 0, it.pod1.z], [it.pod2.x, 0, it.pod2.z]];
  c.spawns.figure = { p: [it.figure.x, 0, it.figure.z], yaw: rad(it.figure.rot || 0) };
  c.spawns.home = [it.home.x, 0, it.home.z];

  const m = FIELD.margin;
  c.bounds = { x0: -FIELD.width / 2 - m, x1: FIELD.width / 2 + m, z0: -FIELD.length / 2 - m, z1: FIELD.length / 2 + FIELD.pilotDepth, y1: FIELD.ceiling };
  return c;
}

// Is world point (x, z) inside a zone rectangle, shrunk by `margin` on every side?
export function inZone(zone, x, z, margin = 0) {
  const p = yawRot([x - zone.x, 0, z - zone.z], -zone.yaw);
  return Math.abs(p[0]) <= zone.w / 2 - margin && Math.abs(p[2]) <= zone.d / 2 - margin;
}

// Did the move prev -> cur pass through a sensor rectangle? Returns +1 (along the normal), -1, or 0.
export function crossed(s, prev, cur) {
  const d0 = (prev[0] - s.o[0]) * s.n[0] + (prev[2] - s.o[2]) * s.n[2];
  const d1 = (cur[0] - s.o[0]) * s.n[0] + (cur[2] - s.o[2]) * s.n[2];
  if ((d0 < 0) === (d1 < 0)) return 0;
  const f = d0 / (d0 - d1);
  const x = prev[0] + (cur[0] - prev[0]) * f, y = prev[1] + (cur[1] - prev[1]) * f, z = prev[2] + (cur[2] - prev[2]) * f;
  const t = (x - s.o[0]) * s.t[0] + (z - s.o[2]) * s.t[2];
  if (Math.abs(t) > s.halfW || y < s.y0 || y > s.y1) return 0;
  return d1 > d0 ? 1 : -1;
}

export function insideTower(tw, p) {
  const q = yawRot([p[0] - tw.x, 0, p[2] - tw.z], -tw.yaw);
  return Math.abs(q[0]) < tw.half && Math.abs(q[2]) < tw.half && p[1] < tw.height;
}
