// three.js meshes for the field, obstacles, zones and loose objects.
import * as THREE from 'three';
import { FIELD, ZONES, BEACON, TOWER } from '../config/course.js';
import { PAYLOADS } from '../config/drone.js';
import { FT } from '../physics/math.js';

const UP = new THREE.Vector3(0, 1, 0);

function label(text, bg, fg = '#ffffff', size = 256) {
  const cv = document.createElement('canvas'); cv.width = cv.height = size;
  const g = cv.getContext('2d');
  g.fillStyle = bg; g.fillRect(0, 0, size, size);
  g.fillStyle = fg; g.textAlign = 'center'; g.textBaseline = 'middle';
  g.font = `bold ${text.length > 1 ? size * 0.3 : size * 0.7}px system-ui, sans-serif`;
  g.fillText(text, size / 2, size / 2 + size * 0.04);
  const t = new THREE.CanvasTexture(cv); t.colorSpace = THREE.SRGBColorSpace; t.anisotropy = 4;
  return t;
}

export function buildField() {
  const g = new THREE.Group();
  const W = FIELD.width, L = FIELD.length;
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(40, 40), new THREE.MeshStandardMaterial({ color: 0x8b8f96, roughness: 0.95 }));
  floor.rotation.x = -Math.PI / 2; floor.position.y = -0.002; floor.receiveShadow = true; g.add(floor);
  const field = new THREE.Mesh(new THREE.PlaneGeometry(W, L), new THREE.MeshStandardMaterial({ color: 0x4a5560, roughness: 0.9 }));
  field.rotation.x = -Math.PI / 2; field.receiveShadow = true; g.add(field);
  const pilot = new THREE.Mesh(new THREE.PlaneGeometry(W, FIELD.pilotDepth), new THREE.MeshStandardMaterial({ color: 0x6b6258, roughness: 0.95 }));
  pilot.rotation.x = -Math.PI / 2; pilot.position.set(0, 0, L / 2 + FIELD.pilotDepth / 2); pilot.receiveShadow = true; g.add(pilot);
  // one-foot grid and boundary tape
  const pts = [];
  for (let i = 0; i <= 10; i++) { const x = -W / 2 + i * FT; pts.push(x, 0.001, -L / 2, x, 0.001, L / 2); }
  for (let i = 0; i <= 20; i++) { const z = -L / 2 + i * FT; pts.push(-W / 2, 0.001, z, W / 2, 0.001, z); }
  const grid = new THREE.LineSegments(new THREE.BufferGeometry().setAttribute('position', new THREE.Float32BufferAttribute(pts, 3)),
    new THREE.LineBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.09 }));
  g.add(grid);
  const tape = new THREE.MeshBasicMaterial({ color: 0xf4f4f4 });
  for (const [w, d, x, z] of [[W + 0.05, 0.05, 0, -L / 2], [W + 0.05, 0.05, 0, L / 2], [0.05, L, -W / 2, 0], [0.05, L, W / 2, 0]]) {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(w, d), tape); m.rotation.x = -Math.PI / 2; m.position.set(x, 0.002, z); g.add(m);
  }
  return g;
}

export function buildCourseMeshes(course) {
  const g = new THREE.Group();
  const pvc = new THREE.MeshStandardMaterial({ color: 0xf2f2ee, roughness: 0.45 });
  const tints = { gate: 0xf2f2ee, corridor: 0xf2f2ee, tower: 0xf2f2ee, floorFrame: 0xf2f2ee };
  void tints;
  for (const c of course.capsules) {
    const a = new THREE.Vector3(...c.a), b = new THREE.Vector3(...c.b), d = b.clone().sub(a), len = d.length();
    const m = new THREE.Mesh(new THREE.CapsuleGeometry(c.r, len, 3, 10), pvc);
    m.position.copy(a).addScaledVector(d, 0.5); m.quaternion.setFromUnitVectors(UP, d.normalize());
    m.castShadow = true; m.receiveShadow = true; g.add(m);
  }
  const board = new THREE.MeshStandardMaterial({ color: 0xe9e4d8, roughness: 0.9, side: THREE.DoubleSide });
  const roofMat = new THREE.MeshStandardMaterial({ color: 0x1e9e50, roughness: 0.9 });
  for (const b of course.boxes) {
    const m = new THREE.Mesh(new THREE.BoxGeometry(b.h[0] * 2, b.h[1] * 2, b.h[2] * 2), b.kind === 'roof' ? roofMat : board);
    m.position.set(...b.c); m.rotation.y = b.yaw; m.castShadow = true; m.receiveShadow = true; g.add(m);
  }
  // roof treatment cross
  {
    const tw = course.tower;
    const m = new THREE.Mesh(new THREE.PlaneGeometry(TOWER.panel * 0.6, TOWER.panel * 0.6), new THREE.MeshBasicMaterial({ map: label('+', '#1e9e50'), transparent: false }));
    m.rotation.x = -Math.PI / 2; m.rotation.z = tw.yaw; m.position.set(tw.x, tw.roofTop + 0.001, tw.z); g.add(m);
  }
  const markers = {};
  for (const mk of course.markers) {
    const m = new THREE.Mesh(new THREE.PlaneGeometry(mk.size, mk.size), new THREE.MeshBasicMaterial({ map: label(mk.id, '#f2c80f', '#111111') }));
    m.position.set(mk.p[0] + mk.n[0] * 0.002, mk.p[1], mk.p[2] + mk.n[2] * 0.002);
    m.lookAt(mk.p[0] + mk.n[0], mk.p[1], mk.p[2] + mk.n[2]); g.add(m); markers[mk.id] = m;
  }
  for (const z of Object.values(course.zones)) {
    const tex = z.id === 'home' ? label('H', '#2f6fde') : z.id === 'treat' ? label('+', '#1e9e50') : null;
    const m = new THREE.Mesh(new THREE.PlaneGeometry(z.w, z.d), new THREE.MeshStandardMaterial({ color: tex ? 0xffffff : ZONES[z.id].color, map: tex, roughness: 0.8 }));
    m.rotation.x = -Math.PI / 2; m.rotation.z = z.yaw; m.position.set(z.x, 0.003, z.z); m.receiveShadow = true; g.add(m);
  }
  const beaconMat = new THREE.MeshStandardMaterial({ color: 0xc03ad6, emissive: 0x000000, roughness: 0.4 });
  const beacon = new THREE.Mesh(new THREE.CylinderGeometry(BEACON.radius, BEACON.radius * 1.03, BEACON.height, 32), beaconMat);
  beacon.position.set(course.beacon.c[0], BEACON.height / 2, course.beacon.c[2]); beacon.castShadow = true; g.add(beacon);
  const glow = new THREE.PointLight(0xff66ff, 0, 2.5); glow.position.set(course.beacon.c[0], 0.25, course.beacon.c[2]); g.add(glow);
  return { group: g, markers, setBeacon(on) { beaconMat.emissive.setHex(on ? 0xff55ff : 0x000000); glow.intensity = on ? 3 : 0; } };
}

const BALL_COLORS = [0xff7a1a, 0xffd21a, 0x3ad65a, 0x2aa7ff, 0xe8457a];

export function buildObjectMeshes(objects) {
  const g = new THREE.Group(), map = {};
  const steel = new THREE.MeshStandardMaterial({ color: 0x9aa0a6, metalness: 0.8, roughness: 0.35 });
  objects.forEach((o, i) => {
    let m;
    if (o.type === 'pod') {
      const P = PAYLOADS.pod; m = new THREE.Group();
      const body = new THREE.Mesh(new THREE.CylinderGeometry(P.radius / Math.cos(Math.PI / 8), P.radius / Math.cos(Math.PI / 8), P.height, 8), new THREE.MeshStandardMaterial({ color: 0xd93025, roughness: 0.6, flatShading: true }));
      body.position.y = P.height / 2; body.rotation.y = Math.PI / 8;
      const washer = new THREE.Mesh(new THREE.CylinderGeometry(0.019, 0.019, 0.003, 24), steel); washer.position.y = P.height + 0.0005;
      const cross = new THREE.Mesh(new THREE.PlaneGeometry(0.07, 0.07), new THREE.MeshBasicMaterial({ map: label('+', '#ffffff', '#d93025') }));
      cross.position.set(0, P.height / 2, P.radius + 0.001);
      m.add(body, washer, cross);
    } else if (o.type === 'figure') {
      const F = PAYLOADS.figure, bodyH = F.size[1] - F.ringOD; m = new THREE.Group();
      const body = new THREE.Mesh(new THREE.BoxGeometry(F.size[0], bodyH, F.size[2]), new THREE.MeshStandardMaterial({ color: 0xff8c1a, roughness: 0.6 }));
      body.position.y = bodyH / 2; m.add(body);
      const wire = (F.ringOD - F.ringID) / 4;
      for (const ry of [0, Math.PI / 2]) {
        const ring = new THREE.Mesh(new THREE.TorusGeometry(F.ringOD / 2 - wire, wire, 8, 28), new THREE.MeshStandardMaterial({ color: 0x222222, roughness: 0.5 }));
        ring.position.y = F.ringCenterY; ring.rotation.y = ry; m.add(ring);
      }
      m.rotation.y = o.yaw;
    } else {
      m = new THREE.Mesh(new THREE.SphereGeometry(o.spheres[0].r, 20, 14), new THREE.MeshStandardMaterial({ color: BALL_COLORS[i % 5], roughness: 0.9 }));
      m.geometry.translate(0, o.spheres[0].r, 0);
    }
    m.traverse((c) => { if (c.isMesh) { c.castShadow = true; c.receiveShadow = true; } });
    g.add(m); map[o.id] = m;
  });
  return {
    group: g, map,
    update(objs) {
      for (const o of objs) {
        const m = map[o.id]; m.position.set(o.p[0], o.p[1], o.p[2]);
        if (o.type === 'pod') { if (o.carried && o.q) m.quaternion.set(o.q[0], o.q[1], o.q[2], o.q[3]); else m.quaternion.identity(); }
      }
    },
  };
}
