// Sphere-vs-primitive contact tests and the penalty contact model.
// Each test returns null or { n: unit normal pointing out of the primitive toward the sphere, d: penetration depth }.
import { sub, dot, len, yawRot } from './math.js';

export function sphereFloor(c, r) {
  const d = r - c[1];
  return d > 0 ? { n: [0, 1, 0], d } : null;
}

export function sphereSphere(c, r, c2, r2) {
  const v = sub(c, c2), l = len(v), d = r + r2 - l;
  if (d <= 0) return null;
  return { n: l > 1e-9 ? [v[0] / l, v[1] / l, v[2] / l] : [0, 1, 0], d };
}

export function sphereCapsule(c, r, cap) {
  const ab = sub(cap.b, cap.a), ac = sub(c, cap.a);
  let t = dot(ac, ab) / dot(ab, ab);
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return sphereSphere(c, r, [cap.a[0] + ab[0] * t, cap.a[1] + ab[1] * t, cap.a[2] + ab[2] * t], cap.r);
}

// Box rotated about the vertical axis. Uses box.col (collision centre / half sizes) when present.
export function sphereBox(c, r, box) {
  const bc = box.col ? box.col.c : box.c, h = box.col ? box.col.h : box.h;
  const p = yawRot(sub(c, bc), -box.yaw);
  const q = [Math.max(-h[0], Math.min(h[0], p[0])), Math.max(-h[1], Math.min(h[1], p[1])), Math.max(-h[2], Math.min(h[2], p[2]))];
  const v = sub(p, q), l = len(v);
  let n, d;
  if (l > 1e-9) {
    d = r - l; if (d <= 0) return null;
    n = [v[0] / l, v[1] / l, v[2] / l];
  } else { // centre inside the box: leave by the nearest face
    let best = 0, bd = Infinity;
    for (let i = 0; i < 3; i++) { const e = h[i] - Math.abs(p[i]); if (e < bd) { bd = e; best = i; } }
    n = [0, 0, 0]; n[best] = p[best] >= 0 ? 1 : -1; d = bd + r;
  }
  return { n: yawRot(n, box.yaw), d };
}

// Upright cylinder: cyl.c = centre of the base, radius r, height h.
export function sphereCylinder(c, r, cyl) {
  const dx = c[0] - cyl.c[0], dz = c[2] - cyl.c[2], y = c[1] - cyl.c[1];
  const rad = Math.hypot(dx, dz);
  const qr = Math.min(rad, cyl.r), qy = Math.max(0, Math.min(cyl.h, y));
  const vr = rad - qr, vy = y - qy, l = Math.hypot(vr, vy);
  if (l > 1e-9) {
    const d = r - l; if (d <= 0) return null;
    const ux = rad > 1e-9 ? dx / rad : 1, uz = rad > 1e-9 ? dz / rad : 0;
    return { n: [(ux * vr) / l, vy / l, (uz * vr) / l], d };
  }
  const side = cyl.r - rad, top = cyl.h - y;
  if (top < side) return { n: [0, 1, 0], d: top + r };
  const ux = rad > 1e-9 ? dx / rad : 1, uz = rad > 1e-9 ? dz / rad : 0;
  return { n: [ux, 0, uz], d: side + r };
}

// Spring-damper normal force with Coulomb-limited tangential friction.
// vrel = velocity of the sphere's contact point relative to the surface.
export function contactForce(hit, vrel, k, c, mu, ct) {
  const vn = dot(vrel, hit.n);
  const fn = Math.max(0, k * hit.d - c * vn);
  const vt = [vrel[0] - hit.n[0] * vn, vrel[1] - hit.n[1] * vn, vrel[2] - hit.n[2] * vn];
  const vtl = len(vt);
  const ft = vtl > 1e-6 ? Math.min(mu * fn, ct * vtl) / vtl : 0;
  return [hit.n[0] * fn - vt[0] * ft, hit.n[1] * fn - vt[1] * ft, hit.n[2] * fn - vt[2] * ft];
}

// Bounding spheres for the broad phase.
export function addBounds(course) {
  for (const cap of course.capsules) {
    cap.bc = [(cap.a[0] + cap.b[0]) / 2, (cap.a[1] + cap.b[1]) / 2, (cap.a[2] + cap.b[2]) / 2];
    cap.br = len(sub(cap.a, cap.b)) / 2 + cap.r;
  }
  for (const b of course.boxes) { const h = b.col ? b.col.h : b.h; b.bc = b.col ? b.col.c : b.c; b.br = Math.hypot(h[0], h[1], h[2]); }
  for (const cy of course.cylinders) { cy.bc = [cy.c[0], cy.c[1] + cy.h / 2, cy.c[2]]; cy.br = Math.hypot(cy.r, cy.h / 2); }
  return course;
}

export function near(c, r, prim) {
  const dx = c[0] - prim.bc[0], dy = c[1] - prim.bc[1], dz = c[2] - prim.bc[2], R = r + prim.br;
  return dx * dx + dy * dy + dz * dz < R * R;
}

// Does the segment a -> b hit any box? Used for marker line-of-sight.
export function segmentBlocked(a, b, boxes, skip = 0.03) {
  const L = len(sub(b, a)), steps = Math.max(2, Math.ceil(L / 0.02));
  for (let i = 1; i < steps; i++) {
    const t = i / steps;
    if (t * L < skip || (1 - t) * L < skip) continue;
    const p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
    for (const box of boxes) if (sphereBox(p, 0.001, box)) return true;
  }
  return false;
}
