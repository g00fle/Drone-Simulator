// Flight controller: sticks -> 4 motor commands. Betaflight-style Angle and Acro modes,
// plus an altitude-hold assist that turns the throttle stick into a climb-rate command.
import { qRot, qRotInv, norm, clamp } from '../physics/math.js';
import { G } from '../physics/quad.js';

const rad = (d) => (d * Math.PI) / 180;

export function createFC() {
  return { mode: 'angle', altHold: true, altI: 0, rateI: [0, 0, 0], armed: true };
}

// sticks: { throttle: -1..1 (alt hold: climb rate; otherwise 0 = hover thrust, -1 = off, +1 = full), pitch: +1 = nose down / forward,
//           roll: +1 = right, yaw: +1 = clockwise seen from above }
// touching = some part of the drone is pressed against an obstacle; capped = it is up against the ceiling net.
export function fcUpdate(fc, s, cfg, sticks, dt, grounded, touching = false, capped = false) {
  const k = cfg.fc, I = cfg.inertia;
  if (!fc.armed) return [0, 0, 0, 0];
  const up = qRot(s.q, [0, 1, 0]);
  let rateSp; // body rates [x, y, z]
  if (fc.mode === 'acro') {
    const ex = (v) => v * (1 - k.acroExpo) + v * v * v * k.acroExpo;
    rateSp = [-ex(sticks.pitch) * rad(k.acroRateDeg[0]), -ex(sticks.yaw) * rad(k.acroRateDeg[1]), -ex(sticks.roll) * rad(k.acroRateDeg[2])];
  } else {
    // Desired "up" direction from stick angles, in the heading frame.
    const nose = qRot(s.q, [0, 0, -1]);
    let f = norm([nose[0], 0, nose[2]]);
    if (f[0] === 0 && f[2] === 0) f = [0, 0, -1];
    const r = [-f[2], 0, f[0]];
    const tp = Math.tan(rad(k.maxAngleDeg) * clamp(sticks.pitch, -1, 1)), tr = Math.tan(rad(k.maxAngleDeg) * clamp(sticks.roll, -1, 1));
    const want = norm([f[0] * tp + r[0] * tr, 1, f[2] * tp + r[2] * tr]);
    const d = qRotInv(s.q, want); // desired up in the body frame; error axis = (0,1,0) x d
    let ex = d[2], ez = -d[0];
    if (d[1] < 0) { const l = Math.hypot(ex, ez) || 1; ex /= l; ez /= l; } // upside down: full-strength correction
    rateSp = [k.angleKp * ex, -sticks.yaw * rad(k.yawRateDeg), k.angleKp * ez];
  }
  // Rate loop with an integral term, so an off-centre load (figure on the hook) does not leave a steady tilt.
  const e = [rateSp[0] - s.w[0], rateSp[1] - s.w[1], rateSp[2] - s.w[2]];
  // The integral must not wind up against something the motors cannot move (a pole, a wall): it is held
  // to a tighter limit while the drone is touching, so it does not fling the drone when the contact lets go.
  if (grounded) fc.rateI = [0, 0, 0];
  else for (let i = 0; i < 3; i++) { const lim = i === 1 ? 40 : touching ? 25 : 40; fc.rateI[i] = clamp(fc.rateI[i] + e[i] * k.rateKi * dt, -lim, lim); }
  const tau = [
    I[0] * (k.rateKp * e[0] + fc.rateI[0]),
    I[1] * (k.yawKp * e[1] + fc.rateI[1]),
    I[2] * (k.rateKp * e[2] + fc.rateI[2]),
  ];

  const fmax = cfg.maxThrust * s.sag;
  let T;
  if (fc.altHold) {
    const vSp = sticks.throttle * k.climbRate;
    if (grounded && sticks.throttle <= 0.02) { fc.altI = 0; T = 0.25 * cfg.mass * G; }
    else {
      const err = vSp - s.v[1];
      if (!touching && !capped) fc.altI = clamp(fc.altI + err * k.altKi * dt, -6, 6);
      // Extra power to make up for tilt, but not while leaning on something: that would only push harder into it.
      T = (cfg.mass * (G + k.altKp * err + fc.altI)) / (touching ? 1 : clamp(up[1], 0.5, 1));
      // Knocked well past the steepest angle the sticks can ask for: hold the power near hover until the drone
      // is upright again, so catching a fall cannot turn into a sideways dash.
      if (up[1] < 0.85) T = Math.min(T, cfg.mass * G * (1 + 2 * clamp((up[1] - 0.5) / 0.35, 0, 1)));
    }
  } else {
    // Manual throttle: a centred stick (or no key held) gives hover thrust, so switching the assist off
    // does not launch the drone. Below centre runs down to zero, above centre up to full power.
    const t = clamp(sticks.throttle, -1, 1), hover = Math.min(cfg.mass * G, 4 * fmax);
    if (grounded && t <= 0.02) T = 0.25 * cfg.mass * G;
    else T = t >= 0 ? hover + t * (4 * fmax - hover) : hover * (1 + t);
  }
  T = clamp(T, 0, 4 * fmax);
  // Self-levelling modes ease off the power when knocked far over, so thrust does not drive the drone sideways
  // or into the floor while it rights itself. Acro is left alone: flips are the pilot's business there.
  if (fc.mode !== 'acro' && up[1] < 0.5) T *= Math.max(0.15, up[1] / 0.5);
  // Yaw is the weakest axis (it only has the props' reaction torque). Cap it so a yaw demand can never
  // ask for more than a share of the current thrust.
  const yawMax = cfg.yawTorquePerThrust * 1.0 * Math.max(T, 0.5 * cfg.mass * G);
  tau[1] = clamp(tau[1], -yawMax, yawMax);
  // Free in the air, levelling may borrow power up to 1.6x hover to right the drone quickly. Pressed against
  // something it may not: that is what used to drive the drone up a pole.
  return mix(cfg, T, tau, fmax, fc.mode === 'acro' || touching || grounded ? 0.6 : 1.6);
}

// Solve the four motor thrusts for a total thrust and body torques. When a motor would saturate, levelling
// (roll and pitch) is served first and yaw gets what is left, so a spin can never cost the drone its balance.
// Total thrust is only raised above what was asked for when levelling needs it, and then no further than
// `floor` times the hover thrust.
export function mix(cfg, T, tau, fmax, floor = 0.6) {
  const rp = [0, 0, 0, 0], yw = [0, 0, 0, 0];
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i < 4; i++) {
    const r = cfg.motorPos[i], a = 4 * r[0] * r[0];
    rp[i] = (tau[0] * -r[2]) / a + (tau[2] * r[0]) / a;
    yw[i] = (tau[1] * cfg.motorSpin[i]) / (4 * cfg.yawTorquePerThrust);
    lo = Math.min(lo, rp[i]); hi = Math.max(hi, rp[i]);
  }
  const idle = Math.min(fmax, (floor * cfg.mass * G) / 4);
  let base = clamp(T / 4, 0, fmax);
  if (-lo > base) base = Math.min(-lo, Math.max(base, idle));
  let s = 1;
  if (hi > 1e-9) s = Math.min(s, (fmax - base) / hi);
  if (lo < -1e-9) s = Math.min(s, base / -lo);
  s = Math.max(0, s);
  let sy = 1;
  for (let i = 0; i < 4; i++) {
    const f0 = base + s * rp[i];
    if (yw[i] > 1e-9) sy = Math.min(sy, (fmax - f0) / yw[i]);
    else if (yw[i] < -1e-9) sy = Math.min(sy, f0 / -yw[i]);
  }
  sy = Math.max(0, sy);
  return rp.map((x, i) => clamp((base + s * x + sy * yw[i]) / fmax, 0, 1));
}
