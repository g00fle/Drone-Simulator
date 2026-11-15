// 6-DOF rigid body quadcopter. State lives in the body-origin-at-centre-of-mass frame.
import { add, scale, cross, qRot, qRotInv, qMul, qNorm, len, clamp } from './math.js';

export const G = 9.81;

export function createQuad() {
  return {
    p: [0, 0, 0], v: [0, 0, 0], q: [0, 0, 0, 1], w: [0, 0, 0], // w = body angular velocity
    motors: [0, 0, 0, 0], // actual thrust fraction 0..1
    sag: 1, // share of maxThrust the battery can deliver right now
    ge: [1, 1, 1, 1], // per-rotor thrust gain from ground and ceiling effect (physics/realism.js)
    power: 1, // 1 = healthy battery, fades to 0 when the pack is empty
    thrust: 0, // total rotor thrust in newtons, last step
    usedWh: 0, usedAh: 0, soc: 1, volts: 0, amps: 0, watts: 0, // battery bookkeeping (physics/realism.js)
    payload: null, // { mass, r: [x,y,z] body-frame attachment point }
  };
}

export function totalMass(s, cfg) { return cfg.mass + (s.payload ? s.payload.mass : 0); }

// cmd: 4 thrust fractions 0..1. ext: { f: world force, t: world torque about the CoM }.
export function stepQuad(s, cfg, cmd, dt, ext) {
  const m = totalMass(s, cfg);
  const a = 1 - Math.exp(-dt / cfg.motorLag);
  let T = 0; const tau = [0, 0, 0];
  const fmax = cfg.maxThrust * s.sag;
  for (let i = 0; i < 4; i++) {
    s.motors[i] += (clamp(cmd[i], 0, 1) * s.power - s.motors[i]) * a;
    // Near the floor or a roof the same motor effort gives a little more thrust (s.ge). The reaction
    // torque that turns the drone in yaw follows the motor effort, not the extra lift.
    const fm = s.motors[i] * fmax, f = fm * s.ge[i], r = cfg.motorPos[i];
    T += f;
    tau[0] += -r[2] * f; tau[2] += r[0] * f;
    tau[1] += cfg.motorSpin[i] * cfg.yawTorquePerThrust * fm;
  }
  s.thrust = T;

  const speed = len(s.v);
  let F = qRot(s.q, [0, T, 0]);
  F[1] -= m * G;
  F = add(F, scale(s.v, -(cfg.dragLinear + cfg.dragQuad * speed)));
  if (ext) F = add(F, ext.f);

  const I = cfg.inertia.slice();
  if (s.payload) {
    const r = s.payload.r, pm = s.payload.mass;
    I[0] += pm * (r[1] * r[1] + r[2] * r[2]); I[1] += pm * (r[0] * r[0] + r[2] * r[2]); I[2] += pm * (r[0] * r[0] + r[1] * r[1]);
    const gt = cross(r, qRotInv(s.q, [0, -pm * G, 0])); // hanging weight pulls the body level
    tau[0] += gt[0]; tau[1] += gt[1]; tau[2] += gt[2];
  }
  if (ext) { const tb = qRotInv(s.q, ext.t); tau[0] += tb[0]; tau[1] += tb[1]; tau[2] += tb[2]; }
  const w = s.w, Iw = [I[0] * w[0], I[1] * w[1], I[2] * w[2]], gyro = cross(w, Iw);
  for (let i = 0; i < 3; i++) w[i] += ((tau[i] - gyro[i] - cfg.angularDrag * w[i]) / I[i]) * dt;

  s.v = add(s.v, scale(F, dt / m));
  s.p = add(s.p, scale(s.v, dt));
  const dq = qMul(s.q, [w[0], w[1], w[2], 0]);
  s.q = qNorm([s.q[0] + 0.5 * dq[0] * dt, s.q[1] + 0.5 * dq[1] * dt, s.q[2] + 0.5 * dq[2] * dt, s.q[3] + 0.5 * dq[3] * dt]);
  return s;
}

// Velocity of a body-fixed point (world frame), r_world = offset from the CoM in world axes.
export function pointVelocity(s, rWorld) {
  return add(s.v, cross(qRot(s.q, s.w), rWorld));
}
