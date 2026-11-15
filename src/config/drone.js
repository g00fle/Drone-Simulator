// Payload quadcopter numbers. Everything here is tunable.
// "Onshape" coordinates below mean the CAD frame: +X right, +Y front, +Z up, z = 0 at the
// underside of the bottom plate, in metres. cad() converts to the simulator body frame
// (x right, y up, z back, origin at the centre of mass).

export const CAD_COM = [0, 0.00066, 0.0230]; // measured: Assembly 1 Mass Properties
export const cad = (x, y, z) => [x - CAD_COM[0], z - CAD_COM[2], -(y - CAD_COM[1])];

const D = 0.1125 * Math.SQRT1_2; // motor offset along each axis (112.5 mm radius on the diagonals)
const LEG = 0.192 * Math.SQRT1_2;

function buildSpheres() {
  // The spheres overlap or sit closer together than a 1/2 in pipe is thick (21 mm), so no pipe or
  // panel edge can slip between them. Outer sizes match the CAD: guards 141 mm OD, feet at -130 mm.
  const s = [];
  for (const [sx, sy] of [[1, 1], [-1, 1], [-1, -1], [1, -1]]) {
    // Prop guard: 141 mm OD ring, z +24..+40.
    for (let i = 0; i < 20; i++) {
      const a = (i / 20) * Math.PI * 2;
      s.push({ p: cad(sx * D + 0.0625 * Math.cos(a), sy * D + 0.0625 * Math.sin(a), 0.032), r: 0.008, part: 'guard' });
    }
    // Leg: foot at -130 mm, then a chain up to the arm.
    for (let i = 0; i < 8; i++) s.push({ p: cad(sx * LEG, sy * LEG, -0.124 + i * 0.0165), r: 0.006, part: i ? 'leg' : 'foot' });
    // Arm, from the body out to the leg mount.
    for (const t of [0.25, 0.42, 0.6, 0.78, 0.95]) s.push({ p: cad(sx * LEG * t, sy * LEG * t, 0.008), r: 0.012, part: 'arm' });
  }
  // Top of the stack / battery: overall top is +80 mm.
  for (const [x, y] of [[0.02, 0.045], [-0.02, 0.045], [0.02, -0.045], [-0.02, -0.045], [0, 0.015], [0, -0.015]]) {
    s.push({ p: cad(x, y, 0.068), r: 0.012, part: 'top' });
  }
  s.push({ p: cad(0, 0.035, 0.03), r: 0.03, part: 'body' });
  s.push({ p: cad(0, 0, 0.02), r: 0.03, part: 'body' });
  s.push({ p: cad(0, -0.035, 0.03), r: 0.03, part: 'body' });
  s.push({ p: cad(0, 0, -0.008), r: 0.012, part: 'magnet' }); // magnet face at -20 mm
  // Hook: down from the plate, then the prong forward.
  for (const z of [-0.012, -0.03, -0.048, -0.066]) s.push({ p: cad(0, 0.098, z), r: 0.005, part: 'hook' });
  s.push({ p: cad(0, 0.1, -0.08), r: 0.004, part: 'hook' });
  s.push({ p: cad(0, 0.1125, -0.077), r: 0.004, part: 'hook' });
  s.push({ p: cad(0, 0.125, -0.074), r: 0.004, part: 'hook' });
  return s;
}

export const DRONE = {
  mass: 0.845, // kg, measured in Onshape (1.863 lb)
  // Moments of inertia about the centre of mass, kg m^2 (Onshape Lxx, Lzz, Lyy).
  inertia: [4.985e-3, 7.644e-3, 4.717e-3], // [pitch axis x, yaw axis y, roll axis z]
  motorPos: [ // body frame, metres. Order: front-right, front-left, rear-left, rear-right
    cad(D, D, 0.03), cad(-D, D, 0.03), cad(-D, -D, 0.03), cad(D, -D, 0.03),
  ],
  motorSpin: [1, -1, 1, -1], // sign of the yaw reaction torque about +y
  maxThrust: 1.3 * 9.81, // N per motor. DEFAULT (2207 / 5 in / 4S class): replace from the spec sheet
  yawTorquePerThrust: 0.016, // m: reaction torque per newton of thrust
  motorLag: 0.04, // s, spin-up time constant
  dragLinear: 0.18, // N per m/s
  dragQuad: 0.035, // N per (m/s)^2
  angularDrag: 0.004, // N m per rad/s
  // Battery. ESTIMATES until the real pack is flown and logged: the plan calls for a 4S LiPo of 1800 to 2200 mAh.
  battery: {
    cells: 4,
    packs: [1500, 1800, 2200], packmAh: 1800, // mAh choices in Settings, and the default
    hoverWatts: 150, // electrical power to hover the bare drone: 68 W ideal for four 5 in props, at about 45 % overall efficiency
    avionicsWatts: 6, // flight controller, receiver, camera and video transmitter
    magnetWatts: 4, // electromagnet while it is switched on
    // Resting volts per cell at 0, 10, 20 ... 100 % charge (typical LiPo curve).
    ocv: [3.27, 3.69, 3.73, 3.77, 3.80, 3.84, 3.87, 3.95, 4.02, 4.11, 4.20],
    rWiring: 0.008, // ohm: leads, connector and speed controllers
    rCellAh: 0.0108, // ohm x Ah per cell (a 1.8 Ah cell is 6 milliohm): bigger packs sag less
    vRef: 16.8, // pack voltage at which maxThrust applies
    lowAt: 0.25, criticalAt: 0.10, // charge left when the warnings fire
    fadeSeconds: 2.5, // an empty pack collapses over about this long
    sagLoad: 0.08, // battery simulation off: thrust lost at full throttle, and the pack never runs down
  },
  // Geometry used by game logic (body frame)
  magnetFace: cad(0, 0, -0.02),
  hookPoint: cad(0, 0.117, -0.078), // middle of the prong
  hookTip: cad(0, 0.125, -0.07),
  camera: { pos: cad(0, 0.066, 0.058), tiltUpDeg: 10, fovDeg: 120 },
  footZ: -0.130 - CAD_COM[2], // feet height in the body frame (y)
  topZ: 0.080 - CAD_COM[2],
  spheres: buildSpheres(),
  boundRadius: 0.24,
  // Flight controller
  fc: {
    maxAngleDeg: 30,
    rateKp: 18, rateKi: 60, yawKp: 9, angleKp: 6.5,
    acroRateDeg: [520, 300, 520], // max pitch, yaw, roll rate deg/s
    acroExpo: 0.45,
    yawRateDeg: 150, // angle mode yaw rate at full stick
    climbRate: 1.2, // m/s at full stick with altitude hold
    altKp: 5, altKi: 3,
  },
};

export const PAYLOADS = {
  pod: { mass: 0.225, radius: 0.0635, height: 0.127 },
  figure: { mass: 0.045, size: [0.0889, 0.127, 0.1016], ringOD: 0.0635, ringID: 0.04445, ringCenterY: 0.095 },
  ball: { mass: 0.012, radius: 0.0345 },
};
