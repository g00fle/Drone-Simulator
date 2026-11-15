// Course dimensions from the 2027 "Operation Lifeline" mission brief (pages 8-13).
// Sizes are exact per the brief's cut lists; the default layout is traced from the page 8
// top view, which the brief says is not to scale. Use the layout editor (L) to move things.
import { IN, FT } from '../physics/math.js';

export const PIPE_R = 0.02134 / 2; // 1/2 in schedule 40 PVC, 21.3 mm OD

export const FIELD = {
  width: 10 * FT, // x
  length: 20 * FT, // z; pilot stands beyond +z
  pilotDepth: 4 * FT,
  pilotEye: [0, 1.6, 10 * FT + 0.9],
  ceiling: 3.2,
  margin: 1.2, // invisible safety net this far outside the field lines
};

export const TOWER = {
  cube: 24 * IN, levels: 2, panel: 23.5 * IN, panelT: 0.005,
  // ASSUMPTIONS (confirm with the Event Coordinator): which sides are open, where markers sit.
  lowerOpen: 'front', // ground-level entrance faces the pilot
  upperOpen: 'right', // upper-level exit
  markers: [
    { id: 'A', wall: 'back', level: 0, points: 5 },
    { id: 'B', wall: 'left', level: 0, points: 5 },
    { id: 'C', wall: 'back', level: 1, points: 10 },
  ],
};

export const GATE = {
  width: 24 * IN, foot: 8 * IN,
  // bottom to top
  openings: [
    { id: 'low', label: 'Low Risk', y0: 0, y1: 30 * IN, points: 2 },
    { id: 'moderate', label: 'Moderate Risk', y0: 30 * IN, y1: 48 * IN, points: 5 },
    { id: 'high', label: 'High Risk', y0: 48 * IN, y1: 60 * IN, points: 10 },
  ],
};

export const FLOOR_FRAME = { length: 58 * IN, width: 18 * IN, stub: 12 * IN, stubAt: 40 * IN, balls: 10 };

export const CORRIDOR = {
  bays: 3, bayWidth: 20 * IN, lower: 14 * IN, height: 44 * IN, foot: 9 * IN,
  mode: 'weave', // 'weave' = all three bays at one level; 'single' = one pass through any bay
  levels: [
    { id: 'lower', label: 'Lower Pathway', points: 10 },
    { id: 'upper', label: 'Upper Pathway', points: 5 },
  ],
};

export const BEACON = { radius: 0.07, height: 0.051, pressForce: 1.0 };

export const ZONES = {
  home: { w: 24 * IN, d: 24 * IN, color: 0x2f6fde, label: 'Home Base' },
  safe: { w: 14 * IN, d: 14 * IN, color: 0xf2c80f, label: 'Safe Zone' },
  contain: { w: 24 * IN, d: 24 * IN, color: 0xd93025, label: 'Chemical Containment Zone' },
  treat: { w: 18 * IN, d: 18 * IN, color: 0x1e9e50, label: 'Treatment Location' },
};

// x across the field (0 = centre line), z along it (0 = middle, + toward the pilot). rot in degrees.
export const DEFAULT_LAYOUT = {
  name: 'Brief page 8',
  items: {
    home: { x: 0, z: 2.62, rot: 0 },
    safe: { x: 1.05, z: 2.45, rot: 0 },
    contain: { x: -1.05, z: 2.4, rot: 0 },
    pod1: { x: -0.3, z: 1.85, rot: 0 },
    pod2: { x: 0.3, z: 1.85, rot: 0 },
    gate: { x: -0.95, z: 1.0, rot: 0 },
    corridor: { x: 0.62, z: 1.0, rot: 0 },
    tower: { x: 0, z: -0.55, rot: 0 },
    figure: { x: 0.75, z: -1.6, rot: 0 },
    floorFrame: { x: -0.65, z: -2.2, rot: 0 },
    treat: { x: 1.0, z: -2.2, rot: 0 },
    beacon: { x: 0.35, z: -2.75, rot: 0 },
  },
};

export const ITEM_LABELS = {
  home: 'Home Base', safe: 'Safe Zone', contain: 'Containment Zone', pod1: 'Medical Pod 1', pod2: 'Medical Pod 2',
  gate: 'Hazard Passage', corridor: 'Communication Corridor', tower: 'Medical Tower', figure: 'Rescue Figure',
  floorFrame: 'Hazmat floor frame', treat: 'Treatment Location', beacon: 'Beacon',
};
