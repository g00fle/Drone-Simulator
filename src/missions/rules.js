// Mission state machine and score sheet for "Operation Lifeline". Pure logic: feed it events.
import { CORRIDOR } from '../config/course.js';

export const RUN_SECONDS = 600;

export const MISSIONS = [
  { n: 1, name: 'Tower Inspection', items: [['m1A', 'Marker A', 5], ['m1B', 'Marker B', 5], ['m1C', 'Marker C', 10], ['m1Exit', 'Exit by upper opening', 5]] },
  { n: 2, name: 'Rescue Recovery', items: [['m2Lift', 'Figure lifted', 10], ['m2Place', 'Placed in Safe Zone', 10]] },
  { n: 3, name: 'Medical Delivery', items: [['m3Roof', 'Pod on tower roof', 15], ['m3Treat', 'Pod at treatment location', 15], ['m3Bonus', 'Operational Bonus', 5]] },
  { n: 4, name: 'Hazard Mitigation', items: [['m4Path', 'Hazard passage', 10], ['m4Pods', 'Hazmat pods contained', 10]] },
  { n: 5, name: 'Beacon Activation', items: [['m5Path', 'Corridor pathway', 10], ['m5Beacon', 'Beacon activated', 10]] },
];
export const FLIGHT_COMPLETE_POINTS = 10;

export function createRules(opts = {}) {
  const r = {
    corridorMode: opts.corridorMode || CORRIDOR.mode,
    time: 0, started: false, over: false, overReason: '', finishTime: null,
    active: 0, status: { 1: 'open', 2: 'open', 3: 'open', 4: 'open', 5: 'open' }, // open | active | canceled | complete
    score: {}, notes: {}, flightComplete: 0,
    m1: { entered: false }, m4: { passage: null }, m5: { level: null, bays: [], lastDir: 0, done: false },
    log: [],
  };
  for (const m of MISSIONS) for (const [id] of m.items) r.score[id] = 0;
  return r;
}

const say = (r, text, kind = 'info') => { r.log.push({ t: r.time, text, kind }); if (r.log.length > 60) r.log.shift(); };
const award = (r, id, pts, text) => {
  if (r.score[id] >= pts) return false;
  r.score[id] = pts; say(r, `+${pts}  ${text}`, 'score'); return true;
};

export function total(r) { return Object.values(r.score).reduce((a, b) => a + b, 0) + r.flightComplete; }
export function missionTotal(r, n) { return MISSIONS[n - 1].items.reduce((a, [id]) => a + r.score[id], 0); }
export function timeLeft(r) { return Math.max(0, RUN_SECONDS - r.time); }

export function tick(r, dt) {
  if (!r.started || r.over) return;
  r.time += dt;
  if (r.time >= RUN_SECONDS) {
    r.time = RUN_SECONDS;
    if (r.active) { r.status[r.active] = 'complete'; r.active = 0; }
    r.over = true; r.overReason = 'Time expired'; r.finishTime = RUN_SECONDS;
    say(r, 'Time expired. Run over.', 'warn');
  }
}

export function declare(r, n, atHome) {
  if (r.over) return fail(r, 'The run is over. Press N for a new run.');
  if (r.active) return fail(r, `Mission ${r.active} is still active. Complete (Enter) or cancel (C) it first.`);
  if (r.status[n] === 'complete') return fail(r, `Mission ${n} is already complete and cannot be repeated.`);
  if (!atHome) return fail(r, 'Return to Home Base before declaring a mission.');
  if (!r.started) { r.started = true; say(r, 'Clock started: 10:00'); }
  r.active = n; r.status[n] = 'active';
  if (n === 1) r.m1.entered = false;
  if (n === 5) r.m5 = { level: null, bays: [], lastDir: 0, done: r.m5.done };
  say(r, `Mission ${n} declared: ${MISSIONS[n - 1].name}`, 'declare');
  return true;
}
function fail(r, text) { say(r, text, 'warn'); return false; }

export function cancel(r) {
  if (!r.active || r.over) return false;
  say(r, `Mission ${r.active} canceled. Points earned so far stay.`, 'warn');
  r.status[r.active] = 'canceled'; r.active = 0; return true;
}

export function completeMission(r, why = '') {
  if (!r.active || r.over) return false;
  say(r, `Mission ${r.active} complete${why ? ': ' + why : ''} (${missionTotal(r, r.active)} pts)`, 'declare');
  r.status[r.active] = 'complete'; r.active = 0; return true;
}

export function flightComplete(r, atHome) {
  if (r.over) return false;
  if (!r.started) return fail(r, 'Declare a mission first to start the clock.');
  if (!atHome) return fail(r, 'Land on Home Base before declaring Flight Complete.');
  if (r.active) completeMission(r);
  r.flightComplete = FLIGHT_COMPLETE_POINTS; r.over = true; r.overReason = 'Flight Complete'; r.finishTime = r.time;
  say(r, `+${FLIGHT_COMPLETE_POINTS}  Flight Complete with ${fmt(timeLeft(r))} left`, 'score');
  return true;
}

export const fmt = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

// Events from the sensors. Each is ignored unless its mission is the active one.
export function event(r, type, d = {}) {
  if (r.over || !r.active) return;
  const a = r.active;
  switch (type) {
    case 'tower_enter': if (a === 1 && !r.m1.entered) { r.m1.entered = true; say(r, 'Entered the tower by the ground-level entrance'); } break;
    case 'marker': if (a === 1 && r.m1.entered) award(r, `m1${d.id}`, d.points, `Marker ${d.id} inspected`); break;
    case 'tower_exit':
      if (a === 1 && r.m1.entered) { award(r, 'm1Exit', 5, 'Left by the upper-level opening'); completeMission(r); }
      break;
    case 'tower_leave_low': // left again through the entrance: the inspection attempt is over
      if (a === 1 && r.m1.entered) { r.m1.entered = false; say(r, 'Left by the entrance. Re-enter to keep inspecting.', 'warn'); }
      break;
    case 'figure_lift': if (a === 2) award(r, 'm2Lift', 10, 'Rescue Figure lifted'); break;
    case 'figure_release':
      if (a === 2) {
        if (d.inSafe) { award(r, 'm2Place', 10, 'Figure placed fully inside the Safe Zone'); completeMission(r); }
        else completeMission(r, 'figure released outside the Safe Zone');
      }
      break;
    case 'pod_rest':
      if (a !== 3) break;
      if (d.where === 'roof' && award(r, 'm3Roof', 15, 'Pod delivered to the tower roof')) {
        if (r.status[1] === 'complete') award(r, 'm3Bonus', 5, 'Operational Bonus: tower inspected before roof delivery');
      }
      if (d.where === 'treat') award(r, 'm3Treat', 15, 'Pod delivered to the treatment location');
      if (r.score.m3Roof && r.score.m3Treat) completeMission(r);
      break;
    case 'gate':
      if (a === 4 && !r.m4.passage) { r.m4.passage = d.id; award(r, 'm4Path', d.points, `${d.label} passage`); }
      break;
    case 'balls':
      if (a === 4) {
        const n = Math.min(10, d.count);
        if (n !== r.score.m4Pods) { r.score.m4Pods = n; say(r, `Hazmat pods in the Containment Zone: ${n}`, 'score'); }
        if (n >= 10) completeMission(r);
      }
      break;
    case 'corridor_cross': if (a === 5) corridor(r, d); break;
    case 'corridor_ground':
      if (a === 5 && r.m5.bays.length && !r.m5.done) { r.m5.bays = []; r.m5.level = null; say(r, 'Touched the floor in the corridor. Restart the pathway.', 'warn'); }
      break;
    case 'beacon': if (a === 5) { award(r, 'm5Beacon', 10, 'Beacon activated'); completeMission(r); } break;
  }
}

function corridor(r, d) {
  const m = r.m5; if (m.done) return;
  const lv = CORRIDOR.levels.find((l) => l.id === d.level);
  const finish = () => { m.done = true; award(r, 'm5Path', lv.points, `${lv.label} completed`); };
  if (r.corridorMode === 'single') return finish();
  const last = m.bays[m.bays.length - 1];
  const ok = m.bays.length && m.level === d.level && Math.abs(d.bay - last) === 1 && !m.bays.includes(d.bay) && d.dir === -m.lastDir;
  if (ok) m.bays.push(d.bay); else { m.bays = [d.bay]; m.level = d.level; }
  m.lastDir = d.dir;
  if (m.bays.length >= CORRIDOR.bays) finish();
  else say(r, `${lv.label}: bay ${m.bays.length} of ${CORRIDOR.bays}`);
}
