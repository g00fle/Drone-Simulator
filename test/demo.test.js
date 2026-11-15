import test from 'node:test';
import assert from 'node:assert/strict';
import { createSim, step, DT, Rules } from '../src/sim.js';
import { DEFAULT_LAYOUT } from '../src/config/course.js';
import { createAutopilot, demoScript } from '../src/demo/autopilot.js';

test('autopilot flies the full run and the score sheet adds up to 120', () => {
  const sim = createSim(DEFAULT_LAYOUT);
  const ap = createAutopilot(sim, demoScript(sim));
  let lastLabel = '', n = 0;
  while (!ap.done && sim.t < 590) {
    step(sim, ap.sticks(DT));
    if (process.env.TRACE && (ap.label !== lastLabel || ++n % 2500 === 0)) {
      lastLabel = ap.label;
      console.log(sim.t.toFixed(1), ap.label, 'step', ap.i, sim.quad.p.map((v) => v.toFixed(2)).join(','), 'score', Rules.total(sim.rules), sim.crashed ? 'CRASHED ' + sim.crashReason : '');
    }
  }
  const r = sim.rules;
  if (process.env.TRACE) for (const l of r.log) console.log(l.t.toFixed(1), l.text);
  assert.ok(!sim.crashed, 'no crash');
  assert.deepEqual(r.score, { m1A: 5, m1B: 5, m1C: 10, m1Exit: 5, m2Lift: 10, m2Place: 10, m3Roof: 15, m3Treat: 15, m3Bonus: 5, m4Path: 10, m4Pods: 0, m5Path: 10, m5Beacon: 10 });
  assert.equal(r.flightComplete, 10);
  assert.equal(Rules.total(r), 120);
  assert.ok(r.finishTime < 600);
});
