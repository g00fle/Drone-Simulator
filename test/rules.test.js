// Score sheet rules from the brief: declare, cancel, repeat, bonus order, one passage, clock, Flight Complete.
import test from 'node:test';
import assert from 'node:assert/strict';
import * as R from '../src/missions/rules.js';

test('must be at Home Base to declare, and the clock starts on the first declaration', () => {
  const r = R.createRules();
  assert.equal(R.declare(r, 1, false), false);
  assert.equal(r.started, false);
  R.tick(r, 5); assert.equal(r.time, 0);
  assert.equal(R.declare(r, 1, true), true);
  R.tick(r, 5); assert.equal(r.time, 5);
  assert.equal(R.declare(r, 2, true), false, 'only one mission at a time');
});

test('a canceled mission keeps its points and can be declared again; a completed one cannot', () => {
  const r = R.createRules();
  R.declare(r, 1, true);
  R.event(r, 'marker', { id: 'A', points: 5 });
  assert.equal(r.score.m1A, 0, 'markers do not count before entering by the entrance');
  R.event(r, 'tower_enter'); R.event(r, 'marker', { id: 'A', points: 5 });
  R.cancel(r);
  assert.equal(R.total(r), 5); assert.equal(r.status[1], 'canceled');
  assert.equal(R.declare(r, 1, true), true);
  R.event(r, 'tower_enter'); R.event(r, 'marker', { id: 'A', points: 5 }); R.event(r, 'marker', { id: 'C', points: 10 });
  R.event(r, 'tower_exit');
  assert.equal(R.missionTotal(r, 1), 20); assert.equal(r.status[1], 'complete'); assert.equal(r.active, 0);
  assert.equal(R.declare(r, 1, true), false);
});

test('events for a mission that is not active score nothing', () => {
  const r = R.createRules();
  R.declare(r, 2, true);
  R.event(r, 'beacon'); R.event(r, 'gate', { id: 'high', points: 10, label: 'High' }); R.event(r, 'pod_rest', { where: 'roof' });
  assert.equal(R.total(r), 0);
});

test('rescue: 10 for the lift, a release outside the Safe Zone ends the attempt', () => {
  const r = R.createRules();
  R.declare(r, 2, true); R.event(r, 'figure_lift'); R.event(r, 'figure_release', { inSafe: false });
  assert.equal(R.missionTotal(r, 2), 10); assert.equal(r.status[2], 'complete');
  const s = R.createRules();
  R.declare(s, 2, true); R.event(s, 'figure_lift'); R.event(s, 'figure_release', { inSafe: true });
  assert.equal(R.missionTotal(s, 2), 20);
});

test('operational bonus needs Mission 1 complete before the roof delivery', () => {
  const late = R.createRules();
  R.declare(late, 3, true); R.event(late, 'pod_rest', { where: 'roof' }); R.event(late, 'pod_rest', { where: 'treat' });
  assert.equal(R.missionTotal(late, 3), 30); assert.equal(late.status[3], 'complete');
  const early = R.createRules();
  R.declare(early, 1, true); R.event(early, 'tower_enter'); R.event(early, 'tower_exit');
  R.declare(early, 3, true); R.event(early, 'pod_rest', { where: 'treat' }); R.event(early, 'pod_rest', { where: 'roof' });
  assert.equal(R.missionTotal(early, 3), 35);
});

test('hazard: only the first passage counts, pods cap at 10 and freeze when the mission ends', () => {
  const r = R.createRules();
  R.declare(r, 4, true);
  R.event(r, 'gate', { id: 'low', points: 2, label: 'Low' }); R.event(r, 'gate', { id: 'high', points: 10, label: 'High' });
  assert.equal(r.score.m4Path, 2);
  R.event(r, 'balls', { count: 4 }); R.event(r, 'balls', { count: 3 });
  assert.equal(r.score.m4Pods, 3, 'a ball that rolls out stops counting while the mission is active');
  R.completeMission(r); R.event(r, 'balls', { count: 0 });
  assert.equal(R.missionTotal(r, 4), 5);
  const full = R.createRules();
  R.declare(full, 4, true); R.event(full, 'balls', { count: 12 });
  assert.equal(full.score.m4Pods, 10); assert.equal(full.status[4], 'complete');
});

test('corridor: weave all three bays at one level; floor contact restarts; first completed pathway only', () => {
  const r = R.createRules({ corridorMode: 'weave' });
  R.declare(r, 5, true);
  const cross = (bay, dir, level = 'lower') => R.event(r, 'corridor_cross', { bay, dir, level });
  cross(0, 1); cross(1, -1); R.event(r, 'corridor_ground');
  cross(2, 1); assert.equal(r.score.m5Path, 0, 'restart after touching the floor');
  cross(1, -1); cross(0, 1);
  assert.equal(r.score.m5Path, 10);
  cross(0, 1, 'upper'); cross(1, -1, 'upper'); cross(2, 1, 'upper');
  assert.equal(r.score.m5Path, 10);
  R.event(r, 'beacon');
  assert.equal(R.missionTotal(r, 5), 20); assert.equal(r.status[5], 'complete');
  const s = R.createRules({ corridorMode: 'single' });
  R.declare(s, 5, true); R.event(s, 'corridor_cross', { bay: 1, dir: 1, level: 'upper' });
  assert.equal(s.score.m5Path, 5);
});

test('Flight Complete needs Home Base and time on the clock; expiry ends the run', () => {
  const r = R.createRules();
  assert.equal(R.flightComplete(r, true), false, 'clock not started');
  R.declare(r, 2, true); R.event(r, 'figure_lift'); R.tick(r, 100);
  assert.equal(R.flightComplete(r, false), false);
  assert.equal(R.flightComplete(r, true), true);
  assert.equal(R.total(r), 20); assert.equal(r.finishTime, 100); assert.equal(r.over, true);
  assert.equal(R.declare(r, 3, true), false);

  const t = R.createRules();
  R.declare(t, 2, true); R.event(t, 'figure_lift'); R.tick(t, 601);
  assert.equal(t.over, true); assert.equal(R.timeLeft(t), 0);
  assert.equal(R.flightComplete(t, true), false);
  R.event(t, 'figure_release', { inSafe: true });
  assert.equal(R.total(t), 10);
});
