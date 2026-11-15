// Browser entry: scene, fixed-step loop, views, actions.
import * as THREE from 'three';
import { createSim, step, DT, resetDrone, toggleMagnet, Rules } from './sim.js';
import { DEFAULT_LAYOUT, FIELD } from './config/course.js';
import { DRONE } from './config/drone.js';
import { qRot } from './physics/math.js';
import { FX_SEED } from './physics/realism.js';
import { insideTower } from './course/geometry.js';
import { buildField, buildCourseMeshes, buildObjectMeshes } from './course/build.js';
import { createDroneVisual } from './drone/model.js';
import { createInput } from './control/input.js';
import { createCameras, VIEWS } from './view/cameras.js';
import { createHud } from './view/hud.js';
import { LINK_TYPES, linkQuality, createLink, stepLink, createPoseDelay } from './view/link.js';
import { createFpvFx } from './view/fpvfx.js';
import { createEditor, loadLayout, cleanLayout } from './course/editor.js';
import { createAutopilot, demoScript } from './demo/autopilot.js';

const app = document.getElementById('app'), canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xb4bec8);
scene.fog = new THREE.Fog(0xb4bec8, 14, 34);
scene.add(new THREE.HemisphereLight(0xffffff, 0x77736c, 1.9));
const sun = new THREE.DirectionalLight(0xfff4e2, 2.4);
sun.position.set(3, 9, 5); sun.target.position.set(0, 0, 0.5); sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048); sun.shadow.bias = -0.0004; sun.shadow.normalBias = 0.01;
Object.assign(sun.shadow.camera, { left: -4.5, right: 4.5, top: 5, bottom: -5, near: 1, far: 22 });
scene.add(sun, sun.target, buildField());

const drone = createDroneVisual();
scene.add(drone.group);
const cams = createCameras(drone.group);
scene.add(cams.los, cams.chase, cams.top);

// ---- state ----
// The four realism switches: fpvLink is drawn here (view/link.js, view/fpvfx.js); ground, wash and battery
// are physics (physics/realism.js) and are handed to the sim.
const REALISM = ['fpvLink', 'ground', 'wash', 'battery'];
const settings = {
  altHold: true, gripper: false, corridorMode: 'weave', mode: 'angle',
  fpvLink: true, linkType: 'analog', ground: true, wash: true, battery: true, packmAh: DRONE.battery.packmAh,
};
try { Object.assign(settings, JSON.parse(localStorage.getItem('lifeline.settings') || '{}')); } catch { /* storage unavailable */ }
// stored values may be stale or hand-edited: keep only what the sim understands
for (const k of REALISM) settings[k] = !!settings[k];
settings.packmAh = Number(settings.packmAh);
if (!DRONE.battery.packs.includes(settings.packmAh)) settings.packmAh = DRONE.battery.packmAh;
if (!LINK_TYPES.includes(settings.linkType)) settings.linkType = LINK_TYPES[0];
const saveSettings = () => { try { localStorage.setItem('lifeline.settings', JSON.stringify(settings)); } catch { /* storage unavailable */ } };
let layout = loadLayout();
let sim, courseVis, objVis, ap = null, speed = 1, view = 'chase', paused = false;
const poseDelay = createPoseDelay(); // recent drone poses, so the FPV camera can show a moment ago
let link = createLink();
const fpvFx = createFpvFx(document.getElementById('fpvFx'), canvas);
const UP = [0, 1, 0];

function dispose(group) {
  group.traverse((o) => {
    o.geometry?.dispose();
    for (const m of [].concat(o.material || [])) { m.map?.dispose(); m.dispose(); }
  });
  scene.remove(group);
}

// seed fixes the random air inside the tower; every ordinary run gets its own.
function newRun(useLayout = layout, seed = Math.floor(Math.random() * 2 ** 31)) {
  if (courseVis) { dispose(courseVis.group); dispose(objVis.group); }
  sim = createSim(useLayout, {
    gripper: settings.gripper, corridorMode: settings.corridorMode, seed,
    realism: { ground: settings.ground, wash: settings.wash, battery: settings.battery, packmAh: settings.packmAh },
  });
  sim.fc.mode = settings.mode; sim.fc.altHold = settings.altHold;
  courseVis = buildCourseMeshes(sim.course); objVis = buildObjectMeshes(sim.objects);
  scene.add(courseVis.group, objVis.group);
  paused = false; acc = 0;
  poseDelay.clear(); link = createLink();
  window.sim = sim; window.simView = { link, group: drone.group }; // handy in the console
}

function startDemo() {
  newRun(cleanLayout(DEFAULT_LAYOUT), FX_SEED); // the script is written for the brief layout, and tested with this tower air
  sim.fc.mode = 'angle'; sim.fc.altHold = true;
  ap = createAutopilot(sim, demoScript(sim)); speed = 1;
  if (view === 'fpv') view = 'chase';
  hud.toggleHelp(false);
}
function stopDemo() { ap = null; speed = 1; }

function toggleEditor() {
  const on = !editor.active;
  stopDemo(); hud.toggleHelp(false);
  if (on) newRun(); // show everything at its start position
  editor.setActive(on);
  if (!on) newRun();
}

function act(a) {
  if (a === 'help') { if (!editor.active) hud.toggleHelp(); return; }
  if (a === 'editor') return toggleEditor();
  if (a === 'demo') return startDemo();
  if (editor.active) return;
  if (hud.helpOpen) { if (a === 'missionComplete') hud.toggleHelp(false); return; }
  if (a === 'newRun') { stopDemo(); return newRun(); }
  if (a === 'view') { view = VIEWS[(VIEWS.indexOf(view) + 1) % VIEWS.length]; return; }
  if (a === 'pause') { paused = !paused; return; }
  if (a === 'speed') { speed = speed >= 4 ? 1 : speed * 2; return; }
  if (a === 'realism') return toggleRealism();
  if (ap) return; // the autopilot is flying
  const r = sim.rules;
  if (a.startsWith('declare')) return void Rules.declare(r, Number(a.slice(7)), sim.atHome);
  switch (a) {
    case 'magnet': toggleMagnet(sim); break;
    case 'missionComplete': Rules.completeMission(r, 'declared by the pilot'); break;
    case 'cancel': Rules.cancel(r); break;
    case 'flightComplete': Rules.flightComplete(r, sim.atHome); break;
    case 'reset': resetDrone(sim); poseDelay.clear(); break; // no delayed picture of the jump home
    case 'mode': settings.mode = sim.fc.mode = sim.fc.mode === 'angle' ? 'acro' : 'angle'; saveSettings(); break;
    case 'altHold': setSetting('altHold', !settings.altHold); hud.syncSettings(); break;
    case 'gripper': setSetting('gripper', !settings.gripper); hud.syncSettings(); break;
  }
}

function setSetting(key, value) {
  settings[key] = value; saveSettings();
  if (!sim) return;
  if (key === 'altHold') { sim.fc.altHold = value; sim.fc.altI = 0; }
  if (key === 'gripper') sim.gripper = value;
  if (key === 'corridorMode') sim.rules.corridorMode = value;
  if (key in sim.realism) sim.realism[key] = value; // ground, wash, battery, packmAh: all take effect at once
}

// One key for all four realism switches: any on -> all off, all off -> all on.
function toggleRealism() {
  const on = !REALISM.some((k) => settings[k]);
  for (const k of REALISM) setSetting(k, on);
  hud.syncSettings();
  sim.rules.log.push({ t: sim.rules.time, text: on ? 'Realism on: video link, ground effect, prop wash, battery.' : 'Realism off.', kind: 'info' });
}

const input = createInput(act);
const hud = createHud({ onAction: act, input, settings, onSetting: setSetting, packs: DRONE.battery.packs, linkTypes: LINK_TYPES });
const editor = createEditor({ canvas, camera: cams.top, scene, getLayout: () => layout, onChange: () => newRun() });
let acc = 0;
const ZERO = { throttle: 0, pitch: 0, roll: 0, yaw: 0 };
newRun();
if (/demo/.test(location.search + location.hash)) startDemo();

function resize() {
  const w = app.clientWidth, h = app.clientHeight;
  if (!w || !h) return;
  renderer.setSize(w, h, false); cams.resize(w / h);
}
new ResizeObserver(resize).observe(app); resize();

let last = performance.now();
function frame(now) {
  requestAnimationFrame(frame);
  const dt = Math.min(0.05, (now - last) / 1000); last = now;
  let sticks = input.update(dt);
  const running = !paused && !editor.active;
  if (hud.helpOpen) sticks = ZERO; // the drone idles behind the help screen
  if (running) {
    acc += dt * (ap ? speed : 1);
    let n = 0;
    while (acc >= DT && n < 120) { step(sim, ap ? ap.sticks(DT) : sticks); acc -= DT; n++; }
    if (acc > DT) acc = 0; // too slow to keep up: drop time rather than spiral
    if (ap && (ap.done || sim.crashed || sim.battLevel === 3)) stopDemo();
  }
  const q = sim.quad, t = now / 1000;
  poseDelay.push(t, q.p, q.q);
  drone.group.position.set(q.p[0], q.p[1], q.p[2]); drone.group.quaternion.set(q.q[0], q.q[1], q.q[2], q.q[3]);
  if (running) drone.update(q, dt);
  objVis.update(sim.objects); courseVis.setBeacon(sim.beaconLit);
  cams.update(dt);
  const shown = editor.active ? 'top' : view;

  // FPV video link: the signal follows the real drone all the time, the picture shows it a moment late.
  const live = running && !hud.helpOpen;
  if (settings.fpvLink && live) {
    stepLink(link, dt, { quality: linkQuality(q.p, qRot(q.q, UP), FIELD.pilotEye, sim.course.boxes), enclosed: insideTower(sim.course.tower, q.p), type: settings.linkType });
  }
  const fpvLink = settings.fpvLink && shown === 'fpv';
  if (fpvLink) {
    const old = poseDelay.sample(t - link.delay); // the other cameras have already used the true pose above
    if (old) { drone.group.position.set(old.p[0], old.p[1], old.p[2]); drone.group.quaternion.set(old.q[0], old.q[1], old.q[2], old.q[3]); }
  }

  scene.fog.far = shown === 'top' ? 200 : 34; scene.fog.near = shown === 'top' ? 190 : 14;
  renderer.render(scene, cams.get(shown));
  if (!fpvLink) fpvFx.draw(null, settings.linkType, now);
  else if (live) fpvFx.draw(link, settings.linkType, now); // paused: the last picture stays up
  hud.update({ sim, view: shown, paused, editor: editor.active, demo: ap ? ap.label || 'starting' : '', speed, info: drone.info, link: fpvLink ? link : null });
}
requestAnimationFrame(frame);
