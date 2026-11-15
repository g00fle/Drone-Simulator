// Top-down layout editor: drag items on the field, rotate, export/import, saved in this browser.
import * as THREE from 'three';
import { DEFAULT_LAYOUT, ITEM_LABELS, FIELD } from '../config/course.js';

const KEY = 'lifeline.layout';
const $ = (id) => document.getElementById(id);

// Keep only known items with finite numbers; anything missing falls back to the brief layout.
export function cleanLayout(raw) {
  const out = { name: typeof raw?.name === 'string' ? raw.name.slice(0, 60) : 'Custom', items: {} };
  for (const [id, d] of Object.entries(DEFAULT_LAYOUT.items)) {
    const it = raw?.items?.[id] || {}, num = (v, def) => (Number.isFinite(v) ? v : def);
    out.items[id] = {
      x: THREE.MathUtils.clamp(num(it.x, d.x), -FIELD.width / 2, FIELD.width / 2),
      z: THREE.MathUtils.clamp(num(it.z, d.z), -FIELD.length / 2, FIELD.length / 2),
      rot: ((num(it.rot, d.rot) % 360) + 360) % 360,
    };
  }
  return out;
}
export function loadLayout() {
  try { const raw = JSON.parse(localStorage.getItem(KEY) || 'null'); if (raw) return cleanLayout(raw); } catch { /* storage unavailable or bad data */ }
  return cleanLayout(DEFAULT_LAYOUT);
}
export function saveLayout(layout) { try { localStorage.setItem(KEY, JSON.stringify(layout)); } catch { /* storage unavailable */ } }

export function createEditor({ canvas, camera, scene, getLayout, onChange }) {
  const ed = { active: false, selected: null };
  const ring = new THREE.Mesh(new THREE.RingGeometry(0.42, 0.47, 48), new THREE.MeshBasicMaterial({ color: 0xf2c80f, depthTest: false, transparent: true }));
  ring.rotation.x = -Math.PI / 2; ring.position.y = 2.5; ring.renderOrder = 10; ring.visible = false;
  const tick = new THREE.Mesh(new THREE.PlaneGeometry(0.05, 0.3), ring.material); tick.position.y = -0.6; ring.add(tick); // points along the item's front
  scene.add(ring);
  const ray = new THREE.Raycaster(), ground = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0), hit = new THREE.Vector3();
  let drag = null;

  const chips = {};
  for (const id of Object.keys(DEFAULT_LAYOUT.items)) {
    const b = document.createElement('button'); b.className = 'chip'; b.textContent = ITEM_LABELS[id] || id;
    b.addEventListener('click', () => select(id)); $('edItems').append(b); chips[id] = b;
  }

  function show() {
    const it = ed.selected && getLayout().items[ed.selected];
    for (const id in chips) chips[id].classList.toggle('on', id === ed.selected);
    ring.visible = ed.active && !!it;
    if (!it) { $('edInfo').textContent = 'Nothing selected'; return; }
    ring.position.set(it.x, 2.5, it.z); ring.rotation.z = (-it.rot * Math.PI) / 180;
    const ft = (m) => (m / 0.3048).toFixed(1);
    $('edInfo').textContent = `${ITEM_LABELS[ed.selected]}: ${ft(it.x)} ft across, ${ft(FIELD.length / 2 - it.z)} ft from the pilot line, ${it.rot}°`;
  }
  function select(id) { ed.selected = id; show(); }
  function commit() { const l = getLayout(); l.name = 'Custom'; saveLayout(l); onChange(l); show(); if (!$('edIO').hidden) $('edText').value = JSON.stringify(l, null, 1); }
  function groundPoint(e) {
    const r = canvas.getBoundingClientRect();
    ray.setFromCamera(new THREE.Vector2(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1), camera);
    return ray.ray.intersectPlane(ground, hit);
  }
  function rotate(deg) {
    if (!ed.selected) return;
    const it = getLayout().items[ed.selected]; it.rot = (((it.rot + deg) % 360) + 360) % 360; commit();
  }

  canvas.addEventListener('pointerdown', (e) => {
    if (!ed.active) return;
    const p = groundPoint(e); if (!p) return;
    let best = null, bd = 0.6;
    for (const [id, it] of Object.entries(getLayout().items)) { const d = Math.hypot(it.x - p.x, it.z - p.z); if (d < bd) { bd = d; best = id; } }
    if (!best) return;
    const it = getLayout().items[best];
    select(best); drag = { id: best, dx: it.x - p.x, dz: it.z - p.z, moved: false }; canvas.setPointerCapture(e.pointerId);
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!drag) return;
    const p = groundPoint(e); if (!p) return;
    const it = getLayout().items[drag.id];
    it.x = THREE.MathUtils.clamp(Math.round((p.x + drag.dx) * 200) / 200, -FIELD.width / 2, FIELD.width / 2);
    it.z = THREE.MathUtils.clamp(Math.round((p.z + drag.dz) * 200) / 200, -FIELD.length / 2, FIELD.length / 2);
    drag.moved = true; show();
  });
  const up = () => { if (drag?.moved) commit(); drag = null; };
  canvas.addEventListener('pointerup', up); canvas.addEventListener('pointercancel', up);
  addEventListener('keydown', (e) => {
    if (!ed.active || e.target.matches?.('input, select, textarea')) return;
    if (e.code === 'KeyQ') rotate(-15); else if (e.code === 'KeyE') rotate(15);
  });

  $('edCCW').addEventListener('click', () => rotate(-15));
  $('edCW').addEventListener('click', () => rotate(15));
  $('edReset').addEventListener('click', () => { const l = getLayout(), d = cleanLayout(DEFAULT_LAYOUT); l.items = d.items; commit(); l.name = d.name; saveLayout(l); });
  $('edExport').addEventListener('click', () => { const io = $('edIO'); io.hidden = !io.hidden; $('edText').value = JSON.stringify(getLayout(), null, 1); $('edMsg').textContent = 'Copy this text to share a layout, or paste one here and apply it.'; });
  $('edCopy').addEventListener('click', async () => {
    try { await navigator.clipboard.writeText($('edText').value); $('edMsg').textContent = 'Copied.'; } catch { $('edText').select(); $('edMsg').textContent = 'Press Ctrl+C to copy the selected text.'; }
  });
  $('edApply').addEventListener('click', () => {
    try { const l = getLayout(), n = cleanLayout(JSON.parse($('edText').value)); l.items = n.items; commit(); $('edMsg').textContent = 'Layout applied.'; }
    catch { $('edMsg').textContent = 'That text is not a valid layout. Paste the text exactly as it was copied.'; }
  });

  ed.setActive = (on) => { ed.active = on; $('editor').hidden = !on; drag = null; show(); };
  return ed;
}
