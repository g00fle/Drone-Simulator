// On-screen display: clock, mission, score sheet, telemetry, messages, overlays, touch sticks.
import { Rules } from '../sim.js';
import { VIEW_NAMES } from './cameras.js';

const $ = (id) => document.getElementById(id);
const set = (el, text) => { if (el.textContent !== text) el.textContent = text; };
const { MISSIONS, FLIGHT_COMPLETE_POINTS, fmt } = Rules;

const HINTS = {
  1: (r) => (r.m1.entered ? 'Hold each marker in the centre of the camera for 1 s, then leave by the upper opening.' : 'Enter the tower through the ground-level entrance.'),
  2: () => 'Slide the hook through a harness ring, lift, set the figure fully inside the yellow Safe Zone and back out.',
  3: () => 'Magnet on (Space), touch a pod washer. One pod on the tower roof, one on the green square.',
  4: () => 'Only your first passage scores. Then push hazmat pods into the red zone. Enter when you are done.',
  5: (r) => (r.corridorMode === 'single' ? 'Fly through one corridor bay, then set a foot on the beacon.' : 'Weave through all three corridor bays at one level, then set a foot on the beacon.'),
};

export function createHud({ onAction, input, settings, onSetting, packs = [], linkTypes = [] }) {
  const hud = { helpOpen: true, endDismissed: false };
  const el = Object.fromEntries(['clock', 'mission', 'hint', 'sheet', 'total', 'missions', 'alt', 'spd', 'thrBar', 'battBar', 'battVal', 'chips', 'log',
    'reticle', 'reticleProg', 'reticleText', 'banner', 'bannerTitle', 'bannerText', 'bannerBtn', 'help', 'end', 'endReason', 'endTotal', 'endTime', 'endRows',
    'speedBtn', 'padMap', 'padAxes', 'modelInfo', 'setAlt', 'setGrip', 'setCorr',
    'setLink', 'setLinkType', 'setGround', 'setWash', 'setBatt', 'setPack'].map((k) => [k, $(k)]));

  // ---- score sheet ----
  const rows = {}, itemEls = {};
  for (const m of MISSIONS) {
    const max = m.items.reduce((a, i) => a + i[2], 0);
    const li = document.createElement('li');
    li.innerHTML = `<button class="m" title="Declare Mission ${m.n}"><span class="num">${m.n}</span><span class="name">${m.name}</span><span class="pill">open</span><span class="pts">0/${max}</span></button><ul class="items"></ul>`;
    for (const [id, name, pts] of m.items) {
      const it = document.createElement('li'); it.innerHTML = `<span>${name}</span><span>0/${pts}</span>`;
      li.lastChild.append(it); itemEls[id] = { li: it, val: it.lastChild, pts };
    }
    li.firstChild.addEventListener('click', () => onAction(`declare${m.n}`));
    el.missions.append(li); rows[m.n] = { li, pill: li.querySelector('.pill'), pts: li.querySelector('.pts'), max };
  }
  const fcLi = document.createElement('li');
  fcLi.innerHTML = `<div class="m"><span class="num">F</span><span class="name">Flight Complete</span><span class="pill">open</span><span class="pts">0/${FLIGHT_COMPLETE_POINTS}</span></div>`;
  el.missions.append(fcLi);
  if (matchMedia('(max-width: 760px)').matches) el.sheet.dataset.open = '0';
  $('sheetToggle').addEventListener('click', () => { el.sheet.dataset.open = el.sheet.dataset.open === '1' ? '0' : '1'; });

  // ---- buttons ----
  for (const b of document.querySelectorAll('[data-act]')) b.addEventListener('click', () => { b.blur(); onAction(b.dataset.act); });
  $('flyBtn').addEventListener('click', () => hud.toggleHelp(false));
  $('demoBtn').addEventListener('click', () => onAction('demo'));
  $('layoutBtn').addEventListener('click', () => onAction('editor'));
  $('edDone').addEventListener('click', () => onAction('editor'));
  $('endNew').addEventListener('click', () => onAction('newRun'));
  $('endClose').addEventListener('click', () => { hud.endDismissed = true; el.end.hidden = true; });
  el.bannerBtn.addEventListener('click', () => onAction(el.bannerBtn.dataset.act));

  hud.toggleHelp = (on = !hud.helpOpen) => { hud.helpOpen = on; el.help.hidden = !on; };

  // ---- settings ----
  el.setAlt.checked = settings.altHold; el.setGrip.checked = settings.gripper; el.setCorr.value = settings.corridorMode;
  el.setAlt.addEventListener('change', () => onSetting('altHold', el.setAlt.checked));
  el.setGrip.addEventListener('change', () => onSetting('gripper', el.setGrip.checked));
  el.setCorr.addEventListener('change', () => onSetting('corridorMode', el.setCorr.value));
  // realism switches: [checkbox, setting]; the two selects pick the video system and the battery pack
  const REAL = [[el.setLink, 'fpvLink'], [el.setGround, 'ground'], [el.setWash, 'wash'], [el.setBatt, 'battery']];
  el.setLinkType.innerHTML = linkTypes.map((t) => `<option value="${t}">${t}</option>`).join('');
  el.setPack.innerHTML = packs.map((p) => `<option value="${p}">${p} mAh</option>`).join('');
  for (const [box, key] of REAL) box.addEventListener('change', () => onSetting(key, box.checked));
  el.setLinkType.addEventListener('change', () => onSetting('linkType', el.setLinkType.value));
  el.setPack.addEventListener('change', () => onSetting('packmAh', Number(el.setPack.value)));
  hud.syncSettings = () => {
    el.setAlt.checked = settings.altHold; el.setGrip.checked = settings.gripper;
    for (const [box, key] of REAL) box.checked = !!settings[key];
    el.setLinkType.value = settings.linkType; el.setPack.value = String(settings.packmAh);
  };
  hud.syncSettings();
  for (const [axis, label] of [['throttle', 'Throttle'], ['yaw', 'Yaw'], ['pitch', 'Pitch'], ['roll', 'Roll']]) {
    const inv = `inv${label}`, row = document.createElement('label');
    row.innerHTML = `<span style="width:64px">${label}</span><select id="pad_${axis}" aria-label="${label} axis">${[0, 1, 2, 3, 4, 5, 6, 7].map((i) => `<option value="${i}">axis ${i}</option>`).join('')}</select><input type="checkbox" id="pad_${inv}"> reversed`;
    el.padMap.append(row);
    const sel = row.querySelector('select'), chk = row.querySelector('input');
    sel.value = input.map[axis]; chk.checked = !!input.map[inv];
    sel.addEventListener('change', () => input.setMap({ [axis]: Number(sel.value) }));
    chk.addEventListener('change', () => input.setMap({ [inv]: chk.checked }));
  }

  // ---- touch sticks ----
  const bindStick = (id, ax, ay) => {
    const pad = $(id), nub = pad.firstElementChild; let pid = null;
    const move = (e) => {
      const r = pad.getBoundingClientRect(), half = r.width / 2;
      let x = (e.clientX - r.left - half) / half, y = -(e.clientY - r.top - half) / half;
      const l = Math.hypot(x, y); if (l > 1) { x /= l; y /= l; }
      input.touch[ax] = x; input.touch[ay] = y;
      nub.style.transform = `translate(${x * half * 0.65}px, ${-y * half * 0.65}px)`;
    };
    const end = (e) => { if (e.pointerId !== pid) return; pid = null; input.touch.active--; input.touch[ax] = 0; input.touch[ay] = 0; nub.style.transform = ''; };
    pad.addEventListener('pointerdown', (e) => { if (pid !== null) return; pid = e.pointerId; pad.setPointerCapture(pid); input.touch.active++; move(e); e.preventDefault(); });
    pad.addEventListener('pointermove', (e) => { if (e.pointerId === pid) move(e); });
    pad.addEventListener('pointerup', end); pad.addEventListener('pointercancel', end);
  };
  bindStick('stickL', 'yaw', 'throttle'); bindStick('stickR', 'roll', 'pitch');

  // ---- messages ----
  let lastRules = null, lastEntry = null; const toasts = [];
  function messages(r, now) {
    if (r !== lastRules) { lastRules = r; lastEntry = null; for (const t of toasts) t.node.remove(); toasts.length = 0; hud.endDismissed = false; }
    const from = lastEntry ? r.log.indexOf(lastEntry) + 1 : 0;
    for (let i = from; i < r.log.length; i++) {
      const node = document.createElement('div'); node.className = `toast ${r.log[i].kind}`; node.textContent = r.log[i].text;
      el.log.append(node); toasts.push({ node, until: now + (r.log[i].kind === 'info' ? 4500 : 6500) });
    }
    if (r.log.length) lastEntry = r.log[r.log.length - 1];
    while (toasts.length && (toasts[0].until < now || toasts.length > 4)) toasts.shift().node.remove();
  }

  // ---- per-frame update (text refreshed 10 times a second) ----
  let nextText = 0;
  hud.update = (s) => {
    const { sim } = s, r = sim.rules, now = performance.now();
    // reticle follows every frame
    const showRet = s.view === 'fpv' && !s.editor;
    el.reticle.hidden = !showRet;
    if (showRet) {
      const seen = sim.markerSeen;
      el.reticleProg.style.strokeDashoffset = String(289 * (1 - (seen ? seen.hold : 0)));
      set(el.reticleText, seen ? `Marker ${seen.id}` : '');
    }
    if (now < nextText) return; nextText = now + 100;
    messages(r, now);

    const left = Rules.timeLeft(r);
    set(el.clock, fmt(Math.ceil(left - 1e-6)));
    el.clock.className = !r.started ? 'idle' : left < 60 ? 'low' : '';
    let mission, hint;
    if (s.demo) { mission = `Demo flight: ${s.demo}`; hint = 'The autopilot is flying a full run. Press N to stop and fly yourself.'; }
    else if (r.over) { mission = r.overReason; hint = 'Press N for a new run.'; }
    else if (r.active) { mission = `Mission ${r.active}: ${MISSIONS[r.active - 1].name}`; hint = HINTS[r.active](r); }
    else if (!r.started) { mission = 'Not started'; hint = sim.atHome ? 'Press 1 to 5 to declare a mission. The clock starts then.' : 'Land on Home Base, then declare a mission.'; }
    else { mission = sim.atHome ? 'At Home Base' : 'Return to Home Base'; hint = sim.atHome ? 'Declare the next mission (1 to 5), or F for Flight Complete.' : 'You must be on Home Base to declare the next mission.'; }
    if (s.paused) mission = `Paused · ${mission}`;
    set(el.mission, mission); set(el.hint, hint);

    set(el.total, String(Rules.total(r)));
    for (const m of MISSIONS) {
      const row = rows[m.n], st = r.status[m.n];
      if (row.li.className !== st) row.li.className = st;
      set(row.pill, st); set(row.pts, `${Rules.missionTotal(r, m.n)}/${row.max}`);
      for (const [id] of m.items) { const it = itemEls[id], v = r.score[id]; set(it.val, `${v}/${it.pts}`); it.li.className = v ? 'got' : ''; }
    }
    fcLi.className = r.flightComplete ? 'complete' : '';
    set(fcLi.querySelector('.pill'), r.flightComplete ? 'complete' : 'open'); set(fcLi.querySelector('.pts'), `${r.flightComplete}/${FLIGHT_COMPLETE_POINTS}`);

    const q = sim.quad, alt = Math.max(0, q.p[1] + sim.cfg.footZ);
    set(el.alt, `${alt.toFixed(2)} m · ${(alt / 0.0254).toFixed(0)} in`);
    set(el.spd, `${Math.hypot(q.v[0], q.v[1], q.v[2]).toFixed(1)} m/s`);
    const thr = sim.lastCmd ? sim.lastCmd.reduce((a, b) => a + b, 0) / 4 : 0;
    el.thrBar.firstElementChild.style.width = `${(thr * 100).toFixed(0)}%`;
    // battery: charge left and pack voltage; yellow when low, red when critical; greyed out when switched off
    const battOn = sim.realism.battery, soc = battOn ? Math.min(1, Math.max(0, q.soc)) : 1;
    el.battBar.firstElementChild.style.width = `${(soc * 100).toFixed(0)}%`;
    const battCls = !battOn ? 'bar off' : sim.battLevel >= 2 ? 'bar low' : sim.battLevel === 1 ? 'bar warn' : 'bar';
    if (el.battBar.className !== battCls) el.battBar.className = battCls;
    set(el.battVal, !battOn ? 'off' : q.volts > 0 ? `${Math.ceil(soc * 100 - 1e-9)}% · ${q.volts.toFixed(1)} V` : `${Math.ceil(soc * 100 - 1e-9)}%`);
    const carried = sim.carried ? { pod: 'Pod', figure: 'Figure', ball: 'Hazmat pod' }[sim.carried.type] : '';
    const realOn = (settings.fpvLink ? 1 : 0) + (sim.realism.ground ? 1 : 0) + (sim.realism.wash ? 1 : 0) + (battOn ? 1 : 0);
    const chips = [
      [sim.magnetOn ? 'Magnet on' : 'Magnet off', sim.magnetOn ? 'on' : ''],
      carried && [`Carrying ${carried}`, 'on'],
      [sim.fc.mode === 'acro' ? 'Acro' : 'Angle', ''],
      [sim.fc.altHold ? 'Alt hold' : 'Manual throttle', ''],
      [VIEW_NAMES[s.view], ''],
      [realOn === 4 ? 'Realism on' : realOn ? `Realism ${realOn}/4` : 'Realism off', ''],
      s.link && [`Video ${Math.round(s.link.q * 100)}%`, s.link.q < 0.8 ? 'warn' : ''],
      sim.atHome && ['Home Base', 'home'],
      input.source !== 'keyboard' && [input.source, ''],
    ].filter(Boolean);
    const html = chips.map(([t, c]) => `<span class="chip ${c}">${t}</span>`).join('');
    if (el.chips.dataset.h !== html) { el.chips.dataset.h = html; el.chips.innerHTML = html; }

    // banner: empty battery, crash or pause
    const flat = sim.battLevel === 3 && !r.over && !s.demo;
    const crash = sim.crashed && !r.over && !s.demo;
    el.banner.hidden = !(flat || crash || s.paused) || s.editor || hud.helpOpen;
    if (flat) { set(el.bannerTitle, 'Battery empty'); set(el.bannerText, 'The motors have cut out and the clock keeps running. A new run starts with a fresh pack; the battery can be switched off in Help (H).'); set(el.bannerBtn, 'New run (N)'); el.bannerBtn.dataset.act = 'newRun'; }
    else if (crash) { set(el.bannerTitle, sim.crashReason || 'Crashed'); set(el.bannerText, 'The motors are off. The clock keeps running.'); set(el.bannerBtn, 'Reset at Home Base (R)'); el.bannerBtn.dataset.act = 'reset'; }
    else if (s.paused) { set(el.bannerTitle, 'Paused'); set(el.bannerText, 'The clock is stopped.'); set(el.bannerBtn, 'Resume (P)'); el.bannerBtn.dataset.act = 'pause'; }

    // end of run
    const showEnd = r.over && !hud.endDismissed && !s.editor;
    if (showEnd && el.end.hidden) {
      set(el.endReason, r.overReason); set(el.endTotal, String(Rules.total(r)));
      set(el.endTime, `Time used ${fmt(r.finishTime)} of 10:00 (recorded for tie-breaks).`);
      el.endRows.innerHTML = MISSIONS.map((m) => `<div><span>${m.n}. ${m.name}</span><span>${Rules.missionTotal(r, m.n)}/${rows[m.n].max}</span></div>`).join('')
        + `<div><span>Flight Complete</span><span>${r.flightComplete}/${FLIGHT_COMPLETE_POINTS}</span></div>`;
    }
    el.end.hidden = !showEnd;

    el.speedBtn.hidden = !s.demo; set(el.speedBtn, `Speed ${s.speed}x`);
    if (hud.helpOpen) {
      set(el.padAxes, input.gamepadName ? `${input.gamepadName.slice(0, 40)}: ${input.rawAxes.map((v, i) => `${i}:${v.toFixed(2)}`).join('  ')}` : 'No controller detected');
      set(el.modelInfo, s.info || '');
    }
  };
  return hud;
}
