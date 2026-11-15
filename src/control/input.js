// Keyboard, gamepad and USB radio (EdgeTX/OpenTX "Joystick" mode shows up as a gamepad).
const KEY_ACTIONS = {
  Space: 'magnet', KeyV: 'view', KeyM: 'mode', KeyR: 'reset', KeyP: 'pause', KeyC: 'cancel', KeyF: 'flightComplete',
  KeyL: 'editor', Enter: 'missionComplete', KeyT: 'altHold', KeyH: 'help', KeyN: 'newRun', KeyG: 'gripper', KeyX: 'realism',
  Digit1: 'declare1', Digit2: 'declare2', Digit3: 'declare3', Digit4: 'declare4', Digit5: 'declare5',
  Numpad1: 'declare1', Numpad2: 'declare2', Numpad3: 'declare3', Numpad4: 'declare4', Numpad5: 'declare5',
};
const FLY_KEYS = new Set(['KeyW', 'KeyS', 'KeyA', 'KeyD', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'ShiftLeft', 'ShiftRight']);

const DEFAULT_MAP = { throttle: 1, yaw: 0, pitch: 3, roll: 2, invThrottle: true, invYaw: false, invPitch: true, invRoll: false };

export function createInput(onAction) {
  const keys = new Set();
  const inp = { sticks: { throttle: 0, pitch: 0, roll: 0, yaw: 0 }, source: 'keyboard', gamepadName: '', map: loadMap(), rawAxes: [], touch: { active: 0, throttle: 0, yaw: 0, pitch: 0, roll: 0 } };
  const smooth = { throttle: 0, pitch: 0, roll: 0, yaw: 0 };

  addEventListener('keydown', (e) => {
    if (e.target.matches?.('input, select, textarea')) return;
    if (e.ctrlKey || e.metaKey || e.altKey) return; // browser shortcuts (Ctrl+R, Ctrl+X, ...) stay the browser's
    if (FLY_KEYS.has(e.code) || KEY_ACTIONS[e.code]) e.preventDefault();
    if (e.repeat) return;
    keys.add(e.code);
    if (KEY_ACTIONS[e.code]) onAction(KEY_ACTIONS[e.code]);
  });
  addEventListener('keyup', (e) => keys.delete(e.code));
  addEventListener('blur', () => keys.clear());

  inp.update = (dt) => {
    const k = (a, b) => (keys.has(a) ? 1 : 0) - (keys.has(b) ? 1 : 0);
    const fine = keys.has('ShiftLeft') || keys.has('ShiftRight') ? 0.4 : 1; // hold Shift for gentle inputs
    const kb = { throttle: k('KeyW', 'KeyS'), yaw: k('KeyD', 'KeyA') * 0.8, pitch: k('ArrowUp', 'ArrowDown') * 0.4 * fine, roll: k('ArrowRight', 'ArrowLeft') * 0.4 * fine };
    kb.throttle *= fine; kb.yaw *= fine;
    for (const a in smooth) smooth[a] += (kb[a] - smooth[a]) * Math.min(1, dt * (kb[a] === 0 ? 14 : 9));

    if (inp.touch && inp.touch.active) { inp.source = 'touch'; Object.assign(inp.sticks, { throttle: inp.touch.throttle, yaw: inp.touch.yaw, pitch: inp.touch.pitch, roll: inp.touch.roll }); return inp.sticks; }
    let pad = null;
    try { for (const p of navigator.getGamepads?.() || []) if (p && p.connected && p.axes.length >= 4) { pad = p; break; } } catch { /* blocked in some frames */ }
    if (pad) {
      const m = inp.map, ax = (i, inv) => { const v = pad.axes[i] || 0; return (Math.abs(v) < 0.04 ? 0 : v) * (inv ? -1 : 1); };
      const g = { throttle: ax(m.throttle, m.invThrottle), yaw: ax(m.yaw, m.invYaw), pitch: ax(m.pitch, m.invPitch), roll: ax(m.roll, m.invRoll) };
      inp.rawAxes = Array.from(pad.axes); inp.gamepadName = pad.id;
      const moved = Math.abs(g.yaw) + Math.abs(g.pitch) + Math.abs(g.roll) > 0.08 || inp.source === 'gamepad';
      const kbActive = Object.values(kb).some((v) => v !== 0);
      if (moved && !kbActive) { inp.source = 'gamepad'; Object.assign(inp.sticks, g); return inp.sticks; }
      if (kbActive) inp.source = 'keyboard';
    } else { inp.gamepadName = ''; inp.source = 'keyboard'; }
    Object.assign(inp.sticks, smooth);
    return inp.sticks;
  };
  inp.setMap = (m) => { inp.map = { ...inp.map, ...m }; try { localStorage.setItem('lifeline.padmap', JSON.stringify(inp.map)); } catch { /* storage unavailable */ } };
  return inp;
}

function loadMap() {
  try { return { ...DEFAULT_MAP, ...JSON.parse(localStorage.getItem('lifeline.padmap') || '{}') }; } catch { return { ...DEFAULT_MAP }; }
}
