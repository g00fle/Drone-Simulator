// Draws what the FPV video link (view/link.js) does to the picture, on a canvas laid over the 3D view.
//   analog:  grain all the time, more static as the signal weakens, colour fading, and during a break-up
//            strips of the picture jump sideways with a burst of static.
//   digital: a clean picture until the signal is weak, then stale blocks, and during a strong break-up
//            the picture freezes on an old frame.
// The delay itself is not drawn here: main.js renders the camera from where the drone was a moment ago.
const W = 384; // overlay width in its own pixels; the browser stretches it over the view
const CELL = 8; // digital block size, in overlay pixels

export function createFpvFx(overlay, view) {
  const ctx = overlay.getContext('2d');
  const noise = document.createElement('canvas'); noise.width = noise.height = 128;
  {
    const n = noise.getContext('2d'), img = n.createImageData(128, 128);
    for (let i = 0; i < img.data.length; i += 4) { const v = (Math.random() * 256) | 0; img.data[i] = img.data[i + 1] = img.data[i + 2] = v; img.data[i + 3] = 255; }
    n.putImageData(img, 0, 0);
  }
  const small = document.createElement('canvas'), sctx = small.getContext('2d'); // the picture at block resolution
  const hold = document.createElement('canvas'), hctx = hold.getContext('2d'); // the frame a frozen picture is stuck on
  let H = 0, shown = false, filter = '', wasFrozen = false, blocks = [], blocksAt = 0, bar = 0, lastNow = 0;
  const rnd = (a, b) => a + Math.random() * (b - a);

  function size() {
    const h = Math.max(2, Math.round((W * view.height) / Math.max(1, view.width)));
    if (h === H && overlay.width === W) return;
    H = h; overlay.width = hold.width = W; overlay.height = hold.height = H;
    small.width = Math.ceil(W / CELL); small.height = Math.ceil(H / CELL);
    wasFrozen = false; blocks = [];
  }
  function setFilter(f) { if (f !== filter) { filter = f; view.style.filter = f; } }
  function show(on) { if (on !== shown) { shown = on; overlay.hidden = !on; } }

  function analog(link, dt) {
    const weak = 1 - link.q;
    // colour fades as the signal weakens
    setFilter(`saturate(${(Math.round(Math.max(0.15, 0.9 - weak * 1.1) * 20) / 20).toFixed(2)}) contrast(0.96)`);
    ctx.clearRect(0, 0, W, H);
    if (link.tear > 0) {
      // strips of the picture knocked sideways
      const strips = 2 + Math.round(link.tear * 5);
      ctx.imageSmoothingEnabled = true;
      for (let i = 0; i < strips; i++) {
        const h = Math.max(2, Math.round(H * rnd(0.02, 0.05 + 0.12 * link.tear))), y = Math.round(rnd(0, H - h));
        const dx = Math.round(rnd(-1, 1) * W * 0.16 * link.tear);
        ctx.drawImage(view, 0, (y / H) * view.height, view.width, (h / H) * view.height, dx, y, W, h);
        ctx.fillStyle = dx > 0 ? '#0b0b0b' : '#e8e8e8'; // the gap the strip leaves behind
        if (dx > 0) ctx.fillRect(0, y, dx, h); else ctx.fillRect(W + dx, y, -dx, h);
      }
    }
    // a faint darker band rolling down the picture, stronger on a weak signal
    bar = (bar + dt * 0.35) % 1.3;
    const by = (bar - 0.15) * H, ba = 0.05 + weak * 0.25;
    const g = ctx.createLinearGradient(0, by - H * 0.12, 0, by + H * 0.12);
    g.addColorStop(0, 'rgba(0,0,0,0)'); g.addColorStop(0.5, `rgba(0,0,0,${ba.toFixed(3)})`); g.addColorStop(1, 'rgba(0,0,0,0)');
    ctx.fillStyle = g; ctx.fillRect(0, by - H * 0.12, W, H * 0.24);
    // static
    ctx.globalAlpha = Math.min(0.85, link.noise * 1.1);
    const ox = (Math.random() * 128) | 0, oy = (Math.random() * 128) | 0;
    for (let y = -oy; y < H; y += 128) for (let x = -ox; x < W; x += 128) ctx.drawImage(noise, x, y);
    ctx.globalAlpha = 1;
  }

  function digital(link, now) {
    setFilter('');
    if (link.frozen) {
      if (!wasFrozen) { hctx.drawImage(view, 0, 0, W, H); wasFrozen = true; } // keep the frame it stuck on
      ctx.imageSmoothingEnabled = true;
      ctx.drawImage(hold, 0, 0);
    } else { wasFrozen = false; ctx.clearRect(0, 0, W, H); }
    if (link.noise <= 0.01) { blocks = []; return; }
    // Runs of blocks along a few rows, showing the picture as it was a moment ago and only in flat colour.
    if (now - blocksAt > 90 || !blocks.length) {
      blocksAt = now; blocks = [];
      sctx.drawImage(view, 0, 0, small.width, small.height);
      const rows = 1 + Math.round(link.noise * 14);
      for (let i = 0; i < rows; i++) {
        blocks.push({ y: (Math.random() * small.height) | 0, x: (Math.random() * small.width) | 0, n: 2 + ((Math.random() * (4 + link.noise * 28)) | 0), tall: Math.random() < link.noise ? 2 : 1 });
      }
    }
    ctx.imageSmoothingEnabled = false;
    for (const b of blocks) {
      const n = Math.min(b.n, small.width - b.x), t = Math.min(b.tall, small.height - b.y);
      ctx.drawImage(small, b.x, b.y, n, t, b.x * CELL, b.y * CELL, n * CELL, t * CELL);
    }
  }

  return {
    // Call right after the 3D view has been rendered, in the same animation frame.
    // link = null (or not the FPV view) removes every effect.
    draw(link, type, now) {
      if (!link) {
        if (shown) { ctx.clearRect(0, 0, overlay.width, overlay.height); show(false); }
        setFilter(''); wasFrozen = false; blocks = []; lastNow = now;
        return;
      }
      size(); show(true);
      const dt = Math.min(0.1, Math.max(0, (now - lastNow) / 1000)); lastNow = now;
      const pix = type === 'digital' ? 'auto' : 'pixelated';
      if (overlay.style.imageRendering !== pix) overlay.style.imageRendering = pix;
      try {
        if (type === 'digital') digital(link, now); else { wasFrozen = false; blocks = []; analog(link, dt); }
      } catch { /* the view could not be copied this frame: skip the effect rather than stop the simulator */ }
    },
  };
}
