'use strict';

// ---- Configuration ----------------------------------------------------------
const BASE_PX_PER_DAY = 20;
const MIN_PX_PER_DAY  = 4;
const MAX_PX_PER_DAY  = 150;
const ZOOM_FACTOR     = 1.5;
const EDGE_PADDING    = 80; // px added before first / after last event

let pxPerDay = BASE_PX_PER_DAY;

// ---- Helpers ----------------------------------------------------------------
const DAY_MS = 86_400_000;

function toMs(iso) { return new Date(iso).getTime(); }

function xAt(ms, startMs) {
  return EDGE_PADDING + ((ms - startMs) / DAY_MS) * pxPerDay;
}

function fmtDate(iso) {
  return new Date(iso).toLocaleDateString('en-GB', {
    day: 'numeric', month: 'short', year: 'numeric'
  });
}

function fmtDatetime(iso) {
  const d = new Date(iso);
  const date = d.toLocaleDateString('en-GB', { day: 'numeric', month: 'short', year: 'numeric' });
  const time = d.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit' });
  return `${date} at ${time}`;
}

// Color for a single commit dot
// mine+merged → repo color | unmerged (anyone) → gray | other-author+merged → steel blue
const COLOR_UNMERGED   = '#5a6270';
const COLOR_OTHER_AUTH = '#c4b87e';

function commitDotColor(commit, repoColor) {
  if (!commit.merged)       return COLOR_UNMERGED;
  if (commit.mine === false) return COLOR_OTHER_AUTH;
  return repoColor;
}

// A commit is "milestone" worthy if its message is mostly uppercase (e.g. "IT WORKS")
function isMilestone(msg) {
  const letters = msg.replace(/[^a-zA-Z]/g, '');
  if (letters.length < 4) return false;
  const upper = letters.replace(/[^A-Z]/g, '');
  return upper.length / letters.length > 0.6;
}

// ---- Time range -------------------------------------------------------------
let timeRange = null;

function computeTimeRange(data) {
  const msList = [];
  for (const repo of data.repos)
    for (const c of repo.commits) msList.push(toMs(c.date));
  for (const m of data.media) msList.push(toMs(m.date));

  return {
    startMs: Math.min(...msList) - 4 * DAY_MS,
    endMs:   Math.max(...msList) + 6 * DAY_MS,
  };
}

// ---- Canvas width -----------------------------------------------------------
function canvasWidth(startMs, endMs) {
  return xAt(endMs, startMs) + EDGE_PADDING;
}

// ---- Legend -----------------------------------------------------------------
function buildLegend(repos) {
  const el = document.getElementById('legend');
  el.innerHTML = '';
  for (const repo of repos) {
    const item = document.createElement('div');
    item.className = 'legend-item';
    item.innerHTML = `
      <span class="legend-swatch" style="background:${repo.color}"></span>
      <span>${repo.label}</span>
      <span class="legend-count">${repo.commits.length}</span>
    `;
    el.appendChild(item);
  }

  const hasUnmerged  = repos.some(r => r.commits.some(c => !c.merged));
  const hasOtherAuth = repos.some(r => r.commits.some(c => c.mine === false));

  if (hasOtherAuth) {
    const item = document.createElement('div');
    item.className = 'legend-item';
    item.innerHTML = `
      <span class="legend-swatch" style="background:${COLOR_OTHER_AUTH}"></span>
      <span>other author</span>
    `;
    el.appendChild(item);
  }

  if (hasUnmerged) {
    const item = document.createElement('div');
    item.className = 'legend-item';
    item.innerHTML = `
      <span class="legend-swatch" style="background:${COLOR_UNMERGED}"></span>
      <span>unmerged branch</span>
    `;
    el.appendChild(item);
  }
}

// ---- Media (photos + videos) ------------------------------------------------
function repositionMedia(startMs) {
  document.getElementById('photos-area').querySelectorAll('.photo-bubble').forEach(b => {
    b.style.left = `${xAt(toMs(b.dataset.date), startMs)}px`;
  });
}

function buildMedia(mediaItems, startMs) {
  const area = document.getElementById('photos-area');
  area.innerHTML = '';

  for (const item of mediaItems) {
    const x = xAt(toMs(item.date), startMs);

    const bubble = document.createElement('div');
    bubble.className = 'photo-bubble' + (item.note ? ' has-note' : '');
    bubble.style.left = `${x}px`;
    bubble.dataset.date = item.date;

    const isVideo = item.type === 'video';

    const noteIcon = item.note
      ? `<div class="note-indicator" aria-label="Has note">
           <svg viewBox="0 0 12 10" fill="currentColor" aria-hidden="true">
             <rect x="0" y="0" width="12" height="1.8" rx="0.9"/>
             <rect x="0" y="4.1" width="12" height="1.8" rx="0.9"/>
             <rect x="0" y="8.2" width="7.5" height="1.8" rx="0.9"/>
           </svg>
         </div>`
      : '';

    const mediaEl = isVideo
      ? `<video src="${item.path}" muted loop playsinline preload="auto" draggable="false"
              disablePictureInPicture disableremoteplayback></video>
         <div class="video-shield"></div>
         <div class="media-play-icon">
           <svg viewBox="0 0 16 16" fill="currentColor"><path d="M4 3l10 5-10 5V3z"/></svg>
         </div>`
      : `<img src="${item.path}" alt="Project photo" loading="eager" draggable="false">`;

    bubble.innerHTML = `
      <div class="photo-inner">
        ${noteIcon}
        <div class="photo-card">
          ${mediaEl}
          <div class="photo-caption">${fmtDate(item.date)}</div>
        </div>
        <div class="photo-tail"></div>
      </div>
      <div class="photo-stem"></div>
    `;

    bubble.addEventListener('click', () => {
      const idx = lbItems.findIndex(m => m.path === item.path);
      if (idx !== -1) openLightbox(idx);
    });

    // Hide bubble if media fails to load (e.g., .gitignored files)
    const mediaNode = bubble.querySelector('img, video');
    if (mediaNode) {
      mediaNode.addEventListener('error', () => {
        bubble.style.display = 'none';
      });
    }

    area.appendChild(bubble);
  }
}

// ---- Ruler ------------------------------------------------------------------
function buildRuler(startMs, endMs) {
  const ruler = document.getElementById('ruler');
  ruler.innerHTML = '';
  const width = canvasWidth(startMs, endMs);

  // Month columns
  const cur = new Date(startMs);
  cur.setDate(1);
  cur.setHours(0, 0, 0, 0);

  while (cur.getTime() <= endMs + 35 * DAY_MS) {
    const x = xAt(cur.getTime(), startMs);
    if (x >= -50 && x <= width + 50) {
      // Month label
      const month = document.createElement('div');
      month.className = 'ruler-month';
      month.style.left = `${x}px`;
      month.innerHTML = `<span class="ruler-month-label">${
        cur.toLocaleDateString('en-GB', { month: 'short', year: 'numeric' })
      }</span>`;
      ruler.appendChild(month);

      // Week ticks within this month
      for (let w = 1; w <= 4; w++) {
        const wd = new Date(cur.getFullYear(), cur.getMonth(), 1 + w * 7);
        if (wd.getMonth() === cur.getMonth()) {
          const wx = xAt(wd.getTime(), startMs);
          const tick = document.createElement('div');
          tick.className = 'ruler-week';
          tick.style.left = `${wx}px`;
          ruler.appendChild(tick);
        }
      }
    }
    cur.setMonth(cur.getMonth() + 1);
  }

  // "Today" marker
  const now = Date.now();
  if (now >= startMs && now <= endMs) {
    const todayX = xAt(now, startMs);
    const line = document.createElement('div');
    line.className = 'ruler-today';
    line.style.left = `${todayX}px`;
    line.innerHTML = `<span class="ruler-today-label">today</span>`;
    ruler.appendChild(line);
  }
}

// ---- Commit lanes -----------------------------------------------------------
function repositionLanes(startMs) {
  document.getElementById('lanes').querySelectorAll('.commit-dot').forEach(dot => {
    dot.style.left = `${xAt(toMs(dot.dataset.date), startMs)}px`;
  });
}

function buildLanes(repos, startMs) {
  const lanes = document.getElementById('lanes');
  lanes.innerHTML = '';

  for (const repo of repos) {
    const lane = document.createElement('div');
    lane.className = 'lane';

    const label = document.createElement('div');
    label.className = 'lane-label';
    label.style.color = repo.color;
    label.textContent = repo.label;
    lane.appendChild(label);

    for (const commit of repo.commits) {
      const x = xAt(toMs(commit.date), startMs);
      const dot = document.createElement('div');
      const dotColor = commitDotColor(commit, repo.color);
      dot.className = 'commit-dot' + (isMilestone(commit.message) ? ' is-milestone' : '');
      dot.style.cssText = `left:${x}px; background:${dotColor}; color:${dotColor}`;
      dot.dataset.message = commit.message;
      dot.dataset.date    = commit.date;
      dot.dataset.hash    = commit.hash;
      dot.dataset.repo    = repo.label;
      dot.dataset.author  = commit.author ?? '';
      dot.dataset.color   = repo.color;   // always repo color for tooltip header
      dot.dataset.merged  = commit.merged;
      dot.dataset.mine    = commit.mine ?? true;
      lane.appendChild(dot);
    }

    lanes.appendChild(lane);
  }
}

// ---- Rebuild everything after zoom ------------------------------------------
function rebuild(data, initial = false) {
  const { startMs, endMs } = timeRange;
  const width = canvasWidth(startMs, endMs);

  document.getElementById('canvas').style.width = `${width}px`;
  if (initial) {
    buildMedia(data.media, startMs);
    buildLanes(data.repos, startMs);
  } else {
    repositionMedia(startMs);
    repositionLanes(startMs);
  }
  buildRuler(startMs, endMs);
  updateZoomLabel();
}

function updateZoomLabel() {
  const pct = Math.round((pxPerDay / BASE_PX_PER_DAY) * 100);
  document.getElementById('zoom-label').textContent = `${pct}%`;
}

// ---- Zoom controls ----------------------------------------------------------
function zoomAround(data, newPxPerDay, anchorCanvasX) {
  const vp = document.getElementById('viewport');
  const { startMs } = timeRange;

  // viewport offset of the anchor point (preserved across zoom)
  const viewportOffset = anchorCanvasX - vp.scrollLeft;

  // time value at anchor (using old pxPerDay)
  const anchorMs = startMs + (anchorCanvasX - EDGE_PADDING) / pxPerDay * DAY_MS;

  pxPerDay = newPxPerDay;
  rebuild(data);

  // scroll so the same time sits at the same viewport offset
  vp.scrollLeft = xAt(anchorMs, startMs) - viewportOffset;
}

function setupZoom(data) {
  const vp = document.getElementById('viewport');

  document.getElementById('zoom-in').addEventListener('click', () => {
    const anchor = vp.scrollLeft + vp.clientWidth / 2;
    zoomAround(data, Math.min(pxPerDay * ZOOM_FACTOR, MAX_PX_PER_DAY), anchor);
  });

  document.getElementById('zoom-out').addEventListener('click', () => {
    const anchor = vp.scrollLeft + vp.clientWidth / 2;
    zoomAround(data, Math.max(pxPerDay / ZOOM_FACTOR, MIN_PX_PER_DAY), anchor);
  });

  document.getElementById('zoom-reset').addEventListener('click', () => {
    const anchor = vp.scrollLeft + vp.clientWidth / 2;
    zoomAround(data, BASE_PX_PER_DAY, anchor);
  });

  // Ctrl/Cmd + wheel zooms around cursor
  vp.addEventListener('wheel', (e) => {
    if (!e.ctrlKey && !e.metaKey) return;
    e.preventDefault();
    const factor = e.deltaY < 0 ? ZOOM_FACTOR : 1 / ZOOM_FACTOR;
    const newPx = Math.min(MAX_PX_PER_DAY, Math.max(MIN_PX_PER_DAY, pxPerDay * factor));
    const anchor = vp.scrollLeft + (e.clientX - vp.getBoundingClientRect().left);
    zoomAround(data, newPx, anchor);
  }, { passive: false });
}

// ---- Horizontal scroll (drag + wheel) ---------------------------------------
function setupScroll() {
  const vp = document.getElementById('viewport');
  let down = false, startX = 0, scrollLeft = 0;

  vp.addEventListener('mousedown', (e) => {
    if (e.target.closest('.commit-dot') || e.target.closest('.photo-bubble')) return;
    down = true;
    vp.classList.add('is-dragging');
    startX = e.clientX;
    scrollLeft = vp.scrollLeft;
  });

  window.addEventListener('mouseup', () => {
    down = false;
    document.getElementById('viewport').classList.remove('is-dragging');
  });

  vp.addEventListener('mousemove', (e) => {
    if (!down) return;
    e.preventDefault();
    vp.scrollLeft = scrollLeft - (e.clientX - startX);
  });

  // Regular wheel → horizontal scroll (no modifier)
  vp.addEventListener('wheel', (e) => {
    if (e.ctrlKey || e.metaKey) return; // handled by zoom
    e.preventDefault();
    vp.scrollLeft += (e.deltaX || e.deltaY) * 1.2;
  }, { passive: false });
}

// ---- Tooltip ----------------------------------------------------------------
function setupTooltip() {
  const tip = document.getElementById('tooltip');
  let visible = false;

  function show(dot) {
    tip.style.display = 'block';
    tip.setAttribute('aria-hidden', 'false');
    const badges = [
      dot.dataset.mine   === 'false' ? `<span class="tt-badge tt-badge--other">other author</span>` : '',
      dot.dataset.merged === 'false' ? `<span class="tt-badge tt-badge--unmerged">unmerged</span>` : '',
    ].join('');
    const authorLine = dot.dataset.mine === 'false' && dot.dataset.author
      ? `<div class="tt-author">${dot.dataset.author}</div>` : '';
    tip.innerHTML = `
      <div class="tt-repo" style="color:${dot.dataset.color}">${dot.dataset.repo}${badges}</div>
      <div class="tt-meta">
        <span class="tt-date">${fmtDatetime(dot.dataset.date)}</span>
        <span class="tt-hash">${dot.dataset.hash}</span>
      </div>
      ${authorLine}
      <div class="tt-message">${dot.dataset.message}</div>
    `;
    visible = true;
  }

  function hide() {
    tip.style.display = 'none';
    tip.setAttribute('aria-hidden', 'true');
    visible = false;
  }

  document.addEventListener('mouseover', (e) => {
    const dot = e.target.closest('.commit-dot');
    if (dot) show(dot); else hide();
  });

  document.addEventListener('mousemove', (e) => {
    if (!visible) return;
    const margin = 14;
    let x = e.clientX + margin;
    let y = e.clientY - 10;
    // Keep within viewport
    if (x + tip.offsetWidth > window.innerWidth - 8)
      x = e.clientX - tip.offsetWidth - margin;
    if (y + tip.offsetHeight > window.innerHeight - 8)
      y = window.innerHeight - tip.offsetHeight - 8;
    if (y < 8) y = 8;
    tip.style.left = `${x}px`;
    tip.style.top  = `${y}px`;
  });
}

// ---- Nearest-bubble hover ---------------------------------------------------
// Instead of CSS :hover (which keeps the big bubble active while the cursor is
// still inside its scaled footprint), we track the bubble whose center X is
// closest to the cursor and apply .is-hovered to only that one.
function setupMediaHover() {
  const viewport = document.getElementById('viewport');
  const area     = document.getElementById('photos-area');
  let current    = null;

  function setHovered(bubble) {
    if (bubble === current) return;

    // Un-hover previous
    if (current) {
      current.classList.remove('is-hovered');
      const v = current.querySelector('video');
      if (v) { v.pause(); v.currentTime = 0; }
    }

    // Hover next
    current = bubble;
    if (current) {
      current.classList.add('is-hovered');
      const v = current.querySelector('video');
      if (v) v.play().catch(() => {});
    }
  }

  viewport.addEventListener('mousemove', (e) => {
    const areaRect   = area.getBoundingClientRect();
    const canvasRect = document.getElementById('canvas').getBoundingClientRect();

    // Vertical band: from well above the area (expanded photos) to just below it
    const inBand = e.clientY >= areaRect.top - 260 && e.clientY <= areaRect.bottom + 10;
    if (!inBand) { setHovered(null); return; }

    // Cursor X in canvas coordinates
    const cursorX = e.clientX - canvasRect.left;

    // Find the nearest bubble by its absolute left position (= its date X)
    let nearest = null, minDist = Infinity;
    area.querySelectorAll('.photo-bubble').forEach(b => {
      const dist = Math.abs(parseFloat(b.style.left) - cursorX);
      if (dist < minDist) { minDist = dist; nearest = b; }
    });

    // Only activate within a reasonable horizontal distance
    const threshold = Math.max(60, pxPerDay * 4);
    setHovered(minDist <= threshold ? nearest : null);
  });

  // Clear when the cursor leaves the viewport entirely
  viewport.addEventListener('mouseleave', () => setHovered(null));
}

// ---- Lightbox / carousel ----------------------------------------------------
let lbItems = [];   // media sorted chronologically
let lbIndex = 0;

function openLightbox(index) {
  lbIndex = index;
  renderLbSlide();
  document.getElementById('lightbox').classList.add('is-open');
}

function closeLightbox() {
  const lb    = document.getElementById('lightbox');
  const stage = document.getElementById('lb-stage');
  lb.classList.remove('is-open');
  stage.querySelectorAll('video').forEach(v => v.pause());
  // Remove the element after the fade-out so Edge's video toolbar disappears
  setTimeout(() => { stage.innerHTML = ''; }, 250);
}

function renderLbSlide() {
  const item  = lbItems[lbIndex];
  const stage = document.getElementById('lb-stage');

  // stop previous video before replacing
  stage.querySelectorAll('video').forEach(v => v.pause());

  if (item.type === 'video') {
    const v = document.createElement('video');
    v.src         = item.path;
    v.className   = 'lb-media';
    v.controls                = true;
    v.autoplay                = true;
    v.muted                   = false;
    v.loop                    = true;
    v.playsInline             = true;
    v.disablePictureInPicture = true;
    v.setAttribute('disableremoteplayback', '');
    stage.replaceChildren(v);
    v.play().catch(() => {});
  } else {
    const img = document.createElement('img');
    img.src       = item.path;
    img.className = 'lb-media';
    img.alt       = fmtDate(item.date);
    stage.replaceChildren(img);
  }

  document.getElementById('lb-caption').textContent = fmtDate(item.date);
  document.getElementById('lb-counter').textContent =
    lbItems.length > 1 ? `${lbIndex + 1} / ${lbItems.length}` : '';

  const noteEl = document.getElementById('lb-note');
  if (item.note) {
    noteEl.textContent = item.note;
    noteEl.hidden = false;
  } else {
    noteEl.hidden = true;
  }

  document.getElementById('lb-prev').disabled = lbIndex === 0;
  document.getElementById('lb-next').disabled = lbIndex === lbItems.length - 1;
}

function setupLightbox(mediaItems) {
  lbItems = [...mediaItems].sort((a, b) => toMs(a.date) - toMs(b.date));

  document.getElementById('lb-close').addEventListener('click', closeLightbox);
  document.getElementById('lb-backdrop').addEventListener('click', closeLightbox);

  document.getElementById('lb-prev').addEventListener('click', () => {
    if (lbIndex > 0) { lbIndex--; renderLbSlide(); }
  });
  document.getElementById('lb-next').addEventListener('click', () => {
    if (lbIndex < lbItems.length - 1) { lbIndex++; renderLbSlide(); }
  });

  document.addEventListener('keydown', (e) => {
    if (!document.getElementById('lightbox').classList.contains('is-open')) return;
    if (e.key === 'Escape')                                    closeLightbox();
    if (e.key === 'ArrowLeft'  && lbIndex > 0)               { lbIndex--; renderLbSlide(); }
    if (e.key === 'ArrowRight' && lbIndex < lbItems.length - 1) { lbIndex++; renderLbSlide(); }
  });
}

// ---- Scroll to start of meaningful activity on load ------------------------
function scrollToStart(startMs) {
  const vp = document.getElementById('viewport');
  // Scroll so there's a bit of padding before the first commit
  const target = xAt(startMs + 2 * DAY_MS, startMs) - 40;
  vp.scrollLeft = Math.max(0, target);
}

// ---- Init -------------------------------------------------------------------
function init() {
  const data = TIMELINE_DATA;
  timeRange = computeTimeRange(data);

  buildLegend(data.repos);
  rebuild(data, true);
  setupZoom(data);
  setupScroll();
  setupTooltip();
  setupMediaHover();
  setupLightbox(data.media);
  updateZoomLabel();
  scrollToStart(timeRange.startMs);
}

document.addEventListener('DOMContentLoaded', init);
