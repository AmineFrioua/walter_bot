/* Walter Admin — map, table placement, live scan overlay, inline drive */

const ROSBRIDGE_URL = `ws://${location.hostname}:9090`;

let ros, mapMeta, waypoints = {};
let pendingClick = null;
let activeTab = 'tables';

// ── ROS connection ─────────────────────────────────────────────────────────

function connect() {
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on('connection', () => {
    subscribeMap();
    subscribeOdom();
    if (cmdVelTopic === null) {
      cmdVelTopic = new ROSLIB.Topic({
        ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist',
      });
    }
    if (!driveLoop) driveLoop = setInterval(publishDrive, 100);
    if (activeTab === 'drive' && !scanActive) startScan('live');
    if (scanActive) startScan('accumulate');
  });

  ros.on('close', () => {
    clearInterval(driveLoop); driveLoop = null;
    stopScan();
    setTimeout(connect, 3000);
  });
}

// ── Map ────────────────────────────────────────────────────────────────────

function subscribeMap() {
  const mapTopic = new ROSLIB.Topic({
    ros, name: '/map',
    messageType: 'nav_msgs/OccupancyGrid',
    throttle_rate: 2000,
  });
  mapTopic.subscribe(msg => {
    mapMeta = msg.info;
    drawMap(msg);
    document.getElementById('map-status').textContent =
      `${msg.info.width} × ${msg.info.height}  ·  ${msg.info.resolution}m/px`;
  });
}

function drawMap(msg) {
  const canvas = document.getElementById('map-canvas');
  const { width, height, data } = msg;
  canvas.width  = width;
  canvas.height = height;
  const ctx = canvas.getContext('2d');
  const img = ctx.createImageData(width, height);

  for (let i = 0; i < data.length; i++) {
    const v = data[i];
    let r, g, b;
    if      (v === -1) { r = 80;  g = 80;  b = 100; }
    else if (v ===  0) { r = 220; g = 220; b = 220; }
    else               { r = 25;  g = 25;  b = 35;  }
    const px = (height - 1 - Math.floor(i / width)) * width + (i % width);
    img.data[px * 4]     = r;
    img.data[px * 4 + 1] = g;
    img.data[px * 4 + 2] = b;
    img.data[px * 4 + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
  renderDots();
}

// ── Odom (shared by scan modes) ────────────────────────────────────────────

let robotOdomPose = { x: 0, y: 0, theta: 0 };
let odomSub = null;

function subscribeOdom() {
  if (odomSub) return;
  odomSub = new ROSLIB.Topic({
    ros, name: '/odom', messageType: 'nav_msgs/Odometry', throttle_rate: 100,
  });
  odomSub.subscribe(msg => {
    const p = msg.pose.pose.position;
    const q = msg.pose.pose.orientation;
    robotOdomPose = { x: p.x, y: p.y, theta: 2 * Math.atan2(q.z, q.w) };
  });
}

// ── Coordinate helpers ─────────────────────────────────────────────────────

function canvasToMap(cx, cy) {
  const wrap   = document.getElementById('map-wrap');
  const scaleX = mapMeta.width  / wrap.clientWidth;
  const scaleY = mapMeta.height / wrap.clientHeight;
  return {
    x: cx * scaleX * mapMeta.resolution + mapMeta.origin.position.x,
    y: (mapMeta.height - cy * scaleY) * mapMeta.resolution + mapMeta.origin.position.y,
  };
}

function mapToCanvas(mx, my) {
  const wrap   = document.getElementById('map-wrap');
  const scaleX = wrap.clientWidth  / mapMeta.width;
  const scaleY = wrap.clientHeight / mapMeta.height;
  const px = (mx - mapMeta.origin.position.x) / mapMeta.resolution;
  const py = (my - mapMeta.origin.position.y) / mapMeta.resolution;
  return { left: px * scaleX, top: (mapMeta.height - py) * scaleY };
}

function mapToCanvasPx(mx, my) {
  if (!mapMeta) return null;
  const px = (mx - mapMeta.origin.position.x) / mapMeta.resolution;
  const py = (my - mapMeta.origin.position.y) / mapMeta.resolution;
  return { x: Math.round(px), y: Math.round(mapMeta.height - py) };
}

// ── Scan canvas sizing ─────────────────────────────────────────────────────

function syncScanCanvas() {
  const scan = document.getElementById('scan-canvas');
  if (mapMeta) {
    const map = document.getElementById('map-canvas');
    if (scan.width  !== map.width)  scan.width  = map.width;
    if (scan.height !== map.height) scan.height = map.height;
  } else {
    // No map yet — size to the wrap so radar view fills the panel
    const wrap = document.getElementById('map-wrap');
    if (scan.width  !== wrap.clientWidth)  scan.width  = wrap.clientWidth;
    if (scan.height !== wrap.clientHeight) scan.height = wrap.clientHeight;
  }
}

// ── Scan modes: 'idle' | 'live' | 'accumulate' ────────────────────────────
//
//  live       — Drive tab is active, Scan: OFF.
//               Clear canvas each frame, draw current scan fan.
//               When no map: draws local polar radar centred on canvas.
//
//  accumulate — Scan: ON (any tab).
//               Never clears. Points build up as robot moves.
//               Requires map to place points correctly.

let scanMode     = 'idle';
let scanActive   = false;   // true when Scan: ON button is pressed
let activeScanSub = null;

function startScan(mode) {
  stopScan();
  scanMode = mode;
  if (!ros) return;

  activeScanSub = new ROSLIB.Topic({
    ros,
    name: '/scan_filtered',
    messageType: 'sensor_msgs/LaserScan',
    throttle_rate: mode === 'live' ? 120 : 400,
  });

  activeScanSub.subscribe(msg => {
    syncScanCanvas();
    const canvas = document.getElementById('scan-canvas');
    const ctx    = canvas.getContext('2d');

    if (mode === 'live') {
      ctx.clearRect(0, 0, canvas.width, canvas.height);

      if (!mapMeta) {
        drawLocalRadar(ctx, canvas, msg);
        return;
      }
    }

    if (!mapMeta) return; // accumulate needs map coords

    ctx.fillStyle = mode === 'live' ? 'rgba(96,165,250,0.85)' : '#3b82f6';
    const { x: rx, y: ry, theta } = robotOdomPose;

    for (let i = 0; i < msg.ranges.length; i++) {
      const r = msg.ranges[i];
      if (!isFinite(r) || r < 0.05 || r > msg.range_max) continue;
      const angle = msg.angle_min + i * msg.angle_increment + theta;
      const pt = mapToCanvasPx(rx + r * Math.cos(angle), ry + r * Math.sin(angle));
      if (!pt || pt.x < 0 || pt.x >= canvas.width || pt.y < 0 || pt.y >= canvas.height) continue;
      ctx.fillRect(pt.x, pt.y, 2, 2);
    }
  });
}

function stopScan() {
  if (activeScanSub) { activeScanSub.unsubscribe(); activeScanSub = null; }
  scanMode = 'idle';
}

function clearScanCanvas() {
  const c = document.getElementById('scan-canvas');
  c.getContext('2d').clearRect(0, 0, c.width, c.height);
}

// ── Local polar radar (no map available) ──────────────────────────────────

function drawLocalRadar(ctx, canvas, msg) {
  const cx = canvas.width  / 2;
  const cy = canvas.height / 2;
  // scale: fit msg.range_max (capped at 6m) across half the canvas
  const maxR   = Math.min(msg.range_max || 6, 6);
  const scale  = Math.min(cx, cy) * 0.85 / maxR;

  // Faint range rings
  ctx.strokeStyle = 'rgba(255,255,255,0.06)';
  ctx.lineWidth   = 1;
  for (let ring = 1; ring <= maxR; ring++) {
    ctx.beginPath();
    ctx.arc(cx, cy, ring * scale, 0, Math.PI * 2);
    ctx.stroke();
  }

  // Scan points
  ctx.fillStyle = 'rgba(96,165,250,0.85)';
  for (let i = 0; i < msg.ranges.length; i++) {
    const r = msg.ranges[i];
    if (!isFinite(r) || r < 0.05 || r > msg.range_max) continue;
    const angle = msg.angle_min + i * msg.angle_increment;
    const px = cx + r * Math.cos(angle) * scale;
    const py = cy - r * Math.sin(angle) * scale;  // Y-flip: ROS +Y = up
    ctx.fillRect(Math.round(px), Math.round(py), 2, 2);
  }

  // Robot dot at centre
  ctx.fillStyle = '#3b82f6';
  ctx.beginPath();
  ctx.arc(cx, cy, 5, 0, Math.PI * 2);
  ctx.fill();

  // Forward direction tick
  ctx.strokeStyle = 'rgba(96,165,250,0.5)';
  ctx.lineWidth   = 1.5;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx, cy - 18);
  ctx.stroke();
}

// ── Table dots overlay ─────────────────────────────────────────────────────

function renderDots() {
  const wrap = document.getElementById('map-wrap');
  wrap.querySelectorAll('.table-dot, .table-label').forEach(el => el.remove());

  Object.entries(waypoints).forEach(([name, wp]) => {
    if (!mapMeta) return;
    const { left, top } = mapToCanvas(wp.x, wp.y);

    const dot = document.createElement('div');
    dot.className = 'table-dot';
    dot.style.left = left + 'px';
    dot.style.top  = top  + 'px';
    dot.title = `${wp.label || name} — click to remove`;
    dot.addEventListener('click', e => { e.stopPropagation(); removeWaypoint(name); });
    wrap.appendChild(dot);

    const label = document.createElement('div');
    label.className = 'table-label';
    label.style.left = left + 'px';
    label.style.top  = top  + 'px';
    label.textContent = wp.label || name;
    wrap.appendChild(label);
  });
}

function renderSidebarList() {
  const list  = document.getElementById('table-list');
  list.innerHTML = '';
  const names = Object.keys(waypoints);

  if (names.length === 0) {
    list.innerHTML = '<p style="color:#334155;font-size:0.8rem;padding:10px">No tables yet.<br>Click the map to add one.</p>';
    return;
  }

  names.forEach(name => {
    const wp  = waypoints[name];
    const row = document.createElement('div');
    row.className = 'table-row';
    row.innerHTML = `
      <span>${wp.label || name}</span>
      <span class="coords">${wp.x.toFixed(2)}, ${wp.y.toFixed(2)}</span>
      <button class="btn btn-danger" style="padding:3px 9px;font-size:0.72rem" data-name="${name}">✕</button>
    `;
    row.querySelector('button').addEventListener('click', () => removeWaypoint(name));
    list.appendChild(row);
  });
}

// ── Waypoint CRUD ──────────────────────────────────────────────────────────

async function addWaypoint(name, x, y) {
  const key = name.toLowerCase().replace(/\s+/g, '_');
  try {
    const resp = await fetch(`/api/waypoints/${encodeURIComponent(key)}`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y, theta: 0.0, label: name }),
    });
    waypoints[key] = await resp.json();
    renderDots();
    renderSidebarList();
  } catch (e) { alert('Failed to save: ' + e); }
}

async function removeWaypoint(name) {
  try {
    await fetch(`/api/waypoints/${encodeURIComponent(name)}`, { method: 'DELETE' });
    delete waypoints[name];
    renderDots();
    renderSidebarList();
  } catch (e) { alert('Failed to delete: ' + e); }
}

// ── Map click (Tables mode only) ───────────────────────────────────────────

document.getElementById('map-wrap').addEventListener('click', e => {
  if (activeTab !== 'tables' || !mapMeta) return;
  const rect = e.currentTarget.getBoundingClientRect();
  pendingClick = canvasToMap(e.clientX - rect.left, e.clientY - rect.top);
  openModal();
});

// ── Modal ──────────────────────────────────────────────────────────────────

function openModal() {
  document.getElementById('modal-backdrop').classList.add('open');
  const input = document.getElementById('modal-input');
  input.value = '';
  input.focus();
}

function closeModal() {
  document.getElementById('modal-backdrop').classList.remove('open');
  pendingClick = null;
}

document.getElementById('modal-cancel').addEventListener('click', closeModal);
document.getElementById('modal-ok').addEventListener('click', () => {
  const name = document.getElementById('modal-input').value.trim();
  if (!name || !pendingClick) return;
  const { x, y } = pendingClick;
  closeModal();
  addWaypoint(name, x, y);
});
document.getElementById('modal-input').addEventListener('keydown', e => {
  if (e.key === 'Enter')  document.getElementById('modal-ok').click();
  if (e.key === 'Escape') closeModal();
});
document.getElementById('modal-backdrop').addEventListener('click', e => {
  if (e.target === document.getElementById('modal-backdrop')) closeModal();
});

// ── Tabs ───────────────────────────────────────────────────────────────────

function switchTab(tab) {
  activeTab = tab;
  document.getElementById('tab-tables').classList.toggle('active', tab === 'tables');
  document.getElementById('tab-drive').classList.toggle('active',  tab === 'drive');
  document.getElementById('panel-tables').style.display = tab === 'tables' ? '' : 'none';
  document.getElementById('panel-drive').classList.toggle('visible', tab === 'drive');
  document.getElementById('map-wrap').style.cursor = tab === 'tables' ? 'crosshair' : 'default';

  if (tab === 'drive' && !scanActive) {
    startScan('live');
  } else if (tab !== 'drive' && scanMode === 'live') {
    stopScan();
    clearScanCanvas();
    clearHeld();
    sendStop();
  } else if (tab !== 'drive') {
    clearHeld();
    sendStop();
  }
}

document.getElementById('tab-tables').addEventListener('click', () => switchTab('tables'));
document.getElementById('tab-drive').addEventListener('click',  () => switchTab('drive'));

// ── Scan toggle (Scan ON/OFF header button) ────────────────────────────────

document.getElementById('scan-toggle').addEventListener('click', () => {
  scanActive = !scanActive;
  const btn = document.getElementById('scan-toggle');
  btn.textContent = scanActive ? 'Scan: ON' : 'Scan';
  btn.classList.toggle('active', scanActive);
  document.getElementById('scan-clear').style.display = scanActive ? '' : 'none';

  if (scanActive) {
    startScan('accumulate');
  } else {
    stopScan();
    clearScanCanvas();
    if (activeTab === 'drive') startScan('live');
  }
});

document.getElementById('scan-clear').addEventListener('click', () => {
  clearScanCanvas();
  // Restart accumulate mode so new points draw on the fresh canvas
  if (scanActive) startScan('accumulate');
});

// ── LiDAR arc toggle ──────────────────────────────────────────────────────

let arcForwardOnly = false;

document.getElementById('arc-toggle').addEventListener('click', () => {
  arcForwardOnly = !arcForwardOnly;
  const deg = arcForwardOnly ? 180.0 : 360.0;
  const btn = document.getElementById('arc-toggle');
  btn.textContent = arcForwardOnly ? 'Fwd only' : 'Full scan';
  btn.classList.toggle('active', arcForwardOnly);
  if (!ros) return;
  const svc = new ROSLIB.Service({
    ros, name: '/lidar_filter/set_parameters',
    serviceType: 'rcl_interfaces/srv/SetParameters',
  });
  svc.callService({
    parameters: [{ name: 'forward_arc_deg', value: { type: 4, double_value: deg } }],
  }, result => {
    const ok = result?.results?.[0]?.successful;
    if (!ok) btn.textContent = arcForwardOnly ? 'Fwd only ?' : 'Full scan ?';
  });
});

// ── Drive — speed ──────────────────────────────────────────────────────────

const LIN_MAX = 0.10, LIN_MIN = 0.01;
const ANG_MAX = 1.00, ANG_MIN = 0.05;
const STEP    = 1.1;

let linSpeed = 0.05, angSpeed = 0.50;
let cmdVelTopic = null, driveLoop = null;
const held = { forward: false, backward: false, left: false, right: false };

function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }
function adjustLin(f)  { linSpeed = clamp(linSpeed * f, LIN_MIN, LIN_MAX); updateSpeedDisplay(); }
function adjustAng(f)  { angSpeed = clamp(angSpeed * f, ANG_MIN, ANG_MAX); updateSpeedDisplay(); }
function adjustBoth(f) { adjustLin(f); adjustAng(f); }

function updateSpeedDisplay() {
  document.getElementById('d-val-lin').textContent = linSpeed.toFixed(2);
  document.getElementById('d-val-ang').textContent = angSpeed.toFixed(2);
}

function publishDrive() {
  if (!cmdVelTopic) return;
  const lin = (held.forward  ? 1 : 0) - (held.backward ? 1 : 0);
  const ang = (held.left     ? 1 : 0) - (held.right    ? 1 : 0);
  cmdVelTopic.publish(new ROSLIB.Message({
    linear:  { x: lin * linSpeed, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: ang * angSpeed },
  }));
}

function sendStop() {
  if (!cmdVelTopic) return;
  cmdVelTopic.publish(new ROSLIB.Message({
    linear: { x:0,y:0,z:0 }, angular: { x:0,y:0,z:0 },
  }));
}

function clearHeld() {
  Object.keys(held).forEach(k => held[k] = false);
  document.querySelectorAll('.dpad-btn').forEach(b => b.classList.remove('pressed'));
}

function emergencyStop() {
  clearHeld(); sendStop();
  const btn = document.getElementById('d-estop');
  btn.classList.add('fired');
  setTimeout(() => btn.classList.remove('fired'), 300);
}

// ── D-pad buttons ──────────────────────────────────────────────────────────

const DIR_MAP = {
  'dpad-fwd':   'forward',
  'dpad-back':  'backward',
  'dpad-left':  'left',
  'dpad-right': 'right',
};

Object.entries(DIR_MAP).forEach(([id, dir]) => {
  const btn = document.getElementById(id);
  const press   = e => { e.preventDefault(); held[dir] = true;  btn.classList.add('pressed'); };
  const release = e => { e.preventDefault(); held[dir] = false; btn.classList.remove('pressed'); };
  btn.addEventListener('pointerdown',   press);
  btn.addEventListener('pointerup',     release);
  btn.addEventListener('pointerleave',  release);
  btn.addEventListener('pointercancel', release);
});

document.getElementById('dpad-stop').addEventListener('pointerdown', e => {
  e.preventDefault(); emergencyStop();
});
document.getElementById('d-estop').addEventListener('click', emergencyStop);

document.getElementById('d-lin-inc').addEventListener('click', () => adjustLin(STEP));
document.getElementById('d-lin-dec').addEventListener('click', () => adjustLin(1/STEP));
document.getElementById('d-ang-inc').addEventListener('click', () => adjustAng(STEP));
document.getElementById('d-ang-dec').addEventListener('click', () => adjustAng(1/STEP));

// ── Keyboard ───────────────────────────────────────────────────────────────

const KEY_DIR = {
  w: 'forward',  ArrowUp:    'forward',
  s: 'backward', ArrowDown:  'backward',
  a: 'left',     ArrowLeft:  'left',
  d: 'right',    ArrowRight: 'right',
};

const SPEED_KEYS = {
  q: () => adjustBoth(STEP),  z: () => adjustBoth(1/STEP),
  '=': () => adjustLin(STEP), '-': () => adjustLin(1/STEP),
  e: () => adjustAng(STEP),   c: () => adjustAng(1/STEP),
};

document.addEventListener('keydown', e => {
  if (activeTab !== 'drive' || e.repeat) return;
  if (e.key === ' ' || e.key === 'k' || e.key === 'K') {
    e.preventDefault(); emergencyStop(); return;
  }
  const speedFn = SPEED_KEYS[e.key] || SPEED_KEYS[e.key.toLowerCase()];
  if (speedFn) { e.preventDefault(); speedFn(); return; }
  const dir = KEY_DIR[e.key] || KEY_DIR[e.key.toLowerCase()];
  if (dir) {
    e.preventDefault();
    held[dir] = true;
    const id = Object.entries(DIR_MAP).find(([, d]) => d === dir)?.[0];
    if (id) document.getElementById(id)?.classList.add('pressed');
  }
});

document.addEventListener('keyup', e => {
  if (activeTab !== 'drive') return;
  const dir = KEY_DIR[e.key] || KEY_DIR[e.key.toLowerCase()];
  if (dir) {
    held[dir] = false;
    const id = Object.entries(DIR_MAP).find(([, d]) => d === dir)?.[0];
    if (id) document.getElementById(id)?.classList.remove('pressed');
  }
});

window.addEventListener('blur', () => { if (activeTab === 'drive') { clearHeld(); sendStop(); } });

// ── Init ───────────────────────────────────────────────────────────────────

async function init() {
  try { waypoints = await fetch('/api/waypoints').then(r => r.json()); } catch (_) {}
  renderSidebarList();
  connect();
}

document.addEventListener('DOMContentLoaded', init);
window.addEventListener('resize', () => { renderDots(); syncScanCanvas(); });
