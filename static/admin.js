/* Admin UI — place / remove table waypoints by clicking the map */

const ROSBRIDGE_URL = `ws://${location.hostname}:9090`;

let ros, mapMeta, waypoints = {};
let pendingClick = null;   // { x, y } in map coords, waiting for name modal

// ── ROS connection ─────────────────────────────────────────────────────────

function connect() {
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });
  ros.on('connection', subscribeMap);
  ros.on('close', () => setTimeout(connect, 3000));
}

function subscribeMap() {
  const mapTopic = new ROSLIB.Topic({
    ros, name: '/map',
    messageType: 'nav_msgs/OccupancyGrid',
    throttle_rate: 2000,
  });
  mapTopic.subscribe(msg => {
    mapMeta = msg.info;
    drawMap(msg);
  });
}

// ── Map render ─────────────────────────────────────────────────────────────

function drawMap(msg) {
  const canvas = document.getElementById('map-canvas');
  const { width, height, data } = msg;
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext('2d');
  const img = ctx.createImageData(width, height);

  for (let i = 0; i < data.length; i++) {
    const v = data[i];
    let r, g, b;
    if (v === -1)      { r = 100; g = 100; b = 120; }
    else if (v === 0)  { r = 230; g = 230; b = 230; }
    else               { r = 30;  g = 30;  b = 40;  }
    const px = (height - 1 - Math.floor(i / width)) * width + (i % width);
    img.data[px * 4]     = r;
    img.data[px * 4 + 1] = g;
    img.data[px * 4 + 2] = b;
    img.data[px * 4 + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
  renderDots();
}

// ── Coordinate helpers ─────────────────────────────────────────────────────

function canvasToMap(cx, cy) {
  const wrap   = document.getElementById('map-wrap');
  const scaleX = mapMeta.width  / wrap.clientWidth;
  const scaleY = mapMeta.height / wrap.clientHeight;
  const px = cx * scaleX;
  const py = mapMeta.height - cy * scaleY;
  const mx = px * mapMeta.resolution + mapMeta.origin.position.x;
  const my = py * mapMeta.resolution + mapMeta.origin.position.y;
  return { x: mx, y: my };
}

function mapToCanvas(mx, my) {
  const wrap   = document.getElementById('map-wrap');
  const scaleX = wrap.clientWidth  / mapMeta.width;
  const scaleY = wrap.clientHeight / mapMeta.height;
  const px = (mx - mapMeta.origin.position.x) / mapMeta.resolution;
  const py = (my - mapMeta.origin.position.y) / mapMeta.resolution;
  return {
    left: px * scaleX,
    top:  (mapMeta.height - py) * scaleY,
  };
}

// ── Dots overlay ───────────────────────────────────────────────────────────

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
    dot.addEventListener('click', e => {
      e.stopPropagation();
      removeWaypoint(name);
    });
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
  const list = document.getElementById('table-list');
  list.innerHTML = '';

  const names = Object.keys(waypoints);
  if (names.length === 0) {
    list.innerHTML = '<p style="color:#64748b;font-size:0.8rem;padding:8px">No tables yet. Click the map to add one.</p>';
    return;
  }

  names.forEach(name => {
    const wp = waypoints[name];
    const row = document.createElement('div');
    row.className = 'table-row';
    row.innerHTML = `
      <span>${wp.label || name}</span>
      <span class="coords">${wp.x.toFixed(2)}, ${wp.y.toFixed(2)}</span>
      <button class="btn btn-danger" style="padding:4px 10px;font-size:0.75rem"
        data-name="${name}">✕</button>
    `;
    row.querySelector('button').addEventListener('click', () => removeWaypoint(name));
    list.appendChild(row);
  });
}

// ── Waypoint CRUD ──────────────────────────────────────────────────────────

async function addWaypoint(name, x, y) {
  const label = name;
  const key   = name.toLowerCase().replace(/\s+/g, '_');
  try {
    const resp = await fetch(`/api/waypoints/${encodeURIComponent(key)}`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y, theta: 0.0, label }),
    });
    waypoints[key] = await resp.json();
    renderDots();
    renderSidebarList();
  } catch (e) {
    alert('Failed to save waypoint: ' + e);
  }
}

async function removeWaypoint(name) {
  try {
    await fetch(`/api/waypoints/${encodeURIComponent(name)}`, { method: 'DELETE' });
    delete waypoints[name];
    renderDots();
    renderSidebarList();
  } catch (e) {
    alert('Failed to delete waypoint: ' + e);
  }
}

// ── Map click ──────────────────────────────────────────────────────────────

document.getElementById('map-wrap').addEventListener('click', e => {
  if (!mapMeta) return;
  const rect = e.currentTarget.getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  pendingClick = canvasToMap(cx, cy);
  openModal();
});

// ── Modal ──────────────────────────────────────────────────────────────────

function openModal() {
  const backdrop = document.getElementById('modal-backdrop');
  const input    = document.getElementById('modal-input');
  backdrop.classList.add('open');
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
  if (!name) return;
  const { x, y } = pendingClick;
  closeModal();
  addWaypoint(name, x, y);
});

document.getElementById('modal-input').addEventListener('keydown', e => {
  if (e.key === 'Enter') document.getElementById('modal-ok').click();
  if (e.key === 'Escape') closeModal();
});

document.getElementById('modal-backdrop').addEventListener('click', e => {
  if (e.target === document.getElementById('modal-backdrop')) closeModal();
});

// ── Init ───────────────────────────────────────────────────────────────────

async function init() {
  try {
    waypoints = await fetch('/api/waypoints').then(r => r.json());
  } catch (_) {}
  renderSidebarList();
  connect();
}

document.addEventListener('DOMContentLoaded', init);
window.addEventListener('resize', renderDots);

// ── Scan overlay ───────────────────────────────────────────────────────────

let scanActive = false;
let odomSub = null, scanSub = null;
let robotOdomPose = { x: 0, y: 0, theta: 0 };

// Convert map-frame coords to raw canvas pixel position (not CSS pixels).
// The scan canvas shares dimensions with the map canvas.
function mapToCanvasPx(mx, my) {
  if (!mapMeta) return null;
  const px = (mx - mapMeta.origin.position.x) / mapMeta.resolution;
  const py = (my - mapMeta.origin.position.y) / mapMeta.resolution;
  return { x: Math.round(px), y: Math.round(mapMeta.height - py) };
}

// Keep scan canvas dimensions in sync with the map canvas.
// Changing width/height clears the canvas — only do it when size actually changes.
function syncScanCanvas() {
  const map  = document.getElementById('map-canvas');
  const scan = document.getElementById('scan-canvas');
  if (scan.width !== map.width)   scan.width  = map.width;
  if (scan.height !== map.height) scan.height = map.height;
}

function startScanOverlay() {
  if (odomSub) return;

  odomSub = new ROSLIB.Topic({
    ros, name: '/odom',
    messageType: 'nav_msgs/Odometry',
    throttle_rate: 100,
  });
  odomSub.subscribe(msg => {
    const p = msg.pose.pose.position;
    const q = msg.pose.pose.orientation;
    robotOdomPose = { x: p.x, y: p.y, theta: 2 * Math.atan2(q.z, q.w) };
  });

  scanSub = new ROSLIB.Topic({
    ros, name: '/scan_filtered',
    messageType: 'sensor_msgs/LaserScan',
    throttle_rate: 400,   // ~2.5 Hz — enough to see progress without flooding WS
  });
  scanSub.subscribe(msg => {
    if (!scanActive || !mapMeta) return;
    syncScanCanvas();

    const canvas = document.getElementById('scan-canvas');
    const ctx    = canvas.getContext('2d');
    ctx.fillStyle = '#3b82f6';

    const { x: rx, y: ry, theta } = robotOdomPose;

    for (let i = 0; i < msg.ranges.length; i++) {
      const r = msg.ranges[i];
      if (!isFinite(r) || r < 0.05 || r > msg.range_max) continue;

      const angle = msg.angle_min + i * msg.angle_increment + theta;
      const mx = rx + r * Math.cos(angle);
      const my = ry + r * Math.sin(angle);

      const pt = mapToCanvasPx(mx, my);
      if (!pt) continue;
      if (pt.x < 0 || pt.x >= canvas.width || pt.y < 0 || pt.y >= canvas.height) continue;

      // 2×2 pixel dot so points are visible at small map resolutions
      ctx.fillRect(pt.x, pt.y, 2, 2);
    }
  });
}

function stopScanOverlay() {
  if (odomSub) { odomSub.unsubscribe(); odomSub = null; }
  if (scanSub) { scanSub.unsubscribe(); scanSub = null; }
}

function clearScanOverlay() {
  const canvas = document.getElementById('scan-canvas');
  canvas.getContext('2d').clearRect(0, 0, canvas.width, canvas.height);
}

document.getElementById('scan-toggle').addEventListener('click', () => {
  scanActive = !scanActive;
  const btn = document.getElementById('scan-toggle');
  btn.textContent = scanActive ? 'Scan: ON' : 'Scan';
  btn.classList.toggle('active', scanActive);
  document.getElementById('scan-clear').style.display = scanActive ? '' : 'none';
  if (scanActive) startScanOverlay();
  else stopScanOverlay();
});

document.getElementById('scan-clear').addEventListener('click', clearScanOverlay);
