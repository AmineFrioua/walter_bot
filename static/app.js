/* Walter — unified UI */

const ROSBRIDGE = `ws://${location.hostname}:9090`;
const STEP = 1.1, LIN_MAX = 0.20, LIN_MIN = 0.01, ANG_MAX = 1.5, ANG_MIN = 0.05;

// ── State ──────────────────────────────────────────────────────────────────

const S = {
  mode: 'mapping',
  ros: null,
  mapMeta: null,
  waypoints: {},          // key → {x,y,theta,label,type,number,width,depth}
  robotPose: {x:0,y:0,theta:0},
  odomSub: null,
  scanSub: null,
  scanActive: false,      // accumulate mode on
  arcFwd: false,
  cmdVel: null,
  driveLoop: null,
  held: {fwd:false,back:false,left:false,right:false},
  linSpeed: 0.05,
  angSpeed: 0.50,
  editorTool: 'table',
  selected: null,         // key of selected editor item
  drag: null,             // {key, startX, startY}
  deliveryTable: null,    // key of selected delivery table
  delivering: false,
  goalPub: null,
  pendingMapClick: null,  // {x,y} in map coords
};

// ── ROS ────────────────────────────────────────────────────────────────────

function connect() {
  S.ros = new ROSLIB.Ros({ url: ROSBRIDGE });

  S.ros.on('connection', () => {
    document.getElementById('ros-dot').className = 'ok';
    document.getElementById('ros-dot').title = 'Connected';
    S.cmdVel = new ROSLIB.Topic({
      ros: S.ros, name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist',
    });
    S.goalPub = new ROSLIB.Topic({
      ros: S.ros, name: '/goal_pose',
      messageType: 'geometry_msgs/PoseStamped',
    });
    subscribeMap();
    subscribeOdom();
    if (!S.driveLoop) S.driveLoop = setInterval(publishDrive, 100);
    // Restart active scan if reconnecting
    if (S.scanActive) startScan('accumulate');
    else if (S.mode === 'mapping' || S.mode === 'drive') startScan('live');
  });

  S.ros.on('error', () => {
    document.getElementById('ros-dot').className = 'err';
    document.getElementById('ros-dot').title = 'Error';
  });

  S.ros.on('close', () => {
    document.getElementById('ros-dot').className = '';
    document.getElementById('ros-dot').title = 'Disconnected';
    clearInterval(S.driveLoop); S.driveLoop = null;
    stopScan();
    setTimeout(connect, 3000);
  });
}

// ── Map ────────────────────────────────────────────────────────────────────

function subscribeMap() {
  const t = new ROSLIB.Topic({
    ros: S.ros, name: '/map',
    messageType: 'nav_msgs/OccupancyGrid',
    throttle_rate: 2000,
  });
  t.subscribe(msg => {
    S.mapMeta = msg.info;
    drawMap(msg);
    document.getElementById('map-placeholder').classList.add('hidden');
    document.getElementById('map-info').textContent =
      `${msg.info.width} × ${msg.info.height}  ·  ${msg.info.resolution.toFixed(3)} m/px`;
  });
}

function drawMap(msg) {
  const canvas = document.getElementById('map-canvas');
  const {width, height, data} = msg;
  canvas.width = width;
  canvas.height = height;
  const ctx = canvas.getContext('2d');
  const img = ctx.createImageData(width, height);
  for (let i = 0; i < data.length; i++) {
    const v = data[i];
    let r, g, b;
    if      (v === -1) { r=70;  g=70;  b=90;  }
    else if (v ===  0) { r=210; g=210; b=210; }
    else               { r=20;  g=20;  b=30;  }
    const px = (height - 1 - Math.floor(i / width)) * width + (i % width);
    img.data[px*4]=r; img.data[px*4+1]=g; img.data[px*4+2]=b; img.data[px*4+3]=255;
  }
  ctx.putImageData(img, 0, 0);
  renderOverlay();
  syncScanCanvas();
}

// ── Odom ───────────────────────────────────────────────────────────────────

function subscribeOdom() {
  if (S.odomSub) return;
  S.odomSub = new ROSLIB.Topic({
    ros: S.ros, name: '/odom',
    messageType: 'nav_msgs/Odometry', throttle_rate: 100,
  });
  S.odomSub.subscribe(msg => {
    const p = msg.pose.pose.position;
    const q = msg.pose.pose.orientation;
    S.robotPose = { x: p.x, y: p.y, theta: 2 * Math.atan2(q.z, q.w) };
    updateRobotMarker();
  });
}

function updateRobotMarker() {
  if (!S.mapMeta) return;
  const {left, top} = mapToCanvas(S.robotPose.x, S.robotPose.y);
  const m = document.getElementById('robot-marker');
  m.style.left = left + 'px';
  m.style.top  = top  + 'px';
  m.style.display = 'block';
}

// ── Coordinate helpers ─────────────────────────────────────────────────────

function mapToCanvas(mx, my) {
  const wrap = document.getElementById('map-area');
  const scaleX = wrap.clientWidth  / S.mapMeta.width;
  const scaleY = wrap.clientHeight / S.mapMeta.height;
  const px = (mx - S.mapMeta.origin.position.x) / S.mapMeta.resolution;
  const py = (my - S.mapMeta.origin.position.y) / S.mapMeta.resolution;
  return { left: px * scaleX, top: (S.mapMeta.height - py) * scaleY };
}

function canvasToMap(cx, cy) {
  const wrap = document.getElementById('map-area');
  const scaleX = S.mapMeta.width  / wrap.clientWidth;
  const scaleY = S.mapMeta.height / wrap.clientHeight;
  return {
    x: cx * scaleX * S.mapMeta.resolution + S.mapMeta.origin.position.x,
    y: (S.mapMeta.height - cy * scaleY) * S.mapMeta.resolution + S.mapMeta.origin.position.y,
  };
}

function mapToCanvasPx(mx, my) {
  if (!S.mapMeta) return null;
  const px = (mx - S.mapMeta.origin.position.x) / S.mapMeta.resolution;
  const py = (my - S.mapMeta.origin.position.y) / S.mapMeta.resolution;
  return { x: Math.round(px), y: Math.round(S.mapMeta.height - py) };
}

// ── Scan canvas ────────────────────────────────────────────────────────────

function syncScanCanvas() {
  const scan = document.getElementById('scan-canvas');
  if (S.mapMeta) {
    const map = document.getElementById('map-canvas');
    if (scan.width  !== map.width)  scan.width  = map.width;
    if (scan.height !== map.height) scan.height = map.height;
  } else {
    const wrap = document.getElementById('map-area');
    if (scan.width  !== wrap.clientWidth)  scan.width  = wrap.clientWidth;
    if (scan.height !== wrap.clientHeight) scan.height = wrap.clientHeight;
  }
}

function clearScanCanvas() {
  const c = document.getElementById('scan-canvas');
  c.getContext('2d').clearRect(0, 0, c.width, c.height);
}

function startScan(mode) {
  stopScan();
  if (!S.ros) return;
  S.scanSub = new ROSLIB.Topic({
    ros: S.ros, name: '/scan_filtered',
    messageType: 'sensor_msgs/LaserScan',
    throttle_rate: mode === 'live' ? 120 : 400,
  });
  S.scanSub.subscribe(msg => {
    syncScanCanvas();
    const canvas = document.getElementById('scan-canvas');
    const ctx = canvas.getContext('2d');
    if (mode === 'live') {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      if (!S.mapMeta) { drawLocalRadar(ctx, canvas, msg); return; }
    }
    if (!S.mapMeta) return;
    ctx.fillStyle = mode === 'live' ? 'rgba(96,165,250,.7)' : '#3b82f6';
    const {x:rx, y:ry, theta} = S.robotPose;
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
  if (S.scanSub) { S.scanSub.unsubscribe(); S.scanSub = null; }
}

function drawLocalRadar(ctx, canvas, msg) {
  const cx = canvas.width / 2, cy = canvas.height / 2;
  const maxR = Math.min(msg.range_max || 6, 6);
  const scale = Math.min(cx, cy) * 0.85 / maxR;
  ctx.strokeStyle = 'rgba(255,255,255,.05)';
  ctx.lineWidth = 1;
  for (let ring = 1; ring <= maxR; ring++) {
    ctx.beginPath(); ctx.arc(cx, cy, ring * scale, 0, Math.PI * 2); ctx.stroke();
  }
  ctx.fillStyle = 'rgba(96,165,250,.8)';
  for (let i = 0; i < msg.ranges.length; i++) {
    const r = msg.ranges[i];
    if (!isFinite(r) || r < 0.05 || r > msg.range_max) continue;
    const angle = msg.angle_min + i * msg.angle_increment;
    ctx.fillRect(
      Math.round(cx + r * Math.cos(angle) * scale),
      Math.round(cy - r * Math.sin(angle) * scale), 2, 2);
  }
  ctx.fillStyle = '#3b82f6';
  ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI*2); ctx.fill();
  ctx.strokeStyle = 'rgba(96,165,250,.4)';
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - 18); ctx.stroke();
}

// ── Overlay rendering (dots, rects, labels) ────────────────────────────────

function renderOverlay() {
  const layer = document.getElementById('overlay-layer');
  layer.innerHTML = '';
  if (!S.mapMeta) return;
  const wrap = document.getElementById('map-area');

  Object.entries(S.waypoints).forEach(([key, wp]) => {
    const {left, top} = mapToCanvas(wp.x, wp.y);

    if (wp.type === 'obstacle') {
      const wPx = (wp.width  || 1.0) / S.mapMeta.resolution * (wrap.clientWidth  / S.mapMeta.width);
      const dPx = (wp.depth  || 1.0) / S.mapMeta.resolution * (wrap.clientHeight / S.mapMeta.height);
      const box = document.createElement('div');
      box.className = 'obstacle-rect' + (S.selected === key ? ' selected' : '');
      box.style.cssText = `left:${left - wPx/2}px;top:${top - dPx/2}px;width:${wPx}px;height:${dPx}px`;
      box.dataset.key = key;
      const lbl = document.createElement('div');
      lbl.className = 'obstacle-label';
      lbl.textContent = wp.label || key;
      box.appendChild(lbl);
      box.addEventListener('pointerdown', onDotPointerDown);
      layer.appendChild(box);
      return;
    }

    // dot (table or origin)
    const dot = document.createElement('div');
    dot.className = `map-dot ${wp.type || 'table'}` + (S.selected === key ? ' selected' : '');
    if (S.delivering && S.deliveryTable === key) dot.classList.add('active-delivery');
    dot.style.left = left + 'px';
    dot.style.top  = top  + 'px';
    dot.dataset.key = key;
    dot.addEventListener('pointerdown', onDotPointerDown);
    layer.appendChild(dot);

    // table size rect
    if ((wp.type === 'table' || !wp.type) && S.mapMeta) {
      const wPx = (wp.width || 0.8) / S.mapMeta.resolution * (wrap.clientWidth  / S.mapMeta.width);
      const dPx = (wp.depth || 0.6) / S.mapMeta.resolution * (wrap.clientHeight / S.mapMeta.height);
      const rect = document.createElement('div');
      rect.className = 'table-rect' + (S.selected === key ? ' selected' : '');
      rect.style.cssText = `left:${left - wPx/2}px;top:${top - dPx/2}px;width:${wPx}px;height:${dPx}px`;
      layer.appendChild(rect);
    }

    const label = document.createElement('div');
    label.className = 'map-label';
    label.style.left = left + 'px';
    label.style.top  = top  + 'px';
    label.textContent = wp.label || key;
    layer.appendChild(label);
  });
}

// ── Drag handling ──────────────────────────────────────────────────────────

function onDotPointerDown(e) {
  if (S.mode !== 'editor') return;
  e.preventDefault();
  e.stopPropagation();
  const key = e.currentTarget.dataset.key;
  S.drag = { key, moved: false };
  S.selected = key;
  e.currentTarget.setPointerCapture(e.pointerId);
  renderOverlay();
  renderEditorForm();
  renderEditorList();
}

document.getElementById('overlay-layer').addEventListener('pointermove', e => {
  if (!S.drag) return;
  S.drag.moved = true;
  const rect = document.getElementById('map-area').getBoundingClientRect();
  const cx = e.clientX - rect.left;
  const cy = e.clientY - rect.top;
  const wp = S.waypoints[S.drag.key];
  if (!wp || !S.mapMeta) return;
  const {x, y} = canvasToMap(cx, cy);
  wp.x = x; wp.y = y;
  renderOverlay();
});

document.getElementById('overlay-layer').addEventListener('pointerup', e => {
  if (!S.drag) return;
  if (S.drag.moved) saveWaypoint(S.drag.key);
  S.drag = null;
});

// ── Waypoints CRUD ─────────────────────────────────────────────────────────

async function loadWaypoints() {
  try { S.waypoints = await fetch('/api/waypoints').then(r => r.json()); }
  catch (_) { S.waypoints = {}; }
}

async function saveWaypoint(key) {
  const wp = S.waypoints[key];
  try {
    const r = await fetch(`/api/waypoints/${encodeURIComponent(key)}`, {
      method: 'PUT',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(wp),
    });
    S.waypoints[key] = await r.json();
  } catch (e) { console.error('save failed', e); }
  renderOverlay();
  renderEditorList();
  renderDeliveryList();
}

async function deleteWaypoint(key) {
  try { await fetch(`/api/waypoints/${encodeURIComponent(key)}`, {method:'DELETE'}); }
  catch (_) {}
  delete S.waypoints[key];
  if (S.selected === key) S.selected = null;
  renderOverlay();
  renderEditorList();
  renderDeliveryList();
  document.getElementById('edit-form-wrap').innerHTML = '';
}

// ── Map click handling ─────────────────────────────────────────────────────

document.getElementById('map-area').addEventListener('click', e => {
  if (S.mode !== 'editor' || !S.mapMeta || S.drag?.moved) return;
  const rect = e.currentTarget.getBoundingClientRect();
  const {x, y} = canvasToMap(e.clientX - rect.left, e.clientY - rect.top);

  if (S.editorTool === 'origin') {
    // Immediately place/move origin
    S.waypoints['origin'] = {x, y, theta:0, label:'Origin', type:'origin'};
    saveWaypoint('origin');
    return;
  }

  // Table or obstacle → open modal
  S.pendingMapClick = {x, y};
  openModal(S.editorTool);
});

// ── Modal ──────────────────────────────────────────────────────────────────

function openModal(type, existingKey) {
  const modal = document.getElementById('modal-backdrop');
  const extras = document.getElementById('m-table-extra');
  document.getElementById('modal-title').textContent =
    existingKey ? `Edit ${type}` : `Add ${type}`;
  extras.style.display = type === 'table' ? '' : 'none';

  const wp = existingKey ? S.waypoints[existingKey] : null;
  document.getElementById('m-name').value   = wp?.label   || '';
  document.getElementById('m-number').value = wp?.number  || '';
  document.getElementById('m-width').value  = wp?.width   || '0.8';
  document.getElementById('m-depth').value  = wp?.depth   || '0.6';
  document.getElementById('m-name').placeholder = type === 'table' ? 'Table 1' : 'Restroom zone';

  modal._editKey = existingKey || null;
  modal._editType = type;
  modal.classList.add('open');
  document.getElementById('m-name').focus();
}

function closeModal() {
  document.getElementById('modal-backdrop').classList.remove('open');
  S.pendingMapClick = null;
}

document.getElementById('modal-cancel').addEventListener('click', closeModal);
document.getElementById('modal-backdrop').addEventListener('click', e => {
  if (e.target === document.getElementById('modal-backdrop')) closeModal();
});

document.getElementById('modal-ok').addEventListener('click', async () => {
  const name   = document.getElementById('m-name').value.trim();
  if (!name) return;
  const modal  = document.getElementById('modal-backdrop');
  const type   = modal._editType;
  const editKey = modal._editKey;
  const key = editKey || name.toLowerCase().replace(/\s+/g, '_') + '_' + Date.now().toString(36);

  const pos = editKey ? {x: S.waypoints[editKey].x, y: S.waypoints[editKey].y}
                      : S.pendingMapClick || {x:0, y:0};

  S.waypoints[key] = {
    x: pos.x, y: pos.y, theta: 0,
    label: name, type,
    ...(type === 'table' ? {
      number: parseInt(document.getElementById('m-number').value) || 1,
      width:  parseFloat(document.getElementById('m-width').value) || 0.8,
      depth:  parseFloat(document.getElementById('m-depth').value) || 0.6,
    } : {
      width:  1.0,
      depth:  1.0,
    }),
  };

  closeModal();
  await saveWaypoint(key);
  S.selected = key;
  renderOverlay();
  renderEditorList();
  renderEditorForm();
});

document.getElementById('m-name').addEventListener('keydown', e => {
  if (e.key === 'Enter')  document.getElementById('modal-ok').click();
  if (e.key === 'Escape') closeModal();
});

// ── Editor panel ───────────────────────────────────────────────────────────

function renderEditorList() {
  const list = document.getElementById('editor-item-list');
  list.innerHTML = '';
  const entries = Object.entries(S.waypoints);
  if (!entries.length) {
    list.innerHTML = '<p style="color:#1e2537;font-size:.75rem;padding:6px 0">No items yet</p>';
    return;
  }
  entries.forEach(([key, wp]) => {
    const row = document.createElement('div');
    row.className = 'ed-item' + (S.selected === key ? ' selected' : '');
    row.innerHTML = `
      <div class="ed-dot ${wp.type || 'table'}"></div>
      <span class="ed-name">${wp.label || key}</span>
      <button class="ed-del" data-key="${key}" title="Delete">✕</button>
    `;
    row.addEventListener('click', e => {
      if (e.target.classList.contains('ed-del')) return;
      S.selected = key;
      renderOverlay(); renderEditorList(); renderEditorForm();
    });
    row.querySelector('.ed-del').addEventListener('click', () => deleteWaypoint(key));
    list.appendChild(row);
  });
}

function renderEditorForm() {
  const wrap = document.getElementById('edit-form-wrap');
  if (!S.selected || !S.waypoints[S.selected]) { wrap.innerHTML = ''; return; }
  const wp = S.waypoints[S.selected];
  const isTable = wp.type === 'table' || !wp.type;
  wrap.innerHTML = `
    <div class="edit-form">
      <div class="form-row">
        <span class="form-lbl">Name</span>
        <input class="form-inp" id="ef-name" value="${wp.label || ''}" maxlength="30">
      </div>
      ${isTable ? `
      <div class="form-row">
        <span class="form-lbl">Number</span>
        <input class="form-inp" id="ef-num" type="number" value="${wp.number || 1}" min="1">
      </div>
      <div class="form-row">
        <span class="form-lbl">Width</span>
        <input class="form-inp" id="ef-w" type="number" step="0.1" value="${wp.width || 0.8}">
        <span style="font-size:.65rem;color:#334155">m</span>
      </div>
      <div class="form-row">
        <span class="form-lbl">Depth</span>
        <input class="form-inp" id="ef-d" type="number" step="0.1" value="${wp.depth || 0.6}">
        <span style="font-size:.65rem;color:#334155">m</span>
      </div>` : ''}
      <div class="btn-row">
        <button class="btn btn-primary" id="ef-save">Save</button>
        <button class="btn btn-danger"  id="ef-del">Delete</button>
      </div>
    </div>`;

  document.getElementById('ef-save').addEventListener('click', () => {
    wp.label = document.getElementById('ef-name').value.trim() || wp.label;
    if (isTable) {
      wp.number = parseInt(document.getElementById('ef-num').value) || wp.number;
      wp.width  = parseFloat(document.getElementById('ef-w').value) || wp.width;
      wp.depth  = parseFloat(document.getElementById('ef-d').value) || wp.depth;
    }
    saveWaypoint(S.selected);
    renderEditorList();
    renderDeliveryList();
    renderOverlay();
  });
  document.getElementById('ef-del').addEventListener('click', () => deleteWaypoint(S.selected));
}

// Editor tool buttons
document.querySelectorAll('.tool-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tool-btn').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    S.editorTool = btn.dataset.tool;
    S.selected = null;
    renderOverlay();
    renderEditorList();
    document.getElementById('edit-form-wrap').innerHTML = '';
    const hints = {
      table:    'Click on the map to place a table.',
      obstacle: 'Click on the map to add a no-go zone.',
      origin:   'Click on the map to set the robot\'s home/charging position.',
    };
    document.getElementById('editor-hint').textContent = hints[S.editorTool];
  });
});

// ── Delivery panel ─────────────────────────────────────────────────────────

function renderDeliveryList() {
  const list = document.getElementById('delivery-table-list');
  list.innerHTML = '';
  const tables = Object.entries(S.waypoints).filter(([,wp]) => wp.type === 'table' || !wp.type);
  if (!tables.length) {
    list.innerHTML = '<p style="color:#1e2537;font-size:.75rem;padding:6px 0">No tables defined yet</p>';
    return;
  }
  tables.sort(([,a],[,b]) => (a.number||99) - (b.number||99));
  tables.forEach(([key, wp]) => {
    const item = document.createElement('div');
    item.className = 'tbl-item' + (S.deliveryTable === key ? ' selected' : '');
    item.innerHTML = `
      <div class="tbl-num">${wp.number || '?'}</div>
      <div class="tbl-meta">
        <div class="tbl-name">${wp.label || key}</div>
        <div class="tbl-dims">${wp.width || 0.8}m × ${wp.depth || 0.6}m</div>
      </div>`;
    item.addEventListener('click', () => {
      if (S.delivering) return;
      S.deliveryTable = S.deliveryTable === key ? null : key;
      renderDeliveryList();
      document.getElementById('send-btn').disabled = !S.deliveryTable;
      renderOverlay();
    });
    list.appendChild(item);
  });
}

document.getElementById('send-btn').addEventListener('click', () => {
  if (!S.deliveryTable || !S.waypoints[S.deliveryTable]) return;
  sendNavGoal(S.waypoints[S.deliveryTable]);
  S.delivering = true;
  setDeliveryStatus('moving', 'Navigating to table…');
  document.getElementById('send-btn').style.display   = 'none';
  document.getElementById('cancel-btn').style.display = '';
  renderOverlay();
});

document.getElementById('cancel-btn').addEventListener('click', cancelDelivery);

function sendNavGoal(wp) {
  if (!S.goalPub) return;
  S.goalPub.publish(new ROSLIB.Message({
    header: { frame_id: 'map' },
    pose: {
      position: { x: wp.x, y: wp.y, z: 0 },
      orientation: {
        x: 0, y: 0,
        z: Math.sin((wp.theta || 0) / 2),
        w: Math.cos((wp.theta || 0) / 2),
      },
    },
  }));
}

function cancelDelivery() {
  // Send cancel via rosbridge action or just stop and go home
  if (S.cmdVel) S.cmdVel.publish(new ROSLIB.Message({
    linear: {x:0,y:0,z:0}, angular: {x:0,y:0,z:0},
  }));
  const origin = S.waypoints['origin'];
  if (origin) sendNavGoal(origin);
  S.delivering = false;
  S.deliveryTable = null;
  setDeliveryStatus('idle', 'Idle');
  document.getElementById('send-btn').style.display   = '';
  document.getElementById('cancel-btn').style.display = 'none';
  document.getElementById('send-btn').disabled = true;
  renderDeliveryList();
  renderOverlay();
}

function setDeliveryStatus(state, text) {
  const dot  = document.getElementById('status-dot');
  const span = document.getElementById('status-text');
  dot.className  = 'status-dot ' + state;
  span.textContent = text;
}

// Subscribe to nav2 result feedback
function subscribeNavStatus() {
  if (!S.ros) return;
  const vel = new ROSLIB.Topic({
    ros: S.ros, name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist', throttle_rate: 300,
  });
  let lastMoving = Date.now();
  vel.subscribe(msg => {
    if (!S.delivering) return;
    const moving = Math.abs(msg.linear.x) > 0.01 || Math.abs(msg.angular.z) > 0.05;
    if (moving) {
      lastMoving = Date.now();
      setDeliveryStatus('moving', 'Navigating…');
    } else if (Date.now() - lastMoving > 1500) {
      setDeliveryStatus('arrived', 'Arrived!');
    }
  });
}

// ── Drive (shared between mapping and drive modes) ─────────────────────────

function publishDrive() {
  if (!S.cmdVel || !['mapping','drive'].includes(S.mode)) return;
  const lin = (S.held.fwd ? 1 : 0) - (S.held.back ? 1 : 0);
  const ang = (S.held.left ? 1 : 0) - (S.held.right ? 1 : 0);
  S.cmdVel.publish(new ROSLIB.Message({
    linear:  {x: lin * S.linSpeed, y:0, z:0},
    angular: {x:0, y:0, z: ang * S.angSpeed},
  }));
}

function sendStop() {
  if (!S.cmdVel) return;
  S.cmdVel.publish(new ROSLIB.Message({
    linear: {x:0,y:0,z:0}, angular: {x:0,y:0,z:0},
  }));
}

function clearHeld() {
  Object.keys(S.held).forEach(k => S.held[k] = false);
  document.querySelectorAll('.dpad-btn').forEach(b => b.classList.remove('pressed'));
}

function emergencyStop() {
  clearHeld(); sendStop();
  document.querySelectorAll('.estop-btn').forEach(b => {
    b.classList.add('fired');
    setTimeout(() => b.classList.remove('fired'), 300);
  });
}

function updateSpeedDisplay() {
  ['lin-val','lin2-val'].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.textContent = S.linSpeed.toFixed(2);
  });
  ['ang-val','ang2-val'].forEach(id => {
    const el = document.getElementById(id);
    if (el) el.textContent = S.angSpeed.toFixed(2);
  });
}

function clamp(v,lo,hi) { return Math.max(lo, Math.min(hi, v)); }
function adjustLin(f)  { S.linSpeed = clamp(S.linSpeed * f, LIN_MIN, LIN_MAX); updateSpeedDisplay(); }
function adjustAng(f)  { S.angSpeed = clamp(S.angSpeed * f, ANG_MIN, ANG_MAX); updateSpeedDisplay(); }

// Wire up all dpad buttons (both mapping and drive panels)
function wireDpad(fwdId, backId, leftId, rightId, stopId) {
  const map = { [fwdId]:'fwd', [backId]:'back', [leftId]:'left', [rightId]:'right' };
  Object.entries(map).forEach(([id, dir]) => {
    const btn = document.getElementById(id);
    if (!btn) return;
    const press   = e => { e.preventDefault(); S.held[dir] = true;  btn.classList.add('pressed'); };
    const release = e => { e.preventDefault(); S.held[dir] = false; btn.classList.remove('pressed'); };
    btn.addEventListener('pointerdown',   press);
    btn.addEventListener('pointerup',     release);
    btn.addEventListener('pointerleave',  release);
    btn.addEventListener('pointercancel', release);
  });
  document.getElementById(stopId)?.addEventListener('pointerdown', e => {
    e.preventDefault(); emergencyStop();
  });
}

wireDpad('dpad-fwd','dpad-back','dpad-left','dpad-right','dpad-stop');
wireDpad('drive-fwd','drive-back','drive-left','drive-right','drive-stop');

document.getElementById('map-estop').addEventListener('click', emergencyStop);
document.getElementById('drive-estop').addEventListener('click', emergencyStop);

// Speed buttons (both panels)
[['lin-inc','lin2-inc', () => adjustLin(STEP)],
 ['lin-dec','lin2-dec', () => adjustLin(1/STEP)],
 ['ang-inc','ang2-inc', () => adjustAng(STEP)],
 ['ang-dec','ang2-dec', () => adjustAng(1/STEP)]].forEach(([id1, id2, fn]) => {
  [id1, id2].forEach(id => document.getElementById(id)?.addEventListener('click', fn));
});

// Keyboard
const KEY_DIR = {
  w:'fwd', ArrowUp:'fwd', s:'back', ArrowDown:'back',
  a:'left', ArrowLeft:'left', d:'right', ArrowRight:'right',
};
const SPEED_KEYS = {
  q: () => { adjustLin(STEP); adjustAng(STEP); },
  z: () => { adjustLin(1/STEP); adjustAng(1/STEP); },
  '=': () => adjustLin(STEP), '-': () => adjustLin(1/STEP),
  e: () => adjustAng(STEP), c: () => adjustAng(1/STEP),
};

document.addEventListener('keydown', e => {
  if (!['mapping','drive'].includes(S.mode) || e.repeat) return;
  if (document.activeElement?.tagName === 'INPUT') return;
  if (e.key === ' ' || e.key === 'k' || e.key === 'K') { e.preventDefault(); emergencyStop(); return; }
  const sf = SPEED_KEYS[e.key] || SPEED_KEYS[e.key.toLowerCase()];
  if (sf) { e.preventDefault(); sf(); return; }
  const dir = KEY_DIR[e.key] || KEY_DIR[e.key.toLowerCase()];
  if (dir) { e.preventDefault(); S.held[dir] = true; }
});

document.addEventListener('keyup', e => {
  if (!['mapping','drive'].includes(S.mode)) return;
  const dir = KEY_DIR[e.key] || KEY_DIR[e.key.toLowerCase()];
  if (dir) S.held[dir] = false;
});

window.addEventListener('blur', () => { clearHeld(); sendStop(); });

// ── LiDAR arc toggle ──────────────────────────────────────────────────────

document.getElementById('arc-toggle').addEventListener('click', () => {
  S.arcFwd = !S.arcFwd;
  const btn = document.getElementById('arc-toggle');
  btn.textContent = S.arcFwd ? 'Fwd only' : 'Full scan';
  btn.classList.toggle('active', S.arcFwd);
  if (!S.ros) return;
  const svc = new ROSLIB.Service({
    ros: S.ros, name: '/lidar_filter/set_parameters',
    serviceType: 'rcl_interfaces/srv/SetParameters',
  });
  svc.callService({
    parameters: [{ name: 'forward_arc_deg', value: { type: 4, double_value: S.arcFwd ? 180.0 : 360.0 } }],
  }, res => {
    if (!res?.results?.[0]?.successful) btn.textContent += ' ?';
  });
});

// ── Scan toggle ───────────────────────────────────────────────────────────

document.getElementById('scan-toggle').addEventListener('click', () => {
  S.scanActive = !S.scanActive;
  const btn = document.getElementById('scan-toggle');
  btn.textContent = S.scanActive ? 'Scan: ON' : 'Scan';
  btn.classList.toggle('active', S.scanActive);
  document.getElementById('scan-clear').style.display = S.scanActive ? '' : 'none';
  if (S.scanActive) startScan('accumulate');
  else { stopScan(); clearScanCanvas(); startScan('live'); }
});

document.getElementById('scan-clear').addEventListener('click', () => {
  clearScanCanvas();
  if (S.scanActive) startScan('accumulate');
});

// ── Mode switching ─────────────────────────────────────────────────────────

function setMode(mode) {
  // Stop drive on mode exit
  if (['mapping','drive'].includes(S.mode)) { clearHeld(); sendStop(); }

  S.mode = mode;

  // Nav buttons
  document.querySelectorAll('.nav-btn').forEach(b =>
    b.classList.toggle('active', b.dataset.mode === mode));

  // Panels
  document.querySelectorAll('.panel-section').forEach(p => p.classList.remove('visible'));
  const panelId = {
    mapping: 'panel-mapping',
    editor:  'panel-editor',
    delivery:'panel-delivery',
    drive:   'panel-drive',
    face:    'panel-face',
  }[mode];
  document.getElementById(panelId)?.classList.add('visible');

  // Map area cursor
  const mapArea = document.getElementById('map-area');
  mapArea.classList.toggle('cursor-cross', mode === 'editor');

  // Show/hide map area for face (keep it visible for all non-face modes)
  // Face mode: map area stays but shows placeholder since it's just a link panel

  // Scan mode management
  if (mode === 'mapping' || mode === 'drive') {
    if (!S.scanActive) startScan('live');
  } else {
    if (!S.scanActive) { stopScan(); clearScanCanvas(); }
  }

  // Editor: clear selection on enter
  if (mode === 'editor') {
    S.selected = null;
    renderOverlay();
    renderEditorList();
    document.getElementById('edit-form-wrap').innerHTML = '';
  }

  // Delivery
  if (mode === 'delivery') {
    renderDeliveryList();
    renderOverlay();
  }
}

document.querySelectorAll('.nav-btn').forEach(btn => {
  btn.addEventListener('click', () => setMode(btn.dataset.mode));
});

// ── Init ───────────────────────────────────────────────────────────────────

async function init() {
  await loadWaypoints();
  renderEditorList();
  renderDeliveryList();
  connect();
  subscribeNavStatus();
}

document.addEventListener('DOMContentLoaded', init);
window.addEventListener('resize', () => { renderOverlay(); syncScanCanvas(); updateRobotMarker(); });
