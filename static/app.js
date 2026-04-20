/* Delivery UI — talks to ROS via rosbridge ws://localhost:9090 */

const ROSBRIDGE_URL = `ws://${location.hostname}:9090`;
const HOME_POSE = { x: 0.0, y: 0.0, theta: 0.0 };

let ros, mapMeta, waypoints = {}, selectedTable = null;
let delivering = false, deliveryTimer = null;
let deliveryWaitS = 10;
let robotPose = null;

// ── ROS connection ─────────────────────────────────────────────────────────

function connect() {
  ros = new ROSLIB.Ros({ url: ROSBRIDGE_URL });

  ros.on('connection', () => {
    setStatus(true);
    subscribeMap();
    subscribeRobotPose();
  });

  ros.on('error', () => setStatus(false));
  ros.on('close', () => { setStatus(false); setTimeout(connect, 3000); });
}

function setStatus(ok) {
  document.getElementById('status-dot').className = ok ? 'connected' : '';
  document.getElementById('status-label').textContent = ok ? 'Connected' : 'Disconnected';
}

// ── Map subscription ───────────────────────────────────────────────────────

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
    if (v === -1)      { r = 100; g = 100; b = 120; }   // unknown — blue-grey
    else if (v === 0)  { r = 230; g = 230; b = 230; }   // free — light
    else               { r = 30;  g = 30;  b = 40;  }   // occupied — dark
    const px = (height - 1 - Math.floor(i / width)) * width + (i % width);
    img.data[px * 4]     = r;
    img.data[px * 4 + 1] = g;
    img.data[px * 4 + 2] = b;
    img.data[px * 4 + 3] = 255;
  }
  ctx.putImageData(img, 0, 0);
  renderOverlays();
}

// ── Robot pose ─────────────────────────────────────────────────────────────

function subscribeRobotPose() {
  const poseTopic = new ROSLIB.Topic({
    ros, name: '/amcl_pose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped',
    throttle_rate: 500,
  });
  poseTopic.subscribe(msg => {
    robotPose = msg.pose.pose;
    updateRobotMarker();
  });

  // fallback: listen to /odom if amcl not running
  const odomTopic = new ROSLIB.Topic({
    ros, name: '/odom',
    messageType: 'nav_msgs/Odometry',
    throttle_rate: 500,
  });
  odomTopic.subscribe(msg => {
    if (!robotPose) {
      robotPose = msg.pose.pose;
      updateRobotMarker();
    }
  });
}

function updateRobotMarker() {
  if (!robotPose || !mapMeta) return;
  const marker = document.getElementById('robot-marker');
  const { left, top } = mapToCanvas(robotPose.position.x, robotPose.position.y);
  marker.style.left = left + 'px';
  marker.style.top  = top  + 'px';
  marker.style.display = 'block';

  // Rotate arrow to match heading
  const q = robotPose.orientation;
  const theta = 2 * Math.atan2(q.z, q.w);
  marker.style.transform = `translate(-50%,-50%) rotate(${theta}rad)`;
}

// ── Coordinate conversion ──────────────────────────────────────────────────

function mapToCanvas(mx, my) {
  if (!mapMeta) return { left: 0, top: 0 };
  const canvas = document.getElementById('map-canvas');
  const wrap   = document.getElementById('map-wrap');
  const scaleX = wrap.clientWidth  / mapMeta.width;
  const scaleY = wrap.clientHeight / mapMeta.height;

  const px = (mx - mapMeta.origin.position.x) / mapMeta.resolution;
  const py = (my - mapMeta.origin.position.y) / mapMeta.resolution;

  // Canvas is Y-flipped
  const cx = px * scaleX;
  const cy = (mapMeta.height - py) * scaleY;
  return { left: cx, top: cy };
}

// ── Overlays (table dots on map) ───────────────────────────────────────────

function renderOverlays() {
  const wrap = document.getElementById('map-wrap');
  wrap.querySelectorAll('.table-dot').forEach(el => el.remove());

  Object.entries(waypoints).forEach(([name, wp]) => {
    const dot = document.createElement('div');
    dot.className = 'table-dot' + (name === selectedTable ? ' target' : '');
    const { left, top } = mapToCanvas(wp.x, wp.y);
    dot.style.left = left + 'px';
    dot.style.top  = top  + 'px';
    wrap.appendChild(dot);
  });
}

// ── Table list ─────────────────────────────────────────────────────────────

function renderTableList() {
  const list = document.getElementById('table-list');
  list.innerHTML = '';

  const names = Object.keys(waypoints);
  if (names.length === 0) {
    list.innerHTML = '<p style="color:#64748b;font-size:0.85rem">No tables configured yet.<br>Go to <a href="/admin" style="color:#3b82f6">Admin</a> to add them.</p>';
    return;
  }

  names.forEach(name => {
    const wp = waypoints[name];
    const btn = document.createElement('button');
    btn.className = 'table-btn' + (name === selectedTable ? ' selected' : '') +
                    (delivering && name === selectedTable ? ' delivering' : '');
    btn.innerHTML = `<span>${wp.label || name}</span>`;
    btn.addEventListener('click', () => {
      if (delivering) return;
      selectedTable = name;
      renderTableList();
      renderOverlays();
      document.getElementById('send-btn').disabled = false;
    });
    list.appendChild(btn);
  });
}

// ── Navigation ─────────────────────────────────────────────────────────────

function sendGoal(x, y, theta) {
  const goalTopic = new ROSLIB.Topic({
    ros, name: '/goal_pose',
    messageType: 'geometry_msgs/PoseStamped',
  });

  const qz = Math.sin(theta / 2);
  const qw = Math.cos(theta / 2);

  goalTopic.publish(new ROSLIB.Message({
    header: { frame_id: 'map', stamp: { secs: 0, nsecs: 0 } },
    pose: {
      position:    { x, y, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: qz, w: qw },
    },
  }));
}

function startDelivery() {
  if (!selectedTable || !waypoints[selectedTable]) return;
  const wp = waypoints[selectedTable];

  delivering = true;
  updateDeliveryUI('🚀 Delivering to ' + (wp.label || selectedTable) + '…');
  sendGoal(wp.x, wp.y, wp.theta);

  const btn = document.getElementById('send-btn');
  btn.textContent = 'Cancel';
  btn.className = 'cancel';
  btn.disabled = false;
  renderTableList();
  renderOverlays();

  // Crude timer — waits deliveryWaitS after we assume arrival and returns home.
  // A real implementation would watch /navigate_to_pose result.
  const estimatedTravelMs = 30000;  // 30s travel budget
  deliveryTimer = setTimeout(() => {
    updateDeliveryUI(`⏳ Arrived — waiting ${deliveryWaitS}s…`);
    deliveryTimer = setTimeout(() => {
      updateDeliveryUI('🏠 Returning home…');
      sendGoal(HOME_POSE.x, HOME_POSE.y, HOME_POSE.theta);
      deliveryTimer = setTimeout(() => {
        endDelivery('✅ Delivery complete!');
      }, 30000);
    }, deliveryWaitS * 1000);
  }, estimatedTravelMs);
}

function cancelDelivery() {
  if (deliveryTimer) { clearTimeout(deliveryTimer); deliveryTimer = null; }
  // Publish zero-velocity to stop robot immediately
  const cmdVel = new ROSLIB.Topic({ ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' });
  cmdVel.publish(new ROSLIB.Message({ linear: { x:0, y:0, z:0 }, angular: { x:0, y:0, z:0 } }));
  endDelivery('🛑 Cancelled');
}

function endDelivery(msg) {
  delivering = false;
  selectedTable = null;
  updateDeliveryUI(msg);
  const btn = document.getElementById('send-btn');
  btn.textContent = 'Send Walter';
  btn.className = '';
  btn.disabled = true;
  renderTableList();
  renderOverlays();
  setTimeout(() => updateDeliveryUI(''), 3000);
}

function updateDeliveryUI(msg) {
  document.getElementById('robot-state').textContent = msg;
}

// ── Init ───────────────────────────────────────────────────────────────────

async function init() {
  try {
    const cfg = await fetch('/api/config').then(r => r.json());
    deliveryWaitS = cfg.delivery_wait_s;
  } catch (_) {}

  try {
    waypoints = await fetch('/api/waypoints').then(r => r.json());
  } catch (_) {}

  renderTableList();

  const btn = document.getElementById('send-btn');
  btn.addEventListener('click', () => {
    if (delivering) cancelDelivery();
    else startDelivery();
  });

  connect();
}

document.addEventListener('DOMContentLoaded', init);
window.addEventListener('resize', renderOverlays);
