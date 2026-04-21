# Walter — Setup Guide

Covers everything from first boot to a working delivery system. Assumes the Docker image is already built (`docker build -t walter_dev .`).

---

## Prerequisites

**Docker image** — everything is handled by the `Dockerfile` (ROS 2, Nav2, SLAM, Flask, smbus2, spidev…). Just build once:
```bash
docker build -t walter_dev .
```

**Pi host** — `web_server.py` runs outside Docker so it can access GPIO/SPI/I2C directly. Flask needs to be installed on the host separately:
```bash
pip3 install flask
```

---

## Step 1 — Boot Walter

```bash
./run_walter.sh
```

This powers the LiDAR via GPIO 17, waits 3 s for spin-up, then starts the Docker container with the full ROS 2 stack (LiDAR → SLAM → rosbridge → Nav2). One command, nothing else needed.

---

## Step 2 — Start the web server

Run this on the Pi host (not inside Docker):

```bash
cd ~/walter_bot
python3 web_server.py
```

Output:
```
Walter web server starting on http://0.0.0.0:5000
   Delivery  : http://localhost:5000/
   Admin     : http://localhost:5000/admin
   Drive     : http://localhost:5000/static/drive.html
   Face      : http://localhost:5000/static/face.html
```

Keep this running. To auto-start on boot see Step 7.

---

## Step 3 — Map the room

Walter needs a map before he can deliver. Two approaches:

### Option A — Autonomous (roomba mapper)

```bash
docker exec -it walter_dev bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
python3 /ros2_ws/roomba_mapper.py
```

Walter explores by himself. Stops when the map stabilises or you press `Ctrl+C`.

### Option B — Manual drive + scan (recommended if Option A struggles)

1. Open the Admin UI: `http://<pi-ip>:5000/admin`
2. Click **Scan** in the header — scan overlay activates (button turns blue)
3. Optionally click **Fwd only** — restricts LiDAR to the forward 180° arc for cleaner mapping while driving forward
4. Open Drive in another tab: `http://<pi-ip>:5000/static/drive.html`
5. Drive Walter around the room with the keyboard — blue scan dots accumulate on the Admin map, building up the room geometry
6. When walls and obstacles are fully visible on the map, stop driving
7. Click **Clear** in Admin to wipe the dot overlay (the underlying SLAM map is unaffected)
8. Click **Fwd only** again to restore full 360° scan for delivery

Both options build the same SLAM map — the manual route just gives you direct control.

### Save the map

Once the room is covered:
```bash
docker exec -it walter_dev bash /ros2_ws/save_map.sh
```

Creates `maps/room_map.pgm` and `maps/room_map.yaml` in the repo directory.

---

## Step 4 — Admin UI — place table markers

Open: `http://<pi-ip>:5000/admin`

**Adding tables:**
1. Click anywhere on the map
2. Type a name (e.g. `Table 3`)
3. Click **Save** — an orange dot appears and saves to `waypoints.json`

Repeat for all tables (7–12 recommended). Changes persist after reboot.

**To remove a table:** click its dot on the map, or click ✕ in the sidebar list.

### Admin header buttons

| Button | What it does |
|---|---|
| **Full scan / Fwd only** | Toggle LiDAR arc between 360° (full) and 180° (forward only). Takes effect immediately — no restart. Use Fwd only while mapping manually, Full scan for delivery. |
| **Scan** | Show/hide live scan dot overlay on the map. Dots accumulate as Walter moves, showing what the LiDAR has seen. |
| **Clear** | Wipe the accumulated scan dot overlay (SLAM map underneath is untouched). |
| **Drive** | Open the keyboard teleoperation page. |

---

## Step 5 — Keyboard drive

Open: `http://<pi-ip>:5000/static/drive.html`

Also reachable via the **Drive** button in the Admin header.

**Key bindings:**

| Key | Action |
|---|---|
| W / A / S / D or Arrow keys | Forward / Left / Backward / Right |
| Q / Z | Both speeds ×1.1 / ×0.9 |
| = / - | Linear speed only up / down |
| E / C | Angular speed only up / down |
| Space or K | Emergency stop |

Speed starts at 0.05 m/s linear, 0.50 r/s angular. Values are shown live and persist between key presses. The page sends a stop command automatically if the browser window loses focus.

---

## Step 6 — Delivery UI

Open on the touchscreen: `http://localhost:5000/`  
Or from a phone on the same network: `http://<pi-ip>:5000/`

1. Tap a table button in the right panel
2. Tap **Send Walter**
3. Walter navigates to the table, waits 10 seconds, returns to `[0, 0]`
4. Tap **Cancel** at any time to stop immediately

The blue dot on the map shows Walter's live position. The selected table turns green while delivering.

---

## Step 7 — Robot face screen

Open fullscreen on the robot's attached screen:
```
http://localhost:5000/static/face.html
```

The eyes react automatically to `/cmd_vel`:

| State | Colour | Trigger |
|---|---|---|
| Idle | Blue | No movement |
| Moving | Orange | Linear velocity |
| Turning | Purple | Angular velocity |
| Arrived | Green (pulsing) | `/walter/face` topic |
| Error | Red | `/walter/face` topic |

To trigger a state from any Python node or the shell:
```bash
docker exec -it walter_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic pub --once /walter/face std_msgs/String '{data: arrived}'
"
```

Valid states: `idle`, `moving`, `turning`, `arrived`, `waiting`, `returning`, `error`

---

## Step 8 — Auto-start web server on boot (optional)

Create `/etc/systemd/system/walter-web.service`:

```ini
[Unit]
Description=Walter Web Server
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/walter_bot
ExecStart=/usr/bin/python3 /home/pi/walter_bot/web_server.py
Restart=always

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable walter-web
sudo systemctl start walter-web
```

---

## Connecting hardware (SPI / I2C / GPIO)

`web_server.py` runs on the Pi host so it has direct hardware access:

```python
import smbus2   # I2C — battery gauge
import spidev   # SPI — load cell ADC
import RPi.GPIO # GPIO — LEDs, relays
```

**Load cell pattern** (background thread + API endpoint):

```python
import threading, spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1_000_000
load_cell_kg = 0.0

def poll_load_cell():
    global load_cell_kg
    while True:
        raw = spi.xfer2([0x00, 0x00])
        load_cell_kg = (raw[0] << 8 | raw[1]) * 0.001
        time.sleep(0.05)

threading.Thread(target=poll_load_cell, daemon=True).start()

@app.route('/api/weight')
def get_weight():
    return jsonify({'kg': load_cell_kg})
```

The delivery UI can poll `/api/weight` and trigger return-home when the value drops (item collected).

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Map not showing in UI | rosbridge not running | Check `start_brain.sh` Stage 2 started rosbridge on port 9090 |
| Tables disappear after reboot | Wrong working directory | Run `web_server.py` from the `walter_bot/` directory |
| Robot ignores navigation goals | `use_sim_time: True` in nav2_params | Must be `False` everywhere — it silently waits for `/clock` |
| Face screen blank | rosbridge unreachable | Docker must run with `--network host` |
| `flask` not found | Installed in Docker, not host | `pip3 install flask` on the Pi host, not inside Docker |
| SLAM `Failed to compute odom pose` | Wrong `base_frame` | Must be `base_link` in `slam_params.yaml`, not `base_footprint` |
| `Fwd only` button shows `?` | Parameter service call failed | Ensure `lidar_filter.py` is running inside Docker |
| Scan dots misaligned from walls | Odom drift | Expected — `/odom` accumulates error over time. Clear and rescan after repositioning. |
