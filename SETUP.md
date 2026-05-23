# Walter — Setup & Operations Guide

Step-by-step procedures for every task. Each section is self-contained; you don't need to read them in order.

> For an overview of what Walter is and how it's architected, see **[README.md](README.md)**.

---

## Table of contents

1. [Prerequisites — first-time install](#1--prerequisites--first-time-install)
2. [Booting Walter](#2--booting-walter)
3. [Building a map (mapping mode)](#3--building-a-map-mapping-mode)
4. [Running deliveries (navigate mode)](#4--running-deliveries-navigate-mode)
5. [Placing tables and zones (editor)](#5--placing-tables-and-zones-editor)
6. [Manual driving](#6--manual-driving)
7. [Foxglove Studio](#7--foxglove-studio)
8. [Web UI tour](#8--web-ui-tour)
9. [Diagnostics — when something is broken](#9--diagnostics--when-something-is-broken)
10. [Drift testing](#10--drift-testing)
11. [Battery monitoring & IBAT logger](#11--battery-monitoring--ibat-logger)
12. [Load cell calibration](#12--load-cell-calibration)
13. [QR code readability test](#13--qr-code-readability-test)
14. [Auto-start on boot](#14--auto-start-on-boot)
15. [Map management](#15--map-management)
16. [Flask API reference](#16--flask-api-reference)
17. [Troubleshooting](#17--troubleshooting)
18. [Hardware wiring](#18--hardware-wiring)

---

## 1 — Prerequisites — first-time install

### 1.1 Raspberry Pi configuration

```bash
sudo raspi-config
```

Enable:
- **Interface Options → SPI** (for load cells)
- **Interface Options → I²C** (motors, IMU, battery)
- **Interface Options → Serial Port** — disable the **serial login shell**, **enable** the serial port hardware

After saving, reboot.

### 1.2 Docker

The whole ROS stack runs in one container. Build it once:

```bash
docker build -t walter_dev .
```

This installs ROS 2 Humble, Nav2, SLAM Toolbox, rosbridge, **foxglove_bridge**, the sllidar driver, and all Python deps. The build takes ~30 min on a Pi 4.

If you've already built the image but want to pick up the new `foxglove_bridge` package without a full rebuild:

```bash
docker exec walter_dev apt-get update
docker exec walter_dev apt-get install -y ros-humble-foxglove-bridge
```

### 1.3 Pi host dependencies

`web_server.py`, `ibat_logger.py`, `load_cell_test.py` etc. run **outside** Docker so they can hit GPIO / SPI / I²C directly. Install on the Pi host:

```bash
pip3 install flask smbus2 spidev RPi.GPIO
```

### 1.4 Verify hardware addresses

```bash
sudo i2cdetect -y 1
```

You should see:
- `0x55` — motor controller (ESP32)
- `0x6A` — LSM6DS IMU
- `0x6B` — BQ25820 battery monitor

If anything is missing, check wiring before continuing.

---

## 2 — Booting Walter

The one-command boot:

```bash
./run_walter.sh
```

What happens in order:

1. **GPIO power-up** — `sudo pinctrl set 17 op dh` enables the LiDAR power rail.
2. **Web server** — `web_server.py` is started on port 5000 in the background.
3. **3 s wait** — for the LiDAR to spin up and stabilise.
4. **Stale-container cleanup** — `docker rm -f walter_dev` removes any leftover container from a previous crash.
5. **Docker start** — launches `walter_dev` with `--network host`, mounts the repo at `/ros2_ws`, runs `start_brain.sh`.

### Mode auto-detection

| Condition | Mode launched |
|---|---|
| `maps/room_map.yaml` exists | **Navigate** (AMCL + Nav2) |
| Map file missing | **Mapping** (SLAM Toolbox only) |

### Override the mode

```bash
./run_walter.sh mapping              # force mapping (build a new map)
./run_walter.sh navigate             # force navigate with maps/room_map
./run_walter.sh navigate my_custom   # navigate with maps/my_custom.yaml
```

### Stopping cleanly

`Ctrl+C` in the launching terminal triggers `start_brain.sh`'s cleanup handler. In mapping mode it **saves the map before exit**.

---

## 3 — Building a map (mapping mode)

First-time setup — no saved map exists yet, so `./run_walter.sh` starts in mapping mode automatically.

### 3.1 Start mapping

```bash
./run_walter.sh
# (or  ./run_walter.sh mapping  to force it)
```

Wait until the terminal prints:

```
✅ Mapping. Drive around, then Save Map in the UI (or Ctrl+C).
```

### 3.2 Open the map UI

In a browser on the same network:

```
http://<pi-ip>:5000/static/map.html
```

The canvas starts blank — the LiDAR is already scanning but SLAM Toolbox hasn't accumulated enough scans yet.

### 3.3 Drive the robot

Either:
- Use the **Drive tab** inside `map.html` (d-pad + speed slider), or
- Open a separate `http://<pi-ip>:5000/static/drive.html` tab and use the keyboard.

Drive **slowly** (0.05–0.10 m/s). SLAM Toolbox adds a scan to the graph only when the robot moves at least `minimum_travel_distance = 0.3 m` OR rotates `minimum_travel_heading = 0.3 rad`. Nudging back and forth produces no map growth.

### 3.4 Watch the map build

What you should see in `map.html`:
- **Grey** — unknown / unobserved cells
- **White** — free space (LiDAR ray passed through)
- **Black** — occupied (LiDAR ray hit a wall)
- **Coloured dots** — current `/scan_filtered` points
- **Orange dot** — closest point in the forward ±15° arc

The `/map` topic publishes only every **`map_update_interval = 8 s`** (from `slam_params.yaml`). Don't be alarmed if the map doesn't grow every second.

### 3.5 Cover the whole room

Drive close to every wall and around obstacles. SLAM's loop-closure runs when you revisit an area — try to make at least one closed loop around the room before finishing.

### 3.6 Save the map

| Method | How |
|---|---|
| **In the UI** | Click **Save Map** in the Drive tab of `map.html` |
| **Auto-save** | Every 2 minutes — see `start_brain.sh` |
| **On Ctrl+C** | Saved automatically before SLAM shuts down |
| **From a shell** | `docker exec -it walter_dev bash /ros2_ws/save_map.sh` |

Saved files land in:

```
maps/room_map.pgm    ← bitmap (white=free, black=occupied, grey=unknown)
maps/room_map.yaml   ← metadata (resolution, origin, thresholds)
```

### 3.7 Switch to navigate

Stop Walter (`Ctrl+C`), then:

```bash
./run_walter.sh
```

`maps/room_map.yaml` exists now → auto-detect picks navigate mode.

### 3.8 If the map isn't accumulating

See [§9 — Diagnostics](#9--diagnostics--when-something-is-broken). Run:

```bash
docker exec -it walter_dev bash /ros2_ws/diagnose_map.sh
```

The script walks the entire pipeline (`/scan → lidar_filter → /scan_filtered → SLAM → /map`) with PASS / FAIL output.

---

## 4 — Running deliveries (navigate mode)

### 4.1 Start in navigate mode

A saved map must exist (see §3). Then:

```bash
./run_walter.sh
# (auto-detects navigate mode, or force with  ./run_walter.sh navigate)
```

Wait until the terminal prints:

```
✅ Walter is live. AMCL localising on saved map.
```

### 4.2 Verify AMCL has converged

Open `http://<pi-ip>:5000/static/editor.html`. The map should appear. AMCL needs ~5 s and a small motion to converge — if Walter's marker doesn't appear, drive it 30 cm forward and back.

### 4.3 Send a delivery

```
http://<pi-ip>:5000/static/delivery.html
```

1. Tap a table from the list (populated from `waypoints.json`).
2. Tap **Send Walter**. The UI publishes a `PoseStamped` to `/goal_pose`.
3. Walter plans, drives, waits `DELIVERY_WAIT_S` seconds (10 by default — set in `web_server.py`), then returns to the **origin** waypoint.
4. **Cancel** stops the navigation goal at any time.

### 4.4 Live position

`/amcl_pose` is shown on the map. If it jumps around, AMCL isn't well-localised — either widen the initial particle cloud or drive the robot to a known landmark.

---

## 5 — Placing tables and zones (editor)

Works in **both** mapping and navigate modes — `editor.html` subscribes to `/map` (published by SLAM or `map_server` depending on mode).

### 5.1 Open the editor

```
http://<pi-ip>:5000/static/editor.html
```

### 5.2 Place a waypoint

1. Pick a type from the toolbar:
   - **Table** — numbered, with width × depth
   - **Zone** — named area (kitchen, bar, dance floor)
   - **Origin** — robot home / return point (only one allowed)
2. Click on the map at the location.
3. Fill in the form that appears (label, dimensions if a table).
4. Hit **Save**.

### 5.3 Edit / move / delete

- **Drag** a marker to reposition.
- **Click** a marker to open its inline edit form.
- The form has a **Delete** button.

All changes write to `waypoints.json` at the repo root.

### 5.4 Schema (`waypoints.json`)

```json
{
  "Table 1": {
    "x": 1.2, "y": -0.5, "theta": 0.0,
    "label": "Table 1",
    "type": "table",
    "number": 1,
    "width": 0.8,
    "depth": 0.6
  },
  "Origin": {
    "x": 0.0, "y": 0.0, "theta": 0.0,
    "label": "Origin",
    "type": "origin"
  }
}
```

---

## 6 — Manual driving

Open `http://<pi-ip>:5000/static/drive.html`

### 6.1 Keyboard

| Key | Action |
|---|---|
| `W` / `↑` | Forward |
| `S` / `↓` | Backward |
| `A` / `←` | Turn left |
| `D` / `→` | Turn right |
| `Q` / `Z` | Both speeds ×1.1 / ×0.9 |
| `=` / `-` | Linear speed up / down |
| `E` / `C` | Angular speed up / down |
| `Space` / `K` | Emergency stop |

Starts at 0.05 m/s linear, 0.50 rad/s angular. A stop command is sent automatically if the window loses focus.

### 6.2 D-pad (touch / mouse)

Identical actions to the keyboard. Uses `pointerdown`/`pointerup` events so it works on touch screens, mouse and stylus.

### 6.3 Without the UI — pure CLI

Inside the Docker container:

```bash
docker exec -it walter_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"
```

---

## 7 — Foxglove Studio

`foxglove_bridge` is on port `:8765` with a topic whitelist of **only** `/scan`, `/scan_filtered`, `/map`, `/tf`, `/tf_static`, `/robot_description`.

### 7.1 Connect

1. Install Foxglove Studio (https://foxglove.dev/download).
2. **Open connection** → **Foxglove WebSocket** → `ws://<pi-ip>:8765`.
3. You should see exactly the 6 topics above in the data source panel.

### 7.2 Build a useful layout

| Panel | Topic | Purpose |
|---|---|---|
| **3D** | `/tf`, `/tf_static`, `/robot_description` | Shows the robot model with the LiDAR frame |
| **Map** | `/map` | Live occupancy grid (in mapping) or saved map (in navigate) |
| **Plot — LaserScan range** | `/scan` | Raw distances per beam |
| **Plot — LaserScan range** | `/scan_filtered` | After lidar_filter — see the blind-spot masking work |

For the 3D panel, after adding it: **+ → Topic → /robot_description** (URDF) and add `/tf` so the model follows the robot's pose.

### 7.3 If a topic isn't appearing

The whitelist is configured in `start_brain.sh`. To temporarily expose more topics:

```bash
docker exec walter_dev ros2 param set /foxglove_bridge topic_whitelist \
  '["/scan","/scan_filtered","/map","/tf","/tf_static","/robot_description","/odom","/imu/data_raw"]'
```

---

## 8 — Web UI tour

| Page | URL | When to use |
|---|---|---|
| Launcher | `/` | Start here — also has the battery widget |
| Map | `/static/map.html` | Mapping mode — drive + watch the map build |
| Editor | `/static/editor.html` | Either mode — place tables / zones / origin |
| Delivery | `/static/delivery.html` | Navigate mode — send Walter to a table |
| Drive | `/static/drive.html` | Either mode — keyboard / d-pad teleop |
| Face | `/static/face.html` | The robot's screen — open fullscreen on the on-robot display |
| Logs | `/static/logs.html` | Live LiDAR / odom / IMU / battery gauges |

### 8.1 Face screen

Open fullscreen on the robot's attached screen:

```
http://localhost:5000/static/face.html
```

Eyes react automatically to `/cmd_vel`. Trigger named states from any node:

```bash
docker exec -it walter_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic pub --once /walter/face std_msgs/String '{data: arrived}'
"
```

Valid states: `idle`, `moving`, `turning`, `arrived`, `waiting`, `returning`, `error`.

### 8.2 Logs page

Real-time sensor gauges fed from rosbridge:

- `/scan` — point count, min / max / mean range
- `/odom` — linear and angular velocity
- `/imu/data_raw` — acceleration X / Y / Z
- `/cmd_vel` — commanded velocity
- **BQ25820 battery** — voltage, current, charge state (polled via Flask `/api/battery`)

Use this before a delivery to confirm everything is publishing.

---

## 9 — Diagnostics — when something is broken

### 9.1 The map pipeline diagnostic

Run inside Docker:

```bash
docker exec -it walter_dev bash /ros2_ws/diagnose_map.sh
```

It walks the chain step-by-step:

```
── 1. ROS nodes running ──────────────
   ✅ /sllidar_node is running
   ✅ /lidar_filter is running
   ✅ /walter_hardware_bridge is running
   ✅ /slam_toolbox is running

── 2. /scan ──────────────────────────
   ✅ /scan publishing at 25.4 Hz

── 3. /scan_filtered ────────────────
   ✅ /scan_filtered publishing at 25.4 Hz

── 4. TF tree ────────────────────────
   ✅ map → base_link  available
   ✅ map → laser_frame  available

── 5. /map topic ─────────────────────
   ✅ /map topic exists
   ✅ Latest /map :  width: 481  height: 392  resolution: 0.05

── 6. Bridges ────────────────────────
   ✅ rosbridge_websocket alive on :9090
   ✅ foxglove_bridge alive on :8765
```

If anything is ❌, the script tells you what's broken AND prints the last 6 lines of SLAM's log.

### 9.2 Tail the live logs

All bridges and SLAM/Nav2 now log to `/tmp/*.log` (instead of `/dev/null`):

```bash
docker exec -it walter_dev tail -f /tmp/slam.log
docker exec -it walter_dev tail -f /tmp/nav2.log
docker exec -it walter_dev tail -f /tmp/rosbridge.log
docker exec -it walter_dev tail -f /tmp/foxglove_bridge.log
```

Common SLAM warnings and what they mean:

| Log line | Meaning |
|---|---|
| `[WARN] No transform from [base_link] to [map]` | TF tree broken — `bridge_node.py` probably crashed |
| `[WARN] Failed to compute odom pose` | `/odom` isn't publishing — ESP32 not responding on I²C 0x55 |
| Repeated `Got scan` with no `Adding scan to graph` | Robot moved less than 0.3 m between scans — drive further |

### 9.3 Inspect individual topics

```bash
docker exec -it walter_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic list                         # all topics
  ros2 topic hz /scan_filtered            # publish rate
  ros2 topic echo /map --once \\
    --qos-durability transient_local      # one /map message
  ros2 run tf2_ros tf2_echo map base_link # TF map → base_link
"
```

---

## 10 — Drift testing

Two tools live at the repo root. Both run inside Docker.

### 10.1 `drift_test.py` — automated

Drives 1 m forward, rotates 180° (IMU-integrated, bias-corrected), drives 1 m back, prints a report and a CSV.

```bash
docker exec -it walter_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /ros2_ws &&
  python3 drift_test.py
"
```

What it produces:

```
══════════════════════════════════════════
  DRIFT TEST REPORT
══════════════════════════════════════════

  IMU CALIBRATION
    Gyro Z bias        : +0.0023 rad/s
    Fwd accel bias     : -0.041  m/s²

  PHASE 1  Forward 1.0 m
    Commanded   : 1.000 m
    Actual IMU  : 0.987 m
    Actual odom : 0.992 m

  PHASE 2  Rotate 180°
    Commanded  : 180.00°
    Actual IMU : 179.43°
    Error      : -0.57°

  FINAL POSITION (odom vs start)
    Position drift : 0.026 m
    Heading drift  : +1.2°
```

Plus a CSV at `/tmp/drift_<timestamp>.csv` with per-tick state, IMU integration, commanded velocity — useful for plotting in a notebook later.

**Copy the CSV to your laptop:**

```bash
scp <user>@<pi-ip>:/tmp/drift_20260523_142351.csv ~/Downloads/
```

### 10.2 `forward_return.py` — interactive

Drive forward at a chosen speed, then either press ENTER or let the robot stop automatically at a target distance.  It then turns 180° and returns the exact same distance.  A full drift report is printed at the end.

**Usage:**

```bash
# Manual mode — press ENTER whenever you want it to turn back
docker exec -it walter_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /ros2_ws &&
  python3 forward_return.py m
"

# Auto-stop mode — provide a target distance in metres (any positive float)
docker exec -it walter_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  cd /ros2_ws &&
  python3 forward_return.py l 0.2    # stops at 20 cm
  python3 forward_return.py m 0.5    # stops at 50 cm
  python3 forward_return.py h 1.0    # stops at 1 m
"
```

If you omit both arguments you get an interactive prompt for level and distance.

**Speed levels:**

| Level | Linear | Angular |
|---|---|---|
| `l` low | 0.02 m/s | 0.01 rad/s |
| `m` medium | 0.04 m/s | 0.03 rad/s |
| `h` high | 0.06 m/s | 0.06 rad/s |

**Live display while driving forward:**

```
# Auto-stop mode
→ 0.312 / 0.500 m   heading: +1.8° (left)   remain: 0.188 m

# Manual mode
→ 0.312 m   heading: +1.8° (left)   (press ENTER)
```

Sign convention: `+` = veering left, `−` = veering right.  Label reads `straight` when deviation is < 0.5°.

**Controls:**
- **ENTER / SPACE** — trigger return immediately (manual mode only)
- **Ctrl+C** — emergency stop, robot halts in place

**Sample report:**

```
  ──────────────────────────────────────────
  ✓ DONE
     Outbound       : 0.500 m
     Return         : 0.503 m
     Final position : drift 0.021 m  (Δx=+0.018, Δy=-0.011)

  Heading-drift while driving straight
     Calibrated target  :  0.0°   (perfect straight line)
     At ENTER (final)   : +1.8°
     Peak during run    :  2.3°
     Assessment         : ~ acceptable (< 5°)
  ──────────────────────────────────────────
```

Assessment thresholds:

| Peak heading drift | Verdict |
|---|---|
| < 2° | ✓ excellent |
| 2° – 5° | ~ acceptable |
| ≥ 5° | ✗ high drift — tune angular PID or wheel trim |

Hardware note: `YAW_SCALE = 2.0` compensates for the vertically-mounted IMU (odom yaw over-reports by 2×).  This affects both the 180° turn and the heading-drift calculation.

---

## 11 — Battery monitoring & IBAT logger

### 11.1 Live battery in the UI

The launcher (`/`) and `logs.html` both show:

- **Voltage** (VBAT) — 24 V (empty) → 32 V (full) for the 8 S Li-ion pack
- **Current** (IBAT) — positive = charging, negative = discharging
- **System voltage** (VSYS) — the rail powering the Pi and motors
- **Charge state** — `not_charging` / `trickle` / `pre_charge` / `fast_charge` / `taper_charge`
- **Power good** — ⚡ when a charger is connected

Polled from `web_server.py:/api/battery` every 30 s.

### 11.2 IBAT logger — long-running CSV capture

`ibat_logger.py` runs on the Pi host (it reads I²C directly, no Docker needed):

```bash
python3 ibat_logger.py idle_with_lidar
# Logs 100 points at 1 Hz, then stops. Or Ctrl+C earlier.
# Output: idle_with_lidar.csv  (columns: t_s, ibat_ma)
```

Useful for measuring how much current the robot draws in different states (idle, mapping, navigating, with/without LiDAR powered).

**Pull the CSV to your laptop:**

```bash
scp <user>@<pi-ip>:~/walter_bot/idle_with_lidar.csv ~/Downloads/
```

---

## 12 — Load cell calibration

The tray weight sensors connect through a 24-bit SPI ADC. `load_cell_test.py` is the test + calibration tool.

### 12.1 Wiring (BCM pin numbers)

| Signal | GPIO | Direction |
|---|---|---|
| `/RESET` | 27 | Output |
| `DRDY` | 22 | Input |
| `/CS` | 8 | Output (manual) |
| `MOSI/MISO/SCLK` | SPI0 CE0 | SPI hardware |

### 12.2 Test the connection

```bash
python3 load_cell_test.py --debug
```

Expected output:

```
  CH0:  +00012345  (  +0.0018 mV)
  CH1:  -00003210  (  -0.0005 mV)   ← target channel
  ...
```

If `DRDY` never asserts:

```
  ⚠  DRDY did not assert within 2 s after reset.
     Possible causes:
       - ADC not powered
       - Wrong SPI mode (try spi.mode = 0b00)
       - Wrong RESET polarity (active-low?)
```

### 12.3 Calibrate (tare + known weight)

```bash
python3 load_cell_test.py --calibrate
```

Two-step wizard:
1. **Tare** — remove all weight, press Enter. 30 samples → zero offset.
2. **Scale** — place a known weight (e.g. 1 kg), enter the value. Computes the scale factor.

Output:

```
  TARE_OFFSET_CODE = -12450
  SCALE_FACTOR_KG  = 0.00000812
```

Copy both into the config block at the top of `load_cell_test.py` and into `web_server.py`'s weight polling thread.

### 12.4 Live monitoring

```bash
python3 load_cell_test.py                # single channel, rolling average
python3 load_cell_test.py --channel 0    # different channel
python3 load_cell_test.py --all          # all 6 channels
python3 load_cell_test.py --samples 500  # capture 500 samples then exit
```

### 12.5 Expose weight via the API

Add to `web_server.py`:

```python
import threading, spidev, RPi.GPIO as GPIO

TARE_OFFSET_CODE = -12450
SCALE_FACTOR_KG  = 0.00000812

_weight_kg = 0.0

def _poll_weight():
    global _weight_kg
    # (set up SPI + GPIO exactly as in load_cell_test.py)
    while True:
        if wait_drdy():
            frame = read_frame(spi)
            code  = decode_24bit(frame[3], frame[4], frame[5])  # CH1
            _weight_kg = (code - TARE_OFFSET_CODE) * SCALE_FACTOR_KG
        time.sleep(0.1)

threading.Thread(target=_poll_weight, daemon=True).start()

@app.route('/api/weight')
def get_weight():
    return jsonify({'kg': round(_weight_kg, 4)})
```

Now the delivery UI can poll `/api/weight` and auto-trigger return-home when the value drops (item collected).

---

## 13 — QR code readability test

`qr_test.py` verifies that QR stickers on tables are scannable BEFORE Walter is deployed.

### 13.1 Install dependencies (Pi host)

```bash
sudo apt-get install libzbar0
pip3 install pyzbar opencv-python Pillow
```

### 13.2 Basic scan

```bash
python3 qr_test.py                       # scan current directory
python3 qr_test.py --dir /path/to/qrs    # specific folder
```

Output:

```
  [  1/6]  table_qr_1.jpg          ✅ IMMEDIATE         data='Table:1'
  [  2/6]  table_qr_2.jpg          🔧 PROCESSING
                                    transform : rotate 90°
                                    decoder   : pyzbar
                                    data      : 'Table:2'
  [  5/6]  torn_label.jpg          ❌ UNREADABLE
```

### 13.3 Other flags

```bash
python3 qr_test.py --debug          # save annotated images to ./qr_debug/
python3 qr_test.py --json out.json  # machine-readable report
python3 qr_test.py --quiet          # summary only
```

The script tries every transform combination (rotate, threshold, CLAHE, sharpen, upscale, crop) and reports which one worked — so you know exactly which lighting / print conditions break a sticker.

---

## 14 — Auto-start on boot

### 14.1 systemd service

`/etc/systemd/system/walter.service`:

```ini
[Unit]
Description=Walter Robot
After=network.target docker.service
Requires=docker.service

[Service]
User=pi
WorkingDirectory=/home/pi/walter_bot
ExecStart=/home/pi/walter_bot/run_walter.sh
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl daemon-reload
sudo systemctl enable walter
sudo systemctl start walter
```

Tail the service log:

```bash
sudo journalctl -u walter -f
```

> `web_server.py` is launched **inside** `run_walter.sh` — you don't need a separate unit for it.

---

## 15 — Map management

### 15.1 Save manually

```bash
# From the UI
#   Click Save Map in map.html (mapping mode only)

# From the host shell
docker exec -it walter_dev bash /ros2_ws/save_map.sh
docker exec -it walter_dev bash /ros2_ws/save_map.sh my_custom_name
```

### 15.2 Use a different saved map

```bash
./run_walter.sh navigate my_custom_name   # loads maps/my_custom_name.yaml
```

### 15.3 Start fresh

```bash
./run_walter.sh mapping
# or remove the file and let auto-detect pick mapping mode:
rm maps/room_map.yaml maps/room_map.pgm
./run_walter.sh
```

### 15.4 Inspect a saved map

The `.yaml` is human-readable:

```yaml
image: room_map.pgm
resolution: 0.050000
origin: [-15.0, -10.0, 0.0]
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

The `.pgm` is a P5 grayscale bitmap — open in any image viewer or convert with ImageMagick.

---

## 16 — Flask API reference

All on port 5000 (`web_server.py` running on the Pi host):

| Method | Path | Description |
|---|---|---|
| GET | `/api/waypoints` | All waypoints as JSON |
| PUT | `/api/waypoints/<name>` | Create or update a waypoint. Body: `{x, y, theta, label, type, ...}` |
| DELETE | `/api/waypoints/<name>` | Delete a waypoint |
| POST | `/api/save_map` | Trigger `save_map.sh` inside Docker. Body: `{"name": "room_map"}` |
| GET | `/api/config` | Server config (currently just `delivery_wait_s`) |
| GET | `/api/battery` | Live BQ25820 reading: `{voltage_v, current_ma, vsys_v, percent, status, charge_stat, power_good, source}` |

---

## 17 — Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Map not showing in browser | rosbridge not connected | Check the green dot in the header. Verify port 9090 is reachable from the browser |
| Map shows but never grows | You're in navigate mode, not mapping | `./run_walter.sh mapping` — only SLAM grows the map |
| Map grows in Foxglove but not the web UI | rosbridge dropping `/map` because of QoS | The web UI subscribes manually with TRANSIENT_LOCAL — check the browser console for errors |
| `diagnose_map.sh` says `/scan_filtered` missing | `lidar_filter.py` crashed | `docker exec walter_dev ps aux \| grep lidar_filter` — restart if missing |
| Walter ignores nav goals | Not in navigate mode | Verify `maps/room_map.yaml` exists; check `/tmp/nav2.log` |
| `flask` not found | Flask only inside Docker | `pip3 install flask` on the Pi **host** |
| Port 5000 already in use | Stale `web_server.py` | `pkill -f web_server.py` then re-run `run_walter.sh` |
| Face screen blank | rosbridge unreachable from browser | Docker must use `--network host` (it does by default in `run_walter.sh`) |
| SLAM `Failed to compute odom pose` | `bridge_node.py` not publishing `/odom` | Check `/tmp/slam.log`; verify motor controller is responding on I²C 0x55 |
| Walter stuck or oscillating in narrow corridors | DWB inflation too small | Increase `inflation_radius` in `config/nav2_params.yaml` |
| `use_sim_time` error in Nav2 | A node has `use_sim_time: True` | Set `use_sim_time: False` everywhere in `config/nav2_params.yaml` |
| Scan dots misaligned from walls | Odometry drift on long runs | Expected — re-save the map after repositioning the robot at the origin |
| Docker container already exists | Stale container from a crash | `docker rm -f walter_dev` (`run_walter.sh` does this automatically) |
| 180° turn results in 90° physical | Vertical IMU mount | `forward_return.py` compensates with `YAW_SCALE = 2.0`; the report shows both odom degrees and physical degrees so you can verify |
| Heading drift ≥ 5° on a straight run | Wheel imbalance or angular PID gain | Run `forward_return.py l 0.5` a few times and compare peak drift; adjust motor trim or PID if consistently high |
| Foxglove sees no topics | foxglove_bridge not running or whitelist wrong | `docker exec walter_dev tail /tmp/foxglove_bridge.log`; verify it started |

---

## 18 — Hardware wiring

| Component | Interface | Address / Port | Notes |
|---|---|---|---|
| RPLidar A1M8 | UART | `/dev/ttyAMA0` @ 115200 baud | Disable serial login in `raspi-config`, keep hardware UART enabled |
| ESP32 motor controller | I²C | `0x55` | Same bus as IMU and battery |
| LSM6DS IMU | I²C | `0x6A` | **Mounted vertically** — affects yaw integration (see `forward_return.py`) |
| BQ25820 battery charger | I²C | `0x6B` | Polled by `web_server.py` via `smbus2` |
| LiDAR power switch | GPIO | Pin 17 | `pinctrl set 17 op dh` in `run_walter.sh` |
| Load cell ADC | SPI | CE0 (`/dev/spidev0.0`) | Manual CS on GPIO 8, DRDY on GPIO 22, RESET on GPIO 27 |
| Battery pack | — | 8 S Li-ion | 24 V (empty) → 32 V (full); `BATTERY_MIN_V` / `BATTERY_MAX_V` in `web_server.py` |
