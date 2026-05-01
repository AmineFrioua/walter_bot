# Walter — Setup Guide

Covers everything from first boot to a working delivery system.

---

## Prerequisites

### Docker image

Everything ROS-related runs in Docker. Build the image once:

```bash
docker build -t walter_dev .
```

This installs ROS 2 Humble, Nav2, SLAM Toolbox, rosbridge, and all Python dependencies.

### Pi host dependencies

`web_server.py` runs outside Docker (so it can access GPIO/SPI/I2C). Install Flask on the Pi host:

```bash
pip3 install flask
```

---

## Step 1 — Boot Walter

```bash
./run_walter.sh
```

What this does, in order:
1. Powers the LiDAR via GPIO 17 (`sudo pinctrl set 17 op dh`)
2. Starts `web_server.py` on port 5000 in the background
3. Waits 3 seconds for the LiDAR to spin up
4. Removes any stale Docker container named `walter_dev`
5. Starts Docker with the ROS 2 stack in the detected mode

**Mode auto-detection:**

| Condition | Mode started |
|---|---|
| `maps/room_map.yaml` exists | Navigate (AMCL + Nav2) |
| No map file found | Mapping (SLAM only) |

Override explicitly:

```bash
./run_walter.sh mapping             # force map-building mode
./run_walter.sh navigate            # force production mode
./run_walter.sh navigate my_map     # navigate with maps/my_map.yaml
```

---

## Step 2 — Open the UI

From a browser on the same network:

```
http://<pi-ip>:5000/
```

The launcher page has cards for all tools. All pages are mobile-friendly — use a phone or tablet if needed.

---

## Step 3 — Build a map (first time only)

On first boot there is no map, so Walter starts in **mapping mode** (SLAM Toolbox, no Nav2).

1. Open `http://<pi-ip>:5000/static/map.html`
2. The map canvas starts blank; the LiDAR is already scanning
3. Open **Drive** tab (or `http://<pi-ip>:5000/static/drive.html` on another tab) and drive Walter around the room
4. Watch the map build live:
   - Blue edges = detected obstacle boundaries
   - Coloured dots = current LiDAR scan
   - Orange dot = closest point in the forward ±15° arc
5. Cover all walls and obstacle perimeters
6. Click **Save Map** in the Drive tab when done

The map is also **auto-saved every 2 minutes** and on Ctrl+C.

Saved files:
```
maps/room_map.pgm    ← bitmap
maps/room_map.yaml   ← metadata (resolution, origin)
```

7. Stop Walter (`Ctrl+C` in the terminal) and restart:

```bash
./run_walter.sh
```

This time `maps/room_map.yaml` exists → Walter auto-starts in **navigate mode**.

---

## Step 4 — Place tables and zones

With Walter running in navigate mode:

1. Open `http://<pi-ip>:5000/static/editor.html`
2. The saved map loads from the `/map` topic (published by `map_server`)
3. **Place a waypoint:**
   - Select type from the toolbar: **Table**, **Zone**, or **Origin**
   - Click anywhere on the map to place it
   - Fill in the label and (for tables) width × depth
   - Click **Save**
4. **Move a waypoint:** drag the marker to a new position
5. **Edit / delete:** click the marker to open the inline form

Waypoints are stored in `waypoints.json` in the repo root and persist across reboots.

**Waypoint schema:**
```json
{
  "Table 1": {
    "x": 1.2, "y": -0.5, "theta": 0.0,
    "label": "Table 1",
    "type": "table",
    "number": 1,
    "width": 0.8,
    "depth": 0.6
  }
}
```

---

## Step 5 — Delivery

1. Open `http://<pi-ip>:5000/static/delivery.html`
2. Tap a table name in the list
3. Tap **Send Walter** — a Nav2 goal is published to `/goal_pose`
4. Walter navigates autonomously to the table and waits (10 s by default), then returns to origin
5. Tap **Cancel** at any time to abort

Walter's live position is shown on the map using `/amcl_pose`.

---

## Step 6 — Manual drive

Open `http://<pi-ip>:5000/static/drive.html`

**Keyboard bindings:**

| Key | Action |
|---|---|
| W / ↑ | Forward |
| S / ↓ | Backward |
| A / ← | Turn left |
| D / → | Turn right |
| Q / Z | Both speeds ×1.1 / ×0.9 |
| = / - | Linear speed up / down |
| E / C | Angular speed up / down |
| Space / K | Emergency stop |

Speed starts at 0.05 m/s linear, 0.50 rad/s angular. Values are shown live and persist between key presses. A stop command is sent automatically when the window loses focus.

The page also has an on-screen **d-pad** for touchscreen / mobile use.

---

## Step 7 — Sensor logs

Open `http://<pi-ip>:5000/static/logs.html`

Live gauges show:
- `/scan` — point count, min / max / mean range
- `/odom` — linear and angular velocity
- `/imu/data_raw` — acceleration X / Y / Z
- `/cmd_vel` — commanded velocity

Useful for verifying hardware is publishing correctly before a delivery run.

---

## Step 8 — Robot face screen

Open fullscreen on the robot's attached screen:

```
http://localhost:5000/static/face.html
```

The eyes react automatically to `/cmd_vel`. Trigger named states from any node or shell:

```bash
docker exec -it walter_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic pub --once /walter/face std_msgs/String '{data: arrived}'
"
```

Valid states: `idle`, `moving`, `turning`, `arrived`, `waiting`, `returning`, `error`

---

## Step 9 — Load cell calibration

Walter's motherboard routes load cell signals through a 24-bit SPI ADC before reaching the Pi. Use `load_cell_test.py` to verify the connection and calibrate.

### Wiring (BCM pins)

| Signal | GPIO | Direction |
|---|---|---|
| `/RESET` | 27 | Output |
| `DRDY` | 22 | Input |
| `/CS` | 8 | Output (manual) |
| `MOSI/MISO/SCLK` | SPI0 CE0 | SPI hardware |

Enable SPI on the Pi if not already done:
```bash
sudo raspi-config   # Interface Options → SPI → Enable
```

Install Python dependencies (Pi host):
```bash
pip3 install spidev RPi.GPIO
```

### Test the connection

```bash
# Quick sanity check — prints one decoded frame from all 6 channels after reset
python3 load_cell_test.py --debug
```

Expected output when wired correctly:
```
  CH0:  +00012345  (  +0.0018 mV)
  CH1:  -00003210  (  -0.0005 mV)   ← target
  ...
```

If DRDY never asserts you'll see:
```
  ⚠  DRDY did not assert within 2 s after reset.
     Possible causes:
       - ADC not powered
       - Wrong SPI mode (try spi.mode = 0b00)
       ...
```

### Calibrate (tare + known weight)

```bash
python3 load_cell_test.py --calibrate
```

The wizard walks through two steps:
1. **Tare** — remove all weight, press Enter, script reads 30 samples and computes the zero offset
2. **Scale** — place a known weight (e.g. 1 kg), enter the value, script computes the scale factor

Output:
```
  TARE_OFFSET_CODE = -12450
  SCALE_FACTOR_KG  = 0.00000812
```

Copy these two values into the config block at the top of `load_cell_test.py` and into the `web_server.py` weight polling thread.

### Live monitoring

```bash
python3 load_cell_test.py                 # single channel, rolling average
python3 load_cell_test.py --channel 0     # read a different channel
python3 load_cell_test.py --all           # show all 6 channels per line
python3 load_cell_test.py --samples 500   # capture 500 samples then exit
```

### Integrate with web server

Once calibrated, add a background polling thread to `web_server.py`:

```python
import threading, spidev, RPi.GPIO as GPIO

TARE_OFFSET_CODE = -12450      # from calibration
SCALE_FACTOR_KG  = 0.00000812

_weight_kg = 0.0

def _poll_weight():
    global _weight_kg
    # (set up GPIO / SPI exactly as in load_cell_test.py)
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

The delivery UI can then poll `/api/weight` and trigger return-home automatically when the value drops below threshold (item collected).

---

## Step 10 — Auto-start on boot (optional)

### `run_walter.sh` on boot

Create `/etc/systemd/system/walter.service`:

```ini
[Unit]
Description=Walter Robot
After=network.target

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
sudo systemctl enable walter
sudo systemctl start walter
```

> **Note:** `web_server.py` is already started by `run_walter.sh` — you do not need a separate service for it.

---

## Map management

### Save a map manually

```bash
# From the UI — click Save Map in map.html (mapping mode only)

# Or from the terminal:
docker exec -it walter_dev bash /ros2_ws/save_map.sh
docker exec -it walter_dev bash /ros2_ws/save_map.sh my_custom_name
```

### Use a different saved map

```bash
./run_walter.sh navigate my_custom_name
# Loads maps/my_custom_name.yaml
```

### Start over with a fresh map

```bash
./run_walter.sh mapping
# Or delete the map file so auto-detect picks mapping mode:
rm maps/room_map.yaml maps/room_map.pgm
./run_walter.sh
```

---

## Flask API reference

`web_server.py` exposes these endpoints (all on port 5000):

| Method | Path | Description |
|---|---|---|
| GET | `/api/waypoints` | Return all waypoints as JSON |
| PUT | `/api/waypoints/<name>` | Create or update a waypoint |
| DELETE | `/api/waypoints/<name>` | Delete a waypoint |
| POST | `/api/save_map` | Trigger `save_map.sh` inside Docker. Body: `{"name": "room_map"}` |
| GET | `/api/config` | Return server config (`delivery_wait_s`) |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Map not showing in browser | rosbridge not connected | Check the ROS dot in the header — should be green. Verify port 9090 is open. |
| Map shows but is empty | Wrong mode | `editor.html` works in both modes; `map.html` needs mapping mode and SLAM running |
| Walter ignores nav goals | Nav2 not running | Only available in navigate mode. Check `maps/room_map.yaml` exists. |
| `flask` not found | Flask not on host | `pip3 install flask` on Pi host (not inside Docker) |
| Port 5000 already in use | Stale web server process | `pkill -f web_server.py` then restart |
| Face screen blank | rosbridge unreachable | Docker must run with `--network host` |
| SLAM `Failed to compute odom pose` | Wrong `base_frame` | Must be `base_link` in `slam_params.yaml` |
| Walter stuck or oscillating | DWB inflation radius | Increase `inflation_radius` in `config/nav2_params.yaml` |
| `use_sim_time` error | Nav2 waiting for `/clock` | Set `use_sim_time: False` in all nav2_params.yaml nodes |
| Scan dots misaligned from walls | Odometry drift | Expected on long runs — re-save the map after repositioning |
| Docker container already exists | Stale container from crash | `docker rm -f walter_dev` — `run_walter.sh` does this automatically |

---

## Hardware wiring reference

| Component | Interface | Address / Port | Notes |
|---|---|---|---|
| RPLidar A1M8 | UART | `/dev/ttyAMA0` @ 115200 | Disable serial login in `raspi-config`, keep hardware UART on |
| Motor controller | I2C | `0x55` | Requires I2C enabled in `raspi-config` |
| LSM6DS IMU | I2C | `0x6A` | Same I2C bus as motor controller |
| LiDAR power | GPIO | Pin 17 | Controlled by `run_walter.sh` via `pinctrl` |
