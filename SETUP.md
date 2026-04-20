# Walter Web App — Setup Tutorial

This guide covers everything from a fresh Pi to a working delivery UI. It assumes the Docker image is already built (`docker build -t walter_dev .`).

---

## Prerequisites

On the Raspberry Pi host:
```bash
pip3 install flask
```

In the Docker image (already in `Dockerfile`):
- `ros-humble-rosbridge-server`
- `ros-humble-nav2-*`
- `ros-humble-slam-toolbox`

---

## Step 1 — Map the room

Before you can deliver to tables, Walter needs a map.

**1a. Boot Walter:**
```bash
./run_walter.sh
```

**1b. Start the roomba mapper** (new terminal, inside Docker):
```bash
docker exec -it walter_dev bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash
python3 /ros2_ws/roomba_mapper.py
```

Walter will explore the room autonomously. You'll see scan and pose logs in the terminal. Let it run until you see:
```
📐 Map stopped growing — exploration complete!
```
or press `Ctrl+C` to stop manually.

**1c. Save the map:**
```bash
docker exec -it walter_dev bash /ros2_ws/save_map.sh
```
This creates `maps/room_map.pgm` and `maps/room_map.yaml` in the repo directory.

---

## Step 2 — Start the web server

Run this **on the Pi host** (not inside Docker):

```bash
cd ~/walter_bot
python3 web_server.py
```

You should see:
```
Walter web server starting on http://0.0.0.0:5000
   Delivery UI : http://localhost:5000/
   Admin UI    : http://localhost:5000/admin
```

The web server runs permanently alongside Docker. If you reboot, start it with:
```bash
python3 ~/walter_bot/web_server.py &
```

---

## Step 3 — Place table markers (Admin UI)

Open on the robot touchscreen or any browser on the same network:
```
http://<pi-ip>:5000/admin
```

You'll see the live map from SLAM (requires Walter + ROS to be running).

**To add a table:**
1. Click anywhere on the map
2. Type a name (e.g. `Table 3`)
3. Click **Save**

An orange dot appears on the map and the table is saved to `waypoints.json`. Repeat for all tables (7–12 recommended).

**To remove a table:** click the orange dot on the map, or click ✕ in the sidebar list.

All changes persist in `waypoints.json` — you don't need to redo this after reboot.

---

## Step 4 — Delivery UI

Open on the touchscreen:
```
http://localhost:5000/
```
Or from a phone on the same WiFi:
```
http://<pi-ip>:5000/
```

**To send Walter on a delivery:**
1. Tap a table button in the right panel
2. Tap **Send Walter**
3. Walter navigates to the table, waits 10 seconds, then returns to `[0, 0]` (home position = where he started)
4. Tap **Cancel** at any time to stop

The blue dot on the map shows Walter's live position. The selected table turns green while delivering.

---

## Step 5 — Keyboard drive (optional)

Before or after placing tables, you can drive Walter manually from any PC on the network:
```
http://<pi-ip>:5000/static/drive.html
```

The Drive page reachable from the Admin header ("Drive" button, top right).

**Key bindings:**

| Key | Action |
|---|---|
| W / A / S / D or Arrow keys | Forward / Left / Backward / Right |
| Q / Z | Scale both speeds up / down 10% |
| = / - | Linear speed up / down 10% |
| E / C | Angular speed up / down 10% |
| Space or K | Emergency stop |

Speed starts at 0.05 m/s linear and 0.50 r/s angular — well below Walter's hardware limits. The current speed values are shown live on screen. The page stops the robot automatically if the browser window loses focus.

---

## Step 6 — Robot face screen

Open fullscreen on the robot's attached screen:
```
http://localhost:5000/static/face.html
```

The eyes automatically react to:
- **Blue / idle** — waiting for a delivery
- **Orange / moving** — travelling to a table
- **Purple / turning** — rotating in place
- **Green / arrived** — reached destination (pulsing)
- **Red / error** — navigation failure

To manually control the face from any Python script on the Pi:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class FacePublisher(Node):
    def __init__(self):
        super().__init__('face_pub')
        self.pub = self.create_publisher(String, '/walter/face', 10)

    def set_face(self, state):  # "arrived", "waiting", "idle", "error", etc.
        msg = String()
        msg.data = state
        self.pub.publish(msg)
```

Or from the shell (useful for testing):
```bash
docker exec -it walter_dev bash -c "
  source /opt/ros/humble/setup.bash &&
  ros2 topic pub --once /walter/face std_msgs/String '{data: arrived}'
"
```

---

## Step 7 — Auto-start on boot (optional)

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

Enable it:
```bash
sudo systemctl enable walter-web
sudo systemctl start walter-web
```

---

## Connecting hardware scripts (SPI / I2C / GPIO)

`web_server.py` runs on the Pi host — outside Docker — so it has **full direct access** to all Pi hardware:

```python
# These all work in web_server.py or any script it imports:
import smbus2          # I2C — battery fuel gauge, extra sensors
import spidev          # SPI — load cell ADC
import RPi.GPIO        # GPIO — LEDs, buttons, relays
```

**Pattern: background thread in web_server.py**

```python
import threading, spidev, time

spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1_000_000

load_cell_value = 0.0

def poll_load_cell():
    global load_cell_value
    while True:
        raw = spi.xfer2([0x00, 0x00])
        load_cell_value = (raw[0] << 8 | raw[1]) * 0.001  # scale to kg
        time.sleep(0.05)

threading.Thread(target=poll_load_cell, daemon=True).start()

@app.route('/api/weight')
def get_weight():
    return jsonify({'kg': load_cell_value})
```

The browser can then poll `/api/weight` every second, or you can push updates via the `/walter/face` ROS topic when the weight drops (item picked up / delivered).

---

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Map not showing in UI | rosbridge not running | Check `start_brain.sh` Stage 2 started rosbridge |
| Tables disappear after reboot | `waypoints.json` path wrong | `web_server.py` saves next to itself — run from `walter_bot/` directory |
| Robot doesn't move after goal | `use_sim_time: True` in nav2_params | Must be `False` everywhere |
| Face screen blank | rosbridge port 9090 blocked | Ensure Docker runs with `--network host` |
| `flask` not found | Not installed on host | `pip3 install flask` (host, not Docker) |
| SLAM `Failed to compute odom pose` | Wrong `base_frame` | Must be `base_link`, not `base_footprint` |
