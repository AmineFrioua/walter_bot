#!/usr/bin/env python3
"""
Walter delivery web server — runs on Pi host (outside Docker), port 5000.
Serves the UI and manages waypoints.json.
The browser talks to ROS via rosbridge at ws://localhost:9090.
"""

from flask import Flask, jsonify, request, send_from_directory
import json, os, math, subprocess, re, struct

app = Flask(__name__, static_folder='static')

WAYPOINTS_FILE = os.path.join(os.path.dirname(__file__), 'waypoints.json')
DELIVERY_WAIT_S = 10  # seconds to wait at destination before returning home


def load_waypoints():
    if not os.path.exists(WAYPOINTS_FILE):
        return {}
    with open(WAYPOINTS_FILE) as f:
        return json.load(f)


def save_waypoints(data):
    with open(WAYPOINTS_FILE, 'w') as f:
        json.dump(data, f, indent=2)


# ── Static UI ─────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return send_from_directory('static', 'index.html')

@app.route('/admin')
def admin():
    return send_from_directory('static', 'admin.html')


# ── Waypoints API ─────────────────────────────────────────────────────────────

@app.route('/api/waypoints', methods=['GET'])
def get_waypoints():
    return jsonify(load_waypoints())

@app.route('/api/waypoints/<name>', methods=['PUT'])
def put_waypoint(name):
    body = request.get_json()
    if not body or 'x' not in body or 'y' not in body:
        return jsonify({'error': 'x and y required'}), 400
    waypoints = load_waypoints()
    # Preserve all fields from the request body so extended schema
    # (type, number, width, depth, etc.) round-trips correctly.
    entry = {k: v for k, v in body.items()}
    # Coerce mandatory numeric fields
    entry['x'] = float(body['x'])
    entry['y'] = float(body['y'])
    entry['theta'] = float(body.get('theta', 0.0))
    entry.setdefault('label', name)
    waypoints[name] = entry
    save_waypoints(waypoints)
    return jsonify(waypoints[name])

@app.route('/api/waypoints/<name>', methods=['DELETE'])
def delete_waypoint(name):
    waypoints = load_waypoints()
    if name not in waypoints:
        return jsonify({'error': 'not found'}), 404
    del waypoints[name]
    save_waypoints(waypoints)
    return jsonify({'ok': True})


# ── Map save API ──────────────────────────────────────────────────────────────

@app.route('/api/save_map', methods=['POST'])
def save_map():
    body = request.get_json(silent=True) or {}
    map_name = body.get('name', 'room_map').strip() or 'room_map'
    # Sanitise: only allow alphanumeric + underscore/hyphen
    map_name = ''.join(c for c in map_name if c.isalnum() or c in ('_', '-'))
    try:
        result = subprocess.run(
            ['docker', 'exec', 'walter_dev', 'bash', '/ros2_ws/save_map.sh', map_name],
            capture_output=True, text=True, timeout=15,
        )
        if result.returncode == 0:
            print(f'[save_map] ✅ saved as "{map_name}"', flush=True)
            return jsonify({'ok': True, 'name': map_name, 'output': result.stdout.strip()})
        else:
            error_msg = result.stdout.strip() or result.stderr.strip() or 'unknown error'
            print(f'[save_map] ❌ failed: {error_msg}', flush=True)
            return jsonify({'ok': False, 'error': error_msg}), 500
    except subprocess.TimeoutExpired:
        msg = 'Timed out — is SLAM Toolbox running?'
        print(f'[save_map] ❌ {msg}', flush=True)
        return jsonify({'ok': False, 'error': msg}), 500
    except Exception as e:
        print(f'[save_map] ❌ exception: {e}', flush=True)
        return jsonify({'ok': False, 'error': str(e)}), 500


# ── Config API ────────────────────────────────────────────────────────────────

@app.route('/api/config', methods=['GET'])
def get_config():
    return jsonify({'delivery_wait_s': DELIVERY_WAIT_S})


# ── Saved-map API ─────────────────────────────────────────────────────────────
# Maps are saved inside the Docker volume which maps to the repo root on the Pi:
#   Docker /ros2_ws/maps/  ==  Pi  <repo>/maps/
MAPS_DIR = os.path.join(os.path.dirname(__file__), 'maps')


def _find_map(name):
    """Return (yaml_path, pgm_path) for a saved map, or (None, None)."""
    name = ''.join(c for c in name if c.isalnum() or c in ('_', '-'))
    for d in [MAPS_DIR, '/ros2_ws/maps']:
        yf = os.path.join(d, name + '.yaml')
        pf = os.path.join(d, name + '.pgm')
        if os.path.exists(yf) and os.path.exists(pf):
            return yf, pf
    return None, None


@app.route('/api/maps', methods=['GET'])
def list_maps():
    """List available saved maps (name only)."""
    seen = {}
    for d in [MAPS_DIR, '/ros2_ws/maps']:
        if not os.path.isdir(d):
            continue
        for f in os.listdir(d):
            if f.endswith('.yaml'):
                n = f[:-5]
                if os.path.exists(os.path.join(d, n + '.pgm')) and n not in seen:
                    seen[n] = True
    return jsonify(sorted(seen.keys()))


@app.route('/api/maps/<name>/data', methods=['GET'])
def get_map_data(name):
    """
    Return a saved PGM map as OccupancyGrid-like JSON so the editor can display
    it even when ROS is not running (or map_server's TRANSIENT_LOCAL publish was
    missed by the browser's VOLATILE rosbridge subscriber).

    Response shape mirrors nav_msgs/OccupancyGrid:
      { info: { width, height, resolution, origin: { position: {x,y,z}, orientation: {x,y,z,w} } },
        data: [int8, ...] }   # 0=free, 100=occupied, -1=unknown
    """
    yaml_path, pgm_path = _find_map(name)
    if not yaml_path:
        return jsonify({'error': f'Map "{name}" not found'}), 404

    # ── Parse YAML metadata (simple key: value, no library needed) ────────────
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            line = line.strip()
            if ':' in line and not line.startswith('#'):
                k, v = line.split(':', 1)
                meta[k.strip()] = v.strip()

    resolution       = float(meta.get('resolution', 0.05))
    negate           = int(meta.get('negate', 0))
    occupied_thresh  = float(meta.get('occupied_thresh', 0.65))
    free_thresh      = float(meta.get('free_thresh', 0.196))

    # origin: "[x, y, z]"  — extract numbers
    origin_str  = meta.get('origin', '[0, 0, 0]')
    origin_vals = [float(v) for v in re.findall(r'[-+]?\d+\.?\d*(?:[eE][-+]?\d+)?', origin_str)]
    ox = origin_vals[0] if len(origin_vals) > 0 else 0.0
    oy = origin_vals[1] if len(origin_vals) > 1 else 0.0

    # ── Parse PGM file ────────────────────────────────────────────────────────
    try:
        with open(pgm_path, 'rb') as f:
            magic = f.readline().strip()
            if magic not in (b'P5', b'P2'):
                return jsonify({'error': f'Unsupported PGM type: {magic}'}), 500

            # skip comment lines
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()

            width, height = map(int, line.split())
            maxval = int(f.readline().strip())

            if magic == b'P5':
                raw = f.read(width * height * (2 if maxval > 255 else 1))
                if maxval > 255:
                    pixels = [struct.unpack('>H', raw[i*2:i*2+2])[0]
                              for i in range(width * height)]
                else:
                    pixels = list(raw)
            else:  # P2 ASCII
                pixels = list(map(int, f.read().split()))

    except Exception as e:
        return jsonify({'error': f'Failed to parse PGM: {e}'}), 500

    # ── Convert to OccupancyGrid values ──────────────────────────────────────
    # ROS convention (negate=0): occ = (maxval - pixel) / maxval
    #   pixel≈0   (black)  → occ≈1.0 → occupied (100)
    #   pixel≈254 (white)  → occ≈0.0 → free     (0)
    #   pixel≈205 (gray)   → occ≈0.2 → unknown  (-1)
    data = []
    for p in pixels:
        occ = (p / maxval) if negate else ((maxval - p) / maxval)
        if occ > occupied_thresh:
            data.append(100)
        elif occ < free_thresh:
            data.append(0)
        else:
            data.append(-1)

    return jsonify({
        'info': {
            'width':      width,
            'height':     height,
            'resolution': resolution,
            'origin': {
                'position':    {'x': ox,  'y': oy,  'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            },
        },
        'data': data,
    })


if __name__ == '__main__':
    print("Walter web server starting on http://0.0.0.0:5000")
    print("   Main UI   : http://localhost:5000/")
    print("   Face      : http://localhost:5000/static/face.html")
    app.run(host='0.0.0.0', port=5000, debug=False)
