#!/usr/bin/env python3
"""
Walter delivery web server — runs on Pi host (outside Docker), port 5000.
Serves the UI and manages waypoints.json.
The browser talks to ROS via rosbridge at ws://localhost:9090.
"""

from flask import Flask, jsonify, request, send_from_directory
import json, os, math, subprocess, re, struct

try:
    from smbus2 import SMBus as _SMBus
    _HAS_SMBUS = True
except ImportError:
    _HAS_SMBUS = False

app = Flask(__name__, static_folder='static')

WAYPOINTS_FILE = os.path.join(os.path.dirname(__file__), 'waypoints.json')
DELIVERY_WAIT_S = 10  # seconds to wait at destination before returning home

# ── Battery config (BQ25820 charger IC at I2C 0x6B) ──────────────────────────
# Voltage range for percentage estimate — adjust to match your pack chemistry.
# Defaults: 3S Li-ion/LiPo (9.0 V empty → 12.6 V full).
BATTERY_MIN_V   = 9.0
BATTERY_MAX_V   = 12.6
_BQ25820_ADDR   = 0x6B

# BQ25820 register addresses (all 16-bit, little-endian: low byte at addr, high at addr+1)
_BQ_ADC_CTRL    = 0x2B  # bit7=ADC_EN, bit6=ADC_RATE(0=continuous)
_BQ_STATUS_1    = 0x21  # bits[2:0]=CHARGE_STAT, bit7=ADC_DONE_STAT
_BQ_STATUS_2    = 0x22  # bit7=PG_STAT (power good / charger connected)
_BQ_IBAT_ADC    = 0x2F  # signed 16-bit, 2 mA/LSB, range ±20 000 mA
_BQ_VBAT_ADC    = 0x33  # unsigned 16-bit, 2 mV/LSB, range 0–65 534 mV
_BQ_VSYS_ADC    = 0x35  # unsigned 16-bit, 2 mV/LSB

_BQ_CHARGE_STAT = {0: 'not_charging', 1: 'trickle', 2: 'pre_charge',
                   3: 'fast_charge', 4: 'taper_charge'}


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


# ── Battery API ───────────────────────────────────────────────────────────────

def _bq_read_byte(bus, reg):
    return bus.read_byte_data(_BQ25820_ADDR, reg)

def _bq_read_word(bus, reg):
    """Read a 16-bit little-endian value: low byte at reg, high byte at reg+1."""
    lo = bus.read_byte_data(_BQ25820_ADDR, reg)
    hi = bus.read_byte_data(_BQ25820_ADDR, reg + 1)
    return (hi << 8) | lo


def _bq25820_read():
    """
    Read VBAT, IBAT, VSYS and charge status from the BQ25820 at I2C 0x6B.
    Enables continuous ADC on first call. Returns a dict or None on failure.

    Register reference (datasheet SLUSFN3A):
      0x2B  ADC_Control    bit7=ADC_EN, bit6=ADC_RATE(0=continuous)
      0x21  Charger_Status_1  bits[2:0]=CHARGE_STAT, bit7=ADC_DONE_STAT
      0x22  Charger_Status_2  bit7=PG_STAT
      0x2F/0x30  IBAT_ADC  signed 16-bit, 2 mA/LSB
      0x33/0x34  VBAT_ADC  unsigned 16-bit, 2 mV/LSB
      0x35/0x36  VSYS_ADC  unsigned 16-bit, 2 mV/LSB
    """
    if not _HAS_SMBUS:
        return None
    try:
        with _SMBus(1) as bus:
            # Enable ADC in continuous mode if not already running
            adc_ctrl = _bq_read_byte(bus, _BQ_ADC_CTRL)
            if not (adc_ctrl & 0x80):
                bus.write_byte_data(_BQ25820_ADDR, _BQ_ADC_CTRL,
                                    (adc_ctrl & 0x3F) | 0x80)  # ADC_EN=1, RATE=continuous

            # VBAT — unsigned 16-bit, 2 mV/LSB
            vbat_raw  = _bq_read_word(bus, _BQ_VBAT_ADC)
            vbat_mv   = vbat_raw * 2
            vbat_v    = vbat_mv / 1000.0

            # IBAT — signed 16-bit 2s complement, 2 mA/LSB
            # Positive = charging, Negative = discharging
            ibat_raw  = _bq_read_word(bus, _BQ_IBAT_ADC)
            if ibat_raw > 32767:
                ibat_raw -= 65536
            ibat_ma   = ibat_raw * 2

            # VSYS — unsigned 16-bit, 2 mV/LSB
            vsys_raw  = _bq_read_word(bus, _BQ_VSYS_ADC)
            vsys_v    = vsys_raw * 2 / 1000.0

            # Charge status
            status1   = _bq_read_byte(bus, _BQ_STATUS_1)
            status2   = _bq_read_byte(bus, _BQ_STATUS_2)
            charge_stat = status1 & 0x07
            power_good  = bool(status2 & 0x80)

        return {
            'vbat_v':      round(vbat_v, 3),
            'ibat_ma':     round(ibat_ma, 0),
            'vsys_v':      round(vsys_v, 3),
            'charge_stat': _BQ_CHARGE_STAT.get(charge_stat, str(charge_stat)),
            'power_good':  power_good,
        }
    except Exception:
        return None


@app.route('/api/battery', methods=['GET'])
def get_battery():
    """
    Return battery state from the BQ25820 charger IC (I2C 0x6B).

    Response:
      voltage_v    — VBAT in volts
      current_ma   — IBAT in mA (+ve = charging, -ve = discharging)
      vsys_v       — system rail voltage
      percent      — 0–100 estimate based on BATTERY_MIN_V / BATTERY_MAX_V
      status       — "ok" | "low" (<25 %) | "critical" (<10 %) | "unknown"
      charge_stat  — "not_charging" | "trickle" | "pre_charge" | "fast_charge" | "taper_charge"
      power_good   — true if a valid input source (charger) is connected
      source       — "bq25820@0x6b"
    """
    data = _bq25820_read()
    if data and data['vbat_v'] > 0.1:
        v   = data['vbat_v']
        pct = int(min(100, max(0,
            (v - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V) * 100)))
        status = 'critical' if pct < 10 else 'low' if pct < 25 else 'ok'
        return jsonify({
            'voltage_v':   v,
            'current_ma':  data['ibat_ma'],
            'vsys_v':      data['vsys_v'],
            'percent':     pct,
            'status':      status,
            'charge_stat': data['charge_stat'],
            'power_good':  data['power_good'],
            'source':      'bq25820@0x6b',
        })

    return jsonify({
        'voltage_v':   None,
        'current_ma':  None,
        'vsys_v':      None,
        'percent':     None,
        'status':      'unknown',
        'charge_stat': None,
        'power_good':  None,
        'source':      'unavailable',
    })


if __name__ == '__main__':
    print("Walter web server starting on http://0.0.0.0:5000")
    print("   Main UI   : http://localhost:5000/")
    print("   Face      : http://localhost:5000/static/face.html")
    app.run(host='0.0.0.0', port=5000, debug=False)
