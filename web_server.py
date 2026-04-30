#!/usr/bin/env python3
"""
Walter delivery web server — runs on Pi host (outside Docker), port 5000.
Serves the UI and manages waypoints.json.
The browser talks to ROS via rosbridge at ws://localhost:9090.
"""

from flask import Flask, jsonify, request, send_from_directory
import json, os, math

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


# ── Config API ────────────────────────────────────────────────────────────────

@app.route('/api/config', methods=['GET'])
def get_config():
    return jsonify({'delivery_wait_s': DELIVERY_WAIT_S})


if __name__ == '__main__':
    print("Walter web server starting on http://0.0.0.0:5000")
    print("   Main UI   : http://localhost:5000/")
    print("   Face      : http://localhost:5000/static/face.html")
    app.run(host='0.0.0.0', port=5000, debug=False)
