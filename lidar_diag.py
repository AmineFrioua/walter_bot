#!/usr/bin/env python3
"""
RPLIDAR A1M8 UART diagnostic — tests TX/RX communication.
Run on the Pi host (not inside Docker):
    python3 lidar_diag.py
"""

import serial
import time
import sys

PORT = "/dev/ttyAMA0"
BAUD = 115200

# RPLIDAR commands
CMD_STOP       = bytes([0xA5, 0x25])
CMD_RESET      = bytes([0xA5, 0x40])
CMD_GET_INFO   = bytes([0xA5, 0x50])
CMD_GET_HEALTH = bytes([0xA5, 0x52])
CMD_SCAN       = bytes([0xA5, 0x20])

# Response descriptor start bytes
RESP_SYNC1, RESP_SYNC2 = 0xA5, 0x5A


def open_port():
    print(f"[1] Opening {PORT} at {BAUD} baud...")
    try:
        s = serial.Serial(
            PORT, BAUD, timeout=2,
            rtscts=False, dsrdtr=False,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        print(f"    OK — port open: {s.name}")
        return s
    except Exception as e:
        print(f"    FAIL — {e}")
        sys.exit(1)


def flush(s):
    s.reset_input_buffer()
    s.reset_output_buffer()


def send(s, cmd, label):
    print(f"\n[TX] {label} → {cmd.hex()}")
    written = s.write(cmd)
    s.flush()
    print(f"     {written} byte(s) written")


def read_bytes(s, n, timeout=2.0):
    deadline = time.time() + timeout
    buf = b""
    while len(buf) < n and time.time() < deadline:
        chunk = s.read(n - len(buf))
        buf += chunk
    return buf


def check_descriptor(data):
    if len(data) < 7:
        print(f"    No/incomplete descriptor — got {len(data)} bytes: {data.hex()}")
        return None
    if data[0] != RESP_SYNC1 or data[1] != RESP_SYNC2:
        print(f"    Bad sync bytes: {data[:2].hex()} (expected a55a)")
        return None
    length = int.from_bytes(data[2:6], "little") & 0x3FFFFFFF
    rtype  = data[6]
    print(f"    Descriptor OK — data length: {length}, type: 0x{rtype:02x}")
    return length, rtype


# ── TEST 1: port open ────────────────────────────────────────────────────────
s = open_port()

# ── TEST 2: STOP (clears any ongoing scan) ───────────────────────────────────
print("\n[2] Sending STOP...")
flush(s)
send(s, CMD_STOP, "STOP")
time.sleep(0.1)
leftover = s.read_all()
print(f"    Drained {len(leftover)} leftover byte(s)")

# ── TEST 3: RESET ────────────────────────────────────────────────────────────
print("\n[3] Sending RESET and reading boot message...")
flush(s)
send(s, CMD_RESET, "RESET")
time.sleep(1.5)
data = s.read_all()
print(f"    Received {len(data)} bytes")
if data:
    print(f"    Raw hex : {data.hex()}")
    try:
        print(f"    As ASCII: {data.decode('ascii', errors='replace')}")
    except Exception:
        pass
else:
    print("    !! No response — TX line may not be reaching the LiDAR")

# ── TEST 4: GET_INFO ─────────────────────────────────────────────────────────
print("\n[4] Sending GET_INFO...")
flush(s)
send(s, CMD_GET_INFO, "GET_INFO")
desc = read_bytes(s, 7)
result = check_descriptor(desc)
if result:
    length, _ = result
    payload = read_bytes(s, length)
    print(f"    Payload ({len(payload)} bytes): {payload.hex()}")
    if len(payload) >= 4:
        print(f"    Model:    {payload[0]}")
        print(f"    Firmware: {payload[2]}.{payload[1]}")
        print(f"    Hardware: {payload[3]}")
    if len(payload) >= 20:
        serial_no = payload[4:20].hex()
        print(f"    Serial:   {serial_no}")

# ── TEST 5: GET_HEALTH ───────────────────────────────────────────────────────
print("\n[5] Sending GET_HEALTH...")
flush(s)
send(s, CMD_GET_HEALTH, "GET_HEALTH")
desc = read_bytes(s, 7)
result = check_descriptor(desc)
if result:
    length, _ = result
    payload = read_bytes(s, length)
    print(f"    Payload: {payload.hex()}")
    if len(payload) >= 1:
        status = payload[0]
        status_str = {0: "Good", 1: "Warning", 2: "Error"}.get(status, f"Unknown({status})")
        print(f"    Health:  {status_str}")

# ── TEST 6: SCAN — read 5 data points ────────────────────────────────────────
print("\n[6] Starting SCAN — reading 5 data points...")
flush(s)
send(s, CMD_SCAN, "SCAN")
desc = read_bytes(s, 7)
result = check_descriptor(desc)
if result:
    print("    Scan started. Reading data points...")
    for i in range(5):
        point = read_bytes(s, 5, timeout=3.0)
        if len(point) == 5:
            quality   = (point[0] >> 2) & 0x3F
            angle_q6  = ((point[1] >> 1) | (point[2] << 7))
            angle     = angle_q6 / 64.0
            dist_q2   = (point[3] | (point[4] << 8))
            distance  = dist_q2 / 4.0
            print(f"    Point {i+1}: angle={angle:.1f}° dist={distance:.1f}mm quality={quality}")
        else:
            print(f"    Point {i+1}: incomplete — got {len(point)} bytes")

# ── DONE ─────────────────────────────────────────────────────────────────────
print("\n[7] Sending STOP and closing...")
send(s, CMD_STOP, "STOP")
time.sleep(0.1)
s.close()
print("    Done.")
