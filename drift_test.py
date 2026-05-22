#!/usr/bin/env python3
"""
drift_test.py — Walter IMU-guided drift characterisation
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Run INSIDE the Docker container (Walter must be running):

  docker exec -it walter_dev bash -c \
    "source /opt/ros/humble/setup.bash && cd /ros2_ws && python3 drift_test.py"

Test sequence
  1. Stationary IMU calibration  (5 s — gyro bias estimation)
  2. Drive forward 1.0 m         (odom-gated, proportional slow-down)
  3. Rotate 180°                 (IMU-integrated, bias-corrected, proportional)
  4. Drive forward 1.0 m         (back toward start)
  5. Print full drift report

Ctrl+C at any point → robot stops immediately, partial report is printed.

Debugging notes
  • Console shows live progress + phase-start coordinates.
  • A CSV log is written to /tmp/drift_<timestamp>.csv with every control tick:
      t_s, state, odom_x, odom_y, odom_yaw_deg,
      imu_gz_raw, imu_gz_bias_corr, yaw_accum_deg,
      phase_dist_m, remaining_m, cmd_vx, cmd_az
  • On "robot never stops" bugs: grep the CSV for rows where phase_dist_m is
    stuck at 0 — that means phase_start_x/y was not captured (odom not ready).
  • On "overshoots" bugs: find the row where remaining_m first crosses 0 and
    compare cmd_vx at that row — should be LIN_MIN, not LIN_SPEED.
"""

import csv
import math
import os
import signal
import statistics
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# ── Test parameters ─────────────────────────────────────────────────────────────
TARGET_DIST    = 1.0        # metres per leg
TARGET_ANGLE   = math.pi    # 180°

LIN_SPEED      = 0.06       # m/s  cruise speed
LIN_MIN        = 0.03       # m/s  minimum speed in slow-down zone (prevents stall)
LIN_RAMP_DIST  = 0.20       # m    start proportional slow-down this far before target

ANG_SPEED      = 0.15       # rad/s  cruise angular speed
ANG_MIN        = 0.05       # rad/s  minimum angular speed in slow-down zone
ANG_RAMP_RAD   = 0.35       # rad    ~20° — start proportional slow-down before target

CALIB_S        = 5.0        # seconds of stationary IMU data for gyro-bias estimate
SETTLE_S       = 0.8        # pause between phases (s) — lets robot fully stop
CTRL_HZ        = 20         # cmd_vel publish rate (Hz)

LOG_DIR        = '/tmp'     # where to write the CSV debug log


# ── Node ─────────────────────────────────────────────────────────────────────────

class DriftTest(Node):

    _CALIB  = 'calibrating'
    _FWD1   = 'fwd1'
    _SETTLE = 'settling'
    _TURN   = 'turning'
    _FWD2   = 'fwd2'
    _DONE   = 'done'

    def __init__(self):
        super().__init__('drift_test')

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom',         self._odom_cb, 10)
        self.create_subscription(Imu,      '/imu/data_raw', self._imu_cb,  10)
        self._ctrl_timer = self.create_timer(1.0 / CTRL_HZ, self._tick)

        # ── Calibration ──────────────────────────────────────────────────────────
        self._calib_t0      = None
        self._gyro_samples  = []
        self._bias          = 0.0
        self._sigma         = 0.0

        # ── Raw IMU (latest sample, for CSV logging) ──────────────────────────────
        self._imu_gz_raw    = 0.0

        # ── Odometry — stored as raw floats, NOT object references ────────────────
        #
        # BUG HISTORY: previously used  self._phase_start = self._odom.pose.pose
        # which stores a Python object ref to the message sub-field.  In rclpy the
        # same buffer can be reused across callbacks, making the "snapshot" silently
        # follow the current position → _phase_dist() always 0 → robot never stops.
        # Fix: copy scalar values (float) at snapshot time.
        #
        self._odom              = None
        self._odom_ready        = False     # True once first message arrives
        self._odom_msg_count    = 0         # count for frequency check

        self._phase_start_x     = None      # float snapshot at start of drive phase
        self._phase_start_y     = None
        self._phase_start_yaw   = None

        self._global_start_x    = None      # float snapshot at very first message
        self._global_start_y    = None
        self._global_start_yaw  = None

        # ── IMU turn integration ──────────────────────────────────────────────────
        self._imu_prev_t        = None
        self._yaw_accum         = 0.0

        # ── State machine ─────────────────────────────────────────────────────────
        self._state             = self._CALIB
        self._next_state        = None
        self._settle_t0         = None
        self._phase_t0          = None      # wall-clock start of each phase

        # ── Results ───────────────────────────────────────────────────────────────
        self._r = dict(
            bias=None, sigma=None, n_calib=0,
            fwd1=None,  fwd1_dt=None,
            turn=None,  turn_dt=None,
            fwd2=None,  fwd2_dt=None,
            dx=None, dy=None, dheading=None,
        )

        # ── CSV log ───────────────────────────────────────────────────────────────
        ts_str    = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(LOG_DIR, f'drift_{ts_str}.csv')
        self._csv_f    = open(self._csv_path, 'w', newline='')
        self._csv_w    = csv.writer(self._csv_f)
        self._csv_w.writerow([
            't_s', 'state',
            'odom_x', 'odom_y', 'odom_yaw_deg',
            'imu_gz_raw', 'imu_gz_bias_corr', 'yaw_accum_deg',
            'phase_dist_m', 'remaining_m',
            'cmd_vx', 'cmd_az',
        ])
        self._t0       = time.monotonic()
        self._last_cmd = (0.0, 0.0)

        # ── Banner ────────────────────────────────────────────────────────────────
        print(f"\n{'─'*62}")
        print("  WALTER DRIFT TEST")
        print(f"  {datetime.now():%Y-%m-%d  %H:%M:%S}")
        print(f"{'─'*62}")
        print(f"  Target        : {TARGET_DIST} m × 2 legs  |  180° turn")
        print(f"  Cruise        : {LIN_SPEED} m/s  /  {ANG_SPEED} rad/s")
        print(f"  Slow-down     : last {LIN_RAMP_DIST} m  /  last {math.degrees(ANG_RAMP_RAD):.0f}°")
        print(f"  Settle pause  : {SETTLE_S} s")
        print(f"  CSV log       : {self._csv_path}")
        print(f"{'─'*62}")
        print(f"\n● Calibrating gyro — keep robot STILL for {CALIB_S:.0f} s …\n")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._odom = msg
        self._odom_msg_count += 1
        if not self._odom_ready:
            p = msg.pose.pose.position
            self._global_start_x   = p.x
            self._global_start_y   = p.y
            self._global_start_yaw = self._yaw_from_quat(msg.pose.pose.orientation)
            self._odom_ready = True
            print(f"\n  [odom] First message — "
                  f"x={p.x:.4f}  y={p.y:.4f}  "
                  f"yaw={math.degrees(self._global_start_yaw):.2f}°")

    def _imu_cb(self, msg: Imu):
        now = time.monotonic()
        gz  = msg.angular_velocity.z
        self._imu_gz_raw = gz

        if self._state == self._CALIB:
            if self._calib_t0 is None:
                self._calib_t0 = now
            self._gyro_samples.append(gz)
            n       = len(self._gyro_samples)
            elapsed = now - self._calib_t0
            if n % 50 == 0:
                print(f"  … {elapsed:.1f}/{CALIB_S:.0f} s  "
                      f"({n} samples,  gz={gz:+.5f} r/s)", end='\r', flush=True)

        elif self._state == self._TURN and self._imu_prev_t is not None:
            dt = now - self._imu_prev_t
            self._yaw_accum += (gz - self._bias) * dt

        self._imu_prev_t = now

    # ── Control tick (CTRL_HZ) ─────────────────────────────────────────────────

    def _tick(self):
        if self._state == self._DONE:
            return

        t_s = time.monotonic() - self._t0

        # ── Calibration ────────────────────────────────────────────────────────
        if self._state == self._CALIB:
            if self._calib_t0 and time.monotonic() - self._calib_t0 >= CALIB_S:
                self._end_calib()
            self._write_csv(t_s, 0.0, 0.0, 0.0, 0.0)
            return

        # ── Settle pause ───────────────────────────────────────────────────────
        if self._state == self._SETTLE:
            self._cmd(0.0, 0.0)
            if time.monotonic() - self._settle_t0 >= SETTLE_S:
                self._begin(self._next_state)
            self._write_csv(t_s, 0.0, 0.0, 0.0, 0.0)
            return

        # Drive states need odom
        if not self._odom_ready or self._odom is None:
            print("  [WARN] waiting for /odom …", end='\r', flush=True)
            return

        # ── Lazy phase-start capture ───────────────────────────────────────────
        # Defensive: _begin() sets phase_start from odom; if odom wasn't ready
        # at that moment, phase_start_x is None and we capture it here instead.
        if self._phase_start_x is None:
            p = self._odom.pose.pose.position
            self._phase_start_x   = p.x
            self._phase_start_y   = p.y
            self._phase_start_yaw = self._yaw_from_quat(self._odom.pose.pose.orientation)
            print(f"\n  [DBG] lazy phase-start captured — "
                  f"x={p.x:.4f}  y={p.y:.4f}  "
                  f"yaw={math.degrees(self._phase_start_yaw):.2f}°")

        d         = self._phase_dist()
        yaw_deg   = math.degrees(self._yaw_accum)
        cur_odom_yaw = math.degrees(
            self._yaw_from_quat(self._odom.pose.pose.orientation))

        # ── Forward leg 1 ──────────────────────────────────────────────────────
        if self._state == self._FWD1:
            remaining = TARGET_DIST - d
            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                self._r['fwd1']    = d
                self._r['fwd1_dt'] = time.monotonic() - self._phase_t0
                p = self._odom.pose.pose.position
                print(f"\n  ✓ Leg 1 done  dist={d:.4f} m  "
                      f"Δ={d - TARGET_DIST:+.4f} m  "
                      f"time={self._r['fwd1_dt']:.1f} s")
                print(f"    end pose  x={p.x:.4f}  y={p.y:.4f}  "
                      f"yaw={cur_odom_yaw:.2f}°")
                self._write_csv(t_s, d, remaining, 0.0, 0.0)
                self._settle_to(self._TURN)
                return
            speed = self._ramp(remaining, LIN_RAMP_DIST, LIN_MIN, LIN_SPEED)
            # Log entry into slow-down zone (once)
            if remaining < LIN_RAMP_DIST and d > 0.01:
                pass   # continuous progress line is enough
            print(f"  → fwd1  {d:.3f}/{TARGET_DIST:.2f} m  "
                  f"remain={remaining:.3f} m  "
                  f"speed={speed:.3f} m/s       ", end='\r', flush=True)
            self._cmd(speed, 0.0)
            self._write_csv(t_s, d, remaining, speed, 0.0)

        # ── 180° rotation ──────────────────────────────────────────────────────
        elif self._state == self._TURN:
            a         = abs(self._yaw_accum)
            remaining = TARGET_ANGLE - a
            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                self._r['turn']    = self._yaw_accum
                self._r['turn_dt'] = time.monotonic() - self._phase_t0
                p = self._odom.pose.pose.position
                print(f"\n  ✓ Turn done   angle={math.degrees(self._yaw_accum):.2f}°  "
                      f"Δ={math.degrees(self._yaw_accum) - 180.0:+.2f}°  "
                      f"time={self._r['turn_dt']:.1f} s")
                print(f"    end pose  x={p.x:.4f}  y={p.y:.4f}  "
                      f"yaw(odom)={cur_odom_yaw:.2f}°  "
                      f"yaw(IMU)={math.degrees(self._yaw_accum):.2f}°")
                self._write_csv(t_s, a, remaining, 0.0, 0.0)
                self._settle_to(self._FWD2)
                return
            # Log every 45° milestone
            milestone = int(math.degrees(a) / 45)
            if not hasattr(self, '_turn_milestone') or milestone > self._turn_milestone:
                self._turn_milestone = milestone
                bias_corr = self._imu_gz_raw - self._bias
                print(f"\n  [IMU] {math.degrees(a):.1f}°  "
                      f"gz_raw={self._imu_gz_raw:+.5f}  "
                      f"gz_corr={bias_corr:+.5f}  "
                      f"yaw_accum={math.degrees(self._yaw_accum):.2f}°")
            ang = self._ramp(remaining, ANG_RAMP_RAD, ANG_MIN, ANG_SPEED)
            print(f"  → turn  {math.degrees(a):.1f}°/{math.degrees(TARGET_ANGLE):.0f}°  "
                  f"remain={math.degrees(remaining):.1f}°  "
                  f"speed={ang:.3f} r/s       ", end='\r', flush=True)
            self._cmd(0.0, ang)
            self._write_csv(t_s, a, remaining, 0.0, ang)

        # ── Forward leg 2 ──────────────────────────────────────────────────────
        elif self._state == self._FWD2:
            remaining = TARGET_DIST - d
            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                self._r['fwd2']    = d
                self._r['fwd2_dt'] = time.monotonic() - self._phase_t0
                p = self._odom.pose.pose.position
                print(f"\n  ✓ Leg 2 done  dist={d:.4f} m  "
                      f"Δ={d - TARGET_DIST:+.4f} m  "
                      f"time={self._r['fwd2_dt']:.1f} s")
                print(f"    end pose  x={p.x:.4f}  y={p.y:.4f}  "
                      f"yaw={cur_odom_yaw:.2f}°")
                self._write_csv(t_s, d, remaining, 0.0, 0.0)
                self._finish()
                return
            speed = self._ramp(remaining, LIN_RAMP_DIST, LIN_MIN, LIN_SPEED)
            print(f"  → fwd2  {d:.3f}/{TARGET_DIST:.2f} m  "
                  f"remain={remaining:.3f} m  "
                  f"speed={speed:.3f} m/s       ", end='\r', flush=True)
            self._cmd(speed, 0.0)
            self._write_csv(t_s, d, remaining, speed, 0.0)

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _ramp(remaining: float, zone: float, v_min: float, v_max: float) -> float:
        """Full speed outside the ramp zone; proportional ramp inside it."""
        if remaining >= zone:
            return v_max
        return max(v_min, v_max * (remaining / zone))

    def _cmd(self, lx: float, az: float):
        self._last_cmd = (lx, az)
        t = Twist()
        t.linear.x  = lx
        t.angular.z = az
        self._pub.publish(t)

    def _phase_dist(self) -> float:
        """Euclidean distance (m) from phase-start snapshot."""
        if self._phase_start_x is None or self._odom is None:
            return 0.0
        p = self._odom.pose.pose.position
        return math.hypot(p.x - self._phase_start_x,
                          p.y - self._phase_start_y)

    def _begin(self, state: str):
        """Transition to a new drive state. Snapshot odom as scalar floats."""
        self._state    = state
        self._phase_t0 = time.monotonic()
        if hasattr(self, '_turn_milestone'):
            del self._turn_milestone

        if self._odom_ready and self._odom is not None:
            p = self._odom.pose.pose.position
            self._phase_start_x   = p.x
            self._phase_start_y   = p.y
            self._phase_start_yaw = self._yaw_from_quat(self._odom.pose.pose.orientation)
            print(f"\n  [DBG] phase-start captured — "
                  f"x={p.x:.4f}  y={p.y:.4f}  "
                  f"yaw={math.degrees(self._phase_start_yaw):.2f}°")
        else:
            # odom not yet ready — lazy capture will happen in first tick
            self._phase_start_x   = None
            self._phase_start_y   = None
            self._phase_start_yaw = None
            print(f"\n  [WARN] odom not ready at _begin({state}) — "
                  "will capture on first tick")

        if state == self._TURN:
            self._yaw_accum  = 0.0
            self._imu_prev_t = None

        labels = {
            self._FWD1: f'● Phase 1 — driving {TARGET_DIST} m forward …',
            self._TURN: '● Phase 2 — rotating 180° (IMU + bias correction) …',
            self._FWD2: f'● Phase 3 — returning {TARGET_DIST} m …',
        }
        if state in labels:
            print(f"\n{labels[state]}")

    def _settle_to(self, next_state: str):
        self._state      = self._SETTLE
        self._next_state = next_state
        self._settle_t0  = time.monotonic()

    def _end_calib(self):
        n = len(self._gyro_samples)
        if n < 2:
            print("\n  ✗ Not enough IMU samples — is /imu/data_raw publishing?")
            return
        self._bias  = statistics.mean(self._gyro_samples)
        self._sigma = statistics.stdev(self._gyro_samples)
        self._r.update(bias=self._bias, sigma=self._sigma, n_calib=n)

        # Odom health check
        if not self._odom_ready:
            print("\n  ⚠  /odom not yet received — check that odom node is running")
        else:
            p   = self._odom.pose.pose.position
            yaw = math.degrees(self._yaw_from_quat(self._odom.pose.pose.orientation))
            freq_est = self._odom_msg_count / CALIB_S
            print(f"\n  [odom] health  x={p.x:.4f}  y={p.y:.4f}  "
                  f"yaw={yaw:.2f}°  "
                  f"~{freq_est:.1f} Hz ({self._odom_msg_count} msgs in {CALIB_S:.0f} s)")
            if freq_est < 5:
                print(f"  ⚠  Odom publishing slowly ({freq_est:.1f} Hz) — "
                      "distance gating may be coarse")

        print(f"\n  ✓ Gyro bias  : {self._bias:+.6f} rad/s")
        print(f"    Gyro σ     : {self._sigma:.6f} rad/s  ({n} samples)")
        if self._sigma > 0.02:
            print("  ⚠  High gyro noise — was the robot moving during calibration?")

        self._begin(self._FWD1)

    def _finish(self):
        self._state = self._DONE
        if self._odom_ready and self._odom is not None:
            p = self._odom.pose.pose.position
            self._r['dx'] = p.x - self._global_start_x
            self._r['dy'] = p.y - self._global_start_y
            y1 = self._yaw_from_quat(self._odom.pose.pose.orientation)
            dh = y1 - self._global_start_yaw
            self._r['dheading'] = math.degrees(math.atan2(math.sin(dh), math.cos(dh)))
        self._csv_f.close()
        print(f"\n  [log] CSV written → {self._csv_path}")
        self.print_report(interrupted=False)

    @staticmethod
    def _yaw_from_quat(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _write_csv(self, t_s: float,
                   phase_dist: float, remaining: float,
                   cmd_vx: float, cmd_az: float):
        if self._odom is None:
            return
        p   = self._odom.pose.pose.position
        yaw = math.degrees(self._yaw_from_quat(self._odom.pose.pose.orientation))
        self._csv_w.writerow([
            f'{t_s:.3f}',
            self._state,
            f'{p.x:.5f}', f'{p.y:.5f}', f'{yaw:.3f}',
            f'{self._imu_gz_raw:.6f}',
            f'{self._imu_gz_raw - self._bias:.6f}',
            f'{math.degrees(self._yaw_accum):.3f}',
            f'{phase_dist:.5f}',
            f'{remaining:.5f}',
            f'{cmd_vx:.4f}', f'{cmd_az:.4f}',
        ])

    def stop(self):
        self._cmd(0.0, 0.0)

    # ── Report ─────────────────────────────────────────────────────────────────

    def print_report(self, interrupted: bool = False):
        r   = self._r
        W   = 62
        tag = '  [INTERRUPTED]' if interrupted else ''

        def row(label, value):
            print(f"    {label:<28}: {value}")

        print(f"\n{'═'*W}")
        print(f"  DRIFT TEST REPORT{tag}")
        print(f"  {datetime.now():%Y-%m-%d  %H:%M:%S}")
        print(f"{'═'*W}")

        print(f"\n  ── IMU CALIBRATION ──────────────────────────────────────")
        if r['bias'] is not None:
            row("Gyro Z bias",     f"{r['bias']:+.6f} rad/s")
            row("Gyro noise σ",    f"{r['sigma']:.6f} rad/s  ({r['n_calib']} samples)")
        else:
            row("Status", "not completed")

        print(f"\n  ── PHASE 1  Forward {TARGET_DIST:.1f} m ──────────────────────────")
        if r['fwd1'] is not None:
            e = r['fwd1'] - TARGET_DIST
            row("Commanded",       f"{TARGET_DIST:.3f} m")
            row("Actual (odom)",   f"{r['fwd1']:.4f} m")
            row("Error",           f"{e:+.4f} m  ({e / TARGET_DIST * 100:+.2f}%)")
            row("Time",            f"{r['fwd1_dt']:.2f} s")
        else:
            row("Status", "not completed")

        print(f"\n  ── PHASE 2  Rotate 180° ────────────────────────────────────")
        if r['turn'] is not None:
            deg = math.degrees(r['turn'])
            e   = deg - 180.0
            row("Commanded",       "180.00°")
            row("Actual (IMU)",    f"{deg:.2f}°")
            row("Error",           f"{e:+.2f}°")
            row("Time",            f"{r['turn_dt']:.2f} s")
        else:
            row("Status", "not completed")

        print(f"\n  ── PHASE 3  Return {TARGET_DIST:.1f} m ──────────────────────────")
        if r['fwd2'] is not None:
            e = r['fwd2'] - TARGET_DIST
            row("Commanded",       f"{TARGET_DIST:.3f} m")
            row("Actual (odom)",   f"{r['fwd2']:.4f} m")
            row("Error",           f"{e:+.4f} m  ({e / TARGET_DIST * 100:+.2f}%)")
            row("Time",            f"{r['fwd2_dt']:.2f} s")
        else:
            row("Status", "not completed")

        print(f"\n  ── FINAL POSITION (odom vs start) ──────────────────────────")
        if r['dx'] is not None:
            drift = math.hypot(r['dx'], r['dy'])
            pct   = drift / (2 * TARGET_DIST) * 100
            row("ΔX (odom)",       f"{r['dx']:+.4f} m")
            row("ΔY (odom)",       f"{r['dy']:+.4f} m")
            row("Position drift",  f"{drift:.4f} m  ({pct:.2f}% of {2*TARGET_DIST:.1f} m)")
            row("Heading drift",   f"{r['dheading']:+.2f}°")
        else:
            row("Status", "not completed")

        if not interrupted:
            print(f"\n  ── DEBUG ────────────────────────────────────────────────────")
            row("CSV log", self._csv_path)
            print(f"    To inspect: column 'phase_dist_m' stuck at 0 → phase_start")
            print(f"    was not captured (odom unavailable at phase begin).")
            print(f"    Overshoot: find first row where remaining_m ≤ 0, check cmd_vx.")

        print(f"\n{'═'*W}\n")


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = DriftTest()

    def _on_sigint(sig, frame):
        print('\n\n  ⚠  Ctrl+C — stopping robot …')
        node.stop()
        try:
            node._csv_f.close()
        except Exception:
            pass
        if node._state not in (DriftTest._CALIB, DriftTest._DONE):
            node.print_report(interrupted=True)
        else:
            print("  (no movement data to report)\n")
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        while rclpy.ok() and node._state != DriftTest._DONE:
            rclpy.spin_once(node, timeout_sec=0.05)
    except SystemExit:
        pass
    except Exception as exc:
        print(f"\n  ✗ Fatal: {exc}")
        node.stop()
    finally:
        if node._state not in (DriftTest._DONE,):
            node.stop()
        try:
            node._csv_f.close()
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
