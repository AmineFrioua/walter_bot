#!/usr/bin/env python3
"""
drift_test.py — Walter IMU-guided drift characterisation
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Run INSIDE the Docker container (Walter must be running):

  docker exec -it walter_dev bash -c \
    "source /opt/ros/humble/setup.bash && cd /ros2_ws && python3 drift_test.py"

Test sequence
  1. Stationary IMU calibration  (5 s — gyro bias estimation)
  2. Drive forward 1.0 m         (odom-gated, low speed)
  3. Rotate 180°                 (IMU-integrated + bias-corrected)
  4. Drive forward 1.0 m         (back toward start)
  5. Print full drift report

Ctrl+C at any point → robot stops immediately, partial report is printed.
"""

import math
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


# ── Test parameters ────────────────────────────────────────────────────────────
TARGET_DIST  = 1.0    # metres per leg
TARGET_ANGLE = math.pi  # 180°
LIN_SPEED    = 0.04   # m/s  — slow for accuracy
ANG_SPEED    = 0.02   # rad/s — slow for accuracy
CALIB_S      = 5.0    # seconds of stationary data for gyro-bias estimate
SETTLE_S     = 0.5    # pause between phases to let the robot fully stop
CTRL_HZ      = 20     # cmd_vel publish rate (Hz)


# ── Node ───────────────────────────────────────────────────────────────────────

class DriftTest(Node):

    # States
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

        # Calibration
        self._calib_t0     = None
        self._gyro_samples = []
        self._bias         = 0.0
        self._sigma        = 0.0

        # Odometry
        self._odom         = None
        self._phase_start  = None   # pose snapshot at start of each drive phase
        self._global_start = None   # pose snapshot at very first odom message

        # IMU turn integration
        self._imu_prev_t   = None
        self._yaw_accum    = 0.0    # integrated bias-corrected yaw for current turn

        # State machine
        self._state        = self._CALIB
        self._next_state   = None
        self._settle_t0    = None

        # Results dict (filled in as phases complete)
        self._r = dict(
            bias=None, sigma=None, n_calib=0,
            fwd1=None,
            turn=None,
            fwd2=None,
            dx=None, dy=None, dheading=None,
        )

        self._interrupted = False

        print(f"\n{'─'*58}")
        print("  WALTER DRIFT TEST")
        print(f"  {datetime.now():%Y-%m-%d  %H:%M:%S}")
        print(f"{'─'*58}")
        print(f"  Distance  : {TARGET_DIST} m × 2 legs  |  Angle : 180°")
        print(f"  Speed     : {LIN_SPEED} m/s linear  /  {ANG_SPEED} rad/s angular")
        print(f"{'─'*58}")
        print(f"\n● Calibrating gyro — keep robot STILL for {CALIB_S:.0f} s …\n")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._odom = msg
        if self._global_start is None:
            self._global_start = msg.pose.pose

    def _imu_cb(self, msg: Imu):
        now = time.monotonic()
        gz  = msg.angular_velocity.z

        if self._state == self._CALIB:
            if self._calib_t0 is None:
                self._calib_t0 = now
            self._gyro_samples.append(gz)
            n       = len(self._gyro_samples)
            elapsed = now - self._calib_t0
            if n % 50 == 0:
                print(f"  … {elapsed:.1f}/{CALIB_S:.0f} s  ({n} samples)", end='\r', flush=True)

        elif self._state == self._TURN and self._imu_prev_t is not None:
            dt = now - self._imu_prev_t
            self._yaw_accum += (gz - self._bias) * dt

        self._imu_prev_t = now

    # ── Control tick (runs at CTRL_HZ) ─────────────────────────────────────────

    def _tick(self):
        if self._state == self._DONE:
            return

        # ── Calibration phase ──────────────────────────────────────────────────
        if self._state == self._CALIB:
            if self._calib_t0 and time.monotonic() - self._calib_t0 >= CALIB_S:
                self._end_calib()
            return

        # ── Inter-phase settle (publish zero until timer expires) ──────────────
        if self._state == self._SETTLE:
            self._cmd(0.0, 0.0)
            if time.monotonic() - self._settle_t0 >= SETTLE_S:
                self._begin(self._next_state)
            return

        # Remaining states need odometry
        if self._odom is None:
            return

        # ── Forward leg 1 ──────────────────────────────────────────────────────
        if self._state == self._FWD1:
            d = self._phase_dist()
            if d >= TARGET_DIST:
                self._r['fwd1'] = d
                print(f"\n  ✓ Leg 1     {d:.4f} m  (error {d - TARGET_DIST:+.4f} m)")
                self._settle_to(self._TURN)
            else:
                self._cmd(LIN_SPEED, 0.0)

        # ── 180° rotation ──────────────────────────────────────────────────────
        elif self._state == self._TURN:
            a = abs(self._yaw_accum)
            if a >= TARGET_ANGLE:
                self._r['turn'] = self._yaw_accum
                deg = math.degrees(self._yaw_accum)
                print(f"  ✓ Turn      {deg:.2f}°  (error {deg - 180.0:+.2f}°)")
                self._settle_to(self._FWD2)
            else:
                self._cmd(0.0, ANG_SPEED)  # CCW (positive z = left turn)

        # ── Forward leg 2 ──────────────────────────────────────────────────────
        elif self._state == self._FWD2:
            d = self._phase_dist()
            if d >= TARGET_DIST:
                self._r['fwd2'] = d
                print(f"  ✓ Leg 2     {d:.4f} m  (error {d - TARGET_DIST:+.4f} m)")
                self._cmd(0.0, 0.0)
                self._finish()
            else:
                self._cmd(LIN_SPEED, 0.0)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _cmd(self, lx: float, az: float):
        t = Twist()
        t.linear.x  = lx
        t.angular.z = az
        self._pub.publish(t)

    def _phase_dist(self) -> float:
        """Euclidean distance from the pose snapshot taken at phase start."""
        if not (self._phase_start and self._odom):
            return 0.0
        p0 = self._phase_start.position
        p1 = self._odom.pose.pose.position
        return math.hypot(p1.x - p0.x, p1.y - p0.y)

    def _begin(self, state: str):
        """Transition into a new drive state, snapshot the current odom pose."""
        self._state = state
        if self._odom:
            self._phase_start = self._odom.pose.pose
        if state == self._TURN:
            self._yaw_accum  = 0.0
            self._imu_prev_t = None
        labels = {
            self._FWD1: '● Phase 1 — driving 1.0 m forward …',
            self._TURN: '● Phase 2 — rotating 180° (IMU) …',
            self._FWD2: '● Phase 3 — driving 1.0 m back …',
        }
        if state in labels:
            print(f"\n{labels[state]}")

    def _settle_to(self, next_state: str):
        """Enter a brief zero-velocity settling pause before next_state."""
        self._state      = self._SETTLE
        self._next_state = next_state
        self._settle_t0  = time.monotonic()

    def _end_calib(self):
        n = len(self._gyro_samples)
        self._bias  = statistics.mean(self._gyro_samples)
        self._sigma = statistics.stdev(self._gyro_samples)
        self._r.update(bias=self._bias, sigma=self._sigma, n_calib=n)
        print(f"\n  ✓ Gyro bias  : {self._bias:+.6f} rad/s")
        print(f"    Gyro σ     : {self._sigma:.6f} rad/s  ({n} samples)")
        if self._sigma > 0.02:
            print("  ⚠  High gyro noise — was the robot moving during calibration?")
        self._begin(self._FWD1)

    def _finish(self):
        """Compute final drift and print report."""
        self._state = self._DONE

        if self._global_start and self._odom:
            p0 = self._global_start.position
            p1 = self._odom.pose.pose.position
            self._r['dx'] = p1.x - p0.x
            self._r['dy'] = p1.y - p0.y

            def yaw_from_quat(q):
                return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            y0 = yaw_from_quat(self._global_start.orientation)
            y1 = yaw_from_quat(self._odom.pose.pose.orientation)
            dh = y1 - y0
            self._r['dheading'] = math.degrees(math.atan2(math.sin(dh), math.cos(dh)))

        self.print_report(interrupted=False)

    def stop(self):
        """Emergency stop — publish zero twist."""
        self._cmd(0.0, 0.0)

    # ── Report ─────────────────────────────────────────────────────────────────

    def print_report(self, interrupted: bool = False):
        r   = self._r
        W   = 58
        tag = '  [INTERRUPTED]' if interrupted else ''

        def row(label, value):
            print(f"    {label:<24}: {value}")

        print(f"\n{'═'*W}")
        print(f"  DRIFT TEST REPORT{tag}")
        print(f"  {datetime.now():%Y-%m-%d  %H:%M:%S}")
        print(f"{'═'*W}")

        # ── IMU calibration ────────────────────────────────────────────────────
        print(f"\n  ── IMU CALIBRATION ──────────────────────────────────")
        if r['bias'] is not None:
            row("Gyro Z bias",    f"{r['bias']:+.6f} rad/s")
            row("Gyro noise σ",   f"{r['sigma']:.6f} rad/s  ({r['n_calib']} samples)")
        else:
            row("Status", "not completed")

        # ── Phase 1 ────────────────────────────────────────────────────────────
        print(f"\n  ── PHASE 1  Forward {TARGET_DIST:.1f} m ──────────────────────────")
        if r['fwd1'] is not None:
            e = r['fwd1'] - TARGET_DIST
            row("Commanded",      f"{TARGET_DIST:.3f} m")
            row("Actual (odom)",  f"{r['fwd1']:.4f} m")
            row("Error",          f"{e:+.4f} m  ({e / TARGET_DIST * 100:+.2f}%)")
        else:
            row("Status", "not completed")

        # ── Phase 2 ────────────────────────────────────────────────────────────
        print(f"\n  ── PHASE 2  Rotate 180° ──────────────────────────────")
        if r['turn'] is not None:
            deg = math.degrees(r['turn'])
            e   = deg - 180.0
            row("Commanded",      "180.00°")
            row("Actual (IMU)",   f"{deg:.2f}°")
            row("Error",          f"{e:+.2f}°")
        else:
            row("Status", "not completed")

        # ── Phase 3 ────────────────────────────────────────────────────────────
        print(f"\n  ── PHASE 3  Return {TARGET_DIST:.1f} m ──────────────────────────")
        if r['fwd2'] is not None:
            e = r['fwd2'] - TARGET_DIST
            row("Commanded",      f"{TARGET_DIST:.3f} m")
            row("Actual (odom)",  f"{r['fwd2']:.4f} m")
            row("Error",          f"{e:+.4f} m  ({e / TARGET_DIST * 100:+.2f}%)")
        else:
            row("Status", "not completed")

        # ── Final position ─────────────────────────────────────────────────────
        print(f"\n  ── FINAL POSITION (odom vs start) ────────────────────")
        if r['dx'] is not None:
            drift = math.hypot(r['dx'], r['dy'])
            pct   = drift / (2 * TARGET_DIST) * 100
            row("ΔX",             f"{r['dx']:+.4f} m")
            row("ΔY",             f"{r['dy']:+.4f} m")
            row("Position drift", f"{drift:.4f} m  ({pct:.2f}% of {2*TARGET_DIST:.1f} m total)")
            row("Heading drift",  f"{r['dheading']:+.2f}°")
        else:
            row("Status", "not completed")

        print(f"\n{'═'*W}\n")


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = DriftTest()

    # Ctrl+C handler: stop robot, print partial report, exit cleanly
    def _on_sigint(sig, frame):
        print('\n\n  ⚠  Ctrl+C — stopping robot …')
        node.stop()
        node._interrupted = True
        if node._state != DriftTest._CALIB:
            node.print_report(interrupted=True)
        else:
            print("  (interrupted during calibration — no movement data)\n")
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
        print(f"\n  ✗ Fatal error: {exc}")
        node.stop()
    finally:
        if node._state not in (DriftTest._DONE,):
            node.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
