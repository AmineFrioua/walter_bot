#!/usr/bin/env python3
"""
drift_test.py — Walter IMU-guided drift characterisation
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Run INSIDE the Docker container (Walter must be running):

  docker exec -it walter_dev bash -c \
    "source /opt/ros/humble/setup.bash && cd /ros2_ws && python3 drift_test.py"

Test sequence
  1. Stationary IMU calibration  (5 s — gyro + forward-accel bias)
  2. Drive forward 1.0 m         (IMU double-integration, proportional slow-down)
  3. Rotate 180°                 (IMU gyro integration, bias-corrected, proportional)
  4. Drive forward 1.0 m         (IMU double-integration)
  5. Print full drift report

Distance measurement — IMU double integration
  Forward acceleration (linear_acceleration.x) is integrated twice:
    accel_corr  = ax - accel_bias_x          # remove gravity tilt + sensor offset
    velocity   += accel_corr * dt            # 1st integration → m/s
    distance   += velocity   * dt            # 2nd integration → m

  Boundary condition: velocity is forced to 0 whenever the robot stops
  (entering a settle phase or _begin()).  This prevents velocity bias from
  accumulating across phases.

  Odometry is still read and logged for cross-checking but is NOT used as
  the stop condition.

IMU axis assumption
  linear_acceleration.x = robot's forward direction.
  To verify: the calibration printout shows mean(ax), mean(ay), mean(az).
  The axis closest to ±9.81 is vertical (gravity); the other two should be ~0.
  If the forward axis is not X, change IMU_FORWARD_AXIS below.

Ctrl+C at any point → robot stops immediately, partial report is printed.

Debugging notes
  CSV written to /tmp/drift_<timestamp>.csv every tick:
    t_s, state,
    odom_x, odom_y, odom_yaw_deg, odom_dist_m,
    ax_raw, ax_corr, imu_vel_fwd, imu_dist_fwd,
    gz_raw, gz_corr, yaw_accum_deg,
    remaining_m, cmd_vx, cmd_az

  "robot never stops"  → imu_dist_fwd stuck at 0: ax or bias is wrong
  "overshoots"         → find row where remaining_m ≤ 0, check cmd_vx
                         should be LIN_MIN, not LIN_SPEED
  "velocity runaway"   → ax_corr large when stationary: re-run calibration
                         on a flat surface without robot moving
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
TARGET_DIST    = 1.0        # metres per leg (TRUE physical metres)
TARGET_ANGLE   = math.pi    # 180° physical

LIN_SPEED      = 0.06       # m/s  cruise speed
LIN_MIN        = 0.03       # m/s  minimum speed in slow-down zone
LIN_RAMP_DIST  = 0.20       # m    start proportional slow-down this far before target

ANG_SPEED      = 0.15       # rad/s  cruise angular speed
ANG_MIN        = 0.05       # rad/s  minimum angular speed in slow-down zone
ANG_RAMP_RAD   = 0.35       # rad    ~20° — start slow-down before target

# ── Sensor scaling (hardware compensation) ──────────────────────────────────────
# The IMU on this robot is mounted vertically — the gyro Z axis used for
# yaw integration reports rotation at 2× the actual physical rate.  To make
# the robot physically turn 180°, the integrated _yaw_accum must reach 2π.
# Compensation:  YAW_SCALE = 2.0  (true_radians = _yaw_accum / YAW_SCALE)
YAW_SCALE      = 2.0

CALIB_S        = 5.0        # seconds of stationary IMU data
SETTLE_S       = 0.8        # pause between phases (s)
CTRL_HZ        = 20         # cmd_vel publish rate (Hz)

# IMU axis that points forward on the robot.
# 0=X, 1=Y, 2=Z.  Check calibration printout: forward axis should show ~0 bias.
IMU_FORWARD_AXIS = 0        # linear_acceleration.x = forward

LOG_DIR        = '/tmp'


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
        self._calib_t0       = None
        self._gyro_samples   = []           # angular_velocity.z
        self._accel_samples  = [[], [], []] # ax, ay, az raw samples
        self._bias_gz        = 0.0          # gyro Z bias
        self._sigma_gz       = 0.0
        self._bias_ax        = 0.0          # forward accel bias (gravity tilt + offset)
        self._sigma_ax       = 0.0

        # ── IMU distance (double integration) ────────────────────────────────────
        # Raw inputs stored for CSV logging
        self._ax_raw         = 0.0
        self._gz_raw         = 0.0
        # Forward motion integrators — reset to 0 on every phase start / stop
        self._imu_vel_fwd    = 0.0          # integrated velocity   (m/s)
        self._imu_dist_fwd   = 0.0          # double-integrated distance (m)
        # Rotation integrator
        self._yaw_accum      = 0.0          # integrated bias-corrected yaw (rad)
        # Shared prev-timestamp for all IMU integration
        self._imu_prev_t     = None

        # ── Odometry (cross-check only, not used for stopping) ────────────────────
        self._odom           = None
        self._odom_ready     = False
        self._odom_msg_count = 0
        # Phase-start snapshot stored as plain floats (not object refs — see note
        # in fix/drift-test-stop-and-logging about buffer reuse aliasing)
        self._phase_start_x  = None
        self._phase_start_y  = None
        self._global_start_x = None
        self._global_start_y = None
        self._global_start_yaw = None

        # ── State machine ─────────────────────────────────────────────────────────
        self._state          = self._CALIB
        self._next_state     = None
        self._settle_t0      = None
        self._phase_t0       = None

        # ── Results ───────────────────────────────────────────────────────────────
        self._r = dict(
            bias_gz=None, sigma_gz=None, n_calib=0,
            bias_ax=None, sigma_ax=None,
            fwd1_imu=None,  fwd1_odom=None,  fwd1_dt=None,
            turn=None,      turn_raw=None,   turn_dt=None,
            fwd2_imu=None,  fwd2_odom=None,  fwd2_dt=None,
            dx=None, dy=None, dheading=None,
        )

        # ── CSV ───────────────────────────────────────────────────────────────────
        ts_str         = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._csv_path = os.path.join(LOG_DIR, f'drift_{ts_str}.csv')
        self._csv_f    = open(self._csv_path, 'w', newline='')
        self._csv_w    = csv.writer(self._csv_f)
        self._csv_w.writerow([
            't_s', 'state',
            'odom_x', 'odom_y', 'odom_yaw_deg', 'odom_dist_m',
            'ax_raw', 'ax_corr', 'imu_vel_fwd', 'imu_dist_fwd',
            'gz_raw', 'gz_corr', 'yaw_accum_deg',
            'remaining_m', 'cmd_vx', 'cmd_az',
        ])
        self._t0       = time.monotonic()
        self._last_cmd = (0.0, 0.0)

        # ── Banner ────────────────────────────────────────────────────────────────
        axis_name = ['X', 'Y', 'Z'][IMU_FORWARD_AXIS]
        print(f"\n{'─'*64}")
        print("  WALTER DRIFT TEST  (IMU distance)")
        print(f"  {datetime.now():%Y-%m-%d  %H:%M:%S}")
        print(f"{'─'*64}")
        print(f"  Target          : {TARGET_DIST} m × 2 legs  |  180° turn (physical)")
        print(f"  Cruise speed    : {LIN_SPEED} m/s  /  {ANG_SPEED} rad/s")
        print(f"  Slow-down zone  : last {LIN_RAMP_DIST} m  /  last {math.degrees(ANG_RAMP_RAD):.0f}°")
        print(f"  Yaw scaling     : ×{YAW_SCALE}  (vertical IMU — raw gyro reports {YAW_SCALE}× actual)")
        print(f"  Distance source : IMU linear_acceleration.{axis_name} (double-integrated)")
        print(f"  Odom            : logged for comparison only")
        print(f"  CSV log         : {self._csv_path}")
        print(f"{'─'*64}")
        print(f"\n● Calibrating IMU — keep robot STILL for {CALIB_S:.0f} s …\n")

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
            print(f"\n  [odom] first msg  x={p.x:.4f}  y={p.y:.4f}  "
                  f"yaw={math.degrees(self._global_start_yaw):.2f}°")

    def _imu_cb(self, msg: Imu):
        now = time.monotonic()
        ax_vals = [msg.linear_acceleration.x,
                   msg.linear_acceleration.y,
                   msg.linear_acceleration.z]
        gz       = msg.angular_velocity.z
        ax       = ax_vals[IMU_FORWARD_AXIS]

        self._ax_raw = ax
        self._gz_raw = gz

        # ── Calibration — collect raw samples ──────────────────────────────────
        if self._state == self._CALIB:
            if self._calib_t0 is None:
                self._calib_t0 = now
            self._gyro_samples.append(gz)
            for i, v in enumerate(ax_vals):
                self._accel_samples[i].append(v)
            n       = len(self._gyro_samples)
            elapsed = now - self._calib_t0
            if n % 50 == 0:
                print(f"  … {elapsed:.1f}/{CALIB_S:.0f} s  ({n} samples)  "
                      f"gz={gz:+.4f}  ax={ax:+.5f}", end='\r', flush=True)
            self._imu_prev_t = now
            return

        # ── Compute dt (guard against large gaps, e.g. across settle pauses) ──
        dt = 0.0
        if self._imu_prev_t is not None:
            dt = now - self._imu_prev_t
            if dt > 0.5:    # ignore first sample after a long gap
                dt = 0.0
        self._imu_prev_t = now

        # ── Rotation integration (TURN state) ──────────────────────────────────
        if self._state == self._TURN and dt > 0.0:
            self._yaw_accum += (gz - self._bias_gz) * dt

        # ── Forward distance integration (FWD1 / FWD2 states) ─────────────────
        elif self._state in (self._FWD1, self._FWD2) and dt > 0.0:
            ax_corr           = ax - self._bias_ax
            self._imu_vel_fwd += ax_corr * dt
            self._imu_dist_fwd += self._imu_vel_fwd * dt

    # ── Control tick ───────────────────────────────────────────────────────────

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

        # ── Common live values ─────────────────────────────────────────────────
        imu_d     = self._imu_dist_fwd
        odom_d    = self._phase_dist()         # for display / CSV only
        yaw_deg   = math.degrees(self._yaw_accum)
        odom_yaw  = (math.degrees(self._yaw_from_quat(self._odom.pose.pose.orientation))
                     if self._odom else 0.0)

        # ── Forward leg 1 ──────────────────────────────────────────────────────
        if self._state == self._FWD1:
            remaining = TARGET_DIST - imu_d
            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                self._imu_vel_fwd = 0.0        # boundary: robot stopped
                self._r['fwd1_imu']  = imu_d
                self._r['fwd1_odom'] = odom_d
                self._r['fwd1_dt']   = time.monotonic() - self._phase_t0
                self._log_phase_end('Leg 1', imu_d, odom_d, self._r['fwd1_dt'])
                self._write_csv(t_s, odom_d, remaining, 0.0, 0.0)
                self._settle_to(self._TURN)
                return
            speed = self._ramp(remaining, LIN_RAMP_DIST, LIN_MIN, LIN_SPEED)
            print(f"  → fwd1  IMU={imu_d:.3f}/{TARGET_DIST:.2f} m  "
                  f"odom={odom_d:.3f} m  "
                  f"remain={remaining:.3f} m  "
                  f"v={speed:.3f} m/s    ", end='\r', flush=True)
            self._cmd(speed, 0.0)
            self._write_csv(t_s, odom_d, remaining, speed, 0.0)

        # ── 180° rotation ──────────────────────────────────────────────────────
        elif self._state == self._TURN:
            # IMU is mounted vertically: _yaw_accum reports YAW_SCALE × actual.
            # Convert to true radians before comparing against TARGET_ANGLE.
            accum_raw      = abs(self._yaw_accum)              # IMU/odom space
            turned_actual  = accum_raw / YAW_SCALE             # true radians
            remaining      = TARGET_ANGLE - turned_actual      # true radians
            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                self._r['turn']    = self._yaw_accum / YAW_SCALE   # store TRUE rad
                self._r['turn_raw'] = self._yaw_accum              # for debugging
                self._r['turn_dt']  = time.monotonic() - self._phase_t0
                p = self._odom.pose.pose.position if self._odom else None
                print(f"\n  ✓ Turn done   actual={math.degrees(turned_actual):.2f}°  "
                      f"raw={math.degrees(accum_raw):.2f}°  "
                      f"Δ={math.degrees(turned_actual) - 180.0:+.2f}°  "
                      f"t={self._r['turn_dt']:.1f} s")
                if p:
                    print(f"    end pose  x={p.x:.4f}  y={p.y:.4f}  "
                          f"yaw(odom)={odom_yaw:.2f}°  "
                          f"yaw(IMU actual)={math.degrees(turned_actual):.2f}°")
                self._write_csv(t_s, odom_d, remaining, 0.0, 0.0)
                self._settle_to(self._FWD2)
                return
            # Log every 45° milestone (of actual rotation)
            milestone = int(math.degrees(turned_actual) / 45)
            if not hasattr(self, '_turn_milestone') or milestone > self._turn_milestone:
                self._turn_milestone = milestone
                gz_corr = self._gz_raw - self._bias_gz
                print(f"\n  [IMU] actual={math.degrees(turned_actual):.1f}°  "
                      f"(raw={math.degrees(accum_raw):.1f}°)  "
                      f"gz={self._gz_raw:+.5f}  corr={gz_corr:+.5f}")
            ang = self._ramp(remaining, ANG_RAMP_RAD, ANG_MIN, ANG_SPEED)
            print(f"  → turn  {math.degrees(turned_actual):.1f}°/{math.degrees(TARGET_ANGLE):.0f}°  "
                  f"remain={math.degrees(remaining):.1f}°  "
                  f"w={ang:.3f} r/s    ", end='\r', flush=True)
            self._cmd(0.0, ang)
            self._write_csv(t_s, odom_d, remaining, 0.0, ang)

        # ── Forward leg 2 ──────────────────────────────────────────────────────
        elif self._state == self._FWD2:
            remaining = TARGET_DIST - imu_d
            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                self._imu_vel_fwd = 0.0
                self._r['fwd2_imu']  = imu_d
                self._r['fwd2_odom'] = odom_d
                self._r['fwd2_dt']   = time.monotonic() - self._phase_t0
                self._log_phase_end('Leg 2', imu_d, odom_d, self._r['fwd2_dt'])
                self._write_csv(t_s, odom_d, remaining, 0.0, 0.0)
                self._finish()
                return
            speed = self._ramp(remaining, LIN_RAMP_DIST, LIN_MIN, LIN_SPEED)
            print(f"  → fwd2  IMU={imu_d:.3f}/{TARGET_DIST:.2f} m  "
                  f"odom={odom_d:.3f} m  "
                  f"remain={remaining:.3f} m  "
                  f"v={speed:.3f} m/s    ", end='\r', flush=True)
            self._cmd(speed, 0.0)
            self._write_csv(t_s, odom_d, remaining, speed, 0.0)

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _ramp(remaining: float, zone: float, v_min: float, v_max: float) -> float:
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
        """Odom Euclidean distance from phase-start snapshot (cross-check only)."""
        if self._phase_start_x is None or self._odom is None:
            return 0.0
        p = self._odom.pose.pose.position
        return math.hypot(p.x - self._phase_start_x,
                          p.y - self._phase_start_y)

    def _begin(self, state: str):
        self._state    = state
        self._phase_t0 = time.monotonic()
        if hasattr(self, '_turn_milestone'):
            del self._turn_milestone

        # Reset IMU distance integrators for the new phase
        self._imu_dist_fwd = 0.0
        self._imu_vel_fwd  = 0.0        # start from rest

        # Snapshot odom phase-start as scalars (for cross-check)
        if self._odom_ready and self._odom is not None:
            p = self._odom.pose.pose.position
            self._phase_start_x = p.x
            self._phase_start_y = p.y
            odom_yaw = math.degrees(self._yaw_from_quat(self._odom.pose.pose.orientation))
            print(f"\n  [DBG] phase-start  "
                  f"x={p.x:.4f}  y={p.y:.4f}  yaw={odom_yaw:.2f}°  "
                  f"(imu_vel reset to 0)")
        else:
            self._phase_start_x = None
            self._phase_start_y = None
            print(f"\n  [WARN] odom not ready at _begin({state})")

        if state == self._TURN:
            self._yaw_accum = 0.0

        labels = {
            self._FWD1: f'● Phase 1 — driving {TARGET_DIST} m forward (IMU) …',
            self._TURN: '● Phase 2 — rotating 180° (IMU gyro) …',
            self._FWD2: f'● Phase 3 — returning {TARGET_DIST} m (IMU) …',
        }
        if state in labels:
            print(f"\n{labels[state]}")

    def _settle_to(self, next_state: str):
        # Force velocity to 0: robot has stopped, prevent bias accumulation
        self._imu_vel_fwd = 0.0
        self._state       = self._SETTLE
        self._next_state  = next_state
        self._settle_t0   = time.monotonic()

    def _end_calib(self):
        n = len(self._gyro_samples)
        if n < 2:
            print("\n  ✗ Not enough IMU samples — is /imu/data_raw publishing?")
            return

        # Gyro bias
        self._bias_gz  = statistics.mean(self._gyro_samples)
        self._sigma_gz = statistics.stdev(self._gyro_samples)

        # Accel bias for all three axes (helps user verify which is forward)
        means  = [statistics.mean(s) for s in self._accel_samples]
        sigmas = [statistics.stdev(s) for s in self._accel_samples]
        self._bias_ax  = means[IMU_FORWARD_AXIS]
        self._sigma_ax = sigmas[IMU_FORWARD_AXIS]

        self._r.update(
            bias_gz=self._bias_gz, sigma_gz=self._sigma_gz,
            bias_ax=self._bias_ax, sigma_ax=self._sigma_ax,
            n_calib=n,
        )

        # Odom health check
        if not self._odom_ready:
            print("\n  ⚠  /odom not yet received — will not have odom cross-check")
        else:
            p        = self._odom.pose.pose.position
            freq_est = self._odom_msg_count / CALIB_S
            print(f"\n  [odom] x={p.x:.4f}  y={p.y:.4f}  ~{freq_est:.1f} Hz")
            if freq_est < 5:
                print(f"  ⚠  Odom slow ({freq_est:.1f} Hz) — cross-check will be coarse")

        print(f"\n  ✓ Gyro bias (z)      : {self._bias_gz:+.6f} rad/s  "
              f"(σ={self._sigma_gz:.5f})")
        print(f"  ✓ Accel bias (ax)    : {self._bias_ax:+.6f} m/s²  "
              f"(σ={self._sigma_ax:.5f})")
        if self._sigma_ax > 0.05:
            print("  ⚠  High accel noise — robot may have been moving during calibration")

        # Print all three axis means so user can verify IMU_FORWARD_AXIS
        axis_names = ['X', 'Y', 'Z']
        print(f"\n  Accel means (verify forward axis ~0, vertical axis ~±9.81):")
        for i, (name, m, s) in enumerate(zip(axis_names, means, sigmas)):
            marker = ' ← forward (IMU_FORWARD_AXIS)' if i == IMU_FORWARD_AXIS else ''
            print(f"    {name}: mean={m:+8.4f} m/s²  σ={s:.4f}{marker}")

        self._begin(self._FWD1)

    def _log_phase_end(self, label: str, imu_d: float, odom_d: float, dt: float):
        diff = imu_d - odom_d if odom_d > 0 else float('nan')
        print(f"\n  ✓ {label} done  IMU={imu_d:.4f} m  odom={odom_d:.4f} m  "
              f"diff={diff:+.4f} m  t={dt:.1f} s")
        if self._odom and self._odom_ready:
            p = self._odom.pose.pose.position
            odom_yaw = math.degrees(self._yaw_from_quat(self._odom.pose.pose.orientation))
            print(f"    end pose  x={p.x:.4f}  y={p.y:.4f}  yaw={odom_yaw:.2f}°")

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
        print(f"\n  [log] CSV → {self._csv_path}")
        self.print_report(interrupted=False)

    @staticmethod
    def _yaw_from_quat(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _write_csv(self, t_s: float, odom_d: float, remaining: float,
                   cmd_vx: float, cmd_az: float):
        odom_x = odom_y = odom_yaw = 0.0
        if self._odom:
            p       = self._odom.pose.pose.position
            odom_x  = p.x
            odom_y  = p.y
            odom_yaw = math.degrees(self._yaw_from_quat(self._odom.pose.pose.orientation))
        self._csv_w.writerow([
            f'{t_s:.3f}',
            self._state,
            f'{odom_x:.5f}', f'{odom_y:.5f}', f'{odom_yaw:.3f}', f'{odom_d:.5f}',
            f'{self._ax_raw:.6f}',
            f'{self._ax_raw - self._bias_ax:.6f}',
            f'{self._imu_vel_fwd:.5f}',
            f'{self._imu_dist_fwd:.5f}',
            f'{self._gz_raw:.6f}',
            f'{self._gz_raw - self._bias_gz:.6f}',
            f'{math.degrees(self._yaw_accum):.3f}',
            f'{remaining:.5f}',
            f'{cmd_vx:.4f}', f'{cmd_az:.4f}',
        ])

    def stop(self):
        self._cmd(0.0, 0.0)

    # ── Report ─────────────────────────────────────────────────────────────────

    def print_report(self, interrupted: bool = False):
        r   = self._r
        W   = 64
        tag = '  [INTERRUPTED]' if interrupted else ''

        def row(label, value):
            print(f"    {label:<30}: {value}")

        print(f"\n{'═'*W}")
        print(f"  DRIFT TEST REPORT{tag}")
        print(f"  {datetime.now():%Y-%m-%d  %H:%M:%S}")
        print(f"{'═'*W}")

        print(f"\n  ── IMU CALIBRATION ────────────────────────────────────────")
        if r['bias_gz'] is not None:
            row("Gyro Z bias",         f"{r['bias_gz']:+.6f} rad/s  (σ={r['sigma_gz']:.5f})")
            row("Fwd accel bias (ax)",  f"{r['bias_ax']:+.6f} m/s²   (σ={r['sigma_ax']:.5f})")
            row("Samples",             str(r['n_calib']))
        else:
            row("Status", "not completed")

        def phase_rows(key_imu, key_odom, key_dt):
            if r[key_imu] is not None:
                e_imu  = r[key_imu] - TARGET_DIST
                e_odom = (r[key_odom] - TARGET_DIST) if r[key_odom] else float('nan')
                row("Commanded",           f"{TARGET_DIST:.3f} m")
                row("Actual  — IMU",       f"{r[key_imu]:.4f} m  (error {e_imu:+.4f} m  {e_imu/TARGET_DIST*100:+.2f}%)")
                row("Actual  — odom",      f"{r[key_odom]:.4f} m  (error {e_odom:+.4f} m)" if r[key_odom] else "—")
                row("IMU vs odom diff",    f"{r[key_imu] - r[key_odom]:+.4f} m" if r[key_odom] else "—")
                row("Time",               f"{r[key_dt]:.2f} s")
            else:
                row("Status", "not completed")

        print(f"\n  ── PHASE 1  Forward {TARGET_DIST:.1f} m ────────────────────────────")
        phase_rows('fwd1_imu', 'fwd1_odom', 'fwd1_dt')

        print(f"\n  ── PHASE 2  Rotate 180° ──────────────────────────────────────")
        if r['turn'] is not None:
            deg     = math.degrees(r['turn'])          # already true / YAW_SCALE
            deg_raw = math.degrees(r.get('turn_raw', r['turn'] * YAW_SCALE))
            row("Commanded (physical)",  "180.00°")
            row("Actual   (physical)",   f"{deg:.2f}°  (error {deg - 180.0:+.2f}°)")
            row("Actual   (IMU raw)",    f"{deg_raw:.2f}°  [÷{YAW_SCALE} = {deg:.2f}°]")
            row("Time",                f"{r['turn_dt']:.2f} s")
        else:
            row("Status", "not completed")

        print(f"\n  ── PHASE 3  Return {TARGET_DIST:.1f} m ─────────────────────────────")
        phase_rows('fwd2_imu', 'fwd2_odom', 'fwd2_dt')

        print(f"\n  ── FINAL POSITION (odom vs start) ────────────────────────────")
        if r['dx'] is not None:
            drift = math.hypot(r['dx'], r['dy'])
            pct   = drift / (2 * TARGET_DIST) * 100
            row("ΔX (odom)",           f"{r['dx']:+.4f} m")
            row("ΔY (odom)",           f"{r['dy']:+.4f} m")
            row("Position drift",      f"{drift:.4f} m  ({pct:.2f}% of {2*TARGET_DIST:.1f} m)")
            row("Heading drift",       f"{r['dheading']:+.2f}°")
        else:
            row("Status", "not completed")

        print(f"\n  ── DEBUG ──────────────────────────────────────────────────────")
        row("CSV log", self._csv_path)
        print(f"    imu_dist_fwd=0 throughout → ax_corr wrong, check IMU_FORWARD_AXIS")
        print(f"    imu_vel_fwd growing at rest → re-run calibration on flat surface")
        print(f"    Large IMU vs odom diff → wheel slip or encoder miscalibration")

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
