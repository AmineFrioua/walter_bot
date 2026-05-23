#!/usr/bin/env python3
"""
forward_return.py — drive forward, press ENTER to return, get drift report
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Run INSIDE the Docker container:
  docker exec -it walter_dev bash -c \
    "source /opt/ros/humble/setup.bash && cd /ros2_ws && python3 forward_return.py [l|m|h]"

Speed levels
  Linear  :  l = 0.02 m/s  |  m = 0.04 m/s  |  h = 0.06 m/s
  Angular :  l = 0.01 r/s  |  m = 0.03 r/s  |  h = 0.06 r/s

Flow
  1. Robot drives forward at chosen speed.
  2. Press ENTER  →  robot decelerates, stops, turns 180°, returns same distance.
  3. Robot stops on its own when it has covered the outbound distance.
     Press ENTER again at any point during the return to stop early.
  4. Drift report printed automatically.

Ctrl+C anywhere  →  emergency stop + partial report.

Hardware compensation (see SCALING section below)
  • Robot's IMU is mounted vertically — odom yaw over-reports rotation by 2×.
    To physically rotate 180° we must turn until odom-yaw has changed by 360°.
    Compensation:  YAW_SCALE  = 2.0
  • Wheel-encoder odom under-reports linear distance by ~10×.
    All distances reported in the script are scaled up so the user sees TRUE
    metres travelled, not the smaller odom-internal value.
    Compensation:  DIST_SCALE = 10.0
"""

import math
import select
import signal
import sys
import termios
import tty
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ── Speed levels ──────────────────────────────────────────────────────────────
LIN_SPEEDS = {'l': 0.02, 'm': 0.04, 'h': 0.06}   # m/s  (commanded — real units)
ANG_SPEEDS = {'l': 0.01, 'm': 0.03, 'h': 0.06}   # rad/s (commanded — real units)

# Deceleration zones (in TRUE physical units)
LIN_RAMP_DIST  = 0.12    # m    — ramp speed over this distance before stopping
LIN_MIN        = 0.01    # m/s  — minimum speed during ramp (prevents stall)
ANG_RAMP_RAD   = 0.25    # rad  — ramp angular speed this many rad before 180°
ANG_MIN        = 0.005   # rad/s

SETTLE_S  = 0.6          # pause at each stop before next phase
CTRL_HZ   = 20           # cmd_vel rate


# ── SCALING ───────────────────────────────────────────────────────────────────
# These constants compensate for hardware quirks on THIS robot.  Re-measure
# them if you change the IMU mount or recalibrate the wheel encoders.
#
# YAW_SCALE  — how many "odom radians" the robot reports per actual radian of
#              physical rotation.  Vertical IMU mount makes this 2.0 (robot
#              physically rotates π but odom reports 2π).
#
# DIST_SCALE — how many TRUE metres the robot actually travels per metre of
#              odom distance.  Encoder under-reporting makes this 10.0
#              (robot physically moves 1 m but odom says 0.1 m).
#
# In the rest of this file:
#   _dist_from()  returns TRUE metres  (odom × DIST_SCALE)
#   _turn_accum   is in odom-yaw space — we compare to (π × YAW_SCALE) for
#                 the stop condition, and divide by YAW_SCALE for display.
YAW_SCALE  = 2.0
DIST_SCALE = 10.0


# ── Helpers ───────────────────────────────────────────────────────────────────

def _ramp(remaining: float, zone: float, v_min: float, v_max: float) -> float:
    if remaining >= zone:
        return v_max
    return max(v_min, v_max * (remaining / zone))

def _yaw(q) -> float:
    """Yaw from quaternion — returned in ODOM-yaw radians (NOT scaled)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def _angle_diff(a: float, b: float) -> float:
    """Signed shortest-path difference a − b, result in [−π, π]."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


# ── Node ──────────────────────────────────────────────────────────────────────

class ForwardReturn(Node):

    _FWD      = 'forward'
    _SLOWING  = 'slowing'    # decelerating after ENTER pressed
    _SETTLE   = 'settling'
    _TURN     = 'turning'
    _RETURN   = 'returning'
    _DONE     = 'done'

    def __init__(self, speed_level: str):
        super().__init__('forward_return')

        self._lin  = LIN_SPEEDS[speed_level]
        self._ang  = ANG_SPEEDS[speed_level]

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self._timer = self.create_timer(1.0 / CTRL_HZ, self._tick)

        # Odom state — snapshots stored as plain floats (never object refs)
        self._odom        = None
        self._odom_ready  = False

        # Phase-start snapshots (raw odom x/y — NOT scaled)
        self._fwd_x  = self._fwd_y  = None   # forward leg start
        self._slow_x = self._slow_y = None   # position when ENTER pressed
        self._ret_x  = self._ret_y  = None   # return leg start

        # Yaw integration during TURN — accumulator avoids the ±π wrap that
        # bit the previous version (180° in odom-space is reachable when the
        # robot has only physically turned 90° — see YAW_SCALE).
        self._turn_yaw_prev = None      # previous odom yaw
        self._turn_accum    = 0.0       # accumulated odom-yaw rotation (rad)

        # State machine
        self._state      = self._FWD
        self._settle_t0  = None
        self._next_state = None

        # Results (all in TRUE physical units)
        self._dist_out   = 0.0    # outbound distance (m, true)
        self._dist_ret   = 0.0    # return distance covered (m, true)
        self._r = dict(
            dist_out=None, dist_ret=None,
            turn_deg=None, turn_odom_deg=None,
            dx=None, dy=None, dheading=None,
        )

        # Keyboard
        self._old_term = None
        try:
            self._old_term = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            pass

        # Banner
        eta_turn = math.pi / self._ang   # seconds to physically turn 180°
        print(f"\n{'─'*64}")
        print("  WALTER FORWARD-RETURN")
        print(f"  {datetime.now():%Y-%m-%d  %H:%M:%S}")
        print(f"{'─'*64}")
        print(f"  Speed level    : {speed_level.upper()}")
        print(f"  Linear speed   : {self._lin} m/s")
        print(f"  Angular speed  : {self._ang} rad/s  "
              f"(180° physical ≈ {eta_turn:.0f} s)")
        print(f"  Scaling        : YAW × {YAW_SCALE}   DIST × {DIST_SCALE}")
        print(f"                   (vertical IMU + encoder under-report)")
        print(f"{'─'*64}")
        print(f"\n● Waiting for odometry …")

    # ── Odom callback ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._odom = msg
        if not self._odom_ready:
            p = msg.pose.pose.position
            self._r['_gx']   = p.x
            self._r['_gy']   = p.y
            self._r['_gyaw'] = _yaw(msg.pose.pose.orientation)
            self._odom_ready = True

    # ── Tick ───────────────────────────────────────────────────────────────────

    def _tick(self):
        if self._state == self._DONE:
            return

        if not self._odom_ready:
            return

        # Lazy capture of forward-start
        if self._fwd_x is None and self._state == self._FWD:
            p = self._odom.pose.pose.position
            self._fwd_x = p.x
            self._fwd_y = p.y
            yaw_deg = math.degrees(_yaw(self._odom.pose.pose.orientation))
            print(f"\n  [ready]  x={p.x:.4f}  y={p.y:.4f}  yaw={yaw_deg:.1f}°")
            print(f"\n● Driving forward at {self._lin} m/s  —  press ENTER to return\n")

        key = self._read_key()
        p   = self._odom.pose.pose.position
        yaw = _yaw(self._odom.pose.pose.orientation)

        # ── FORWARD ────────────────────────────────────────────────────────────
        if self._state == self._FWD:
            d = self._dist_from(self._fwd_x, self._fwd_y)   # TRUE metres
            print(f"  → {d:.3f} m            ", end='\r', flush=True)
            if key:
                self._slow_x = p.x
                self._slow_y = p.y
                self._dist_out = d
                self._state = self._SLOWING
                print(f"\n  [ENTER] decelerating at {d:.3f} m …")
            else:
                self._cmd(self._lin, 0.0)

        # ── SLOWING (decelerate to stop) ───────────────────────────────────────
        elif self._state == self._SLOWING:
            d_since_trigger = self._dist_from(self._slow_x, self._slow_y)
            remaining = LIN_RAMP_DIST - d_since_trigger
            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                self._dist_out = self._dist_from(self._fwd_x, self._fwd_y)
                self._r['dist_out'] = self._dist_out
                print(f"  ✓ Stopped  outbound = {self._dist_out:.4f} m")
                self._enter_settle(self._TURN)
            else:
                speed = _ramp(remaining, LIN_RAMP_DIST, LIN_MIN, self._lin)
                self._cmd(speed, 0.0)
                print(f"  → slowing {speed:.3f} m/s  "
                      f"remain={remaining:.3f} m         ", end='\r', flush=True)

        # ── SETTLE ─────────────────────────────────────────────────────────────
        elif self._state == self._SETTLE:
            self._cmd(0.0, 0.0)
            if time.monotonic() - self._settle_t0 >= SETTLE_S:
                self._begin(self._next_state)

        # ── TURN ───────────────────────────────────────────────────────────────
        elif self._state == self._TURN:
            # First tick of TURN: initialise accumulator
            if self._turn_yaw_prev is None:
                self._turn_yaw_prev = yaw
                self._turn_accum    = 0.0
                eta = math.pi / self._ang   # seconds to physically turn 180°
                print(f"\n  [turn]  start yaw={math.degrees(yaw):.1f}°  "
                      f"target 180° physical  "
                      f"(= {math.degrees(math.pi * YAW_SCALE):.0f}° in odom yaw)  "
                      f"ETA ≈ {eta:.0f} s")

            # Accumulate small per-tick deltas (always within [−π, π] so no
            # wrap problem even when total odom rotation exceeds π)
            dy = _angle_diff(yaw, self._turn_yaw_prev)
            self._turn_accum += dy
            self._turn_yaw_prev = yaw

            turned_odom   = abs(self._turn_accum)
            turned_actual = turned_odom / YAW_SCALE      # true radians
            remaining     = math.pi - turned_actual

            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                self._r['turn_deg']      = math.degrees(turned_actual)
                self._r['turn_odom_deg'] = math.degrees(turned_odom)
                print(f"\n  ✓ Turn done  actual={math.degrees(turned_actual):.1f}°  "
                      f"odom={math.degrees(turned_odom):.1f}°  "
                      f"(Δ {math.degrees(turned_actual) - 180.0:+.1f}°)")
                self._enter_settle(self._RETURN)
            else:
                ang = _ramp(remaining, ANG_RAMP_RAD, ANG_MIN, self._ang)
                self._cmd(0.0, ang)
                print(f"  → turn  {math.degrees(turned_actual):.1f}°/180°  "
                      f"(odom {math.degrees(turned_odom):.1f}°/"
                      f"{math.degrees(math.pi * YAW_SCALE):.0f}°)  "
                      f"w={ang:.4f} r/s    ", end='\r', flush=True)

        # ── RETURN ─────────────────────────────────────────────────────────────
        elif self._state == self._RETURN:
            d         = self._dist_from(self._ret_x, self._ret_y)   # TRUE metres
            remaining = self._dist_out - d
            self._dist_ret = d

            if key:
                self._cmd(0.0, 0.0)
                self._r['dist_ret'] = d
                print(f"\n  [ENTER] stopped early at {d:.4f} m of "
                      f"{self._dist_out:.4f} m")
                self._finish(yaw)
                return

            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                self._r['dist_ret'] = d
                print(f"\n  ✓ Return done  {d:.4f} m")
                self._finish(yaw)
            else:
                speed = _ramp(remaining, LIN_RAMP_DIST, LIN_MIN, self._lin)
                self._cmd(speed, 0.0)
                print(f"  ← {d:.3f}/{self._dist_out:.3f} m  "
                      f"remain={remaining:.3f} m  "
                      f"v={speed:.3f} m/s    ", end='\r', flush=True)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _dist_from(self, sx, sy) -> float:
        """Distance in TRUE metres (odom × DIST_SCALE)."""
        if sx is None or self._odom is None:
            return 0.0
        p = self._odom.pose.pose.position
        return math.hypot(p.x - sx, p.y - sy) * DIST_SCALE

    def _cmd(self, lx: float, az: float):
        t = Twist()
        t.linear.x  = lx
        t.angular.z = az
        self._pub.publish(t)

    def _enter_settle(self, next_state: str):
        self._state      = self._SETTLE
        self._next_state = next_state
        self._settle_t0  = time.monotonic()

    def _begin(self, state: str):
        self._state = state
        if state == self._TURN:
            # Reset accumulator state — captured on first TURN tick
            self._turn_yaw_prev = None
            self._turn_accum    = 0.0
            print(f"\n● Turning 180° at {self._ang} rad/s …")
        elif state == self._RETURN:
            p = self._odom.pose.pose.position
            self._ret_x = p.x
            self._ret_y = p.y
            yaw_deg = math.degrees(_yaw(self._odom.pose.pose.orientation))
            print(f"\n  [return start]  x={p.x:.4f}  y={p.y:.4f}  "
                  f"yaw={yaw_deg:.1f}°")
            print(f"\n● Returning {self._dist_out:.3f} m  —  "
                  f"press ENTER to stop early\n")

    def _finish(self, current_yaw: float):
        self._state = self._DONE
        gx   = self._r.get('_gx')
        gy   = self._r.get('_gy')
        gyaw = self._r.get('_gyaw')
        if gx is not None and self._odom is not None:
            p = self._odom.pose.pose.position
            # Position drift in TRUE metres
            self._r['dx'] = (p.x - gx) * DIST_SCALE
            self._r['dy'] = (p.y - gy) * DIST_SCALE
            # Heading drift in actual degrees (assumes small drift well within ±π/YAW_SCALE)
            dh_odom = _angle_diff(current_yaw, gyaw)
            self._r['dheading'] = math.degrees(dh_odom / YAW_SCALE)
        self.print_report()

    def _read_key(self) -> bool:
        try:
            if select.select([sys.stdin], [], [], 0.0)[0]:
                ch = sys.stdin.read(1)
                return ch in ('\r', '\n', ' ')
        except Exception:
            pass
        return False

    def stop(self):
        self._cmd(0.0, 0.0)

    def restore_terminal(self):
        if self._old_term is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(),
                                  termios.TCSADRAIN, self._old_term)
            except Exception:
                pass

    # ── Report ─────────────────────────────────────────────────────────────────

    def print_report(self, interrupted: bool = False):
        r   = self._r
        W   = 64
        tag = '  [INTERRUPTED]' if interrupted else ''

        def row(label, value):
            print(f"    {label:<30}: {value}")

        print(f"\n{'═'*W}")
        print(f"  FORWARD-RETURN REPORT{tag}")
        print(f"  {datetime.now():%Y-%m-%d  %H:%M:%S}")
        print(f"{'═'*W}")

        print(f"\n  ── SPEED SETTINGS ──────────────────────────────────────────")
        row("Linear speed",            f"{self._lin} m/s")
        row("Angular speed",           f"{self._ang} rad/s")
        row("Scaling",                 f"YAW × {YAW_SCALE}   DIST × {DIST_SCALE}")

        print(f"\n  ── OUTBOUND LEG ────────────────────────────────────────────")
        if r['dist_out'] is not None:
            row("Distance traveled (true)", f"{r['dist_out']:.4f} m")
        else:
            row("Status", "not completed")

        print(f"\n  ── TURN ────────────────────────────────────────────────────")
        if r['turn_deg'] is not None:
            e = r['turn_deg'] - 180.0
            row("Commanded (physical)",    "180.00°")
            row("Actual  (physical)",      f"{r['turn_deg']:.2f}°")
            row("Actual  (odom-yaw)",      f"{r['turn_odom_deg']:.2f}°  "
                                            f"[÷{YAW_SCALE} = {r['turn_deg']:.2f}°]")
            row("Error",                   f"{e:+.2f}°")
        else:
            row("Status", "not completed")

        print(f"\n  ── RETURN LEG ──────────────────────────────────────────────")
        if r['dist_ret'] is not None and r['dist_out'] is not None:
            diff = r['dist_ret'] - r['dist_out']
            pct  = (diff / r['dist_out'] * 100) if r['dist_out'] > 0 else 0.0
            row("Target",                  f"{r['dist_out']:.4f} m")
            row("Actual (true)",           f"{r['dist_ret']:.4f} m")
            row("Leg error",               f"{diff:+.4f} m  ({pct:+.2f}%)")
        else:
            row("Status", "not completed")

        print(f"\n  ── DRIFT (final position vs start) ─────────────────────────")
        if r.get('dx') is not None:
            drift = math.hypot(r['dx'], r['dy'])
            total = (r['dist_out'] or 0) + (r['dist_ret'] or 0)
            pct   = drift / total * 100 if total > 0 else 0
            row("ΔX (true)",               f"{r['dx']:+.4f} m")
            row("ΔY (true)",               f"{r['dy']:+.4f} m")
            row("Position drift",          f"{drift:.4f} m  "
                                            f"({pct:.2f}% of {total:.2f} m)")
            row("Heading drift",           f"{r['dheading']:+.2f}°")
        else:
            row("Status", "not completed")

        print(f"\n{'═'*W}\n")


# ── Entry point ───────────────────────────────────────────────────────────────

def _pick_speed() -> str:
    levels = list(LIN_SPEEDS.keys())
    if len(sys.argv) > 1 and sys.argv[1].lower() in levels:
        return sys.argv[1].lower()

    print("\n  Speed level?")
    for k in levels:
        print(f"    {k}  —  linear {LIN_SPEEDS[k]} m/s  /  angular {ANG_SPEEDS[k]} rad/s")
    print()
    while True:
        choice = input("  Enter l / m / h : ").strip().lower()
        if choice in levels:
            return choice
        print("  Please enter l, m, or h.")


def main():
    level = _pick_speed()
    rclpy.init()
    node = ForwardReturn(level)

    def _on_sigint(sig, frame):
        print('\n\n  ⚠  Ctrl+C — stopping robot …')
        node.stop()
        node.restore_terminal()
        if node._state not in (ForwardReturn._FWD, ForwardReturn._DONE):
            node.print_report(interrupted=True)
        else:
            print("  (no data to report)\n")
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _on_sigint)

    try:
        while rclpy.ok() and node._state != ForwardReturn._DONE:
            rclpy.spin_once(node, timeout_sec=0.05)
    except SystemExit:
        pass
    except Exception as exc:
        print(f"\n  ✗ Fatal: {exc}")
        node.stop()
    finally:
        node.restore_terminal()
        if node._state not in (ForwardReturn._DONE,):
            node.stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
