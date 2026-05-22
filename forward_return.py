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
LIN_SPEEDS = {'l': 0.02, 'm': 0.04, 'h': 0.06}   # m/s
ANG_SPEEDS = {'l': 0.01, 'm': 0.03, 'h': 0.06}   # rad/s

# Deceleration zones
LIN_RAMP_DIST  = 0.12    # m    — ramp speed over this distance before stopping
LIN_MIN        = 0.01    # m/s  — minimum speed during ramp (prevents stall)
ANG_RAMP_RAD   = 0.25    # rad  — ramp angular speed this many rad before 180°
ANG_MIN        = 0.005   # rad/s

SETTLE_S  = 0.6          # pause at each stop before next phase
CTRL_HZ   = 20           # cmd_vel rate


# ── Helpers ───────────────────────────────────────────────────────────────────

def _ramp(remaining: float, zone: float, v_min: float, v_max: float) -> float:
    if remaining >= zone:
        return v_max
    return max(v_min, v_max * (remaining / zone))

def _yaw(q) -> float:
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

        # Phase-start snapshots
        self._fwd_x  = self._fwd_y  = None   # forward leg start
        self._slow_x = self._slow_y = None   # position when ENTER pressed
        self._ret_x  = self._ret_y  = None   # return leg start
        self._turn_yaw_start = None           # yaw at start of turn

        # State machine
        self._state      = self._FWD
        self._settle_t0  = None
        self._next_state = None

        # Results
        self._dist_out   = 0.0    # outbound distance at stop
        self._dist_ret   = 0.0    # return distance covered
        self._r = dict(
            dist_out=None, dist_ret=None,
            turn_deg=None,
            dx=None, dy=None, dheading=None,
        )

        # Keyboard (set cbreak so we get each keypress immediately)
        self._old_term = None
        try:
            self._old_term = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            pass   # not a real TTY (e.g. piped input) — fall back gracefully

        # Banner
        eta_turn = math.degrees(math.pi) / math.degrees(self._ang)
        print(f"\n{'─'*60}")
        print("  WALTER FORWARD-RETURN")
        print(f"  {datetime.now():%Y-%m-%d  %H:%M:%S}")
        print(f"{'─'*60}")
        print(f"  Speed level   : {speed_level.upper()}")
        print(f"  Linear speed  : {self._lin} m/s")
        print(f"  Angular speed : {self._ang} rad/s  "
              f"(180° ≈ {eta_turn:.0f} s)")
        print(f"{'─'*60}")
        print(f"\n● Waiting for odometry …")

    # ── Odom callback ──────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self._odom = msg
        if not self._odom_ready:
            p = msg.pose.pose.position
            # Capture global start as floats
            self._r['_gx']  = p.x
            self._r['_gy']  = p.y
            self._r['_gyaw'] = _yaw(msg.pose.pose.orientation)
            self._odom_ready = True

    # ── Tick ───────────────────────────────────────────────────────────────────

    def _tick(self):
        if self._state == self._DONE:
            return

        # Wait for first odom message
        if not self._odom_ready:
            return

        # Lazy capture of forward-start
        if self._fwd_x is None and self._state == self._FWD:
            p = self._odom.pose.pose.position
            self._fwd_x = p.x
            self._fwd_y = p.y
            print(f"\n  [ready]  x={p.x:.4f}  y={p.y:.4f}  "
                  f"yaw={math.degrees(_yaw(self._odom.pose.pose.orientation)):.1f}°")
            print(f"\n● Driving forward at {self._lin} m/s  —  press ENTER to return\n")

        # Check for keypress
        key = self._read_key()

        p   = self._odom.pose.pose.position
        yaw = _yaw(self._odom.pose.pose.orientation)

        # ── FORWARD ────────────────────────────────────────────────────────────
        if self._state == self._FWD:
            d = self._dist_from(self._fwd_x, self._fwd_y)
            print(f"  → {d:.3f} m        ", end='\r', flush=True)
            if key:
                # User pressed ENTER — start decelerating
                self._slow_x = p.x
                self._slow_y = p.y
                self._dist_out = d    # will be updated at actual stop
                self._state = self._SLOWING
                print(f"\n  [ENTER] decelerating at {d:.3f} m …")
            else:
                self._cmd(self._lin, 0.0)

        # ── SLOWING (decelerate to stop) ────────────────────────────────────────
        elif self._state == self._SLOWING:
            d_since_trigger = self._dist_from(self._slow_x, self._slow_y)
            remaining = LIN_RAMP_DIST - d_since_trigger
            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                # Record actual outbound distance
                self._dist_out = self._dist_from(self._fwd_x, self._fwd_y)
                self._r['dist_out'] = self._dist_out
                print(f"  ✓ Stopped  outbound = {self._dist_out:.4f} m")
                self._enter_settle(self._TURN)
            else:
                speed = _ramp(remaining, LIN_RAMP_DIST, LIN_MIN, self._lin)
                self._cmd(speed, 0.0)
                print(f"  → slowing {speed:.3f} m/s …  ", end='\r', flush=True)

        # ── SETTLE ─────────────────────────────────────────────────────────────
        elif self._state == self._SETTLE:
            self._cmd(0.0, 0.0)
            if time.monotonic() - self._settle_t0 >= SETTLE_S:
                self._begin(self._next_state)

        # ── TURN ───────────────────────────────────────────────────────────────
        elif self._state == self._TURN:
            if self._turn_yaw_start is None:
                self._turn_yaw_start = yaw   # captured on first tick of TURN
                eta = math.pi / self._ang
                print(f"\n  [turn]  start yaw={math.degrees(yaw):.1f}°  "
                      f"ETA ≈ {eta:.0f} s")

            turned    = abs(_angle_diff(yaw, self._turn_yaw_start))
            remaining = math.pi - turned
            if remaining <= 0.0:
                self._cmd(0.0, 0.0)
                self._r['turn_deg'] = math.degrees(turned)
                print(f"\n  ✓ Turn done  {math.degrees(turned):.1f}°  "
                      f"(Δ {math.degrees(turned) - 180.0:+.1f}°)")
                self._enter_settle(self._RETURN)
            else:
                ang = _ramp(remaining, ANG_RAMP_RAD, ANG_MIN, self._ang)
                self._cmd(0.0, ang)
                print(f"  → turned {math.degrees(turned):.1f}°  "
                      f"remain {math.degrees(remaining):.1f}°  "
                      f"w={ang:.4f} r/s    ", end='\r', flush=True)

        # ── RETURN ─────────────────────────────────────────────────────────────
        elif self._state == self._RETURN:
            d         = self._dist_from(self._ret_x, self._ret_y)
            remaining = self._dist_out - d
            self._dist_ret = d

            if key:
                # User pressed ENTER again — stop early
                self._cmd(0.0, 0.0)
                self._r['dist_ret'] = d
                print(f"\n  [ENTER] stopped early at {d:.4f} m of {self._dist_out:.4f} m")
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
        if sx is None or self._odom is None:
            return 0.0
        p = self._odom.pose.pose.position
        return math.hypot(p.x - sx, p.y - sy)

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
            self._turn_yaw_start = None   # captured on first tick
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
            self._r['dx']       = p.x - gx
            self._r['dy']       = p.y - gy
            dh = _angle_diff(current_yaw, gyaw)
            self._r['dheading'] = math.degrees(dh)
        self.print_report()

    def _read_key(self) -> bool:
        """Return True if ENTER or SPACE was pressed since last tick."""
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
        W   = 60
        tag = '  [INTERRUPTED]' if interrupted else ''

        def row(label, value):
            print(f"    {label:<28}: {value}")

        print(f"\n{'═'*W}")
        print(f"  FORWARD-RETURN REPORT{tag}")
        print(f"  {datetime.now():%Y-%m-%d  %H:%M:%S}")
        print(f"{'═'*W}")

        print(f"\n  ── SPEED SETTINGS ──────────────────────────────────────")
        row("Linear speed",        f"{self._lin} m/s")
        row("Angular speed",       f"{self._ang} rad/s")

        print(f"\n  ── OUTBOUND LEG ────────────────────────────────────────")
        if r['dist_out'] is not None:
            row("Distance traveled",   f"{r['dist_out']:.4f} m")
        else:
            row("Status", "not completed")

        print(f"\n  ── TURN ────────────────────────────────────────────────")
        if r['turn_deg'] is not None:
            e = r['turn_deg'] - 180.0
            row("Commanded",           "180.0°")
            row("Actual (odom yaw)",   f"{r['turn_deg']:.2f}°")
            row("Error",               f"{e:+.2f}°")
        else:
            row("Status", "not completed")

        print(f"\n  ── RETURN LEG ──────────────────────────────────────────")
        if r['dist_ret'] is not None and r['dist_out'] is not None:
            diff = r['dist_ret'] - r['dist_out']
            row("Target",              f"{r['dist_out']:.4f} m")
            row("Actual (odom)",       f"{r['dist_ret']:.4f} m")
            row("Leg error",           f"{diff:+.4f} m  ({diff/r['dist_out']*100:+.2f}%)"
                                        if r['dist_out'] > 0 else f"{diff:+.4f} m")
        else:
            row("Status", "not completed")

        print(f"\n  ── DRIFT (final position vs start) ─────────────────────")
        if r.get('dx') is not None:
            drift = math.hypot(r['dx'], r['dy'])
            total = (r['dist_out'] or 0) + (r['dist_ret'] or 0)
            pct   = drift / total * 100 if total > 0 else 0
            row("ΔX",                  f"{r['dx']:+.4f} m")
            row("ΔY",                  f"{r['dy']:+.4f} m")
            row("Position drift",      f"{drift:.4f} m  ({pct:.2f}% of {total:.2f} m)")
            row("Heading drift",       f"{r['dheading']:+.2f}°")
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
