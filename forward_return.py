#!/usr/bin/env python3
"""
forward_return.py — Drive forward.  Press ENTER → robot turns 180° and
                     comes back the SAME distance.  Print drift report.

Run inside Docker:
  docker exec -it walter_dev bash -c "
    source /opt/ros/humble/setup.bash &&
    cd /ros2_ws &&
    python3 forward_return.py l"        # l / m / h

Speed levels:
   l :  0.02 m/s linear  /  0.01 rad/s angular
   m :  0.04 m/s linear  /  0.03 rad/s angular
   h :  0.06 m/s linear  /  0.06 rad/s angular

This is intentionally as simple as possible — odom for distance, odom yaw
for the 180° turn, with one hardware compensation: the IMU is mounted
VERTICALLY, so /odom yaw reports 2× the actual physical rotation.
YAW_SCALE = 2.0 compensates by waiting until the accumulated odom yaw
reaches 2π before the physical rotation reaches π.

Distance is left in odom units — outbound and return use the same
measurement so the symmetry is preserved regardless of any encoder
scale factor.

Ctrl+C anywhere → stops the robot and exits.
"""

import math
import select
import signal
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ── Speed levels ──────────────────────────────────────────────────────────────
LIN_SPEEDS = {'l': 0.02, 'm': 0.04, 'h': 0.06}   # m/s
ANG_SPEEDS = {'l': 0.01, 'm': 0.03, 'h': 0.06}   # rad/s

# Hardware compensation: IMU is mounted vertically → /odom yaw is 2× actual.
YAW_SCALE = 2.0

# Brief pause between motion phases (seconds)
PAUSE_S = 1.0


def _yaw_from_quat(q):
    """Z-axis yaw from a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _angle_diff(a, b):
    """Shortest signed angular difference a − b, normalised to (−π, π]."""
    d = a - b
    while d >  math.pi: d -= 2 * math.pi
    while d < -math.pi: d += 2 * math.pi
    return d


# ── Node ──────────────────────────────────────────────────────────────────────

class ForwardReturn(Node):

    # Phase enum
    FORWARD, PAUSE_BEFORE_TURN, TURNING, PAUSE_BEFORE_RETURN, RETURNING, DONE = range(6)

    def __init__(self, level):
        super().__init__('forward_return')

        self._lin = LIN_SPEEDS[level]
        self._ang = ANG_SPEEDS[level]

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_timer(0.05, self._tick)        # 20 Hz control loop

        # State
        self._state    = self.FORWARD
        self._odom     = None
        self._pause_t0 = None

        # Position snapshots
        self._fwd_x0 = self._fwd_y0 = None     # where forward leg started
        self._ret_x0 = self._ret_y0 = None     # where return leg started
        self._dist_out = 0.0                   # distance recorded at ENTER

        # Yaw accumulator (raw odom-yaw radians, includes 2× scaling)
        self._yaw_prev  = None
        self._yaw_accum = 0.0

        # Terminal setup so we can read ENTER without it appearing on screen
        try:
            self._old_term = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
        except Exception:
            self._old_term = None

        # Banner
        eta = math.pi / self._ang
        print(f"\n  Walter forward-return  —  level {level.upper()}")
        print(f"  Linear  {self._lin} m/s    Angular  {self._ang} rad/s")
        print(f"  (180° physical turn ≈ {eta:.0f} s)")
        print(f"\n  Driving forward …  press ENTER to turn back and return\n")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self._odom = msg

    # ── Helpers ───────────────────────────────────────────────────────────

    def _read_enter(self):
        """Non-blocking check for ENTER / SPACE keypress."""
        try:
            if select.select([sys.stdin], [], [], 0.0)[0]:
                return sys.stdin.read(1) in ('\r', '\n', ' ')
        except Exception:
            pass
        return False

    def _cmd(self, lx, az):
        t = Twist()
        t.linear.x  = lx
        t.angular.z = az
        self._pub.publish(t)

    # ── Main control loop (20 Hz) ─────────────────────────────────────────

    def _tick(self):
        if self._odom is None or self._state == self.DONE:
            return

        p   = self._odom.pose.pose.position
        yaw = _yaw_from_quat(self._odom.pose.pose.orientation)

        # Lazy snapshot of the forward-leg start position on first tick
        if self._fwd_x0 is None:
            self._fwd_x0, self._fwd_y0 = p.x, p.y

        # ── FORWARD — drive until user hits ENTER ────────────────────────
        if self._state == self.FORWARD:
            d = math.hypot(p.x - self._fwd_x0, p.y - self._fwd_y0)
            print(f"  → forward  {d:.3f} m   (press ENTER)       ",
                  end='\r', flush=True)
            if self._read_enter():
                self._dist_out = d
                self._cmd(0.0, 0.0)
                self._pause_t0 = time.monotonic()
                self._state    = self.PAUSE_BEFORE_TURN
                print(f"\n  [ENTER]  outbound = {d:.3f} m  —  stopping before turn")
            else:
                self._cmd(self._lin, 0.0)
            return

        # ── PAUSE before turn ───────────────────────────────────────────
        if self._state == self.PAUSE_BEFORE_TURN:
            self._cmd(0.0, 0.0)
            if time.monotonic() - self._pause_t0 >= PAUSE_S:
                self._yaw_prev  = yaw
                self._yaw_accum = 0.0
                self._state     = self.TURNING
                print(f"  Turning 180° at {self._ang} rad/s …")
            return

        # ── TURNING — integrate odom yaw, stop when 2π in odom space ────
        if self._state == self.TURNING:
            # Accumulate per-tick delta (small, so wrap-safe)
            self._yaw_accum += _angle_diff(yaw, self._yaw_prev)
            self._yaw_prev   = yaw

            turned_raw      = abs(self._yaw_accum)           # odom yaw radians
            turned_physical = turned_raw / YAW_SCALE          # actual radians
            target_raw      = math.pi * YAW_SCALE             # = 2π

            if turned_raw >= target_raw:
                self._cmd(0.0, 0.0)
                self._pause_t0 = time.monotonic()
                self._state    = self.PAUSE_BEFORE_RETURN
                print(f"\n  ✓ Turn done  ({math.degrees(turned_physical):.1f}° physical, "
                      f"{math.degrees(turned_raw):.1f}° odom)")
            else:
                self._cmd(0.0, self._ang)
                print(f"  → turning  {math.degrees(turned_physical):.1f}° / 180°       ",
                      end='\r', flush=True)
            return

        # ── PAUSE before return ─────────────────────────────────────────
        if self._state == self.PAUSE_BEFORE_RETURN:
            self._cmd(0.0, 0.0)
            if time.monotonic() - self._pause_t0 >= PAUSE_S:
                self._ret_x0, self._ret_y0 = p.x, p.y
                self._state = self.RETURNING
                print(f"  Returning {self._dist_out:.3f} m …")
            return

        # ── RETURNING — drive until we have covered dist_out ────────────
        if self._state == self.RETURNING:
            d = math.hypot(p.x - self._ret_x0, p.y - self._ret_y0)
            if d >= self._dist_out:
                self._cmd(0.0, 0.0)
                self._state = self.DONE
                self._report(p, d)
            else:
                self._cmd(self._lin, 0.0)
                print(f"  ← returning  {d:.3f} / {self._dist_out:.3f} m       ",
                      end='\r', flush=True)
            return

    # ── Report ────────────────────────────────────────────────────────────

    def _report(self, p, dist_ret):
        dx = p.x - self._fwd_x0
        dy = p.y - self._fwd_y0
        drift = math.hypot(dx, dy)
        print()
        print(f"  ──────────────────────────────────────────")
        print(f"  ✓ DONE")
        print(f"     Outbound     : {self._dist_out:.3f} m")
        print(f"     Return       : {dist_ret:.3f} m")
        print(f"     Final drift  : {drift:.3f} m  (Δx={dx:+.3f}, Δy={dy:+.3f})")
        print(f"  ──────────────────────────────────────────\n")

    # ── Cleanup ───────────────────────────────────────────────────────────

    def cleanup(self):
        self._cmd(0.0, 0.0)
        if self._old_term is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(),
                                  termios.TCSADRAIN, self._old_term)
            except Exception:
                pass


# ── Entry point ───────────────────────────────────────────────────────────────

def _pick_level():
    """Return l/m/h either from argv[1] or by interactive prompt."""
    if len(sys.argv) > 1 and sys.argv[1].lower() in LIN_SPEEDS:
        return sys.argv[1].lower()
    print("Speed levels:")
    for k in LIN_SPEEDS:
        print(f"  {k}  :  linear {LIN_SPEEDS[k]} m/s   angular {ANG_SPEEDS[k]} rad/s")
    while True:
        c = input("Pick level [l/m/h]: ").strip().lower()
        if c in LIN_SPEEDS:
            return c


def main():
    level = _pick_level()
    rclpy.init()
    node = ForwardReturn(level)

    def _sigint(sig, frame):
        print("\n  Ctrl+C — stopping robot")
        node.cleanup()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)
    signal.signal(signal.SIGINT, _sigint)

    try:
        while rclpy.ok() and node._state != ForwardReturn.DONE:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.cleanup()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
