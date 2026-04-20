import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
import math, time, random, threading

# ── Tuning ────────────────────────────────────────────────────────────────────
MAX_GOALS          = 60
MAX_GOAL_DIST      = 2.0     # metres per move
WALL_MARGIN        = 0.5     # metres — minimum clear space to consider a direction open
GOAL_TIMEOUT_S     = 30.0
BOXED_IN_RANGE     = 0.5
STABLE_MAP_COUNT   = 8
MAX_FAILURES       = 3
FORWARD_ARC_DEG    = 15.0    # narrow arc — only stop for things directly ahead
OBSTACLE_RANGE     = 0.45    # metres — stop if anything this close in forward arc
ROTATION_WARMUP_S  = 1.5     # seconds to wait after goal sent before checking obstacles


# ── Sensor Node ───────────────────────────────────────────────────────────────

class SensorNode(Node):
    def __init__(self):
        super().__init__('roomba_sensor')
        self.scan          = None
        self.map_received  = False
        self._unknown_prev = None
        self._stale_count  = 0

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan_filtered', self._on_scan, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

    def _on_scan(self, msg):
        self.scan = msg

    def _on_map(self, msg):
        self.map_received = True
        unknown = sum(1 for c in msg.data if c == -1)
        if self._unknown_prev is not None and abs(unknown - self._unknown_prev) < 30:
            self._stale_count += 1
        else:
            self._stale_count = 0
        self._unknown_prev = unknown

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x     = t.transform.translation.x
            y     = t.transform.translation.y
            theta = 2.0 * math.atan2(t.transform.rotation.z, t.transform.rotation.w)
            return x, y, theta
        except Exception as e:
            print(f"   [TF] lookup failed: {e}")
            return None

    def obstacle_ahead(self):
        """Check narrow forward arc only — ignores side beams during rotation."""
        if self.scan is None:
            return False, None, None
        arc_rad = math.radians(FORWARD_ARC_DEG)
        for i, r in enumerate(self.scan.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            angle = self.scan.angle_min + i * self.scan.angle_increment
            if abs(angle) <= arc_rad and r < OBSTACLE_RANGE:
                return True, math.degrees(angle), r
        return False, None, None

    def best_open_direction(self):
        """Return the angle (robot-relative) with the longest clear path."""
        if self.scan is None:
            return 0.0
        best_angle, best_range = 0.0, 0.0
        for i, r in enumerate(self.scan.ranges):
            if math.isnan(r) or math.isinf(r):
                r = 12.0  # treat inf as max range
            if r > best_range:
                best_range = r
                best_angle = self.scan.angle_min + i * self.scan.angle_increment
        return best_angle, best_range

    def is_boxed_in(self):
        if self.scan is None:
            return False
        valid = [r for r in self.scan.ranges if not math.isnan(r) and not math.isinf(r)]
        return bool(valid) and max(valid) < BOXED_IN_RANGE

    def scan_summary(self):
        if self.scan is None:
            return "no scan"
        valid = [r for r in self.scan.ranges if not math.isnan(r) and not math.isinf(r)]
        if not valid:
            return "all inf"
        fwd_idx = int((0.0 - self.scan.angle_min) / self.scan.angle_increment)
        fwd_idx = max(0, min(fwd_idx, len(self.scan.ranges) - 1))
        fwd_r   = self.scan.ranges[fwd_idx]
        return f"min={min(valid):.2f}m  max={max(valid):.2f}m  fwd={fwd_r:.2f}m"

    def map_is_stable(self):
        return self._stale_count >= STABLE_MAP_COUNT

    def reset_map_stability(self):
        self._stale_count = 0


# ── Helpers ───────────────────────────────────────────────────────────────────

def make_pose(navigator, x, y, theta):
    p = PoseStamped()
    p.header.frame_id    = 'map'
    p.header.stamp       = navigator.get_clock().now().to_msg()
    p.pose.position.x    = float(x)
    p.pose.position.y    = float(y)
    p.pose.orientation.z = math.sin(theta / 2.0)
    p.pose.orientation.w = math.cos(theta / 2.0)
    return p

def spin_node(node):
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    sensor    = SensorNode()
    navigator = BasicNavigator()

    t = threading.Thread(target=spin_node, args=(sensor,), daemon=True)
    t.start()

    print("⏳ Waiting for Nav2 action server...")
    navigator.nav_to_pose_client.wait_for_server()
    print("✅ Nav2 is active.")

    print("🗺️  Waiting for SLAM map...")
    while not sensor.map_received:
        time.sleep(0.5)
        print("   ...no /map yet")
    print("✅ Map received — giving SLAM 3s to stabilise...")
    time.sleep(3.0)

    input("🛑 Place Walter in the room. Press ENTER to start exploration...")
    print("🏠 Starting straight-line exploration...\n")

    goal_number       = 0
    consecutive_fails = 0
    current_heading   = None   # absolute map heading — kept until blocked

    try:
        while goal_number < MAX_GOALS:

            print(f"\n[SCAN] {sensor.scan_summary()}")
            print(f"[MAP ] stale={sensor._stale_count}/{STABLE_MAP_COUNT}  "
                  f"stable={sensor.map_is_stable()}  boxed_in={sensor.is_boxed_in()}")

            if sensor.is_boxed_in():
                print("🧱 Walls in all directions — room fully mapped!")
                break
            if sensor.map_is_stable():
                print("📐 Map stopped growing — exploration complete!")
                break

            pose = sensor.get_robot_pose()
            if pose is None:
                print("⚠️  Cannot get robot pose — retrying...")
                time.sleep(1.0)
                continue
            rx, ry, rtheta = pose
            print(f"[POSE] x={rx:.2f}  y={ry:.2f}  heading={math.degrees(rtheta):.1f}°")

            # ── Pick direction ─────────────────────────────────────────────────
            need_new_direction = current_heading is None or consecutive_fails >= MAX_FAILURES

            if need_new_direction:
                best_angle_rel, best_range = sensor.best_open_direction()
                current_heading = rtheta + best_angle_rel
                consecutive_fails = 0
                print(f"[DIR ] NEW direction: {math.degrees(best_angle_rel):+.1f}° rel "
                      f"→ abs heading={math.degrees(current_heading):.1f}°  "
                      f"clear={best_range:.2f}m")
            else:
                print(f"[DIR ] KEEPING heading={math.degrees(current_heading):.1f}°")

            # ── Compute goal ───────────────────────────────────────────────────
            goal_x = rx + MAX_GOAL_DIST * math.cos(current_heading)
            goal_y = ry + MAX_GOAL_DIST * math.sin(current_heading)

            goal_number += 1
            print(f"📍 Goal {goal_number}/{MAX_GOALS}: "
                  f"→ [{goal_x:.2f}, {goal_y:.2f}]  dist={MAX_GOAL_DIST:.1f}m")

            navigator.goToPose(make_pose(navigator, goal_x, goal_y, current_heading))

            # ── Wait for rotation to finish before checking obstacles ──────────
            print(f"   [ROT ] waiting {ROTATION_WARMUP_S}s for rotation to complete...")
            time.sleep(ROTATION_WARMUP_S)

            # ── Monitor forward travel ─────────────────────────────────────────
            blocked     = False
            deadline    = time.time() + GOAL_TIMEOUT_S
            check_count = 0

            while not navigator.isTaskComplete():
                check_count += 1

                if time.time() > deadline:
                    print(f"   ⏱️  Timeout — cancelling")
                    navigator.cancelTask()
                    blocked = True
                    break

                hit, hit_angle, hit_range = sensor.obstacle_ahead()
                if hit:
                    print(f"   🧱 Obstacle at {hit_angle:+.1f}°  "
                          f"range={hit_range:.2f}m — cancelling")
                    navigator.cancelTask()
                    blocked = True
                    break

                if check_count % 10 == 0:
                    print(f"   [MOVE] {sensor.scan_summary()}")

                time.sleep(0.3)

            result = navigator.getResult()
            print(f"   [RESULT] {result}  blocked={blocked}")

            if result == TaskResult.SUCCEEDED:
                print("   ✅ Reached — continuing same direction")
                consecutive_fails = 0
                sensor.reset_map_stability()
                time.sleep(1.0)
            elif blocked:
                print("   🚧 Blocked — will pick new direction next iteration")
                current_heading = None  # force new direction pick
                consecutive_fails = 0
                time.sleep(0.5)
            else:
                consecutive_fails += 1
                print(f"   🚧 Nav2 failed ({consecutive_fails}/{MAX_FAILURES}) — "
                      f"{'picking new direction' if consecutive_fails >= MAX_FAILURES else 'retrying same direction'}")
                if consecutive_fails >= MAX_FAILURES:
                    current_heading = None
                time.sleep(0.5)

        # ── Return home ────────────────────────────────────────────────────────
        print("\n🏁 Exploration done! Returning to start [0, 0]...")
        navigator.goToPose(make_pose(navigator, 0.0, 0.0, 0.0))
        deadline = time.time() + 60.0
        while not navigator.isTaskComplete():
            if time.time() > deadline:
                navigator.cancelTask()
                break
            time.sleep(0.5)

        if navigator.getResult() == TaskResult.SUCCEEDED:
            print("🎉 Walter is home. Mapping complete!")
        else:
            print("⚠️  Could not return home — manual retrieval needed.")

    except KeyboardInterrupt:
        print("\n🛑 EMERGENCY STOP")
        navigator.cancelTask()
        time.sleep(1.0)
        print("🛑 Walter secured.")

    finally:
        sensor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
