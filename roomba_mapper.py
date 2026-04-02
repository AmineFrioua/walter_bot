import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import random
import time
import math

def create_pose(navigator, x, y, theta_rad):
    """Helper function to create a Nav2 target coordinate"""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    
    # Convert the angle into a quaternion (which ROS requires for rotation)
    pose.pose.orientation.z = math.sin(theta_rad / 2.0)
    pose.pose.orientation.w = math.cos(theta_rad / 2.0)
    return pose

def main():
    rclpy.init()
    navigator = BasicNavigator()

    print("🟢 Waiting for Nav2 to spin up...")
    # navigator.waitUntilNav2Active()

    input("🛑 Position Walter in the center of the room. Press ENTER to start Auto-Mapping...")

    home_pose = create_pose(navigator, 0.0, 0.0, 0.0)
    navigator.setInitialPose(home_pose)
    print("🏠 Origin Set! Starting Roomba Exploration Mode...")

    exploration_goals = 10
    room_radius = 1.5

    try:
        for i in range(exploration_goals):
            rand_x = random.uniform(-room_radius, room_radius)
            rand_y = random.uniform(-room_radius, room_radius)
            rand_theta = random.uniform(-3.14, 3.14)

            target = create_pose(navigator, rand_x, rand_y, rand_theta)
            print(f"\n📍 Goal {i+1}/{exploration_goals}: Driving to [X: {rand_x:.2f}, Y: {rand_y:.2f}]")
            
            navigator.goToPose(target)

            while not navigator.isTaskComplete():
                time.sleep(1.0)

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("✅ Reached target! Pausing for SLAM snapshot...")
                time.sleep(2.0)
            elif result == TaskResult.FAILED:
                print("🚧 Target blocked! Skipping to the next random spot.")
                time.sleep(1.0)

        print("\n🏁 Exploration Complete! Returning to base...")
        navigator.goToPose(home_pose)
        while not navigator.isTaskComplete():
            time.sleep(1.0)
            
        print("🎉 Auto-Mapping Finished!")

    except KeyboardInterrupt:
        # --- THE EMERGENCY BRAKE ---
        print("\n\n🛑 EMERGENCY STOP ACTIVATED! 🛑")
        print("Canceling current Nav2 mission and hitting the brakes...")
        navigator.cancelTask()
        time.sleep(1.0) # Give Nav2 a second to send the 0.0 velocity command
        print("🛑 Walter is secured.")

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
