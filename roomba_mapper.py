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
    navigator.waitUntilNav2Active()

    # Pause so you can position Walter perfectly before the mission starts
    input("🛑 Position Walter in the center of the room. Press ENTER to start Auto-Mapping...")

    # 1. SET THE ANCHOR POINT
    home_pose = create_pose(navigator, 0.0, 0.0, 0.0)
    navigator.setInitialPose(home_pose)
    print("🏠 Origin Set! Starting Roomba Exploration Mode...")

    # --- MISSION PARAMETERS ---
    exploration_goals = 10  # How many random spots to drive to
    room_radius = 1.5       # The max distance (in meters) he is allowed to pick a target
    # --------------------------

    for i in range(exploration_goals):
        # 2. THE DICE ROLL
        rand_x = random.uniform(-room_radius, room_radius)
        rand_y = random.uniform(-room_radius, room_radius)
        rand_theta = random.uniform(-3.14, 3.14) # Random direction to face at the end

        target = create_pose(navigator, rand_x, rand_y, rand_theta)
        print(f"\n📍 Goal {i+1}/{exploration_goals}: Driving to [X: {rand_x:.2f}, Y: {rand_y:.2f}]")
        
        # 3. SEND THE COMMAND TO NAV2
        navigator.goToPose(target)

        # 4. MONITOR THE DRIVE
        while not navigator.isTaskComplete():
            time.sleep(1.0) # Check in once a second while he drives

        # 5. CHECK THE RESULT
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print("✅ Reached target! Pausing for SLAM snapshot...")
            time.sleep(2.0) # Pause so SLAM gets a clean, blur-free laser scan of the walls
        elif result == TaskResult.FAILED:
            print("🚧 Target is inside a wall or blocked! Skipping to the next random spot.")
            time.sleep(1.0)

    # 6. MISSION COMPLETE - GO HOME
    print("\n🏁 Exploration Complete! Returning to starting position...")
    navigator.goToPose(home_pose)
    
    while not navigator.isTaskComplete():
        time.sleep(1.0)
        
    print("🎉 Auto-Mapping Finished. You can now save the map!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
