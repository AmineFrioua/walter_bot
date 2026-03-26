import time
import numpy as np
from rplidar import RPLidar, RPLidarException

# --- CONFIGURATION ---
PORT_NAME = '/dev/ttyUSB0'  # Check your port
BAUD_RATE = 115200          # A1=115200, A2/A3=256000
FRONT_ANGLE_RANGE = 20      # Look +/- 20 degrees from center (Total 40 degree cone)
STOP_DISTANCE_MM = 500      # 50cm Stop Threshold

def run():
    print(f"Connecting to Lidar on {PORT_NAME}...")
    lidar = RPLidar(PORT_NAME, baudrate=BAUD_RATE)

    print("-" * 50)
    print(f"SAFETY MONITOR ACTIVE")
    print(f"Scanning Front Sector: +/- {FRONT_ANGLE_RANGE}°")
    print(f"Stop Threshold: {STOP_DISTANCE_MM} mm")
    print("-" * 50)

    try:
        # iter_scans yields a full 360 scan at a time
        for scan in lidar.iter_scans(max_buf_meas=500):

            # 1. CONVERT TO NUMPY FOR SPEED
            # Scan format is list of tuples: (quality, angle, distance)
            scan_data = np.array(scan)

            if len(scan_data) == 0: continue

            angles = scan_data[:, 1]   # 0 to 360 degrees
            distances = scan_data[:, 2] # Distance in mm

            # 2. FILTER FOR "FRONT" SECTOR
            # "Front" is usually 0 degrees.
            # We want angles > 340 OR angles < 20 (handling the 360 wrap-around)

            # Create a mask (True/False list) for points in our front cone
            front_mask = (angles > (360 - FRONT_ANGLE_RANGE)) | (angles < FRONT_ANGLE_RANGE)

            # Get only the distances in the front sector
            front_distances = distances[front_mask]

            # 3. ANALYZE DISTANCES
            if len(front_distances) > 0:
                # Find the closest object in the front cone
                min_dist = np.min(front_distances)

                # Check if it is a valid reading (Lidar sometimes returns 0 for "infinity" or error)
                if min_dist > 10:

                    # --- LOGIC TRIGGER ---
                    if min_dist < STOP_DISTANCE_MM:
                        print(f"OBSTACLE DETECTED! Dist: {min_dist:.0f}mm (STOP MOTORS)")

                        # FUTURE: robot.stop_motors()

                    else:
                        print(f"✓ Path Clear. Min Front Dist: {min_dist:.0f}mm")
            else:
                print("? No points in front sector")

    except KeyboardInterrupt:
        print("\nStopping...")
    except RPLidarException as e:
        print(f"Lidar Error: {e}")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

if __name__ == '__main__':
    run()