"""
Motor Control Module for Walter Robot
Sends motor commands to ESP32 slave via I2C
"""

import struct
import time
from typing import Optional
from smbus2 import SMBus

# ── IMU constants (LSM6DSV, I2C 0x6A) ──────────────────────────────────────────
# Gyro/accel axis mapping (robot vertical mount, matches gyroscope.py):
#   Yaw  (turning L/R)  = sensor GY  (register 0x24)
#   Accel forward/back  = sensor AX  (register 0x28)
_IMU_ADDR        = 0x6A
_GYRO_FACTOR     = 0.00875   # deg/s per LSB  (±250 dps range)
_ACCEL_FACTOR    = 0.000061  # g per LSB      (±2 g range)
_GYRO_DEADZONE   = 0.3       # deg/s  — below this treat as noise
_ACCEL_DEADZONE  = 0.02      # g      — below this treat as noise
# Degrees before target at which to send STOP to compensate motor inertia overshoot.
# With decel ramp active the robot is already slow here, so a small value is fine.
# Increase slightly if still overshooting; decrease if stopping short.
BRAKE_LEAD_DEG   = 5.0
# Degrees before target at which to start slowing down (proportional ramp).
DECEL_ZONE_DEG   = 45.0
# Minimum PWM during the decel ramp.  Must be above your motors' stiction threshold.
# If the robot stalls/stutters near the end, increase this value.
MIN_TURN_SPEED   = 65


class IMUHelper:
    """
    Minimal LSM6DSV wrapper for accuracy / drift testing.
    Opens its own SMBus(1) file descriptor so it does not conflict with
    the motor-controller bus that talks to the ESP32 (address 0x55).
    """

    def __init__(self):
        self._bus = SMBus(1)
        # CTRL1_XL: ODR = 120 Hz, ±2 g
        self._bus.write_byte_data(_IMU_ADDR, 0x10, 0x06)
        # CTRL2_G:  ODR = 120 Hz, ±250 dps
        self._bus.write_byte_data(_IMU_ADDR, 0x11, 0x06)
        self._gx_off = self._gy_off = self._gz_off = 0.0

    def _read16s(self, lo_reg: int) -> int:
        lo = self._bus.read_byte_data(_IMU_ADDR, lo_reg)
        hi = self._bus.read_byte_data(_IMU_ADDR, lo_reg + 1)
        val = (hi << 8) | lo
        return val - 0x10000 if val & 0x8000 else val

    def calibrate(self, samples: int = 150) -> None:
        """Collect gyro bias while robot is still (~1.5 s)."""
        print(f"  Calibrating IMU ({samples} samples) — keep robot still...")
        sx = sy = sz = 0.0
        for _ in range(samples):
            sx += self._read16s(0x22) * _GYRO_FACTOR
            sy += self._read16s(0x24) * _GYRO_FACTOR
            sz += self._read16s(0x26) * _GYRO_FACTOR
            time.sleep(0.01)
        self._gx_off, self._gy_off, self._gz_off = sx/samples, sy/samples, sz/samples
        print(f"  Offsets  GX:{self._gx_off:+.3f}  GY:{self._gy_off:+.3f}  GZ:{self._gz_off:+.3f} deg/s")

    def read_gyro(self) -> tuple:
        """Return calibrated (gx, gy, gz) in deg/s."""
        gx = self._read16s(0x22) * _GYRO_FACTOR - self._gx_off
        gy = self._read16s(0x24) * _GYRO_FACTOR - self._gy_off
        gz = self._read16s(0x26) * _GYRO_FACTOR - self._gz_off
        return gx, gy, gz

    def read_accel(self) -> tuple:
        """Return raw (ax, ay, az) in g."""
        ax = self._read16s(0x28) * _ACCEL_FACTOR
        ay = self._read16s(0x2A) * _ACCEL_FACTOR
        az = self._read16s(0x2C) * _ACCEL_FACTOR
        return ax, ay, az

    def close(self):
        self._bus.close()


class MotorController:
    """
    Controls motors via ESP32 slave over I2C

    Command Protocol:
    - Command byte (20-26) + 2 bytes speed + 2 bytes duration (ms)

    Commands:
    20: Move forward
    21: Move reverse
    22: Turn right (pivot)
    23: Turn left (pivot)
    24: Turn right (soft)
    25: Turn left (soft)
    26: Stop motors
    """

    # Command codes
    CMD_FORWARD = 20
    CMD_REVERSE = 21
    CMD_TURN_RIGHT = 22
    CMD_TURN_LEFT = 23
    CMD_TURN_RIGHT_SOFT = 24
    CMD_TURN_LEFT_SOFT = 25
    CMD_STOP = 26

    # Default parameters
    DEFAULT_SPEED = 100  # ~30% duty cycle (0-1023)
    DEFAULT_DURATION = 1000  # 1 second

    def __init__(self, i2c_address: int = 0x55, bus: int = 1):
        """
        Initialize motor controller

        Args:
            i2c_address: ESP32 slave I2C address (default 0x55)
            bus: I2C bus number (default 1)
        """
        self.bus = SMBus(bus)
        self.address = i2c_address
        print(f"Motor controller initialized on I2C bus {bus}, address 0x{i2c_address:02X}")

    def test_connection(self):
        """
        Test I2C connection by sending command 1 and expecting 'HELLO' response

        Returns:
            bool: True if connection successful, False otherwise
        """
        self.bus.write_byte(self.address, 1)
        time.sleep(0.1)  # Small delay for ESP32 to process
        response = self.bus.read_i2c_block_data(self.address, 0, 5)
        response_str = ''.join([chr(b) for b in response])
        print(f"Test response: {response_str}")
        return response_str == "HELLO"

    def _send_motor_command(self, command: int, speed: int = 0, duration: int = 0):
        """
        Send motor command to ESP32

        Args:
            command: Command code (20-26)
            speed: Motor speed (0-1023)
            duration: Duration in milliseconds
        """
        if command == self.CMD_STOP:
            self.bus.write_byte(self.address, command)
            print(f"Sent: STOP motors")
        else:
            data = [
                command,
                speed & 0xFF,
                (speed >> 8) & 0xFF,
                duration & 0xFF,
                (duration >> 8) & 0xFF
            ]
            self.bus.write_i2c_block_data(self.address, data[0], data[1:])
            print(f"Sent: cmd={command}, speed={speed}, duration={duration}ms")

    def forward(self, speed: Optional[int] = None, duration: Optional[int] = None):
        """
        Move forward

        Args:
            speed: Motor speed 0-1023 (default 300)
            duration: Duration in milliseconds (default 1000)
        """
        speed = speed if speed is not None else self.DEFAULT_SPEED
        duration = duration if duration is not None else self.DEFAULT_DURATION
        self._send_motor_command(self.CMD_FORWARD, speed, duration)

    def reverse(self, speed: Optional[int] = None, duration: Optional[int] = None):
        """
        Move in reverse

        Args:
            speed: Motor speed 0-1023 (default 300)
            duration: Duration in milliseconds (default 1000)
        """
        speed = speed if speed is not None else self.DEFAULT_SPEED
        duration = duration if duration is not None else self.DEFAULT_DURATION
        self._send_motor_command(self.CMD_REVERSE, speed, duration)

    def turn_right(self, speed: Optional[int] = None, duration: Optional[int] = None):
        """
        Turn right (pivot turn - both motors)

        Args:
            speed: Motor speed 0-1023 (default 300)
            duration: Duration in milliseconds (default 1000)
        """
        speed = speed if speed is not None else self.DEFAULT_SPEED
        duration = duration if duration is not None else self.DEFAULT_DURATION
        self._send_motor_command(self.CMD_TURN_RIGHT, speed, duration)

    def turn_left(self, speed: Optional[int] = None, duration: Optional[int] = None):
        """
        Turn left (pivot turn - both motors)

        Args:
            speed: Motor speed 0-1023 (default 300)
            duration: Duration in milliseconds (default 1000)
        """
        speed = speed if speed is not None else self.DEFAULT_SPEED
        duration = duration if duration is not None else self.DEFAULT_DURATION
        self._send_motor_command(self.CMD_TURN_LEFT, speed, duration)

    def turn_right_soft(self, speed: Optional[int] = None, duration: Optional[int] = None):
        """
        Turn right softly (only left motor moves)

        Args:
            speed: Motor speed 0-1023 (default 300)
            duration: Duration in milliseconds (default 1000)
        """
        speed = speed if speed is not None else self.DEFAULT_SPEED
        duration = duration if duration is not None else self.DEFAULT_DURATION
        self._send_motor_command(self.CMD_TURN_RIGHT_SOFT, speed, duration)

    def turn_left_soft(self, speed: Optional[int] = None, duration: Optional[int] = None):
        """
        Turn left softly (only right motor moves)

        Args:
            speed: Motor speed 0-1023 (default 300)
            duration: Duration in milliseconds (default 1000)
        """
        speed = speed if speed is not None else self.DEFAULT_SPEED
        duration = duration if duration is not None else self.DEFAULT_DURATION
        self._send_motor_command(self.CMD_TURN_LEFT_SOFT, speed, duration)

    def stop(self):
        """Stop both motors immediately"""
        self._send_motor_command(self.CMD_STOP)

    # ── IMU ACCURACY TESTS ────────────────────────────────────────────────────

    def turn_degrees_imu(
        self,
        degrees: float = 90.0,
        speed: int = 300,
        direction: str = 'right',
        multiplier: float = 1.0,
    ) -> dict:
        """
        Closed-loop turn using gyroscope feedback.

        Sends an indefinite turn command, integrates the yaw axis (sensor GY)
        in real-time, and sends STOP when accumulated angle reaches
        (degrees * multiplier) - BRAKE_LEAD_DEG, then reports actual angle.

        Args:
            degrees:    Target rotation in degrees (e.g. 90).
            speed:      Motor PWM speed 0-1023.
            direction:  'right' or 'left'.
            multiplier: Scale factor applied to degrees (e.g. 2.0 → 180°).

        Returns:
            dict with target_deg, actual_deg, elapsed_ms, overshoot_deg.
        """
        target = abs(degrees) * multiplier
        cmd = self.CMD_TURN_RIGHT if direction == 'right' else self.CMD_TURN_LEFT

        imu = IMUHelper()
        imu.calibrate()

        yaw = 0.0
        dt_poll = 0.008  # 8 ms ≈ 125 Hz poll rate

        print(f"\n  Turning {direction} | target={target:.1f}°  (={degrees}°×{multiplier})")
        print(f"  Decel starts at {DECEL_ZONE_DEG}° remaining, brake at {BRAKE_LEAD_DEG}° remaining")

        # duration=0 → indefinite move; ESP32 waits for explicit stop command
        self._send_motor_command(cmd, speed, 0)
        t_start = time.perf_counter()
        last_t = t_start
        last_sent_speed = speed

        try:
            while True:
                now = time.perf_counter()
                dt = now - last_t
                last_t = now

                _, gy, _ = imu.read_gyro()   # yaw = sensor GY
                if abs(gy) > _GYRO_DEADZONE:
                    yaw += abs(gy) * dt

                remaining = target - yaw

                # Proportional deceleration: linearly scale speed from base_speed
                # down to MIN_TURN_SPEED across the DECEL_ZONE_DEG window.
                # Only resend command when speed has changed by ≥10 units to
                # avoid saturating the I2C bus with redundant writes.
                if BRAKE_LEAD_DEG < remaining <= DECEL_ZONE_DEG:
                    ratio = (remaining - BRAKE_LEAD_DEG) / (DECEL_ZONE_DEG - BRAKE_LEAD_DEG)
                    new_speed = int(MIN_TURN_SPEED + (speed - MIN_TURN_SPEED) * ratio)
                    new_speed = max(new_speed, MIN_TURN_SPEED)
                    if abs(new_speed - last_sent_speed) >= 10:
                        self._send_motor_command(cmd, new_speed, 0)
                        last_sent_speed = new_speed

                if remaining <= BRAKE_LEAD_DEG:
                    break

                time.sleep(dt_poll)
        finally:
            self._send_motor_command(self.CMD_STOP)
            elapsed_ms = (time.perf_counter() - t_start) * 1000
            # Keep polling for ~400 ms after stop to capture coasting overshoot
            coast_end = time.perf_counter() + 0.4
            while time.perf_counter() < coast_end:
                now = time.perf_counter()
                dt = now - last_t
                last_t = now
                _, gy, _ = imu.read_gyro()
                if abs(gy) > _GYRO_DEADZONE:
                    yaw += abs(gy) * dt
                time.sleep(dt_poll)
            imu.close()

        result = {
            'target_deg':   target,
            'actual_deg':   round(yaw, 1),
            'elapsed_ms':   round(elapsed_ms),
            'overshoot_deg': round(yaw - target, 1),
        }
        print(f"  ┌─ TARGET  : {target:.1f}°")
        print(f"  ├─ ACTUAL  : {result['actual_deg']:.1f}°")
        print(f"  ├─ OVERSHOOT: {result['overshoot_deg']:+.1f}°  (adjust BRAKE_LEAD_DEG to tune)")
        print(f"  └─ DURATION: {result['elapsed_ms']} ms")
        return result

    def move_distance_imu(
        self,
        direction: str = 'forward',
        speed: int = 300,
        duration_ms: int = 2000,
    ) -> dict:
        """
        Timed move with parallel IMU accelerometer integration.

        Runs the motors for duration_ms, then reports:
        - Actual timer elapsed
        - IMU-estimated distance (double-integrated AX)
        - Peak acceleration seen

        NOTE: IMU distance is a rough drift-check tool only. Double-integration
        of cheap MEMS accelerometers accumulates error quickly. Use encoders
        for reliable odometry. The value here lets you see if the robot is
        even approximately reaching the expected distance.

        Args:
            direction:   'forward' or 'reverse'.
            speed:       Motor PWM speed 0-1023.
            duration_ms: How long to drive (milliseconds).

        Returns:
            dict with direction, speed, timer_ms, imu_dist_cm, peak_accel_g.
        """
        G = 9.81  # m/s²
        cmd = self.CMD_FORWARD if direction == 'forward' else self.CMD_REVERSE

        imu = IMUHelper()
        imu.calibrate()

        vel_ms = 0.0   # velocity  m/s
        dist_m = 0.0   # distance  m
        peak_ax = 0.0
        dt_poll = 0.008

        print(f"\n  Moving {direction} | speed={speed} | duration={duration_ms} ms")

        self._send_motor_command(cmd, speed, duration_ms)
        t_start = time.perf_counter()
        last_t = t_start

        while True:
            now = time.perf_counter()
            if (now - t_start) * 1000 >= duration_ms:
                break

            dt = now - last_t
            last_t = now

            ax, _, _ = imu.read_accel()   # forward accel = sensor AX
            if abs(ax) > _ACCEL_DEADZONE:
                accel_ms2 = ax * G
                vel_ms  += accel_ms2 * dt
                dist_m  += vel_ms * dt

            if abs(ax) > abs(peak_ax):
                peak_ax = ax

            time.sleep(dt_poll)

        elapsed_ms = (time.perf_counter() - t_start) * 1000
        imu.close()

        result = {
            'direction':    direction,
            'speed':        speed,
            'timer_ms':     round(elapsed_ms),
            'imu_dist_cm':  round(abs(dist_m) * 100, 1),
            'peak_accel_g': round(peak_ax, 4),
        }
        print(f"  ┌─ TIMER ELAPSED : {result['timer_ms']} ms")
        print(f"  ├─ IMU DISTANCE  : ~{result['imu_dist_cm']} cm  (double-integration estimate)")
        print(f"  ├─ PEAK ACCEL AX : {result['peak_accel_g']:+.4f} g")
        print(f"  └─ Note: IMU distance drifts — use for qualitative comparison only.")
        return result

    def close(self):
        """Close I2C connection"""
        if hasattr(self, 'bus'):
            self.bus.close()


# Example usage
if __name__ == "__main__":
    import time

    motors = MotorController()

    def print_menu():
        print("\n" + "="*50)
        print("  WALTER ROBOT - MOTOR CONTROL TEST MENU")
        print("="*50)
        print("T. Test I2C Connection (HELLO)")
        print("1. Move Forward")
        print("2. Move Reverse")
        print("3. Turn Right (pivot)")
        print("4. Turn Left (pivot)")
        print("5. Turn Right (soft)")
        print("6. Turn Left (soft)")
        print("7. Stop Motors")
        print("8. Custom Command (enter speed & duration)")
        print("9. Run Full Test Sequence")
        print("--- IMU ACCURACY TESTS ---")
        print("A. Turn X° closed-loop (IMU gyro feedback)")
        print("B. Move distance (timer + IMU accel report)")
        print("0. Exit")
        print("="*50)

    def get_speed_duration():
        speed = int(input("Enter speed (0-1023, default 300): ") or "300")
        duration = int(input("Enter duration in ms (default 1000): ") or "1000")
        return speed, duration

    def run_full_test():
        print("\n>>> Running full test sequence...")

        print("\n1. Moving forward...")
        motors.forward(speed=100, duration=2000)
        time.sleep(2.5)

        print("2. Moving reverse...")
        motors.reverse(speed=100, duration=2000)
        time.sleep(2.5)

        print("3. Turning right (pivot)...")
        motors.turn_right(speed=100, duration=1500)
        time.sleep(2)

        print("4. Turning left (pivot)...")
        motors.turn_left(speed=100, duration=1500)
        time.sleep(2)

        print("5. Turning right (soft)...")
        motors.turn_right_soft(speed=100, duration=1500)
        time.sleep(2)

        print("6. Turning left (soft)...")
        motors.turn_left_soft(speed=100, duration=1500)
        time.sleep(2)

        print("7. Stopping...")
        motors.stop()

        print("\n>>> Full test sequence complete!")

    print("\nWalter Robot Motor Controller")
    print("Ready to test motor movements\n")

    while True:
        print_menu()
        choice = input("\nSelect option (0-9, T): ").strip().upper()

        if choice == "T":
            print("\n>>> Testing I2C connection...")
            if motors.test_connection():
                print("✓ Connection successful! ESP32 responded with HELLO")
            else:
                print("✗ Connection failed or unexpected response")

        elif choice == "0":
            print("\nExiting motor controller...")
            motors.stop()
            break

        elif choice == "1":
            speed, duration = get_speed_duration()
            print(f"\n>>> Moving FORWARD: {speed} speed, {duration}ms")
            motors.forward(speed=speed, duration=duration)

        elif choice == "2":
            speed, duration = get_speed_duration()
            print(f"\n>>> Moving REVERSE: {speed} speed, {duration}ms")
            motors.reverse(speed=speed, duration=duration)

        elif choice == "3":
            speed, duration = get_speed_duration()
            print(f"\n>>> Turning RIGHT (pivot): {speed} speed, {duration}ms")
            motors.turn_right(speed=speed, duration=duration)

        elif choice == "4":
            speed, duration = get_speed_duration()
            print(f"\n>>> Turning LEFT (pivot): {speed} speed, {duration}ms")
            motors.turn_left(speed=speed, duration=duration)

        elif choice == "5":
            speed, duration = get_speed_duration()
            print(f"\n>>> Turning RIGHT (soft): {speed} speed, {duration}ms")
            motors.turn_right_soft(speed=speed, duration=duration)

        elif choice == "6":
            speed, duration = get_speed_duration()
            print(f"\n>>> Turning LEFT (soft): {speed} speed, {duration}ms")
            motors.turn_left_soft(speed=speed, duration=duration)

        elif choice == "7":
            print("\n>>> STOPPING motors")
            motors.stop()

        elif choice == "8":
            print("\n>>> Custom Command")
            print("Commands: 20=Forward, 21=Reverse, 22=TurnRight, 23=TurnLeft")
            cmd = int(input("Enter command code (20-26): "))
            if 20 <= cmd <= 26:
                if cmd == 26:
                    motors.stop()
                else:
                    speed, duration = get_speed_duration()
                    motors._send_motor_command(cmd, speed, duration)
            else:
                print("Invalid command code")

        elif choice == "9":
            run_full_test()

        elif choice == "A":
            print("\n>>> IMU Closed-Loop Turn Test")
            deg = float(input("  Target degrees (default 90): ") or "90")
            mult = float(input("  Multiplier — e.g. 1.0=90°, 2.0=180° (default 1.0): ") or "1.0")
            spd = int(input("  Speed 0-1023 (default 100): ") or "100")
            dirn = input("  Direction right/left (default right): ").strip().lower() or "right"
            if dirn not in ('right', 'left'):
                print("  Invalid direction — using 'right'")
                dirn = 'right'
            print(f"  BRAKE_LEAD_DEG = {BRAKE_LEAD_DEG}° (edit constant at top of file to tune)")
            motors.turn_degrees_imu(degrees=deg, speed=spd, direction=dirn, multiplier=mult)

        elif choice == "B":
            print("\n>>> IMU Distance Test (timer move + accel report)")
            dirn = input("  Direction forward/reverse (default forward): ").strip().lower() or "forward"
            if dirn not in ('forward', 'reverse'):
                print("  Invalid direction — using 'forward'")
                dirn = 'forward'
            spd = int(input("  Speed 0-1023 (default 100): ") or "100")
            dur = int(input("  Duration ms — tune so robot travels ~1m (default 2000): ") or "2000")
            motors.move_distance_imu(direction=dirn, speed=spd, duration_ms=dur)

        else:
            print("\n⚠️  Invalid option. Please select 0-9 or A/B.")

        # Small delay before showing menu again
        time.sleep(0.5)

    motors.close()
    print("Motor controller closed. Goodbye!")
