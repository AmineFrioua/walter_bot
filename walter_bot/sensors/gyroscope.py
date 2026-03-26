#!/usr/bin/env python3
"""
Robot Monitoring System (Vertical Mount)
- Yaw:   Sensor Y (Turning Left/Right)
- Pitch: Sensor X (Tilting Forward/Back)
- Roll:  Sensor Z (Leaning Side-to-Side)
"""
import time
from smbus2 import SMBus
from pathlib import Path
from datetime import datetime

bus = SMBus(1)

class DataLogger:
    def __init__(self, log_file: str = "robot_log.txt", interval: float = 900.0):
        self.log_file = Path(log_file)
        self.interval = interval
        self.last_log_time = time.time()
        self._initialize_log()

    def _initialize_log(self):
        if not self.log_file.exists():
            with open(self.log_file, 'w') as f:
                f.write(f"=== Robot Log Created: {datetime.now()} ===\n")
                f.write("Timestamp | Yaw_Turn | Pitch_Tilt | Roll_Lean | VBAT_V | Status\n")

    def should_log(self) -> bool:
        return (time.time() - self.last_log_time) >= self.interval

    def log(self, data: dict):
        try:
            timestamp = datetime.now().strftime('%H:%M:%S')
            log_line = (
                f"{timestamp} | "
                f"{data['yaw']:6.1f} | {data['pitch']:6.1f} | {data['roll']:6.1f} | "
                f"{data['VBAT']:.3f} | {data['charge_str']}\n"
            )
            with open(self.log_file, 'a') as f:
                f.write(log_line)
            self.last_log_time = time.time()
            print(f"✓ Saved to log")
        except Exception as e:
            print(f"Log Error: {e}")

class I2CDevice:
    def __init__(self, bus: SMBus, addr: int):
        self.bus = bus
        self.addr = addr
    def read8(self, reg): return self.bus.read_byte_data(self.addr, reg)
    def write8(self, reg, val): self.bus.write_byte_data(self.addr, reg, val)
    def read16s(self, lo, hi):
        val = (self.read8(hi) << 8) | self.read8(lo)
        return val - 0x10000 if val & 0x8000 else val

class BQ25820(I2CDevice):
    REG_ADC_CTRL = 0x2B
    def enable(self):
        self.write8(0x2C, 0x00)
        self.write8(self.REG_ADC_CTRL, self.read8(self.REG_ADC_CTRL) | 0x40)
    def get_data(self):
        self.write8(self.REG_ADC_CTRL, self.read8(self.REG_ADC_CTRL) | 0x80)
        time.sleep(0.005)
        vbat = self.read16s(0x33, 0x34) * 0.002
        status = self.read8(0x21) & 0x07
        status_map = ["Not Chg", "Trickle", "Pre-Chg", "Fast", "Taper", "Rsvd", "Top-Off", "Done"]
        return {"VBAT": vbat, "status_str": status_map[status]}

class LSM6DSV(I2CDevice):
    def initialize(self):
        self.write8(0x10, 0x06) # CTRL1 XL
        self.write8(0x11, 0x06) # CTRL2 G

    def read_gyro(self):
        factor = 0.00875
        gx = self.read16s(0x22, 0x23) * factor
        gy = self.read16s(0x24, 0x25) * factor
        gz = self.read16s(0x26, 0x27) * factor
        return gx, gy, gz

class PositionTracker:
    def __init__(self, imu, battery, logger):
        self.imu = imu
        self.battery = battery
        self.logger = logger

        # Robot Oriented Angles
        self.yaw = 0.0   # Turn (Sensor Y)
        self.pitch = 0.0 # Tilt (Sensor X)
        self.roll = 0.0  # Lean (Sensor Z)

        # Raw Offsets
        self.off_x = 0.0
        self.off_y = 0.0
        self.off_z = 0.0

        self.last_integrator_time = time.time()
        self.last_print_time = time.time()

    def calibrate(self):
        print("Calibrating (Robot Vertical)... Keep Still!")
        samples = 100
        tx, ty, tz = 0, 0, 0

        for _ in range(samples):
            gx, gy, gz = self.imu.read_gyro()
            tx += gx
            ty += gy
            tz += gz
            time.sleep(0.01)

        self.off_x = tx / samples
        self.off_y = ty / samples
        self.off_z = tz / samples

        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0

        print(f"Offsets -> X: {self.off_x:.2f}, Y: {self.off_y:.2f}, Z: {self.off_z:.2f}")

    def run(self):
        print("Starting Tracker (Vertical Mode). Ctrl+C to stop.")

        try:
            while True:
                now = time.time()
                dt = now - self.last_integrator_time
                self.last_integrator_time = now

                gx, gy, gz = self.imu.read_gyro()

                rgx = gx - self.off_x
                rgy = gy - self.off_y
                rgz = gz - self.off_z

                if abs(rgx) < 0.5: rgx = 0
                if abs(rgy) < 0.5: rgy = 0
                if abs(rgz) < 0.5: rgz = 0

                self.yaw += rgy * dt
                self.pitch += rgx * dt
                self.roll += rgz * dt

                if (now - self.last_print_time) >= 1.0:
                    batt = self.battery.get_data()

                    data = {
                        "yaw": self.yaw,
                        "pitch": self.pitch,
                        "roll": self.roll,
                        "VBAT": batt['VBAT'],
                        "charge_str": batt['status_str']
                    }

                    self._print_status(data)

                    if self.logger.should_log():
                        self.logger.log(data)

                    self.last_print_time = now

                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\nStopped.")

    def _print_status(self, data):
        print(f"\rYAW(Turn):{data['yaw']:>6.1f}° | PITCH:{data['pitch']:>6.1f}° | ROLL:{data['roll']:>6.1f}° | Bat:{data['VBAT']:.2f}V", end="", flush=True)

if __name__ == "__main__":
    try:
        imu = LSM6DSV(bus, 0x6A)
        imu.initialize()

        bq = BQ25820(bus, 0x6B)
        bq.enable()

        logger = DataLogger()

        tracker = PositionTracker(imu, bq, logger)
        tracker.calibrate()
        tracker.run()

    except Exception as e:
        print(f"Error: {e}")