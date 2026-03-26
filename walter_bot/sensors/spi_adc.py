import spidev
import time
import RPi.GPIO as GPIO

# --- PIN DEFINITIONS ---
RESET_PIN = 27
DRDY_PIN = 22
CS_PIN = 8  # Manual Chip Select

# --- GPIO SETUP ---
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(RESET_PIN, GPIO.OUT)
GPIO.setup(DRDY_PIN, GPIO.IN)
GPIO.setup(CS_PIN, GPIO.OUT, initial=GPIO.HIGH) # Do Not Talk yet

# --- SPI SETUP ---
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000
spi.mode = 0b01
spi.no_cs = True  # CRITICAL: Manual CS control

# --- HARDWARE RESET ---
print("Resetting ADC...")
GPIO.output(RESET_PIN, GPIO.LOW)
time.sleep(0.1)
GPIO.output(RESET_PIN, GPIO.HIGH)
time.sleep(0.5) # Wait for the chip to fully boot up

# --- 24-BIT MATH FUNCTION ---
def decode_24bit_signed(byte1, byte2, byte3):
    """Combines 3 bytes and converts from two's complement to signed integer."""
    val = (byte1 << 16) | (byte2 << 8) | byte3
    if val & 0x800000: # If the 24th bit is 1, the number is negative
        val -= 0x1000000
    return val

print("-" * 50)
print("Reading Load Cell on Channel 1 (AIN1P / AIN1N)")
print("Press Ctrl+C to stop.")
print("-" * 50)

try:
    while True:
        # 1. Wait for DRDY to go LOW (ADC has fresh data)
        timeout = 1000
        while GPIO.input(DRDY_PIN) == GPIO.HIGH and timeout > 0:
            time.sleep(0.001)
            timeout -= 1

        if timeout == 0:
            print("⚠️ Timeout waiting for DRDY. Is the clock missing?")
            continue

        # 2. Clock out the 18-byte data frame
        GPIO.output(CS_PIN, GPIO.LOW)
        response = spi.xfer2([0x00] * 18)
        GPIO.output(CS_PIN, GPIO.HIGH)

        # 3. Parse Channel 1 (Word 2 -> Bytes 6, 7, 8)
        ch1_raw = decode_24bit_signed(response[6], response[7], response[8])

        # 4. Convert to Voltage
        # Default VREF is 1.2V. Default PGA Gain is 1.
        # Formula: Voltage = Code * (1.2V / Gain) / 2^23
        voltage_mv = (ch1_raw * 1.2 / (1 << 23)) * 1000

        # 5. Print the formatted result
        print(f"CH1 Raw Code: {ch1_raw:10d}  |  Voltage: {voltage_mv:8.4f} mV")

        # Pause so we don't flood the terminal screen
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nData collection stopped by user.")

finally:
    spi.close()
    GPIO.cleanup()
    print("SPI and GPIO cleaned up.")