#!/usr/bin/env python3
"""
Load Cell Test Script — Walter Robot
=====================================
Reads 24-bit ADC data via SPI from load cells wired through an on-board ADC.

ADC frame : 18 bytes = 6 channels × 3 bytes (24-bit two's complement each)
            Word 0 → CH0 (bytes 0-2), Word 1 → CH1 (bytes 3-5), …
Reference : 1.2 V internal, PGA gain = 1 (defaults — adjust in config block)

Pin wiring (BCM numbering):
  GPIO 27 → /RESET (active-low)
  GPIO 22 → DRDY   (input — goes LOW when fresh sample is ready)
  GPIO 8  → /CS    (manual chip-select, tied to SPI CE0 pad)

Usage:
  python3 load_cell_test.py                  # live single-channel read
  python3 load_cell_test.py --channel 2      # read channel 2 instead
  python3 load_cell_test.py --all            # show all 6 channels
  python3 load_cell_test.py --debug          # + raw bytes per frame
  python3 load_cell_test.py --calibrate      # interactive tare + scale wizard
  python3 load_cell_test.py --samples 200    # capture N samples then exit
"""

import spidev
import time
import sys
import argparse
import statistics
import RPi.GPIO as GPIO

# ── Configuration ──────────────────────────────────────────────────────────────

RESET_PIN      = 27
DRDY_PIN       = 22
CS_PIN         = 8

SPI_BUS        = 0
SPI_DEVICE     = 0
SPI_SPEED_HZ   = 1_000_000
SPI_MODE       = 0b01           # CPOL=0, CPHA=1  (check your ADC datasheet)

FRAME_BYTES    = 18             # 6 channels × 3 bytes each
NUM_CHANNELS   = 6
DEFAULT_CH     = 1              # load cell channel (0-indexed)

VREF_V         = 1.2            # Internal reference voltage (V)
PGA_GAIN       = 1              # Programmable gain amplifier setting
ADC_BITS       = 24             # Resolution

DRDY_TIMEOUT_MS  = 1000         # Max wait for DRDY to assert (ms)
SAMPLE_INTERVAL  = 0.2          # Seconds between terminal updates

# Calibration — set after running --calibrate
TARE_OFFSET_CODE = 0            # Raw code when load cell is empty
SCALE_FACTOR_KG  = 1e-4         # kg per raw code (rough default; calibrate!)

WINDOW_SIZE    = 10             # Rolling-average sample count

# ── Helpers ────────────────────────────────────────────────────────────────────

def decode_24bit(b0: int, b1: int, b2: int) -> int:
    """Combine three bytes into a signed 24-bit two's-complement integer."""
    raw = (b0 << 16) | (b1 << 8) | b2
    if raw & 0x800000:          # MSB set → negative
        raw -= 0x1000000
    return raw


def code_to_mv(code: int) -> float:
    """Convert raw ADC code to millivolts using VREF and PGA gain."""
    return code * (VREF_V / PGA_GAIN) / (1 << (ADC_BITS - 1)) * 1000.0


def code_to_kg(code: int) -> float:
    """Convert raw ADC code to kilograms using calibration constants."""
    return (code - TARE_OFFSET_CODE) * SCALE_FACTOR_KG


def bar(value: float, lo: float, hi: float, width: int = 20) -> str:
    """ASCII progress bar scaled between lo and hi."""
    fraction = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    filled = int(fraction * width)
    return "[" + "█" * filled + "░" * (width - filled) + "]"


# ── Hardware setup / teardown ──────────────────────────────────────────────────

def setup_gpio() -> None:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(DRDY_PIN,  GPIO.IN)
    GPIO.setup(CS_PIN,    GPIO.OUT, initial=GPIO.HIGH)  # CS idle-high = deselected
    print(f"  GPIO OK  (RESET={RESET_PIN}, DRDY={DRDY_PIN}, CS={CS_PIN})")


def setup_spi() -> spidev.SpiDev:
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode         = SPI_MODE
    spi.no_cs        = True     # CRITICAL: we drive CS manually
    print(f"  SPI  OK  (bus={SPI_BUS}.{SPI_DEVICE}, speed={SPI_SPEED_HZ//1000} kHz, mode={SPI_MODE:#04b})")
    return spi


def hardware_reset() -> None:
    print("  ADC reset…", end=" ", flush=True)
    GPIO.output(RESET_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    time.sleep(0.5)             # allow ADC to boot and stabilise
    print("done")


def wait_drdy(timeout_ms: int = DRDY_TIMEOUT_MS) -> bool:
    """Block until DRDY goes LOW (data ready) or timeout expires. Returns True on success."""
    deadline = time.monotonic() + timeout_ms / 1000.0
    while GPIO.input(DRDY_PIN) == GPIO.HIGH:
        if time.monotonic() > deadline:
            return False
        time.sleep(0.0005)
    return True


def read_frame(spi: spidev.SpiDev) -> list[int]:
    """Clock out one full 18-byte data frame from the ADC."""
    GPIO.output(CS_PIN, GPIO.LOW)
    response = spi.xfer2([0x00] * FRAME_BYTES)
    GPIO.output(CS_PIN, GPIO.HIGH)
    return response


def parse_all_channels(frame: list[int]) -> list[int]:
    """Decode all 6 channels from an 18-byte frame."""
    return [
        decode_24bit(frame[i*3], frame[i*3 + 1], frame[i*3 + 2])
        for i in range(NUM_CHANNELS)
    ]


# ── Modes ──────────────────────────────────────────────────────────────────────

def run_live(spi: spidev.SpiDev, channel: int, show_all: bool, debug: bool,
             max_samples: int) -> None:
    """Continuously read and display load cell data."""

    window: list[int] = []
    sample_count = 0
    error_count  = 0
    start_time   = time.monotonic()

    print(f"\n{'─'*60}")
    print(f"  Channel  : CH{channel}  (0-indexed)")
    print(f"  VREF     : {VREF_V} V   PGA gain: {PGA_GAIN}")
    print(f"  Tare     : {TARE_OFFSET_CODE}   Scale: {SCALE_FACTOR_KG} kg/code")
    if max_samples:
        print(f"  Capturing: {max_samples} samples then exit")
    print(f"{'─'*60}")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            # ── Wait for fresh data ──
            if not wait_drdy():
                error_count += 1
                print(f"  ⚠  DRDY timeout #{error_count} — is the ADC clock running?")
                if error_count >= 5:
                    print("  ✖  Too many timeouts. Check wiring and SPI mode.")
                    break
                continue

            # ── Read frame ──
            frame   = read_frame(spi)
            codes   = parse_all_channels(frame)
            ch_code = codes[channel]

            # ── Rolling average ──
            window.append(ch_code)
            if len(window) > WINDOW_SIZE:
                window.pop(0)
            avg_code = statistics.mean(window)

            # ── Derived values ──
            mv       = code_to_mv(ch_code)
            avg_mv   = code_to_mv(avg_code)
            kg       = code_to_kg(ch_code)
            avg_kg   = code_to_kg(avg_code)
            noise    = statistics.stdev(window) if len(window) > 1 else 0.0

            sample_count += 1

            # ── Debug: raw bytes ──
            if debug:
                hex_str = " ".join(f"{b:02X}" for b in frame)
                print(f"  [{sample_count:5d}] raw: {hex_str}")
                print(f"         all ch codes: {codes}")

            # ── Main display ──
            vol_bar = bar(mv, -1200, 1200)
            print(
                f"  [{sample_count:5d}]  "
                f"CH{channel} raw: {ch_code:+10d}  "
                f"  {mv:+9.4f} mV  {vol_bar}  "
                f"  avg: {avg_mv:+9.4f} mV  "
                f"  weight: {avg_kg:+7.4f} kg  "
                f"  noise: {code_to_mv(noise):.4f} mV"
            )

            # ── All channels ──
            if show_all:
                ch_line = "         all: " + "  ".join(
                    f"CH{i}={code_to_mv(c):+8.3f}mV" for i, c in enumerate(codes)
                )
                print(ch_line)

            # ── Exit after N samples ──
            if max_samples and sample_count >= max_samples:
                elapsed = time.monotonic() - start_time
                rate    = sample_count / elapsed
                print(f"\n  Captured {sample_count} samples in {elapsed:.1f} s ({rate:.1f} Hz)")
                break

            time.sleep(SAMPLE_INTERVAL)

    except KeyboardInterrupt:
        elapsed = time.monotonic() - start_time
        rate    = sample_count / max(elapsed, 0.001)
        print(f"\n\n  Stopped.  {sample_count} samples  {elapsed:.1f} s  {rate:.1f} Hz")


def run_calibrate(spi: spidev.SpiDev, channel: int) -> None:
    """Interactive two-point calibration wizard."""

    def stable_read(n: int = 30) -> float:
        """Average n consecutive readings."""
        samples = []
        for _ in range(n):
            if not wait_drdy():
                print("  ⚠  DRDY timeout during calibration read.")
                continue
            frame   = read_frame(spi)
            codes   = parse_all_channels(frame)
            samples.append(codes[channel])
            time.sleep(0.05)
        if not samples:
            raise RuntimeError("No samples received — check ADC wiring.")
        return statistics.mean(samples)

    print(f"\n{'─'*60}")
    print("  CALIBRATION WIZARD")
    print(f"  Channel: CH{channel}")
    print(f"{'─'*60}\n")

    # ── Step 1: Tare ──
    input("  Step 1/2 — Remove ALL weight from the load cell, then press Enter…")
    print("  Reading tare (30 samples)…")
    tare_code = stable_read(30)
    tare_mv   = code_to_mv(tare_code)
    print(f"  Tare code : {tare_code:+.1f}  ({tare_mv:+.4f} mV)\n")

    # ── Step 2: Known weight ──
    known_str = input("  Step 2/2 — Place a KNOWN weight on the load cell.\n"
                      "  Enter the weight in kg (e.g. 1.0): ").strip()
    try:
        known_kg = float(known_str)
    except ValueError:
        print("  ✖  Invalid weight. Aborting.")
        return

    print(f"  Reading loaded ({known_kg} kg, 30 samples)…")
    loaded_code = stable_read(30)
    loaded_mv   = code_to_mv(loaded_code)
    print(f"  Loaded code: {loaded_code:+.1f}  ({loaded_mv:+.4f} mV)")

    # ── Compute constants ──
    delta_code = loaded_code - tare_code
    if abs(delta_code) < 10:
        print("\n  ⚠  Delta too small — load cell may not be responding.")
        print(f"     Delta code: {delta_code:.1f}  (expect > 100 for a healthy cell)")
        return

    scale = known_kg / delta_code
    sensitivity_mv_per_kg = (loaded_mv - tare_mv) / known_kg

    print(f"\n{'─'*60}")
    print("  CALIBRATION RESULTS")
    print(f"{'─'*60}")
    print(f"  TARE_OFFSET_CODE = {tare_code:.0f}")
    print(f"  SCALE_FACTOR_KG  = {scale:.8f}   ({scale*1000:.6f} g/code)")
    print(f"  Sensitivity      : {sensitivity_mv_per_kg:.4f} mV/kg")
    print(f"{'─'*60}")
    print("\n  Copy the two lines above into load_cell_test.py (config block)")
    print("  and into web_server.py if you want live weight via /api/weight.\n")


# ── Entrypoint ─────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Walter load cell ADC test")
    parser.add_argument("--channel",   type=int,  default=DEFAULT_CH,
                        help=f"ADC channel to read (0-{NUM_CHANNELS-1}, default {DEFAULT_CH})")
    parser.add_argument("--all",       action="store_true",
                        help="Print all channel voltages on each line")
    parser.add_argument("--debug",     action="store_true",
                        help="Dump raw hex bytes and all decoded codes per frame")
    parser.add_argument("--calibrate", action="store_true",
                        help="Run interactive tare + scale calibration wizard")
    parser.add_argument("--samples",   type=int,  default=0,
                        help="Capture N samples then exit (0 = run forever)")
    args = parser.parse_args()

    if not (0 <= args.channel < NUM_CHANNELS):
        print(f"Error: --channel must be 0–{NUM_CHANNELS-1}")
        sys.exit(1)

    print("\nWalter — Load Cell ADC Test")
    print("="*60)
    print("Initialising hardware…")

    try:
        setup_gpio()
        spi = setup_spi()
        hardware_reset()

        # Quick sanity check — read one frame immediately after reset
        print("\nSanity check (1 frame after reset)…")
        if wait_drdy(timeout_ms=2000):
            frame = read_frame(spi)
            codes = parse_all_channels(frame)
            for i, c in enumerate(codes):
                flag = " ← target" if i == args.channel else ""
                print(f"  CH{i}: {c:+10d}  ({code_to_mv(c):+9.4f} mV){flag}")
        else:
            print("  ⚠  DRDY did not assert within 2 s after reset.")
            print("     Possible causes:")
            print("       - ADC not powered")
            print("       - Wrong SPI mode (try spi.mode = 0b00)")
            print("       - DRDY pin wired to wrong GPIO")
            print("       - Missing external clock (some ADCs need MCLK)")
            print()

        if args.calibrate:
            run_calibrate(spi, args.channel)
        else:
            run_live(spi, args.channel, args.all, args.debug, args.samples)

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"\n  ✖  Fatal error: {exc}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        try:
            spi.close()
        except Exception:
            pass
        GPIO.cleanup()
        print("SPI and GPIO cleaned up.")


if __name__ == "__main__":
    main()
