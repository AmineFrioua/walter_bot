#!/usr/bin/env python3
"""
Load Cell Test Script — Walter Robot
=====================================
Chip   : Texas Instruments ADS131M04
         4-channel, 24-bit, simultaneously-sampling delta-sigma ADC
         Datasheet: https://www.ti.com/lit/ds/symlink/ads131m04.pdf

SPI frame (18 bytes = 6 words × 3 bytes, 24-bit word size, MSB-first):
  Word 0  bytes  0- 2 : RESPONSE / STATUS  ← NOT data; echoes status + cmd ACK
  Word 1  bytes  3- 5 : CH0 data           (AIN0P / AIN0N)
  Word 2  bytes  6- 8 : CH1 data           (AIN1P / AIN1N)
  Word 3  bytes  9-11 : CH2 data           (AIN2P / AIN2N)
  Word 4  bytes 12-14 : CH3 data           (AIN3P / AIN3N)
  Word 5  bytes 15-17 : CRC-CCITT output   (16-bit MSB-aligned, byte 17 = 0x00)

STATUS word bits (upper 16 bits of Word 0, i.e. bytes 0-1):
  Bit 15  LOCK     — 1 = registers locked (default after reset; read-only is fine)
  Bit 14  RESYNC   — 1 = resync in progress
  Bit 13  REGMAP   — 1 = register-map CRC changed
  Bit 12  CRC_ERR  — 1 = input frame had CRC error
  Bit 10  RESET    — 1 = device just completed a reset
  Bits 9-8 WLENGTH — current word-length setting (0x00 = 24-bit)
  Bits 3-0 DRDY3-0 — per-channel data-ready flags

Special response codes (appear in Word 0 of the frame following the command):
  0xFF24  RSP_RESET_OK — device acknowledged a RESET and is ready (ADS131M04)
  0x0011  RSP_RESET_NOK — reset command received but failed

Pin wiring (BCM numbering):
  GPIO 27 → /RESET  (active-low)
  GPIO 22 → DRDY    (input — LOW = fresh sample ready)
  GPIO 8  → /CS     (manual chip-select, tied to SPI CE0 pad)

Usage:
  python3 load_cell_test.py                   # live read of CH1 (default)
  python3 load_cell_test.py --channel 0       # read CH0 instead
  python3 load_cell_test.py --all             # show all 4 channels per line
  python3 load_cell_test.py --debug           # + raw hex bytes and STATUS decode
  python3 load_cell_test.py --diagnose        # full connection diagnostic then exit
  python3 load_cell_test.py --calibrate       # interactive tare + scale wizard
  python3 load_cell_test.py --samples 200     # capture N samples then exit
"""

import spidev
import time
import sys
import argparse
import statistics
import RPi.GPIO as GPIO

# ── Configuration ──────────────────────────────────────────────────────────────

RESET_PIN       = 27
DRDY_PIN        = 22
CS_PIN          = 8

SPI_BUS         = 0
SPI_DEVICE      = 0
SPI_SPEED_HZ    = 1_000_000    # 1 MHz — conservative; chip supports up to 25 MHz
SPI_MODE        = 0b01         # Mode 1 (CPOL=0, CPHA=1) — required by ADS131M04

# ADS131M04 frame layout
FRAME_BYTES     = 18           # 6 words × 3 bytes
NUM_DATA_CH     = 4            # CH0–CH3 (words 1–4)
DEFAULT_CH      = 1            # CH1 = AIN1P/AIN1N (matches original wiring)

VREF_V          = 1.2          # Internal reference voltage
PGA_GAIN        = 1            # PGA gain (1, 2, 4, 8, 16, 32, 64, 128)
ADC_BITS        = 24

DRDY_TIMEOUT_MS = 1000
SAMPLE_INTERVAL = 0.2
WINDOW_SIZE     = 10

# Calibration constants — update after running --calibrate
TARE_OFFSET_CODE = 0           # Raw code with no load
SCALE_FACTOR_KG  = 1e-4        # kg per raw code (rough; calibrate!)

# ── ADS131M04 command words (16-bit, sent MSB-aligned in a 24-bit word) ────────

CMD_NULL        = 0x0000       # No-op: just read current data
CMD_RESET       = 0x0011       # Software reset
CMD_STANDBY     = 0x0022
CMD_WAKEUP      = 0x0033
CMD_LOCK        = 0x0555
CMD_UNLOCK      = 0x0655       # Unlock registers (required before WREG)

# RREG: read register at addr, 1 register = 0xA000 | (addr << 7) | 0
def _rreg(addr: int) -> int:
    return 0xA000 | ((addr & 0x1F) << 7)

REG_ID          = 0x00
REG_STATUS      = 0x01
REG_MODE        = 0x02
REG_CLOCK       = 0x03
REG_GAIN        = 0x04

RSP_RESET_OK    = 0xFF24       # ADS131M04 acknowledges reset (0xFF20 | CHANCNT=4)
RSP_RESET_NOK   = 0x0011

# STATUS word bit masks
STATUS_LOCK     = 0x8000
STATUS_RESYNC   = 0x4000
STATUS_REGMAP   = 0x2000
STATUS_CRC_ERR  = 0x1000
STATUS_RESET    = 0x0400
STATUS_WLENGTH  = 0x0300       # 0x00 = 24-bit words (default)
STATUS_DRDY3    = 0x0008
STATUS_DRDY2    = 0x0004
STATUS_DRDY1    = 0x0002
STATUS_DRDY0    = 0x0001

# ── Low-level helpers ──────────────────────────────────────────────────────────

def decode_24bit(b0: int, b1: int, b2: int) -> int:
    """Three bytes → signed 24-bit two's-complement integer."""
    raw = (b0 << 16) | (b1 << 8) | b2
    return raw - 0x1000000 if (raw & 0x800000) else raw


def code_to_mv(code: int) -> float:
    """Raw ADC code → millivolts (using VREF and PGA gain)."""
    # Full-scale = ±VREF/GAIN, represented by ±2^23 codes
    return code * (VREF_V / PGA_GAIN) / (1 << (ADC_BITS - 1)) * 1000.0


def code_to_kg(code: int) -> float:
    return (code - TARE_OFFSET_CODE) * SCALE_FACTOR_KG


def crc_ccitt(data: bytes) -> int:
    """CRC-CCITT (poly=0x1021, init=0xFFFF) over a byte sequence.
    ADS131M04 applies this to the first 15 bytes of each output frame."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def parse_status(frame: list[int]) -> int:
    """Extract 16-bit STATUS value from Word 0 (bytes 0-1) of the received frame."""
    return (frame[0] << 8) | frame[1]


def parse_channel(frame: list[int], ch: int) -> int:
    """Extract signed 24-bit code for data channel ch (0–3).
    Channel words start at Word 1 (byte offset 3), so CH0=bytes 3-5, CH1=bytes 6-8…"""
    off = (ch + 1) * 3
    return decode_24bit(frame[off], frame[off + 1], frame[off + 2])


def parse_all_channels(frame: list[int]) -> list[int]:
    return [parse_channel(frame, ch) for ch in range(NUM_DATA_CH)]


def parse_rx_crc(frame: list[int]) -> int:
    """Extract 16-bit CRC from Word 5 (bytes 15-16; byte 17 is zero-padding)."""
    return (frame[15] << 8) | frame[16]


def check_crc(frame: list[int]) -> bool:
    """Verify the chip's output CRC against locally computed value."""
    return crc_ccitt(bytes(frame[:15])) == parse_rx_crc(frame)


def bar(value: float, lo: float, hi: float, width: int = 20) -> str:
    frac = max(0.0, min(1.0, (value - lo) / (hi - lo)))
    n = int(frac * width)
    return "[" + "█" * n + "░" * (width - n) + "]"


# ── Hardware setup ─────────────────────────────────────────────────────────────

def setup_gpio() -> None:
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(RESET_PIN, GPIO.OUT, initial=GPIO.HIGH)
    GPIO.setup(DRDY_PIN,  GPIO.IN)
    GPIO.setup(CS_PIN,    GPIO.OUT, initial=GPIO.HIGH)
    print(f"  GPIO OK  (RESET=GPIO{RESET_PIN}, DRDY=GPIO{DRDY_PIN}, CS=GPIO{CS_PIN})")


def setup_spi() -> spidev.SpiDev:
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, SPI_DEVICE)
    spi.max_speed_hz = SPI_SPEED_HZ
    spi.mode         = SPI_MODE
    spi.no_cs        = True    # CRITICAL — we drive CS manually
    print(f"  SPI  OK  (bus={SPI_BUS}.{SPI_DEVICE}, {SPI_SPEED_HZ//1000} kHz, mode={SPI_MODE:#04b})")
    return spi


def hardware_reset() -> None:
    print("  ADC reset…", end=" ", flush=True)
    GPIO.output(RESET_PIN, GPIO.LOW)
    time.sleep(0.1)
    GPIO.output(RESET_PIN, GPIO.HIGH)
    time.sleep(0.5)
    print("done")


def wait_drdy(timeout_ms: int = DRDY_TIMEOUT_MS) -> bool:
    deadline = time.monotonic() + timeout_ms / 1000.0
    while GPIO.input(DRDY_PIN) == GPIO.HIGH:
        if time.monotonic() > deadline:
            return False
        time.sleep(0.0005)
    return True


def send_frame(spi: spidev.SpiDev, cmd: int = CMD_NULL) -> list[int]:
    """Send one 18-byte SPI frame with cmd in Word 0; return the received frame.
    cmd is a 16-bit command word; it is sent MSB-aligned in a 24-bit word slot."""
    tx = [(cmd >> 8) & 0xFF, cmd & 0xFF, 0x00] + [0x00] * 15
    GPIO.output(CS_PIN, GPIO.LOW)
    rx = spi.xfer2(tx)
    GPIO.output(CS_PIN, GPIO.HIGH)
    return rx


# ── Diagnostic helpers ─────────────────────────────────────────────────────────

def _frame_signature(frame: list[int]) -> str:
    """Classify a raw frame for quick wiring diagnosis."""
    if all(b == 0x00 for b in frame):
        return "ALL_ZERO"
    if all(b == 0xFF for b in frame):
        return "ALL_FF"
    return "DATA"


def _describe_status(status: int) -> list[str]:
    flags = []
    if status & STATUS_LOCK:    flags.append("LOCK(registers locked)")
    if status & STATUS_RESYNC:  flags.append("RESYNC")
    if status & STATUS_REGMAP:  flags.append("REGMAP(register CRC changed)")
    if status & STATUS_CRC_ERR: flags.append("CRC_ERR(bad input frame)")
    if status & STATUS_RESET:   flags.append("RESET(just reset)")
    wlen = (status & STATUS_WLENGTH) >> 8
    flags.append(f"WLENGTH={wlen}({'24-bit' if wlen == 0 else '16-bit' if wlen == 1 else '32-bit'})")
    drdy = [i for i in range(4) if status & (1 << i)]
    flags.append(f"DRDY={drdy if drdy else 'none'}")
    return flags


def run_diagnose(spi: spidev.SpiDev) -> bool:
    """Full connection diagnostic. Returns True if chip is healthy."""
    print(f"\n{'═'*60}")
    print("  ADS131M04 CONNECTION DIAGNOSTIC")
    print(f"{'═'*60}\n")
    ok = True

    # ── Step 1: DRDY after hardware reset ──────────────────────────────────────
    print("  [1/5] Checking DRDY asserts after hardware reset…")
    hardware_reset()
    if not wait_drdy(timeout_ms=2000):
        print("  ✖  DRDY never went LOW within 2 s.")
        print("     Likely causes:")
        print("       • ADC not powered (check 3.3 V supply on VDD/IOVDD)")
        print("       • DRDY wired to wrong GPIO (expected GPIO 22)")
        print("       • /RESET stuck low (check GPIO 27 wiring)")
        print("       • Chip in permanent standby — try sending WAKEUP")
        return False
    print("  ✔  DRDY asserted\n")

    # ── Step 2: Frame bus check ────────────────────────────────────────────────
    print("  [2/5] Reading first frame (reset-response frame)…")
    frame = send_frame(spi, CMD_NULL)
    sig = _frame_signature(frame)
    hex_str = " ".join(f"{b:02X}" for b in frame)
    print(f"       raw: {hex_str}")

    if sig == "ALL_ZERO":
        print("  ✖  All bytes are 0x00 — MISO is stuck LOW.")
        print("     Likely causes:")
        print("       • SPI MISO line not connected to ADC DOUT")
        print("       • ADC powered but CS not wiring correctly (chip not selected)")
        print("       • Chip output driver disabled")
        return False

    if sig == "ALL_FF":
        print("  ✖  All bytes are 0xFF — MISO is floating HIGH.")
        print("     Likely causes:")
        print("       • No chip on the SPI bus at all (open-circuit MISO)")
        print("       • Wrong SPI device node (/dev/spidev0.0 vs spidev0.1)")
        return False

    print("  ✔  Non-trivial data received\n")

    # ── Step 3: Reset response code ───────────────────────────────────────────
    print("  [3/5] Checking reset-response word…")
    response_word = parse_status(frame)   # first 2 bytes of frame = response word
    print(f"       Response word: 0x{response_word:04X}  (expected 0x{RSP_RESET_OK:04X})")

    if response_word == RSP_RESET_OK:
        print("  ✔  Got RSP_RESET_OK (0xFF24) — ADS131M04 confirmed present and ready\n")
    else:
        print(f"  ⚠  Unexpected response word 0x{response_word:04X}")
        if response_word == RSP_RESET_NOK:
            print("     Got RSP_RESET_NOK — chip received RESET but failed internal checks")
        elif response_word & 0xFF00 == 0xFF00:
            chancnt = response_word & 0x00FF
            print(f"     Looks like a different ADS131M0x chip ({chancnt} channels)")
            print("     Update RSP_RESET_OK and NUM_DATA_CH in this script's config.")
        else:
            print("     Wrong SPI mode? Try SPI_MODE = 0b00 or 0b11 and rerun.")
        ok = False
        print()

    # ── Step 4: STATUS word decode ────────────────────────────────────────────
    print("  [4/5] Decoding STATUS word from a live NULL frame…")
    # Read one more frame so we get real STATUS (not the reset-response code)
    if wait_drdy(timeout_ms=1000):
        frame2 = send_frame(spi, CMD_NULL)
        status = parse_status(frame2)
        flags  = _describe_status(status)
        print(f"       STATUS = 0x{status:04X}")
        for f in flags:
            is_problem = "ERR" in f or ("RESYNC" in f) or ("REGMAP" in f and "changed" in f)
            marker = "  ⚠" if is_problem else "  ✔"
            print(f"  {marker}  {f}")
        if status & STATUS_LOCK:
            print("       (LOCK is normal after reset — read-only access still works)")
        if status & STATUS_CRC_ERR:
            print("  ✖  CRC_ERR: the chip flagged a bad input frame from us.")
            print("       Reduce SPI_SPEED_HZ or check for signal integrity issues.")
            ok = False
    else:
        print("  ✖  DRDY timed out waiting for STATUS frame")
        ok = False
    print()

    # ── Step 5: CRC validation ────────────────────────────────────────────────
    print("  [5/5] CRC validation (3 consecutive frames)…")
    crc_fails = 0
    for i in range(3):
        if not wait_drdy(timeout_ms=1000):
            print(f"  ✖  DRDY timeout on frame {i+1}")
            ok = False
            break
        fr = send_frame(spi, CMD_NULL)
        computed = crc_ccitt(bytes(fr[:15]))
        received = parse_rx_crc(fr)
        passed   = computed == received
        if not passed:
            crc_fails += 1
        print(f"       Frame {i+1}: computed=0x{computed:04X}  received=0x{received:04X}  {'✔' if passed else '✖ MISMATCH'}")

    if crc_fails == 0:
        print("  ✔  All CRC checks passed\n")
    else:
        print(f"  ✖  {crc_fails}/3 CRC mismatches — SPI noise or wrong speed.")
        print("       Try lowering SPI_SPEED_HZ (e.g. 500000) and rerun.\n")
        ok = False

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"{'═'*60}")
    if ok:
        print("  ✔  DIAGNOSTIC PASSED — ADS131M04 connected and healthy")
    else:
        print("  ✖  DIAGNOSTIC FOUND ISSUES — see messages above")
    print(f"{'═'*60}\n")
    return ok


# ── Live read mode ─────────────────────────────────────────────────────────────

def run_live(spi: spidev.SpiDev, channel: int, show_all: bool, debug: bool,
             max_samples: int) -> None:

    window:  list[int] = []
    n_ok    = 0
    n_err   = 0
    n_crc   = 0
    t_start = time.monotonic()

    print(f"\n{'─'*60}")
    print(f"  Chip     : ADS131M04 (4-channel 24-bit ADC)")
    print(f"  Channel  : CH{channel}  (AIN{channel}P / AIN{channel}N)")
    print(f"  VREF     : {VREF_V} V   PGA gain: {PGA_GAIN}×")
    print(f"  Tare     : {TARE_OFFSET_CODE}   Scale: {SCALE_FACTOR_KG} kg/code")
    if max_samples:
        print(f"  Capturing: {max_samples} samples then exit")
    print(f"{'─'*60}")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            if not wait_drdy():
                n_err += 1
                print(f"  ⚠  DRDY timeout #{n_err}")
                if n_err >= 5:
                    print("  ✖  Too many timeouts — run --diagnose to check wiring.")
                    break
                continue

            frame  = send_frame(spi, CMD_NULL)
            status = parse_status(frame)
            codes  = parse_all_channels(frame)
            code   = codes[channel]
            crc_ok = check_crc(frame)

            if not crc_ok:
                n_crc += 1

            window.append(code)
            if len(window) > WINDOW_SIZE:
                window.pop(0)
            avg  = statistics.mean(window)
            n_ok += 1

            mv     = code_to_mv(code)
            avg_mv = code_to_mv(avg)
            noise  = code_to_mv(statistics.stdev(window)) if len(window) > 1 else 0.0
            kg     = code_to_kg(avg)

            if debug:
                hex_str = " ".join(f"{b:02X}" for b in frame)
                flags   = ", ".join(_describe_status(status))
                crc_tag = "✔" if crc_ok else "✖CRC"
                print(f"  [{n_ok:5d}] raw  : {hex_str}  {crc_tag}")
                print(f"         status: 0x{status:04X}  ({flags})")
                for i, c in enumerate(codes):
                    tag = " ← target" if i == channel else ""
                    print(f"         CH{i}: {c:+10d}  {code_to_mv(c):+9.4f} mV{tag}")
            else:
                crc_warn = " ⚠CRC" if not crc_ok else ""
                lock_warn = " LOCK" if (status & STATUS_LOCK) else ""
                vol_bar = bar(mv, -1200, 1200)
                print(
                    f"  [{n_ok:5d}]  CH{channel}: {code:+10d}  {mv:+9.4f} mV  {vol_bar}"
                    f"  avg: {avg_mv:+9.4f} mV  {kg:+7.4f} kg  noise: {noise:.4f} mV"
                    f"{crc_warn}{lock_warn}"
                )
                if show_all:
                    all_str = "  ".join(
                        f"CH{i}={code_to_mv(c):+8.3f}mV" for i, c in enumerate(codes)
                    )
                    print(f"           all: {all_str}")

            if max_samples and n_ok >= max_samples:
                elapsed = time.monotonic() - t_start
                print(f"\n  Captured {n_ok} samples in {elapsed:.1f} s  "
                      f"({n_ok/elapsed:.1f} Hz)  CRC errors: {n_crc}")
                break

            time.sleep(SAMPLE_INTERVAL)

    except KeyboardInterrupt:
        elapsed = time.monotonic() - t_start
        print(f"\n\n  Stopped.  {n_ok} samples  {elapsed:.1f} s  "
              f"{n_ok/max(elapsed,0.001):.1f} Hz  CRC errors: {n_crc}")


# ── Calibration wizard ─────────────────────────────────────────────────────────

def run_calibrate(spi: spidev.SpiDev, channel: int) -> None:

    def stable_avg(n: int = 30) -> float:
        samples = []
        for _ in range(n):
            if not wait_drdy():
                print("  ⚠  DRDY timeout — skipping sample")
                continue
            frame = send_frame(spi, CMD_NULL)
            samples.append(parse_channel(frame, channel))
            time.sleep(0.05)
        if not samples:
            raise RuntimeError("No samples — check ADC wiring.")
        return statistics.mean(samples)

    print(f"\n{'─'*60}")
    print(f"  CALIBRATION WIZARD — CH{channel}  (AIN{channel}P / AIN{channel}N)")
    print(f"{'─'*60}\n")

    input("  Step 1/2 — Remove ALL weight from the load cell, then press Enter…")
    print("  Averaging 30 tare samples…")
    tare = stable_avg(30)
    print(f"  Tare  code: {tare:+.1f}  ({code_to_mv(tare):+.4f} mV)\n")

    raw = input("  Step 2/2 — Place a KNOWN weight. Enter kg (e.g. 1.0): ").strip()
    try:
        known_kg = float(raw)
    except ValueError:
        print("  ✖  Not a number. Aborting.")
        return

    print(f"  Averaging 30 loaded samples ({known_kg} kg)…")
    loaded = stable_avg(30)
    print(f"  Loaded code: {loaded:+.1f}  ({code_to_mv(loaded):+.4f} mV)")

    delta = loaded - tare
    if abs(delta) < 10:
        print(f"\n  ⚠  Delta only {delta:.1f} codes — load cell may not be responding.")
        print("     Check wiring, PGA gain, and that the right channel is selected.")
        return

    scale = known_kg / delta
    sens  = (code_to_mv(loaded) - code_to_mv(tare)) / known_kg

    print(f"\n{'─'*60}")
    print("  RESULTS — paste these into the config block at the top of this file")
    print(f"{'─'*60}")
    print(f"  TARE_OFFSET_CODE = {tare:.0f}")
    print(f"  SCALE_FACTOR_KG  = {scale:.8f}   # {scale*1000:.6f} g/code")
    print(f"  Sensitivity      : {sens:.4f} mV/kg")
    print(f"{'─'*60}\n")


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="ADS131M04 load cell test for Walter robot"
    )
    parser.add_argument("--channel",   type=int,  default=DEFAULT_CH,
                        help=f"Data channel to read (0–{NUM_DATA_CH-1}, default {DEFAULT_CH})")
    parser.add_argument("--all",       action="store_true",
                        help="Print all 4 channels on every line")
    parser.add_argument("--debug",     action="store_true",
                        help="Dump raw frame bytes, STATUS decode, and all channel codes")
    parser.add_argument("--diagnose",  action="store_true",
                        help="Run full connection diagnostic then exit")
    parser.add_argument("--calibrate", action="store_true",
                        help="Interactive tare + known-weight calibration wizard")
    parser.add_argument("--samples",   type=int,  default=0,
                        help="Capture N samples then exit (0 = run forever)")
    args = parser.parse_args()

    if not (0 <= args.channel < NUM_DATA_CH):
        print(f"Error: --channel must be 0–{NUM_DATA_CH-1}")
        sys.exit(1)

    print("\nWalter — ADS131M04 Load Cell Test")
    print("=" * 60)
    print("Frame layout: [RESPONSE(3)] [CH0(3)] [CH1(3)] [CH2(3)] [CH3(3)] [CRC(3)]")
    print("SPI mode 1 (CPOL=0, CPHA=1) | 24-bit words | VREF=1.2V")
    print("=" * 60)
    print("Initialising hardware…")

    try:
        setup_gpio()
        spi = setup_spi()
        hardware_reset()

        if args.diagnose:
            run_diagnose(spi)
            return

        # ── Quick boot check ───────────────────────────────────────────────────
        print("\nBoot check — reading first frame after reset…")
        if not wait_drdy(timeout_ms=2000):
            print("  ✖  DRDY did not assert within 2 s after reset.")
            print("     Run  python3 load_cell_test.py --diagnose  for full diagnosis.")
            sys.exit(1)

        frame = send_frame(spi, CMD_NULL)
        rsp   = parse_status(frame)
        sig   = _frame_signature(frame)
        hex_str = " ".join(f"{b:02X}" for b in frame)
        print(f"  Raw frame : {hex_str}")

        if sig == "ALL_ZERO":
            print("  ✖  All zeros — MISO stuck low. Run --diagnose.")
            sys.exit(1)
        if sig == "ALL_FF":
            print("  ✖  All 0xFF — MISO floating. Run --diagnose.")
            sys.exit(1)

        if rsp == RSP_RESET_OK:
            print(f"  ✔  RSP_RESET_OK (0x{rsp:04X}) — ADS131M04 present and ready")
        else:
            print(f"  ⚠  Response word 0x{rsp:04X} (expected 0x{RSP_RESET_OK:04X})")
            print("     Chip may be a different model or SPI mode is wrong.")
            print("     Continuing anyway — run --diagnose for full analysis.")

        crc_ok = check_crc(frame)
        print(f"  CRC check : {'✔ passed' if crc_ok else '✖ FAILED — check SPI speed/wiring'}")

        # Show all 4 channel codes from the first frame
        codes = parse_all_channels(frame)
        for i, c in enumerate(codes):
            tag = " ← will read this" if i == args.channel else ""
            print(f"  CH{i} = {c:+10d}  ({code_to_mv(c):+9.4f} mV){tag}")
        print()

        if args.calibrate:
            run_calibrate(spi, args.channel)
        else:
            run_live(spi, args.channel, args.all, args.debug, args.samples)

    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"\n  ✖  Fatal: {exc}")
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
