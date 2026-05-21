#!/usr/bin/env python3
"""
ibat_logger.py — Standalone BQ25820 IBAT logger.

Usage:
    python3 ibat_logger.py <name>

    <name>  Output filename (with or without .csv).
            e.g.  python3 ibat_logger.py drive_test_01

Logs up to 100 readings at 1 Hz to <name>.csv then exits.
Stop early at any time with Ctrl+C — partial data is still saved.

CSV columns:
    t_s       elapsed time in seconds
    ibat_ma   battery current in mA  (+ve = charging, -ve = discharging)
"""

import sys
import csv
import time
import signal

try:
    from smbus2 import SMBus
except ImportError:
    print("Error: smbus2 not installed.  Run:  pip3 install smbus2")
    sys.exit(1)

# ── BQ25820 constants ─────────────────────────────────────────────────────────
BQ_ADDR      = 0x6B
ADC_CTRL_REG = 0x2B   # bit7 = ADC_EN, bit6 = ADC_RATE (0 = continuous)
IBAT_L_REG   = 0x2F   # IBAT low  byte [7:0]
IBAT_H_REG   = 0x30   # IBAT high byte [15:8]
# 16-bit signed 2s complement, 2 mA / LSB

MAX_POINTS   = 100
INTERVAL_S   = 1.0


def _enable_adc(bus):
    ctrl = bus.read_byte_data(BQ_ADDR, ADC_CTRL_REG)
    if not (ctrl & 0x80):
        bus.write_byte_data(BQ_ADDR, ADC_CTRL_REG, (ctrl & 0x3F) | 0x80)


def _read_ibat_ma(bus):
    lo  = bus.read_byte_data(BQ_ADDR, IBAT_L_REG)
    hi  = bus.read_byte_data(BQ_ADDR, IBAT_H_REG)
    raw = (hi << 8) | lo
    if raw > 32767:
        raw -= 65536
    return raw * 2   # 2 mA / LSB


def _save_csv(filename, rows):
    with open(filename, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=['t_s', 'ibat_ma'])
        w.writeheader()
        w.writerows(rows)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    name     = sys.argv[1]
    filename = name if name.endswith('.csv') else name + '.csv'
    rows     = []
    _stop    = False

    def _on_sigint(sig, frame):
        nonlocal _stop
        _stop = True

    signal.signal(signal.SIGINT,  _on_sigint)
    signal.signal(signal.SIGTERM, _on_sigint)

    print(f"BQ25820 IBAT logger → {filename}")
    print(f"Max {MAX_POINTS} points at 1 Hz  |  Ctrl+C to stop early\n")
    print(f"{'  t(s)':>7}  {'IBAT (mA)':>10}  state")
    print("─" * 35)

    try:
        with SMBus(1) as bus:
            _enable_adc(bus)
            t0 = time.monotonic()

            for i in range(MAX_POINTS):
                if _stop:
                    break

                t_s  = round(time.monotonic() - t0, 2)
                ibat = _read_ibat_ma(bus)
                rows.append({'t_s': t_s, 'ibat_ma': ibat})

                if   ibat >  50: state = 'charging'
                elif ibat < -50: state = 'discharging'
                else:            state = 'idle'

                sign = '+' if ibat >= 0 else ''
                print(f"  {t_s:>5.1f}s  {sign}{ibat:>8.0f} mA  {state}")

                # Drift-free sleep to the next 1 s boundary
                deadline = t0 + (i + 1) * INTERVAL_S
                wait     = deadline - time.monotonic()
                if wait > 0:
                    time.sleep(wait)

    except OSError as e:
        print(f"\nI2C error: {e}")
        print("  Check that the BQ25820 is wired and i2cdetect -y 1 shows 0x6b")
        if rows:
            _save_csv(filename, rows)
            print(f"  Partial data ({len(rows)} pts) saved to {filename}")
        sys.exit(1)

    _save_csv(filename, rows)
    print(f"\n✓ Saved {len(rows)} points → {filename}")


if __name__ == '__main__':
    main()
