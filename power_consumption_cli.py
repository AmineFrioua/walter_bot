#!/usr/bin/env python3
"""
pi_power_monitor.py
═══════════════════════════════════════════════════════════════════
Real-time power & system health monitor for Raspberry Pi robot
controllers. Designed for setups with LiDAR, screens, and other
USB peripherals where power stability is critical.

Usage:
    python3 pi_power_monitor.py               # default refresh 1s
    python3 pi_power_monitor.py --rate 0.5    # refresh every 500ms
    python3 pi_power_monitor.py --log power.csv  # also log to CSV

Requirements:
    pip3 install rich psutil
═══════════════════════════════════════════════════════════════════
"""

import os
import sys
import time
import argparse
import subprocess
import csv
import signal
from datetime import datetime
from collections import deque
from pathlib import Path

# ── Dependency check ────────────────────────────────────────────
missing = []
try:
    import psutil
except ImportError:
    missing.append("psutil")

try:
    from rich.console import Console
    from rich.table import Table
    from rich.layout import Layout
    from rich.panel import Panel
    from rich.live import Live
    from rich.text import Text
    from rich.columns import Columns
    from rich.rule import Rule
    from rich import box
    from rich.padding import Padding
    from rich.align import Align
except ImportError:
    missing.append("rich")

if missing:
    print(f"[ERROR] Missing packages: {', '.join(missing)}")
    print(f"  Install with: pip3 install {' '.join(missing)}")
    sys.exit(1)

# ── Constants & thresholds ──────────────────────────────────────
VERSION = "1.1.0"

THRESHOLDS = {
    "temp_warn":     65.0,   # °C — yellow warning
    "temp_crit":     80.0,   # °C — red critical
    "volt_min_warn": 1.15,   # V  — core voltage yellow
    "volt_min_crit": 1.10,   # V  — core voltage red
    "cpu_warn":      75.0,   # %  — CPU usage yellow
    "cpu_crit":      95.0,   # %  — CPU usage red
    "mem_warn":      80.0,   # %  — RAM usage yellow
    "mem_crit":      92.0,   # %  — RAM usage red
}

# Pi 4 estimated power draw by component (Watts)
POWER_ESTIMATE = {
    "base_idle":  2.7,   # Pi 4 baseline with nothing happening
    "per_cpu_w":  0.4,   # extra watts per 100% CPU core load
    "per_usb_w":  0.0,   # filled from USB device data
}

THROTTLE_FLAGS = {
    0:  "Under-voltage detected",
    1:  "Arm freq capped",
    2:  "Currently throttled",
    3:  "Soft temp limit",
    16: "Under-voltage occurred",
    17: "Arm freq cap occurred",
    18: "Throttling occurred",
    19: "Soft temp limit occurred",
}

HISTORY_SIZE = 60   # data points kept for sparkline
MAX_ALERTS   = 12   # alert log lines to show

console = Console()


# ═══════════════════════════════════════════════════════════════
#  DATA COLLECTION
# ═══════════════════════════════════════════════════════════════

def _run(cmd: list[str]) -> str:
    """Run a shell command and return stdout, or '' on failure."""
    try:
        return subprocess.check_output(cmd, stderr=subprocess.DEVNULL,
                                       timeout=1).decode().strip()
    except Exception:
        return ""


def get_vcgencmd_volt(domain: str) -> float | None:
    """Return voltage for a vcgencmd domain (e.g. 'core', 'sdram_c')."""
    out = _run(["vcgencmd", "measure_volts", domain])
    # output: "volt=1.2125V"
    if out.startswith("volt="):
        try:
            return float(out[5:].rstrip("V"))
        except ValueError:
            pass
    return None


def get_voltages() -> dict:
    domains = {"core": None, "sdram_c": None, "sdram_i": None, "sdram_p": None}
    for domain in domains:
        domains[domain] = get_vcgencmd_volt(domain)
    return domains


def get_throttle_status() -> dict:
    """
    Returns throttle bitmask and human-readable active flags.
    vcgencmd get_throttled → "throttled=0x50000"
    """
    out = _run(["vcgencmd", "get_throttled"])
    result = {"raw": 0, "active": [], "historical": [], "healthy": True}
    if out.startswith("throttled="):
        try:
            val = int(out[10:], 16)
            result["raw"] = val
            for bit, desc in THROTTLE_FLAGS.items():
                if val & (1 << bit):
                    if bit <= 3:
                        result["active"].append(desc)
                        result["healthy"] = False
                    else:
                        result["historical"].append(desc)
        except ValueError:
            pass
    return result


def get_temperature() -> float | None:
    """CPU temperature in °C."""
    # Try psutil first (most reliable)
    try:
        temps = psutil.sensors_temperatures()
        for key in ("cpu_thermal", "coretemp", "soc_thermal"):
            if key in temps and temps[key]:
                return temps[key][0].current
    except (AttributeError, Exception):
        pass
    # Fallback: vcgencmd
    out = _run(["vcgencmd", "measure_temp"])
    if out.startswith("temp="):
        try:
            return float(out[5:].rstrip("'C"))
        except ValueError:
            pass
    # Fallback: /sys
    for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
        try:
            return int(path.read_text().strip()) / 1000.0
        except Exception:
            pass
    return None


def _sysfs_max_freq_mhz() -> int:
    """Read the hardware-defined max CPU frequency from sysfs (Hz → MHz)."""
    try:
        hz = int(Path(
            "/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq"
        ).read_text().strip())
        return hz // 1000
    except Exception:
        return 0


def get_cpu_freq_info() -> tuple[int, int]:
    """
    Return (current_mhz, max_mhz) from a SINGLE psutil call so both
    values come from the same snapshot and avoid governor race conditions.
    Falls back to vcgencmd / sysfs if psutil is unavailable.
    """
    try:
        freq = psutil.cpu_freq()
        if freq:
            current = int(freq.current)
            # psutil sometimes reports max=0 on Pi — fall back to sysfs
            maximum = int(freq.max) if freq.max else _sysfs_max_freq_mhz()
            if not maximum:
                maximum = 1500   # Pi 4 stock max (not 1800 which is overclocked)
            return current, maximum
    except Exception:
        pass

    # vcgencmd fallback for current
    out = _run(["vcgencmd", "measure_clock", "arm"])
    current = 0
    if "=" in out:
        try:
            current = int(out.split("=")[1]) // 1_000_000
        except ValueError:
            pass

    maximum = _sysfs_max_freq_mhz() or 1500
    return current, maximum


def get_usb_devices() -> list[dict]:
    """
    Walk /sys/bus/usb/devices and collect device info.
    Returns list of {path, vendor, product, manufacturer, max_ma, product_name}
    """
    devices = []
    usb_root = Path("/sys/bus/usb/devices")
    if not usb_root.exists():
        return devices

    def read_file(p: Path) -> str:
        try:
            return p.read_text().strip()
        except Exception:
            return ""

    for dev_path in sorted(usb_root.iterdir()):
        # Skip interface nodes (have a colon) and entries without idVendor
        name = dev_path.name
        if ":" in name or not (dev_path / "idVendor").exists():
            continue

        vendor_id = read_file(dev_path / "idVendor")

        # Skip Linux Foundation root hubs — the host controllers built into
        # the Pi's SoC; always present regardless of what's plugged in
        if vendor_id == "1d6b":
            continue

        # Skip USB hubs (class 09) — this catches the Pi 4's internal VL805
        # hub chip which shows up even with zero external devices connected
        if read_file(dev_path / "bDeviceClass") == "09":
            continue

        # Skip devices the kernel has disabled / not authorised
        if read_file(dev_path / "authorized") == "0":
            continue

        max_power_raw = read_file(dev_path / "bMaxPower")   # e.g. "500mA" or "250"
        if not max_power_raw:
            continue

        # Parse mA value
        try:
            if "mA" in max_power_raw:
                max_ma = int(max_power_raw.replace("mA", "").strip())
            else:
                # Value is in units of 2mA (USB 2) or 8mA (USB 3)
                speed = read_file(dev_path / "speed")
                unit = 8 if speed in ("5000", "10000") else 2
                max_ma = int(max_power_raw) * unit
        except ValueError:
            continue

        manufacturer  = read_file(dev_path / "manufacturer") or "Unknown"
        product       = read_file(dev_path / "product")      or "USB Device"
        product_id    = read_file(dev_path / "idProduct")
        # power/runtime_status: "active", "suspended", "unsupported", etc.
        power_status  = read_file(dev_path / "power" / "runtime_status") or "unknown"

        devices.append({
            "path":         name,
            "vendor":       vendor_id,
            "product":      product_id,
            "name":         product,
            "mfr":          manufacturer,
            "max_ma":       max_ma,
            "power_status": power_status,
        })

    return devices


def get_memory_info() -> dict:
    mem  = psutil.virtual_memory()
    swap = psutil.swap_memory()
    return {
        "ram_total_gb":     mem.total     / 1024**3,
        "ram_used_gb":      mem.used      / 1024**3,
        "ram_available_gb": mem.available / 1024**3,
        "ram_free_gb":      mem.free      / 1024**3,
        # buffers & cached: Linux-only, graceful fallback
        "ram_cached_gb":    getattr(mem, "cached",  0) / 1024**3,
        "ram_buffers_gb":   getattr(mem, "buffers", 0) / 1024**3,
        "ram_pct":          mem.percent,
        "swap_total_gb":    swap.total / 1024**3,
        "swap_used_gb":     swap.used  / 1024**3,
        "swap_pct":         swap.percent,
    }


TOP_PROC_COUNT = 8   # how many processes to display

def get_top_processes(n: int = TOP_PROC_COUNT) -> list[dict]:
    """
    Return the top N processes sorted by CPU %, then RAM.
    Uses psutil.process_iter for a single efficient pass.
    """
    procs = []
    attrs = ["pid", "name", "cpu_percent", "memory_info", "status", "username"]
    for p in psutil.process_iter(attrs, ad_value=None):
        try:
            info   = p.info
            mem_mb = (info["memory_info"].rss / 1024**2
                      if info["memory_info"] else 0.0)
            procs.append({
                "pid":     info["pid"],
                "name":    (info["name"] or "?")[:22],
                "cpu_pct": info["cpu_percent"] or 0.0,
                "mem_mb":  mem_mb,
                "status":  info["status"] or "?",
                "user":    (info["username"] or "")[:10],
            })
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    procs.sort(key=lambda x: (x["cpu_pct"], x["mem_mb"]), reverse=True)
    return procs[:n]


def collect_all() -> dict:
    """Gather all metrics in one pass."""
    cpu_per_core = psutil.cpu_percent(percpu=True, interval=None)
    cpu_total    = psutil.cpu_percent(interval=None)
    return {
        "ts":           datetime.now(),
        "voltages":     get_voltages(),
        "throttle":     get_throttle_status(),
        "temp":         get_temperature(),
        "cpu_pct":      cpu_total,
        "cpu_cores":    cpu_per_core,
        "cpu_freq_mhz": (_freq := get_cpu_freq_info())[0],
        "cpu_max_mhz":  _freq[1],
        "memory":       get_memory_info(),
        "usb_devices":  get_usb_devices(),
        "processes":    get_top_processes(),
        "proc_total":   len(psutil.pids()),
    }


# ═══════════════════════════════════════════════════════════════
#  POWER ESTIMATION
# ═══════════════════════════════════════════════════════════════

def estimate_power(data: dict) -> dict:
    """Rough power estimation based on load and USB devices."""
    num_cores  = len(data["cpu_cores"])
    avg_load   = data["cpu_pct"] / 100.0
    cpu_watts  = avg_load * num_cores * POWER_ESTIMATE["per_cpu_w"]
    usb_watts  = sum(d["max_ma"] for d in data["usb_devices"]) / 1000.0 * 5.0
    total_w    = POWER_ESTIMATE["base_idle"] + cpu_watts + usb_watts
    total_amps = total_w / 5.0
    return {
        "base_w":  POWER_ESTIMATE["base_idle"],
        "cpu_w":   round(cpu_watts, 2),
        "usb_w":   round(usb_watts, 2),
        "total_w": round(total_w, 2),
        "amps":    round(total_amps, 2),
    }


# ═══════════════════════════════════════════════════════════════
#  HELPERS & UI UTILITIES
# ═══════════════════════════════════════════════════════════════

def bar(pct: float, width: int = 20,
        warn: float = 70, crit: float = 90) -> Text:
    """Render a horizontal bar with colour thresholds."""
    filled = int(pct / 100 * width)
    bar_str = "█" * filled + "░" * (width - filled)
    if pct >= crit:
        style = "bold red"
    elif pct >= warn:
        style = "bold yellow"
    else:
        style = "bold green"
    return Text(f"[{bar_str}] {pct:5.1f}%", style=style)


def volt_colour(v: float | None, domain: str = "core") -> Text:
    if v is None:
        return Text("N/A", style="dim")
    s = f"{v:.4f} V"
    # Core voltage: normal ~1.2V, concern < 1.15V
    if domain == "core":
        if v < THRESHOLDS["volt_min_crit"]:
            return Text(f"{s}  ⚠ LOW", style="bold red")
        elif v < THRESHOLDS["volt_min_warn"]:
            return Text(f"{s}  !", style="yellow")
        else:
            return Text(f"{s}  ✓", style="green")
    return Text(s, style="cyan")


def temp_colour(t: float | None) -> Text:
    if t is None:
        return Text("N/A", style="dim")
    s = f"{t:.1f}°C"
    if t >= THRESHOLDS["temp_crit"]:
        return Text(f"{s}  🔥 CRITICAL", style="bold red blink")
    elif t >= THRESHOLDS["temp_warn"]:
        return Text(f"{s}  ⚠", style="bold yellow")
    else:
        return Text(f"{s}  ✓", style="green")


def sparkline(history: deque, width: int = 30) -> str:
    """Convert a history of 0-100 floats into a unicode spark line."""
    blocks = " ▁▂▃▄▅▆▇█"
    pts = list(history)
    if not pts:
        return " " * width
    # Sample to width
    step = max(1, len(pts) // width)
    sampled = pts[-width * step::step][-width:]
    top = max(100, max(sampled))
    return "".join(blocks[min(8, int(v / top * 8))] for v in sampled)


# ═══════════════════════════════════════════════════════════════
#  PANEL BUILDERS
# ═══════════════════════════════════════════════════════════════

def panel_power_status(data: dict, power: dict) -> Panel:
    thr = data["throttle"]
    t   = Table.grid(padding=(0, 2))
    t.add_column(style="bold", min_width=20)
    t.add_column()

    if thr["healthy"] and not thr["historical"]:
        status = Text("✅  HEALTHY", style="bold green")
    elif thr["active"]:
        status = Text("🔴  ISSUE DETECTED", style="bold red blink")
    else:
        status = Text("⚠️  PAST EVENTS", style="bold yellow")

    t.add_row("Status:", status)

    if thr["active"]:
        for flag in thr["active"]:
            t.add_row("", Text(f"  ↳ {flag}", style="red"))
    else:
        t.add_row("Active flags:", Text("None", style="dim green"))

    if thr["historical"]:
        t.add_row("Since boot:", Text(", ".join(thr["historical"]),
                                      style="yellow"))

    t.add_row("", "")
    t.add_row("Est. Input:",
              Text(f"{power['total_w']:.1f} W  @ 5V → {power['amps']:.2f} A",
                   style="bold cyan"))
    t.add_row("  Base Pi:",   Text(f"{power['base_w']:.1f} W", style="dim"))
    t.add_row("  CPU load:",  Text(f"{power['cpu_w']:.1f} W", style="dim"))
    t.add_row("  USB devs:",  Text(f"{power['usb_w']:.1f} W", style="dim"))

    return Panel(t, title="[bold white]⚡ POWER STATUS", border_style="bright_blue",
                 box=box.ROUNDED)


def panel_voltages(data: dict) -> Panel:
    volts = data["voltages"]
    t = Table.grid(padding=(0, 3))
    t.add_column(style="dim", min_width=10)
    t.add_column()

    labels = {
        "core":   "Core",
        "sdram_c": "SDRAM-C",
        "sdram_i": "SDRAM-I",
        "sdram_p": "SDRAM-P",
    }
    for key, label in labels.items():
        dom = "core" if key == "core" else "other"
        t.add_row(f"{label}:", volt_colour(volts.get(key), dom))

    note = Text("\nNote: core volt varies with load.", style="dim italic")
    return Panel(
        Padding(t, (0, 0)),
        title="[bold white]🔋 VOLTAGES (vcgencmd)",
        border_style="cyan",
        box=box.ROUNDED,
    )


def panel_cpu_thermal(data: dict, temp_hist: deque, cpu_hist: deque) -> Panel:
    t = Table.grid(padding=(0, 2))
    t.add_column(style="dim", min_width=14)
    t.add_column()

    temp = data["temp"]
    t.add_row("Temperature:", temp_colour(temp))

    if temp is not None:
        # Scale bar so 100°C = full. Pass scaled warn/crit so colours
        # trigger at the right actual temperature (65°C → 65%, 80°C → 80%).
        temp_pct = min(100.0, temp)
        t.add_row("", bar(temp_pct,
                          warn=THRESHOLDS["temp_warn"],   # 65 → yellow at 65°C
                          crit=THRESHOLDS["temp_crit"]))

    t.add_row("", "")
    freq     = data["cpu_freq_mhz"]
    max_freq = data["cpu_max_mhz"]
    freq_pct = (freq / max_freq * 100) if max_freq else 0
    t.add_row("CPU Freq:",
              Text(f"{freq} MHz / {max_freq} MHz", style="cyan"))
    t.add_row("", bar(freq_pct, warn=90, crit=99))

    t.add_row("", "")
    t.add_row("CPU Total:",
              Text(f"{data['cpu_pct']:.1f}%",
                   style="bold red" if data["cpu_pct"] >= THRESHOLDS["cpu_crit"]
                   else "yellow" if data["cpu_pct"] >= THRESHOLDS["cpu_warn"]
                   else "green"))
    t.add_row("", bar(data["cpu_pct"],
                      warn=THRESHOLDS["cpu_warn"],
                      crit=THRESHOLDS["cpu_crit"]))

    # Per-core bars
    cores = data.get("cpu_cores", [])
    if cores:
        t.add_row("", "")
        t.add_row("Per core:", Text(""))
        row_parts = []
        for i, c in enumerate(cores):
            colour = ("red" if c >= THRESHOLDS["cpu_crit"]
                      else "yellow" if c >= THRESHOLDS["cpu_warn"]
                      else "green")
            row_parts.append(Text(f"C{i}:{c:4.0f}% ", style=colour))
        combined = Text().join(row_parts) if row_parts else Text("")
        t.add_row("", combined)

    # Sparklines
    t.add_row("", "")
    t.add_row("CPU history:", Text(sparkline(cpu_hist), style="green"))
    if temp is not None:
        t.add_row("Temp history:", Text(sparkline(temp_hist), style="yellow"))

    return Panel(t, title="[bold white]🖥  CPU & THERMAL",
                 border_style="magenta", box=box.ROUNDED)


def panel_memory(data: dict) -> Panel:
    mem  = data["memory"]
    t    = Table.grid(padding=(0, 2))
    t.add_column(style="dim", min_width=12)
    t.add_column()

    used_mb  = mem["ram_used_gb"]      * 1024
    avail_mb = mem["ram_available_gb"] * 1024
    total_mb = mem["ram_total_gb"]     * 1024

    # ── Total used (prominent) ──────────────────────────────────
    t.add_row("RAM Used:",
              Text(f"{used_mb:,.0f} MB  ({mem['ram_used_gb']:.2f} GB)"
                   f"  of  {mem['ram_total_gb']:.1f} GB",
                   style="bold cyan"))
    t.add_row("", bar(mem["ram_pct"],
                      warn=THRESHOLDS["mem_warn"],
                      crit=THRESHOLDS["mem_crit"]))

    # ── Breakdown ───────────────────────────────────────────────
    t.add_row("Available:",
              Text(f"{avail_mb:,.0f} MB  ({mem['ram_available_gb']:.2f} GB)",
                   style="green"))

    cached_mb  = mem.get("ram_cached_gb",  0) * 1024
    buffers_mb = mem.get("ram_buffers_gb", 0) * 1024
    if cached_mb > 0:
        t.add_row("Cached:",  Text(f"{cached_mb:,.0f} MB",  style="dim"))
    if buffers_mb > 0:
        t.add_row("Buffers:", Text(f"{buffers_mb:,.0f} MB", style="dim"))

    # ── Swap ────────────────────────────────────────────────────
    if mem["swap_total_gb"] > 0:
        t.add_row("", "")
        swap_used_mb  = mem["swap_used_gb"]  * 1024
        swap_total_mb = mem["swap_total_gb"] * 1024
        t.add_row("Swap:",
                  Text(f"{swap_used_mb:,.0f} MB / {swap_total_mb:,.0f} MB",
                       style="cyan"))
        t.add_row("", bar(mem["swap_pct"], warn=50, crit=80))

    return Panel(t, title="[bold white]💾 MEMORY",
                 border_style="blue", box=box.ROUNDED)


def panel_processes(data: dict) -> Panel:
    """Top processes ranked by CPU usage."""
    procs       = data.get("processes", [])
    proc_total  = data.get("proc_total", 0)

    t = Table(box=box.SIMPLE_HEAD, header_style="bold dim",
              show_header=True, padding=(0, 1))
    t.add_column("PID",     justify="right", style="dim", min_width=6)
    t.add_column("Process",                  min_width=22)
    t.add_column("CPU %",   justify="right", min_width=7)
    t.add_column("RAM MB",  justify="right", min_width=8)
    t.add_column("Status",                   min_width=10)
    t.add_column("User",    style="dim",     min_width=8)

    for p in procs:
        cpu_style = (
            "bold red"    if p["cpu_pct"] >= THRESHOLDS["cpu_crit"] else
            "bold yellow" if p["cpu_pct"] >= THRESHOLDS["cpu_warn"] else
            "green"
        )
        mem_style = (
            "bold red"    if p["mem_mb"] >= 512 else
            "yellow"      if p["mem_mb"] >= 200 else
            "dim"
        )
        status_style = (
            "green" if p["status"] == "running" else
            "cyan"  if p["status"] == "sleeping" else
            "dim"
        )
        t.add_row(
            str(p["pid"]),
            p["name"],
            Text(f"{p['cpu_pct']:.1f}", style=cpu_style),
            Text(f"{p['mem_mb']:.0f}",  style=mem_style),
            Text(p["status"],           style=status_style),
            p["user"],
        )

    if not procs:
        t.add_row("—", Text("No process data", style="dim"), "—", "—", "—", "—")

    return Panel(
        t,
        title=f"[bold white]⚙  TOP PROCESSES  [dim](total on system: {proc_total})",
        border_style="green",
        box=box.ROUNDED,
    )


def panel_usb(data: dict) -> Panel:
    devices  = data["usb_devices"]
    t = Table(box=box.SIMPLE_HEAD, header_style="bold dim",
              show_header=True, padding=(0, 1))
    t.add_column("Path",      style="dim",  min_width=7)
    t.add_column("Device",                  min_width=22)
    t.add_column("Max mA",    justify="right", min_width=7)
    t.add_column("Est. W",    justify="right", min_width=6)
    t.add_column("Pwr State",               min_width=10)

    total_ma = 0
    for d in devices:
        ma      = d["max_ma"]
        est_w   = ma / 1000.0 * 5.0
        total_ma += ma
        ma_col  = Text(f"{ma}", style=(
            "red"    if ma >= 900 else
            "yellow" if ma >= 500 else
            "green"
        ))
        ps      = d.get("power_status", "unknown")
        ps_style = (
            "bold green" if ps == "active"    else
            "dim"        if ps == "suspended" else
            "dim"
        )
        t.add_row(d["path"], d["name"][:28], ma_col,
                  f"{est_w:.1f}", Text(ps, style=ps_style))

    if not devices:
        t.add_row("—", Text("No USB devices found", style="dim"), "—", "—", "—")

    # Hardware limitation note
    note = Text(
        f"\n  ⚡ Declared max draw: {total_ma} mA "
        f"({total_ma / 1000.0 * 5.0:.1f} W @ 5V nominal)\n"
        "  ⚠  Actual USB output voltage cannot be read in software — the Pi\n"
        "     has no ADC on the USB rail. For real-time V/A measurement,\n"
        "     wire an INA219 sensor inline on the 5V USB line.",
        style="dim italic",
    )

    content = Table.grid()
    content.add_row(t)
    content.add_row(note)

    return Panel(
        content,
        title=f"[bold white]🔌 USB DEVICES ({len(devices)})",
        border_style="yellow",
        box=box.ROUNDED,
    )


def panel_alerts(alerts: deque) -> Panel:
    t = Table.grid(padding=(0, 1))
    if not alerts:
        t.add_row(Text("No alerts — system is stable ✓", style="dim green"))
    else:
        for ts, msg, style in list(alerts)[-MAX_ALERTS:]:
            t.add_row(Text(f"[{ts}]", style="dim"),
                      Text(msg, style=style))
    return Panel(t, title="[bold white]🔔 ALERT LOG",
                 border_style="red", box=box.ROUNDED)


# ═══════════════════════════════════════════════════════════════
#  ALERT DETECTION
# ═══════════════════════════════════════════════════════════════

def check_alerts(data: dict, last: dict, alerts: deque):
    now = datetime.now().strftime("%H:%M:%S")

    def add(msg, style="bold red"):
        alerts.append((now, msg, style))

    temp = data["temp"]
    if temp is not None:
        if temp >= THRESHOLDS["temp_crit"] and (
                last.get("temp") or 0) < THRESHOLDS["temp_crit"]:
            add(f"🔥 CRITICAL temp: {temp:.1f}°C")
        elif temp >= THRESHOLDS["temp_warn"] and (
                last.get("temp") or 0) < THRESHOLDS["temp_warn"]:
            add(f"⚠️  High temp: {temp:.1f}°C", style="yellow")
        elif temp < THRESHOLDS["temp_warn"] and (
                last.get("temp") or 0) >= THRESHOLDS["temp_warn"]:
            add(f"✅ Temp back to normal: {temp:.1f}°C", style="green")

    thr = data["throttle"]
    last_thr = last.get("throttle", {})
    if thr["active"] and not last_thr.get("active"):
        add(f"🔴 Pi throttled: {', '.join(thr['active'])}")
    elif not thr["active"] and last_thr.get("active"):
        add("✅ Throttling cleared", style="green")

    if 0 in [1 << b for b in range(4) if thr["raw"] & (1 << b)]:
        pass  # under-voltage in active flags → already caught above

    cpu = data["cpu_pct"]
    last_cpu = last.get("cpu_pct", 0)
    if cpu >= THRESHOLDS["cpu_crit"] and last_cpu < THRESHOLDS["cpu_crit"]:
        add(f"🔴 CPU critical: {cpu:.0f}%")
    elif cpu < THRESHOLDS["cpu_warn"] and last_cpu >= THRESHOLDS["cpu_crit"]:
        add(f"✅ CPU load normalised: {cpu:.0f}%", style="green")


# ═══════════════════════════════════════════════════════════════
#  CSV LOGGER
# ═══════════════════════════════════════════════════════════════

def write_csv_row(writer, data: dict, power: dict):
    volts = data["voltages"]
    writer.writerow({
        "timestamp":   data["ts"].isoformat(),
        "temp_c":      data["temp"],
        "cpu_pct":     data["cpu_pct"],
        "cpu_freq_mhz": data["cpu_freq_mhz"],
        "ram_pct":     data["memory"]["ram_pct"],
        "volt_core":   volts.get("core"),
        "volt_sdram_c": volts.get("sdram_c"),
        "throttle_raw": data["throttle"]["raw"],
        "est_total_w": power["total_w"],
        "est_amps":    power["amps"],
        "usb_count":   len(data["usb_devices"]),
    })


CSV_FIELDS = ["timestamp", "temp_c", "cpu_pct", "cpu_freq_mhz",
              "ram_pct", "volt_core", "volt_sdram_c", "throttle_raw",
              "est_total_w", "est_amps", "usb_count"]


# ═══════════════════════════════════════════════════════════════
#  MAIN LOOP
# ═══════════════════════════════════════════════════════════════

def build_layout(data: dict, power: dict,
                 temp_hist: deque, cpu_hist: deque,
                 alerts: deque, rate: float) -> Layout:

    ts_str = data["ts"].strftime("%Y-%m-%d  %H:%M:%S")
    title  = (f"[bold white]🤖  Pi Robot Power Monitor  v{VERSION}"
              f"[dim]   •   {ts_str}   •   refresh {rate:.1f}s")

    layout = Layout()
    layout.split_column(
        Layout(name="title",      size=3),
        Layout(name="row1",       size=16),  # power status + voltages
        Layout(name="row2",       size=18),  # CPU/thermal + memory (taller for new mem fields)
        Layout(name="processes",  size=13),  # top processes
        Layout(name="usb",        size=14),  # USB devices + hardware note
        Layout(name="alerts",     size=8),   # alert log
    )
    layout["row1"].split_row(
        Layout(panel_power_status(data, power), name="status"),
        Layout(panel_voltages(data),            name="volts"),
    )
    layout["row2"].split_row(
        Layout(panel_cpu_thermal(data, temp_hist, cpu_hist), name="cpu"),
        Layout(panel_memory(data),                           name="mem"),
    )
    layout["title"].update(
        Panel(Align.center(Text(title)), border_style="bright_blue",
              box=box.HORIZONTALS)
    )
    layout["processes"].update(panel_processes(data))
    layout["usb"].update(panel_usb(data))
    layout["alerts"].update(panel_alerts(alerts))
    return layout


def run_monitor(rate: float = 1.0, log_path: str | None = None):
    temp_hist = deque(maxlen=HISTORY_SIZE)
    cpu_hist  = deque(maxlen=HISTORY_SIZE)
    alerts    = deque(maxlen=MAX_ALERTS)
    last_data: dict = {}

    csv_file   = None
    csv_writer = None
    if log_path:
        csv_file   = open(log_path, "w", newline="")
        csv_writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        csv_writer.writeheader()
        alerts.append((
            datetime.now().strftime("%H:%M:%S"),
            f"📁 Logging to {log_path}",
            "dim cyan"
        ))

    # Warm up psutil cpu_percent (first call always returns 0)
    psutil.cpu_percent(interval=None)
    psutil.cpu_percent(percpu=True, interval=None)
    time.sleep(0.3)

    def shutdown(sig, frame):
        if csv_file:
            csv_file.close()
        console.print("\n[dim]Monitor stopped.[/dim]")
        sys.exit(0)

    signal.signal(signal.SIGINT,  shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    with Live(console=console, screen=True, refresh_per_second=4) as live:
        while True:
            data  = collect_all()
            power = estimate_power(data)

            temp = data["temp"]
            if temp is not None:
                temp_hist.append(temp)
            cpu_hist.append(data["cpu_pct"])

            check_alerts(data, last_data, alerts)

            if csv_writer:
                write_csv_row(csv_writer, data, power)
                csv_file.flush()

            layout = build_layout(data, power, temp_hist, cpu_hist,
                                  alerts, rate)
            live.update(layout)

            last_data = {
                "temp":    data["temp"],
                "cpu_pct": data["cpu_pct"],
                "throttle": data["throttle"],
            }

            time.sleep(rate)


# ═══════════════════════════════════════════════════════════════
#  ENTRY POINT
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Real-time power monitor for Raspberry Pi robot controllers."
    )
    parser.add_argument(
        "--rate", type=float, default=1.0,
        help="Refresh interval in seconds (default: 1.0)"
    )
    parser.add_argument(
        "--log", type=str, default=None, metavar="FILE",
        help="Optional CSV file path for logging (e.g. power.csv)"
    )
    args = parser.parse_args()

    if not (0.1 <= args.rate <= 60):
        parser.error("--rate must be between 0.1 and 60 seconds")

    # Quick sanity check
    if not _run(["which", "vcgencmd"]):
        console.print(
            "[yellow]⚠  vcgencmd not found — voltage & throttle data will be "
            "unavailable.\n   This script is designed for Raspberry Pi OS.[/yellow]\n"
        )

    run_monitor(rate=args.rate, log_path=args.log)


if __name__ == "__main__":
    main()
