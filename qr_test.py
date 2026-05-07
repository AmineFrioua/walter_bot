#!/usr/bin/env python3
"""
QR Code Readability Tester — Walter Robot
==========================================
Scans a folder of images, tries to decode QR codes using two independent
decoders (pyzbar + OpenCV), applies a progressive set of image transforms
when direct decoding fails, and produces a per-image readability report.

Dependencies:
    pip3 install pyzbar opencv-python Pillow

ZBar shared library (required by pyzbar):
    Debian/Ubuntu/Pi:  sudo apt-get install libzbar0
    macOS:             brew install zbar

Usage:
    python3 qr_test.py                          # scan current directory
    python3 qr_test.py --dir /path/to/images    # scan a specific folder
    python3 qr_test.py --debug                  # save annotated images → ./qr_debug/
    python3 qr_test.py --json report.json       # write JSON report
    python3 qr_test.py --quiet                  # summary only, no per-image lines

Result levels:
    IMMEDIATE ✅     decoded on the raw image — no changes needed
    PROCESSING 🔧    decoded after one or more transforms (transform chain reported)
    UNREADABLE ❌    every strategy failed
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import traceback
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np
from PIL import Image, ImageDraw

try:
    from pyzbar import pyzbar as _pyzbar
    PYZBAR_OK = True
except ImportError:
    PYZBAR_OK = False
    print("⚠  pyzbar not found — install with: pip3 install pyzbar  (and: sudo apt-get install libzbar0)")

# ── Supported image extensions ─────────────────────────────────────────────────

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".webp", ".gif"}

# ── Result data classes ────────────────────────────────────────────────────────

@dataclass
class DecodeHit:
    data: list[str]           # decoded QR payloads
    decoder: str              # "pyzbar" | "cv2" | "both"
    transform: str            # "none" or human-readable description of what was applied
    points: Optional[list] = None  # bounding polygon for debug drawing

@dataclass
class ImageReport:
    file: str
    status: str               # "IMMEDIATE" | "PROCESSING" | "UNREADABLE" | "ERROR"
    transforms: list[str] = field(default_factory=list)   # chain that worked
    data: list[str]  = field(default_factory=list)        # decoded payloads
    decoder: str     = ""
    error: str       = ""                                  # set if file couldn't be loaded


# ── Low-level decode helpers ───────────────────────────────────────────────────

def _pil_to_bgr(img: Image.Image) -> np.ndarray:
    rgb = np.array(img.convert("RGB"))
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)


def _decode_pyzbar(img: Image.Image) -> Optional[DecodeHit]:
    """Try pyzbar. Returns DecodeHit on success, None otherwise."""
    if not PYZBAR_OK:
        return None
    try:
        objs = _pyzbar.decode(img)
        qrs  = [o for o in objs if o.type == "QRCODE"]
        if not qrs:
            return None
        data   = [o.data.decode("utf-8", errors="replace") for o in qrs]
        points = [[p.x, p.y] for p in qrs[0].polygon]
        return DecodeHit(data=data, decoder="pyzbar", transform="none", points=points)
    except Exception:
        return None


def _decode_cv2(img: Image.Image) -> Optional[DecodeHit]:
    """Try OpenCV QRCodeDetector. Returns DecodeHit on success, None otherwise."""
    try:
        bgr      = _pil_to_bgr(img)
        detector = cv2.QRCodeDetector()
        data, pts, _ = detector.detectAndDecode(bgr)
        if not data:
            # try the newer WeChatQRCode detector if available (more robust)
            try:
                wechat = cv2.wechat_qrcode_WeChatQRCode()
                texts, _rects = wechat.detectAndDecode(bgr)
                if texts:
                    return DecodeHit(data=list(texts), decoder="cv2-wechat", transform="none")
            except Exception:
                pass
            return None
        points = pts.astype(int).tolist() if pts is not None else None
        return DecodeHit(data=[data], decoder="cv2", transform="none", points=points)
    except Exception:
        return None


def try_decode(img: Image.Image) -> Optional[DecodeHit]:
    """Run both decoders; return the first hit, or None."""
    hit = _decode_pyzbar(img)
    if hit:
        return hit
    return _decode_cv2(img)


# ── Image transforms ───────────────────────────────────────────────────────────
# Each transform takes a PIL Image and returns a PIL Image.
# They are PURE — they do not modify the input.

def _rotate(img: Image.Image, deg: int) -> Image.Image:
    return img.rotate(deg, expand=True)

def _grayscale(img: Image.Image) -> Image.Image:
    return img.convert("L").convert("RGB")

def _invert(img: Image.Image) -> Image.Image:
    arr = np.array(img.convert("RGB"))
    return Image.fromarray(255 - arr)

def _clahe(img: Image.Image) -> Image.Image:
    gray = np.array(img.convert("L"))
    eq   = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    return Image.fromarray(eq).convert("RGB")

def _otsu(img: Image.Image) -> Image.Image:
    gray   = np.array(img.convert("L"))
    _, thr = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return Image.fromarray(thr).convert("RGB")

def _sharpen(img: Image.Image) -> Image.Image:
    arr     = np.array(img.convert("RGB"))
    kernel  = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    result  = cv2.filter2D(arr, -1, kernel)
    return Image.fromarray(result.astype(np.uint8))

def _upscale(img: Image.Image, factor: int) -> Image.Image:
    w, h = img.size
    return img.resize((w * factor, h * factor), Image.LANCZOS)

def _crop_centre(img: Image.Image, fraction: float) -> Image.Image:
    w, h = img.size
    dx   = int(w * (1 - fraction) / 2)
    dy   = int(h * (1 - fraction) / 2)
    return img.crop((dx, dy, w - dx, h - dy))

def _denoise(img: Image.Image) -> Image.Image:
    """Gaussian blur then sharpen — helps with noisy/compressed images."""
    arr     = np.array(img.convert("RGB"))
    blurred = cv2.GaussianBlur(arr, (3, 3), 0)
    kernel  = np.array([[0, -1, 0], [-1, 5, -1], [0, -1, 0]])
    result  = cv2.filter2D(blurred, -1, kernel)
    return Image.fromarray(result.astype(np.uint8))

def _adaptive_threshold(img: Image.Image) -> Image.Image:
    """Adaptive (block-local) threshold — better for uneven lighting."""
    gray = np.array(img.convert("L"))
    thr  = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )
    return Image.fromarray(thr).convert("RGB")


# ── Transform pipeline ─────────────────────────────────────────────────────────
# Each entry: (label, function).  Tried in order; stop on first success.
# Combos are listed last — they're slower and cover edge cases.

def _build_pipeline() -> list[tuple[str, Callable[[Image.Image], Image.Image]]]:
    return [
        # Stage 2 — Orientation
        ("rotate 90°",                    lambda i: _rotate(i, 90)),
        ("rotate 180°",                   lambda i: _rotate(i, 180)),
        ("rotate 270°",                   lambda i: _rotate(i, 270)),

        # Stage 3 — Colour space
        ("grayscale",                     _grayscale),
        ("invert",                        _invert),

        # Stage 4 — Contrast / sharpness
        ("CLAHE",                         _clahe),
        ("threshold (Otsu)",              _otsu),
        ("adaptive threshold",            _adaptive_threshold),
        ("sharpen",                       _sharpen),
        ("denoise + sharpen",             _denoise),

        # Stage 5 — Scale / crop
        ("upscale 2×",                    lambda i: _upscale(i, 2)),
        ("upscale 3×",                    lambda i: _upscale(i, 3)),
        ("crop centre 50%",               lambda i: _crop_centre(i, 0.50)),
        ("crop centre 25%",               lambda i: _crop_centre(i, 0.25)),

        # Stage 6 — Combos
        ("grayscale + rotate 90°",        lambda i: _rotate(_grayscale(i), 90)),
        ("grayscale + rotate 180°",       lambda i: _rotate(_grayscale(i), 180)),
        ("grayscale + rotate 270°",       lambda i: _rotate(_grayscale(i), 270)),
        ("CLAHE + threshold",             lambda i: _otsu(_clahe(i))),
        ("grayscale + CLAHE + threshold", lambda i: _otsu(_clahe(_grayscale(i)))),
        ("upscale 2× + threshold",        lambda i: _otsu(_upscale(i, 2))),
        ("upscale 3× + threshold",        lambda i: _otsu(_upscale(i, 3))),
        ("upscale 2× + CLAHE",            lambda i: _clahe(_upscale(i, 2))),
        ("upscale 2× + CLAHE + threshold",lambda i: _otsu(_clahe(_upscale(i, 2)))),
        ("upscale 3× + CLAHE + threshold",lambda i: _otsu(_clahe(_upscale(i, 3)))),
        ("crop 50% + upscale 2×",         lambda i: _upscale(_crop_centre(i, 0.50), 2)),
        ("crop 25% + upscale 3×",         lambda i: _upscale(_crop_centre(i, 0.25), 3)),
        ("invert + threshold",            lambda i: _otsu(_invert(i))),
        ("grayscale + adaptive threshold",lambda i: _adaptive_threshold(_grayscale(i))),
        ("upscale 2× + adaptive threshold",lambda i: _adaptive_threshold(_upscale(i, 2))),
    ]


# ── Per-image analysis ─────────────────────────────────────────────────────────

def analyse_image(path: Path, debug_dir: Optional[Path] = None) -> ImageReport:
    """Run the full decode pipeline on one image file."""
    # ── Load ──────────────────────────────────────────────────────────────────
    try:
        img = Image.open(path).convert("RGB")
    except Exception as exc:
        return ImageReport(file=path.name, status="ERROR", error=str(exc))

    # ── Stage 1: Direct decode ────────────────────────────────────────────────
    hit = try_decode(img)
    if hit:
        if debug_dir:
            _save_debug(img, path, hit, "direct", debug_dir)
        return ImageReport(
            file    = path.name,
            status  = "IMMEDIATE",
            transforms = [],
            data    = hit.data,
            decoder = hit.decoder,
        )

    # ── Stages 2–6: Progressive transforms ────────────────────────────────────
    pipeline = _build_pipeline()
    for label, transform_fn in pipeline:
        try:
            transformed = transform_fn(img)
            hit = try_decode(transformed)
            if hit:
                hit.transform = label
                if debug_dir:
                    _save_debug(transformed, path, hit, label, debug_dir)
                return ImageReport(
                    file       = path.name,
                    status     = "PROCESSING",
                    transforms = [label],
                    data       = hit.data,
                    decoder    = hit.decoder,
                )
        except Exception:
            continue  # skip broken transform, try the next

    # ── Nothing worked ────────────────────────────────────────────────────────
    if debug_dir:
        _save_debug(img, path, None, "unreadable", debug_dir)
    return ImageReport(file=path.name, status="UNREADABLE")


# ── Debug image output ─────────────────────────────────────────────────────────

def _save_debug(img: Image.Image, src: Path,
                hit: Optional[DecodeHit], label: str,
                debug_dir: Path) -> None:
    """Save an annotated copy of img to debug_dir."""
    debug_dir.mkdir(parents=True, exist_ok=True)
    annotated = img.copy().convert("RGB")
    draw      = ImageDraw.Draw(annotated)

    if hit and hit.points:
        # Draw the QR code bounding polygon
        try:
            pts = [(int(p[0]), int(p[1])) for p in hit.points]
            if len(pts) >= 3:
                draw.polygon(pts, outline=(0, 220, 0))
                for pt in pts:
                    draw.ellipse([pt[0]-4, pt[1]-4, pt[0]+4, pt[1]+4],
                                 fill=(0, 220, 0))
        except Exception:
            pass

    # Overlay: status text
    status_color = (0, 200, 0) if hit else (220, 0, 0)
    summary = f"[{label}] {hit.data[0][:40] if hit else 'UNREADABLE'}"
    _draw_label(draw, summary, (4, 4), status_color)

    stem     = src.stem
    safe_lbl = label.replace("/", "-").replace(" ", "_").replace("×", "x")
    out_path = debug_dir / f"{stem}__{safe_lbl}.jpg"
    annotated.save(out_path, quality=90)


def _draw_label(draw: ImageDraw.ImageDraw, text: str,
                xy: tuple[int, int], colour: tuple[int, int, int]) -> None:
    """Draw text with a dark shadow so it is visible on any background."""
    x, y = xy
    draw.text((x + 1, y + 1), text, fill=(0, 0, 0))
    draw.text((x,     y    ), text, fill=colour)


# ── Directory scan ─────────────────────────────────────────────────────────────

def scan_directory(scan_dir: Path) -> list[Path]:
    paths = sorted(
        p for p in scan_dir.iterdir()
        if p.is_file() and p.suffix.lower() in IMAGE_EXTS
    )
    return paths


# ── Reporting ──────────────────────────────────────────────────────────────────

STATUS_ICON = {
    "IMMEDIATE":  "✅",
    "PROCESSING": "🔧",
    "UNREADABLE": "❌",
    "ERROR":      "💥",
}

def print_report(reports: list[ImageReport], scan_dir: Path, quiet: bool) -> None:
    total   = len(reports)
    n_imm   = sum(1 for r in reports if r.status == "IMMEDIATE")
    n_proc  = sum(1 for r in reports if r.status == "PROCESSING")
    n_unrd  = sum(1 for r in reports if r.status == "UNREADABLE")
    n_err   = sum(1 for r in reports if r.status == "ERROR")

    width = 70
    print(f"\n{'═'*width}")
    print(f"  Walter — QR Code Readability Test")
    print(f"  Scanning : {scan_dir}")
    print(f"  Found    : {total} image{'s' if total != 1 else ''}")
    print(f"{'═'*width}")

    if not quiet:
        print()
        for i, r in enumerate(reports, 1):
            icon   = STATUS_ICON.get(r.status, "?")
            prefix = f"  [{i:>3}/{total}]  {r.file:<30}"

            if r.status == "IMMEDIATE":
                payload = _short_payload(r.data)
                print(f"{prefix}{icon} IMMEDIATE         data={payload!r}")

            elif r.status == "PROCESSING":
                tx      = " → ".join(r.transforms) if r.transforms else "?"
                payload = _short_payload(r.data)
                print(f"{prefix}{icon} PROCESSING")
                print(f"{'':>42}transform : {tx}")
                print(f"{'':>42}decoder   : {r.decoder}")
                print(f"{'':>42}data      : {payload!r}")

            elif r.status == "UNREADABLE":
                print(f"{prefix}{icon} UNREADABLE        (no QR found after all strategies)")

            elif r.status == "ERROR":
                print(f"{prefix}{icon} ERROR             {r.error}")

        print()

    print(f"{'─'*width}")
    print(f"  SUMMARY   total={total}   "
          f"✅ immediate={n_imm}   "
          f"🔧 with-processing={n_proc}   "
          f"❌ unreadable={n_unrd}"
          + (f"   💥 errors={n_err}" if n_err else ""))
    print(f"{'═'*width}\n")


def _short_payload(data: list[str]) -> str:
    if not data:
        return ""
    s = data[0]
    if len(data) > 1:
        s += f" (+{len(data)-1} more)"
    return s[:80]


def write_json(reports: list[ImageReport], scan_dir: Path, out_path: Path) -> None:
    payload = {
        "timestamp": datetime.now().isoformat(),
        "scan_dir":  str(scan_dir),
        "summary": {
            "total":      len(reports),
            "immediate":  sum(1 for r in reports if r.status == "IMMEDIATE"),
            "processing": sum(1 for r in reports if r.status == "PROCESSING"),
            "unreadable": sum(1 for r in reports if r.status == "UNREADABLE"),
            "errors":     sum(1 for r in reports if r.status == "ERROR"),
        },
        "images": [asdict(r) for r in reports],
    }
    with open(out_path, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"  JSON report written → {out_path}")


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="QR code readability tester — scans a folder and reports per image",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--dir",   default=".",
                        help="Directory to scan (default: current directory)")
    parser.add_argument("--debug", action="store_true",
                        help="Save annotated images to ./qr_debug/")
    parser.add_argument("--json",  metavar="FILE",
                        help="Write JSON report to FILE (e.g. report.json)")
    parser.add_argument("--quiet", action="store_true",
                        help="Only show summary, not per-image lines")
    args = parser.parse_args()

    scan_dir  = Path(args.dir).resolve()
    debug_dir = Path("qr_debug") if args.debug else None

    if not scan_dir.is_dir():
        print(f"Error: '{scan_dir}' is not a directory.")
        sys.exit(1)

    if not PYZBAR_OK:
        print("Warning: pyzbar unavailable — only OpenCV decoder will be used.\n")

    images = scan_directory(scan_dir)
    if not images:
        print(f"No image files found in {scan_dir}")
        print(f"Supported extensions: {', '.join(sorted(IMAGE_EXTS))}")
        sys.exit(0)

    # ── Run analysis ────────────────────────────────────────────────────────
    reports: list[ImageReport] = []
    total = len(images)
    for i, path in enumerate(images, 1):
        print(f"\r  Analysing {i}/{total}: {path.name:<40}", end="", flush=True)
        report = analyse_image(path, debug_dir=debug_dir)
        reports.append(report)
    print()  # newline after progress line

    # ── Output ──────────────────────────────────────────────────────────────
    print_report(reports, scan_dir, args.quiet)

    if args.json:
        write_json(reports, scan_dir, Path(args.json))

    if debug_dir:
        print(f"  Debug images → {debug_dir.resolve()}/\n")

    # Exit code: 0 if all images have a QR, 1 if any are unreadable
    if any(r.status in ("UNREADABLE", "ERROR") for r in reports):
        sys.exit(1)


if __name__ == "__main__":
    main()
