#!/usr/bin/env python3
"""Regenerate the README screenshots in docs/images with the headless app.

Each shot is rendered by `orrery_app --headless` (an offscreen OpenGL 4.6
context; no window is shown), re-encoded losslessly as an RGB PNG at the
highest zlib effort with Pillow, and written without any text, time or
profile chunks, so the images carry no tool names, user names or paths.
Without Pillow the rendered PNG is kept as it is, minus those chunks.
Palette reduction is deliberately not used: it bands the colour maps and
changes the colours that the legends explain.

Usage:
  python tools/make_screenshots.py --app <path to orrery_app> [--only NAME ...]

The list of shots, with their experiments, frames and sizes, is SHOTS below
and in docs/screenshots.md. Exit status 0 when every shot was written.
"""

from __future__ import annotations

import argparse
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
OUT_DIR = ROOT / "docs" / "images"

# (file name, experiment name or path relative to the repository, frame, size)
SHOTS = [
    ("space-solar-system.png", "solar_system", 3000, "1600x900"),
    ("space-black-hole.png", "black_hole", 0, "1600x900"),
    ("space-figure-eight.png", "figure_eight", 1200, "1600x900"),
    ("aero-naca0012-lbm.png", "aero_naca0012_lbm", 0, "1600x900"),
    ("aero-apollo-newtonian.png", "aero_apollo_capsule", 0, "1600x900"),
    ("aero-naca0012-stall-wake.png", "docs/screenshots/naca0012_stall_lbm.json", 0, "1600x900"),
    ("entry-apollo.png", "entry_apollo_like", 0, "1600x900"),
]

KEPT_CHUNKS = {b"IHDR", b"PLTE", b"tRNS", b"IDAT", b"IEND", b"gAMA", b"sRGB"}
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def png_chunks(data: bytes) -> list[tuple[bytes, bytes]]:
    if not data.startswith(PNG_SIGNATURE):
        raise ValueError("not a PNG file")
    chunks = []
    pos = len(PNG_SIGNATURE)
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        kind = data[pos + 4:pos + 8]
        chunks.append((kind, data[pos + 8:pos + 8 + length]))
        pos += 12 + length
    return chunks


def strip_metadata(path: Path) -> None:
    """Rewrite a PNG keeping only image chunks (no text, time or profile)."""
    out = bytearray(PNG_SIGNATURE)
    for kind, body in png_chunks(path.read_bytes()):
        if kind not in KEPT_CHUNKS:
            continue
        out += struct.pack(">I", len(body)) + kind + body
        out += struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF)
    path.write_bytes(bytes(out))


def recompress(source: Path, target: Path) -> bool:
    try:
        from PIL import Image
    except ImportError:
        shutil.copyfile(source, target)
        return False
    with Image.open(source) as image:
        rgb = image.convert("RGB")
    rgb.save(target, format="PNG", optimize=True)
    return True


def experiment_argument(experiment: str) -> str:
    if experiment.endswith(".json"):
        return str(ROOT / experiment)
    return experiment


def render(app: Path, shot: tuple[str, str, int, str], work: Path) -> Path | None:
    name, experiment, frame, size = shot
    raw = work / name
    command = [str(app), "--headless", "--experiment", experiment_argument(experiment),
               "--frame", str(frame), "--size", size, "--out", str(raw)]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    print(result.stdout.strip())
    if result.returncode != 0 or not raw.is_file():
        print(f"error: {name}: orrery_app exited with {result.returncode}: "
              f"{result.stderr.strip()}")
        return None
    return raw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--app", type=Path, required=True, help="path to orrery_app")
    parser.add_argument("--only", nargs="*", default=[], help="file names to regenerate")
    args = parser.parse_args()
    shots = [s for s in SHOTS if not args.only or s[0] in args.only]
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        for shot in shots:
            raw = render(args.app, shot, Path(tmp))
            if raw is None:
                failures += 1
                continue
            target = OUT_DIR / shot[0]
            if not recompress(raw, target):
                print("note: Pillow not found, the image is not recompressed")
            strip_metadata(target)
            print(f"{target.relative_to(ROOT).as_posix()}: {target.stat().st_size // 1024} KiB")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
