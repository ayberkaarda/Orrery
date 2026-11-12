#!/usr/bin/env python3
"""Checks the PNG writer against an independent decoder: Python's zlib.

Runs the hidden test case "[.png_export]" of orrery_test_render, which writes
PNG files of known content, then decodes every file with the standard library
only (struct for the chunks, zlib.crc32 for the chunk checksums,
zlib.decompress for the IDAT stream, the five PNG row filters by hand) and
compares the pixels with the same patterns computed here.

Usage: check_png_zlib.py <orrery_test_render executable> <output directory>
"""

from __future__ import annotations

import struct
import subprocess
import sys
import zlib
from pathlib import Path


def decode_png(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError(f"{path}: bad signature")
    pos = 8
    idat = bytearray()
    width = height = 0
    seen_end = False
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos : pos + 4])
        kind = data[pos + 4 : pos + 8]
        body = data[pos + 8 : pos + 8 + length]
        (crc,) = struct.unpack(">I", data[pos + 8 + length : pos + 12 + length])
        if zlib.crc32(kind + body) & 0xFFFFFFFF != crc:
            raise ValueError(f"{path}: CRC mismatch in {kind!r}")
        if kind == b"IHDR":
            width, height, depth, color, method, filtering, interlace = struct.unpack(
                ">IIBBBBB", body
            )
            if (depth, color, method, filtering, interlace) != (8, 6, 0, 0, 0):
                raise ValueError(f"{path}: not 8-bit RGBA")
        elif kind == b"IDAT":
            idat += body
        elif kind == b"IEND":
            seen_end = True
            pos += 12 + length
            break
        pos += 12 + length
    if not seen_end or pos != len(data):
        raise ValueError(f"{path}: missing IEND or trailing bytes")
    stream = zlib.decompressobj()
    raw = stream.decompress(bytes(idat))
    if not stream.eof or stream.unused_data:
        raise ValueError(f"{path}: incomplete zlib stream or trailing data")
    stride = width * 4
    if len(raw) != height * (stride + 1):
        raise ValueError(f"{path}: {len(raw)} bytes inflated, expected {height * (stride + 1)}")
    pixels = bytearray(height * stride)
    previous = bytearray(stride)
    for y in range(height):
        kind = raw[y * (stride + 1)]
        line = raw[y * (stride + 1) + 1 : (y + 1) * (stride + 1)]
        row = bytearray(stride)
        for i in range(stride):
            a = row[i - 4] if i >= 4 else 0
            b = previous[i]
            c = previous[i - 4] if i >= 4 else 0
            if kind == 0:
                p = 0
            elif kind == 1:
                p = a
            elif kind == 2:
                p = b
            elif kind == 3:
                p = (a + b) // 2
            elif kind == 4:
                estimate = a + b - c
                pa, pb, pc = abs(estimate - a), abs(estimate - b), abs(estimate - c)
                p = a if pa <= pb and pa <= pc else (b if pb <= pc else c)
            else:
                raise ValueError(f"{path}: invalid filter {kind} in row {y}")
            row[i] = (line[i] + p) & 0xFF
        pixels[y * stride : (y + 1) * stride] = row
        previous = row
    return width, height, bytes(pixels)


def fixed_pattern(x: int, y: int, c: int) -> int:
    return ((x * x + 3 * y + 17 * c) * 13) % 251


def dynamic_pattern(x: int, y: int, c: int) -> int:
    if c == 3:
        return 255
    return (((x * 2654435761 + y * 40503 + c * 977) >> 7) & 7) * 30


def gradient(x: int, y: int, c: int) -> int:
    return 255 if c == 3 else (x * (c + 1) + y * 2) % 256


def flat(x: int, y: int, c: int) -> int:
    return 40 * c + 7


def noise(x: int, y: int, c: int) -> int:
    mask = 0xFFFFFFFF
    h = ((x * 73856093) & mask) ^ ((y * 19349663) & mask) ^ ((c * 83492791) & mask)
    h ^= h >> 13
    h = (h * 0x5BD1E995) & mask
    h ^= h >> 15
    return h & 0xFF


CASES = {
    "fixed_pattern": (67, 45, fixed_pattern),
    "dynamic_pattern": (300, 200, dynamic_pattern),
    "gradient": (640, 360, gradient),
    "flat": (500, 300, flat),
    "noise": (211, 157, noise),
}


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    executable, output = str(Path(sys.argv[1]).resolve()), Path(sys.argv[2])
    for name in CASES:
        (output / f"png_check_{name}.png").unlink(missing_ok=True)
    run = subprocess.run([executable, "[.png_export]"], capture_output=True, text=True)
    if run.returncode != 0:
        print(run.stdout, run.stderr)
        return 1
    failures = 0
    for name, (width, height, pattern) in CASES.items():
        path = output / f"png_check_{name}.png"
        expected = bytes(
            pattern(x, y, c) for y in range(height) for x in range(width) for c in range(4)
        )
        try:
            w, h, pixels = decode_png(path)
        except (OSError, ValueError, zlib.error) as error:
            print(f"FAIL {name}: {error}")
            failures += 1
            continue
        raw_size = (width * 4 + 1) * height
        if (w, h) != (width, height) or pixels != expected:
            print(f"FAIL {name}: pixels differ from the pattern")
            failures += 1
        else:
            size = path.stat().st_size
            print(f"ok {name} {w}x{h}: {size} bytes ({100.0 * size / raw_size:.1f} % of raw)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
