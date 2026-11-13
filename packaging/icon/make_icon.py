#!/usr/bin/env python3
"""Draws the Orrery application icon and writes it as PNG, ICO and SVG.

The icon is a sun inside a tilted orbit with one planet, on a dark disc. It is
computed analytically (supersampled coverage), so the output depends only on
this script: no artwork, fonts or image libraries are involved. Only basic
floating-point operations and sqrt are used (no libm transcendental calls),
which IEEE 754 makes identical on every platform.

    python packaging/icon/make_icon.py           # rewrite the files beside this script
    python packaging/icon/make_icon.py --check   # exit 1 if a file is out of date

Outputs: orrery.svg (scalable source for desktop icon themes), orrery.png
(256 px, Linux desktop and AppImage), orrery.ico (16 to 256 px, Windows
executable, installer and shortcuts).
"""

import argparse
import math
import struct
import sys
import zlib
from pathlib import Path

HERE = Path(__file__).resolve().parent
ICO_SIZES = (16, 24, 32, 48, 64, 128, 256)
PNG_SIZE = 256

# Geometry in unit coordinates (x right, y down, icon spans [0, 1]).
CENTER = 0.5
DISC_RADIUS = 0.47
SUN_RADIUS = 0.125
GLOW_RADIUS = 0.205
ORBIT_A = 0.37
ORBIT_B = 0.165
ORBIT_STROKE = 0.03
# Orbit tilt of -24 degrees, as literal cosine and sine.
TILT_COS = 0.9135454576426009
TILT_SIN = -0.4067366430758002
# Planet at orbit parameter 35 degrees (front half), as cosine and sine.
PLANET_COS = 0.8191520442889918
PLANET_SIN = 0.5735764363510461
PLANET_RADIUS = 0.06
PLANET_HALO = 0.03

DISC_INNER = (0.106, 0.169, 0.325)
DISC_OUTER = (0.039, 0.067, 0.149)
ORBIT_COLOR = (0.561, 0.702, 1.0)
SUN_CORE = (1.0, 0.890, 0.580)
SUN_EDGE = (1.0, 0.584, 0.200)
PLANET_COLOR = (0.459, 0.878, 0.784)


def mix(a, b, t):
    return tuple(x + (y - x) * t for x, y in zip(a, b))


def over(dst, color, alpha):
    """Composites a straight-alpha colour over a premultiplied (r, g, b, a)."""
    r, g, b, a = dst
    return (r * (1 - alpha) + color[0] * alpha, g * (1 - alpha) + color[1] * alpha,
            b * (1 - alpha) + color[2] * alpha, a * (1 - alpha) + alpha)


def planet_center():
    x = ORBIT_A * PLANET_COS
    y = ORBIT_B * PLANET_SIN
    return (CENTER + x * TILT_COS - y * TILT_SIN, CENTER + x * TILT_SIN + y * TILT_COS)


def orbit(x, y, stroke):
    """Ring coverage and whether the point lies on the front half."""
    dx, dy = x - CENTER, y - CENTER
    u = dx * TILT_COS + dy * TILT_SIN
    v = -dx * TILT_SIN + dy * TILT_COS
    g = math.sqrt((u / ORBIT_A) ** 2 + (v / ORBIT_B) ** 2)
    if g == 0.0:
        return 0.0, False
    gu = u / (ORBIT_A * ORBIT_A * g)
    gv = v / (ORBIT_B * ORBIT_B * g)
    distance = abs(g - 1.0) / math.sqrt(gu * gu + gv * gv)
    return (1.0 if distance <= stroke / 2 else 0.0), v > 0.0


def sample(x, y, stroke, px):
    """Premultiplied colour of one sample; px is the size of a pixel."""
    out = (0.0, 0.0, 0.0, 0.0)
    r = math.sqrt((x - CENTER) ** 2 + (y - CENTER) ** 2)
    if r > DISC_RADIUS:
        return out
    out = over(out, mix(DISC_INNER, DISC_OUTER, (r / DISC_RADIUS) ** 2), 1.0)
    ring, front = orbit(x, y, stroke)
    if ring > 0.0 and not front:
        out = over(out, ORBIT_COLOR, 0.55 * ring)
    if r < GLOW_RADIUS:
        t = 1.0 - (r - SUN_RADIUS) / (GLOW_RADIUS - SUN_RADIUS)
        if t < 1.0:
            out = over(out, SUN_EDGE, 0.35 * t * t)
    if r < SUN_RADIUS:
        out = over(out, mix(SUN_CORE, SUN_EDGE, (r / SUN_RADIUS) ** 2), 1.0)
    if ring > 0.0 and front:
        out = over(out, ORBIT_COLOR, 0.9 * ring)
    cx, cy = planet_center()
    d = math.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    halo = PLANET_RADIUS + max(PLANET_HALO, 1.5 * px)
    if d < halo:
        out = over(out, DISC_OUTER, 1.0)
    if d < PLANET_RADIUS:
        shade = max(0.0, min(1.0, ((x - cx) + (y - cy)) / (2 * PLANET_RADIUS) + 0.5))
        out = over(out, mix(PLANET_COLOR, DISC_OUTER, 0.45 * shade), 1.0)
    return out


def render(size):
    """RGBA rows (bytes) of the icon at size x size pixels."""
    ss = 4 if size >= 64 else 6
    px = 1.0 / size
    # Keep the orbit at least about 1.3 px wide at small sizes.
    stroke = max(ORBIT_STROKE, 1.3 * px)
    rows = []
    for j in range(size):
        row = bytearray()
        for i in range(size):
            acc = [0.0, 0.0, 0.0, 0.0]
            for sj in range(ss):
                for si in range(ss):
                    x = (i + (si + 0.5) / ss) * px
                    y = (j + (sj + 0.5) / ss) * px
                    s = sample(x, y, stroke, px)
                    for k in range(4):
                        acc[k] += s[k]
            n = ss * ss
            a = acc[3] / n
            if a <= 0.0:
                row += b"\0\0\0\0"
                continue
            row += bytes(int(min(255, max(0, round(acc[k] / n / a * 255)))) for k in range(3))
            row.append(int(round(a * 255)))
        rows.append(bytes(row))
    return rows


def png_bytes(rows):
    size = len(rows)

    def chunk(kind, data):
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body))

    raw = b"".join(b"\0" + row for row in rows)
    header = struct.pack(">IIBBBBB", size, size, 8, 6, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header) + chunk(b"IDAT", zlib.compress(raw, 9)) +
            chunk(b"IEND", b""))


def ico_bytes(images):
    """ICO with PNG-compressed entries (Windows Vista and later)."""
    header = struct.pack("<HHH", 0, 1, len(images))
    offset = len(header) + 16 * len(images)
    entries, blobs = b"", b""
    for size, data in images:
        dim = 0 if size >= 256 else size
        entries += struct.pack("<BBBBHHII", dim, dim, 0, 0, 1, 32, len(data), offset)
        blobs += data
        offset += len(data)
    return header + entries + blobs


def svg_text():
    cx, cy = planet_center()
    tilt = -24
    return f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 256 256" width="256" height="256">
  <!-- Generated by make_icon.py; edit the script, not this file. -->
  <defs>
    <radialGradient id="disc"><stop offset="0" stop-color="{hexc(DISC_INNER)}"/><stop offset="1" stop-color="{hexc(DISC_OUTER)}"/></radialGradient>
    <radialGradient id="sun"><stop offset="0" stop-color="{hexc(SUN_CORE)}"/><stop offset="1" stop-color="{hexc(SUN_EDGE)}"/></radialGradient>
    <radialGradient id="glow"><stop offset="{SUN_RADIUS / GLOW_RADIUS:.3f}" stop-color="{hexc(SUN_EDGE)}" stop-opacity="0.35"/><stop offset="1" stop-color="{hexc(SUN_EDGE)}" stop-opacity="0"/></radialGradient>
    <clipPath id="back"><rect x="-200" y="-200" width="400" height="200"/></clipPath>
    <clipPath id="front"><rect x="-200" y="0" width="400" height="200"/></clipPath>
  </defs>
  <circle cx="128" cy="128" r="{u(DISC_RADIUS)}" fill="url(#disc)"/>
  <g transform="translate(128 128) rotate({tilt})" fill="none" stroke="{hexc(ORBIT_COLOR)}" stroke-width="{u(ORBIT_STROKE)}">
    <ellipse rx="{u(ORBIT_A)}" ry="{u(ORBIT_B)}" stroke-opacity="0.55" clip-path="url(#back)"/>
  </g>
  <circle cx="128" cy="128" r="{u(GLOW_RADIUS)}" fill="url(#glow)"/>
  <circle cx="128" cy="128" r="{u(SUN_RADIUS)}" fill="url(#sun)"/>
  <g transform="translate(128 128) rotate({tilt})" fill="none" stroke="{hexc(ORBIT_COLOR)}" stroke-width="{u(ORBIT_STROKE)}">
    <ellipse rx="{u(ORBIT_A)}" ry="{u(ORBIT_B)}" stroke-opacity="0.9" clip-path="url(#front)"/>
  </g>
  <circle cx="{cx * 256:.2f}" cy="{cy * 256:.2f}" r="{u(PLANET_RADIUS + PLANET_HALO)}" fill="{hexc(DISC_OUTER)}"/>
  <circle cx="{cx * 256:.2f}" cy="{cy * 256:.2f}" r="{u(PLANET_RADIUS)}" fill="{hexc(PLANET_COLOR)}"/>
</svg>
"""


def u(value):
    return f"{value * 256:.2f}"


def hexc(color):
    return "#" + "".join(f"{int(round(c * 255)):02x}" for c in color)


def png_pixels(data):
    """IHDR and decompressed scanlines of a PNG written by png_bytes.

    zlib builds may compress the same scanlines differently, so --check
    compares the pixels, not the compressed bytes.
    """
    pos, header, idat = 8, b"", b""
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        kind, body = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + length]
        if kind == b"IHDR":
            header = body
        elif kind == b"IDAT":
            idat += body
        pos += 12 + length
    return header, zlib.decompress(idat)


def comparable(name, data):
    try:
        if name.endswith(".png"):
            return png_pixels(data)
        if name.endswith(".ico"):
            (count,) = struct.unpack("<H", data[4:6])
            images = []
            for k in range(count):
                size, offset = struct.unpack("<II", data[6 + 16 * k + 8:6 + 16 * k + 16])
                images.append(png_pixels(data[offset:offset + size]))
            return images
    except (struct.error, zlib.error):
        return None
    # A checkout may convert the SVG's line endings.
    return data.replace(b"\r\n", b"\n")


def outputs():
    images = {size: png_bytes(render(size)) for size in sorted(set(ICO_SIZES + (PNG_SIZE,)))}
    return {
        "orrery.png": images[PNG_SIZE],
        "orrery.ico": ico_bytes([(size, images[size]) for size in ICO_SIZES]),
        "orrery.svg": svg_text().encode("ascii"),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="compare instead of writing")
    parser.add_argument("--out", type=Path, default=HERE, help="output directory")
    args = parser.parse_args()
    stale = []
    for name, data in outputs().items():
        path = args.out / name
        if args.check:
            if not path.is_file() or comparable(name, path.read_bytes()) != comparable(name, data):
                stale.append(name)
        else:
            path.write_bytes(data)
            print(f"wrote {name} ({len(data)} bytes)")
    if stale:
        print("out of date: " + ", ".join(stale) + " (run make_icon.py)", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
