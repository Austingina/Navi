#!/usr/bin/env python3
"""Render a "share" (SAE) colorized survey PLY as a top-down GeoTIFF basemap.

Unlike the L2Pro / 3DGS scan handled by ``zhicheng_align.py`` (all-float32
vertices + SH color + a local offset from PLY header comments), the SAE scanner
writes:

  * mixed-type vertices, e.g.
        double x, y, z          (already absolute UTM, EPSG:32649)
        uchar  red, green, blue (direct 0-255 colour, NO SH decode)
        float  nx, ny, nz, scalar_Intensity, scalar_Return_Number,
               scalar_Number_Of_Returns
        double scalar_Gps_Time
  * NO offset: x/y ARE the true UTM 49N easting/northing (metres);
  * NO min/max/offset/epsg header comments.

So this script parses the PLY header generically (any property order/type),
memmaps the file with the matching structured dtype, computes the XY/Z bounds
in a first pass, then averages colour into a north-up orthophoto with the
GeoTIFF georef embedded (tiepoint = raw UTM, no offset added).

Dependency-light: stdlib + numpy + PIL only (/usr/bin/python3).

Run:
    /usr/bin/python3 scripts/share_align.py
    /usr/bin/python3 scripts/share_align.py --res 0.1 --out /tmp/share.tif
    /usr/bin/python3 scripts/share_align.py /path/to/PointCloud_colorized.ply
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np

_SCRIPTS = Path(__file__).resolve().parent
_CAMPUS = _SCRIPTS.parent
_REPO = _CAMPUS.parent

# Default input (the SAE share export; sits at the Navi workspace root).
_DEFAULT_PLY = Path("/home/navi/Navi/2026-09-09_14-20-36_PointCloud_colorized.ply")

SCAN_EPSG = 32649  # UTM 49N; coords are already absolute (no offset).

# Raster cell cap: refuse to allocate an insane image, hint to raise --res.
_MAX_CELLS = 400_000_000

# PLY scalar type -> little-endian numpy dtype string.
_PLY_TYPES = {
    "char": "i1", "int8": "i1",
    "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2",
    "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4",
    "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4",
    "double": "f8", "float64": "f8",
}


def parse_ply_header(path: str):
    """Parse a binary_little_endian PLY header generically.

    Returns (data_offset, structured_dtype, n_vertex, field_names).
    Raises ValueError on ascii / big-endian / list properties (not supported).
    """
    with open(path, "rb") as f:
        raw = f.read(65536)
    marker = b"end_header\n"
    idx = raw.find(marker)
    if idx < 0:
        raise ValueError("PLY header too large / missing end_header")
    data_offset = idx + len(marker)

    fmt = None
    n_vertex = None
    in_vertex = False
    names: list[str] = []
    types: list[str] = []
    for lb in raw[:idx].split(b"\n"):
        parts = lb.decode("ascii", "replace").split()
        if not parts:
            continue
        tok = parts[0]
        if tok == "format" and len(parts) >= 2:
            fmt = parts[1]
        elif tok == "element" and len(parts) >= 3:
            in_vertex = parts[1] == "vertex"
            if in_vertex:
                n_vertex = int(parts[2])
        elif tok == "property" and in_vertex:
            if parts[1] == "list":
                raise ValueError("list properties in vertex element not supported")
            ptype = parts[1]
            if ptype not in _PLY_TYPES:
                raise ValueError(f"unsupported PLY property type: {ptype}")
            names.append(parts[-1])
            types.append(_PLY_TYPES[ptype])

    if fmt != "binary_little_endian":
        raise ValueError(f"only binary_little_endian PLY supported, got: {fmt}")
    if not names or n_vertex is None:
        raise ValueError("no vertex element / properties found in PLY header")

    dt = np.dtype([(n, "<" + t) for n, t in zip(names, types)])

    # Sanity: file size must match header count exactly.
    nbytes = os.path.getsize(path) - data_offset
    if nbytes % dt.itemsize != 0:
        raise ValueError(
            f"file body ({nbytes} B) not a multiple of record size ({dt.itemsize} B)"
        )
    n_records = nbytes // dt.itemsize
    if n_records != n_vertex:
        # Trust the actual file size, but warn.
        print(f"  warning: header vertex={n_vertex} but file has {n_records} records; "
              f"using {n_records}")
        n_vertex = n_records
    return data_offset, dt, n_vertex, names


def compute_bounds(mm, n: int, chunk: int):
    """First pass: XY/Z min/max over all points (chunked, memory-light)."""
    xmin = ymin = zmin = math.inf
    xmax = ymax = zmax = -math.inf
    for s in range(0, n, chunk):
        e = min(s + chunk, n)
        blk = mm[s:e]
        x = np.asarray(blk["x"], dtype=np.float64)
        y = np.asarray(blk["y"], dtype=np.float64)
        z = np.asarray(blk["z"], dtype=np.float64)
        xmin, xmax = min(xmin, float(x.min())), max(xmax, float(x.max()))
        ymin, ymax = min(ymin, float(y.min())), max(ymax, float(y.max()))
        zmin, zmax = min(zmin, float(z.min())), max(zmax, float(z.max()))
    return xmin, xmax, ymin, ymax, zmin, zmax


def render_ground_ortho(
    ply_path: str,
    res: float = 0.05,
    z_lo: float | None = None,
    z_hi: float | None = None,
    gain: float = 1.0,
    gamma: float = 1.0,
    chunk: int = 2_000_000,
):
    """Top-down mean-colour raster (north-up) of the colorized point cloud.

    UTM coords (metres): pixel col = (x - xmin)/res, row = (ymax - y)/res.
    ``z_lo``/``z_hi`` are optional ABSOLUTE z (UTM height) filters; None = keep
    all. Returns (rgb_uint8 HxWx3, extent_utm=(xmin,xmax,ymin,ymax), filled).
    """
    data_offset, dt, n, names = parse_ply_header(ply_path)
    for req in ("x", "y", "z", "red", "green", "blue"):
        if req not in names:
            raise ValueError(f"PLY missing required property '{req}' (has: {names})")
    mm = np.memmap(ply_path, dtype=dt, mode="r", offset=data_offset, shape=(n,))

    xmin, xmax, ymin, ymax, zmin, zmax = compute_bounds(mm, n, chunk)
    print(f"  bounds UTM: E[{xmin:.2f},{xmax:.2f}] N[{ymin:.2f},{ymax:.2f}] "
          f"Z[{zmin:.2f},{zmax:.2f}]  ({xmax-xmin:.1f} x {ymax-ymin:.1f} m)")

    W = int(math.ceil((xmax - xmin) / res))
    H = int(math.ceil((ymax - ymin) / res))
    if W <= 0 or H <= 0:
        raise ValueError("degenerate extent; check the PLY coordinates")
    if H * W > _MAX_CELLS:
        raise ValueError(
            f"raster {W}x{H} = {H*W:,} cells exceeds cap ({_MAX_CELLS:,}); "
            f"increase --res (current {res} m/px)"
        )

    sum_rgb = np.zeros((H * W, 3), dtype=np.float64)
    cnt = np.zeros(H * W, dtype=np.int64)

    kept = 0
    for s in range(0, n, chunk):
        e = min(s + chunk, n)
        blk = mm[s:e]
        x = np.asarray(blk["x"], dtype=np.float64)
        y = np.asarray(blk["y"], dtype=np.float64)
        z = np.asarray(blk["z"], dtype=np.float64)

        m = np.ones(x.shape, dtype=bool)
        if z_lo is not None:
            m &= z >= z_lo
        if z_hi is not None:
            m &= z <= z_hi
        if not m.any():
            continue

        xs, ys = x[m], y[m]
        col = ((xs - xmin) / res).astype(np.int64)
        row = ((ymax - ys) / res).astype(np.int64)
        ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
        col, row = col[ok], row[ok]
        flat = row * W + col

        r = np.asarray(blk["red"])[m][ok]
        g = np.asarray(blk["green"])[m][ok]
        b = np.asarray(blk["blue"])[m][ok]
        rgb = np.stack([r, g, b], axis=1).astype(np.float64)  # 0-255
        np.add.at(sum_rgb, flat, rgb)
        np.add.at(cnt, flat, 1)
        kept += int(ok.sum())

    img = np.zeros((H * W, 3), dtype=np.float64)
    nz = cnt > 0
    mean = sum_rgb[nz] / cnt[nz, None] / 255.0          # -> 0..1
    if gain != 1.0:
        mean = np.clip(mean * gain, 0.0, 1.0)
    if gamma != 1.0:
        mean = np.clip(mean, 0.0, 1.0) ** gamma
    img[nz] = np.clip(mean, 0.0, 1.0)
    img = (img * 255.0).astype(np.uint8).reshape(H, W, 3)

    filled = nz.reshape(H, W)
    print(f"  ortho: {W}x{H}px @ {res} m/px, pts used={kept:,}, "
          f"filled cells={int(nz.sum()):,}/{H*W:,}")
    return img, (xmin, xmax, ymin, ymax), filled


def export_geotiff(ply_path, out_tif, res=0.05, epsg=SCAN_EPSG,
                   z_lo=None, z_hi=None, gain=1.0, gamma=1.0):
    """Write a self-contained GeoTIFF (georef embedded, EPSG:32649 by default).

    Coordinates are already absolute UTM (no offset), so the raster tiepoint is
    simply the top-left corner (xmin, ymax). No GDAL needed: GeoTIFF tags via PIL.
    """
    from PIL import Image
    from PIL.TiffImagePlugin import ImageFileDirectory_v2

    img, extent, filled = render_ground_ortho(
        ply_path, res=res, z_lo=z_lo, z_hi=z_hi, gain=gain, gamma=gamma
    )
    xmin, xmax, ymin, ymax = extent
    H, W = img.shape[:2]

    rgba = np.zeros((H, W, 4), dtype=np.uint8)
    rgba[..., :3] = img
    rgba[..., 3] = np.where(filled, 255, 0).astype(np.uint8)

    # Tiepoint (PixelIsArea): raster (0,0) = top-left *corner*. No offset added.
    x_tl = xmin
    y_tl = ymax

    ifd = ImageFileDirectory_v2()
    ifd[33550] = (res, res, 0.0)                       # ModelPixelScale
    ifd.tagtype[33550] = 12                            # DOUBLE
    ifd[33922] = (0.0, 0.0, 0.0, x_tl, y_tl, 0.0)      # ModelTiepoint
    ifd.tagtype[33922] = 12
    ifd[34735] = (                                     # GeoKeyDirectory
        1, 1, 0, 3,
        1024, 0, 1, 1,        # GTModelType = projected
        1025, 0, 1, 1,        # GTRasterType = PixelIsArea
        3072, 0, 1, epsg,     # ProjectedCSType = EPSG code
    )
    ifd.tagtype[34735] = 3                             # SHORT

    Image.fromarray(rgba, "RGBA").save(
        out_tif, tiffinfo=ifd, compression="tiff_adobe_deflate"
    )
    print(f"  saved GeoTIFF -> {out_tif}  ({W}x{H}px, {res} m/px, EPSG:{epsg})")
    print(f"  top-left UTM = ({x_tl:.2f}, {y_tl:.2f}), "
          f"ground size = {W * res:.1f} x {H * res:.1f} m")
    return out_tif


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="SAE 'share' colorized PLY -> GeoTIFF basemap (EPSG:32649, no offset)"
    )
    ap.add_argument(
        "ply",
        nargs="?",
        default=str(_DEFAULT_PLY),
        help="path to *_PointCloud_colorized.ply (default: <repo>/2026-09-09_..._colorized.ply)",
    )
    ap.add_argument(
        "--out",
        default=str(_CAMPUS / "share_ortho.tif"),
        help="output GeoTIFF path",
    )
    ap.add_argument("--res", type=float, default=0.05, help="metres per pixel")
    ap.add_argument("--epsg", type=int, default=SCAN_EPSG, help="override CRS EPSG")
    ap.add_argument("--z-lo", type=float, default=None,
                    help="optional absolute UTM z lower bound (keep z >= z_lo)")
    ap.add_argument("--z-hi", type=float, default=None,
                    help="optional absolute UTM z upper bound (keep z <= z_hi)")
    ap.add_argument("--gain", type=float, default=1.0,
                    help="brightness gain applied to mean colour (default 1.0)")
    ap.add_argument("--gamma", type=float, default=1.0,
                    help="gamma applied to mean colour (default 1.0 = none)")
    args = ap.parse_args(argv)

    if not os.path.isfile(args.ply):
        print(f"error: PLY not found: {args.ply}", file=sys.stderr)
        return 1

    export_geotiff(args.ply, args.out, res=args.res, epsg=args.epsg,
                   z_lo=args.z_lo, z_hi=args.z_hi, gain=args.gain, gamma=args.gamma)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
