#!/usr/bin/env python3
"""Render the L2Pro 智城 scan (zhicheng/) as a top-down GeoTIFF basemap.

The scan PLY stores local coordinates; true UTM 49N (EPSG:32649) =
stored + offset (from PLY header comments, with hard-coded fallbacks).
Near-ground points are averaged into an orthophoto with georef embedded
in the TIFF (no GDAL / sidecar world file needed).

Dependency-light: stdlib + numpy + PIL only (/usr/bin/python3).

Run:
    /usr/bin/python3 scripts/zhicheng_align.py
    /usr/bin/python3 scripts/zhicheng_align.py --res 0.1 --out /tmp/ortho.tif
    /usr/bin/python3 scripts/zhicheng_align.py /path/to/point_cloud.ply
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

# WGS84 / UTM constants
_A = 6378137.0
_F = 1.0 / 298.257223563
_K0 = 0.9996
_E2 = _F * (2 - _F)
_EP2 = _E2 / (1 - _E2)

# Scan georeference fallbacks (from PLY header comments, iteration_100)
SCAN_OFFSET_E = 801814.1094338207039982
SCAN_OFFSET_N = 2499371.9000139287672937
SCAN_OFFSET_Z = 29.3282999992370605
SCAN_EPSG = 32649  # UTM 49N
SCAN_ZONE = 49
_SCAN_XMIN, _SCAN_XMAX = -182.5084229, 538.4031372
_SCAN_YMIN, _SCAN_YMAX = -410.8949280, 464.6614075

_SH_C0 = 0.28209479177387814  # SH degree-0 basis -> RGB


def _zone_central_meridian(zone: int) -> float:
    return math.radians(6 * zone - 183)


def lonlat_to_utm(lon_deg: float, lat_deg: float, zone: int = 49) -> tuple[float, float]:
    """WGS84 lon/lat (deg) -> UTM easting, northing (m). Northern hemisphere."""
    lon = math.radians(lon_deg)
    lat = math.radians(lat_deg)
    lon0 = _zone_central_meridian(zone)

    N = _A / math.sqrt(1 - _E2 * math.sin(lat) ** 2)
    T = math.tan(lat) ** 2
    C = _EP2 * math.cos(lat) ** 2
    Acoef = (lon - lon0) * math.cos(lat)

    M = _A * (
        (1 - _E2 / 4 - 3 * _E2**2 / 64 - 5 * _E2**3 / 256) * lat
        - (3 * _E2 / 8 + 3 * _E2**2 / 32 + 45 * _E2**3 / 1024) * math.sin(2 * lat)
        + (15 * _E2**2 / 256 + 45 * _E2**3 / 1024) * math.sin(4 * lat)
        - (35 * _E2**3 / 3072) * math.sin(6 * lat)
    )

    easting = _K0 * N * (
        Acoef
        + (1 - T + C) * Acoef**3 / 6
        + (5 - 18 * T + T**2 + 72 * C - 58 * _EP2) * Acoef**5 / 120
    ) + 500000.0
    northing = _K0 * (
        M
        + N * math.tan(lat) * (
            Acoef**2 / 2
            + (5 - T + 9 * C + 4 * C**2) * Acoef**4 / 24
            + (61 - 58 * T + T**2 + 600 * C - 330 * _EP2) * Acoef**6 / 720
        )
    )
    return easting, northing


def utm_to_lonlat(easting: float, northing: float, zone: int = 49) -> tuple[float, float]:
    """UTM easting, northing (m) -> WGS84 lon/lat (deg). Northern hemisphere."""
    lon0 = _zone_central_meridian(zone)
    x = easting - 500000.0
    y = northing

    M = y / _K0
    mu = M / (_A * (1 - _E2 / 4 - 3 * _E2**2 / 64 - 5 * _E2**3 / 256))
    e1 = (1 - math.sqrt(1 - _E2)) / (1 + math.sqrt(1 - _E2))

    phi1 = (
        mu
        + (3 * e1 / 2 - 27 * e1**3 / 32) * math.sin(2 * mu)
        + (21 * e1**2 / 16 - 55 * e1**4 / 32) * math.sin(4 * mu)
        + (151 * e1**3 / 96) * math.sin(6 * mu)
        + (1097 * e1**4 / 512) * math.sin(8 * mu)
    )

    N1 = _A / math.sqrt(1 - _E2 * math.sin(phi1) ** 2)
    T1 = math.tan(phi1) ** 2
    C1 = _EP2 * math.cos(phi1) ** 2
    R1 = _A * (1 - _E2) / (1 - _E2 * math.sin(phi1) ** 2) ** 1.5
    D = x / (N1 * _K0)

    lat = phi1 - (N1 * math.tan(phi1) / R1) * (
        D**2 / 2
        - (5 + 3 * T1 + 10 * C1 - 4 * C1**2 - 9 * _EP2) * D**4 / 24
        + (61 + 90 * T1 + 298 * C1 + 45 * T1**2 - 252 * _EP2 - 3 * C1**2) * D**6 / 720
    )
    lon = lon0 + (
        D
        - (1 + 2 * T1 + C1) * D**3 / 6
        + (5 - 2 * C1 + 28 * T1 - 3 * C1**2 + 8 * _EP2 + 24 * T1**2) * D**5 / 120
    ) / math.cos(phi1)
    return math.degrees(lon), math.degrees(lat)


def ply_header(path: str):
    """Return (header_bytes, n_props) for a binary_little_endian float PLY."""
    with open(path, "rb") as f:
        raw = f.read(8192)
    end = raw.find(b"end_header\n") + len(b"end_header\n")
    nprops = sum(1 for l in raw[:end].split(b"\n") if l.startswith(b"property"))
    return end, nprops


def _parse_ply_comments(path: str) -> dict:
    """Read numeric `comment key value` pairs and optional `comment epsg N`."""
    with open(path, "rb") as f:
        raw = f.read(8192)
    end = raw.find(b"end_header\n")
    if end < 0:
        return {}
    out: dict = {}
    for lb in raw[:end].split(b"\n"):
        parts = lb.decode("ascii", "replace").split()
        if len(parts) < 3 or parts[0] != "comment":
            continue
        key = parts[1].lower()
        try:
            out[key] = int(float(parts[2])) if key == "epsg" else float(parts[2])
        except ValueError:
            pass
    return out


def render_ground_ortho(
    ply_path: str,
    res: float = 0.4,
    z_lo: float = -6.0,
    z_hi: float = 4.0,
    opacity_min: float = 0.4,
    chunk: int = 2_000_000,
):
    """Top-down mean-color raster of near-ground points (an ortho-like basemap).

    Local scan coords: pixel col = (x - xmin)/res, row = (ymax - y)/res
    (north up). Returns (rgb_uint8 HxWx3, extent_local, filled) where
    extent_local = (xmin, xmax, ymin, ymax).
    """
    offset, nprops = ply_header(ply_path)
    nbytes = os.path.getsize(ply_path) - offset
    n = nbytes // (nprops * 4)
    mm = np.memmap(ply_path, dtype=np.float32, mode="r", offset=offset,
                   shape=(n, nprops))

    c = _parse_ply_comments(ply_path)
    xmin = c.get("minx", _SCAN_XMIN)
    xmax = c.get("maxx", _SCAN_XMAX)
    ymin = c.get("miny", _SCAN_YMIN)
    ymax = c.get("maxy", _SCAN_YMAX)
    W = int(math.ceil((xmax - xmin) / res))
    H = int(math.ceil((ymax - ymin) / res))

    sum_rgb = np.zeros((H * W, 3), dtype=np.float32)
    cnt = np.zeros(H * W, dtype=np.int32)

    kept = 0
    for s in range(0, n, chunk):
        e = min(s + chunk, n)
        blk = np.asarray(mm[s:e])
        x, y, z = blk[:, 0], blk[:, 1], blk[:, 2]
        op = blk[:, 9]
        sig = 1.0 / (1.0 + np.exp(-op))
        m = (z >= z_lo) & (z <= z_hi) & (sig >= opacity_min)
        if not m.any():
            continue
        xs, ys = x[m], y[m]
        col = ((xs - xmin) / res).astype(np.int64)
        row = ((ymax - ys) / res).astype(np.int64)
        ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
        col, row = col[ok], row[ok]
        flat = row * W + col
        rgb = np.clip(0.5 + _SH_C0 * blk[m][ok][:, 6:9], 0.0, 1.0)
        np.add.at(sum_rgb, flat, rgb)
        np.add.at(cnt, flat, 1)
        kept += int(ok.sum())

    img = np.zeros((H * W, 3), dtype=np.float32)
    nz = cnt > 0
    mean = sum_rgb[nz] / cnt[nz, None]
    # brighten: gamma + gain so the dark road surface reads clearly
    mean = np.clip(mean * 1.8, 0, 1) ** 0.7
    img[nz] = mean
    img = (img * 255).astype(np.uint8).reshape(H, W, 3)

    filled = nz.reshape(H, W)
    print(f"  ortho: {W}x{H}px @ {res} m/px, ground pts used={kept:,}, "
          f"filled cells={int(nz.sum()):,}/{H*W:,}")
    return img, (xmin, xmax, ymin, ymax), filled


def export_geotiff(ply_path, out_tif, res=0.2, epsg=None):
    """Write a self-contained GeoTIFF (georef embedded, EPSG:32649 by default).

    No GDAL needed: GeoTIFF tags are written via PIL.
    """
    from PIL import Image
    from PIL.TiffImagePlugin import ImageFileDirectory_v2

    c = _parse_ply_comments(ply_path)
    off_e = c.get("offsetx", SCAN_OFFSET_E)
    off_n = c.get("offsety", SCAN_OFFSET_N)
    if epsg is None:
        epsg = int(c.get("epsg", SCAN_EPSG))

    img, extent, filled = render_ground_ortho(ply_path, res=res)
    xmin, xmax, ymin, ymax = extent
    H, W = img.shape[:2]

    rgba = np.zeros((H, W, 4), dtype=np.uint8)
    rgba[..., :3] = img
    rgba[..., 3] = np.where(filled, 255, 0).astype(np.uint8)

    # Tiepoint (PixelIsArea): raster (0,0) = top-left *corner* of the image.
    x_tl = off_e + xmin
    y_tl = off_n + ymax

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


def _default_ply() -> str:
    return str(_REPO / "zhicheng/point_cloud/iteration_100/point_cloud.ply")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="L2Pro / 3DGS point_cloud.ply -> GeoTIFF basemap (EPSG:32649)"
    )
    ap.add_argument(
        "ply",
        nargs="?",
        default=_default_ply(),
        help="path to point_cloud.ply (default: <repo>/zhicheng/.../point_cloud.ply)",
    )
    ap.add_argument(
        "--out",
        default=str(_CAMPUS / "zhicheng_ortho.tif"),
        help="output GeoTIFF path",
    )
    ap.add_argument("--res", type=float, default=0.2, help="meters per pixel")
    ap.add_argument("--epsg", type=int, default=None, help="override CRS EPSG")
    args = ap.parse_args(argv)

    if not os.path.isfile(args.ply):
        print(f"error: PLY not found: {args.ply}", file=sys.stderr)
        return 1

    export_geotiff(args.ply, args.out, res=args.res, epsg=args.epsg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
