#!/usr/bin/env python3
"""Bake QGIS-edited GPKG roads into a single OSM PBF (production artifact).

Reads the base pbf + gpkg overlay (same osm_id merge as build_graph_merged),
writes a new .osm.pbf with updated way geometries/tags. Nodes are reused when
coordinates match; new vertices get new node ids.

Usage:
    /usr/bin/python3 scripts/merge_pbf_gpkg.py
    /usr/bin/python3 scripts/merge_pbf_gpkg.py \\
        --pbf ../../xili_5km_with_extra.osm.pbf \\
        --gpkg ../new_road.gpkg \\
        --output ../../xili_5km_merged.osm.pbf

After merge, run the planner on the merged pbf with dev overlay OFF:
    /usr/bin/python3 route_planner_node.py --ros-args \\
        -p pbf_path:=/home/navi/nav_test/xili_5km_merged.osm.pbf \\
        -p dev_overlay:=false
"""

from __future__ import annotations

import argparse
import math
import sqlite3
import sys
from pathlib import Path

import osmium
from osmium.osm.mutable import Node, Relation, Way

# Parent campus_nav/ for route_graph
_CAMPUS = Path(__file__).resolve().parent.parent
_REPO = _CAMPUS.parent
if str(_CAMPUS) not in sys.path:
    sys.path.insert(0, str(_CAMPUS))

from route_graph import QUANT, _parse_gpkg_blob

# GPKG metadata columns — not written as OSM tags.
_SKIP_TAG_COLS = {
    "fid", "geom", "osm_id", "osm_type", "full_id",
    "osm_version", "osm_timestamp", "osm_uid", "osm_user", "osm_changeset",
}


def _coord_key(lon: float, lat: float):
    return (round(lon * QUANT), round(lat * QUANT))


def _tags_dict(obj) -> dict:
    return {t.k: t.v for t in obj.tags}


def load_gpkg_overlay(gpkg_path: str, table: str = "osmfile") -> dict:
    """osm_id -> {tags: dict, lines: list of [(lon,lat), ...]}."""
    con = sqlite3.connect(gpkg_path)
    cols = [
        r[1] for r in con.execute(f"PRAGMA table_info({table})")
        if r[1] not in ("fid", "geom")
    ]
    sel = ", ".join(f'"{c}"' for c in ["osm_id", "geom"] + [c for c in cols if c != "osm_id"])
    out = {}
    for row in con.execute(f"SELECT {sel} FROM {table} WHERE osm_id IS NOT NULL"):
        osm_id = str(row[0])
        geom = row[1]
        tag_vals = row[2:]
        tag_cols = [c for c in cols if c != "osm_id"]
        tags = {
            k: v for k, v in zip(tag_cols, tag_vals)
            if k not in _SKIP_TAG_COLS and v is not None and v != ""
        }
        lines = _parse_gpkg_blob(geom)
        if not lines:
            continue
        if "highway" not in tags:
            continue
        out[osm_id] = {"tags": tags, "lines": lines}
    con.close()
    return out


def merge_tags(pbf_tags: dict, gpkg_tags: dict) -> dict:
    merged = dict(pbf_tags)
    merged.update(gpkg_tags)
    return merged


class _RelationScan(osmium.SimpleHandler):
    """Pre-pass: pick relations to keep + collect their member way ids."""

    def __init__(self):
        super().__init__()
        self.keep_rel_ids: set[int] = set()
        self.keep_member_ways: set[int] = set()

    def relation(self, r):
        tags = {t.k: t.v for t in r.tags}
        rtype = tags.get("type")
        is_building_mp = rtype == "multipolygon" and "building" in tags
        is_restriction = rtype == "restriction"
        if not (is_building_mp or is_restriction):
            return
        self.keep_rel_ids.add(r.id)
        for m in r.members:
            if m.type == "w":
                self.keep_member_ways.add(m.ref)


class _Collector(osmium.SimpleHandler):
    def __init__(self, overlay: dict, keep_rel_ids: set, keep_member_ways: set):
        super().__init__()
        self.overlay = overlay
        self.keep_rel_ids = keep_rel_ids
        self.keep_member_ways = keep_member_ways
        self.nodes: dict[int, tuple[float, float]] = {}
        self.coord_index: dict[tuple[int, int], int] = {}
        self.ways: dict[int, Way] = {}
        self.relations: dict[int, Relation] = {}
        self.max_node_id = 0
        self._next_node_id = None
        self.overlay_seen: set[str] = set()
        self.max_way_id = 0
        self.stats = {
            "nodes": 0, "ways_kept": 0, "ways_building": 0,
            "ways_overridden": 0, "ways_added": 0, "relations": 0, "new_nodes": 0,
        }

    def _register_node(self, nid: int, lon: float, lat: float):
        self.nodes[nid] = (lon, lat)
        self.coord_index[_coord_key(lon, lat)] = nid
        if nid > self.max_node_id:
            self.max_node_id = nid

    def _resolve_node(self, lon: float, lat: float) -> int:
        k = _coord_key(lon, lat)
        if k in self.coord_index:
            return self.coord_index[k]
        if self._next_node_id is None:
            self._next_node_id = self.max_node_id + 1
        nid = self._next_node_id
        self._next_node_id += 1
        self._register_node(nid, lon, lat)
        self.stats["new_nodes"] += 1
        return nid

    def node(self, n):
        if n.id not in self.nodes:
            self._register_node(n.id, n.location.lon, n.location.lat)
            self.stats["nodes"] += 1

    def way(self, w):
        if w.id > self.max_way_id:
            self.max_way_id = w.id
        wid = str(w.id)
        if wid in self.overlay:
            gw = self.overlay[wid]
            line = gw["lines"][0]
            if gw["tags"].get("oneway") == "-1":
                line = line[::-1]
            nids = [self._resolve_node(lon, lat) for lon, lat in line]
            if len(nids) < 2:
                return
            tags = merge_tags(_tags_dict(w), gw["tags"])
            self.ways[w.id] = Way(id=w.id, nodes=nids, tags=tags)
            self.overlay_seen.add(wid)
            self.stats["ways_overridden"] += 1
            return
        tags = _tags_dict(w)
        # Keep roads, buildings, and any way referenced by a kept relation
        # (e.g. multipolygon building outer/inner rings, which carry no tag).
        keep = (
            "highway" in tags
            or "building" in tags
            or w.id in self.keep_member_ways
        )
        if not keep:
            return
        self.ways[w.id] = Way(id=w.id, nodes=list(w.nodes), tags=tags)
        if "highway" in tags:
            self.stats["ways_kept"] += 1
        else:
            self.stats["ways_building"] += 1

    def relation(self, r):
        if r.id not in self.keep_rel_ids:
            return
        members = [(m.type, m.ref, m.role) for m in r.members]
        self.relations[r.id] = Relation(
            id=r.id, members=members, tags=_tags_dict(r),
        )
        self.stats["relations"] += 1

    def add_new_overlay_ways(self):
        """Emit overlay ways whose osm_id is NOT in the base pbf as brand-new
        ways (new geometry). New vertices reuse existing nodes when coordinates
        match (connects new roads to the current network), else get new ids."""
        next_wid = self.max_way_id + 1
        self.added_way_ids: set[int] = set()
        for osm_id, gw in self.overlay.items():
            if osm_id in self.overlay_seen:
                continue
            for line in gw["lines"]:
                if gw["tags"].get("oneway") == "-1":
                    line = line[::-1]
                nids = [self._resolve_node(lon, lat) for lon, lat in line]
                if len(nids) < 2:
                    continue
                try:
                    wid = int(osm_id)
                except (TypeError, ValueError):
                    wid = next_wid
                    next_wid += 1
                if wid in self.ways:
                    wid = next_wid
                    next_wid += 1
                self.ways[wid] = Way(id=wid, nodes=nids, tags=dict(gw["tags"]))
                self.added_way_ids.add(wid)
                self.stats["ways_added"] += 1

    def node_dangling_ends_to_base(self, tol_m=6.0):
        """Connect loose ends of the newly added ways to the existing road
        network: project each dangling endpoint onto the nearest existing
        highway segment and, if within tol, split that segment (insert a shared
        node) so routing can traverse between new and existing roads."""
        from collections import Counter

        added = getattr(self, "added_way_ids", set())
        if not added:
            return
        # base ways keep NodeRef objects (from list(w.nodes)); normalize every
        # way to plain int node ids so splitting/lookup is uniform.
        def _nid(x):
            return x.ref if hasattr(x, "ref") else int(x)
        for w in self.ways.values():
            w.nodes = [_nid(n) for n in w.nodes]
        # dangling endpoints = appear once across added ways' ends, and not
        # already shared with a base node.
        base_nodes = set()
        for wid, w in self.ways.items():
            if wid not in added:
                base_nodes.update(w.nodes)
        endc = Counter()
        for wid in added:
            nodes = self.ways[wid].nodes
            endc[nodes[0]] += 1
            endc[nodes[-1]] += 1
        cands = [nid for nid, c in endc.items() if c == 1 and nid not in base_nodes]
        if not cands:
            return

        # footprint bbox (+pad) from added vertices, to limit segment search
        xs = [self.nodes[n][0] for wid in added for n in self.ways[wid].nodes]
        ys = [self.nodes[n][1] for wid in added for n in self.ways[wid].nodes]
        pad = 0.0006
        lo_x, hi_x, lo_y, hi_y = min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad

        # base highway segments overlapping the footprint
        segs = []  # (wid, seg_index, (lon1,lat1), (lon2,lat2))
        for wid, w in self.ways.items():
            if wid in added or "highway" not in w.tags:
                continue
            ns = w.nodes
            for i in range(len(ns) - 1):
                a = self.nodes[ns[i]]
                b = self.nodes[ns[i + 1]]
                if (max(a[0], b[0]) < lo_x or min(a[0], b[0]) > hi_x or
                        max(a[1], b[1]) < lo_y or min(a[1], b[1]) > hi_y):
                    continue
                segs.append((wid, i, a, b))
        if not segs:
            return

        coslat = math.cos(math.radians((lo_y + hi_y) / 2))
        m_per_deg = math.pi / 180 * 6378137.0
        tol_deg = tol_m / m_per_deg

        def proj(p, a, b):
            # planar projection in scaled-lon/lat; returns (dist_deg, t, point)
            ax = (a[0] - p[0]) * coslat; ay = a[1] - p[1]
            bx = (b[0] - p[0]) * coslat; by = b[1] - p[1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            if L2 < 1e-18:
                return math.hypot(ax, ay), 0.0, a
            t = max(0.0, min(1.0, -(ax * dx + ay * dy) / L2))
            qx, qy = ax + t * dx, ay + t * dy
            d = math.hypot(qx, qy)
            pt = (p[0] + (qx / coslat), p[1] + qy)
            return d, t, pt

        connected = 0
        for nid in cands:
            p = self.nodes[nid]
            best = None
            for wid, i, a, b in segs:
                d, t, pt = proj(p, a, b)
                if best is None or d < best[0]:
                    best = (d, wid, i, t, pt, a, b)
            if best is None or best[0] > tol_deg:
                continue
            _d, wid, i, t, pt, a, b = best
            w = self.ways[wid]
            # near an existing endpoint of the segment -> just merge coordinates
            if t < 0.05 or t > 0.95:
                shared = w.nodes[i] if t < 0.05 else w.nodes[i + 1]
                self.nodes[nid] = self.nodes[shared]
                self.coord_index[_coord_key(*self.nodes[shared])] = shared
                # rewrite added ways to use the shared node id
                self._replace_node(nid, shared, added)
            else:
                # split the base segment: move endpoint to projection, insert it
                self.nodes[nid] = pt
                w.nodes = w.nodes[:i + 1] + [nid] + w.nodes[i + 1:]
            connected += 1
        self.stats["ends_connected"] = connected

    def _replace_node(self, old, new, way_ids):
        for wid in way_ids:
            w = self.ways[wid]
            if old in w.nodes:
                w.nodes = [new if x == old else x for x in w.nodes]


def write_pbf(collector: _Collector, output_path: str):
    writer = osmium.SimpleWriter(str(output_path), overwrite=True)
    for nid in sorted(collector.nodes):
        lon, lat = collector.nodes[nid]
        writer.add_node(Node(id=nid, location=osmium.osm.Location(lon, lat)))
    for wid in sorted(collector.ways):
        writer.add_way(collector.ways[wid])
    for rid in sorted(collector.relations):
        writer.add_relation(collector.relations[rid])
    writer.close()


def merge_pbf_gpkg(
    pbf_path: str,
    gpkg_path: str,
    output_path: str,
    gpkg_table: str = "osmfile",
) -> dict:
    overlay = load_gpkg_overlay(gpkg_path, gpkg_table)
    scan = _RelationScan()
    scan.apply_file(pbf_path)
    col = _Collector(overlay, scan.keep_rel_ids, scan.keep_member_ways)
    col.apply_file(pbf_path, locations=True)
    col.add_new_overlay_ways()
    col.node_dangling_ends_to_base()
    write_pbf(col, output_path)
    col.stats["gpkg_overlay_count"] = len(overlay)
    col.stats["output"] = str(output_path)
    return col.stats


def main():
    ap = argparse.ArgumentParser(description="Merge GPKG road edits into one OSM PBF.")
    ap.add_argument(
        "--pbf", default=str(_REPO / "xili_5km_with_extra.osm.pbf"),
        help="Base OSM PBF",
    )
    ap.add_argument(
        "--gpkg", default=str(_CAMPUS / "new_road.gpkg"),
        help="QGIS-edited GPKG overlay",
    )
    ap.add_argument(
        "--table", default="osmfile", help="Highway layer name in GPKG",
    )
    ap.add_argument(
        "--output", default=str(_REPO / "xili_5km_merged.osm.pbf"),
        help="Output merged PBF path",
    )
    args = ap.parse_args()

    for label, p in (("pbf", args.pbf), ("gpkg", args.gpkg)):
        if not Path(p).is_file():
            print(f"error: {label} not found: {p}", file=sys.stderr)
            sys.exit(1)

    print(f"merging\n  pbf: {args.pbf}\n  gpkg: {args.gpkg}\n  -> {args.output}")
    stats = merge_pbf_gpkg(args.pbf, args.gpkg, args.output, args.table)
    print("done:", stats)


if __name__ == "__main__":
    main()
