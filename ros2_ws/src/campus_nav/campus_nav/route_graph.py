"""Campus routing graph built directly from an OSM-derived GeoPackage.

Dependency-free on purpose: only the Python standard library + numpy are used,
so the same module runs both under conda Python and under the ROS 2 (Humble)
system Python that ships rclpy.

Pipeline:
    GeoPackage (route_edges, LINESTRING, EPSG:4326)
      -> parse GPKG geometry blobs (manual WKB reader)
      -> keep only walkable highways
      -> connect shared OSM vertices into a node/edge graph
      -> A* shortest path (haversine cost + heuristic)
      -> ENU (local metric) conversion for ROS campus_map frame
"""

from __future__ import annotations

import math
import os
import re
import sqlite3
import struct
from dataclasses import dataclass, field
from heapq import heappop, heappush

import numpy as np

# ---------------------------------------------------------------------------
# Highway filtering (walking / robot in a campus)
# ---------------------------------------------------------------------------
# Roads a pedestrian/robot may use.
WALKABLE_HIGHWAYS = {
    "footway", "path", "steps", "pedestrian", "living_street", "corridor",
    "residential", "service", "unclassified", "track", "cycleway", "road",
    "tertiary", "tertiary_link", "secondary", "secondary_link",
    "primary", "primary_link", "busway","residental"
}
# Never walkable (motor only).
BLOCKED_HIGHWAYS = {
    "motorway", "motorway_link", "trunk", "trunk_link",
    "construction", "proposed", "raceway",
}

# Coordinate quantization (decimal degrees). 1e-7 deg ~ 1.1cm; OSM nodes shared
# between ways carry identical coordinates, so this merges them into one node.
QUANT = 1e7

EARTH_R = 6378137.0  # WGS84 mean radius (m)


# ---------------------------------------------------------------------------
# GeoPackage geometry blob parsing  (https://www.geopackage.org/spec/)
# ---------------------------------------------------------------------------
_ENVELOPE_BYTES = {0: 0, 1: 32, 2: 48, 3: 48, 4: 64}


def _parse_gpkg_blob(blob: bytes):
    """Return a list of line strings; each is a list of (lon, lat) tuples."""
    if blob is None or len(blob) < 8 or blob[0:2] != b"GP":
        return []
    flags = blob[3]
    env_indicator = (flags >> 1) & 0x07
    header_len = 8 + _ENVELOPE_BYTES.get(env_indicator, 0)
    return _parse_wkb(blob[header_len:])


def _read_linestring(buf, off, bo, dims):
    n = struct.unpack_from(bo + "I", buf, off)[0]
    off += 4
    pts = []
    step = 8 * dims
    for _ in range(n):
        x, y = struct.unpack_from(bo + "dd", buf, off)
        pts.append((x, y))
        off += step
    return pts, off


def _wkb_dims(gtype: int) -> int:
    hi = gtype // 1000  # ISO WKB: 1=Z, 2=M, 3=ZM
    return 2 + (1 if hi in (1, 3) else 0) + (1 if hi in (2, 3) else 0)


def _parse_wkb(wkb: bytes):
    bo = "<" if wkb[0] == 1 else ">"
    gtype = struct.unpack_from(bo + "I", wkb, 1)[0]
    base = gtype % 1000
    dims = _wkb_dims(gtype)
    off = 5
    if base == 1:  # Point (entry-point layers); returned as a 1-vertex "line"
        x, y = struct.unpack_from(bo + "dd", wkb, off)
        return [[(x, y)]]
    if base == 2:  # LineString
        pts, _ = _read_linestring(wkb, off, bo, dims)
        return [pts] if len(pts) >= 2 else []
    if base == 5:  # MultiLineString
        nls = struct.unpack_from(bo + "I", wkb, off)[0]
        off += 4
        out = []
        for _ in range(nls):
            bo2 = "<" if wkb[off] == 1 else ">"
            off += 1
            gt2 = struct.unpack_from(bo2 + "I", wkb, off)[0]
            off += 4
            pts, off = _read_linestring(wkb, off, bo2, _wkb_dims(gt2))
            if len(pts) >= 2:
                out.append(pts)
        return out
    return []


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def haversine(lon1, lat1, lon2, lat2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlmb = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlmb / 2) ** 2
    return 2 * EARTH_R * math.asin(math.sqrt(a))


def _key(lon, lat):
    return (round(lon * QUANT), round(lat * QUANT))


# ---------------------------------------------------------------------------
# Graph
# ---------------------------------------------------------------------------
@dataclass
class RouteGraph:
    # node id (int) -> (lon, lat)
    coords: dict = field(default_factory=dict)
    # node id -> list of (neighbor_id, weight_m)
    adj: dict = field(default_factory=dict)
    _key_to_id: dict = field(default_factory=dict)
    # via_node_id -> list of {kind: "no"/"only", from: set(way_id), to: set(way_id), rtype}
    restrictions: dict = field(default_factory=dict)
    _id_coords: np.ndarray = None  # (N,2) lon,lat for nearest lookup
    _id_list: np.ndarray = None

    # -- construction --------------------------------------------------------
    def _node(self, lon, lat):
        k = _key(lon, lat)
        nid = self._key_to_id.get(k)
        if nid is None:
            nid = len(self.coords)
            self._key_to_id[k] = nid
            self.coords[nid] = (lon, lat)
            self.adj[nid] = []
        return nid

    def _add_edge(self, a, b, w, way_id=None, oneway=False):
        self.adj[a].append((b, w, way_id))
        if not oneway:
            self.adj[b].append((a, w, way_id))

    def finalize(self):
        ids = sorted(self.coords)
        self._id_list = np.array(ids, dtype=np.int64)
        self._id_coords = np.array([self.coords[i] for i in ids], dtype=np.float64)
        self._build_edge_index()
        return self

    def _build_edge_index(self):
        """Cache unique undirected edges for point-on-edge projection."""
        seen = set()
        eu, ev, wid = [], [], []
        for u, nbrs in self.adj.items():
            for v, _w, wi in nbrs:
                key = (u, v) if u < v else (v, u)
                if key in seen:
                    continue
                seen.add(key)
                eu.append(key[0])
                ev.append(key[1])
                wid.append(wi)
        self._edge_u = np.array(eu, dtype=np.int64)
        self._edge_v = np.array(ev, dtype=np.int64)
        self._edge_wayid = wid
        if eu:
            self._edge_ax = np.array([self.coords[a][0] for a in eu])
            self._edge_ay = np.array([self.coords[a][1] for a in eu])
            self._edge_bx = np.array([self.coords[b][0] for b in ev])
            self._edge_by = np.array([self.coords[b][1] for b in ev])
        else:
            self._edge_ax = None

    # -- queries -------------------------------------------------------------
    def nearest_node(self, lon, lat):
        d = (self._id_coords[:, 0] - lon) ** 2 + (self._id_coords[:, 1] - lat) ** 2
        return int(self._id_list[int(np.argmin(d))])

    def project_to_edge(self, lon, lat):
        """Project (lon, lat) onto the nearest road segment.

        Returns a dict with the segment endpoints (u, v), the parameter t in
        [0, 1] measured from u, the projected lon/lat, segment length L (m),
        distance from the query point (m), and the segment's way_id. Returns
        None if the graph has no edges.
        """
        if getattr(self, "_edge_ax", None) is None:
            return None
        clat = math.cos(math.radians(lat))
        ax, ay = self._edge_ax, self._edge_ay
        bx, by = self._edge_bx, self._edge_by
        dx = (bx - ax) * clat
        dy = by - ay
        seg2 = dx * dx + dy * dy
        px = (lon - ax) * clat
        py = lat - ay
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (px * dx + py * dy) / seg2
        t = np.where(seg2 > 0, t, 0.0)
        t = np.clip(t, 0.0, 1.0)
        cx = ax + t * (bx - ax)
        cy = ay + t * (by - ay)
        ddx = (lon - cx) * clat
        ddy = lat - cy
        i = int(np.argmin(ddx * ddx + ddy * ddy))
        plon, plat = float(cx[i]), float(cy[i])
        u, v = int(self._edge_u[i]), int(self._edge_v[i])
        ulon, ulat = self.coords[u]
        vlon, vlat = self.coords[v]
        return {
            "u": u, "v": v, "t": float(t[i]),
            "lon": plon, "lat": plat,
            "L": haversine(ulon, ulat, vlon, vlat),
            "dist": haversine(lon, lat, plon, plat),
            "way_id": self._edge_wayid[i],
        }

    def plan_points(self, slon, slat, glon, glat, respect_turns=False):
        """Plan between two free points on roads (not restricted to vertices).

        Each endpoint is projected onto its nearest segment; a temporary node
        is inserted there and connected to the segment endpoints, so the route
        can start/end anywhere along a road. Returns
        (lonlat_path, length_m, start_proj, goal_proj) or None.
        """
        sp = self.project_to_edge(slon, slat)
        gp = self.project_to_edge(glon, glat)
        if sp is None or gp is None:
            return None

        S, G = -1, -2
        try:
            self.coords[S] = (sp["lon"], sp["lat"])
            self.adj[S] = []
            self.coords[G] = (gp["lon"], gp["lat"])
            self.adj[G] = []
            # start: outgoing edges S -> u/v
            self.adj[S].append((sp["u"], sp["t"] * sp["L"], sp["way_id"]))
            self.adj[S].append((sp["v"], (1 - sp["t"]) * sp["L"], sp["way_id"]))
            # goal: incoming edges u/v -> G
            self.adj[gp["u"]].append((G, gp["t"] * gp["L"], gp["way_id"]))
            self.adj[gp["v"]].append((G, (1 - gp["t"]) * gp["L"], gp["way_id"]))
            # both points on the same segment: allow a direct connection
            if {sp["u"], sp["v"]} == {gp["u"], gp["v"]}:
                w = abs(sp["t"] - gp["t"]) * sp["L"]
                self.adj[S].append((G, w, sp["way_id"]))

            path, length = self.astar(S, G, respect_turns=respect_turns)
            if path is None:
                return None
            lonlat = [self.coords[n] for n in path]
            return lonlat, length, sp, gp
        finally:
            self.coords.pop(S, None)
            self.adj.pop(S, None)
            self.coords.pop(G, None)
            self.adj.pop(G, None)
            for node in (gp["u"], gp["v"]):
                if node in self.adj:
                    self.adj[node] = [
                        e for e in self.adj[node] if e[0] not in (S, G)
                    ]

    def plan_to_goal(self, slon, slat, glon, glat, entries=None,
                     off_network_m=5.0, search_radius_m=40.0,
                     offroad_weight=1.0, respect_turns=False,
                     max_candidates=8):
        """Plan between two points either of which may sit off the road network.

        A point inside a fenced area (plaza, courtyard) has no road under it.
        When start or goal is further than off_network_m from any road and
        entry points are known, the route uses the entry minimising

            offroad_weight * straight_access_leg + road_network_cost

        for that end. An entry authored off the road is kept as a via point so
        the robot is driven through the opening rather than handed a straight
        line from the roadside. The full path structure is:

            start -> (start_via) -> road_start -> ... -> road_end -> (via) -> goal

        where start_via / via are absent when the entry sits on the road.
        Off-network hops are never densified; Nav2 plans each hop locally.

        Returns a dict, or None when nothing is reachable:
            path               [(lon, lat), ...]
            length_m           total length including all off-network hops
            road_m             the on-road part alone (length_m minus every
                               off-network hop). ~0 means both ends attach to
                               the road at the same place, i.e. the route leaves
                               and re-enters through one opening.
            start_proj         road projection of the start-side entry (or start)
            goal_proj          road projection of the goal-side entry (or goal)
            road_start_index   index of the first on-road point; None if start on-network
            start_via_index    index of the start-side via entry; None if on the road
            road_end_index     index of the last on-road point; None if goal on-network
            via_index          index of the goal-side via entry; None if on the road
            start_entry_name   name of the chosen start entry (None if not used)
            entry_name         name of the chosen goal entry (None if not used)
            start_entry_lead_m road_proj -> start_via distance (0 when no via)
            entry_lead_m       road_proj -> goal_via distance (0 when no via)
            start_offroad_m    start -> start_via (or road_proj) distance
            offroad_m          goal_via (or road_proj) -> goal distance
            start_offnetwork_m start distance to the nearest road
            goal_offnetwork_m  goal distance to the nearest road
            start_entry_missing  start is off-network but no entry covers it
            entry_missing        goal is off-network but no entry covers it
        """
        sp0 = self.project_to_edge(slon, slat)
        gp0 = self.project_to_edge(glon, glat)
        if sp0 is None or gp0 is None:
            return None
        start_off_network = sp0["dist"]
        goal_off_network = gp0["dist"]

        def _plain(start_entry_missing=False, entry_missing=False):
            res = self.plan_points(slon, slat, glon, glat,
                                   respect_turns=respect_turns)
            if res is None:
                return None
            lonlat, length, sp, gp = res
            return {
                "path": lonlat, "length_m": length, "road_m": length,
                "start_proj": sp, "goal_proj": gp,
                "road_start_index": None, "start_via_index": None,
                "road_end_index": None, "via_index": None,
                "start_entry_name": None, "entry_name": None,
                "start_entry_lead_m": 0.0, "entry_lead_m": 0.0,
                "start_offroad_m": 0.0, "offroad_m": 0.0,
                "start_offnetwork_m": start_off_network,
                "goal_offnetwork_m": goal_off_network,
                "start_entry_missing": start_entry_missing,
                "entry_missing": entry_missing,
            }

        start_is_off = start_off_network > off_network_m and bool(entries)
        goal_is_off = goal_off_network > off_network_m and bool(entries)

        if not start_is_off and not goal_is_off:
            return _plain(
                start_entry_missing=start_off_network > off_network_m,
                entry_missing=goal_off_network > off_network_m,
            )

        # Pre-filter on the authored coordinate, with slack, only to bound how
        # many searches a campus-wide layer costs. The radius itself is enforced
        # below on the real access leg.
        def _prefilter(ref_lon, ref_lat):
            cands = []
            for e in entries:
                radius = e.get("radius_m") or search_radius_m
                rough = haversine(e["lon"], e["lat"], ref_lon, ref_lat)
                if rough <= radius + _ENTRY_PREFILTER_SLACK_M:
                    cands.append((rough, radius, e))
            cands.sort(key=lambda c: c[0])
            return cands[:max_candidates]

        goal_cands = _prefilter(glon, glat) if goal_is_off else []
        start_cands = _prefilter(slon, slat) if start_is_off else []

        def _offroad_legs(road_proj, entry, point_lon, point_lat):
            """Return (lead, straight) for one entry relative to an off-network point.

            lead     = road_proj -> entry (0 when the entry sits on the road)
            straight = entry (or road_proj) -> off-network point
            All off-network distances are measured from the road projection
            outwards so an over-long hop cannot slip through the radius check.
            """
            lead = haversine(road_proj["lon"], road_proj["lat"],
                             entry["lon"], entry["lat"])
            if lead < _ENTRY_VIA_MIN_M:
                lead = 0.0
                straight = haversine(road_proj["lon"], road_proj["lat"],
                                     point_lon, point_lat)
            else:
                straight = haversine(entry["lon"], entry["lat"],
                                     point_lon, point_lat)
            return lead, straight

        # ── goal off-network only ─────────────────────────────────────────────
        if goal_is_off and not start_is_off:
            best = None
            for _rough, radius, e in goal_cands:
                res = self.plan_points(slon, slat, e["lon"], e["lat"],
                                       respect_turns=respect_turns)
                if res is None:
                    continue
                lonlat, length, sp, gp = res
                lead, straight = _offroad_legs(gp, e, glon, glat)
                if straight > radius or lead > radius:
                    continue
                cost = length + offroad_weight * (lead + straight)
                if best is None or cost < best[0]:
                    best = (cost, lonlat, length, sp, gp, lead, straight, e)
            if best is None:
                return _plain(entry_missing=True)
            _cost, lonlat, road_len, sp, gp, lead, straight, e = best
            path = list(lonlat)
            road_end_index = len(path) - 1
            via_index = None
            if lead:
                via_index = len(path)
                path.append((e["lon"], e["lat"]))
            path.append((glon, glat))
            return {
                "path": path, "length_m": road_len + lead + straight,
                "road_m": road_len,
                "start_proj": sp, "goal_proj": gp,
                "road_start_index": None, "start_via_index": None,
                "road_end_index": road_end_index, "via_index": via_index,
                "start_entry_name": None, "entry_name": e.get("name"),
                "start_entry_lead_m": 0.0, "entry_lead_m": lead,
                "start_offroad_m": 0.0, "offroad_m": straight,
                "start_offnetwork_m": start_off_network,
                "goal_offnetwork_m": goal_off_network,
                "start_entry_missing": False, "entry_missing": False,
            }

        # ── start off-network only ────────────────────────────────────────────
        if start_is_off and not goal_is_off:
            best = None
            for _rough, radius, e in start_cands:
                res = self.plan_points(e["lon"], e["lat"], glon, glat,
                                       respect_turns=respect_turns)
                if res is None:
                    continue
                lonlat, length, sp, gp = res
                # sp is the road projection of the entry; measure the off-road
                # legs from it, the same way goal-side measures from gp.
                lead, straight = _offroad_legs(sp, e, slon, slat)
                if straight > radius or lead > radius:
                    continue
                cost = offroad_weight * (straight + lead) + length
                if best is None or cost < best[0]:
                    best = (cost, lonlat, length, sp, gp, lead, straight, e)
            if best is None:
                return _plain(start_entry_missing=True)
            _cost, lonlat, road_len, sp, gp, lead, straight, e = best
            # Build: start -> (start_via) -> road_start -> ... -> goal_proj
            path = [(slon, slat)]
            start_via_index = None
            if lead:
                start_via_index = len(path)
                path.append((e["lon"], e["lat"]))
            road_start_index = len(path)
            path.extend(lonlat)
            return {
                "path": path, "length_m": straight + lead + road_len,
                "road_m": road_len,
                "start_proj": sp, "goal_proj": gp,
                "road_start_index": road_start_index,
                "start_via_index": start_via_index,
                "road_end_index": None, "via_index": None,
                "start_entry_name": e.get("name"), "entry_name": None,
                "start_entry_lead_m": lead, "entry_lead_m": 0.0,
                "start_offroad_m": straight, "offroad_m": 0.0,
                "start_offnetwork_m": start_off_network,
                "goal_offnetwork_m": goal_off_network,
                "start_entry_missing": False, "entry_missing": False,
            }

        # ── both off-network ──────────────────────────────────────────────────
        best = None
        for _rough_s, radius_s, e_s in start_cands:
            for _rough_g, radius_g, e_g in goal_cands:
                res = self.plan_points(e_s["lon"], e_s["lat"],
                                       e_g["lon"], e_g["lat"],
                                       respect_turns=respect_turns)
                if res is None:
                    continue
                lonlat, length, sp, gp = res
                s_lead, s_straight = _offroad_legs(sp, e_s, slon, slat)
                g_lead, g_straight = _offroad_legs(gp, e_g, glon, glat)
                if s_straight > radius_s or s_lead > radius_s:
                    continue
                if g_straight > radius_g or g_lead > radius_g:
                    continue
                cost = (offroad_weight * (s_straight + s_lead)
                        + length
                        + offroad_weight * (g_lead + g_straight))
                if best is None or cost < best[0]:
                    best = (cost, lonlat, length, sp, gp,
                            s_lead, s_straight, e_s,
                            g_lead, g_straight, e_g)
        if best is None:
            return _plain(start_entry_missing=True, entry_missing=True)
        (_cost, lonlat, road_len, sp, gp,
         s_lead, s_straight, e_s, g_lead, g_straight, e_g) = best
        path = [(slon, slat)]
        start_via_index = None
        if s_lead:
            start_via_index = len(path)
            path.append((e_s["lon"], e_s["lat"]))
        road_start_index = len(path)
        path.extend(lonlat)
        road_end_index = len(path) - 1
        via_index = None
        if g_lead:
            via_index = len(path)
            path.append((e_g["lon"], e_g["lat"]))
        path.append((glon, glat))
        return {
            "path": path,
            "length_m": s_straight + s_lead + road_len + g_lead + g_straight,
            "road_m": road_len,
            "start_proj": sp, "goal_proj": gp,
            "road_start_index": road_start_index,
            "start_via_index": start_via_index,
            "road_end_index": road_end_index, "via_index": via_index,
            "start_entry_name": e_s.get("name"), "entry_name": e_g.get("name"),
            "start_entry_lead_m": s_lead, "entry_lead_m": g_lead,
            "start_offroad_m": s_straight, "offroad_m": g_straight,
            "start_offnetwork_m": start_off_network,
            "goal_offnetwork_m": goal_off_network,
            "start_entry_missing": False, "entry_missing": False,
        }

    def plan_auto(self, slon, slat, glon, glat, entries=None,
                  direct_max_m=40.0, always_direct_m=8.0,
                  detour_ratio_threshold=1.6, same_entry_road_m=2.0,
                  **plan_to_goal_kwargs):
        """Pick between following the road network and driving straight there.

        Routing along the roads is the safe default: the graph encodes topology
        the robot cannot see, so it never walks into a fenced dead end. But for a
        goal a few metres away — or one where the roads detour all the way around
        a block — snapping to the network is a pointless excursion.

        The only way to tell the two apart is to compare the two candidate routes,
        which needs the graph. That is why this lives here rather than in the
        caller: plan_to_goal already computes the road route (total length, the
        on-road part alone, and how far each end sits off the network), so the
        comparison is free.

        The governing idea: a direct drive is only sound when the straight line
        does not cross the boundary of a fenced area. Both ends inside the same
        one, or both out on the network, are fine. One of each is not — that line
        must cross, and an entry point is the only description we have of where
        crossing is allowed.

        The decision, in order:

          1. Both ends off-network and road_m ~ 0  ->  the route leaves and
             re-enters through a single opening, so the on-road leg is pure
             waste. Straight there if it fits the costmap; otherwise via the
             entry as a waypoint (see below).
          2. straight > direct_max_m               ->  road. A direct goal this
             far out is rejected by the planner, so there is nothing to weigh.
          3. no road route at all                  ->  direct, for lack of
             anything else (straight is already within direct_max_m here).
          4. exactly one end off-network           ->  road, unconditionally.
             The straight line would cross the enclosure boundary; the roads and
             the authored opening are the only sanctioned way through.
          5. straight <= always_direct_m           ->  direct. Snapping to a road
             and back costs more than just driving it.
          6. either end has entry_missing          ->  direct. That end is
             off-network with no opening authored, so plan_to_goal fell back to
             the nearest road, which may well be across a fence.
          7. road_total/straight >= threshold      ->  direct. The roads detour.
          8. otherwise                             ->  road.

        Rule 1 must precede rule 2: routing via the entry exists precisely to
        serve the case where the straight shot does not fit.

        Rule 4 must precede rules 5-7, which are all shortcuts that would
        otherwise send the robot across a boundary — a goal 7 m off the road is
        both "trivially close" and on the far side of a fence.

        Rule 6 divides by the *total* road route (length_m, access legs
        included), not the on-road part, so "drive out of one courtyard and into
        another" is counted against the road option the way it should be.

        Rule 1 keys off road_m rather than comparing the two entries by name:
        an entry name is neither unique nor required, and two distinct entries a
        metre apart should behave the same as one.

        The dict returned is plan_to_goal's, plus:
            mode             'road' | 'direct' | 'direct_via_entry'
            decision_reason  short tag naming the rule that fired
            straight_m       straight-line start -> goal
            road_total_m     length_m of the road route (None if unreachable)
            detour_ratio     road_total_m / straight_m (None if unreachable)

        For the two direct modes the path is trimmed to just the hops the caller
        has to drive, and every entry/off-road field is zeroed, so a caller that
        only knows plan_to_goal's shape keeps working:

            direct            [start, goal]         road_end_index=0
            direct_via_entry  [start, entry, goal]   road_start_index=road_end_index=1

        Marking the middle point as both the start and the end of the "on-road"
        span is what stops the caller from interpolating along either hop — both
        are off-network and may cut through the very fence they exist to skirt.

        Returns None when neither option is possible.
        """
        straight = haversine(slon, slat, glon, glat)
        plan = self.plan_to_goal(slon, slat, glon, glat, entries=entries,
                                 **plan_to_goal_kwargs)

        road_total = plan["length_m"] if plan else None
        ratio = (road_total / straight) if (plan and straight > 1e-6) else None
        # Same threshold plan_to_goal classified the ends with, so "off-network"
        # means the same thing in both places.
        off_network_m = plan_to_goal_kwargs.get("off_network_m", 5.0)
        start_off_network = plan["start_offnetwork_m"] if plan else None
        goal_off_network = plan["goal_offnetwork_m"] if plan else None

        def _decide(mode, reason, path, road_start_index, road_end_index,
                    entry=None):
            out = dict(plan) if plan else {}
            out.update({
                "path": path,
                "length_m": sum(
                    haversine(a[0], a[1], b[0], b[1])
                    for a, b in zip(path, path[1:])
                ),
                "road_m": 0.0,
                "road_start_index": road_start_index,
                "road_end_index": road_end_index,
                "start_via_index": None, "via_index": None,
                "start_entry_lead_m": 0.0, "entry_lead_m": 0.0,
                "start_offroad_m": 0.0, "offroad_m": 0.0,
                # Cleared so the caller does not warn about an unauthored
                # opening on a route that no longer touches the network.
                "start_entry_missing": False, "entry_missing": False,
                "start_entry_name": None,
                "entry_name": entry.get("name") if entry else None,
                "mode": mode, "decision_reason": reason,
                "straight_m": straight, "road_total_m": road_total,
                "detour_ratio": ratio,
            })
            # plan_to_goal was skipped entirely (no road route): fill in the keys
            # it would have provided so every mode returns the same shape.
            out.setdefault("start_proj", None)
            out.setdefault("goal_proj", None)
            out.setdefault("start_offnetwork_m", None)
            out.setdefault("goal_offnetwork_m", None)
            return out

        def _road(reason):
            out = dict(plan)
            out.update({
                "mode": "road", "decision_reason": reason,
                "straight_m": straight, "road_total_m": road_total,
                "detour_ratio": ratio,
            })
            return out

        start = (slon, slat)
        goal = (glon, glat)

        # -- 1. one opening serves both ends -------------------------------------
        # Both ends hang off the network and the on-road leg between their
        # attachment points is ~nothing: they share an opening, so the road
        # excursion is out-and-straight-back.
        if (plan is not None
                and plan["road_start_index"] is not None
                and plan["road_end_index"] is not None
                and plan["road_m"] <= same_entry_road_m):
            if straight <= direct_max_m:
                return _decide("direct", "single_entry_both_ends",
                               [start, goal], None, 0)
            # Too far to hand over in one piece. The entry is a legal waypoint
            # for exactly this: plan_to_goal only kept it if both of its access
            # legs were inside search_radius_m, so each half is drivable even
            # though the whole is not.
            via = plan["path"][plan["road_start_index"]]
            if plan["start_via_index"] is not None:
                via = plan["path"][plan["start_via_index"]]
            return _decide("direct_via_entry", "single_entry_via_waypoint",
                           [start, via, goal], 1, 1,
                           entry={"name": plan["start_entry_name"]})

        # -- 2. beyond the costmap: the direct option does not exist -------------
        if straight > direct_max_m:
            if plan is None:
                return None
            return _road("beyond_direct_max")

        # -- 3. no road route ---------------------------------------------------
        if plan is None:
            return _decide("direct", "no_road_route", [start, goal], None, 0)

        # -- 4. one end inside, one end outside: the roads are the only way ------
        # A straight line between them has to cross the boundary of whatever the
        # off-network end sits inside — a fence, hedge, kerb, planting. That
        # boundary is exactly what an entry point exists to describe: it says
        # where crossing is allowed. Going direct here bypasses the one piece of
        # information we have about it, so no shortcut rule below may override
        # this. (Both ends outside is different: rule 1 already covers the case
        # where they share an opening, and then the straight line never leaves
        # the enclosure.)
        #
        # This holds even with no entry authored. The road route then stops at the
        # goal's projection, a few metres short, and plan_to_goal already reports
        # entry_missing so the caller can say to add one — better than driving at
        # an unmapped fence.
        if (start_off_network is not None and goal_off_network is not None
                and (start_off_network > off_network_m)
                != (goal_off_network > off_network_m)):
            return _road("crosses_network_boundary")

        # -- 5. close enough that snapping to a road is the detour ---------------
        if straight <= always_direct_m:
            return _decide("direct", "trivially_close", [start, goal], None, 0)

        # -- 6. the road plan is not trustworthy --------------------------------
        if plan["entry_missing"] or plan["start_entry_missing"]:
            return _decide("direct", "entry_missing", [start, goal], None, 0)

        # -- 7. the roads detour ------------------------------------------------
        if ratio is not None and ratio >= detour_ratio_threshold:
            return _decide("direct", "road_detours", [start, goal], None, 0)

        # -- 8. default: follow the roads ---------------------------------------
        return _road("road_is_direct_enough")

    def astar(self, start, goal, respect_turns=False):
        """Return (node_id_path, length_m) or (None, inf).

        If respect_turns and self.restrictions is set, turn restrictions
        (no_*/only_*) are enforced via a turn-aware search.
        """
        if respect_turns and getattr(self, "restrictions", None):
            return self._astar_turns(start, goal)
        if start == goal:
            return [start], 0.0
        glon, glat = self.coords[goal]

        def h(n):
            lon, lat = self.coords[n]
            return haversine(lon, lat, glon, glat)

        g = {start: 0.0}
        came = {}
        pq = [(h(start), start)]
        visited = set()
        while pq:
            _, cur = heappop(pq)
            if cur == goal:
                path = [cur]
                while cur in came:
                    cur = came[cur]
                    path.append(cur)
                return path[::-1], g[goal]
            if cur in visited:
                continue
            visited.add(cur)
            base = g[cur]
            for nb, w, _wid in self.adj[cur]:
                ng = base + w
                if ng < g.get(nb, math.inf):
                    g[nb] = ng
                    came[nb] = cur
                    heappush(pq, (ng + h(nb), nb))
        return None, math.inf

    def _turn_allowed(self, via, way_in, way_out):
        """False if turning at `via` from way_in to way_out is restricted."""
        rs = self.restrictions.get(via)
        if not rs:
            return True
        for r in rs:
            if way_in not in r["from"]:
                continue
            if r["kind"] == "no":
                if way_out in r["to"]:
                    return False
            else:  # only_* : from this way, only `to` ways are permitted
                if way_out not in r["to"]:
                    return False
        return True

    def _astar_turns(self, start, goal):
        """Turn-aware A*. State = (node, incoming_way_id)."""
        if start == goal:
            return [start], 0.0
        glon, glat = self.coords[goal]

        def h(n):
            lon, lat = self.coords[n]
            return haversine(lon, lat, glon, glat)

        start_state = (start, None)
        g = {start_state: 0.0}
        came = {}
        pq = [(h(start), start_state)]
        visited = set()
        while pq:
            _, state = heappop(pq)
            cur, way_in = state
            if cur == goal:
                path = [cur]
                s = state
                while s in came:
                    s = came[s]
                    path.append(s[0])
                return path[::-1], g[state]
            if state in visited:
                continue
            visited.add(state)
            base = g[state]
            for nb, w, way_out in self.adj[cur]:
                if way_in is not None and not self._turn_allowed(cur, way_in, way_out):
                    continue
                ns = (nb, way_out)
                ng = base + w
                if ng < g.get(ns, math.inf):
                    g[ns] = ng
                    came[ns] = state
                    heappush(pq, (ng + h(nb), ns))
        return None, math.inf

    def path_lonlat(self, node_path):
        return [self.coords[n] for n in node_path]

    # -- connectivity diagnostics -------------------------------------------
    def components(self):
        seen = set()
        comps = []
        for s in self.coords:
            if s in seen:
                continue
            stack = [s]
            comp = []
            seen.add(s)
            while stack:
                u = stack.pop()
                comp.append(u)
                for v, _w, _wid in self.adj[u]:
                    if v not in seen:
                        seen.add(v)
                        stack.append(v)
            comps.append(comp)
        comps.sort(key=len, reverse=True)
        return comps


# ---------------------------------------------------------------------------
# Builder
# ---------------------------------------------------------------------------
def build_graph(
    gpkg_path: str,
    table: str = "route_edges",
    mode: str = "walk",
    respect_oneway: bool = False,
    allow_private: bool = True,
):
    """Build a RouteGraph from the GeoPackage.

    mode="walk": keep WALKABLE_HIGHWAYS, ignore oneway (pedestrian/robot).
    mode="car" : keep drivable highways, honor oneway when respect_oneway.
    mode="all" : keep every highway-tagged edge.
    allow_private: keep access=private/no roads (campus internal roads).
    """
    con = sqlite3.connect(gpkg_path)
    cur = con.execute(
        f"SELECT geom, highway, oneway, foot, access FROM {table} "
        f"WHERE highway IS NOT NULL AND highway != ''"
    )
    g = RouteGraph()
    kept = skipped = 0
    for geom, highway, oneway, foot, access in cur:
        if not _keep(highway, foot, access, mode, allow_private):
            skipped += 1
            continue
        lines = _parse_gpkg_blob(geom)
        if not lines:
            continue
        kept += 1
        ow = respect_oneway and mode == "car" and (oneway in ("yes", "true", "1"))
        rev = oneway == "-1"
        for pts in lines:
            prev = None
            seq = pts[::-1] if rev else pts
            for lon, lat in seq:
                nid = g._node(lon, lat)
                if prev is not None:
                    plon, plat = g.coords[prev]
                    w = haversine(plon, plat, lon, lat)
                    if w > 0:
                        g._add_edge(prev, nid, w, oneway=ow)
                prev = nid
    con.close()
    g.finalize()
    g.stats = {"edges_kept": kept, "edges_skipped": skipped, "nodes": len(g.coords)}
    return g


def _keep(highway, foot, access, mode, allow_private=True):
    if foot in ("no", "private"):
        return False
    if mode == "all":
        return highway not in ("proposed", "construction")
    if mode == "walk":
        if highway in BLOCKED_HIGHWAYS:
            # still allow if explicitly footable
            return foot in ("yes", "designated", "permissive")
        if highway in WALKABLE_HIGHWAYS:
            # allow_private keeps access=private/no roads (campus internal roads
            # the robot owns); turn off to honor strict public access.
            if allow_private:
                return True
            return access not in ("no", "private") or foot in ("yes", "designated")
        return False
    if mode == "car":
        return highway not in {
            "footway", "path", "steps", "pedestrian", "corridor", "cycleway",
            "construction", "proposed",
        }
    return True


# ---------------------------------------------------------------------------
# Entry points: hand-authored "the robot may leave the road here" markers
# ---------------------------------------------------------------------------
# A goal on a plaza or in a courtyard has no road under it, and the fence around
# it is not mapped, so geometry alone cannot tell where entering is allowed.
# Two authoring routes feed the same candidate list:
#   - a GeoPackage POINT layer (QGIS: one click per opening)
#   - ways tagged campus_entry=yes in the pbf (a stub drawn from the sidewalk
#     into the area; its dangling end is the anchor)
# How far past the radius an authored point may sit before it is not even worth
# a search. Only the pre-filter uses it; the radius is enforced on the real leg.
# Points authored more than this far off the road may not be considered at all.
_ENTRY_PREFILTER_SLACK_M = 15.0
# Below this the authored point is effectively on the road, and adding it as a
# via point would only produce a near-zero-length goal for Nav2 to chase.
_ENTRY_VIA_MIN_M = 1.0
_ENTRY_NAME_COLS = ("name", "Name", "label", "title")
_ENTRY_RADIUS_COLS = ("radius_m", "radius", "range_m")
_ENTRY_GEOM_COLS = ("geom", "geometry", "the_geom")
_TRUTHY = ("yes", "true", "1")


def _parse_gpkg_point(blob):
    """Return (lon, lat) from a GeoPackage POINT blob, or None."""
    for pts in _parse_gpkg_blob(blob) or []:
        if pts:
            return float(pts[0][0]), float(pts[0][1])
    return None


def load_entry_points(gpkg_path: str, table: str = "entries"):
    """Load entry points from a GeoPackage POINT layer.

    Returns [{"lon", "lat", "name", "radius_m", "source"}]. A missing file,
    missing table or missing optional column is not an error: the feature stays
    inactive until the layer is authored, so nothing changes for setups that
    never create it. Optional columns: name (label), radius_m (per-entry
    override of the global search radius).
    """
    if not gpkg_path or not os.path.exists(gpkg_path):
        return []
    con = sqlite3.connect(f"file:{gpkg_path}?mode=ro", uri=True)
    try:
        cols = [r[1] for r in con.execute(f'PRAGMA table_info("{table}")')]
        if not cols:
            return []
        geom_col = None
        try:
            row = con.execute(
                "SELECT column_name FROM gpkg_geometry_columns WHERE table_name = ?",
                (table,),
            ).fetchone()
            if row and row[0] in cols:
                geom_col = row[0]
        except sqlite3.Error:
            pass
        if geom_col is None:
            geom_col = next((c for c in _ENTRY_GEOM_COLS if c in cols), None)
        if geom_col is None:
            return []
        name_col = next((c for c in _ENTRY_NAME_COLS if c in cols), None)
        radius_col = next((c for c in _ENTRY_RADIUS_COLS if c in cols), None)
        sel = [geom_col] + [c for c in (name_col, radius_col) if c]
        query = ", ".join(f'"{c}"' for c in sel)
        out = []
        for i, row in enumerate(con.execute(f'SELECT {query} FROM "{table}"')):
            lonlat = _parse_gpkg_point(row[0])
            if lonlat is None:
                continue
            vals = dict(zip(sel, row))
            radius = vals.get(radius_col) if radius_col else None
            try:
                radius = float(radius) if radius not in (None, "") else None
            except (TypeError, ValueError):
                radius = None
            name = vals.get(name_col) if name_col else None
            out.append({
                "lon": lonlat[0], "lat": lonlat[1],
                "name": str(name) if name not in (None, "") else f"entry{i + 1}",
                "radius_m": radius,
                "source": "gpkg",
            })
        return out
    except sqlite3.Error:
        return []
    finally:
        con.close()


def _node_id_at(g, lon, lat, tol_m=0.5):
    """Graph node id at a coordinate, for either node-id scheme.

    pbf graphs key nodes by OSM id, merged/gpkg graphs by quantized coordinate,
    so look the coordinate up both ways.
    """
    nid = g._key_to_id.get(_key(lon, lat))
    if nid is not None:
        return nid
    if getattr(g, "_id_coords", None) is None or not len(g.coords):
        return None
    nid = g.nearest_node(lon, lat)
    nlon, nlat = g.coords[nid]
    return nid if haversine(lon, lat, nlon, nlat) <= tol_m else None


def load_stub_entry_points(pbf_path: str, graph, tag: str = "campus_entry"):
    """Entry anchors from stub ways tagged campus_entry=yes.

    A stub is drawn in QGIS from the sidewalk into the fenced area; the end that
    leads nowhere else (lowest graph degree) is the anchor. Ties resolve to the
    way's last vertex, matching the "draw outward-in" convention.
    """
    if not pbf_path or not os.path.exists(pbf_path):
        return []
    try:
        import osmium
    except ImportError:
        return []
    stubs = []

    class _Handler(osmium.SimpleHandler):
        def way(self, w):
            tags = {t.k: t.v for t in w.tags}
            if str(tags.get(tag, "")).strip().lower() not in _TRUTHY:
                return
            pts = [
                (nd.location.lon, nd.location.lat)
                for nd in w.nodes
                if nd.location.valid()
            ]
            if len(pts) >= 2:
                stubs.append((tags.get("name") or f"stub{w.id}", pts))

    _Handler().apply_file(pbf_path, locations=True)
    out = []
    for name, pts in stubs:
        ends = (pts[0], pts[-1])
        degrees = []
        for lon, lat in ends:
            nid = _node_id_at(graph, lon, lat)
            degrees.append(len(graph.adj.get(nid, ())) if nid is not None else 0)
        anchor = ends[0] if degrees[0] < degrees[1] else ends[1]
        out.append({
            "lon": anchor[0], "lat": anchor[1], "name": str(name),
            "radius_m": None, "source": "stub",
        })
    return out


# ---------------------------------------------------------------------------
# Local ENU projection (for ROS map frame)
# ---------------------------------------------------------------------------
class ENU:
    """Equirectangular local tangent plane around a datum (good for a few km)."""

    def __init__(self, lat0, lon0):
        self.lat0 = lat0
        self.lon0 = lon0
        self.cos0 = math.cos(math.radians(lat0))

    def to_xy(self, lon, lat):
        east = math.radians(lon - self.lon0) * EARTH_R * self.cos0
        north = math.radians(lat - self.lat0) * EARTH_R
        return east, north

    def to_lonlat(self, east, north):
        lat = self.lat0 + math.degrees(north / EARTH_R)
        lon = self.lon0 + math.degrees(east / (EARTH_R * self.cos0))
        return lon, lat


def _build_restrictions(restr_raw):
    """Index restriction relations by their via-node id.

    Only node-via restrictions are supported (the common case); via-way
    restrictions are skipped.
    """
    out = {}
    for _rid, rtype, members in restr_raw:
        via = None
        froms, tos = set(), set()
        for mtype, ref, role in members:
            if role == "via" and mtype == "n":
                via = ref
            elif role == "from" and mtype == "w":
                froms.add(ref)
            elif role == "to" and mtype == "w":
                tos.add(ref)
        if via is None or not froms or not tos:
            continue
        kind = "only" if rtype.startswith("only_") else "no"
        out.setdefault(via, []).append(
            {"kind": kind, "from": froms, "to": tos, "rtype": rtype}
        )
    return out


# ---------------------------------------------------------------------------
# OSM PBF -> graph directly via pyosmium (preferred when pyosmium is available)
# ---------------------------------------------------------------------------
def build_graph_from_pbf(
    pbf_path: str,
    mode: str = "walk",
    respect_oneway: bool = False,
    want_restrictions: bool = True,
    allow_private: bool = True,
):
    """Build a RouteGraph straight from an .osm.pbf using pyosmium.

    Requires `osmium` (pyosmium) importable in the running interpreter.
    Reads the pbf directly; uses OSM node ids as graph identity.
    allow_private: keep access=private/no roads (campus internal roads).
    """
    import osmium  # local import: only needed for this path

    g = RouteGraph()
    counters = {"kept": 0, "skipped": 0}
    restr_raw = []

    def _tag(tags, key, default=None):
        return tags[key] if key in tags else default

    class _Handler(osmium.SimpleHandler):
        def way(self, w):
            tags = w.tags
            hw = _tag(tags, "highway")
            if not hw:
                return
            if not _keep(hw, _tag(tags, "foot"), _tag(tags, "access"), mode,
                         allow_private):
                counters["skipped"] += 1
                return
            counters["kept"] += 1
            wid = w.id
            ow = (
                respect_oneway and mode == "car"
                and _tag(tags, "oneway") in ("yes", "true", "1")
            )
            seq = list(w.nodes)
            if _tag(tags, "oneway") == "-1":
                seq = seq[::-1]
            prev = None
            for nd in seq:
                if not nd.location.valid():
                    prev = None  # missing location splits the way
                    continue
                nid = nd.ref
                if nid not in g.coords:
                    g.coords[nid] = (nd.location.lon, nd.location.lat)
                    g.adj[nid] = []
                if prev is not None:
                    plon, plat = g.coords[prev]
                    lon, lat = g.coords[nid]
                    wlen = haversine(plon, plat, lon, lat)
                    if wlen > 0:
                        g._add_edge(prev, nid, wlen, way_id=wid, oneway=ow)
                prev = nid

        def relation(self, r):
            if not want_restrictions:
                return
            if _tag(r.tags, "type") != "restriction":
                return
            members = [(str(m.type)[0].lower(), m.ref, m.role) for m in r.members]
            restr_raw.append((r.id, _tag(r.tags, "restriction", ""), members))

    _Handler().apply_file(pbf_path, locations=True)

    if want_restrictions:
        g.restrictions = _build_restrictions(restr_raw)
    g.finalize()
    g.stats = {
        "ways_kept": counters["kept"], "ways_skipped": counters["skipped"],
        "nodes": len(g.coords), "restrictions": len(g.restrictions),
    }
    return g


# ---------------------------------------------------------------------------
# PBF + edited GPKG overlay (QGIS export replaces geometries by osm_id)
# ---------------------------------------------------------------------------
def _load_gpkg_highway_ways(gpkg_path, table="osmfile"):
    """Load highway ways from a QGIS OSM export GPKG, keyed by osm_id string."""
    con = sqlite3.connect(gpkg_path)
    cur = con.execute(
        f"SELECT osm_id, geom, highway, oneway, foot, access FROM {table} "
        f"WHERE highway IS NOT NULL AND highway != '' AND osm_id IS NOT NULL"
    )
    out = {}
    for osm_id, geom, highway, oneway, foot, access in cur:
        lines = _parse_gpkg_blob(geom)
        if not lines:
            continue
        out[str(osm_id)] = {
            "tags": {
                "highway": highway,
                "oneway": oneway,
                "foot": foot,
                "access": access,
            },
            "lines": lines,
        }
    con.close()
    return out


def _add_way_linestring(g, wid, tags, pts, mode, respect_oneway, allow_private):
    """Add one linestring to graph g using coordinate-quantized nodes."""
    hw = tags.get("highway")
    if not hw or not _keep(hw, tags.get("foot"), tags.get("access"), mode, allow_private):
        return False
    try:
        way_id = int(wid)
    except (TypeError, ValueError):
        way_id = None
    ow = (
        respect_oneway and mode == "car"
        and tags.get("oneway") in ("yes", "true", "1")
    )
    seq = list(pts)
    if tags.get("oneway") == "-1":
        seq = seq[::-1]
    prev = None
    for lon, lat in seq:
        nid = g._node(lon, lat)
        if prev is not None:
            plon, plat = g.coords[prev]
            wlen = haversine(plon, plat, lon, lat)
            if wlen > 0:
                g._add_edge(prev, nid, wlen, way_id=way_id, oneway=ow)
        prev = nid
    return True


def build_graph_merged(
    pbf_path: str,
    gpkg_overlay: str,
    gpkg_table: str = "osmfile",
    mode: str = "walk",
    respect_oneway: bool = False,
    want_restrictions: bool = True,
    allow_private: bool = True,
):
    """Merge base network from pbf with QGIS-edited geometries in a GPKG.

    For every osm_id present in the GPKG, the edited geometry/tags replace the
    pbf version. Remaining pbf ways are kept as-is. Uses coordinate-quantized
    nodes so edited vertices connect to the rest of the network.
    """
    import osmium

    overlay = _load_gpkg_highway_ways(gpkg_overlay, gpkg_table)
    overlay_ids = set(overlay)

    g = RouteGraph()
    stats = {"pbf": 0, "gpkg_override": 0, "skipped": 0}
    restr_raw = []
    applied_overlay = set()

    def _tag(tags, key, default=None):
        return tags.get(key, default)

    def _tags_dict(w):
        return {t.k: t.v for t in w.tags}

    class _Handler(osmium.SimpleHandler):
        def way(self, w):
            wid = str(w.id)
            if wid in overlay_ids:
                gw = overlay[wid]
                for line in gw["lines"]:
                    if _add_way_linestring(
                        g, wid, gw["tags"], line, mode, respect_oneway, allow_private
                    ):
                        stats["gpkg_override"] += 1
                applied_overlay.add(wid)
                return
            tags = _tags_dict(w)
            if "highway" not in tags:
                return
            pts = [
                (nd.location.lon, nd.location.lat)
                for nd in w.nodes
                if nd.location.valid()
            ]
            if len(pts) < 2:
                return
            if _add_way_linestring(
                g, wid, tags, pts, mode, respect_oneway, allow_private
            ):
                stats["pbf"] += 1
            else:
                stats["skipped"] += 1

        def relation(self, r):
            if not want_restrictions:
                return
            rtags = _tags_dict(r)
            if _tag(rtags, "type") != "restriction":
                return
            members = [(str(m.type)[0].lower(), m.ref, m.role) for m in r.members]
            restr_raw.append((r.id, _tag(rtags, "restriction", ""), members))

    _Handler().apply_file(pbf_path, locations=True)

    for wid, gw in overlay.items():
        if wid in applied_overlay:
            continue
        for line in gw["lines"]:
            if _add_way_linestring(
                g, wid, gw["tags"], line, mode, respect_oneway, allow_private
            ):
                stats["gpkg_override"] += 1

    if want_restrictions:
        g.restrictions = _build_restrictions(restr_raw)
    g.finalize()
    g.stats = {
        "ways_pbf": stats["pbf"],
        "ways_gpkg_override": stats["gpkg_override"],
        "ways_skipped": stats["skipped"],
        "gpkg_overlay_count": len(overlay),
        "nodes": len(g.coords),
        "restrictions": len(g.restrictions),
    }
    return g


def _pt_eq(a, b):
    return _key(a[0], a[1]) == _key(b[0], b[1])


def _close_polygon(pts):
    """Return a closed ring (first point repeated at end) or None if too small."""
    if len(pts) < 3:
        return None
    if not _pt_eq(pts[0], pts[-1]):
        pts = list(pts) + [pts[0]]
    return pts


def _chain_way_segments(segments):
    """Join OSM way line segments sharing endpoints into closed ring(s)."""
    segs = [list(s) for s in segments if len(s) >= 2]
    rings = []
    while segs:
        ring = segs.pop(0)
        changed = True
        while changed:
            changed = False
            for i, seg in enumerate(segs):
                if _pt_eq(ring[-1], seg[0]):
                    ring.extend(seg[1:])
                    segs.pop(i)
                    changed = True
                    break
                if _pt_eq(ring[-1], seg[-1]):
                    ring.extend(reversed(seg[:-1]))
                    segs.pop(i)
                    changed = True
                    break
                if _pt_eq(ring[0], seg[-1]):
                    ring = seg[:-1] + ring
                    segs.pop(i)
                    changed = True
                    break
                if _pt_eq(ring[0], seg[0]):
                    ring = list(reversed(seg[1:])) + ring
                    segs.pop(i)
                    changed = True
                    break
        closed = _close_polygon(ring)
        if closed:
            rings.append(closed)
    return rings


@dataclass
class BuildingFootprint:
    """Closed lon/lat outer ring + inner rings (holes) + extrusion height (m)."""
    ring: list[tuple[float, float]]
    height_m: float
    holes: list[list[tuple[float, float]]] = field(default_factory=list)


def _is_building_area(tags: dict) -> bool:
    """True when tags describe a building footprint (not campus/landuse outline)."""
    if "building" not in tags:
        return False
    b = str(tags["building"]).strip().lower()
    if b in ("no", "none", "false", "0"):
        return False
    if tags.get("area") == "no":
        return False
    return True


def parse_building_height(
    tags: dict,
    default_m: float = 12.0,
    level_height_m: float = 3.0,
) -> float:
    """Estimate building height from common OSM tags."""
    for key in ("height", "building:height"):
        if key in tags:
            m = re.search(r"([\d.]+)", str(tags[key]))
            if m:
                return max(3.0, min(float(m.group(1)), 200.0))
    if "building:levels" in tags:
        m = re.search(r"([\d.]+)", str(tags["building:levels"]))
        if m:
            return max(3.0, min(float(m.group(1)) * level_height_m, 200.0))
    btype = tags.get("building", "yes")
    # rough defaults by type when no height tag
    type_defaults = {
        "house": 9.0, "residential": 12.0, "apartments": 18.0,
        "commercial": 15.0, "industrial": 10.0, "warehouse": 8.0,
        "school": 12.0, "university": 15.0, "hospital": 18.0,
        "garage": 3.5, "shed": 3.0, "roof": 4.0,
    }
    return type_defaults.get(btype, default_m)


def load_building_footprints(
    pbf_path: str,
    default_height_m: float = 12.0,
    level_height_m: float = 3.0,
) -> list[BuildingFootprint]:
    """Load building footprints with height from an OSM PBF."""
    import osmium

    way_pts: dict[int, list[tuple[float, float]]] = {}
    way_tags: dict[int, dict] = {}
    mp_member_ids: set[int] = set()
    # (outer_way_ids, inner_way_ids, tags)
    mp_groups: list[tuple[list[int], list[int], dict]] = []

    class _Handler(osmium.SimpleHandler):
        def way(self, w):
            tags = {t.k: t.v for t in w.tags}
            pts = [
                (nd.location.lon, nd.location.lat)
                for nd in w.nodes
                if nd.location.valid()
            ]
            if len(pts) >= 2:
                way_pts[w.id] = pts
            if _is_building_area(tags):
                way_tags[w.id] = tags

        def relation(self, r):
            tags = {t.k: t.v for t in r.tags}
            if tags.get("type") != "multipolygon" or not _is_building_area(tags):
                return
            outer_ids, inner_ids = [], []
            for m in r.members:
                if m.type != "w":
                    continue
                mp_member_ids.add(m.ref)
                if m.role == "outer":
                    outer_ids.append(m.ref)
                elif m.role == "inner":
                    inner_ids.append(m.ref)
            if outer_ids:
                mp_groups.append((outer_ids, inner_ids, tags))

    _Handler().apply_file(pbf_path, locations=True)

    footprints: list[BuildingFootprint] = []

    for wid, pts in way_pts.items():
        if wid in mp_member_ids or wid not in way_tags:
            continue
        closed = _close_polygon(pts)
        if closed:
            h = parse_building_height(
                way_tags[wid], default_m=default_height_m, level_height_m=level_height_m,
            )
            footprints.append(BuildingFootprint(closed, h))

    for outer_ids, inner_ids, tags in mp_groups:
        outer_segs = [way_pts[wid] for wid in outer_ids if wid in way_pts]
        inner_segs = [way_pts[wid] for wid in inner_ids if wid in way_pts]
        outer_rings = _chain_way_segments(outer_segs)
        inner_rings = _chain_way_segments(inner_segs)
        h = parse_building_height(
            tags, default_m=default_height_m, level_height_m=level_height_m,
        )
        for ring in outer_rings:
            # assign holes whose first vertex falls inside this outer ring
            holes = [
                hr for hr in inner_rings
                if _point_in_ring(hr[0], ring)
            ] if len(outer_rings) > 1 else list(inner_rings)
            footprints.append(BuildingFootprint(ring, h, holes))

    return footprints


def _point_in_ring(pt, ring) -> bool:
    """Ray-casting point-in-polygon test (ring may be closed or open)."""
    x, y = pt
    pts = ring[:-1] if len(ring) > 1 and ring[0] == ring[-1] else ring
    inside = False
    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            xint = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            if x < xint:
                inside = not inside
    return inside


def load_building_polygons(pbf_path: str) -> list[list[tuple[float, float]]]:
    """Backward-compatible wrapper: rings only, no height."""
    return [fp.ring for fp in load_building_footprints(pbf_path)]


def building_footprints_enu(footprints: list[BuildingFootprint], enu: "ENU"):
    """Convert footprints to (outer_ring, height_m, holes) in ENU meters."""
    out = []
    for fp in footprints:
        enu_ring = [enu.to_xy(lon, lat) for lon, lat in fp.ring]
        if len(enu_ring) < 4:
            continue
        enu_holes = []
        for hole in fp.holes:
            eh = [enu.to_xy(lon, lat) for lon, lat in hole]
            if len(eh) >= 4:
                enu_holes.append(eh)
        out.append((enu_ring, fp.height_m, enu_holes))
    return out


def building_polygons_enu(polygons, enu: "ENU"):
    """Convert closed lon/lat rings to closed ENU rings."""
    out = []
    for ring in polygons:
        enu_ring = [enu.to_xy(lon, lat) for lon, lat in ring]
        if len(enu_ring) >= 4:
            out.append(enu_ring)
    return out


def _ring_signed_area_xy(pts) -> float:
    """Signed area in map xy; positive = counter-clockwise."""
    ring = pts[:-1] if len(pts) > 1 and pts[0] == pts[-1] else list(pts)
    area = 0.0
    for i in range(len(ring)):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % len(ring)]
        area += x0 * y1 - x1 * y0
    return area * 0.5


def _ensure_ccw_ring(ring):
    """Force outer ring to CCW (RViz lights front faces; CW rings look black)."""
    pts = ring[:-1] if len(ring) > 1 and ring[0] == ring[-1] else list(ring)
    if _ring_signed_area_xy(pts) < 0:
        pts = list(reversed(pts))
    return _close_polygon(pts)


def _open_ring(ring):
    """Strip a repeated closing vertex if present."""
    return ring[:-1] if len(ring) > 1 and ring[0] == ring[-1] else list(ring)


def _orient(ring, ccw=True):
    pts = _open_ring(ring)
    a = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        a += x0 * y1 - x1 * y0
    is_ccw = a > 0
    if is_ccw != ccw:
        pts = list(reversed(pts))
    return pts


def _tri_cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _point_in_tri(p, a, b, c):
    d1 = _tri_cross(p, a, b)
    d2 = _tri_cross(p, b, c)
    d3 = _tri_cross(p, c, a)
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)


def _point_in_tri_strict(p, a, b, c):
    """True only if p is strictly inside triangle abc (boundary excluded)."""
    d1 = _tri_cross(p, a, b)
    d2 = _tri_cross(p, b, c)
    d3 = _tri_cross(p, c, a)
    if d1 == 0 or d2 == 0 or d3 == 0:
        return False
    has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
    has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)
    return not (has_neg and has_pos)


def _bridge_hole(outer, hole):
    """Merge a CW hole into a CCW outer ring via a visible bridge (Eberly)."""
    mi = max(range(len(hole)), key=lambda i: hole[i][0])
    M = hole[mi]
    best_x = math.inf
    inter = None
    edge_i = None
    for i in range(len(outer)):
        a = outer[i]
        b = outer[(i + 1) % len(outer)]
        if (a[1] > M[1]) == (b[1] > M[1]):
            continue
        t = (M[1] - a[1]) / (b[1] - a[1])
        x = a[0] + t * (b[0] - a[0])
        if x >= M[0] and x < best_x:
            best_x = x
            inter = (x, M[1])
            edge_i = i
    if inter is None:
        return outer  # hole not enclosed; skip bridging
    i, j = edge_i, (edge_i + 1) % len(outer)
    p_idx = i if outer[i][0] > outer[j][0] else j
    P = outer[p_idx]
    # If reflex vertices lie inside triangle (M, inter, P), bridge to the
    # one with smallest angle to the +x ray (closest to M) instead.
    best_ang = -1.0
    cand = p_idx
    for k in range(len(outer)):
        Q = outer[k]
        if Q == P:
            continue
        if _point_in_tri(Q, M, inter, P):
            dx = Q[0] - M[0]
            dy = Q[1] - M[1]
            d = math.hypot(dx, dy)
            if d == 0:
                continue
            ang = dx / d
            if ang > best_ang:
                best_ang = ang
                cand = k
    n = len(hole)
    hole_loop = [hole[(mi + s) % n] for s in range(n)] + [hole[mi]]
    return outer[:cand + 1] + hole_loop + outer[cand:]


def _ear_clip(poly):
    """Ear-clip a simple CCW polygon (no holes) -> list of (a,b,c) triangles."""
    pts = list(poly)
    if len(pts) < 3:
        return []
    idx = list(range(len(pts)))
    tris = []
    guard = 0
    limit = 3 * len(idx) * len(idx) + 10
    while len(idx) > 3 and guard < limit:
        guard += 1
        clipped = False
        m = len(idx)
        for k in range(m):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % m]
            a, b, c = pts[i0], pts[i1], pts[i2]
            if _tri_cross(a, b, c) <= 0:
                continue  # reflex or degenerate
            ok = True
            for j in idx:
                if j in (i0, i1, i2):
                    continue
                q = pts[j]
                # skip vertices coincident with the ear corners (bridge dupes)
                if q == a or q == b or q == c:
                    continue
                if _point_in_tri_strict(q, a, b, c):
                    ok = False
                    break
            if ok:
                tris.append((a, b, c))
                idx.pop(k)
                clipped = True
                break
        if not clipped:
            break
    if len(idx) == 3:
        tris.append((pts[idx[0]], pts[idx[1]], pts[idx[2]]))
    return tris


def _triangulate_with_holes(outer, holes):
    """Triangulate a polygon with optional holes -> list of (a,b,c) triangles."""
    poly = _orient(outer, ccw=True)
    if holes:
        prepared = [_orient(h, ccw=False) for h in holes if len(_open_ring(h)) >= 3]
        prepared.sort(key=lambda h: max(p[0] for p in h), reverse=True)
        for hole in prepared:
            poly = _bridge_hole(poly, hole)
    return _ear_clip(poly)


def extrude_ring_triangles(ring, height_m: float, holes=None):
    """Extrude outer ring (+ holes) to walls + roof; returns flat x,y,z tuples."""
    if height_m <= 0:
        return []
    h = float(height_m)
    flat = []

    def add_walls(pts_ccw, outward=True):
        pts = pts_ccw
        n = len(pts)
        for i in range(n):
            x0, y0 = pts[i]
            x1, y1 = pts[(i + 1) % n]
            if outward:
                flat.extend((
                    x0, y0, 0.0, x1, y1, 0.0, x1, y1, h,
                    x0, y0, 0.0, x1, y1, h, x0, y0, h,
                ))
            else:
                flat.extend((
                    x1, y1, 0.0, x0, y0, 0.0, x0, y0, h,
                    x1, y1, 0.0, x0, y0, h, x1, y1, h,
                ))

    outer_ccw = _orient(ring, ccw=True)
    if len(outer_ccw) < 3:
        return []
    add_walls(outer_ccw, outward=True)
    for hole in (holes or []):
        hole_ccw = _orient(hole, ccw=True)
        if len(hole_ccw) >= 3:
            add_walls(hole_ccw, outward=False)

    # roof (top face) with holes
    for a, b, c in _triangulate_with_holes(ring, holes or []):
        flat.extend((a[0], a[1], h, b[0], b[1], h, c[0], c[1], h))
    return flat


def triangulate_fan(ring, holes=None):
    """Triangulate a polygon (+ holes) -> flat (x,y) pairs for TRIANGLE_LIST."""
    flat = []
    for a, b, c in _triangulate_with_holes(ring, holes or []):
        flat.extend((a[0], a[1], b[0], b[1], c[0], c[1]))
    return flat


# ---------------------------------------------------------------------------
# Visualization helpers (ENU edges + occupancy raster) -- ROS-agnostic
# ---------------------------------------------------------------------------
def graph_edges_enu(g: "RouteGraph", enu: "ENU"):
    """Unique undirected edges as (e1, n1, e2, n2) in ENU meters."""
    seen = set()
    out = []
    for u, nbrs in g.adj.items():
        ue, un = enu.to_xy(*g.coords[u])
        for v, _w, _wid in nbrs:
            key = (u, v) if u < v else (v, u)
            if key in seen:
                continue
            seen.add(key)
            ve, vn = enu.to_xy(*g.coords[v])
            out.append((ue, un, ve, vn))
    return out


def rasterize_occupancy(
    edges, resolution=3.0, max_cells=25_000_000, pad=10.0, occ=100, bg=-1
):
    """Rasterize ENU edges into an OccupancyGrid-style int8 array.

    Returns (grid[H,W] int8, resolution, origin_e, origin_n, width, height).
    Road cells = `occ` (100, shown black), background = `bg` (-1, unknown).
    Resolution is auto-coarsened so the grid stays under `max_cells`.
    """
    arr = np.asarray(edges, dtype=np.float64)
    min_e = arr[:, [0, 2]].min() - pad
    min_n = arr[:, [1, 3]].min() - pad
    max_e = arr[:, [0, 2]].max() + pad
    max_n = arr[:, [1, 3]].max() + pad
    span_e, span_n = max_e - min_e, max_n - min_n

    res = float(resolution)
    while math.ceil(span_e / res) * math.ceil(span_n / res) > max_cells:
        res *= 1.5
    width = int(math.ceil(span_e / res))
    height = int(math.ceil(span_n / res))
    grid = np.full((height, width), bg, dtype=np.int8)

    for e1, n1, e2, n2 in edges:
        x0 = int((e1 - min_e) / res); y0 = int((n1 - min_n) / res)
        x1 = int((e2 - min_e) / res); y1 = int((n2 - min_n) / res)
        dx, dy = x1 - x0, y1 - y0
        steps = max(abs(dx), abs(dy), 1)
        for i in range(steps + 1):
            x = x0 + dx * i // steps
            y = y0 + dy * i // steps
            if 0 <= x < width and 0 <= y < height:
                grid[y, x] = occ
    return grid, res, min_e, min_n, width, height


def bbox(gpkg_path, table="route_edges"):
    """Quick data extent + suggested datum origin (center)."""
    g = build_graph(gpkg_path, table, mode="all")
    c = g._id_coords
    lon_min, lat_min = c.min(axis=0)
    lon_max, lat_max = c.max(axis=0)
    return {
        "lon_min": float(lon_min), "lon_max": float(lon_max),
        "lat_min": float(lat_min), "lat_max": float(lat_max),
        "center_lon": float((lon_min + lon_max) / 2),
        "center_lat": float((lat_min + lat_max) / 2),
    }
