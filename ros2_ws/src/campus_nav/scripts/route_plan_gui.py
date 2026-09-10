#!/usr/bin/env python3
"""Small browser GUI for exercising campus_nav RouteGraph.plan_auto().

Serves one page (inline Leaflet map) plus two JSON endpoints from the stdlib
http.server: GET /api/data returns the road network + entry points, POST
/api/plan runs plan_auto() on a start/goal pair. It only plans -- no rclpy, no
Nav2 goal is ever sent, so it is safe to run next to a live stack.

Needs `campus_nav.route_graph` and `ament_index_python` importable. numpy,
PyYAML and pyosmium come from the system python, but ament_index_python ships
with ROS, so /opt/ros/jazzy/setup.bash must be sourced no matter which of the
two recipes below you use.

Run (LAN -- open it from any machine on the same network):

    cd ~/Navi/ros2_ws
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 src/campus_nav/scripts/route_plan_gui.py --host 0.0.0.0 --no-browser

    # then browse to http://<this-machine-ip>:8765/   e.g. http://192.168.1.100:8765/
    # `hostname -I` lists the addresses; on the robot eth1 is the wired LAN,
    # wlan0 the campus wifi, ap0 the robot's own hotspot.

Run (no colcon build required -- ROS core only, package root on PYTHONPATH):

    cd ~/Navi/ros2_ws/src/campus_nav
    source /opt/ros/jazzy/setup.bash
    PYTHONPATH=.:$PYTHONPATH /usr/bin/python3 scripts/route_plan_gui.py \
        --host 0.0.0.0 --no-browser

    # Keep the `:$PYTHONPATH` suffix. A bare `PYTHONPATH=.` wipes the entry that
    # sourcing ROS just added, and the script dies on ament_index_python.

Run (local only; binds 127.0.0.1 and opens a browser for you):

    python3 src/campus_nav/scripts/route_plan_gui.py

Smoke-test it without a browser:

    curl -s http://127.0.0.1:8765/api/data | head -c 200
    curl -s -X POST http://127.0.0.1:8765/api/plan -H 'Content-Type: application/json' \
      -d '{"start":{"lon":113.9350708,"lat":22.5743576},
           "goal":{"lon":113.9395,"lat":22.5770}}'

Useful flags:

    --host / --port      listen address; default 127.0.0.1:8765 (loopback only,
                         so LAN access needs --host 0.0.0.0 explicitly)
    --no-browser         do not try to open a browser (required on a headless robot)
    --verbose            log every HTTP request
    --config <yaml>      route_planner ROS params; defaults to the package's
                         config/route_planner.yaml and supplies every tuning
                         default below
    --pbf / --entries    road network (.osm.pbf) and entry-point layer (.gpkg)
    --entry-search-radius-m, --entry-offroad-weight, --direct-max-m,
    --always-direct-m, --detour-ratio-threshold, --same-entry-road-m,
    --off-network-m, --[no-]respect-oneway, --[no-]respect-turns,
    --[no-]reject-entry-network-crossing
                         override any route_planner.yaml value for a what-if
                         run without editing the YAML

Two gotchas:

* `ros2 run campus_nav route_plan_gui` does NOT work. setup.py declares the
  console_script as `campus_nav.route_plan_gui`, but this file lives under
  scripts/ and is never installed, so the wrapper dies with ModuleNotFoundError.
  Launch it by path, as above.
* The page pulls Leaflet and the OSM raster tiles from the public internet
  (unpkg.com, tile.openstreetmap.org). With no internet the basemap stays blank,
  while the campus network and the planned route still draw normally -- all of
  that data is served locally.
"""

from __future__ import annotations

import argparse
import json
import threading
import webbrowser
import yaml
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse


from ament_index_python.packages import get_package_share_directory

from campus_nav.route_graph import (
    build_graph_from_pbf,
    haversine,
    load_entry_points,
    load_stub_entry_points,
)


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _default_data_root() -> Path:
    """Prefer the source tree; fall back to the installed package share."""
    if (PACKAGE_ROOT / "config" / "route_planner.yaml").is_file():
        return PACKAGE_ROOT
    return Path(get_package_share_directory("campus_nav"))


CAMPUS_DATA = _default_data_root()


HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Campus Route Plan Tester</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <style>
    html, body, #map { height: 100%; margin: 0; }
    body { font: 14px/1.4 system-ui, sans-serif; }
    #panel { position:absolute; z-index:1000; left:12px; top:12px; width:330px;
      background:#fffffff2; padding:12px; border-radius:9px; box-shadow:0 2px 14px #0005; }
    #panel h3 { margin:0 0 8px; }
    button { margin:3px 4px 3px 0; padding:6px 10px; cursor:pointer; }
    button.active { color:white; background:#1769aa; }
    #status { margin-top:8px; white-space:pre-wrap; max-height:240px; overflow:auto; }
    .hint { color:#555; font-size:12px; }
  </style>
</head>
<body>
<div id="panel">
  <h3>Campus route plan 测试</h3>
  <button id="startBtn" onclick="setMode('start')">选择起点</button>
  <button id="goalBtn" onclick="setMode('goal')">选择终点</button>
  <button onclick="planRoute()">规划路径</button>
  <button onclick="resetAll()">清空</button>
  <div class="hint">先选择按钮，再点击地图；需要微调时重新选择起点或终点。GUI 只规划，不发送 Nav2。</div>
  <div id="status">正在加载路网……</div>
</div>
<div id="map"></div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const map = L.map('map');
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 21, attribution: '&copy; OpenStreetMap contributors'
}).addTo(map);
const networkLayer = L.geoJSON(null, {style:{color:'#2486d1',weight:2,opacity:.72}}).addTo(map);
const entryLayer = L.layerGroup().addTo(map);
const routeLayer = L.layerGroup().addTo(map);
L.control.layers(null, {'Campus network':networkLayer, 'Entries':entryLayer,
  'Planned route':routeLayer}).addTo(map);

let mode = 'start', startMarker = null, goalMarker = null;
const statusEl = document.getElementById('status');
function setMode(m) {
  mode=m;
  document.getElementById('startBtn').classList.toggle('active',m==='start');
  document.getElementById('goalBtn').classList.toggle('active',m==='goal');
}
function markerAt(kind, ll) {
  const old = kind==='start' ? startMarker : goalMarker;
  if (old) map.removeLayer(old);
  const color = kind==='start' ? '#17a65b' : '#e43d30';
  const marker = L.circleMarker(ll,{radius:8,color:'#fff',weight:2,fillColor:color,
    fillOpacity:1, draggable:true}).addTo(map).bindTooltip(kind==='start'?'起点':'终点');
  if (kind==='start') startMarker=marker; else goalMarker=marker;
}
map.on('click', e => { markerAt(mode,e.latlng); setMode(mode==='start'?'goal':'start'); });

async function loadData() {
  try {
    const data=await (await fetch('/api/data')).json();
    networkLayer.addData(data.network);
    for (const e of data.entries) {
      L.circleMarker([e.lat,e.lon],{radius:6,color:'#073',weight:2,
        fillColor:'#22d6c5',fillOpacity:.95}).bindPopup(
        `<b>${e.name}</b><br>radius: ${e.radius_m ?? data.config.entry_search_radius_m} m<br>${e.source}`
      ).addTo(entryLayer);
    }
    const bounds=networkLayer.getBounds();
    if (bounds.isValid()) map.fitBounds(bounds.pad(.03));
    else map.setView([data.config.datum_lat,data.config.datum_lon],18);
    statusEl.textContent=`路网 ${data.stats.nodes} nodes / ${data.network.features.length} segments\n`+
      `入口 ${data.entries.length} 个\nentry radius: ${data.config.entry_search_radius_m} m\n`+
      `entry weight: ${data.config.entry_offroad_weight}`;
    setMode('start');
  } catch(e) { statusEl.textContent='加载失败：'+e; }
}
async function planRoute() {
  if (!startMarker || !goalMarker) { statusEl.textContent='请先选择起点和终点。'; return; }
  statusEl.textContent='规划中……'; routeLayer.clearLayers();
  const s=startMarker.getLatLng(), g=goalMarker.getLatLng();
  try {
    const res=await fetch('/api/plan',{method:'POST',headers:{'Content-Type':'application/json'},
      body:JSON.stringify({start:{lon:s.lng,lat:s.lat},goal:{lon:g.lng,lat:g.lat}})});
    const data=await res.json();
    if (!res.ok) throw new Error(data.error || res.statusText);
    const route=L.polyline(data.path.map(p=>[p[1],p[0]]),
      {color:data.mode==='road'?'#ff5b22':'#a224d1',weight:6,opacity:.9}).addTo(routeLayer);
    map.fitBounds(route.getBounds().pad(.25));
    statusEl.textContent=
      `mode: ${data.mode}\nreason: ${data.decision_reason}\n`+
      `entry: ${data.start_entry_name||'-'} → ${data.entry_name||'-'}\n`+
      `straight: ${data.straight_m.toFixed(1)} m\n`+
      `selected path: ${data.length_m.toFixed(1)} m\n`+
      `road candidate: ${data.road_total_m==null?'unreachable':data.road_total_m.toFixed(1)+' m'}\n`+
      `detour ratio: ${data.detour_ratio==null?'n/a':data.detour_ratio.toFixed(3)}`;
  } catch(e) { statusEl.textContent='规划失败：'+e.message; }
}
function resetAll() {
  if(startMarker) map.removeLayer(startMarker); if(goalMarker) map.removeLayer(goalMarker);
  startMarker=goalMarker=null; routeLayer.clearLayers(); setMode('start'); statusEl.textContent='已清空。';
}
loadData();
</script>
</body></html>"""


class App:
    def __init__(self, args):
        self.args = args
        print(f"Loading campus graph: {args.pbf}", flush=True)
        self.graph = build_graph_from_pbf(
            str(args.pbf), mode="walk", respect_oneway=args.respect_oneway,
            allow_private=True,
        )
        self.entries = load_entry_points(str(args.entries), table="entries")
        self.entries += load_stub_entry_points(str(args.pbf), self.graph)
        self.lock = threading.Lock()  # plan_points temporarily edits the graph.
        features = []
        for u, v in zip(self.graph._edge_u, self.graph._edge_v):
            a, b = self.graph.coords[int(u)], self.graph.coords[int(v)]
            features.append({
                "type": "Feature",
                "properties": {},
                "geometry": {"type": "LineString", "coordinates": [a, b]},
            })
        self.data = {
            "network": {"type": "FeatureCollection", "features": features},
            "entries": self.entries,
            "stats": self.graph.stats,
            "config": {
                "datum_lat": args.datum_lat, "datum_lon": args.datum_lon,
                "entry_search_radius_m": args.entry_search_radius_m,
                "entry_offroad_weight": args.entry_offroad_weight,
                "direct_max_m": args.direct_max_m,
            },
        }
        print(f"Ready: {self.graph.stats}; {len(self.entries)} entries", flush=True)

    def plan(self, request):
        start, goal = request["start"], request["goal"]
        with self.lock:
            plan = self.graph.plan_auto(
                float(start["lon"]), float(start["lat"]),
                float(goal["lon"]), float(goal["lat"]),
                entries=self.entries,
                off_network_m=self.args.off_network_m,
                search_radius_m=self.args.entry_search_radius_m,
                offroad_weight=self.args.entry_offroad_weight,
                reject_entry_network_crossing=self.args.reject_entry_network_crossing,
                respect_turns=self.args.respect_turns,
                direct_max_m=self.args.direct_max_m,
                always_direct_m=self.args.always_direct_m,
                detour_ratio_threshold=self.args.detour_ratio_threshold,
                same_entry_road_m=self.args.same_entry_road_m,
            )
        if plan is None:
            raise ValueError("道路不可达，且直线距离超过 direct_max_m")
        keys = (
            "path", "mode", "decision_reason", "length_m", "road_m",
            "straight_m", "road_total_m", "detour_ratio", "entry_name",
            "start_entry_name", "entry_missing", "start_entry_missing",
        )
        return {key: plan.get(key) for key in keys}


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        def send_json(self, obj, status=200):
            raw = json.dumps(obj, ensure_ascii=False, separators=(",", ":")).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/":
                raw = HTML.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(raw)))
                self.end_headers()
                self.wfile.write(raw)
            elif path == "/api/data":
                self.send_json(app.data)
            else:
                self.send_error(404)

        def do_POST(self):
            if urlparse(self.path).path != "/api/plan":
                self.send_error(404)
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                request = json.loads(self.rfile.read(size))
                self.send_json(app.plan(request))
            except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                self.send_json({"error": str(exc)}, 400)
            except Exception as exc:  # Keep the GUI alive and expose the failure.
                self.send_json({"error": f"{type(exc).__name__}: {exc}"}, 500)

        def log_message(self, fmt, *args):
            if app.args.verbose:
                super().log_message(fmt, *args)

    return Handler


def _route_planner_params(config_path):
    """Read the same ROS parameter YAML used by campus_route_planner."""
    with Path(config_path).open(encoding="utf-8") as stream:
        doc = yaml.safe_load(stream) or {}
    return doc.get("/**", {}).get("ros__parameters", {})


def parse_args():
    default_config = CAMPUS_DATA / "config" / "route_planner.yaml"
    pre = argparse.ArgumentParser(add_help=False)
    pre.add_argument("--config", type=Path, default=default_config)
    known, _ = pre.parse_known_args()
    params = _route_planner_params(known.config)

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--config", type=Path, default=known.config,
                        help="route_planner ROS parameter YAML")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--pbf", type=Path, default=CAMPUS_DATA / "xili_5km_finetuned.osm.pbf")
    parser.add_argument("--entries", type=Path, default=CAMPUS_DATA / "campus_entries.gpkg")
    parser.add_argument("--datum-lat", type=float, default=params.get("datum_lat", 22.574357630323675))
    parser.add_argument("--datum-lon", type=float, default=params.get("datum_lon", 113.93507078035577))
    parser.add_argument("--off-network-m", type=float, default=params.get("off_network_m", 5.0))
    parser.add_argument("--entry-search-radius-m", type=float, default=params.get("entry_search_radius_m", 99.5))
    parser.add_argument("--entry-offroad-weight", type=float, default=params.get("entry_offroad_weight", 1.0))
    parser.add_argument("--reject-entry-network-crossing",
                        action=argparse.BooleanOptionalAction,
                        default=bool(params.get("reject_entry_network_crossing", True)))
    parser.add_argument("--direct-max-m", type=float, default=params.get("direct_max_m", 99.5))
    parser.add_argument("--always-direct-m", type=float, default=params.get("always_direct_m", 8.0))
    parser.add_argument("--detour-ratio-threshold", type=float, default=params.get("detour_ratio_threshold", 1.6))
    parser.add_argument("--same-entry-road-m", type=float, default=params.get("same_entry_road_m", 2.0))
    parser.add_argument("--respect-oneway", action=argparse.BooleanOptionalAction,
                        default=bool(params.get("respect_oneway", False)))
    parser.add_argument("--respect-turns", action=argparse.BooleanOptionalAction,
                        default=bool(params.get("respect_turns", False)))
    return parser.parse_args()


def main():
    args = parse_args()
    app = App(args)
    server = ThreadingHTTPServer((args.host, args.port), make_handler(app))
    url = f"http://{args.host}:{args.port}/"
    print(f"Route plan GUI: {url}  (Ctrl-C to stop)", flush=True)
    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
