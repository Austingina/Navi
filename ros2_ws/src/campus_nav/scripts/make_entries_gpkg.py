#!/usr/bin/env python3
"""Create an empty entry-point GeoPackage for QGIS authoring.

The route planner reads this layer to decide where the robot may leave the road
network for a goal that sits off it (a plaza or courtyard behind a fence). One
point per opening; the planner picks the opening with the cheapest
road-distance + off-road-distance total.

Written with plain sqlite3 so it works without GDAL/ogr installed.

    /usr/bin/python3 scripts/make_entries_gpkg.py
    /usr/bin/python3 scripts/make_entries_gpkg.py --output /tmp/x.gpkg --table entries

Then in QGIS: Layer - Add Layer - Add Vector Layer, pick the .gpkg, toggle
editing and click a point at every opening. Optional columns: `name` shows up in
the planner log and in RViz, `radius_m` overrides how far from a goal this
opening is still considered.
"""

from __future__ import annotations

import argparse
import os
import sqlite3

GPKG_APPLICATION_ID = 1196444487  # 'GPKG'
GPKG_USER_VERSION = 10300  # 1.3

_SRS_ROWS = [
    ("Undefined cartesian SRS", -1, "NONE", -1, "undefined", None),
    ("Undefined geographic SRS", 0, "NONE", 0, "undefined", None),
    (
        "WGS 84 geodetic", 4326, "EPSG", 4326,
        'GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,'
        '298.257223563]],PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433]]',
        "longitude/latitude coordinates in decimal degrees",
    ),
]


def create_entries_gpkg(path: str, table: str = "entries") -> None:
    con = sqlite3.connect(path)
    try:
        con.execute(f"PRAGMA application_id = {GPKG_APPLICATION_ID}")
        con.execute(f"PRAGMA user_version = {GPKG_USER_VERSION}")
        con.executescript(
            """
            CREATE TABLE IF NOT EXISTS gpkg_spatial_ref_sys (
                srs_name TEXT NOT NULL,
                srs_id INTEGER NOT NULL PRIMARY KEY,
                organization TEXT NOT NULL,
                organization_coordsys_id INTEGER NOT NULL,
                definition TEXT NOT NULL,
                description TEXT
            );
            CREATE TABLE IF NOT EXISTS gpkg_contents (
                table_name TEXT NOT NULL PRIMARY KEY,
                data_type TEXT NOT NULL,
                identifier TEXT UNIQUE,
                description TEXT DEFAULT '',
                last_change DATETIME NOT NULL DEFAULT
                    (strftime('%Y-%m-%dT%H:%M:%fZ','now')),
                min_x DOUBLE, min_y DOUBLE, max_x DOUBLE, max_y DOUBLE,
                srs_id INTEGER,
                CONSTRAINT fk_gc_r_srs_id FOREIGN KEY (srs_id)
                    REFERENCES gpkg_spatial_ref_sys(srs_id)
            );
            CREATE TABLE IF NOT EXISTS gpkg_geometry_columns (
                table_name TEXT NOT NULL,
                column_name TEXT NOT NULL,
                geometry_type_name TEXT NOT NULL,
                srs_id INTEGER NOT NULL,
                z TINYINT NOT NULL,
                m TINYINT NOT NULL,
                CONSTRAINT pk_geom_cols PRIMARY KEY (table_name, column_name),
                CONSTRAINT fk_gc_tn FOREIGN KEY (table_name)
                    REFERENCES gpkg_contents(table_name)
            );
            """
        )
        con.executemany(
            "INSERT OR IGNORE INTO gpkg_spatial_ref_sys VALUES (?,?,?,?,?,?)",
            _SRS_ROWS,
        )
        con.execute(
            f'CREATE TABLE IF NOT EXISTS "{table}" ('
            "fid INTEGER PRIMARY KEY AUTOINCREMENT, "
            "geom POINT, "
            "name TEXT, "
            "radius_m REAL)"
        )
        con.execute(
            "INSERT OR REPLACE INTO gpkg_contents "
            "(table_name, data_type, identifier, description, srs_id) "
            "VALUES (?, 'features', ?, ?, 4326)",
            (table, table, "Off-network entry points for campus_nav"),
        )
        con.execute(
            "INSERT OR REPLACE INTO gpkg_geometry_columns VALUES (?, 'geom', "
            "'POINT', 4326, 0, 0)",
            (table,),
        )
        con.commit()
    finally:
        con.close()


def main() -> None:
    pkg_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        "--output", default=os.path.join(pkg_root, "campus_entries.gpkg")
    )
    ap.add_argument("--table", default="entries")
    args = ap.parse_args()
    if os.path.exists(args.output):
        raise SystemExit(f"refusing to overwrite existing {args.output}")
    create_entries_gpkg(args.output, args.table)
    print(f"created {args.output} (layer '{args.table}', POINT, EPSG:4326)")


if __name__ == "__main__":
    main()
