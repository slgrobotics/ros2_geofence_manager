#!/usr/bin/env python3

from __future__ import annotations

import json
from pathlib import Path
from typing import List, Tuple

from geofence_manager.common_data import GeofenceDefinition, Point2D


def load_geofence_from_qgc_plan(file_path: str) -> GeofenceDefinition:

    # ----------------------------------------------------------
    # Load a geofence polygon from a QGroundControl .plan file.

    # Expected QGC structure:

    # {
    #   "fileType": "Plan",
    #   "geoFence": {
    #     "polygons": [
    #       {
    #         "inclusion": true,
    #         "polygon": [
    #           [lat, lon],
    #           [lat, lon],
    #           ...
    #         ]
    #       }
    #     ]
    #   }
    # }
    #
    # Notes:
    # - Returns the first inclusion polygon found.
    # - Points are returned as (lat, lon).
    # - frame_id is set to "wgs84".
    # - name defaults to the file stem.
    # ----------------------------------------------------------

    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f"QGC plan file not found: '{file_path}'")

    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)

    if not isinstance(data, dict):
        raise ValueError("QGC plan file must contain a top-level JSON object.")

    file_type = data.get("fileType")
    if file_type != "Plan":
        raise ValueError(f"Unsupported QGC fileType: {file_type!r}; expected 'Plan'.")

    geo_fence = data.get("geoFence")
    if not isinstance(geo_fence, dict):
        raise ValueError("QGC plan file does not contain a valid 'geoFence' object.")

    polygons = geo_fence.get("polygons")
    if not isinstance(polygons, list):
        raise ValueError("QGC plan file does not contain a valid 'geoFence.polygons' list.")

    selected_polygon = None
    for i, poly in enumerate(polygons):
        if not isinstance(poly, dict):
            continue

        inclusion = poly.get("inclusion", False)
        raw_polygon = poly.get("polygon")

        if inclusion and isinstance(raw_polygon, list):
            selected_polygon = raw_polygon
            break

    if selected_polygon is None:
        raise ValueError("No inclusion geofence polygon found in QGC plan file.")

    points: List[Point2D] = []
    for i, item in enumerate(selected_polygon):
        if not isinstance(item, list) or len(item) < 2:
            raise ValueError(
                f"QGC polygon point {i} must be a list like [lat, lon], got: {item!r}"
            )

        try:
            lat = float(item[0])
            lon = float(item[1])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"QGC polygon point {i} has invalid numeric values: {item!r}") from exc

        points.append((lat, lon))

    if len(points) < 3:
        raise ValueError("QGC geofence polygon must contain at least 3 points.")

    return GeofenceDefinition(
        name=path.stem,
        frame_id="wgs84",
        points=points,
    )
