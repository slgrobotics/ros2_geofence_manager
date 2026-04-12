#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import yaml


XY = Tuple[float, float]


@dataclass
class GeofenceDefinition:
    name: str
    frame_id: str
    points: List[XY]


def load_geofence_from_yaml(file_path: str) -> GeofenceDefinition:
    """
    Load a simple geofence polygon from YAML.

    Expected format:

    geofence:
      name: home_area
      frame_id: map
      points:
        - {x: 0.0, y: 0.0}
        - {x: 10.0, y: 0.0}
        - {x: 10.0, y: 10.0}
        - {x: 0.0, y: 10.0}
    """
    path = Path(file_path)
    if not path.is_file():
        raise FileNotFoundError(f"Geofence YAML file not found: '{file_path}'")

    with path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if not isinstance(data, dict):
        raise ValueError("Geofence YAML must contain a top-level mapping.")

    geofence = data.get("geofence")
    if not isinstance(geofence, dict):
        raise ValueError("Geofence YAML must contain a top-level 'geofence' mapping.")

    name = str(geofence.get("name", "geofence"))
    frame_id = str(geofence.get("frame_id", "map"))

    raw_points = geofence.get("points")
    if not isinstance(raw_points, list):
        raise ValueError("'geofence.points' must be a list.")

    points: List[XY] = []
    for i, item in enumerate(raw_points):
        if not isinstance(item, dict):
            raise ValueError(f"Point {i} must be a mapping like {{x: ..., y: ...}}.")

        if "x" not in item or "y" not in item:
            raise ValueError(f"Point {i} must contain both 'x' and 'y'.")

        try:
            x = float(item["x"])
            y = float(item["y"])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Point {i} has invalid numeric values: {item}") from exc

        points.append((x, y))

    if len(points) < 3:
        raise ValueError("Geofence polygon must contain at least 3 points.")

    return GeofenceDefinition(
        name=name,
        frame_id=frame_id,
        points=points,
    )
