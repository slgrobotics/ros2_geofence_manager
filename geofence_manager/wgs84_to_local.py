#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import List

from geofence_manager.common_data import Point2D, GeofenceDefinition, LocalFrameDefinition, EARTH_RADIUS_M


def latlon_to_local_xy(
    lat_deg: float,
    lon_deg: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
) -> Point2D:
    """
    Convert WGS84 latitude/longitude to local Cartesian x/y in meters.

    Uses an equirectangular approximation:
      x = R * dlon * cos(lat0)
      y = R * dlat

    where:
      x points east
      y points north

    Good enough for small geofence areas.
    """
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    lat0 = math.radians(origin_lat_deg)
    lon0 = math.radians(origin_lon_deg)

    dlat = lat - lat0
    dlon = lon - lon0

    x_m = EARTH_RADIUS_M * dlon * math.cos(lat0)
    y_m = EARTH_RADIUS_M * dlat
    return (x_m, y_m)


def convert_wgs84_polygon_to_local(
    geofence_wgs84: GeofenceDefinition,
    origin_lat_deg: float | None = None,
    origin_lon_deg: float | None = None,
    reference_frame: str = "local_cartesian",
) -> tuple[GeofenceDefinition, LocalFrameDefinition]:
    """
    Convert a WGS84 geofence polygon into a local Cartesian frame.

    Assumes geofence_wgs84.points are stored as:
      (lat, lon)

    Returns:
      - converted GeofenceDefinition with points in meters
      - LocalFrameDefinition describing the chosen origin
    """
    if geofence_wgs84.reference_frame.lower() != "wgs84":
        raise ValueError(
            f"Expected WGS84 geofence, got reference_frame='{geofence_wgs84.reference_frame}' instead."
        )

    if len(geofence_wgs84.points) < 3:
        raise ValueError("Geofence polygon must contain at least 3 points.")

    if origin_lat_deg is None or origin_lon_deg is None:
        # Use polygon centroid in geographic coordinates as a stable local origin.
        lat_vals = [p[0] for p in geofence_wgs84.points]
        lon_vals = [p[1] for p in geofence_wgs84.points]
        origin_lat_deg = sum(lat_vals) / len(lat_vals)
        origin_lon_deg = sum(lon_vals) / len(lon_vals)

    local_points: List[Point2D] = []
    for lat_deg, lon_deg in geofence_wgs84.points:
        local_points.append(
            latlon_to_local_xy(
                lat_deg=lat_deg,
                lon_deg=lon_deg,
                origin_lat_deg=origin_lat_deg,
                origin_lon_deg=origin_lon_deg,
            )
        )

    local_geofence = GeofenceDefinition(
        zone_name=geofence_wgs84.zone_name,
        reference_frame=reference_frame,
        points=local_points,
    )

    local_frame = LocalFrameDefinition(
        origin_lat_deg=origin_lat_deg,
        origin_lon_deg=origin_lon_deg,
        reference_frame=reference_frame,
    )

    return local_geofence, local_frame
