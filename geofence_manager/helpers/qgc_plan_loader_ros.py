#!/usr/bin/env python3

from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point

from geofence_manager.helpers.common_data import (
    BreachReturnPoint,
    GeofenceCollection,
    GeofenceZoneCircle,
    GeofenceZonePolygon,
    LocalFrameDefinition,
    Point2D,
)
from geofence_manager.helpers.qgc_plan_loader import load_geofence_collection_from_qgc_plan
from robot_localization.srv import FromLL, ToLL


def _from_ll(
    node: Node,
    client,
    lat: float,
    lon: float,
    timeout_sec: float,
) -> Point2D:
    request = FromLL.Request()
    request.ll_point = GeoPoint()
    request.ll_point.latitude = float(lat)
    request.ll_point.longitude = float(lon)
    request.ll_point.altitude = 0.0

    future = client.call_async(request)
    node.get_logger().debug(f"Calling fromLL for lat={lat}, lon={lon}")
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if not future.done() or future.result() is None:
        raise RuntimeError(f"fromLL call failed for ({lat}, {lon})")

    response = future.result()
    return (float(response.map_point.x), float(response.map_point.y))


def _to_ll_origin(
    node: Node,
    client,
    timeout_sec: float,
) -> tuple[float, float]:
    request = ToLL.Request()
    request.map_point = Point()
    request.map_point.x = 0.0
    request.map_point.y = 0.0
    request.map_point.z = 0.0

    future = client.call_async(request)
    node.get_logger().debug("Calling toLL for map origin")
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

    if not future.done() or future.result() is None:
        raise RuntimeError("toLL call failed for map origin (0, 0, 0)")

    response = future.result()
    return (
        float(response.ll_point.latitude),
        float(response.ll_point.longitude),
    )


def load_geofence_from_qgc_plan_ros(
    node: Node,
    file_path: str,
    from_service_name: str = "/fromLL",
    to_service_name: str = "/toLL",
    frame_id: str = "map",
    timeout_sec: float = 5.0,
) -> tuple[GeofenceCollection, LocalFrameDefinition]:
    """
    Load a QGroundControl .plan geofence collection and convert it into ROS Cartesian coordinates
    using robot_localization/navsat_transform_node services.

    Returns:
        - GeofenceCollection with geometry in ROS Cartesian coordinates
        - LocalFrameDefinition describing the WGS84 location of (0, 0) in that frame
    """
    geofence_wgs84 = load_geofence_collection_from_qgc_plan(file_path)

    wgs84_reference_frame = "wgs84"  # We expect the QGC geofence to be defined in WGS84 lat/lon coordinates

    if geofence_wgs84.reference_frame.lower() != wgs84_reference_frame:
        raise ValueError(
            f"Expected WGS84 geofence from QGC loader, got '{geofence_wgs84.reference_frame}' instead of '{wgs84_reference_frame}'."
        )

    from_client = node.create_client(FromLL, from_service_name)
    if not from_client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(
            f"Service '{from_service_name}' not available after {timeout_sec:.1f} s"
        )

    to_client = node.create_client(ToLL, to_service_name)
    if not to_client.wait_for_service(timeout_sec=timeout_sec):
        raise RuntimeError(
            f"Service '{to_service_name}' not available after {timeout_sec:.1f} s"
        )

    local_polygons: List[GeofenceZonePolygon] = []
    for poly in geofence_wgs84.polygons:
        local_points = [
            _from_ll(node, from_client, lat, lon, timeout_sec)
            for lat, lon in poly.points
        ]

        local_polygons.append(
            GeofenceZonePolygon(
                zone_name=poly.zone_name,
                points=local_points,
                inclusion=poly.inclusion,
                reference_frame=wgs84_reference_frame,
            )
        )

    local_circles: List[GeofenceZoneCircle] = []
    for circle in geofence_wgs84.circles:
        center_xy = _from_ll(
            node,
            from_client,
            circle.center[0],
            circle.center[1],
            timeout_sec,
        )

        local_circles.append(
            GeofenceZoneCircle(
                zone_name=circle.zone_name,
                center=center_xy,
                radius_m=circle.radius_m,
                inclusion=circle.inclusion,
                reference_frame=wgs84_reference_frame,
            )
        )

    local_breach_return = None
    if geofence_wgs84.breach_return is not None:
        breach_xy = _from_ll(
            node,
            from_client,
            geofence_wgs84.breach_return.point[0],
            geofence_wgs84.breach_return.point[1],
            timeout_sec,
        )

        local_breach_return = BreachReturnPoint(
            point=breach_xy,
            altitude_m=geofence_wgs84.breach_return.altitude_m,
            reference_frame=wgs84_reference_frame,
        )

    lat0, lon0 = _to_ll_origin(node, to_client, timeout_sec)

    local_geofence = GeofenceCollection(
        source_name=geofence_wgs84.source_name,
        reference_frame=wgs84_reference_frame,
        polygons=local_polygons,
        circles=local_circles,
        breach_return=local_breach_return,
    )

    local_frame = LocalFrameDefinition(
        origin_lat_deg=lat0,
        origin_lon_deg=lon0,
        frame_id=frame_id,
    )

    return local_geofence, local_frame
