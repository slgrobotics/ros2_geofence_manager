#!/usr/bin/env python3

from __future__ import annotations

from typing import List

import rclpy
from rclpy.node import Node
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Point

from geofence_manager.helpers.common_data import (
    GeofenceDefinition,
    LocalFrameDefinition,
    Point2D,
)
from geofence_manager.helpers.qgc_plan_loader import load_geofence_from_qgc_plan
from robot_localization.srv import FromLL, ToLL


def load_geofence_from_qgc_plan_ros(
    node: Node,
    file_path: str,
    from_service_name: str = "/fromLL",
    to_service_name: str = "/toLL",
    frame_id: str = "map",
    timeout_sec: float = 5.0,
) -> tuple[GeofenceDefinition, LocalFrameDefinition]:
    """
    Load a QGroundControl .plan geofence and convert it into ROS Cartesian coordinates
    using robot_localization/navsat_transform_node services.

    Returns:
        - GeofenceDefinition with points in ROS map/world Cartesian coordinates
        - LocalFrameDefinition describing the WGS84 location of (0, 0) in that frame
    """
    geofence_wgs84 = load_geofence_from_qgc_plan(file_path)

    if geofence_wgs84.reference_frame.lower() != "wgs84":
        raise ValueError(
            f"Expected WGS84 geofence from QGC loader, got '{geofence_wgs84.reference_frame}'"
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

    local_points: List[Point2D] = []

    for i, (lat, lon) in enumerate(geofence_wgs84.points):
        from_request = FromLL.Request()
        from_request.ll_point = GeoPoint()
        from_request.ll_point.latitude = float(lat)
        from_request.ll_point.longitude = float(lon)
        from_request.ll_point.altitude = 0.0

        future = from_client.call_async(from_request)
        node.get_logger().debug(
            f"Calling {from_service_name} for polygon vertex {i}: lat={lat}, lon={lon}"
        )
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)

        if not future.done() or future.result() is None:
            raise RuntimeError(
                f"fromLL call failed for vertex {i} ({lat}, {lon})"
            )

        response = future.result()
        local_points.append((float(response.map_point.x), float(response.map_point.y)))

    if len(local_points) < 3:
        raise ValueError("Converted geofence polygon must contain at least 3 points.")


    # As converted to local reference frame, we can update the geofence definition accordingly.:
    local_geofence = GeofenceDefinition(
        zone_name=geofence_wgs84.zone_name,
        reference_frame=geofence_wgs84.reference_frame.lower(),  # keep the original reference frame in the geofence definition for downstream use if needed
        points=local_points,
    )

    to_request = ToLL.Request()
    to_request.map_point = Point()
    to_request.map_point.x = 0.0
    to_request.map_point.y = 0.0
    to_request.map_point.z = 0.0

    to_future = to_client.call_async(to_request)
    node.get_logger().debug(f"Calling {to_service_name} for map origin")
    rclpy.spin_until_future_complete(node, to_future, timeout_sec=timeout_sec)

    if not to_future.done() or to_future.result() is None:
        raise RuntimeError("toLL call failed for map origin (0, 0)")

    to_response = to_future.result()
    lat0 = float(to_response.ll_point.latitude)
    lon0 = float(to_response.ll_point.longitude)

    # And also return the local ("conversion") reference frame definition for downstream use if needed.
    local_frame = LocalFrameDefinition(
        origin_lat_deg=lat0,
        origin_lon_deg=lon0,
        frame_id=frame_id,
    )

    return local_geofence, local_frame
