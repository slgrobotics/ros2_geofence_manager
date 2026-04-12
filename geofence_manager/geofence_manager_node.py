#!/usr/bin/env python3

from __future__ import annotations

from typing import List, Optional, Sequence, Tuple

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Point, PolygonStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from geofence_manager_interfaces.msg import GeofenceStatus
from geofence_manager_interfaces.srv import IsPoseAllowed

from geofence_manager.geometry_utils import (
    closest_point_on_polygon,
    distance_to_polygon_edges,
    point_in_polygon,
)
from geofence_manager.polygon_loader import load_geofence_from_yaml


XY = Tuple[float, float]


class GeofenceManagerNode(Node):
    """
    Publish geofence status for the current robot pose and answer pose legality checks.

    Version 1 assumptions:
    - Geofence polygon is already defined in the map/world frame.
    - Pose source is either geometry_msgs/PoseStamped or nav_msgs/Odometry.
    - No TF transforms are performed here.
    """

    def __init__(self) -> None:
        super().__init__("geofence_manager")

        self.declare_parameter("geofence_file", "")
        self.declare_parameter("pose_topic", "/robot_pose")
        self.declare_parameter("pose_source_type", "pose_stamped")  # pose_stamped | odometry
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("near_boundary_threshold_m", 2.0)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("publish_polygon", True)
        self.declare_parameter("status_publish_rate_hz", 5.0)
        self.declare_parameter("localization_timeout_sec", 2.0)

        self._geofence_file = str(self.get_parameter("geofence_file").value)
        self._pose_topic = str(self.get_parameter("pose_topic").value)
        self._pose_source_type = str(self.get_parameter("pose_source_type").value).strip().lower()
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._near_boundary_threshold_m = float(self.get_parameter("near_boundary_threshold_m").value)
        self._publish_markers = bool(self.get_parameter("publish_markers").value)
        self._publish_polygon = bool(self.get_parameter("publish_polygon").value)
        self._status_publish_rate_hz = float(self.get_parameter("status_publish_rate_hz").value)
        self._localization_timeout_sec = float(self.get_parameter("localization_timeout_sec").value)

        self._zone_name: str = ""
        self._polygon_frame_id: str = self._world_frame
        self._polygon_xy: List[XY] = []

        self._last_pose_x: Optional[float] = None
        self._last_pose_y: Optional[float] = None
        self._last_pose_stamp = None
        self._last_pose_frame_id: str = ""

        self._load_polygon_or_fail()

        self._status_pub = self.create_publisher(GeofenceStatus, "/geofence/status", 10)
        self._inside_pub = self.create_publisher(Bool, "/geofence/is_inside", 10)

        polygon_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._polygon_pub = self.create_publisher(PolygonStamped, "/geofence/polygon", polygon_qos)
        self._marker_pub = self.create_publisher(MarkerArray, "/geofence/markers", polygon_qos)

        self._is_pose_allowed_srv = self.create_service(
            IsPoseAllowed,
            "/geofence/is_pose_allowed",
            self._handle_is_pose_allowed,
        )

        if self._pose_source_type == "pose_stamped":
            self._pose_sub = self.create_subscription(
                PoseStamped,
                self._pose_topic,
                self._pose_stamped_callback,
                10,
            )
        elif self._pose_source_type == "odometry":
            self._pose_sub = self.create_subscription(
                Odometry,
                self._pose_topic,
                self._odometry_callback,
                10,
            )
        else:
            raise ValueError(
                f"Unsupported pose_source_type '{self._pose_source_type}'. "
                f"Use 'pose_stamped' or 'odometry'."
            )

        timer_period = 1.0 / self._status_publish_rate_hz if self._status_publish_rate_hz > 0.0 else 0.2
        self._status_timer = self.create_timer(timer_period, self._publish_status_timer_callback)

        if self._publish_polygon:
            self._publish_polygon_msg()

        if self._publish_markers:
            self._publish_marker_msg()

        self.get_logger().info(
            f"geofence_manager started | file='{self._geofence_file}' | "
            f"zone='{self._zone_name}' | frame='{self._polygon_frame_id}' | "
            f"points={len(self._polygon_xy)} | pose_topic='{self._pose_topic}' | "
            f"pose_source_type='{self._pose_source_type}'"
        )

    def _load_polygon_or_fail(self) -> None:
        if not self._geofence_file:
            raise ValueError("Parameter 'geofence_file' must not be empty.")

        geofence = load_geofence_from_yaml(self._geofence_file)

        self._zone_name = geofence.name
        self._polygon_frame_id = geofence.frame_id or self._world_frame
        self._polygon_xy = list(geofence.points)

        if len(self._polygon_xy) < 3:
            raise ValueError("Geofence polygon must contain at least 3 points.")

        if self._polygon_frame_id != self._world_frame:
            self.get_logger().warn(
                f"Geofence frame '{self._polygon_frame_id}' differs from world_frame "
                f"'{self._world_frame}'. This node does not transform frames in v1."
            )

    def _pose_stamped_callback(self, msg: PoseStamped) -> None:
        self._store_pose(
            x=msg.pose.position.x,
            y=msg.pose.position.y,
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id,
        )

    def _odometry_callback(self, msg: Odometry) -> None:
        self._store_pose(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id,
        )

    def _store_pose(self, x: float, y: float, stamp, frame_id: str) -> None:
        self._last_pose_x = float(x)
        self._last_pose_y = float(y)
        self._last_pose_stamp = stamp
        self._last_pose_frame_id = frame_id.strip()

    def _publish_status_timer_callback(self) -> None:
        status = self._build_status_from_latest_pose()
        if status is None:
            return

        self._status_pub.publish(status)

        inside_msg = Bool()
        inside_msg.data = status.is_inside
        self._inside_pub.publish(inside_msg)

    def _build_status_from_latest_pose(self) -> Optional[GeofenceStatus]:
        msg = GeofenceStatus()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = self._polygon_frame_id
        msg.zone_name = self._zone_name

        if self._last_pose_x is None or self._last_pose_y is None or self._last_pose_stamp is None:
            msg.is_inside = False
            msg.is_near_boundary = False
            msg.localization_valid = False
            msg.distance_to_boundary_m = math.inf
            msg.closest_boundary_point = Point()
            msg.state = "UNKNOWN"
            return msg

        if self._last_pose_frame_id and self._last_pose_frame_id != self._polygon_frame_id:
            self.get_logger().warn(
                f"Pose frame '{self._last_pose_frame_id}' does not match geofence frame "
                f"'{self._polygon_frame_id}'. Publishing UNKNOWN state."
            )
            msg.is_inside = False
            msg.is_near_boundary = False
            msg.localization_valid = False
            msg.distance_to_boundary_m = math.inf
            msg.closest_boundary_point = Point()
            msg.state = "UNKNOWN"
            return msg

        pose_age_sec = self._seconds_since_stamp(self._last_pose_stamp)
        localization_valid = pose_age_sec <= self._localization_timeout_sec

        if not localization_valid:
            msg.is_inside = False
            msg.is_near_boundary = False
            msg.localization_valid = False
            msg.distance_to_boundary_m = math.inf
            msg.closest_boundary_point = Point()
            msg.state = "UNKNOWN"
            return msg

        x = self._last_pose_x
        y = self._last_pose_y

        inside = point_in_polygon(x, y, self._polygon_xy)
        distance_m = distance_to_polygon_edges(x, y, self._polygon_xy)
        closest_x, closest_y = closest_point_on_polygon(x, y, self._polygon_xy)

        msg.is_inside = inside
        msg.is_near_boundary = inside and distance_m <= self._near_boundary_threshold_m
        msg.localization_valid = True
        msg.distance_to_boundary_m = float(distance_m)

        closest_point = Point()
        closest_point.x = float(closest_x)
        closest_point.y = float(closest_y)
        closest_point.z = 0.0
        msg.closest_boundary_point = closest_point

        if inside:
            msg.state = "NEAR_BOUNDARY" if msg.is_near_boundary else "INSIDE"
        else:
            msg.state = "OUTSIDE"

        return msg

    def _handle_is_pose_allowed(self, request: IsPoseAllowed.Request, response: IsPoseAllowed.Response):
        pose = request.pose

        request_frame = pose.header.frame_id.strip()
        if request_frame and request_frame != self._polygon_frame_id:
            response.allowed = False
            response.distance_to_boundary_m = -1.0
            response.reason = (
                f"pose frame '{request_frame}' does not match geofence frame "
                f"'{self._polygon_frame_id}'"
            )
            return response

        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)

        inside = point_in_polygon(x, y, self._polygon_xy)
        distance_m = distance_to_polygon_edges(x, y, self._polygon_xy)

        response.allowed = inside
        response.distance_to_boundary_m = float(distance_m)
        response.reason = "inside geofence" if inside else "outside geofence"
        return response

    def _publish_polygon_msg(self) -> None:
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._polygon_frame_id

        for x, y in self._closed_polygon_xy():
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            msg.polygon.points.append(pt)

        self._polygon_pub.publish(msg)

    def _publish_marker_msg(self) -> None:
        marker_array = MarkerArray()

        line_marker = Marker()
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.header.frame_id = self._polygon_frame_id
        line_marker.ns = "geofence_boundary"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.pose.orientation.w = 1.0
        line_marker.scale.x = 0.15
        line_marker.color.a = 1.0
        line_marker.color.r = 1.0
        line_marker.color.g = 0.2
        line_marker.color.b = 0.2

        for x, y in self._closed_polygon_xy():
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.05
            line_marker.points.append(pt)

        marker_array.markers.append(line_marker)

        vertex_marker = Marker()
        vertex_marker.header.stamp = line_marker.header.stamp
        vertex_marker.header.frame_id = self._polygon_frame_id
        vertex_marker.ns = "geofence_vertices"
        vertex_marker.id = 1
        vertex_marker.type = Marker.SPHERE_LIST
        vertex_marker.action = Marker.ADD
        vertex_marker.pose.orientation.w = 1.0
        vertex_marker.scale.x = 0.25
        vertex_marker.scale.y = 0.25
        vertex_marker.scale.z = 0.25
        vertex_marker.color.a = 1.0
        vertex_marker.color.r = 1.0
        vertex_marker.color.g = 1.0
        vertex_marker.color.b = 0.2

        for x, y in self._polygon_xy:
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.05
            vertex_marker.points.append(pt)

        marker_array.markers.append(vertex_marker)

        self._marker_pub.publish(marker_array)

    def _closed_polygon_xy(self) -> Sequence[XY]:
        if not self._polygon_xy:
            return []

        if self._polygon_xy[0] == self._polygon_xy[-1]:
            return self._polygon_xy

        return [*self._polygon_xy, self._polygon_xy[0]]

    def _seconds_since_stamp(self, stamp) -> float:
        now = self.get_clock().now()
        then = rclpy.time.Time.from_msg(stamp)
        return max(0.0, (now - then).nanoseconds / 1e9)


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node = GeofenceManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
