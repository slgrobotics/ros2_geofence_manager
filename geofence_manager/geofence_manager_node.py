#!/usr/bin/env python3

# ----------------------------------------------------------------------------
#  Geofence manager node for outdoor robots using polygon-based boundaries.
# 
#  This node monitors the robot pose relative to a configured geofence polygon
#  and provides real-time status, geometry insights, and helper services for
#  higher-level behaviors.
# 
#  Core responsibilities:
#  - Classify robot state: INSIDE, NEAR_BOUNDARY, OUTSIDE, or UNKNOWN
#  - Compute nearest boundary point and distance to the geofence
#  - Publish geofence polygon and visualization markers
#  - Provide geometry-based services (e.g., pose validity, bounce target)
# 
#  Published topics:
#  - /geofence/status                  (GeofenceStatus)
#  - /geofence/is_inside               (std_msgs/Bool)
#  - /geofence/nearest_boundary_point  (geometry_msgs/PointStamped)
#  - /geofence/distance_to_boundary    (std_msgs/Float32)
#  - /geofence/polygon                 (geometry_msgs/PolygonStamped, latched)
#  - /geofence/markers                 (visualization_msgs/MarkerArray)
# 
#  Services:
#  - /geofence/is_pose_allowed         (IsPoseAllowed)
#  - /geofence/compute_bounce_target   (ComputeBounceTarget)
# 
#  Visualization:
#  - Static polygon boundary and vertices
#  - Dynamic inward-normal marker at nearest boundary point
# 
#  Design notes:
#  - Operates entirely in a single frame (no TF transforms)
#  - Uses hysteresis and debounce to stabilize state transitions
#  - Separates geometry computation from behavior (e.g., wandering/patrol)
# 
#  Intended use:
#  - Safety layer for outdoor navigation (GPS + odometry)
#  - Input to behavior trees, patrol/wander nodes, and recovery logic
# ----------------------------------------------------------------------------


from __future__ import annotations

from typing import List, Optional, Sequence, Tuple
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import Point, Point32, PolygonStamped, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from visualization_msgs.msg import Marker, MarkerArray

from geofence_manager_interfaces.msg import GeofenceStatus
from geofence_manager_interfaces.srv import IsPoseAllowed
from geofence_manager_interfaces.srv import ComputeBounceTarget

from geofence_manager.geometry_utils import point_in_polygon
from geofence_manager.polygon_loader import load_geofence_from_yaml
from geofence_manager.geometry_bounce import (compute_bounce_target, compute_nearest_boundary_hit,)

XY = Tuple[float, float]
INVALID_DISTANCE_M = -1.0

@dataclass
class BoundaryContext:
    x: float
    y: float
    inside: bool
    distance_m: float
    closest_point: XY
    segment_index: int
    tangent_unit: XY
    inward_normal_unit: XY


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
        self.declare_parameter("pose_source_type", "pose_stamped")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("near_boundary_threshold_m", 2.0)
        self.declare_parameter("near_boundary_hysteresis_m", 0.5)
        self.declare_parameter("outside_debounce_count", 3)
        self.declare_parameter("publish_polygon", True)
        self.declare_parameter("publish_markers", True)
        self.declare_parameter("markers_offset_z", 0.3)  # Height of the marker above the ground in RViz, in meters.
        self.declare_parameter("status_publish_rate_hz", 5.0)
        self.declare_parameter("static_publish_rate_hz", 1.0)
        self.declare_parameter("localization_timeout_sec", 2.0)
        self.declare_parameter("publish_inward_normal_marker", True)
        self.declare_parameter("inward_normal_marker_length_m", 1.0)

        self._geofence_file = str(self.get_parameter("geofence_file").value)
        self._pose_topic = str(self.get_parameter("pose_topic").value)
        self._pose_source_type = str(self.get_parameter("pose_source_type").value).strip().lower()
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._near_boundary_threshold_m = float(self.get_parameter("near_boundary_threshold_m").value)
        self._near_boundary_hysteresis_m = float(self.get_parameter("near_boundary_hysteresis_m").value)
        self._outside_debounce_count = int(self.get_parameter("outside_debounce_count").value)
        self._publish_polygon = bool(self.get_parameter("publish_polygon").value)
        self._publish_markers = bool(self.get_parameter("publish_markers").value)
        self._markers_offset_z = float(self.get_parameter("markers_offset_z").value)
        self._status_publish_rate_hz = float(self.get_parameter("status_publish_rate_hz").value)
        self._static_publish_rate_hz = float(self.get_parameter("static_publish_rate_hz").value)
        self._localization_timeout_sec = float(self.get_parameter("localization_timeout_sec").value)
        self._publish_inward_normal_marker = bool(self.get_parameter("publish_inward_normal_marker").value)
        self._inward_normal_marker_length_m = float(self.get_parameter("inward_normal_marker_length_m").value)

        use_sim_time = self.get_parameter("use_sim_time").value
        self.get_logger().info(f"use_sim_time: {use_sim_time}")

        self._zone_name: str = ""
        self._polygon_frame_id: str = self._world_frame
        self._polygon_xy: List[XY] = []

        self._last_pose_x: Optional[float] = None
        self._last_pose_y: Optional[float] = None
        self._last_pose_stamp = None
        self._last_pose_frame_id: str = ""

        self._last_frame_mismatch_warn_key: Optional[str] = None
        self._last_localization_valid: Optional[bool] = None
        self._last_status_state: Optional[str] = None

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

        self._nearest_boundary_point_pub = self.create_publisher(
            PointStamped,
            "/geofence/nearest_boundary_point",
            10,
        )

        self._distance_to_boundary_pub = self.create_publisher(
            Float32,
            "/geofence/distance_to_boundary",
            10,
        )

        self.create_service(
            IsPoseAllowed,
            "/geofence/is_pose_allowed",
            self._handle_is_pose_allowed,
        )

        self.create_service(
            ComputeBounceTarget,
            "/geofence/compute_bounce_target",
            self._handle_compute_bounce_target,
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

        status_timer_period = 1.0 / self._status_publish_rate_hz if self._status_publish_rate_hz > 0.0 else 0.2
        self._status_timer = self.create_timer(status_timer_period, self._publish_status_timer_callback)
        
        static_timer_period = 1.0 / self._static_publish_rate_hz if self._static_publish_rate_hz > 0.0 else 1.0
        self._static_timer = self.create_timer(static_timer_period, self._publish_static_outputs_timer_callback)

        self._stable_state: str = "UNKNOWN"
        self._raw_outside_count: int = 0
        self._raw_inside_count: int = 0

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
        if not rclpy.ok():
            return

        status, ctx = self._build_status_and_context_from_latest_pose()
        self._status_pub.publish(status)

        inside_msg = Bool()
        inside_msg.data = status.is_inside
        self._inside_pub.publish(inside_msg)

        if ctx is not None:
            self._publish_boundary_aux_topics(ctx, status.header.stamp)
            self._publish_dynamic_markers(ctx, status.header.stamp)
        else:
            self._delete_dynamic_markers(status.header.stamp)

        if status.state != self._last_status_state:
            self.get_logger().info(
                f"Geofence state changed to {status.state}"
                f"{' (zone=' + self._zone_name + ')' if self._zone_name else ''}"
            )
            self._last_status_state = status.state


    def _publish_static_outputs_timer_callback(self) -> None:
        if not rclpy.ok():
            return

        if self._publish_polygon:
            self._publish_polygon_msg()

        if self._publish_markers:
            self._publish_marker_msg()


    def _publish_boundary_aux_topics(self, ctx: BoundaryContext, stamp_msg) -> None:
        point_msg = PointStamped()
        point_msg.header.stamp = stamp_msg
        point_msg.header.frame_id = self._polygon_frame_id
        point_msg.point.x = float(ctx.closest_point[0])
        point_msg.point.y = float(ctx.closest_point[1])
        point_msg.point.z = 0.0
        self._nearest_boundary_point_pub.publish(point_msg)

        dist_msg = Float32()
        dist_msg.data = float(ctx.distance_m)
        self._distance_to_boundary_pub.publish(dist_msg)


    def _publish_dynamic_markers(self, ctx: BoundaryContext, stamp_msg) -> None:
        marker_array = MarkerArray()

        if self._publish_inward_normal_marker:
            marker_array.markers.append(self._make_inward_normal_marker(ctx, stamp_msg))

        if marker_array.markers:
            self._marker_pub.publish(marker_array)


    def _delete_dynamic_markers(self, stamp_msg) -> None:
        marker_array = MarkerArray()

        marker = Marker()
        marker.header.stamp = stamp_msg
        marker.header.frame_id = self._polygon_frame_id
        marker.ns = "geofence_inward_normal"
        marker.id = 100
        marker.action = Marker.DELETE

        marker_array.markers.append(marker)
        self._marker_pub.publish(marker_array)


    def _make_inward_normal_marker(self, ctx: BoundaryContext, stamp_msg) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp_msg
        marker.header.frame_id = self._polygon_frame_id
        marker.ns = "geofence_inward_normal"
        marker.id = 100
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.08
        marker.scale.y = 0.16
        marker.scale.z = 0.16

        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 1.0
        marker.color.b = 0.2

        start = Point()
        start.x = float(ctx.closest_point[0])
        start.y = float(ctx.closest_point[1])
        start.z = self._markers_offset_z

        end = Point()
        end.x = float(
            ctx.closest_point[0] + self._inward_normal_marker_length_m * ctx.inward_normal_unit[0]
        )
        end.y = float(
            ctx.closest_point[1] + self._inward_normal_marker_length_m * ctx.inward_normal_unit[1]
        )
        end.z = self._markers_offset_z

        marker.points = [start, end]
        return marker


    def _classify_state_with_hysteresis(self, inside: bool, distance_m: float) -> str:
        """
        Classify geofence state using hysteresis and debounce.

        States:
        - INSIDE
        - NEAR_BOUNDARY
        - OUTSIDE
        - UNKNOWN
        """
        prev = self._stable_state

        near_enter = self._near_boundary_threshold_m
        near_exit = self._near_boundary_threshold_m + self._near_boundary_hysteresis_m

        # Debounce raw inside/outside classification.
        if inside:
            self._raw_inside_count += 1
            self._raw_outside_count = 0
        else:
            self._raw_outside_count += 1
            self._raw_inside_count = 0

        outside_confirmed = self._raw_outside_count >= self._outside_debounce_count
        inside_confirmed = self._raw_inside_count >= 1

        # UNKNOWN bootstrap behavior
        if prev == "UNKNOWN":
            if outside_confirmed:
                return "OUTSIDE"
            if inside and distance_m <= near_enter:
                return "NEAR_BOUNDARY"
            if inside_confirmed:
                return "INSIDE"
            return "UNKNOWN"

        # Once OUTSIDE, require a confirmed return inside.
        if prev == "OUTSIDE":
            if not inside_confirmed:
                return "OUTSIDE"
            return "NEAR_BOUNDARY" if distance_m <= near_exit else "INSIDE"

        # INSIDE transitions
        if prev == "INSIDE":
            if outside_confirmed:
                return "OUTSIDE"
            if inside and distance_m <= near_enter:
                return "NEAR_BOUNDARY"
            return "INSIDE"

        # NEAR_BOUNDARY transitions
        if prev == "NEAR_BOUNDARY":
            if outside_confirmed:
                return "OUTSIDE"
            if inside and distance_m >= near_exit:
                return "INSIDE"
            return "NEAR_BOUNDARY"

        return "UNKNOWN"


    def _handle_compute_bounce_target(
        self,
        request: ComputeBounceTarget.Request,
        response: ComputeBounceTarget.Response,
    ) -> ComputeBounceTarget.Response:
        if self._last_pose_x is None or self._last_pose_y is None or self._last_pose_stamp is None:
            response.success = False
            response.reason = "no valid pose available"
            return response

        if self._last_pose_frame_id and self._last_pose_frame_id != self._polygon_frame_id:
            response.success = False
            response.reason = (
                f"pose frame '{self._last_pose_frame_id}' does not match geofence frame "
                f"'{self._polygon_frame_id}'"
            )
            return response

        pose_age_sec = self._seconds_since_stamp(self._last_pose_stamp)
        if pose_age_sec > self._localization_timeout_sec:
            response.success = False
            response.reason = "latest pose is stale"
            return response

        result = compute_bounce_target(
            robot_xy=(self._last_pose_x, self._last_pose_y),
            polygon=self._polygon_xy,
            bounce_angle_deg=float(request.bounce_angle_deg),
            start_inset_m=float(request.start_inset_m),
            goal_inset_m=float(request.goal_inset_m),
            center_bias=float(request.outside_recovery_bias),
        )

        response.success = result.success
        response.reason = result.reason

        if not result.success:
            return response

        response.target_pose.header.stamp = self.get_clock().now().to_msg()
        response.target_pose.header.frame_id = self._polygon_frame_id
        response.target_pose.pose.position.x = float(result.target_point[0])
        response.target_pose.pose.position.y = float(result.target_point[1])
        response.target_pose.pose.position.z = 0.0
        response.target_pose.pose.orientation.w = 1.0

        response.boundary_point.x = float(result.boundary_point[0])
        response.boundary_point.y = float(result.boundary_point[1])
        response.boundary_point.z = 0.0

        response.far_boundary_point.x = float(result.far_boundary_point[0])
        response.far_boundary_point.y = float(result.far_boundary_point[1])
        response.far_boundary_point.z = 0.0

        ctx = self._compute_boundary_context(self._last_pose_x, self._last_pose_y)
        response.distance_to_boundary_m = float(ctx.distance_m)
        response.used_recovery_mode = bool(getattr(result, "used_recovery_mode", False))

        return response


    def _compute_boundary_context(self, x: float, y: float) -> BoundaryContext:
        hit = compute_nearest_boundary_hit((x, y), self._polygon_xy)
        inside = point_in_polygon(x, y, self._polygon_xy)
        return BoundaryContext(
            x=x,
            y=y,
            inside=inside,
            distance_m=hit.distance_m,
            closest_point=hit.closest_point,
            segment_index=hit.segment_index,
            tangent_unit=hit.tangent_unit,
            inward_normal_unit=hit.inward_normal_unit,
        )


    def _build_status_and_context_from_latest_pose(
        self,
    ) -> Tuple[GeofenceStatus, Optional[BoundaryContext]]:
        msg = GeofenceStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._polygon_frame_id
        msg.zone_name = self._zone_name
        msg.closest_boundary_point = Point()
        msg.distance_to_boundary_m = INVALID_DISTANCE_M

        if self._last_pose_x is None or self._last_pose_y is None or self._last_pose_stamp is None:
            msg.is_inside = False
            msg.is_near_boundary = False
            msg.localization_valid = False
            msg.state = "UNKNOWN"
            self._raw_inside_count = 0
            self._raw_outside_count = 0
            self._stable_state = "UNKNOWN"
            return msg, None

        if self._last_pose_frame_id and self._last_pose_frame_id != self._polygon_frame_id:
            warn_key = f"{self._last_pose_frame_id}->{self._polygon_frame_id}"
            if warn_key != self._last_frame_mismatch_warn_key:
                self.get_logger().warn(
                    f"Pose frame '{self._last_pose_frame_id}' does not match geofence frame "
                    f"'{self._polygon_frame_id}'. Publishing UNKNOWN state."
                )
                self._last_frame_mismatch_warn_key = warn_key

            msg.is_inside = False
            msg.is_near_boundary = False
            msg.localization_valid = False
            msg.state = "UNKNOWN"
            self._raw_inside_count = 0
            self._raw_outside_count = 0
            self._stable_state = "UNKNOWN"
            return msg, None

        self._last_frame_mismatch_warn_key = None

        pose_age_sec = self._seconds_since_stamp(self._last_pose_stamp)
        localization_valid = pose_age_sec <= self._localization_timeout_sec

        if localization_valid != self._last_localization_valid:
            if not localization_valid:
                self.get_logger().warn(
                    f"Localization stale: latest pose age is {pose_age_sec:.2f} s "
                    f"(timeout {self._localization_timeout_sec:.2f} s)."
                )
            self._last_localization_valid = localization_valid

        if not localization_valid:
            msg.is_inside = False
            msg.is_near_boundary = False
            msg.localization_valid = False
            msg.state = "UNKNOWN"
            self._raw_inside_count = 0
            self._raw_outside_count = 0
            self._stable_state = "UNKNOWN"
            return msg, None

        ctx = self._compute_boundary_context(self._last_pose_x, self._last_pose_y)

        state = self._classify_state_with_hysteresis(ctx.inside, ctx.distance_m)
        self._stable_state = state

        msg.localization_valid = True
        msg.distance_to_boundary_m = float(ctx.distance_m)

        msg.closest_boundary_point.x = float(ctx.closest_point[0])
        msg.closest_boundary_point.y = float(ctx.closest_point[1])
        msg.closest_boundary_point.z = 0.0

        msg.state = state
        msg.is_inside = state in ("INSIDE", "NEAR_BOUNDARY")
        msg.is_near_boundary = state == "NEAR_BOUNDARY"

        return msg, ctx


    def _handle_is_pose_allowed(
        self,
        request: IsPoseAllowed.Request,
        response: IsPoseAllowed.Response,
    ) -> IsPoseAllowed.Response:
        pose = request.pose

        request_frame = pose.header.frame_id.strip()
        if request_frame and request_frame != self._polygon_frame_id:
            response.allowed = False
            response.distance_to_boundary_m = INVALID_DISTANCE_M
            response.reason = (
                f"pose frame '{request_frame}' does not match geofence frame "
                f"'{self._polygon_frame_id}'"
            )
            return response

        x = float(pose.pose.position.x)
        y = float(pose.pose.position.y)

        ctx = self._compute_boundary_context(x, y)
        response.allowed = ctx.inside
        response.distance_to_boundary_m = float(ctx.distance_m)
        response.reason = "inside geofence" if ctx.inside else "outside geofence"
        return response


    def _publish_polygon_msg(self) -> None:
        msg = PolygonStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._polygon_frame_id

        for x, y in self._closed_polygon_xy():
            pt = Point32()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = 0.0
            msg.polygon.points.append(pt)

        self._polygon_pub.publish(msg)


    def _publish_marker_msg(self) -> None:
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        line_marker = Marker()
        line_marker.header.stamp = stamp
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
            pt.z = self._markers_offset_z
            line_marker.points.append(pt)

        marker_array.markers.append(line_marker)

        vertex_marker = Marker()
        vertex_marker.header.stamp = stamp
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
            pt.z = self._markers_offset_z
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
        then = Time.from_msg(stamp)
        return max(0.0, (now - then).nanoseconds / 1e9)


    def destroy_node(self) -> bool:
        if hasattr(self, "_status_timer"):
            self._status_timer.cancel()
        if hasattr(self, "_static_timer"):
            self._static_timer.cancel()
        print("Destroying geofence_manager node.")
        return super().destroy_node()


def main(args: Optional[Sequence[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[GeofenceManagerNode] = None

    try:
        node = GeofenceManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            print("Ctrl+C received, shutting down geofence_manager.")
    except Exception:
        if node is not None:
            node.get_logger().exception("Unhandled exception in geofence_manager.")
        raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
