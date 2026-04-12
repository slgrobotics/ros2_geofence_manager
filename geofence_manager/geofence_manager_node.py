import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PolygonStamped
from std_msgs.msg import Bool

from geofence_manager_interfaces.msg import GeofenceStatus
from geofence_manager_interfaces.srv import IsPoseAllowed

class GeofenceManagerNode(Node):

    def __init__(self):
        super().__init__('geofence_manager')

        self.declare_parameter('geofence_file', '')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('pose_topic', '/odometry/global')
        self.declare_parameter('near_boundary_threshold_m', 2.0)

        self.get_logger().info('Geofence Manager started')

        # Publishers
        self.status_pub = self.create_publisher(
            GeofenceStatus,
            '/geofence/status',
            10
        )

        self.inside_pub = self.create_publisher(
            Bool,
            '/geofence/is_inside',
            10
        )

        # Services
        self.create_service(
            IsPoseAllowed,
            '/geofence/is_pose_allowed',
            self.handle_is_pose_allowed
        )

        # TODO: load polygon
        # TODO: subscribe to pose
        # TODO: compute status

    def handle_is_pose_allowed(self, request, response):
        # TODO: implement real check
        response.allowed = True
        response.distance_to_boundary_m = 999.0
        response.reason = "not implemented"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
