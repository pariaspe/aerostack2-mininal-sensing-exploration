"""A ROS 2 node to publish markers for geofence visualization in RViz."""

from as2_msgs.msg import PolygonList
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from visualization_msgs.msg import Marker, MarkerArray


class GeozonesMarkerPublisherNode(Node):
    """Geozones main class."""

    MARKERS_FREQ = 0.1
    MARKERS_LIFETIME = 5  # seconds

    def __init__(self) -> None:
        super().__init__('geozones_marker_publisher_node')

        self.declare_parameter('namespace', 'drone0')
        self.namespace = (
            self.get_parameter('namespace').get_parameter_value().string_value
        )

        self.declare_parameter('color', 'green')
        self.color = self.get_parameter('color').get_parameter_value().string_value

        self.geozones_sub = self.create_subscription(
            PolygonList, f'/{self.namespace}/geozones_rviz', self.geofence_callback, 1
        )
        self.geozones_pub = self.create_publisher(
            MarkerArray, f'/viz/{self.namespace}/geofences', qos_profile_system_default
        )

    def geofence_callback(self, msg: PolygonList) -> None:
        """Geofence callback."""
        match self.color:
            case 'red':
                r, g, b = 1.0, 0.0, 0.0
            case 'green':
                r, g, b = 0.0, 1.0, 0.0
            case 'blue':
                r, g, b = 0.0, 0.0, 1.0
            case _:
                r, g, b = 1.0, 0.0, 0.0

        marker_array = MarkerArray()
        for i, polygon in enumerate(msg.polygons):
            marker = Marker()
            marker.header = polygon.header
            marker.ns = 'polygons'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.lifetime = Duration(seconds=self.MARKERS_LIFETIME).to_msg()
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1  # Line width
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.5  # Fully opaque
            for point in polygon.polygon.points:
                rviz_point = Point()
                rviz_point.x = point.x
                rviz_point.y = point.y
                rviz_point.z = point.z
                marker.points.append(rviz_point)
            # Close the polygon
            rviz_point = Point()
            rviz_point.x = polygon.polygon.points[0].x
            rviz_point.y = polygon.polygon.points[0].y
            rviz_point.z = polygon.polygon.points[0].z
            marker.points.append(rviz_point)
            marker_array.markers.append(marker)

        self.geozones_pub.publish(marker_array)
