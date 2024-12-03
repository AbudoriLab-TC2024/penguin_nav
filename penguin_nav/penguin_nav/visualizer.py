from typing import List

from rclpy.node import Node
from rclpy.duration import Duration
from visualization_msgs.msg import MarkerArray, Marker
import std_msgs.msg
from geometry_msgs.msg import Point


from penguin_nav.waypoint import Waypoint


class Visualizer:
    def __init__(self, node: Node):
        self._node = node
        self._pub = node.create_publisher(MarkerArray, "/visualization_marker_array", 5)

    def publish(self, wp: List[Waypoint], name, base_color, scale=1.0):
        # delete old
        self._pub.publish(
            MarkerArray(
                markers=[
                    Marker(ns=name, action=Marker.DELETEALL),
                    Marker(ns=f"{name}_torelance", action=Marker.DELETEALL),
                ]
            )
        )
        header = std_msgs.msg.Header(
            frame_id="map", stamp=self._node.get_clock().now().to_msg()
        )
        markers: List[Marker] = []
        for i, p in enumerate(wp):
            if p.action == "stop":
                color = (1.0, 0.0, 0.0)
            elif p.action == "continue":
                color = (0.0, 0.0, 1.0)
            else:
                color = base_color

            m = Marker()
            m.ns = name
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.lifetime = Duration().to_msg()
            m.scale.x = 0.4 * scale
            m.scale.y = 0.1 * scale
            m.scale.z = 0.1 * scale
            m.color.r = color[0]
            m.color.g = color[1]
            m.color.b = color[2]
            m.color.a = 1.0

            m.pose = p.to_pose()
            m.header = header

            markers.append(m)

        for i, p in enumerate(wp):
            m = Marker()
            m.ns = f"{name}_torelance"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.lifetime = Duration().to_msg()
            m.scale.x = 0.02
            m.color.r = 0.0
            m.color.g = 0.0
            m.color.b = 1.0
            m.color.a = 1.0
            m.header = header
            left = p.left()
            right = p.right()
            m.points.append(Point(x=left[0], y=left[1], z=0.0))
            m.points.append(Point(x=right[0], y=right[1], z=0.0))

            markers.append(m)

        self._pub.publish(MarkerArray(markers=markers))
