#!/usr/bin/env python3
"""Republish Gazebo's /camera/points_gz as /camera/points in camera_link.

Gazebo emits the cloud in x-forward/z-up axes but the sensor's single
frame_id must stay camera_optical_frame for the depth image/camera_info
(RViz DepthCloud assumes z forward). Left as-is, the cloud renders rolled 90
degrees; relabeling to camera_link (same axes as the data) fixes that.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointsFrameRelabel(Node):
    def __init__(self) -> None:
        super().__init__("points_frame_relabel")
        self.declare_parameter("frame_id", "camera_link")
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.publisher = self.create_publisher(PointCloud2, "/camera/points", 5)
        self.create_subscription(PointCloud2, "/camera/points_gz", self.on_points, qos_profile_sensor_data)

    def on_points(self, message: PointCloud2) -> None:
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = PointsFrameRelabel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
