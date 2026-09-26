"""Body attitude estimate for posture control and evaluation (E4).

Publishes /state/attitude (geometry_msgs/Vector3Stamped: x = roll, y = pitch, z = 0,
radians, REP-103) at the IMU rate from /imu/data using attitude.ComplementaryFilter
(gyro + accelerometer only - never the simulator's orientation field, which is ground
truth). locomotion_node's posture controller subscribes to it; hexapod_evaluation
records it next to the ground-truth tilt to report the estimation error.

Still not implemented (the fused TerrainFeatures message, contact ratio / slip index):
publishing fabricated values would mislead hexapod_gait_selector, see the README.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .attitude import ComplementaryFilter


class StateEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("state_estimator_node")
        self.declare_parameter("attitude_tau_s", 0.5)
        self.filter = ComplementaryFilter(tau_s=float(self.get_parameter("attitude_tau_s").value))
        self.publisher = self.create_publisher(Vector3Stamped, "/state/attitude", 10)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.get_logger().info("state_estimator_node: /imu/data -> /state/attitude (complementary filter)")

    def on_imu(self, msg: Imu) -> None:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        g, a = msg.angular_velocity, msg.linear_acceleration
        roll, pitch = self.filter.update(t, (g.x, g.y, g.z), (a.x, a.y, a.z))
        out = Vector3Stamped()
        out.header = msg.header
        out.vector.x, out.vector.y, out.vector.z = float(roll), float(pitch), 0.0
        self.publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = StateEstimatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
