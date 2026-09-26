"""P2 stub -- not yet implemented.

Planned pipeline (see the project plan section 5): subscribe
/camera/points (sensor_msgs/PointCloud2, bridged from Gazebo's rgbd_camera
by hexapod_simulation/config/bridge.yaml), rotate into a gravity-aligned
frame using /imu/data, crop to the region 0.3-1.0 m ahead of the robot,
voxel-downsample, RANSAC plane fit for slope, take the plane residual as
roughness, and bin an elevation grid for max step height. Publish the
result as hexapod_interfaces/msg/TerrainFeatures (slope/roughness/step
fields only -- hexapod_state_estimation fills in the body-state fields).

This stub exists so the workspace builds and so hexapod_gait_selector has
a documented, addressable input topic to point at, not because there is
placeholder logic worth shipping here yet: publishing fabricated terrain
numbers would be actively misleading to whatever subscribes to them.
"""
import rclpy
from rclpy.node import Node


class TerrainEstimatorNode(Node):
    def __init__(self) -> None:
        super().__init__("terrain_estimator_node")
        self.get_logger().warning(
            "terrain_estimator_node is a P2 stub: it does not process "
            "/camera/points yet and publishes nothing. See "
            "hexapod_perception/README.md."
        )


def main() -> None:
    rclpy.init()
    node = TerrainEstimatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
