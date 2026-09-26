"""Persistent voxel map built from the depth camera's point cloud.

Each /camera/points cloud is transformed into a fixed frame (default `odom`),
snapped to a voxel grid, and merged into a set of occupied voxels that is
never forgotten (up to max_voxels). The map is republished as
/map_cloud (latched, so RViz shows it even if opened later). Call
/map/clear (std_srvs/Trigger) to start over.

Needs a TF chain target_frame -> camera frame; in simulation the odom ->
base_link part is Gazebo's ground-truth pose (see hexapod.urdf.xacro).
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
from tf2_ros import Buffer, ExtrapolationException, TransformException, TransformListener

_BITS = 21
_OFFSET = 1 << (_BITS - 1)
_MASK = (1 << _BITS) - 1


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )
    )


class VoxelMapNode(Node):
    def __init__(self) -> None:
        super().__init__("voxel_map_node")
        self.declare_parameter("input_topic", "/camera/points")
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("resolution_m", 0.05)
        self.declare_parameter("min_range_m", 0.3)
        self.declare_parameter("max_range_m", 1.8)
        self.declare_parameter("max_voxels", 500_000)
        self.declare_parameter("integrate_rate_hz", 5.0)
        self.declare_parameter("publish_rate_hz", 1.0)

        self.target_frame = str(self.get_parameter("target_frame").value)
        self.resolution = float(self.get_parameter("resolution_m").value)
        self.min_range = float(self.get_parameter("min_range_m").value)
        self.max_range = float(self.get_parameter("max_range_m").value)
        self.max_voxels = int(self.get_parameter("max_voxels").value)
        self.min_period = 1.0 / float(self.get_parameter("integrate_rate_hz").value)

        self.voxels: set[int] = set()
        self.dirty = False
        self.last_integrated = None
        self.full_warned = False

        self.tf_buffer = Buffer()
        # Own spin thread: lookups must not depend on this node's executor.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(PointCloud2, "/map_cloud", latched)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter("input_topic").value),
            self.on_points,
            qos_profile_sensor_data,
        )
        self.create_service(Trigger, "/map/clear", self.on_clear)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate_hz").value), self.publish_map)
        self.get_logger().info(
            f"voxel_map_node: {self.resolution} m voxels in '{self.target_frame}', "
            f"range {self.min_range}-{self.max_range} m"
        )

    def on_clear(self, _request, response):
        count = len(self.voxels)
        self.voxels.clear()
        self.dirty = True
        self.full_warned = False
        response.success = True
        response.message = f"cleared {count} voxels"
        return response

    def on_points(self, cloud: PointCloud2) -> None:
        now = self.get_clock().now()
        if self.last_integrated is not None and (now - self.last_integrated).nanoseconds * 1e-9 < self.min_period:
            return
        try:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, cloud.header.frame_id, cloud.header.stamp
                )
            except ExtrapolationException:
                # Cloud stamp can be a millisecond ahead of the newest odom
                # TF; the robot barely moves in that time, so use the latest.
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame, cloud.header.frame_id, rclpy.time.Time()
                )
        except TransformException as error:
            self.get_logger().warning(f"waiting for TF: {error}", throttle_duration_sec=5.0)
            return
        self.last_integrated = now

        offsets = {field.name: field.offset for field in cloud.fields}
        if not {"x", "y", "z"}.issubset(offsets):
            self.get_logger().error("input cloud has no x/y/z fields", throttle_duration_sec=5.0)
            return
        raw = np.frombuffer(cloud.data, dtype=np.uint8).reshape(-1, cloud.point_step)
        points = np.stack(
            [raw[:, offsets[axis] : offsets[axis] + 4].copy().view(np.float32).ravel() for axis in "xyz"],
            axis=1,
        ).astype(np.float64)
        distance = np.linalg.norm(points, axis=1)
        keep = np.isfinite(distance) & (distance >= self.min_range) & (distance <= self.max_range)
        points = points[keep]
        if points.size == 0:
            return

        q = transform.transform.rotation
        t = transform.transform.translation
        world = points @ _quat_to_matrix(q.x, q.y, q.z, q.w).T + np.array((t.x, t.y, t.z))
        index = np.floor(world / self.resolution).astype(np.int64) + _OFFSET
        valid = np.all((index >= 0) & (index <= _MASK), axis=1)
        index = index[valid]
        keys = np.unique((index[:, 0] << (2 * _BITS)) | (index[:, 1] << _BITS) | index[:, 2])

        before = len(self.voxels)
        if before + keys.size > self.max_voxels:
            keys = keys[: max(0, self.max_voxels - before)]
            if not self.full_warned:
                self.get_logger().warning(f"map reached max_voxels={self.max_voxels}; new voxels dropped")
                self.full_warned = True
        self.voxels.update(keys.tolist())
        if len(self.voxels) != before:
            self.dirty = True

    def publish_map(self) -> None:
        if not self.dirty:
            return
        self.dirty = False
        keys = np.fromiter(self.voxels, dtype=np.int64, count=len(self.voxels))
        index = np.stack(
            ((keys >> (2 * _BITS)) & _MASK, (keys >> _BITS) & _MASK, keys & _MASK), axis=1
        )
        centers = ((index - _OFFSET + 0.5) * self.resolution).astype(np.float32)

        message = PointCloud2()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.target_frame
        message.height = 1
        message.width = centers.shape[0]
        message.fields = [
            PointField(name=axis, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, axis in enumerate("xyz")
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * centers.shape[0]
        message.is_dense = True
        message.data = centers.tobytes()
        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = VoxelMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
