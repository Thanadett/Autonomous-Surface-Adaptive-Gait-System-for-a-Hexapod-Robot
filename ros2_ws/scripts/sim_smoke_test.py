#!/usr/bin/env python3
"""Smoke test for the full simulation stack (hexapod_bringup/sim.launch.py).

Run through scripts/sim_smoke_test.sh, which starts the launch file, runs this
checker against it and shuts everything down again. Stand-alone use (with the
simulation already running):

    python3 scripts/sim_smoke_test.py

What it checks, in order:
  1. controller_manager has joint_state_broadcaster + hexapod_controller active
     and locomotion_node reports /locomotion/ready (robot_description parsed,
     CadRobotKinematics accepted the URDF).
  2. Standing: base_link height/tilt from the ground-truth odom -> base_link TF,
     all 18 joints in /joint_states, sensor rates measured in *sim* time
     (IMU, depth image, point cloud, camera_info) and a contact message from
     every foot (limit-switch emulation).
  3. Walking forward in MANUAL:tripod, then MANUAL:wave: forward speed along
     the initial heading, worst roll/pitch, lowest body height (fall check).

Thresholds are deliberately loose - this is a "does the pipeline work" test,
not a performance evaluation (that is E1-E7 in the simulation plan).
Exit code 0 = no FAIL (WARNs allowed), 1 = at least one FAIL.
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time

from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist, Vector3Stamped, WrenchStamped
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # reported as a FAIL below instead of crashing
    Contacts = None
try:
    from hexapod_interfaces.srv import SetGaitMode
    from hexapod_interfaces.msg import LocomotionState
except ImportError:
    SetGaitMode = None
    LocomotionState = None

LEGS = ("front_left", "middle_left", "rear_left", "front_right", "middle_right", "rear_right")
JOINTS = [f"{leg}_{seg}_joint" for leg in LEGS for seg in ("coxa", "femur", "tibia")]
FOOT_RADIUS = 0.0163
STANCE_HEIGHT = 0.100  # hexapod_locomotion/config/gait.yaml
WALK_SPEED = 0.025     # m/s = common test speed of E2-E7 (Wave's top speed with its 2.4 s cycle)
WALK_SECONDS = 12.0    # wall time per gait

results: list[tuple[str, str, str]] = []  # (level, check, detail)


def record(level: str, check: str, detail: str) -> None:
    results.append((level, check, detail))
    print(f"[{level:4s}] {check}: {detail}", flush=True)


class TopicStats:
    """Message count and sim-time span (header stamps) for one topic."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.count = 0
        self.first = None
        self.last = None
        self.last_msg = None

    def add(self, msg) -> None:
        stamp = getattr(getattr(msg, "header", None), "stamp", None)
        if stamp is not None:
            t = stamp.sec + stamp.nanosec * 1e-9
            if t > 0.0:
                self.first = t if self.first is None else self.first
                self.last = t
        self.count += 1
        self.last_msg = msg

    def rate(self) -> float | None:
        if self.count < 2 or self.first is None or self.last is None or self.last <= self.first:
            return None
        return (self.count - 1) / (self.last - self.first)


class SmokeTest(Node):
    def __init__(self) -> None:
        super().__init__(
            "hexapod_sim_smoke_test",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.stats: dict[str, TopicStats] = {}
        self._sub("/imu/data", Imu)
        self._sub("/camera/depth_image", Image)
        self._sub("/camera/points", PointCloud2)
        self._sub("/camera/camera_info", CameraInfo)
        self._sub("/joint_states", JointState)
        self._sub("/joint_wrench/front_left_femur", WrenchStamped)  # joint F/T sensors (CoT, E2)
        self._sub("/state/attitude", Vector3Stamped)                # IMU attitude estimate (E4 posture control)
        self._sub("/camera/image", Image)                           # RGB (E6 snapshots)
        self.loco_swing_max = 0
        self.loco_margin_min = math.inf
        if LocomotionState is not None:
            self.create_subscription(LocomotionState, "/locomotion_state", self._on_loco, 10)
        self.contact_hits = {leg: 0 for leg in LEGS}
        if Contacts is not None:
            for leg in LEGS:
                self.create_subscription(
                    Contacts, f"/foot_contacts/{leg}",
                    lambda msg, leg=leg: self._on_contact(leg, msg), qos_profile_sensor_data)
        self.ready = False
        self.create_subscription(Bool, "/locomotion/ready", self._on_ready, 10)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.list_controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.set_gait = self.create_client(SetGaitMode, "/set_gait_mode") if SetGaitMode else None

    # -- plumbing -------------------------------------------------------
    def _sub(self, topic: str, msg_type) -> None:
        stats = TopicStats()
        self.stats[topic] = stats
        self.create_subscription(msg_type, topic, stats.add, qos_profile_sensor_data)

    def _on_contact(self, leg: str, msg) -> None:
        if len(msg.contacts) > 0:
            self.contact_hits[leg] += 1

    def _on_loco(self, msg) -> None:
        self.loco_swing_max = max(self.loco_swing_max, sum(bool(v) for v in msg.leg_in_swing))
        if math.isfinite(msg.stability_margin_m):
            self.loco_margin_min = min(self.loco_margin_min, msg.stability_margin_m)

    def _on_ready(self, msg: Bool) -> None:
        self.ready = self.ready or bool(msg.data)

    def spin_for(self, seconds: float, every=None) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if every is not None:
                every()

    def call(self, client, request, timeout: float = 5.0):
        if client is None or not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result() if future.done() else None

    def base_pose(self):
        """(sim time, x, y, z, roll, pitch, yaw) of base_link in odom, or None."""
        try:
            tf = self.tf_buffer.lookup_transform("odom", "base_link", Time())
        except Exception:  # noqa: BLE001 - any TF failure just means "not yet"
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        return stamp, t.x, t.y, t.z, roll, pitch, yaw

    def publish_cmd(self, vx: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        self.cmd.publish(msg)

    # -- phases ---------------------------------------------------------
    def wait_for_stack(self, timeout: float = 120.0) -> bool:
        end = time.monotonic() + timeout
        active: dict[str, str] = {}
        while time.monotonic() < end:
            response = self.call(self.list_controllers, ListControllers.Request(), timeout=3.0)
            if response is not None:
                active = {c.name: c.state for c in response.controller}
                if active.get("joint_state_broadcaster") == "active" and active.get("hexapod_controller") == "active":
                    break
            self.spin_for(1.0)
        else:
            record("FAIL", "controllers", f"not both active after {timeout:.0f} s: {active or 'controller_manager not reachable'}")
            return False
        record("PASS", "controllers", "joint_state_broadcaster + hexapod_controller active")

        self.spin_for(0.1)
        end = time.monotonic() + 60.0
        while not self.ready and time.monotonic() < end:
            self.spin_for(0.5)
        if not self.ready:
            record("FAIL", "locomotion_node", "/locomotion/ready never became true (robot_description/URDF not accepted? see launch log)")
            return False
        record("PASS", "locomotion_node", "ready (CadRobotKinematics parsed the URDF)")
        return True

    def check_standing(self) -> None:
        self.spin_for(6.0)  # stand-up ramp (1.5 s sim) + settling
        for s in self.stats.values():
            s.reset()
        self.contact_hits = {leg: 0 for leg in LEGS}
        self.spin_for(5.0)

        pose = self.base_pose()
        if pose is None:
            record("FAIL", "odom TF", "no odom -> base_link transform (OdometryPublisher / bridge /odom_tf)")
        else:
            _, _, _, z, roll, pitch, _ = pose
            expected = STANCE_HEIGHT + FOOT_RADIUS
            level = "PASS" if abs(z - expected) < 0.03 else "FAIL"
            record(level, "standing height", f"base_link z = {z * 1000:.1f} mm (expected ~{expected * 1000:.0f} mm on flat ground)")
            tilt = max(abs(math.degrees(roll)), abs(math.degrees(pitch)))
            record("PASS" if tilt < 5.0 else "FAIL", "standing tilt",
                   f"roll {math.degrees(roll):+.1f} deg, pitch {math.degrees(pitch):+.1f} deg")

        js = self.stats["/joint_states"].last_msg
        missing = sorted(set(JOINTS) - set(js.name)) if js is not None else JOINTS
        record("PASS" if not missing else "FAIL", "joint_states",
               "all 18 joints present" if not missing else f"missing {missing}")

        for topic, minimum, nominal in (("/imu/data", 90.0, 100), ("/joint_states", 20.0, 100),
                                        ("/camera/depth_image", 10.0, 30), ("/camera/points", 5.0, 30),
                                        ("/camera/camera_info", 10.0, 30)):
            rate = self.stats[topic].rate()
            if rate is None:
                record("FAIL", topic, f"no data ({self.stats[topic].count} msgs)")
            else:
                record("PASS" if rate >= minimum else "WARN", topic,
                       f"{rate:.1f} Hz sim-time (configured {nominal} Hz)")

        rate = self.stats["/joint_wrench/front_left_femur"].rate()
        # WARN only: without the F/T sensors E2 still runs, just without CoT / peak torque
        record("PASS" if rate is not None and rate >= 40.0 else "WARN", "/joint_wrench (joint F/T)",
               f"{rate:.1f} Hz sim-time (configured 50 Hz)" if rate is not None
               else "no data - check the ForceTorque world plugin / bridge.yaml (E2 CoT will be NaN)")

        att = self.stats["/state/attitude"]
        if att.rate() is None or att.last_msg is None:
            record("FAIL", "/state/attitude", "no data - state_estimator_node not running? (posture control needs it)")
        else:
            roll, pitch = math.degrees(att.last_msg.vector.x), math.degrees(att.last_msg.vector.y)
            record("PASS" if abs(roll) < 2 and abs(pitch) < 2 else "FAIL", "/state/attitude",
                   f"{att.rate():.0f} Hz, standing roll {roll:+.2f} / pitch {pitch:+.2f} deg (ground truth ~0)")
        rgb = self.stats["/camera/image"].rate()
        record("PASS" if rgb is not None and rgb >= 10 else "WARN", "/camera/image",
               f"{rgb:.1f} Hz" if rgb is not None else "no data (bridge.yaml /camera/image)")

        if Contacts is None:
            record("FAIL", "foot contacts", "ros_gz_interfaces not importable")
        else:
            silent = [leg for leg, n in self.contact_hits.items() if n == 0]
            detail = ", ".join(f"{leg} {n}" for leg, n in self.contact_hits.items())
            record("PASS" if not silent else "FAIL", "foot contacts (standing, 5 s)",
                   detail + ("" if not silent else f" -> no contact from {silent}"))

    def walk(self, gait: str) -> None:
        if self.set_gait is not None:
            response = self.call(self.set_gait, SetGaitMode.Request(mode=f"MANUAL:{gait}"))
            if response is None or not response.accepted:
                record("FAIL", f"set_gait_mode {gait}", "service unavailable or rejected")
                return
            # let locomotion_node apply the switch while standing still (it then completes at
            # once); walking straight away would blend over the first cycle instead
            for _ in range(10):
                self.publish_cmd(0.0)
                self.spin_for(0.1)
        start = self.base_pose()
        if start is None:
            record("FAIL", f"walk {gait}", "no odom TF")
            return
        self.loco_swing_max, self.loco_margin_min = 0, math.inf
        worst_tilt, lowest = 0.0, start[3]

        def tick():
            nonlocal worst_tilt, lowest
            self.publish_cmd(WALK_SPEED)
            pose = self.base_pose()
            if pose is not None:
                worst_tilt = max(worst_tilt, abs(math.degrees(pose[4])), abs(math.degrees(pose[5])))
                lowest = min(lowest, pose[3])

        self.spin_for(WALK_SECONDS, tick)
        end = self.base_pose()
        for _ in range(10):
            self.publish_cmd(0.0)
            self.spin_for(0.1)
        self.spin_for(2.0)
        if end is None or end[0] <= start[0]:
            record("FAIL", f"walk {gait}", "odom TF stopped updating")
            return
        dt = end[0] - start[0]
        dx, dy = end[1] - start[1], end[2] - start[2]
        forward = dx * math.cos(start[6]) + dy * math.sin(start[6])
        speed = forward / dt
        record("PASS" if speed > 0.3 * WALK_SPEED else "FAIL", f"walk {gait} speed",
               f"{speed * 1000:.1f} mm/s forward over {dt:.1f} s sim (commanded {WALK_SPEED * 1000:.0f} mm/s)")
        record("PASS" if worst_tilt < 15.0 else "FAIL", f"walk {gait} tilt", f"worst |roll|/|pitch| {worst_tilt:.1f} deg")
        record("PASS" if lowest > 0.06 else "FAIL", f"walk {gait} body height", f"lowest base_link z {lowest * 1000:.1f} mm")
        # steady state (the switch happened at rest): tripod 3, ripple 2, wave 1 legs in swing
        expected_swing = {"tripod": 3, "ripple": 2, "wave": 1}[gait]
        ok = self.loco_swing_max == expected_swing and 0.0 < self.loco_margin_min < math.inf
        record("PASS" if ok else "FAIL", f"walk {gait} locomotion_state",
               f"max legs in swing {self.loco_swing_max} (expected {expected_swing}), "
               f"planned SSM min {self.loco_margin_min * 1000:.1f} mm")


def main() -> int:
    rclpy.init()
    node = SmokeTest()
    try:
        if node.wait_for_stack():
            node.check_standing()
            node.walk("tripod")
            node.walk("wave")
    except KeyboardInterrupt:
        record("FAIL", "smoke test", "interrupted")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    fails = sum(1 for level, _, _ in results if level == "FAIL")
    warns = sum(1 for level, _, _ in results if level == "WARN")
    print(f"\nSUMMARY: {len(results) - fails - warns} pass, {warns} warn, {fails} fail", flush=True)
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
