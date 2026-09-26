"""Automated walking trials in a running simulation: E2, E2b, E3, E4, E5 (+ E3 calibration).

    ros2 run hexapod_evaluation e2_runner --out results/e2 --config .../e2.yaml   # one plan
    ros2 run hexapod_evaluation e2_runner ... --quick      # 2 trials x 0.5 m per condition
    ros2 run hexapod_evaluation e2_runner ... --check      # 1 short trial of the first/last condition
    ros2 run hexapod_evaluation e2_runner ... --resume     # skip trials already in e2_trials.csv

Normally started by scripts/run_e2.sh (one plan) or scripts/run_all.sh (the overnight
batch), which launch hexapod_bringup/sim.launch.py (flat_world) first. One simulation
serves every trial of a plan:

  per trial:  stop -> posture (body/step height) -> gait -> posture control on/off ->
              [terrain plans: park the robot away, create the terrain piece
              (scenario.py: friction mat, ramp, obstacle)] -> teleport onto the start +
              random offset (+-2 cm, +-3 deg) -> settle -> walk at the commanded speed until
              goal / fall / time limit -> stop -> [park, remove the piece] -> metrics
  order:      repetition-major (rep 1 of every condition, then rep 2, ...) so an
              interrupted run still has every condition; E5 runs ladders (plan "obstacles",
              e2_plan.ladder_groups); E3 calibration (plan "calibration") runs e3_calib.
  seeds:      --seed fixes the start offsets. Metrics use ground truth; the IMU (and the
              attitude estimate from it) is recorded for E4/E6.

Samples: every ground-truth pose (/tf odom->base_link, 50 Hz) with the nearest
/joint_states, latest joint torques (/joint_wrench/*), contacts (/foot_contacts/*), the
LocomotionState, raw IMU, /state/attitude. Metrics: metrics.evaluate().
Camera (plan record_camera, launch with use_camera:=true): RGB + depth, 4x downsampled,
at 2 Hz with the pose, into snapshots/<trial>.npz - the E6 dataset and the offline
check of the obstacle-height estimate (E5).

Outputs (--out): e2_trials.csv, e2_summary.md/.json, e2_meta.json, e2_robot.urdf,
samples/<trial>.npz (recompute metrics offline: e2_reevaluate), snapshots/, bags/ (--bag).
"""
from __future__ import annotations

import argparse
from collections import deque
import csv
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped, WrenchStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, JointState
from std_msgs.msg import Bool, Float64, String
from tf2_msgs.msg import TFMessage

from hexapod_interfaces.msg import LocomotionState
from hexapod_interfaces.srv import SetGaitMode
from hexapod_kinematics import LEGS, SEGMENTS, WholeBodyModel

from .metrics import JOINTS, TrialSamples, TrialSpec, evaluate, fall_reason, foot_radius_from_urdf, quat_to_rpy
from .e2_plan import conditions, gait_cycle_times, gait_params, ladder_groups, load_plan
from .e2_report import write_summary
from . import scenario as scn

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # contact metrics become NaN, reported in the log
    Contacts = None

CONTACT_HOLD_S = 0.025   # a foot counts as "in contact" this long after a non-empty contact message
SNAPSHOT_PERIOD_S = 0.5  # camera snapshots while walking (E6 dataset)
SNAPSHOT_STRIDE = 4      # 424 x 240 -> 106 x 60
BAG_TOPICS = ["/tf", "/joint_states", "/locomotion_state", "/cmd_vel", "/imu/data", "/state/attitude"] + \
    [f"/foot_contacts/{leg}" for leg in LEGS] + [f"/joint_wrench/{leg}_{seg}" for leg in LEGS for seg in SEGMENTS]
ROW_KEYS = ("trial", "condition", "group", "speed_label", "rep", "t_command", "warmup_s", "distance_m",
            "stance_m", "step_m", "cadence_scale", "posture_control", "scenario", "surface_mu", "soft",
            "slope_deg", "obstacle_h_m", "ground_x", "ground_y", "ground_z",
            "start_x", "start_y", "start_yaw_deg", "torque_msgs", "contact_msgs",
            "imu_msgs", "snapshots", "start_height_mm", "rtf")
TERRAIN_PREFIXES = ("scn_", "calib_")   # models this runner creates (everything else belongs to the world)
START_HEIGHT_TOL_M = 0.025              # base_link height above the terrain surface after settling vs expected
MIN_WALL_RTF = 0.25                     # a walk that runs slower than this is aborted (wall-clock guard)


class RunAbort(RuntimeError):
    """The world is no longer in a known state (terrain left behind, simulation stalled):
    stop the plan instead of writing trials that walked on the wrong terrain."""


def image_array(msg: Image) -> np.ndarray | None:
    """sensor_msgs/Image (rgb8 / 32FC1) -> numpy, SNAPSHOT_STRIDE-downsampled."""
    if msg is None or msg.height == 0:
        return None
    if msg.encoding in ("rgb8", "bgr8"):
        data = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
        image = data.reshape(msg.height, msg.width, 3)
        if msg.encoding == "bgr8":
            image = image[:, :, ::-1]
        return np.ascontiguousarray(image[::SNAPSHOT_STRIDE, ::SNAPSHOT_STRIDE])
    if msg.encoding == "32FC1":
        data = np.frombuffer(bytes(msg.data), np.float32).reshape(msg.height, msg.step // 4)[:, : msg.width]
        return data[::SNAPSHOT_STRIDE, ::SNAPSHOT_STRIDE].astype(np.float16)
    return None


class TrialRecorder(Node):
    def __init__(self, model: WholeBodyModel, axes: dict[str, np.ndarray], record_camera: bool = False) -> None:
        super().__init__("e2_runner", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.model = model
        self.axes = axes
        self.recording = False
        self.rows: list[dict] = []
        self.pose = None                     # (t, pos, quat) latest ground truth
        self.joint_buffer: deque = deque(maxlen=64)
        self.torque = {name: (math.nan, -math.inf) for name in JOINTS}   # (tau, stamp)
        self.last_contact = np.full(6, -math.inf)
        self.contact_messages = 0
        self.torque_messages = 0
        self.imu_messages = 0
        self.loco: LocomotionState | None = None
        self.imu = [math.nan] * 6
        self.attitude = [math.nan, math.nan]
        self.rgb_msg = None
        self.depth_msg = None
        self.snapshots: list[dict] = []
        self.last_snapshot_t = -math.inf

        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 50)
        self.create_subscription(LocomotionState, "/locomotion_state", self.on_loco, 20)
        self.create_subscription(Imu, "/imu/data", self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped, "/state/attitude", self.on_attitude, 20)
        if Contacts is not None:
            for i, leg in enumerate(LEGS):
                self.create_subscription(Contacts, f"/foot_contacts/{leg}",
                                         lambda msg, i=i: self.on_contact(i, msg), qos_profile_sensor_data)
        for name in JOINTS:
            topic = "/joint_wrench/" + name[: -len("_joint")]
            self.create_subscription(WrenchStamped, topic, lambda msg, name=name: self.on_wrench(name, msg),
                                     qos_profile_sensor_data)
        if record_camera:
            self.create_subscription(Image, "/camera/image", self.on_rgb, qos_profile_sensor_data)
            self.create_subscription(Image, "/camera/depth_image", self.on_depth, qos_profile_sensor_data)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.stance_cmd = self.create_publisher(Float64, "/stance_height_command", 10)
        self.step_cmd = self.create_publisher(Float64, "/step_height_command", 10)
        self.posture_cmd = self.create_publisher(Bool, "/posture_control/enable", 10)
        self.set_mode = self.create_client(SetGaitMode, "/set_gait_mode")

    # -- callbacks (kept minimal: ~2k msgs/s arrive during a trial) -----------
    @staticmethod
    def _stamp(header) -> float:
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def on_joint_state(self, msg: JointState) -> None:
        if not hasattr(self, "_js_index") or len(self._js_index) != len(msg.name):
            index = {name: i for i, name in enumerate(msg.name)}
            if not all(name in index for name in JOINTS):
                return
            self._js_index = [index[name] for name in JOINTS]
        idx = self._js_index
        velocity = [msg.velocity[i] for i in idx] if len(msg.velocity) == len(msg.name) else [math.nan] * 18
        self.joint_buffer.append((self._stamp(msg.header), [msg.position[i] for i in idx], velocity))

    def on_wrench(self, name: str, msg: WrenchStamped) -> None:
        torque = msg.wrench.torque
        tau = float(np.dot(self.axes[name], (torque.x, torque.y, torque.z)))
        self.torque[name] = (tau, self._stamp(msg.header))
        self.torque_messages += 1

    def on_contact(self, leg_index: int, msg) -> None:
        if len(msg.contacts) > 0:
            self.last_contact[leg_index] = self._stamp(msg.header)
            self.contact_messages += 1

    def on_loco(self, msg: LocomotionState) -> None:
        self.loco = msg

    def on_imu(self, msg: Imu) -> None:
        g, a = msg.angular_velocity, msg.linear_acceleration
        self.imu = [g.x, g.y, g.z, a.x, a.y, a.z]
        self.imu_messages += 1

    def on_attitude(self, msg: Vector3Stamped) -> None:
        self.attitude = [msg.vector.x, msg.vector.y]

    def on_rgb(self, msg: Image) -> None:
        self.rgb_msg = msg

    def on_depth(self, msg: Image) -> None:
        self.depth_msg = msg

    def on_tf(self, msg: TFMessage) -> None:
        for tf in msg.transforms:
            if tf.child_frame_id != "base_link" or tf.header.frame_id != "odom":
                continue
            t = self._stamp(tf.header)
            p, q = tf.transform.translation, tf.transform.rotation
            self.pose = (t, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
            if self.recording:
                self._record(t, self.pose[1], self.pose[2])

    def _record(self, t: float, pos, quat) -> None:
        if not self.joint_buffer:
            return
        stamp, q, qd = min(self.joint_buffer, key=lambda item: abs(item[0] - t))
        tau = [value if t - stamp_w < 0.1 else math.nan for value, stamp_w in (self.torque[n] for n in JOINTS)]
        loco = self.loco
        posture = ([getattr(loco, "posture_roll_rad", math.nan), getattr(loco, "posture_pitch_rad", math.nan)]
                   if loco is not None else [math.nan, math.nan])
        self.rows.append({
            "t": t, "pos": pos, "quat": quat, "q": q, "qd": qd, "tau": tau,
            "contact": list(t - self.last_contact <= CONTACT_HOLD_S),
            "planned_ssm": loco.stability_margin_m if loco is not None else math.nan,
            "swing_plan": list(loco.leg_in_swing) if loco is not None else [False] * 6,
            "imu": list(self.imu), "att_est": list(self.attitude), "posture": posture,
        })
        if t - self.last_snapshot_t >= SNAPSHOT_PERIOD_S and (self.rgb_msg is not None or self.depth_msg is not None):
            self.last_snapshot_t = t
            self.snapshots.append({"t": t, "pos": pos, "quat": quat,
                                   "rgb": image_array(self.rgb_msg), "depth": image_array(self.depth_msg)})

    # -- helpers ---------------------------------------------------------------
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_sim(self, seconds: float, every=None, stop=None, wall_limit: float | None = None) -> bool:
        """Spin for `seconds` of sim time; returns True if `stop()` ended it early."""
        start = self.now()
        wall_end = time.monotonic() + (wall_limit if wall_limit is not None else 60.0 + 10.0 * seconds)
        while self.now() - start < seconds and time.monotonic() < wall_end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if every is not None:
                every()
            if stop is not None and stop():
                return True
        return False

    def send(self, vx: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        self.cmd.publish(msg)

    def set_posture(self, stance: float, step: float, posture_control: bool = False, timeout_sim: float = 12.0) -> bool:
        """Command body/step height and posture control at rest; wait until locomotion_node
        reports them (the step is clamped to what the legs reach at that body height)."""
        def send():
            self.send(0.0)
            self.stance_cmd.publish(Float64(data=float(stance)))
            self.step_cmd.publish(Float64(data=float(step)))
            self.posture_cmd.publish(Bool(data=bool(posture_control)))

        def reached():
            loco = self.loco
            if loco is None or not hasattr(loco, "stance_height_m"):
                return False
            if hasattr(loco, "posture_control") and bool(loco.posture_control) != bool(posture_control):
                return False
            return abs(loco.stance_height_m - stance) < 5e-4 and abs(loco.step_height_m - step) < 5e-4
        return self.spin_sim(timeout_sim, every=send, stop=reached)

    def set_gait(self, gait: str, timeout: float = 10.0) -> bool:
        if not self.set_mode.wait_for_service(timeout_sec=timeout):
            return False
        future = self.set_mode.call_async(SetGaitMode.Request(mode=f"MANUAL:{gait}"))
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or not future.result().accepted:
            return False
        # locomotion_node applies it on its next tick; at rest the switch completes at once
        return self.spin_sim(5.0, every=lambda: self.send(0.0),
                             stop=lambda: self.loco is not None and self.loco.active_gait == gait
                             and self.loco.transition_state != "blending")


def teleport(world: str, model: str, x: float, y: float, z: float, yaw: float,
             quat: tuple | None = None) -> tuple[bool, str]:
    """Move the whole model (gz UserCommands; honours GZ_PARTITION). quat overrides yaw."""
    qx, qy, qz, qw = quat if quat is not None else (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return scn.gz_set_pose(world, model, x, y, z, qx, qy, qz, qw)


def start_bag(path: str):
    return subprocess.Popen(["ros2", "bag", "record", "--use-sim-time", "-o", path] + BAG_TOPICS,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)


def stop_bag(process) -> None:
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=15)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def wait_for_description(node: Node, timeout: float = 120.0) -> str | None:
    holder = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    sub = node.create_subscription(String, "/robot_description", lambda m: holder.setdefault("xml", m.data), qos)
    end = time.monotonic() + timeout
    while "xml" not in holder and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return holder.get("xml")


def save_snapshots(path: str, snaps: list[dict]) -> int:
    keep = [s for s in snaps if s["rgb"] is not None or s["depth"] is not None]
    if not keep:
        return 0
    arrays = {"t": np.array([s["t"] for s in keep]), "pos": np.array([s["pos"] for s in keep]),
              "quat": np.array([s["quat"] for s in keep])}
    if all(s["rgb"] is not None for s in keep):
        arrays["rgb"] = np.stack([s["rgb"] for s in keep])
    if all(s["depth"] is not None for s in keep):
        arrays["depth"] = np.stack([s["depth"] for s in keep])
    np.savez_compressed(path, **arrays)
    return len(keep)


class Runner:
    """Shared state of one plan run + the per-trial procedure."""

    def __init__(self, node: TrialRecorder, args, plan: dict, model: WholeBodyModel, xml: str, meta: dict,
                 nominal: tuple, cycles: dict, stand_z: float, log) -> None:
        self.node, self.args, self.plan, self.model, self.meta = node, args, plan, model, meta
        self.nominal, self.cycles, self.stand_z, self.log = nominal, cycles, stand_z, log
        self.foot_radius = foot_radius_from_urdf(xml)
        self.effort_limit = meta["effort_limit_nm"]
        self.velocity_limit = meta["velocity_limit_rad_s"]
        self.rng = np.random.default_rng(args.seed)
        self.csv_path = os.path.join(args.out, "e2_trials.csv")
        self.fields: list[str] | None = None
        self.results: list[dict] = []
        self.done_ids: set[str] = set()
        self.skipped = 0
        self.slow_in_row = 0
        if args.resume and os.path.exists(self.csv_path):
            with open(self.csv_path, encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                self.fields = list(reader.fieldnames or [])
                self.results = list(reader)
            self.done_ids = {r["trial"] for r in self.results}
            log(f"resume: {len(self.done_ids)} trial(s) already in {self.csv_path}")
        elif os.path.exists(self.csv_path):
            os.replace(self.csv_path, self.csv_path + ".old")

    def park(self) -> None:
        park = self.plan.get("park", {"x": self.plan["start"]["x"] - 2.5, "y": self.plan["start"]["y"] + 2.0})
        for _ in range(3):
            ok, _ = teleport(self.plan["world"], self.plan["model"], park["x"], park["y"], self.stand_z + 0.01, 0.0)
            if ok:
                break
        self.node.spin_sim(1.0, every=lambda: self.node.send(0.0))

    def prune(self, valid_condition, distance: float) -> None:
        """Resume: drop rows the current plan would not produce (condition removed from the
        plan, other course length) so the summary never mixes plan versions. The csv as it
        was is kept as e2_trials.pruned_<time>.csv."""
        def same_distance(row) -> bool:
            try:
                return abs(float(row.get("distance_m")) - distance) < 1e-6
            except (TypeError, ValueError):
                return False
        keep = [r for r in self.results if valid_condition(r["condition"]) and same_distance(r)]
        if len(keep) == len(self.results):
            return
        copy = self.csv_path.replace(".csv", time.strftime(".pruned_%Y%m%d_%H%M%S.csv"))
        os.replace(self.csv_path, copy)
        self.log(f"resume: dropped {len(self.results) - len(keep)} of {len(self.results)} trial(s) "
                 f"(condition not in the plan or distance != {distance} m); previous csv -> {copy}")
        self.results = keep
        self.done_ids = {r["trial"] for r in keep}
        with open(self.csv_path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=self.fields, extrasaction="ignore", restval="")
            writer.writeheader()
            writer.writerows(keep)

    # -- terrain bookkeeping ------------------------------------------------------
    # 25-26 Sep 2026 (E3/E4 overnight): gz-transport replies were lost ("NodeShared::
    # RecvSrvRequest() error sending response: Host unreachable" after the network interface
    # went down), so `gz service` reported timeouts although Gazebo had executed the
    # request. The runner then skipped a trial whose ramp did exist, or believed a removal
    # had failed, and later trials stood on two overlapping pieces (wrong slope, robot
    # wedged, RTF collapsed to ~0.03). Fixes: GZ_IP=127.0.0.1 (sim_guard.sh), and the
    # service reply is no longer trusted - the world content is read back from
    # /world/<w>/pose/info before and after every trial.
    def _terrain_in_world(self) -> list[str] | None:
        names = scn.gz_model_names(self.plan["world"])
        return None if names is None else sorted(n for n in names if n.startswith(TERRAIN_PREFIXES))

    def clear_terrain(self, context: str) -> None:
        left: list[str] | None = None
        for attempt in range(4):
            left = self._terrain_in_world()
            if left == []:
                return
            if left is None:
                self.log(f"{context}: /world/{self.plan['world']}/pose/info unreadable (attempt {attempt + 1})")
            else:
                if attempt:
                    self.log(f"{context}: removing leftover terrain {left} (attempt {attempt + 1})")
                for name in left:
                    scn.gz_remove(self.plan["world"], name)
            self.node.spin_sim(1.0, every=lambda: self.node.send(0.0))
        raise RunAbort(f"{context}: terrain {left if left is not None else '(world state unreadable)'} "
                       "cannot be removed - stopping so no trial runs on the wrong terrain")

    def create_terrain(self, trial_id: str, name: str, sdf: str) -> bool:
        """Create the piece and confirm it is the only terrain in the world."""
        for attempt in range(2):
            ok, detail = scn.gz_create(self.plan["world"], sdf)
            for _ in range(4):
                self.node.spin_sim(0.5, every=lambda: self.node.send(0.0))
                present = self._terrain_in_world()
                if present == [name]:
                    if not ok:
                        self.log(f"{trial_id}: create reply lost ({detail[:80]}) but the piece is in the world - continuing")
                    return True
                if present and name in present:
                    raise RunAbort(f"{trial_id}: unexpected terrain in the world: {present}")
            self.log(f"{trial_id}: terrain piece not in the world after create (attempt {attempt + 1}: {detail[:120]})")
            self.clear_terrain(trial_id)   # a late create must not survive into the next attempt
        return False

    def trial(self, cond: dict, trial_id: str, rep: int, extra: dict | None = None) -> dict | None:
        row = self._trial(cond, trial_id, rep, extra)
        if row is None:
            self.skipped += 1
        return row

    def _trial(self, cond: dict, trial_id: str, rep: int, extra: dict | None = None) -> dict | None:
        node, plan, log = self.node, self.plan, self.log
        gait, speed = cond["gait"], cond["speed"]
        stance, step = cond.get("stance", self.nominal[0]), cond.get("step", self.nominal[1])
        posture_control = bool(cond.get("posture_control", False))
        scenario = cond.get("scenario")
        distance = self.args.distance or (0.3 if self.args.check else 0.5 if self.args.quick else float(plan["distance_m"]))
        # offsets are drawn for every trial (also skipped ones) so a resumed run reuses them
        dx, dy, dyaw_u = self.rng.uniform(-1.0, 1.0, size=3)
        # 1. stop, posture, gait (at rest the switch completes at once)
        node.spin_sim(3.0, every=lambda: node.send(0.0))
        if not node.set_posture(stance, step, posture_control):
            log(f"{trial_id}: SKIP - posture body {stance * 1000:.0f} / step {step * 1000:.0f} mm / "
                f"posture control {posture_control} not confirmed by locomotion_node (rebuilt?)")
            return None
        if not node.set_gait(gait):
            log(f"{trial_id}: SKIP - set_gait_mode MANUAL:{gait} failed or never applied")
            return None
        # 2. terrain piece (park first: never create / delete under the robot)
        start = plan["start"]
        piece = scn.terrain(scenario, (start["x"], start["y"]), f"scn_{trial_id}")
        spawned = False
        if piece["sdf"] is not None:
            self.park()
            self.clear_terrain(trial_id)
            spawned = True   # from here on the piece may exist even if the create reply is lost
            if not self.create_terrain(trial_id, f"scn_{trial_id}", piece["sdf"]):
                log(f"{trial_id}: SKIP - could not create the terrain piece")
                self.park()
                self.clear_terrain(trial_id)
                return None
            node.spin_sim(0.5, every=lambda: node.send(0.0))
        try:
            return self._walk(cond, trial_id, rep, gait, speed, stance, step, posture_control, scenario, piece,
                              distance, (dx, dy, dyaw_u), extra)
        finally:
            if spawned:
                self.park()
                ok, detail = scn.gz_remove(plan["world"], f"scn_{trial_id}")
                if not ok:
                    log(f"{trial_id}: remove reply: {detail[:120]} - checking the world")
                self.clear_terrain(trial_id)   # raises RunAbort if the piece stays

    def _walk(self, cond, trial_id, rep, gait, speed, stance, step, posture_control, scenario, piece,
              distance, offsets, extra) -> dict | None:
        node, plan, log = self.node, self.plan, self.log
        jitter, start, fall_cfg = plan["jitter"], plan["start"], plan["fall"]
        dx, dy, dyaw_u = offsets
        x0 = start["x"] + dx * jitter["xy_m"]
        y0 = start["y"] + dy * jitter["xy_m"]
        yaw0 = math.radians(start.get("yaw_deg", 0.0) + dyaw_u * jitter["yaw_deg"])
        base_height = self.stand_z + (stance - self.nominal[0])        # base_link above the feet' ground
        px, py, pz, qx, qy, qz, qw = scn.start_pose(piece["ground_point"], piece["slope_deg"],
                                                   base_height + 0.005, x0, y0, yaw0)
        # 3. teleport onto the start
        ok, detail = False, ""
        for attempt in range(3):  # the gz service call timed out once in 50 trials (E2 25 Sep 2026)
            ok, detail = teleport(plan["world"], plan["model"], px, py, pz, yaw0, (qx, qy, qz, qw))
            if ok:
                break
            log(f"{trial_id}: teleport attempt {attempt + 1} failed ({detail}), retrying")
            node.spin_sim(1.0, every=lambda: node.send(0.0))
        if not ok:
            log(f"{trial_id}: SKIP - teleport failed: {detail}")
            return None
        node.spin_sim(float(plan.get("settle_s", 2.0)), every=lambda: node.send(0.0))
        # start check: base_link height above the intended surface (along its normal). A robot
        # standing on a leftover / missing piece or wedged after a bad teleport is off by far
        # more than the ~5 mm of settling.
        a = math.radians(piece["slope_deg"])
        gx, gy, gz = piece["ground_point"]
        _, p_now, _ = node.pose
        start_height = -math.sin(a) * (p_now[0] - gx) + math.cos(a) * (p_now[2] - gz)
        if abs(start_height - base_height) > START_HEIGHT_TOL_M:
            log(f"{trial_id}: SKIP - start check: base {start_height * 1000:.0f} mm above the {piece['describe']} "
                f"surface, expected {base_height * 1000:.0f} mm (terrain not as planned / bad teleport)")
            return None
        # 4. walk
        loco = node.loco
        cadence = loco.cadence_scale if loco is not None and getattr(loco, "cadence_scale", 0) > 0 else 1.0
        spec = TrialSpec(gait, speed, distance_m=distance, time_factor=float(plan["time_factor"]),
                         warmup_s=float(plan["warmup_cycles"]) * self.cycles[gait] * cadence,
                         tilt_deg=float(fall_cfg["tilt_deg"]), min_z_m=float(fall_cfg["min_z_m"]),
                         slope_deg=piece["slope_deg"], ground_point=tuple(piece["ground_point"]),
                         measure_sink=bool(scenario and scenario.get("type") == "patch"))
        node.rows, node.snapshots = [], []
        node.last_snapshot_t = -math.inf
        node.torque_messages = node.contact_messages = node.imu_messages = 0
        bag = start_bag(os.path.join(self.args.out, "bags", trial_id)) if self.args.bag else None
        node.recording = True
        t_command = node.now()
        origin = {}
        along = np.array((math.cos(math.radians(spec.slope_deg)), math.sin(math.radians(spec.slope_deg))))

        def done() -> bool:
            if node.pose is None:
                return False
            t, pos, quat = node.pose
            if fall_reason(pos, quat, spec.tilt_deg, spec.min_z_m, spec.slope_deg, spec.ground_point):
                return True
            if t < t_command + spec.warmup_s:
                return False
            if "p" not in origin:
                yaw = quat_to_rpy(quat)[2]
                origin.update(p=np.array(pos), h=np.array((math.cos(yaw), math.sin(yaw))))
            d = np.array(pos) - origin["p"]
            progress = along[0] * float(d[:2] @ origin["h"]) + along[1] * float(d[2])
            return progress >= spec.distance_m + 0.01

        sim_budget = spec.warmup_s + spec.time_limit_s + 1.0
        wall0, sim0 = time.monotonic(), node.now()
        finished = node.spin_sim(sim_budget, every=lambda: node.send(speed), stop=done,
                                 wall_limit=30.0 + sim_budget / MIN_WALL_RTF)
        sim_elapsed, wall_elapsed = node.now() - sim0, time.monotonic() - wall0
        rtf = sim_elapsed / max(wall_elapsed, 1e-3)
        node.recording = False
        for _ in range(5):
            node.send(0.0)
        stop_bag(bag)
        if not finished and sim_elapsed < sim_budget - 0.5:
            self.slow_in_row += 1
            log(f"{trial_id}: SKIP - simulation too slow: {sim_elapsed:.0f} s sim in {wall_elapsed:.0f} s "
                f"(RTF {rtf:.2f} < {MIN_WALL_RTF})")
            if self.slow_in_row >= 3:
                raise RunAbort("simulation too slow in 3 trials in a row")
            return None
        self.slow_in_row = 0
        # 5. metrics
        if not node.rows:
            log(f"{trial_id}: no samples recorded")
            return None
        samples = TrialSamples.from_rows(node.rows)
        if not self.args.no_samples:
            os.makedirs(os.path.join(self.args.out, "samples"), exist_ok=True)
            samples.save(os.path.join(self.args.out, "samples", trial_id + ".npz"))
        n_snap = 0
        if node.snapshots:
            os.makedirs(os.path.join(self.args.out, "snapshots"), exist_ok=True)
            n_snap = save_snapshots(os.path.join(self.args.out, "snapshots", trial_id + ".npz"), node.snapshots)
        scenario = scenario or {"type": "flat"}
        row = {key: "" for key in ROW_KEYS}
        row.update({"trial": trial_id, "condition": cond["name"], "group": cond.get("group", ""),
                    "speed_label": cond.get("label", ""), "rep": rep,
                    "t_command": t_command, "warmup_s": spec.warmup_s, "distance_m": spec.distance_m,
                    "stance_m": stance, "step_m": step, "cadence_scale": cadence,
                    "posture_control": posture_control, "scenario": json.dumps(scenario, sort_keys=True),
                    "surface_mu": scenario.get("mu", ""), "soft": scenario.get("soft", ""),
                    "slope_deg": piece["slope_deg"], "obstacle_h_m": scenario.get("height_m", ""),
                    "ground_x": piece["ground_point"][0], "ground_y": piece["ground_point"][1],
                    "ground_z": piece["ground_point"][2],
                    "start_x": x0, "start_y": y0, "start_yaw_deg": math.degrees(yaw0),
                    "torque_msgs": node.torque_messages, "contact_msgs": node.contact_messages,
                    "imu_msgs": node.imu_messages, "snapshots": n_snap,
                    "start_height_mm": start_height * 1000.0, "rtf": rtf})
        row.update(extra or {})
        row.update(evaluate(samples, spec, self.model, t_command=t_command, foot_radius=self.foot_radius,
                            effort_limit=self.effort_limit, velocity_limit=self.velocity_limit))
        self._write(row)
        log(f"{trial_id}: {'OK  ' if row['success'] else 'FAIL'} [{piece['describe']}] "
            f"v {row.get('speed_m_s', math.nan) * 1000:6.1f} mm/s  "
            f"SSMmin {row.get('ssm_min_m', math.nan) * 1000:6.1f} mm  "
            f"slip {row.get('slip_mean_m', math.nan) * 1000:5.1f} mm  CoT {row.get('cot', math.nan):6.2f}  "
            f"pitch {row.get('pitch_mean_deg', math.nan):5.1f} deg  "
            f"{('fall:' + row['fall_reason']) if row.get('fell') else ''}"
            + ("" if node.torque_messages else "  [no /joint_wrench data -> CoT NaN]")
            + ("" if node.contact_messages else "  [no contact data]"))
        node.spin_sim(1.0, every=lambda: node.send(0.0))
        return row

    def _write(self, row: dict) -> None:
        self.results.append(row)
        if self.fields is None or any(key not in self.fields for key in row):
            # first row, or a resumed csv written by an older runner: (re)write with all columns
            self.fields = (self.fields or []) + [key for key in row if key not in (self.fields or [])]
            with open(self.csv_path, "w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=self.fields, extrasaction="ignore", restval="")
                writer.writeheader()
                writer.writerows(self.results)
            return
        with open(self.csv_path, "a", newline="", encoding="utf-8") as handle:
            csv.DictWriter(handle, fieldnames=self.fields, extrasaction="ignore").writerow(row)

    # -- plan kinds -------------------------------------------------------------
    def run_conditions(self, conds: list[dict], n: int) -> None:
        for rep in range(1, n + 1):
            for cond in conds:
                trial_id = f"{cond['name']}_{rep:02d}"
                if trial_id in self.done_ids:
                    self.rng.uniform(size=3)
                    continue
                self.trial(cond, trial_id, rep)

    def run_ladders(self, groups: list[dict], n: int) -> None:
        for group in groups:
            need = min(group["min_success"], n)
            early_fail = n - need + 1
            for height in group["heights"]:
                name = f"{group['name']}_h{height * 1000:03.0f}"
                cond = dict(group, name=name, group=group["name"],
                            scenario={"type": "obstacle", "height_m": height})
                previous = [r for r in self.results if r["condition"] == name]
                wins = sum(str(r["success"]).lower() in ("true", "1") for r in previous)
                losses = len(previous) - wins
                rep = len(previous)
                while rep < n and losses < early_fail:
                    rep += 1
                    row = self.trial(cond, f"{name}_{rep:02d}", rep, extra={"group": group["name"]})
                    if row is None:
                        losses += 1  # a skipped trial counts as a failure for the stopping rule
                        continue
                    if row["success"]:
                        wins += 1
                    else:
                        losses += 1
                if wins < need:
                    self.log(f"{group['name']}: ladder stops at {height * 1000:.0f} mm ({wins}/{rep})")
                    break


def run_calibration(node: TrialRecorder, plan: dict, out: str, log) -> int:
    """E3 calibration: friction stick/slide on tilted mats, drop test on rigid / soft mats."""
    world, area = plan["world"], plan["area"]
    ax, ay = float(area["x"]), float(area["y"])
    margin = float(plan.get("margin_deg", 4.0))
    results = {"friction": [], "drop": []}

    def settle(seconds):
        node.spin_sim(seconds)

    def slide_test(mu: float, angle_deg: float) -> dict:
        a = math.radians(angle_deg)
        mat = scn.box_model_sdf("calib_mat", (1.0, 0.6, 0.05), (ax, ay, 0.3, 0, -a, 0), mu=mu,
                                colour=scn._colour(mu))
        n = np.array((-math.sin(a), 0.0, math.cos(a)))
        centre = np.array((ax, ay, 0.3)) + n * (0.025 + 0.05 + 0.002)
        box = scn.box_model_sdf("calib_box", (0.1, 0.1, 0.1), (*centre, 0, -a, 0), mu=1.0,
                                colour=(0.9, 0.1, 0.1), static=False, mass=1.0)
        entry = {"mu": mu, "angle_deg": angle_deg, "expected": "slide" if angle_deg > math.degrees(math.atan(mu)) else "stick"}
        try:
            if not scn.gz_create(world, mat)[0] or not scn.gz_create(world, box)[0]:
                entry["result"] = "spawn failed"
                return entry
            settle(0.5)
            p0 = scn.gz_model_position(world, "calib_box")
            settle(3.0)
            p1 = scn.gz_model_position(world, "calib_box")
            if p0 is None or p1 is None:
                entry["result"] = "pose unavailable"
                return entry
            moved = float(np.linalg.norm(np.array(p1) - np.array(p0)))
            entry.update(moved_m=moved, result="slide" if moved > 0.02 else "stick")
            entry["ok"] = entry["result"] == entry["expected"]
            return entry
        finally:
            scn.gz_remove(world, "calib_box")
            scn.gz_remove(world, "calib_mat")
            settle(0.3)

    for mu in plan["mu_values"]:
        critical = math.degrees(math.atan(mu))
        for angle in (critical - margin, critical + margin):
            entry = slide_test(float(mu), max(0.5, angle))
            results["friction"].append(entry)
            log(f"friction mu={mu:g} at {angle:5.1f} deg (critical {critical:4.1f}): {entry.get('result')} "
                f"(expected {entry['expected']}, moved {entry.get('moved_m', math.nan) * 1000:.0f} mm)")

    for soft in (False, True):
        kp = float(plan["soft"]["kp"]) if soft else None
        kd = float(plan["soft"]["kd"]) if soft else None
        mat = scn.box_model_sdf("calib_mat", (0.6, 0.6, 0.02), (ax, ay, 0.01, 0, 0, 0), mu=0.8, kp=kp, kd=kd)
        box = scn.box_model_sdf("calib_box", (0.1, 0.1, 0.1), (ax, ay, 0.02 + 0.05 + 0.10, 0, 0, 0),
                                static=False, mass=1.0)
        entry = {"soft": soft, "kp": kp, "kd": kd}
        try:
            if scn.gz_create(world, mat)[0] and scn.gz_create(world, box)[0]:
                settle(2.5)
                p = scn.gz_model_position(world, "calib_box")
                if p is not None:
                    entry["penetration_mm"] = (0.02 + 0.05 - p[2]) * 1000
            entry.setdefault("penetration_mm", float("nan"))
        finally:
            scn.gz_remove(world, "calib_box")
            scn.gz_remove(world, "calib_mat")
            settle(0.3)
        results["drop"].append(entry)
        log(f"drop test {'soft' if soft else 'rigid'} mat: resting penetration {entry['penetration_mm']:.2f} mm "
            f"(expected ~{9.80665 / kp * 1000:.1f} mm if kp is honoured)" if soft else
            f"drop test rigid mat: resting penetration {entry['penetration_mm']:.2f} mm")

    lines = ["# E3 calibration - friction and soft-floor checks", "",
             "| mu | angle (°) | critical atan(mu) (°) | expected | result | moved (mm) |", "|---:|---:|---:|---|---|---:|"]
    for e in results["friction"]:
        lines.append(f"| {e['mu']:g} | {e['angle_deg']:.1f} | {math.degrees(math.atan(e['mu'])):.1f} | {e['expected']} | "
                     f"{e.get('result', '?')} | {e.get('moved_m', math.nan) * 1000:.0f} |")
    lines += ["", "| mat | kp (N/m) | kd (N s/m) | resting penetration (mm) |", "|---|---:|---:|---:|"]
    for e in results["drop"]:
        lines.append(f"| {'soft' if e['soft'] else 'rigid'} | {e['kp'] or '-'} | {e['kd'] or '-'} | {e['penetration_mm']:.2f} |")
    friction_ok = all(e.get("ok") for e in results["friction"])
    soft_rigid = [e["penetration_mm"] for e in results["drop"]]
    lines += ["", f"Friction model as specified: {'YES' if friction_ok else 'NO - see rows'}.",
              "Soft floor honoured: " + ("YES" if len(soft_rigid) == 2 and soft_rigid[1] - soft_rigid[0] > 1.0
                                          else "NO (soft and rigid mats behave alike - report E3 'soft' as rigid-contact limitation)")]
    with open(os.path.join(out, "e3_calib.md"), "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")
    with open(os.path.join(out, "e3_calib.json"), "w", encoding="utf-8") as handle:
        json.dump(results, handle, indent=2)
    print("\n".join(lines), flush=True)
    return 0 if results["friction"] else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="results/e2")
    parser.add_argument("--config", help="plan yaml (default: share/hexapod_evaluation/config/e2.yaml)")
    parser.add_argument("--gait-config", help="gait.yaml for cycle times (default: hexapod_locomotion's)")
    parser.add_argument("--n", type=int, help="trials per condition (default from the plan)")
    parser.add_argument("--distance", type=float, help="course length m (default from the plan)")
    parser.add_argument("--gaits", nargs="*", help="subset of gaits")
    parser.add_argument("--seed", type=int, default=2)
    parser.add_argument("--quick", action="store_true", help="2 trials x 0.5 m per condition")
    parser.add_argument("--check", action="store_true",
                        help="pipeline check: 1 trial x 0.3 m of the first and last condition (or first ladder step)")
    parser.add_argument("--resume", action="store_true", help="skip trials already in e2_trials.csv")
    parser.add_argument("--redo", metavar="REGEX",
                        help="with --resume: run the conditions matching REGEX again (their rows are dropped, "
                             "the csv before is kept as e2_trials.pruned_<time>.csv), e.g. '_pc$'")
    parser.add_argument("--bag", action="store_true", help="rosbag2 per trial (bags/<trial>)")
    parser.add_argument("--no-samples", action="store_true", help="do not keep samples/<trial>.npz")
    args = parser.parse_args(argv)

    plan = load_plan(args.config)
    cycles = gait_cycle_times(args.gait_config)
    loco_params = gait_params(args.gait_config)
    nominal = (float(loco_params.get("stance_height_m", 0.10)), float(loco_params.get("step_height_m", 0.02)))
    max_stride = float(loco_params.get("max_stride_m", 0.05))
    n = 1 if args.check else args.n or (2 if args.quick else int(plan["n"]))
    os.makedirs(args.out, exist_ok=True)
    log = lambda text: (print(text, flush=True))  # noqa: E731

    rclpy.init()
    boot = Node("e2_runner_boot", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    xml = wait_for_description(boot)
    boot.destroy_node()
    if xml is None:
        print("no /robot_description - is sim.launch.py running?", file=sys.stderr)
        rclpy.try_shutdown()
        return 1
    model = WholeBodyModel.from_urdf(xml)
    with open(os.path.join(args.out, "e2_robot.urdf"), "w", encoding="utf-8") as handle:
        handle.write(xml)  # the exact model the trials ran with (e2_reevaluate uses it)
    axes = {j.name: j.axis for j in model.joints if j.name in JOINTS}
    import xml.etree.ElementTree as ET
    limits = {e.attrib["name"]: e.find("limit") for e in ET.fromstring(xml).findall("joint") if e.attrib["name"] in JOINTS}
    effort_limit = float(limits[JOINTS[1]].attrib.get("effort", "3.29"))
    velocity_limit = float(limits[JOINTS[1]].attrib.get("velocity", "3.65"))

    node = TrialRecorder(model, axes, record_camera=bool(plan.get("record_camera", False)))
    if plan.get("calibration"):
        log("waiting for the simulation clock ...")
        node.spin_sim(600.0, stop=lambda: node.pose is not None, wall_limit=180.0)
        status = run_calibration(node, plan, args.out, log)
        node.destroy_node()
        rclpy.try_shutdown()
        return status

    if "obstacles" in plan:
        groups = ladder_groups(plan, args.gaits, cycles, max_stride)
        conds = []
        if args.check:
            groups = [dict(g, heights=g["heights"][:1]) for g in groups[:1] + groups[-1:]]
    else:
        groups = []
        conds = conditions(plan, args.gaits, cycles, max_stride)
        if args.check:
            conds = conds[:1] + (conds[-1:] if len(conds) > 1 else [])
    meta = {
        "generated": time.strftime("%Y-%m-%d %H:%M:%S"), "seed": args.seed, "n": n,
        "distance_m": args.distance or (0.3 if args.check else 0.5 if args.quick else float(plan["distance_m"])),
        "plan": plan, "cycle_times_s": cycles, "conditions": conds or groups, "mass_kg": model.total_mass,
        "foot_radius_m": foot_radius_from_urdf(xml), "effort_limit_nm": effort_limit,
        "velocity_limit_rad_s": velocity_limit, "gz_partition": os.environ.get("GZ_PARTITION", ""),
        "mode": "check" if args.check else "quick" if args.quick else "full",
    }
    with open(os.path.join(args.out, "e2_meta.json"), "w", encoding="utf-8") as handle:
        json.dump(meta, handle, indent=2)

    # wait for the stack: locomotion ready + a ground-truth pose
    log("waiting for locomotion_node and ground-truth pose ...")
    node.spin_sim(600.0, every=lambda: node.send(0.0),
                  stop=lambda: node.pose is not None and node.loco is not None, wall_limit=180.0)
    if node.pose is None or node.loco is None:
        log("ABORT: no /tf odom->base_link or /locomotion_state (stack not running?)")
        rclpy.try_shutdown()
        return 1
    node.set_posture(*nominal)
    node.spin_sim(float(plan.get("settle_s", 2.0)) + 2.0, every=lambda: node.send(0.0))
    stand_z = node.pose[1][2]
    log(f"standing base_link z = {stand_z * 1000:.1f} mm; mass {model.total_mass:.3f} kg; "
        f"{len(conds) or len(groups)} {'conditions' if conds else 'ladder groups'} x {n} trials, {meta['distance_m']} m; "
        f"camera snapshots {'on' if plan.get('record_camera') else 'off'}")
    if plan.get("record_camera"):
        node.spin_sim(3.0, every=lambda: node.send(0.0))
        if node.rgb_msg is None and node.depth_msg is None:
            log("WARNING: record_camera is set but no /camera/image or /camera/depth_image arrives "
                "(launch with use_camera:=true) - trials run without snapshots")

    runner = Runner(node, args, plan, model, xml, meta, nominal, cycles, stand_z, log)
    if args.resume:
        names = {c["name"] for c in conds}
        prefixes = tuple(g["name"] + "_h" for g in groups)
        redo = re.compile(args.redo) if args.redo else None
        runner.prune(lambda c: (c in names or (bool(prefixes) and c.startswith(prefixes)))
                     and not (redo and redo.search(c)), meta["distance_m"])
    aborted = None
    try:
        if any((c.get("scenario") or {}).get("type", "flat") != "flat" for c in conds) or groups:
            runner.clear_terrain("start")   # also proves /world/<w>/pose/info is readable
            log("world state readable via pose/info; no terrain left over")
        if groups:
            runner.run_ladders(groups, n)
        else:
            runner.run_conditions(conds, n)
    except RunAbort as error:
        aborted = str(error)
        log(f"ABORT: {aborted}")
    finally:
        node.set_posture(*nominal)  # leave the robot in the nominal posture, posture control off
        summary_md = write_summary(runner.results, meta, args.out)
        print(summary_md, flush=True)
        node.destroy_node()
        rclpy.try_shutdown()
    if aborted:
        log(f"run aborted after {len(runner.results)} trial(s) - fix the cause, then start again with --resume")
        return 3
    if args.check:
        # a check passes only if every trial ran (pass/fail of the walk itself does not matter)
        log(f"CHECK {'PASSED' if runner.results and not runner.skipped else 'FAILED'}: "
            f"{len(runner.results)} trial(s) ran, {runner.skipped} skipped")
        return 0 if runner.results and not runner.skipped else 2
    return 0 if runner.results else 1


if __name__ == "__main__":
    sys.exit(main())
