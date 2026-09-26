"""ROS 2 node wrapping CadWalkingPlanner: the bottom of the architecture
diagram in the project plan (locomotion_controller).

Fixes carried over from the project plan's "known issues" (section 2):
this node reads elapsed time from a single source -- self.get_clock() --
for both the per-tick dt AND the command-staleness timeout. The old
control_manager_node/gait_planner_node pairing used time.monotonic() (wall
clock) for the deadman timeout but ROS time (sim clock, via
get_clock().now()) for dt; under Gazebo's real_time_factor < 1 those two
clocks disagree, so the deadman could fire early or late relative to what
the simulated robot experienced. With use_sim_time set consistently, both
now come from the same clock.
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool, Float64, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from hexapod_interfaces.msg import GaitSelection, LocomotionState
from hexapod_interfaces.srv import SetGaitMode
from hexapod_kinematics import LEGS, CadRobotKinematics, WholeBodyModel

from .planner import CadWalkingPlanner, WalkingSettings
from .trajectory import smoothstep5

SEGMENTS = ("coxa", "femur", "tibia")
GAIT_NAMES = {GaitSelection.TRIPOD: "tripod", GaitSelection.RIPPLE: "ripple", GaitSelection.WAVE: "wave"}


class LocomotionNode(Node):
    def __init__(self) -> None:
        super().__init__("locomotion_node")

        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("cycle_time_s", 0.95)
        self.declare_parameter("stance_height_m", 0.100)
        self.declare_parameter("step_height_m", 0.020)
        self.declare_parameter("max_stride_m", 0.050)
        self.declare_parameter("max_linear_x_m_s", 0.10)
        self.declare_parameter("max_linear_y_m_s", 0.10)
        self.declare_parameter("max_angular_z_rad_s", 0.40)
        self.declare_parameter("command_smoothing_s", 0.18)
        self.declare_parameter("command_timeout_s", 0.8)
        self.declare_parameter("standup_duration_s", 1.5)
        self.declare_parameter("gesture_duration_s", 3.0)
        self.declare_parameter("min_stance_height_m", 0.060)
        self.declare_parameter("max_stance_height_m", 0.135)
        self.declare_parameter("stance_height_rate_m_s", 0.020)
        # Step (swing apex) height, commanded on /step_height_command (Float64, m). Clamped by the
        # planner to what the legs reach at the current body height; the gait cycle stretches with
        # it (cadence.py) so joint speeds stay within the servo limit.
        self.declare_parameter("max_step_height_m", 0.050)
        self.declare_parameter("step_height_rate_m_s", 0.040)
        # Posture control (E4): integral loop on the estimated body tilt (/state/attitude from
        # hexapod_state_estimation) -> per-foot height offsets (planner.set_tilt_correction).
        # Off by default; /posture_control/enable (std_msgs/Bool) switches it at run time.
        self.declare_parameter("posture_control", False)
        self.declare_parameter("posture_ki", 1.5)          # 1/s: tilt error -> correction rate
        self.declare_parameter("posture_max_deg", 20.0)    # request limit (the reach limit is usually lower)
        self.declare_parameter("trajectory_horizon_s", 0.04)
        self.declare_parameter("blend_cycles", 1.0)
        self.declare_parameter("ik_solver", "analytic")  # analytic | dls (see planner.WalkingSettings)
        # Per-gait cycle times (0 = use cycle_time_s), see gait.yaml / WalkingSettings.cycle_time_by_gait
        for gait in ("tripod", "ripple", "wave"):
            self.declare_parameter(f"cycle_time_{gait}_s", 0.0)
        self.declare_parameter("initial_gait", "tripod")
        self.declare_parameter("trajectory_topic", "/hexapod_controller/joint_trajectory")

        self.update_rate = float(self.get_parameter("update_rate_hz").value)
        self.period = 1.0 / self.update_rate
        self.command_timeout = float(self.get_parameter("command_timeout_s").value)
        self.standup_duration = float(self.get_parameter("standup_duration_s").value)
        self.gesture_duration = float(self.get_parameter("gesture_duration_s").value)
        self.min_stance_height = float(self.get_parameter("min_stance_height_m").value)
        self.max_stance_height = float(self.get_parameter("max_stance_height_m").value)
        self.stance_height_rate = float(self.get_parameter("stance_height_rate_m_s").value)
        self.horizon = float(self.get_parameter("trajectory_horizon_s").value)
        self.step_height_rate = float(self.get_parameter("step_height_rate_m_s").value)
        self.posture_enabled = bool(self.get_parameter("posture_control").value)
        self.posture_ki = float(self.get_parameter("posture_ki").value)
        self.posture_max = math.radians(float(self.get_parameter("posture_max_deg").value))
        self.attitude = None          # (stamp s, roll, pitch) from /state/attitude
        self.posture_request = [0.0, 0.0]   # integrator state (roll, pitch) rad
        self.target_step_height = float(self.get_parameter("step_height_m").value)

        self.robot: CadRobotKinematics | None = None
        self.mass_model: WholeBodyModel | None = None
        self.planner: CadWalkingPlanner | None = None
        self.ready = False
        self.joint_names: list[str] = [
            f"{leg}_{segment}_joint" for leg in LEGS for segment in SEGMENTS
        ]
        self.have_joint_states = False

        self.command = np.zeros(3)
        self.last_command_time = None
        self.target_stance_height = float(self.get_parameter("stance_height_m").value)
        self.current_stance_height = self.target_stance_height

        self.gait_mode = "AUTO"
        self.manual_gait = str(self.get_parameter("initial_gait").value)
        self.auto_gait = self.manual_gait
        self.gait_selection_stamp = None

        self.gesture_name: str | None = None
        self.gesture_start_time = None
        self.gesture_start_positions: np.ndarray | None = None

        self.standup_start_time = None
        self.standup_start_positions: np.ndarray | None = None

        # robot_state_publisher publishes /robot_description once, latched
        # (transient_local). A default volatile subscription misses it if this
        # node starts even slightly later, so the node would never get ready.
        description_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            String, "/robot_description", self.on_robot_description, description_qos
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_command, 10)
        self.create_subscription(Float64, "/stance_height_command", self.on_stance_height, 10)
        self.create_subscription(Float64, "/step_height_command", self.on_step_height, 10)
        self.create_subscription(Vector3Stamped, "/state/attitude", self.on_attitude, 10)
        self.create_subscription(Bool, "/posture_control/enable", self.on_posture_enable, 10)
        self.create_subscription(String, "/gesture_command", self.on_gesture, 10)
        self.create_subscription(GaitSelection, "/gait_selection", self.on_gait_selection, 10)
        self.create_service(SetGaitMode, "/set_gait_mode", self.on_set_gait_mode)

        trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.trajectory_publisher = self.create_publisher(JointTrajectory, trajectory_topic, 10)
        self.state_publisher = self.create_publisher(LocomotionState, "/locomotion_state", 10)
        self.ready_publisher = self.create_publisher(Bool, "/locomotion/ready", 10)

        self.last_update_ros_time = self.get_clock().now()
        self.create_timer(self.period, self.update)
        self.get_logger().info("locomotion_node started; waiting for robot_description + joint_states")

    # -- setup -----------------------------------------------------------
    def on_robot_description(self, message: String) -> None:
        if self.robot is not None:
            return
        try:
            self.robot = CadRobotKinematics.from_urdf(message.data)
        except ValueError as error:
            self.get_logger().error(f"Failed to parse robot_description: {error}")
            return
        try:
            # link masses for LocomotionState.stability_margin_m (planned SSM)
            self.mass_model = WholeBodyModel.from_urdf(message.data)
        except (ValueError, KeyError, AttributeError) as error:
            self.mass_model = None
            self.get_logger().warning(f"No mass model ({error}); stability_margin_m will be NaN")
        self._try_become_ready()

    def on_joint_state(self, message: JointState) -> None:
        if not self.have_joint_states and set(self.joint_names).issubset(set(message.name)):
            self.have_joint_states = True
            self._try_become_ready()

    def _try_become_ready(self) -> None:
        if self.ready or self.robot is None or not self.have_joint_states:
            return
        settings = WalkingSettings(
            cycle_time=float(self.get_parameter("cycle_time_s").value),
            stance_height=self.target_stance_height,
            step_height=float(self.get_parameter("step_height_m").value),
            max_stride=float(self.get_parameter("max_stride_m").value),
            max_linear_x=float(self.get_parameter("max_linear_x_m_s").value),
            max_linear_y=float(self.get_parameter("max_linear_y_m_s").value),
            max_angular_z=float(self.get_parameter("max_angular_z_rad_s").value),
            smoothing_time=float(self.get_parameter("command_smoothing_s").value),
            blend_cycles=float(self.get_parameter("blend_cycles").value),
            ik_solver=str(self.get_parameter("ik_solver").value),
            max_step_height=float(self.get_parameter("max_step_height_m").value),
            min_stance_height=self.min_stance_height,
            max_stance_height=self.max_stance_height,
            cycle_time_by_gait={
                gait: float(self.get_parameter(f"cycle_time_{gait}_s").value)
                for gait in ("tripod", "ripple", "wave")
                if float(self.get_parameter(f"cycle_time_{gait}_s").value) > 0.0
            },
        )
        try:
            self.planner = CadWalkingPlanner(self.robot, settings, self.manual_gait, self.mass_model)
        except ValueError as error:
            self.get_logger().error(f"Planner init failed: {error}")
            return
        self.ready = True
        self.standup_start_time = self.get_clock().now()
        self.standup_start_positions = self.planner.neutral_positions.copy()
        self.get_logger().info(
            f"CAD gait ready (ik_solver={settings.ik_solver}, cycle times "
            + ", ".join(f"{g} {self.planner.cycle_time_of(g):.2f} s" for g in ("tripod", "ripple", "wave")) + ")")

    # -- command inputs ----------------------------------------------------
    def on_command(self, message: Twist) -> None:
        self.command = np.array((message.linear.x, message.linear.y, message.angular.z))
        self.last_command_time = self.get_clock().now()

    def on_stance_height(self, message: Float64) -> None:
        self.target_stance_height = float(
            np.clip(message.data, self.min_stance_height, self.max_stance_height)
        )

    def on_step_height(self, message: Float64) -> None:
        if math.isfinite(message.data):
            self.target_step_height = float(np.clip(
                message.data, 0.0, float(self.get_parameter("max_step_height_m").value)))

    def on_attitude(self, message: Vector3Stamped) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.attitude = (stamp, float(message.vector.x), float(message.vector.y))

    def on_posture_enable(self, message: Bool) -> None:
        self.posture_enabled = bool(message.data)

    def _update_posture(self, now_s: float, dt: float) -> None:
        """Integral posture loop. Enabled: the request integrates the measured tilt (so a
        constant slope ends up fully cancelled as far as the reach allows); anti-windup by
        re-reading what the planner actually applied. Disabled or no fresh attitude: the
        request decays to zero in ~0.5 s."""
        fresh = self.attitude is not None and now_s - self.attitude[0] < 0.2
        if self.posture_enabled and fresh:
            _, roll, pitch = self.attitude
            self.posture_request[0] += self.posture_ki * roll * dt
            self.posture_request[1] += self.posture_ki * pitch * dt
        else:
            decay = min(1.0, 2.0 * dt)
            self.posture_request[0] -= decay * self.posture_request[0]
            self.posture_request[1] -= decay * self.posture_request[1]
        for i in range(2):
            self.posture_request[i] = float(np.clip(self.posture_request[i], -self.posture_max, self.posture_max))
        applied = self.planner.set_tilt_correction(*self.posture_request)
        self.posture_request = [applied[0], applied[1]]

    def on_gesture(self, message: String) -> None:
        if not self.ready or self.gesture_name is not None:
            return
        name = message.data.strip().lower()
        if name not in ("bow", "wave", "sway"):
            self.get_logger().warning(f"Unknown gesture '{message.data}'")
            return
        self.gesture_name = name
        self.gesture_start_time = self.get_clock().now()
        self.gesture_start_positions = np.concatenate(
            [self.planner.angles[leg] for leg in LEGS]
        )

    def on_gait_selection(self, message: GaitSelection) -> None:
        self.auto_gait = GAIT_NAMES.get(message.gait, self.auto_gait)
        self.gait_selection_stamp = self.get_clock().now()

    def on_set_gait_mode(self, request: SetGaitMode.Request, response: SetGaitMode.Response):
        mode = request.mode.strip()
        if mode.upper() == "AUTO":
            self.gait_mode = "AUTO"
            response.accepted = True
            response.message = "switched to AUTO (gait_selector drives gait choice)"
        elif mode.upper().startswith("MANUAL:"):
            gait = mode.split(":", 1)[1].strip().lower()
            if gait not in ("tripod", "ripple", "wave"):
                response.accepted = False
                response.message = f"unknown gait '{gait}'"
                return response
            self.gait_mode = "MANUAL"
            self.manual_gait = gait
            response.accepted = True
            response.message = f"switched to MANUAL:{gait}"
        else:
            response.accepted = False
            response.message = "mode must be 'AUTO' or 'MANUAL:<gait>'"
        return response

    # -- main loop -----------------------------------------------------
    def _publish_trajectory(self, positions: np.ndarray) -> None:
        message = JointTrajectory()
        # Leave header.stamp zero: joint_trajectory_controller treats it as
        # "start now". A real stamp plus time_from_start=0 is rejected as
        # "ends in the past" by the time it arrives.
        message.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions.tolist()
        point.time_from_start = Duration(seconds=self.horizon).to_msg()
        message.points = [point]
        self.trajectory_publisher.publish(message)

    def _publish_state(self, transition_state: str) -> None:
        self.ready_publisher.publish(Bool(data=self.ready))
        if not self.ready:
            return
        message = LocomotionState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.active_gait = self.planner.active_gait
        message.transition_state = transition_state
        message.phase = self.planner.transition.phase % 1.0
        message.leg_in_swing = [bool(self.planner.leg_in_swing[leg]) for leg in LEGS]
        message.ik_residual_max_m = float(self.planner.last_ik_residual_max)
        # planned SSM (commanded pose, level body); NaN until the first walking/gesture tick
        message.stability_margin_m = float(self.planner.last_stability_margin)
        message.stance_height_m = float(self.planner.stance_height)
        message.step_height_m = float(self.planner.step_height)
        message.cadence_scale = float(self.planner.cadence_scale_of(self.planner.active_gait))
        message.posture_control = bool(self.posture_enabled)
        message.posture_roll_rad = float(self.planner.tilt_correction[0])
        message.posture_pitch_rad = float(self.planner.tilt_correction[1])
        message.estop = False
        self.state_publisher.publish(message)

    def update(self) -> None:
        if not self.ready:
            # (was published unconditionally before: every tick then carried two
            # LocomotionState messages, the first one always "walking")
            self._publish_state("standing")
            return

        now = self.get_clock().now()
        elapsed = (now - self.last_update_ros_time).nanoseconds * 1e-9
        self.last_update_ros_time = now
        if not math.isfinite(elapsed) or elapsed <= 0:
            elapsed = self.period
        dt = min(elapsed, 0.1)

        # -- standup ramp --
        standup_elapsed = (now - self.standup_start_time).nanoseconds * 1e-9
        if standup_elapsed < self.standup_duration:
            ratio = smoothstep5(max(0.0, min(1.0, standup_elapsed / self.standup_duration)))
            neutral = self.planner.neutral_positions
            self._publish_trajectory(
                self.standup_start_positions + ratio * (neutral - self.standup_start_positions)
            )
            self._publish_state("standing")
            return

        # -- gesture playback --
        if self.gesture_name is not None:
            elapsed_gesture = (now - self.gesture_start_time).nanoseconds * 1e-9
            progress = min(1.0, max(0.0, elapsed_gesture / self.gesture_duration))
            try:
                positions = self.planner.gesture_positions(
                    self.gesture_name, progress, self.gesture_start_positions
                )
            except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
                self.get_logger().error(f"Gesture rejected: {error}")
                self.gesture_name = None
                self._publish_state("walking")
                return
            self._publish_trajectory(positions)
            self._publish_state("gesture")
            if elapsed_gesture >= self.gesture_duration:
                self.gesture_name = None
                self.command[:] = 0.0
                self.last_command_time = None
            return

        # -- stance height ramp --
        height_error = self.target_stance_height - self.current_stance_height
        max_height_step = self.stance_height_rate * dt
        if abs(height_error) > 1e-9:
            step = math.copysign(min(abs(height_error), max_height_step), height_error)
            try:
                self.planner.set_stance_height(self.current_stance_height + step)
                self.current_stance_height += step
            except ValueError as error:
                self.target_stance_height = self.current_stance_height
                self.get_logger().error(f"Height command rejected: {error}")

        # -- step height ramp towards the command, limited to what the legs reach at the
        #    current body height (the command is kept, so raising the body later lets the
        #    step follow up again) --
        goal = min(self.target_step_height, self.planner.max_step_height())
        step_error = goal - self.planner.step_height
        if abs(step_error) > 1e-9:
            self.planner.set_step_height(
                self.planner.step_height + math.copysign(min(abs(step_error), self.step_height_rate * dt), step_error))

        # -- posture control (per-foot height offsets from the IMU attitude estimate) --
        self._update_posture(now.nanoseconds * 1e-9, dt)

        # -- gait mode (AUTO follows gait_selector, MANUAL is pinned) --
        desired_gait = self.manual_gait if self.gait_mode == "MANUAL" else self.auto_gait
        if desired_gait != self.planner.active_gait:
            self.planner.set_gait(desired_gait)

        # -- command deadman: same clock (now) used for dt above --
        command = self.command
        if (
            self.last_command_time is None
            or (now - self.last_command_time).nanoseconds * 1e-9 > self.command_timeout
        ):
            command = np.zeros(3)

        try:
            positions = self.planner.update(command, dt, lookahead=self.horizon)
        except (RuntimeError, ValueError, np.linalg.LinAlgError) as error:
            self.get_logger().error(f"Walking command rejected: {error}")
            self._publish_state("safe_stop")
            return
        self._publish_trajectory(positions)
        self._publish_state("blending" if self.planner.transitioning else "walking")


def main() -> None:
    rclpy.init()
    node = LocomotionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C / launch shutdown: rclpy's signal handler already shut the context down
    finally:
        node.destroy_node()
        rclpy.try_shutdown()  # plain shutdown() raises "rcl_shutdown already called" here


if __name__ == "__main__":
    main()
