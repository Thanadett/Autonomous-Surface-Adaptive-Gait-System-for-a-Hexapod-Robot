"""E1 part B - foot-tip accuracy of the whole chain in Gazebo, body fixed in the air.

Started by launch/e1_fixed_base.launch.py (robot spawned with
fixed_base:=true, only joint_state_broadcaster + hexapod_controller running,
no locomotion_node). For every point of the E1 walking grid all six legs
are sent to their own target at once:

    target (body frame) --IK--> q_cmd --JointTrajectory--> Gazebo
    after settling: q_meas (mean of /joint_states over the last `average_s`)
    foot-tip error   = |FK(q_meas) - target|      (what the chapter reports)
    tracking error   = |q_meas - q_cmd| per joint  (controller + gravity)
    IK residual      = |FK(q_cmd) - target|

FK(q_meas) equals the simulated foot position because Gazebo integrates the
same rigid URDF chain; the URDF chain itself was checked against the CAD
(0.08 mm, hexapod_description/doc/verification_log.txt).

Writes <out_dir>/e1_sim_samples.csv and e1_sim_summary.md/.json and exits,
which ends the launch.
"""
from __future__ import annotations

import csv
import json
import math
import os
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from hexapod_kinematics import LEGS, SEGMENTS, CadRobotKinematics

from .e1_targets import DEFAULT_STANCE_HEIGHT_M, nominal_foot, walking_grid
from .stats import summarize


def _duration(seconds: float) -> DurationMsg:
    whole = int(seconds)
    return DurationMsg(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


class E1SimNode(Node):
    def __init__(self) -> None:
        super().__init__("e1_sim")
        self.declare_parameter("out_dir", "results/e1")
        self.declare_parameter("solver", "analytic")  # analytic = locomotion_node default; or dls
        self.declare_parameter("move_s", 0.6)
        self.declare_parameter("settle_s", 1.2)
        self.declare_parameter("average_s", 0.2)
        self.declare_parameter("trajectory_topic", "/hexapod_controller/joint_trajectory")
        self.declare_parameter("controller", "hexapod_controller")
        self.declare_parameter("max_targets", 0)  # 0 = whole grid (for quick checks: e.g. 5)

        self.robot: CadRobotKinematics | None = None
        self.joint_names = [f"{leg}_{seg}_joint" for leg in LEGS for seg in SEGMENTS]
        self.samples: list[tuple[float, np.ndarray]] = []  # (sim time, 18 positions)
        description_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                     reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, "/robot_description", self.on_description, description_qos)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 50)
        self.publisher = self.create_publisher(
            JointTrajectory, str(self.get_parameter("trajectory_topic").value), 10)
        self.list_controllers = self.create_client(ListControllers, "/controller_manager/list_controllers")

    # -- inputs ---------------------------------------------------------
    def on_description(self, msg: String) -> None:
        if self.robot is None:
            self.robot = CadRobotKinematics.from_urdf(msg.data)

    def on_joint_state(self, msg: JointState) -> None:
        index = {name: i for i, name in enumerate(msg.name)}
        if not all(name in index for name in self.joint_names):
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q = np.array([msg.position[index[name]] for name in self.joint_names])
        self.samples.append((t, q))
        if len(self.samples) > 2000:
            del self.samples[:1000]

    # -- helpers --------------------------------------------------------
    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_until(self, predicate, timeout_wall: float) -> bool:
        end = time.monotonic() + timeout_wall
        while time.monotonic() < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        return predicate()

    def wait_sim(self, seconds: float) -> None:
        start = self.now_s()
        self.spin_until(lambda: self.now_s() - start >= seconds, timeout_wall=60.0 + 20.0 * seconds)

    def controller_active(self) -> bool:
        if not self.list_controllers.service_is_ready():
            return False
        future = self.list_controllers.call_async(ListControllers.Request())
        self.spin_until(future.done, 3.0)
        if not future.done() or future.result() is None:
            return False
        name = str(self.get_parameter("controller").value)
        return any(c.name == name and c.state == "active" for c in future.result().controller)

    def command(self, q: np.ndarray, move_s: float) -> None:
        msg = JointTrajectory()
        msg.joint_names = self.joint_names  # stamp 0 = start now (JTC)
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in q]
        point.velocities = [0.0] * len(q)
        point.time_from_start = _duration(move_s)
        msg.points = [point]
        self.publisher.publish(msg)

    def measured(self, window_s: float) -> np.ndarray | None:
        if not self.samples:
            return None
        latest = self.samples[-1][0]
        recent = [q for t, q in self.samples if t >= latest - window_s]
        return np.mean(recent, axis=0)

    # -- experiment -----------------------------------------------------
    def run(self) -> int:
        log = self.get_logger()
        if not self.spin_until(lambda: self.robot is not None and bool(self.samples), 120.0):
            log.error("no /robot_description or /joint_states - is the simulation running?")
            return 1
        if not self.spin_until(self.controller_active, 120.0):
            log.error("hexapod_controller never became active")
            return 1
        solver = str(self.get_parameter("solver").value)
        move_s = float(self.get_parameter("move_s").value)
        settle_s = float(self.get_parameter("settle_s").value)
        average_s = float(self.get_parameter("average_s").value)
        robot = self.robot
        grids = walking_grid(robot)
        count = len(grids[LEGS[0]])
        limit = int(self.get_parameter("max_targets").value)
        if limit > 0:
            count = min(count, limit)

        def ik(leg: str, target: np.ndarray, seed: np.ndarray):
            chain = robot.legs[leg]
            return chain.inverse(target, seed) if solver == "dls" else chain.inverse_analytic(target, seed)

        # start from the nominal stance so the first grid point is a normal move
        q_cmd = np.concatenate([
            ik(leg, nominal_foot(robot, leg, DEFAULT_STANCE_HEIGHT_M), np.zeros(3)).angles for leg in LEGS])
        self.command(q_cmd, 2.0)
        self.wait_sim(3.0)

        rows = []
        started = time.monotonic()
        for i in range(count):
            residuals = {}
            for li, leg in enumerate(LEGS):
                result = ik(leg, grids[leg][i], q_cmd[3 * li:3 * li + 3])
                q_cmd[3 * li:3 * li + 3] = result.angles
                residuals[leg] = (result.converged, result.error)
            self.command(q_cmd, move_s)
            self.wait_sim(move_s + settle_s)
            q_meas = self.measured(average_s)
            for li, leg in enumerate(LEGS):
                target = grids[leg][i]
                qm = q_meas[3 * li:3 * li + 3]
                qc = q_cmd[3 * li:3 * li + 3]
                actual = robot.legs[leg].forward(qm)
                err = actual - target
                rows.append({
                    "index": i, "leg": leg, "solver": solver,
                    "target_x_m": target[0], "target_y_m": target[1], "target_z_m": target[2],
                    "ik_converged": int(residuals[leg][0]), "ik_residual_m": residuals[leg][1],
                    "foot_error_m": float(np.linalg.norm(err)),
                    "foot_error_x_m": err[0], "foot_error_y_m": err[1], "foot_error_z_m": err[2],
                    "track_coxa_rad": qm[0] - qc[0], "track_femur_rad": qm[1] - qc[1], "track_tibia_rad": qm[2] - qc[2],
                })
            if i % 10 == 0:
                log.info(f"E1 target {i + 1}/{count}: foot error max this pose "
                         f"{max(r['foot_error_m'] for r in rows[-6:]) * 1000:.2f} mm")
        self.write(rows, solver, time.monotonic() - started)
        return 0

    def write(self, rows: list[dict], solver: str, wall_s: float) -> None:
        out_dir = str(self.get_parameter("out_dir").value)
        os.makedirs(out_dir, exist_ok=True)
        with open(os.path.join(out_dir, "e1_sim_samples.csv"), "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

        def mm(key, subset):
            s = summarize(r[key] for r in subset)
            return {"n": s.n, "mean": s.mean * 1e3, "sd": s.sd * 1e3, "p95": s.p95 * 1e3, "max": s.max * 1e3}

        def deg(key, subset):
            s = summarize(abs(r[key]) for r in subset)
            return {"mean": math.degrees(s.mean), "sd": math.degrees(s.sd), "max": math.degrees(s.max)}

        summary = {
            "all": mm("foot_error_m", rows),
            "per_leg": {leg: mm("foot_error_m", [r for r in rows if r["leg"] == leg]) for leg in LEGS},
            "per_axis_abs": {a: mm(f"foot_error_{a}_m", [dict(r, **{f"foot_error_{a}_m": abs(r[f"foot_error_{a}_m"])}) for r in rows]) for a in "xyz"},
            "tracking_abs_deg": {j: deg(f"track_{j}_rad", rows) for j in SEGMENTS},
            "ik_residual": mm("ik_residual_m", rows),
            "ik_failures": sum(1 for r in rows if not r["ik_converged"]),
        }
        params = {k: self.get_parameter(k).value for k in ("move_s", "settle_s", "average_s")}
        meta = {"generated": time.strftime("%Y-%m-%d %H:%M:%S"), "solver": solver, "wall_s": wall_s, **params}
        with open(os.path.join(out_dir, "e1_sim_summary.json"), "w", encoding="utf-8") as handle:
            json.dump({"meta": meta, "summary": summary}, handle, indent=2)

        a = summary["all"]
        lines = [
            "# E1 (Gazebo, body fixed in the air) - foot-tip error of IK + controller",
            "",
            f"- Generated {meta['generated']}; IK solver: {solver}; move {params['move_s']} s + settle {params['settle_s']} s, "
            f"joint positions averaged over the last {params['average_s']} s (sim time); wall time {wall_s:.0f} s",
            f"- Targets: E1 walking grid, {a['n'] // 6} per leg x 6 legs = {a['n']} (see hexapod_evaluation/e1_targets.py)",
            "- Foot-tip error = |FK(q_measured) - target|",
            "",
            f"**All legs: {a['mean']:.2f} ± {a['sd']:.2f} mm (mean ± SD), p95 {a['p95']:.2f} mm, max {a['max']:.2f} mm, n = {a['n']}**",
            "",
            "| Leg | n | Mean ± SD (mm) | p95 (mm) | Max (mm) |",
            "|---|---:|---:|---:|---:|",
        ]
        for leg, e in summary["per_leg"].items():
            lines.append(f"| {leg} | {e['n']} | {e['mean']:.2f} ± {e['sd']:.2f} | {e['p95']:.2f} | {e['max']:.2f} |")
        lines += ["", "| Axis (body frame) | Mean abs error ± SD (mm) | Max (mm) |", "|---|---:|---:|"]
        for axis, e in summary["per_axis_abs"].items():
            lines.append(f"| {axis} | {e['mean']:.2f} ± {e['sd']:.2f} | {e['max']:.2f} |")
        lines += ["", "| Joint | Mean abs(q_meas - q_cmd) ± SD (deg) | Max (deg) |", "|---|---:|---:|"]
        for joint, e in summary["tracking_abs_deg"].items():
            lines.append(f"| {joint} | {e['mean']:.3f} ± {e['sd']:.3f} | {e['max']:.3f} |")
        r = summary["ik_residual"]
        lines += ["", f"IK residual |FK(q_cmd) - target|: mean {r['mean']:.6f} mm, max {r['max']:.6f} mm; IK failures: {summary['ik_failures']}"]
        text = "\n".join(lines) + "\n"
        with open(os.path.join(out_dir, "e1_sim_summary.md"), "w", encoding="utf-8") as handle:
            handle.write(text)
        print(text, flush=True)


def main() -> None:
    rclpy.init()
    node = E1SimNode()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
