#!/usr/bin/env python3
"""Gazebo screenshots for the thesis (chapter 3 / 4.1.3-4.1.5), 1920 x 1080 PNG.

    bash scripts/capture_figures.sh            # starts the simulation, runs this, cleans up

A static camera model ("fig_cam", camera sensor, no visual) is spawned at a viewpoint
relative to the robot, one frame is bridged to ROS (/fig_cam/image) and written as PNG;
then the camera is removed. The robot is posed with the same tools as the experiments
(scenario.py terrain pieces, locomotion_node posture/gait/posture-control commands), so
the pictures show exactly the E3-E5 set-ups. Output: results/figures_gazebo/*.png.
"""
from __future__ import annotations

import argparse
import math
import os
import struct
import subprocess
import sys
import time
import zlib

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from hexapod_evaluation import scenario as scn
from hexapod_evaluation.e2_runner import JOINTS, TrialRecorder, teleport, wait_for_description
from hexapod_kinematics import WholeBodyModel

WORLD, MODEL = "flat_world", "hexapod"
START = (-5.45, -1.4)
PARK = (-8.0, 1.0)
NOMINAL = (0.100, 0.020)
RAISED = (0.135, 0.050)
CAM_TOPIC = "/fig_cam/image"


def write_png(path: str, rgb: np.ndarray) -> None:
    """Minimal PNG writer (no PIL/OpenCV dependency)."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def camera_sdf(name: str, pos, target, fov: float, width: int = 1920, height: int = 1080) -> str:
    dx, dy, dz = (t - p for t, p in zip(target, pos))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))   # +pitch about y tilts the view (+x) downwards
    return (f"<?xml version='1.0'?><sdf version='1.10'><model name='{name}'><static>true</static>"
            f"<pose>{pos[0]:.4f} {pos[1]:.4f} {pos[2]:.4f} 0 {pitch:.5f} {yaw:.5f}</pose>"
            "<link name='link'><sensor name='cam' type='camera'><always_on>1</always_on>"
            f"<update_rate>4</update_rate><topic>{CAM_TOPIC}</topic><camera>"
            f"<horizontal_fov>{fov:.4f}</horizontal_fov><image><width>{width}</width><height>{height}</height>"
            "<format>R8G8B8</format></image><clip><near>0.03</near><far>60</far></clip>"
            "<anti_aliasing>4</anti_aliasing></camera></sensor></link></model></sdf>")


class Capture:
    def __init__(self, node: TrialRecorder, out: str, log) -> None:
        self.node, self.out, self.log = node, out, log
        self.frames: list = []
        node.create_subscription(Image, CAM_TOPIC, self.frames.append, qos_profile_sensor_data)
        self.pieces: list[str] = []
        self.speed = 0.0
        self.shots = 0

    # -- world helpers --------------------------------------------------------
    def spin(self, seconds: float) -> None:
        self.node.spin_sim(seconds, every=lambda: self.node.send(self.speed))

    def add(self, sdf: str, name: str) -> None:
        scn.gz_create(WORLD, sdf)
        self.pieces.append(name)

    def clear(self) -> None:
        self.speed = 0.0
        teleport(WORLD, MODEL, PARK[0], PARK[1], 0.13, 0.0)
        self.spin(1.0)
        for name in self.pieces:
            scn.gz_remove(WORLD, name)
        self.pieces = []
        self.spin(1.0)

    def place(self, ground_point, slope_deg: float, stance: float, stand_z: float, x: float, y: float) -> None:
        base_h = stand_z + (stance - NOMINAL[0])
        px, py, pz, qx, qy, qz, qw = scn.start_pose(ground_point, slope_deg, base_h + 0.005, x, y, 0.0)
        teleport(WORLD, MODEL, px, py, pz, 0.0, (qx, qy, qz, qw))
        self.spin(2.0)

    def posture(self, stance_step, gait: str, posture_control: bool = False) -> None:
        self.speed = 0.0
        self.node.set_posture(stance_step[0], stance_step[1], posture_control)
        self.node.set_gait(gait)

    # -- camera ---------------------------------------------------------------
    def shot(self, name: str, offset, fov_deg: float = 45.0, look_offset=(0.0, 0.0, 0.0)) -> bool:
        """Camera at robot base + offset (world axes), looking at base + look_offset."""
        _, p, _ = self.node.pose
        target = (p[0] + look_offset[0], p[1] + look_offset[1], p[2] + look_offset[2])
        cam = (p[0] + offset[0], p[1] + offset[1], p[2] + offset[2])
        self.shots += 1
        cam_name = f"fig_cam_{self.shots}"   # unique: a camera that failed to disappear never answers for the next shot
        scn.gz_create(WORLD, camera_sdf(cam_name, cam, target, math.radians(fov_deg)))
        self.spin(0.3)
        self.frames.clear()
        t_end = time.monotonic() + 60.0
        # the first frames of a new sensor can be black / partly rendered: take the 4th
        while len(self.frames) < 4 and time.monotonic() < t_end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.node.send(self.speed)
        ok = len(self.frames) >= 4
        if ok:
            msg = self.frames[-1]
            img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.step)[:, : msg.width * 3]
            img = img.reshape(msg.height, msg.width, 3)
            if msg.encoding == "bgr8":
                img = img[:, :, ::-1]
            path = os.path.join(self.out, name + ".png")
            write_png(path, np.ascontiguousarray(img))
            self.log(f"saved {path} ({msg.width}x{msg.height}, mean level {img.mean():.0f})")
        else:
            self.log(f"{name}: no camera frame on {CAM_TOPIC} within 60 s (bridge running? Sensors system?)")
        scn.gz_remove(WORLD, cam_name)
        self.spin(0.5)
        return ok


FLOOR_SDF = ("<?xml version='1.0'?><sdf version='1.10'><model name='fig_floor'><static>true</static>"
             "<pose>-5 -1 0.0003 0 0 0</pose><link name='link'><visual name='visual'><geometry><box>"
             "<size>30 30 0.0006</size></box></geometry><material><ambient>0.78 0.78 0.76 1</ambient>"
             "<diffuse>0.82 0.82 0.80 1</diffuse></material></visual></link></model></sdf>")


# Viewpoints: camera offset from the robot base (world axes, robot walks along +x),
# field of view (deg) and aim point offset from the base.
CLOSE = {
    "34_front_left": ((0.75, -0.75, 0.45), 40, (0, 0, 0)),
    "34_front_right": ((0.75, 0.75, 0.45), 40, (0, 0, 0)),
    "34_rear_left": ((-0.75, -0.75, 0.45), 40, (0, 0, 0)),
    "side": ((0.0, -1.25, 0.05), 40, (0, 0, 0)),
    "front": ((1.15, 0.0, 0.15), 40, (0, 0, -0.03)),
    "top": ((0.0, -0.02, 1.35), 40, (0, 0, 0)),
    "low_34": ((0.55, -0.45, 0.06), 50, (0, 0, -0.02)),
}
WIDE = {   # whole scene: robot + terrain piece
    "wide_side": ((-0.3, -2.6, 0.30), 45, (-0.35, 0.0, -0.12)),
    "wide_34_front": ((1.5, -1.6, 1.0), 45, (-0.35, 0.0, -0.12)),
    "wide_34_rear": ((-1.9, -1.4, 0.9), 45, (0.2, 0.0, -0.12)),
    "wide_front": ((2.2, 0.0, 0.6), 40, (-0.3, 0.0, -0.1)),
    "wide_top": ((-0.3, -0.05, 2.6), 45, (-0.3, 0.0, 0.0)),
}


def shots(c: Capture, prefix: str, *view_sets) -> None:
    c.speed = 0.0            # same pose in every view: stop, let the feet settle
    c.spin(1.5)
    for views in view_sets:
        for view, (offset, fov, look) in views.items():
            c.shot(f"{prefix}_{view}", offset, fov, look_offset=look)


def scenes(c: Capture, stand_z: float) -> None:
    x0, y0 = START
    # light-grey visual-only floor (no collision, 0.6 mm) over the black default ground plane
    scn.gz_create(WORLD, FLOOR_SDF)
    c.spin(0.5)

    # 1. robot, nominal and raised posture (chapter 3 posture range)
    for label, post in (("nominal", NOMINAL), ("raised", RAISED)):
        c.posture(post, "tripod")
        c.place((x0, y0, 0.0), 0.0, post[0], stand_z, x0, y0)
        shots(c, f"g01_robot_{label}", CLOSE)

    # 2. E3: the four friction mats side by side (colour = mu), robot on the first mats
    c.posture(NOMINAL, "tripod")
    for i, mu in enumerate((1.0, 0.6, 0.3, 0.15)):
        colour = scn.PATCH_COLOURS[mu]
        c.add(scn.box_model_sdf(f"fig_mat_{i}", (0.7, 1.2, 0.02), (x0 - 1.05 + 0.7 * i, y0, 0.01, 0, 0, 0),
                                mu=mu, colour=colour), f"fig_mat_{i}")
    c.spin(0.5)
    c.place((x0, y0, 0.02), 0.0, NOMINAL[0], stand_z, x0 + 0.0, y0)
    mats = {
        "overview": ((-0.2, -2.3, 1.5), 55, (-0.35, 0.0, -0.1)),
        "top": ((-0.35, -0.05, 3.0), 55, (-0.35, 0.0, 0.0)),
        "along": ((-2.6, -0.9, 0.9), 45, (0.0, 0.0, -0.1)),
        "low": ((1.2, -1.2, 0.35), 50, (-0.5, 0.0, -0.1)),
    }
    shots(c, "g03_e3_friction_mats", mats)
    c.clear()

    # 3. E4: 15 deg ramp, Wave, posture control off / on (side views show the levelled body)
    for pc in (False, True):
        piece = scn.terrain({"type": "ramp", "angle_deg": 15}, (x0, y0), "fig_ramp")
        c.posture(NOMINAL, "wave", pc)
        c.add(piece["sdf"], "fig_ramp")
        c.spin(0.5)
        c.place(piece["ground_point"], piece["slope_deg"], NOMINAL[0], stand_z, x0, y0)
        c.speed = 0.025
        c.spin(8.0)                      # walk + let the posture integrator settle
        shots(c, f"g04_e4_ramp15_wave_pc_{'on' if pc else 'off'}", WIDE, CLOSE)
        c.clear()

    # 4. E5: raised Tripod stepping over a 20 mm box; nominal Tripod stalled on a 10 mm box
    for label, post, h, walk in (("raised_20mm", RAISED, 0.02, 22.0), ("nominal_10mm_stall", NOMINAL, 0.01, 30.0)):
        piece = scn.terrain({"type": "obstacle", "height_m": h}, (x0, y0), "fig_box")
        c.posture(post, "tripod")
        c.add(piece["sdf"], "fig_box")
        c.spin(0.5)
        c.place(piece["ground_point"], 0.0, post[0], stand_z, x0, y0)
        c.speed = 0.025
        c.spin(walk)
        shots(c, f"g05_e5_tripod_{label}", WIDE, CLOSE)
        c.clear()
    c.posture(NOMINAL, "tripod")
    scn.gz_remove(WORLD, "fig_floor")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="results/figures_gazebo")
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)
    log = lambda text: print(text, flush=True)  # noqa: E731
    bridge = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                               f"{CAM_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    rclpy.init()
    try:
        boot = rclpy.create_node("fig_boot")
        xml = wait_for_description(boot)
        boot.destroy_node()
        if xml is None:
            log("no /robot_description - is sim.launch.py running?")
            return 1
        model = WholeBodyModel.from_urdf(xml)
        axes = {j.name: j.axis for j in model.joints if j.name in JOINTS}
        node = TrialRecorder(model, axes)
        node.spin_sim(600.0, every=lambda: node.send(0.0),
                      stop=lambda: node.pose is not None and node.loco is not None, wall_limit=180.0)
        if node.pose is None or node.loco is None:
            log("no ground-truth pose / locomotion_state - stack not running")
            return 1
        node.set_posture(*NOMINAL)
        node.spin_sim(4.0, every=lambda: node.send(0.0))
        stand_z = node.pose[1][2]
        log(f"standing base z = {stand_z * 1000:.1f} mm; capturing ...")
        cap = Capture(node, args.out, log)
        try:
            scenes(cap, stand_z)
        finally:
            cap.clear()
        n = len([f for f in os.listdir(args.out) if f.endswith(".png")])
        log(f"done: {n} image(s) in {args.out}")
        return 0 if n else 2
    finally:
        rclpy.try_shutdown()
        try:
            os.killpg(bridge.pid, 15)
        except ProcessLookupError:
            pass


if __name__ == "__main__":
    sys.exit(main())
