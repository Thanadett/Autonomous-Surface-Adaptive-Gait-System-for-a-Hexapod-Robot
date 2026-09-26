"""Foot-tip target sets for E1 (IK/FK verification, chapter 4.1.1).

Two sets, both expressed in the body_link frame (= base_link, identity):

* walking grid - the part of the workspace the gait planner actually uses:
  the planner's nominal foot position (zero-pose x/y, z = -stance height)
  shifted by dx, dy in {-30, -15, 0, 15, 30} mm and z in {-70, -90, -110,
  -130} mm. The x/y range covers max_stride_m = 50 mm (+-25 mm about the
  nominal point) plus margin; the z range covers min/max_stance_height_m
  (70-135 mm) in gait.yaml. 5 x 5 x 4 = 100 targets per leg.
* joint-space samples - uniform random joint angles inside the URDF limits,
  mapped through FK. Every target is reachable by construction, so this set
  measures IK convergence over the *whole* workspace, not just the easy
  middle, and gives a known ground-truth solution to compare against.
"""
from __future__ import annotations

import numpy as np

from hexapod_kinematics import LEGS, CadRobotKinematics

GRID_DXY_M = (-0.030, -0.015, 0.0, 0.015, 0.030)
GRID_Z_M = (-0.070, -0.090, -0.110, -0.130)
DEFAULT_STANCE_HEIGHT_M = 0.100  # hexapod_locomotion/config/gait.yaml


def nominal_foot(robot: CadRobotKinematics, leg: str, stance_height: float = DEFAULT_STANCE_HEIGHT_M) -> np.ndarray:
    """Same nominal point as CadWalkingPlanner: zero-pose x/y, z = -stance height."""
    target = np.asarray(robot.legs[leg].forward(np.zeros(3)), dtype=float).copy()
    target[2] = -stance_height
    return target


def walking_grid(robot: CadRobotKinematics) -> dict[str, np.ndarray]:
    grids = {}
    for leg in LEGS:
        base = nominal_foot(robot, leg)
        points = [
            (base[0] + dx, base[1] + dy, z)
            for z in GRID_Z_M
            for dx in GRID_DXY_M
            for dy in GRID_DXY_M
        ]
        grids[leg] = np.asarray(points, dtype=float)
    return grids


def joint_space_samples(
    robot: CadRobotKinematics, count: int, rng: np.random.Generator
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Per leg: (targets (count, 3), true joint angles (count, 3))."""
    samples = {}
    for leg in LEGS:
        chain = robot.legs[leg]
        lower = np.array([joint.lower for joint in chain.joints])
        upper = np.array([joint.upper for joint in chain.joints])
        angles = rng.uniform(lower, upper, size=(count, 3))
        targets = np.array([chain.forward(q) for q in angles])
        samples[leg] = (targets, angles)
    return samples
