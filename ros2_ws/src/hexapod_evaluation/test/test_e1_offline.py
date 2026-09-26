"""E1 offline pipeline on a synthetic leg model (no xacro / ROS needed)."""
import math

import numpy as np

from hexapod_kinematics import LEGS, CadRobotKinematics
from hexapod_evaluation.e1_ik_offline import markdown, run, summarize_rows
from hexapod_evaluation.e1_targets import GRID_DXY_M, GRID_Z_M, nominal_foot, walking_grid


def _urdf() -> str:
    # Same structure as final1.SLDASM (radial mounts, coxa->femur drop,
    # tibia hanging straight down at q=0, lateral foot offset).
    links = ["<link name=\"body_link\"/>"]
    joints = []
    for index, leg in enumerate(LEGS):
        yaw = [0.785, 1.571, 2.356, -0.785, -1.571, -2.356][index]
        links += [f"<link name=\"{leg}_{s}_link\"/>" for s in ("coxa", "femur", "tibia", "foot")]
        joints += [
            f"""<joint name="{leg}_coxa_joint"><parent link="body_link"/><child link="{leg}_coxa_link"/>
                  <origin xyz="{0.12 * math.cos(yaw):.5f} {0.12 * math.sin(yaw):.5f} 0.006" rpy="0 0 {yaw}"/>
                  <axis xyz="0 0 1"/><limit lower="-1.5708" upper="1.5708"/></joint>""",
            f"""<joint name="{leg}_femur_joint"><parent link="{leg}_coxa_link"/><child link="{leg}_femur_link"/>
                  <origin xyz="0.05685 0 -0.019442" rpy="0 0 0"/>
                  <axis xyz="0 -1 0"/><limit lower="-1.5708" upper="1.5708"/></joint>""",
            f"""<joint name="{leg}_tibia_joint"><parent link="{leg}_femur_link"/><child link="{leg}_tibia_link"/>
                  <origin xyz="0.08 0 0" rpy="0 1.5708 0"/>
                  <axis xyz="0 -1 0"/><limit lower="-1.5708" upper="1.5708"/></joint>""",
            f"""<joint name="{leg}_foot_joint"><parent link="{leg}_tibia_link"/><child link="{leg}_foot_link"/>
                  <origin xyz="0.13003 -0.002 0" rpy="0 0 0"/></joint>""",
        ]
    return "<robot name=\"t\">" + "".join(links) + "".join(joints) + "</robot>"


def test_walking_grid_shape_and_centre() -> None:
    robot = CadRobotKinematics.from_urdf(_urdf())
    grids = walking_grid(robot)
    per_leg = len(GRID_DXY_M) ** 2 * len(GRID_Z_M)
    for leg in LEGS:
        assert grids[leg].shape == (per_leg, 3)
        centre = nominal_foot(robot, leg)
        assert np.isclose(grids[leg][:, 0].mean(), centre[0])
        assert np.isclose(grids[leg][:, 1].mean(), centre[1])


def test_run_summary_and_report() -> None:
    robot = CadRobotKinematics.from_urdf(_urdf())
    rows = run(robot, samples_per_leg=20, seed=3)
    summary = summarize_rows(rows)
    grid = summary["walking_grid/analytic/warm"]
    assert grid["n"] == 6 * len(GRID_DXY_M) ** 2 * len(GRID_Z_M)
    assert grid["converged_pct"] == 100.0
    assert grid["error_all_mm"]["max"] < 1e-6
    assert summary["joint_space/analytic/cold"]["converged_pct"] == 100.0
    assert "walking_grid/dls/warm" in summary
    text = markdown(summary, {"generated": "t", "host": "h", "python": "3"})
    assert "Real-time budget, analytic" in text
