"""Unit tests for CadRobotKinematics/LegKinematics.

Deliberately self-contained: builds a small synthetic 3-joint-per-leg URDF
string (not the real CAD export) so this test exercises the FK/IK math
without depending on hexapod_description's file layout or on xacro being
installed. hexapod_description/scripts/validate_urdf.sh is what checks the
real CAD model end to end.
"""
import math

import numpy as np
import pytest

from hexapod_kinematics import LEGS, CadRobotKinematics


def _synthetic_urdf() -> str:
    links = ["<link name=\"body_link\"/>"]
    joints = []
    for leg in LEGS:
        links += [
            f"<link name=\"{leg}_coxa_link\"/>",
            f"<link name=\"{leg}_femur_link\"/>",
            f"<link name=\"{leg}_tibia_link\"/>",
        ]
        joints += [
            f"""<joint name="{leg}_coxa_joint">
                  <parent link="body_link"/><child link="{leg}_coxa_link"/>
                  <origin xyz="0.08 0.05 0" rpy="0 0 0"/>
                  <axis xyz="0 0 1"/><limit lower="-1.57" upper="1.57"/>
                </joint>""",
            f"""<joint name="{leg}_femur_joint">
                  <parent link="{leg}_coxa_link"/><child link="{leg}_femur_link"/>
                  <origin xyz="0.04 0 0" rpy="0 0 0"/>
                  <axis xyz="0 -1 0"/><limit lower="-1.2" upper="1.2"/>
                </joint>""",
            f"""<joint name="{leg}_tibia_joint">
                  <parent link="{leg}_femur_link"/><child link="{leg}_tibia_link"/>
                  <origin xyz="0.06 0 0" rpy="0 0 0"/>
                  <axis xyz="0 -1 0"/><limit lower="-1.2" upper="1.2"/>
                </joint>""",
            f"""<joint name="{leg}_foot_joint">
                  <parent link="{leg}_tibia_link"/><child link="{leg}_foot_link"/>
                  <origin xyz="0.09 0 0" rpy="0 0 0"/>
                </joint>""",
        ]
        links.append(f"<link name=\"{leg}_foot_link\"/>")
    return "<robot name=\"synthetic\">" + "".join(links) + "".join(joints) + "</robot>"


@pytest.fixture(scope="module")
def robot() -> CadRobotKinematics:
    return CadRobotKinematics.from_urdf(_synthetic_urdf())


def test_all_six_legs_parsed(robot: CadRobotKinematics) -> None:
    assert set(robot.legs) == set(LEGS)
    assert len(robot.joint_names) == 18


def test_forward_zero_matches_chain_length(robot: CadRobotKinematics) -> None:
    leg = robot.legs["front_left"]
    position = leg.forward(np.zeros(3))
    # coxa(0.08,0.05,0) + femur(0.04,0,0) + tibia(0.06,0,0) + foot(0.09,0,0)
    assert math.isclose(position[0], 0.08 + 0.04 + 0.06 + 0.09, abs_tol=1e-9)
    assert math.isclose(position[1], 0.05, abs_tol=1e-9)
    assert math.isclose(position[2], 0.0, abs_tol=1e-9)


@pytest.mark.parametrize("leg_name", LEGS)
def test_inverse_round_trips_a_reachable_target(robot: CadRobotKinematics, leg_name: str) -> None:
    leg = robot.legs[leg_name]
    truth_angles = np.array([0.2, -0.3, 0.15])
    target = leg.forward(truth_angles)
    result = leg.inverse(target)
    assert result.converged
    assert result.error < 1e-6
    recovered = leg.forward(result.angles)
    assert np.allclose(recovered, target, atol=1e-6)


def test_inverse_rejects_out_of_reach_target(robot: CadRobotKinematics) -> None:
    leg = robot.legs["front_left"]
    far_away = np.array([100.0, 100.0, 100.0])
    result = leg.inverse(far_away, max_iterations=20)
    assert not result.converged


# -- closed-form IK (inverse_analytic) ------------------------------------

def _folded_urdf() -> str:
    """Harder geometry for the analytic solver: yawed/raised mounts, a
    vertical coxa-femur offset, folded femur/tibia origins (rotations about
    y), a +y tibia axis and a lateral foot offset - the same structure as
    the final1.SLDASM legs, with different numbers."""
    links = ["<link name=\"body_link\"/>"]
    joints = []
    for index, leg in enumerate(LEGS):
        yaw = -2.4 + 0.95 * index
        links += [f"<link name=\"{leg}_{s}_link\"/>" for s in ("coxa", "femur", "tibia", "foot")]
        joints += [
            f"""<joint name="{leg}_coxa_joint"><parent link="body_link"/><child link="{leg}_coxa_link"/>
                  <origin xyz="{0.1 * math.cos(yaw):.6f} {0.1 * math.sin(yaw):.6f} 0.006" rpy="0 0 {yaw:.6f}"/>
                  <axis xyz="0 0 1"/><limit lower="-1.57" upper="1.57"/></joint>""",
            f"""<joint name="{leg}_femur_joint"><parent link="{leg}_coxa_link"/><child link="{leg}_femur_link"/>
                  <origin xyz="0.057 0 -0.019" rpy="0 0.3 0"/>
                  <axis xyz="0 -1 0"/><limit lower="-1.57" upper="1.57"/></joint>""",
            f"""<joint name="{leg}_tibia_joint"><parent link="{leg}_femur_link"/><child link="{leg}_tibia_link"/>
                  <origin xyz="0.08 0.001 0" rpy="0 1.4 0"/>
                  <axis xyz="0 1 0"/><limit lower="-1.57" upper="1.57"/></joint>""",
            f"""<joint name="{leg}_foot_joint"><parent link="{leg}_tibia_link"/><child link="{leg}_foot_link"/>
                  <origin xyz="0.13 -0.002 0" rpy="0 0 0"/></joint>""",
        ]
    return "<robot name=\"folded\">" + "".join(links) + "".join(joints) + "</robot>"


@pytest.mark.parametrize("urdf", [_synthetic_urdf, _folded_urdf], ids=["straight", "folded"])
def test_analytic_ik_round_trips_whole_workspace(urdf) -> None:
    robot = CadRobotKinematics.from_urdf(urdf())
    rng = np.random.default_rng(7)
    for leg_name in LEGS:
        leg = robot.legs[leg_name]
        assert leg.has_analytic_ik
        lower = np.array([j.lower for j in leg.joints])
        upper = np.array([j.upper for j in leg.joints])
        for truth in rng.uniform(lower, upper, size=(200, 3)):
            target = leg.forward(truth)
            result = leg.inverse_analytic(target, truth)
            assert result.converged
            assert result.error < 1e-9
            assert np.all(result.angles >= lower) and np.all(result.angles <= upper)


def test_analytic_ik_picks_the_branch_nearest_the_seed() -> None:
    leg = CadRobotKinematics.from_urdf(_folded_urdf()).legs["front_left"]
    truth = np.array([0.2, 0.4, -0.5])
    result = leg.inverse_analytic(leg.forward(truth), truth + 0.05)
    assert np.allclose(result.angles, truth, atol=1e-9)


def test_analytic_ik_matches_dls_on_the_same_branch() -> None:
    leg = CadRobotKinematics.from_urdf(_folded_urdf()).legs["middle_right"]
    truth = np.array([-0.1, 0.25, -0.3])
    target = leg.forward(truth)
    dls = leg.inverse(target, truth + 0.02)
    analytic = leg.inverse_analytic(target, truth + 0.02)
    assert dls.converged and analytic.converged
    assert np.allclose(dls.angles, analytic.angles, atol=1e-5)


def test_analytic_ik_reports_unreachable_target() -> None:
    leg = CadRobotKinematics.from_urdf(_folded_urdf()).legs["front_left"]
    result = leg.inverse_analytic(np.array([1.0, 1.0, 1.0]))
    assert not result.converged
    assert result.error > 0.5
    lower = np.array([j.lower for j in leg.joints])
    upper = np.array([j.upper for j in leg.joints])
    assert np.all(result.angles >= lower) and np.all(result.angles <= upper)
