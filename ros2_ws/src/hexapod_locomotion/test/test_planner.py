import math
import numpy as np
import pytest

from hexapod_kinematics import LEGS, CadRobotKinematics
from hexapod_locomotion.planner import CadWalkingPlanner, WalkingSettings


def _synthetic_urdf() -> str:
    """Small 3-joint-per-leg URDF, independent of the real CAD export --
    see hexapod_kinematics/test/test_cad_kinematics.py for the same
    pattern and the reasoning behind it."""
    links = ["<link name=\"body_link\"/>"]
    joints = []
    for leg in LEGS:
        links += [
            f"<link name=\"{leg}_coxa_link\"/>",
            f"<link name=\"{leg}_femur_link\"/>",
            f"<link name=\"{leg}_tibia_link\"/>",
            f"<link name=\"{leg}_foot_link\"/>",
        ]
        joints += [
            f"""<joint name="{leg}_coxa_joint">
                  <parent link="body_link"/><child link="{leg}_coxa_link"/>
                  <origin xyz="0.08 0.05 0" rpy="0 0 0"/>
                  <axis xyz="0 0 1"/><limit lower="-1.57" upper="1.57"/>
                </joint>""",
            # femur/tibia origins carry a rest-pose fold (mirroring the real
            # CAD's alpha0/knee offsets in geometry.yaml) so the chain is
            # not fully outstretched at q=0 -- a fully outstretched leg has
            # no downward stroke left for any positive stance height.
            f"""<joint name="{leg}_femur_joint">
                  <parent link="{leg}_coxa_link"/><child link="{leg}_femur_link"/>
                  <origin xyz="0.04 0 0" rpy="0 0.86 0"/>
                  <axis xyz="0 -1 0"/><limit lower="-1.2" upper="1.2"/>
                </joint>""",
            f"""<joint name="{leg}_tibia_joint">
                  <parent link="{leg}_femur_link"/><child link="{leg}_tibia_link"/>
                  <origin xyz="0.06 0 0" rpy="0 -2.03 0"/>
                  <axis xyz="0 -1 0"/><limit lower="-1.2" upper="1.2"/>
                </joint>""",
            f"""<joint name="{leg}_foot_joint">
                  <parent link="{leg}_tibia_link"/><child link="{leg}_foot_link"/>
                  <origin xyz="0.09 0 0" rpy="0 0 0"/>
                </joint>""",
        ]
    return "<robot name=\"synthetic\">" + "".join(links) + "".join(joints) + "</robot>"


@pytest.fixture
def robot() -> CadRobotKinematics:
    return CadRobotKinematics.from_urdf(_synthetic_urdf())


@pytest.fixture
def planner(robot: CadRobotKinematics) -> CadWalkingPlanner:
    return CadWalkingPlanner(robot, WalkingSettings(stance_height=0.08), "tripod")


def test_neutral_stance_is_reachable(planner: CadWalkingPlanner) -> None:
    assert planner.active_gait == "tripod"
    for leg in LEGS:
        assert np.all(np.isfinite(planner.neutral_angles[leg]))


def test_zero_command_keeps_feet_at_neutral(planner: CadWalkingPlanner) -> None:
    for _ in range(50):
        planner.update((0.0, 0.0, 0.0), dt=0.02)
    for leg in LEGS:
        np.testing.assert_allclose(
            planner.angles[leg], planner.neutral_angles[leg], atol=1e-6
        )


def test_forward_walking_advances_phase(planner: CadWalkingPlanner) -> None:
    for _ in range(30):
        planner.update((0.05, 0.0, 0.0), dt=0.02)
    assert planner.transition.phase > 0.0


def test_gait_switch_blends_without_a_position_jump(planner: CadWalkingPlanner) -> None:
    """The whole point of gait_transition.py: switching gaits mid-stride must
    not move any foot by more than a single tick's worth of travel."""
    for _ in range(10):
        planner.update((0.05, 0.0, 0.0), dt=0.02)
    before = planner.update((0.05, 0.0, 0.0), dt=0.02).copy()

    planner.set_gait("wave")
    assert planner.transitioning
    after = planner.update((0.05, 0.0, 0.0), dt=0.02)

    # One 0.02 s tick at these speeds moves each joint only a little; a hard
    # phase reset (the old behavior) would jump by much more than this.
    assert np.max(np.abs(after - before)) < 0.05


def test_gait_switch_eventually_settles_on_the_target(planner: CadWalkingPlanner) -> None:
    planner.set_gait("ripple")
    for _ in range(200):
        # 0.03 (was 0.05): the velocity-matched swing overshoots the stride ends by up to
        # 10 %, which is outside the small synthetic leg's workspace at 0.05 m/s
        planner.update((0.03, 0.0, 0.0), dt=0.02)
    assert not planner.transitioning
    assert planner.active_gait == "ripple"


def test_stopping_freezes_phase_instead_of_resetting(planner: CadWalkingPlanner) -> None:
    for _ in range(10):
        planner.update((0.05, 0.0, 0.0), dt=0.02)
    # Command smoothing (command_smoothing_s) decays the filtered command
    # towards zero gradually, not instantly -- give it plenty of ticks to
    # settle before checking that phase has actually frozen, not merely
    # slowed down.
    for _ in range(300):
        planner.update((0.0, 0.0, 0.0), dt=0.02)
    settled_phase = planner.transition.phase
    for _ in range(10):
        planner.update((0.0, 0.0, 0.0), dt=0.02)
    # The old behavior reset phase to 0 every stopped tick; the fix freezes
    # it instead -- it must neither jump back to 0 nor keep advancing.
    assert planner.transition.phase == pytest.approx(settled_phase)
    assert settled_phase > 0.0


def test_stance_height_change_keeps_feet_reachable(planner: CadWalkingPlanner) -> None:
    planner.set_stance_height(0.06)
    for leg in LEGS:
        assert planner.nominal_feet[leg][2] == pytest.approx(-0.06)


def test_gesture_returns_to_neutral_free_joint_seed(planner: CadWalkingPlanner) -> None:
    start = planner.neutral_positions.copy()
    positions = planner.gesture_positions("bow", 1.0, start)
    assert positions.shape == (18,)
    assert np.all(np.isfinite(positions))


# -- IK solver selection (E1: analytic is the default) ----------------------

def test_default_solver_is_analytic_and_matches_dls(robot: CadRobotKinematics) -> None:
    analytic = CadWalkingPlanner(robot, WalkingSettings(stance_height=0.08), "tripod")
    dls = CadWalkingPlanner(robot, WalkingSettings(stance_height=0.08, ik_solver="dls"), "tripod")
    assert analytic.settings.ik_solver == "analytic"
    assert np.allclose(analytic.neutral_positions, dls.neutral_positions, atol=1e-5)
    for _ in range(120):
        qa = analytic.update((0.04, 0.0, 0.1), 0.02)
        qd = dls.update((0.04, 0.0, 0.1), 0.02)
        assert np.allclose(qa, qd, atol=1e-5)
    assert analytic.ik_counts["analytic"] > 0 and analytic.ik_counts["dls"] == 0
    assert dls.ik_counts["dls"] > 0 and dls.ik_counts["analytic"] == 0
    assert analytic.last_ik_residual_max < 1e-7


def test_unknown_solver_rejected(robot: CadRobotKinematics) -> None:
    with pytest.raises(ValueError):
        CadWalkingPlanner(robot, WalkingSettings(ik_solver="newton"), "tripod")


def test_branch_guard_prefers_the_continuous_solution(robot: CadRobotKinematics, monkeypatch) -> None:
    planner = CadWalkingPlanner(robot, WalkingSettings(stance_height=0.08), "tripod")
    leg = LEGS[0]
    chain = robot.legs[leg]
    seed = planner.angles[leg].copy()
    target = chain.forward(seed)
    honest = chain.inverse_analytic

    def flipped(target, seed=None, **kwargs):
        # simulate the closed form landing on a far branch (e.g. knee flipped)
        result = honest(target, seed, **kwargs)
        return type(result)(result.angles + np.array([0.0, 0.8, -0.8]), True, 0.0, 0)

    monkeypatch.setattr(chain, "inverse_analytic", flipped)
    chosen = planner._solve(leg, target, seed, dt=0.02)
    assert planner.ik_counts["branch_guard"] == 1
    assert np.allclose(chosen.angles, seed, atol=1e-5)  # DLS stayed on the current branch


# -- per-gait cycle time -------------------------------------------------------

def test_per_gait_cycle_time_and_smooth_blend(robot: CadRobotKinematics) -> None:
    # auto_cadence off: at stance 80 mm the cadence table stretches the cycle by ~1 %
    settings = WalkingSettings(stance_height=0.08, cycle_time_by_gait={"tripod": 0.95, "wave": 2.4}, auto_cadence=False)
    planner = CadWalkingPlanner(robot, settings, "tripod")
    assert planner.cycle_time_of("tripod") == 0.95
    assert planner.cycle_time_of("wave") == 2.4
    assert planner.cycle_time_of("ripple") == settings.cycle_time  # fallback
    planner.update((0.01, 0.0, 0.0), 0.02)
    planner.set_gait("wave")
    seen = []
    for _ in range(400):
        seen.append(planner.current_cycle_time())
        planner.update((0.01, 0.0, 0.0), 0.02)
    assert seen[0] == 0.95 and planner.current_cycle_time() == 2.4
    assert all(b >= a - 1e-12 for a, b in zip(seen, seen[1:]))  # monotonic, no jump


def test_invalid_cycle_time_rejected(robot: CadRobotKinematics) -> None:
    with pytest.raises(ValueError):
        CadWalkingPlanner(robot, WalkingSettings(cycle_time_by_gait={"wave": 0.0}), "tripod")


# -- Step 2: leg_in_swing, planned SSM, gait switch at rest -----------------

def _with_mass(xml: str) -> str:
    """Synthetic URDF plus a 1 kg body at the origin (and light legs) for the mass model."""
    xml = xml.replace(
        '<link name="body_link"/>',
        '<link name="base_link"><inertial><mass value="0.0"/></inertial></link>'
        '<joint name="base_joint" type="fixed"><parent link="base_link"/><child link="body_link"/></joint>'
        '<link name="body_link"><inertial><origin xyz="0 0 0"/><mass value="1.0"/></inertial></link>',
    )
    for leg in LEGS:
        xml = xml.replace(
            f'<link name="{leg}_femur_link"/>',
            f'<link name="{leg}_femur_link"><inertial><origin xyz="0.03 0 0"/><mass value="0.05"/></inertial></link>',
        )
    return xml


def _hexagon_urdf() -> str:
    """The synthetic legs mounted around a hexagon (the base fixture mounts all six at
    one point, which is fine for IK tests but has no support polygon)."""
    import math

    xml = _synthetic_urdf()
    yaws = {"front_left": 45, "middle_left": 90, "rear_left": 135,
            "front_right": -45, "middle_right": -90, "rear_right": -135}
    for leg, yaw in yaws.items():
        a = math.radians(yaw)
        xml = xml.replace(
            f'<child link="{leg}_coxa_link"/>\n                  <origin xyz="0.08 0.05 0" rpy="0 0 0"/>',
            f'<child link="{leg}_coxa_link"/>\n                  <origin xyz="{0.1 * math.cos(a):.4f} {0.1 * math.sin(a):.4f} 0" rpy="0 0 {a:.6f}"/>',
        )
    assert xml.count('xyz="0.08 0.05 0"') == 0
    return xml


def _massed_planner(**kwargs):
    from hexapod_kinematics import WholeBodyModel

    xml = _with_mass(_hexagon_urdf())
    robot = CadRobotKinematics.from_urdf(xml)
    return CadWalkingPlanner(robot, WalkingSettings(stance_height=0.08, **kwargs), "tripod", WholeBodyModel.from_urdf(xml))


def test_leg_in_swing_follows_the_gait() -> None:
    planner = _massed_planner()
    swing_counts = set()
    for _ in range(100):
        planner.update((0.03, 0.0, 0.0), 0.02)
        swing_counts.add(sum(planner.leg_in_swing.values()))
    # tripod: three legs swing at a time (0 only at the duty-factor boundaries)
    assert max(swing_counts) == 3
    for _ in range(150):  # command smoothing (0.18 s) has to decay below the motion threshold
        planner.update((0.0, 0.0, 0.0), 0.02)
    assert not any(planner.leg_in_swing.values())


def test_planned_stability_margin_is_positive_and_smaller_on_three_legs() -> None:
    planner = _massed_planner()
    planner.update((0.0, 0.0, 0.0), 0.02)
    standing = planner.last_stability_margin
    assert standing > 0.0
    margins = []
    for _ in range(100):
        planner.update((0.03, 0.0, 0.0), 0.02)
        if sum(planner.leg_in_swing.values()) == 3:
            margins.append(planner.last_stability_margin)
    assert margins and 0.0 < min(margins) < standing


def test_margin_is_nan_without_mass_model(planner: CadWalkingPlanner) -> None:
    planner.update((0.03, 0.0, 0.0), 0.02)
    assert np.isnan(planner.last_stability_margin)


def test_gait_switch_at_rest_completes_immediately(planner: CadWalkingPlanner) -> None:
    before = planner.update((0.0, 0.0, 0.0), 0.02).copy()
    assert planner.set_gait("wave")
    after = planner.update((0.0, 0.0, 0.0), 0.02)
    assert planner.active_gait == "wave" and not planner.transitioning
    assert np.allclose(before, after, atol=1e-9)  # no foot moved


def test_stability_margin_geometry() -> None:
    from hexapod_kinematics import stability_margin

    square = [(-1, -1), (1, -1), (1, 1), (-1, 1), (0, 0)]
    assert stability_margin(square, (0, 0)) == pytest.approx(1.0)
    assert stability_margin(square, (0.5, 0.0)) == pytest.approx(0.5)
    assert stability_margin(square, (2.0, 0.0)) == pytest.approx(-1.0)
    assert stability_margin([(0, 0), (2, 0)], (1, 1)) == pytest.approx(-1.0)
    assert stability_margin([], (0, 0)) == -np.inf


@pytest.mark.parametrize("gait", ["tripod", "ripple", "wave"])
def test_stance_feet_move_together(gait: str) -> None:
    """All feet on the ground must move at the same body-frame velocity (no feet
    pushing against each other) - the smoothstep stance profile broke this for
    Ripple and Wave (E2 quick run, 25 Sep 2026)."""
    planner = _massed_planner()
    planner.set_gait(gait)
    dt, previous, spreads = 0.02, None, []
    for k in range(400):
        planner.update((0.03, 0.0, 0.0), dt)
        feet = {leg: planner.last_targets[leg].copy() for leg in LEGS}
        swing = dict(planner.leg_in_swing)
        if previous is not None and k > 150:  # after the blend and the command ramp
            prev_feet, prev_swing = previous
            vx = [(feet[l][0] - prev_feet[l][0]) / dt for l in LEGS if not swing[l] and not prev_swing[l]]
            if len(vx) > 1:
                spreads.append(max(vx) - min(vx))
        previous = (feet, swing)
    assert spreads and max(spreads) < 1e-6


def test_foot_velocity_is_continuous_around_the_cycle() -> None:
    from hexapod_locomotion.trajectory import vector_foot_trajectory

    for duty in (0.5, 2 / 3, 5 / 6):
        phases = np.linspace(0.0, 1.0, 20001)
        x = np.array([vector_foot_trajectory(p, (1.0, 0.0), 0.02, duty)[0] for p in phases])
        velocity = np.gradient(x, phases)
        assert np.max(np.abs(np.diff(velocity))) < 0.02           # no step at lift-off / touch-down
        stance = velocity[phases < duty * 0.99]
        assert np.allclose(stance[5:], -1.0 / duty, atol=1e-6)      # constant stance velocity
        assert np.max(np.abs(x)) < 0.6                              # overshoot <= 10 % of the stride


# -- step height / cadence (E2b) ------------------------------------------------

def test_cadence_table_defaults_and_monotonic() -> None:
    from hexapod_locomotion.cadence import cadence_scale, max_step_height_for

    for gait in ("tripod", "ripple", "wave"):
        assert cadence_scale(gait, 0.10, 0.02) == pytest.approx(1.0)       # gait.yaml cycle unchanged
        ks = [cadence_scale(gait, 0.10, s) for s in (0.02, 0.03, 0.04, 0.05)]
        assert ks == sorted(ks) and ks[-1] > 1.8                          # higher step -> slower cadence
    assert max_step_height_for(0.07) == pytest.approx(0.02)
    assert max_step_height_for(0.085) == pytest.approx(0.04)
    assert max_step_height_for(0.135) == pytest.approx(0.05)


def test_step_height_is_clamped_by_body_height_and_stretches_the_cycle(robot: CadRobotKinematics) -> None:
    settings = WalkingSettings(stance_height=0.08, cycle_time_by_gait={"tripod": 0.95})
    planner = CadWalkingPlanner(robot, settings, "tripod")
    base = planner.cycle_time_of("tripod")
    assert planner.set_step_height(0.2) == pytest.approx(planner.max_step_height())
    assert planner.step_height > 0.02 and planner.cycle_time_of("tripod") > base
    raw = CadWalkingPlanner(robot, WalkingSettings(stance_height=0.08, auto_cadence=False), "tripod")
    raw.set_step_height(0.03)
    assert raw.cycle_time_of("tripod") == pytest.approx(0.95)


# -- posture control (E4) ---------------------------------------------------------

def test_tilt_correction_signs_and_reach_limit() -> None:
    planner = _massed_planner(min_stance_height=0.05, max_stance_height=0.11)
    front = [leg for leg in LEGS if planner.nominal_feet[leg][0] > 0.01]
    left = [leg for leg in LEGS if planner.nominal_feet[leg][1] > 0.01]
    applied = planner.set_tilt_correction(0.0, math.radians(5))     # nose-down measured -> lower front feet
    assert applied[1] > 0 and all(planner._foot_dz[leg] < 0 for leg in front)
    planner.set_tilt_correction(math.radians(5), 0.0)               # left-up measured -> raise left feet
    assert all(planner._foot_dz[leg] > 0 for leg in left)
    applied = planner.set_tilt_correction(0.0, math.radians(40))    # far beyond the reach: scaled down
    assert 0 < applied[1] < math.radians(40)
    assert min(planner._foot_dz.values()) >= planner.stance_height - 0.11 - 1e-9
    assert max(planner._foot_dz.values()) <= planner.stance_height - 0.05 + 1e-9
    before = planner.update((0.0, 0.0, 0.0), 0.02).copy()
    planner.set_tilt_correction(0.0, 0.0)
    after = planner.update((0.0, 0.0, 0.0), 0.02)
    assert not np.allclose(before, after)   # the offsets reach the joint targets


def test_tilt_correction_follows_the_slope_plane_during_stance() -> None:
    # E4 26 Sep 2026: with a constant per-leg offset the stance stroke stayed horizontal
    # while the (levelled) body stood on a slope; the correction must be linear in the
    # foot's current x so every stance foot keeps its height above the inclined ground.
    planner = _massed_planner(min_stance_height=0.05, max_stance_height=0.11)
    applied = planner.set_tilt_correction(0.0, math.radians(4))
    assert applied[1] > 0
    k = planner._tilt_slope[0]
    xs = {leg: [] for leg in LEGS}
    for _ in range(300):
        planner.update((0.03, 0.0, 0.0), dt=0.02)
        for leg in LEGS:
            if planner.leg_in_swing[leg]:
                continue
            target = planner.last_targets[leg]
            # height above the plane z = -k x (body frame) = the uncorrected stance height
            assert abs(target[2] + k * target[0] - planner.nominal_feet[leg][2]) < 1e-4, leg
            xs[leg].append(target[0])
    assert max(max(v) - min(v) for v in xs.values()) > 0.01   # the feet did travel along x
