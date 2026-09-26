"""Step 2 metrics (SSM, slip, CoT, success/fall) on synthetic data - no ROS needed."""
import math

import numpy as np
import pytest

from hexapod_kinematics import LEGS, WholeBodyModel
from hexapod_evaluation.metrics import (
    JOINTS, TrialSamples, TrialSpec, evaluate, fall_reason, mechanical_energy, quat_to_rpy, stance_slip,
)


def _urdf() -> str:
    """Radial six-leg robot like final1 (test_e1_offline.py) plus masses."""
    links = ['<link name="base_link"/>',
             '<joint name="b" type="fixed"><parent link="base_link"/><child link="body_link"/></joint>',
             '<link name="body_link"><inertial><origin xyz="0 0 0"/><mass value="2.0"/></inertial></link>']
    joints = []
    for index, leg in enumerate(LEGS):
        yaw = [0.785, 1.571, 2.356, -0.785, -1.571, -2.356][index]
        for s in ("coxa", "femur", "tibia", "foot"):
            links.append(f'<link name="{leg}_{s}_link"><inertial><origin xyz="0.02 0 0"/><mass value="0.05"/></inertial>'
                         + ('<collision><geometry><sphere radius="0.0163"/></geometry></collision>' if s == "foot" else "")
                         + "</link>")
        joints += [
            f"""<joint name="{leg}_coxa_joint" type="revolute"><parent link="body_link"/><child link="{leg}_coxa_link"/>
                  <origin xyz="{0.12 * math.cos(yaw):.5f} {0.12 * math.sin(yaw):.5f} 0.006" rpy="0 0 {yaw}"/>
                  <axis xyz="0 0 1"/><limit lower="-1.5708" upper="1.5708"/></joint>""",
            f"""<joint name="{leg}_femur_joint" type="revolute"><parent link="{leg}_coxa_link"/><child link="{leg}_femur_link"/>
                  <origin xyz="0.05685 0 -0.019442" rpy="0 0 0"/>
                  <axis xyz="0 -1 0"/><limit lower="-1.5708" upper="1.5708"/></joint>""",
            f"""<joint name="{leg}_tibia_joint" type="revolute"><parent link="{leg}_femur_link"/><child link="{leg}_tibia_link"/>
                  <origin xyz="0.08 0 0" rpy="0 1.5708 0"/>
                  <axis xyz="0 -1 0"/><limit lower="-1.5708" upper="1.5708"/></joint>""",
            f"""<joint name="{leg}_foot_joint" type="fixed"><parent link="{leg}_tibia_link"/><child link="{leg}_foot_link"/>
                  <origin xyz="0.13003 -0.002 0" rpy="0 0 0"/></joint>""",
        ]
    return '<robot name="t">' + "".join(links) + "".join(joints) + "</robot>"


@pytest.fixture(scope="module")
def model() -> WholeBodyModel:
    return WholeBodyModel.from_urdf(_urdf())


def _straight_walk(speed=0.05, seconds=50.0, rate=50.0, tilt_at=None, tau=0.5, qd=1.0):
    n = int(seconds * rate)
    t = np.arange(n) / rate
    pos = np.zeros((n, 3))
    pos[:, 0] = speed * t
    pos[:, 2] = 0.13
    quat = np.tile((0.0, 0.0, 0.0, 1.0), (n, 1))
    if tilt_at is not None:
        k = int(tilt_at * rate)
        half = math.radians(40.0) / 2
        quat[k:] = (math.sin(half), 0.0, 0.0, math.cos(half))  # 40 deg roll
    return TrialSamples(
        t=t, pos=pos, quat=quat, q=np.zeros((n, 18)), qd=np.full((n, 18), qd), tau=np.full((n, 18), tau),
        contact=np.ones((n, 6), dtype=bool), planned_ssm=np.full(n, 0.09),
    )


def test_straight_walk_success_speed_ssm_cot(model: WholeBodyModel) -> None:
    samples = _straight_walk()
    spec = TrialSpec("tripod", 0.05, distance_m=2.0, warmup_s=1.0)
    out = evaluate(samples, spec, model, t_command=0.0)
    assert out["success"] and not out["fell"]
    assert out["speed_m_s"] == pytest.approx(0.05, rel=1e-3)
    assert out["time_to_goal_s"] == pytest.approx(40.0, abs=0.05)
    assert out["lateral_drift_m"] == pytest.approx(0.0, abs=1e-9)
    # feet at zero pose around a hexagon, CoM inside -> positive margin
    assert out["ssm_min_m"] > 0.05 and out["ssm_negative_frac"] == 0.0
    # 18 joints x |0.5 N.m x 1 rad/s| = 9 W over the 40 s window
    assert out["energy_j"] == pytest.approx(9.0 * 40.0, rel=1e-2)
    assert out["cot"] == pytest.approx(out["energy_j"] / (model.total_mass * 9.80665 * 2.0), rel=1e-6)
    # rigid feet translating with the body while "in contact" = pure slip of the whole stride
    assert out["slip_mean_m"] == pytest.approx(0.05 * (out["window_s"] - 2 / 50.0), rel=0.02)
    assert out["tau_peak_femur_pct"] == pytest.approx(100 * 0.5 / 3.29)


def test_too_slow_is_not_a_success(model: WholeBodyModel) -> None:
    samples = _straight_walk(speed=0.02, seconds=120.0)
    out = evaluate(samples, TrialSpec("wave", 0.05, distance_m=2.0), model, t_command=0.0)
    assert not out["success"] and not out["fell"]  # 100 s needed, limit 80 s


def test_fall_ends_the_trial(model: WholeBodyModel) -> None:
    samples = _straight_walk(tilt_at=10.0)
    out = evaluate(samples, TrialSpec("tripod", 0.05), model, t_command=0.0)
    assert out["fell"] and not out["success"] and out["fall_reason"] == "roll"
    assert out["window_s"] == pytest.approx(9.0, abs=0.05)


def test_fall_reason_thresholds() -> None:
    assert fall_reason((0, 0, 0.1), (0, 0, 0, 1), 30.0, 0.06) is None
    assert fall_reason((0, 0, 0.05), (0, 0, 0, 1), 30.0, 0.06) == "height"
    half = math.radians(31) / 2
    assert fall_reason((0, 0, 0.1), (0, math.sin(half), 0, math.cos(half)), 30.0, 0.06) == "pitch"
    assert math.degrees(quat_to_rpy((0, math.sin(half), 0, math.cos(half)))[1]) == pytest.approx(31.0)


def _rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))


def test_rolling_ball_is_not_slip_but_skidding_is() -> None:
    r, n = 0.0163, 30
    angles = np.linspace(0.0, 0.3, n)
    rolling = np.zeros((n, 1, 3))
    rolling[:, 0, 0] = r * angles          # centre advances r*theta: pure rolling about +y
    rotations = np.stack([_rot_y(a) for a in angles])[:, None]
    contact = np.ones((n, 1), dtype=bool)
    (_, _, _, net, _, centre), = stance_slip(rolling, rotations, contact, r)
    assert net == pytest.approx(0.0, abs=1e-6) and centre > 0.004
    fixed_centre = np.zeros((n, 1, 3))     # ball spinning in place = skidding r*theta
    (_, _, _, net, path, centre), = stance_slip(fixed_centre, rotations, contact, r)
    trimmed = angles[-2] - angles[1]
    assert net == pytest.approx(r * trimmed, rel=1e-3) and path == pytest.approx(net, rel=1e-6) and centre == 0.0


def test_energy_needs_torque_data() -> None:
    t = np.linspace(0, 1, 11)
    assert math.isnan(mechanical_energy(t, np.full((11, 18), np.nan), np.ones((11, 18))))
    assert mechanical_energy(t, np.full((11, 18), -2.0), np.ones((11, 18))) == pytest.approx(36.0)


def test_samples_roundtrip(tmp_path) -> None:
    samples = _straight_walk(seconds=2.0)
    path = str(tmp_path / "trial.npz")
    samples.save(path)
    loaded = TrialSamples.load(path)
    assert np.array_equal(loaded.contact, samples.contact) and loaded.q.shape == (100, 18)
    assert len(JOINTS) == 18


def test_e2_conditions_and_cycle_times(tmp_path) -> None:
    from hexapod_evaluation.e2_plan import conditions, gait_cycle_times

    plan = {"gaits": ["tripod", "ripple", "wave"],
            "speeds": {"common": 0.025, "max": {"tripod": 0.1, "ripple": 0.062, "wave": 0.025}}}
    names = [c["name"] for c in conditions(plan)]
    assert names == ["tripod_common", "tripod_max", "ripple_common", "ripple_max", "wave_common_max"]
    assert [c["name"] for c in conditions(plan, ["wave"])] == ["wave_common_max"]
    gait = tmp_path / "gait.yaml"
    gait.write_text("locomotion_node:\n  ros__parameters:\n    cycle_time_s: 0.95\n    cycle_time_wave_s: 2.4\n")
    assert gait_cycle_times(str(gait)) == {"tripod": 0.95, "ripple": 0.95, "wave": 2.4}


def test_e2_report_tables(tmp_path, model: WholeBodyModel) -> None:
    from hexapod_evaluation.e2_report import write_summary

    rows = []
    for rep, tilt in enumerate((None, None, 10.0)):
        out = evaluate(_straight_walk(tilt_at=tilt), TrialSpec("tripod", 0.05), model, t_command=0.0)
        out.update(trial=f"t{rep}", condition="tripod_common", speed_label="common", rep=rep + 1)
        rows.append(out)
    text = write_summary(rows, {"generated": "now", "distance_m": 2.0}, str(tmp_path))
    assert "| tripod_common | 50 | 2/3 (67%) | 1 (roll) |" in text
    assert (tmp_path / "e2_summary.json").exists()


def test_support_mask_keeps_hovering_stance_feet() -> None:
    from hexapod_evaluation.metrics import support_mask

    n = 5
    contact = np.zeros((n, 2), dtype=bool)
    contact[0] = True                       # both touch down at z = 0.0163
    centres = np.zeros((n, 2, 3))
    centres[:, :, 2] = 0.0163
    centres[2:, 0, 2] = 0.0163 + 0.0005     # leg 0 hovers 0.5 mm (unloaded) -> still support
    centres[2:, 1, 2] = 0.0163 + 0.02       # leg 1 lifted 20 mm
    swing = np.zeros((n, 2), dtype=bool)
    mask = support_mask(contact, swing, centres)
    assert mask[:, 0].all() and mask[:2, 1].all() and not mask[2:, 1].any()
    swing[:, 0] = True                      # the plan flags are ignored: position decides
    assert support_mask(contact, swing, centres)[:, 0].all()


def test_e2_reevaluate_roundtrip(tmp_path, model: WholeBodyModel) -> None:
    import csv
    import json

    from hexapod_evaluation.e2_reevaluate import main as reevaluate

    (tmp_path / "samples").mkdir()
    (tmp_path / "e2_robot.urdf").write_text(_urdf())
    plan = {"time_factor": 2.0, "fall": {"tilt_deg": 30.0, "min_z_m": 0.06}}
    (tmp_path / "e2_meta.json").write_text(json.dumps({"plan": plan, "distance_m": 2.0}))
    samples = _straight_walk()
    samples.save(str(tmp_path / "samples" / "t1.npz"))
    row = {"trial": "t1", "condition": "tripod_common", "speed_label": "common", "rep": 1,
           "t_command": 0.0, "warmup_s": 1.0, "distance_m": 2.0}
    row.update(evaluate(samples, TrialSpec("tripod", 0.05), model, t_command=0.0))
    row["speed_m_s"] = -1.0  # stale value that must be recomputed
    with open(tmp_path / "e2_trials.csv", "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row))
        writer.writeheader()
        writer.writerow(row)
    assert reevaluate(["--out", str(tmp_path)]) == 0
    rows = list(csv.DictReader(open(tmp_path / "e2_trials.csv")))
    assert float(rows[0]["speed_m_s"]) == pytest.approx(0.05, rel=1e-3)
    assert (tmp_path / "e2_trials.prev.csv").exists()


def test_ramp_progress_is_along_the_slope_and_tilt_is_relative(model: WholeBodyModel) -> None:
    a = math.radians(20)
    samples = _straight_walk(speed=0.05, seconds=30.0)
    s = samples.t * 0.05
    samples.pos[:, 0] = s * math.cos(a)
    samples.pos[:, 2] = 0.3 + s * math.sin(a) + 0.12 / math.cos(a)
    samples.quat[:] = (0.0, math.sin(-a / 2), 0.0, math.cos(-a / 2))   # nose up 20 deg
    spec = TrialSpec("wave", 0.05, distance_m=1.0, warmup_s=1.0, slope_deg=20.0, ground_point=(0.0, 0.0, 0.3))
    out = evaluate(samples, spec, model, t_command=0.0)
    assert out["success"] and not out["fell"]                 # -20 deg pitch is not a fall on the ramp
    assert out["speed_m_s"] == pytest.approx(0.05, rel=1e-3)   # measured along the slope
    assert out["pitch_mean_deg"] == pytest.approx(-20.0, abs=1e-6)
    assert out["tilt_rel_max_deg"] == pytest.approx(0.0, abs=1e-6)
    flat = TrialSpec("wave", 0.05, distance_m=1.0, warmup_s=1.0, ground_point=(0.0, 0.0, 0.3))
    assert evaluate(samples, flat, model, t_command=0.0)["speed_m_s"] < 0.048   # horizontal only


def test_support_mask_counts_unloaded_foot_on_a_ramp_plane() -> None:
    # E4 26 Sep 2026: a foot that just landed up-slope (8.7 mm above its last touch-down
    # on a 10 deg ramp) but carries no load yet must still count as support.
    import math
    from hexapod_evaluation.metrics import plane_normal, support_mask
    a, r = math.radians(10.0), 0.016
    n = plane_normal(10.0)
    contact = np.array([[True], [False], [False]])
    # sample 0: in contact at x = 0; 1: landed at x = 0.05 (on the plane), unloaded; 2: 10 mm above the plane
    on_plane = lambda x, lift=0.0: np.array((x, 0.0, x * math.tan(a))) + n * (r + lift)  # noqa: E731
    centres = np.array([[on_plane(0.0)], [on_plane(0.05)], [on_plane(0.05, 0.010)]])
    swing = np.zeros_like(contact)
    old = support_mask(contact, swing, centres)
    new = support_mask(contact, swing, centres, plane=((0.0, 0.0, 0.0), n), foot_radius=r)
    assert not old[1, 0]               # last-touch-down rule alone misses it
    assert new[1, 0] and not new[2, 0]
