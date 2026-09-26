"""Locomotion metrics shared by E2-E7 (simulation plan, "นิยามร่วม").

Pure numpy on logged samples, so every experiment computes them the same way
and they can be unit-tested without ROS/Gazebo. Inputs are ground truth from
the simulator (OdometryPublisher pose, measured joint states, contact
sensors, joint force/torque sensors) - evaluation only, never fed to the
gait-selection model.

Definitions
-----------
success   reach `distance_m` along the initial heading, counted from the end
          of the warm-up, within `time_factor` x the nominal time
          (distance / commanded speed), without a fall.
fall      |roll| or |pitch| > tilt_deg, or base_link z < min_z_m.
SSM       signed horizontal distance CoM projection -> support polygon edge
          (support feet = in contact, or within 2 mm of their last touch-down height -
          see support_mask), > 0 inside. The contact-only value is
          reported as ssm_contact_min_m. See hexapod_kinematics.whole_body.
slip      per stance phase (continuous contact), the horizontal displacement
          of the foot's *material contact point*: foot-centre motion plus the
          rotation of the ball foot about its centre (r = 16.3 mm), so a ball
          that rolls without sliding is not counted as slip.
CoT       mechanical: sum over joints of integral |tau * omega| dt / (m g d),
          tau from the joint F/T sensors (component along the joint axis).
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math
import xml.etree.ElementTree as ET

import numpy as np

from hexapod_kinematics import LEGS, SEGMENTS, WholeBodyModel, stability_margin

G = 9.80665
JOINTS = tuple(f"{leg}_{seg}_joint" for leg in LEGS for seg in SEGMENTS)


@dataclass
class TrialSamples:
    """Time-aligned samples of one trial (one row per ground-truth pose, ~50 Hz)."""

    t: np.ndarray                    # (n,) sim time s
    pos: np.ndarray                  # (n, 3) base_link in odom/world, m
    quat: np.ndarray                 # (n, 4) x y z w
    q: np.ndarray                    # (n, 18) joint positions, JOINTS order
    qd: np.ndarray                   # (n, 18) joint velocities
    tau: np.ndarray                  # (n, 18) joint torque about the axis (NaN = no sensor data)
    contact: np.ndarray              # (n, 6) bool, LEGS order
    planned_ssm: np.ndarray = field(default=None)  # (n,) LocomotionState.stability_margin_m
    swing_plan: np.ndarray = field(default=None)   # (n, 6) LocomotionState.leg_in_swing
    # optional (E3+): raw IMU (gyro xyz, accel xyz), attitude estimate (roll, pitch) from
    # hexapod_state_estimation, applied posture correction (roll, pitch) - NaN when absent
    imu: np.ndarray = field(default=None)          # (n, 6)
    att_est: np.ndarray = field(default=None)      # (n, 2)
    posture: np.ndarray = field(default=None)      # (n, 2)

    OPTIONAL = ("imu", "att_est", "posture")
    FIELDS = ("t", "pos", "quat", "q", "qd", "tau", "contact", "planned_ssm", "swing_plan") + OPTIONAL

    def __post_init__(self) -> None:
        n = len(self.t)
        if self.planned_ssm is None:
            self.planned_ssm = np.full(n, np.nan)
        if self.swing_plan is None:
            self.swing_plan = np.zeros((n, 6), dtype=bool)
        for key, width in (("imu", 6), ("att_est", 2), ("posture", 2)):
            if getattr(self, key) is None:
                setattr(self, key, np.full((n, width), np.nan))

    @classmethod
    def from_rows(cls, rows: list[dict]) -> "TrialSamples":
        def col(key, width=None):
            values = [r[key] for r in rows]
            return np.asarray(values, dtype=float if key not in ("contact", "swing_plan") else bool)
        extra = {key: col(key) for key in cls.OPTIONAL if rows and key in rows[0]}
        return cls(t=col("t"), pos=col("pos"), quat=col("quat"), q=col("q"), qd=col("qd"),
                   tau=col("tau"), contact=col("contact"), planned_ssm=col("planned_ssm"),
                   swing_plan=col("swing_plan"), **extra)

    def save(self, path: str) -> None:
        np.savez_compressed(path, **{k: getattr(self, k) for k in self.FIELDS})

    @classmethod
    def load(cls, path: str) -> "TrialSamples":
        data = np.load(path)
        return cls(**{k: data[k] for k in data.files if k in cls.FIELDS})


@dataclass(frozen=True)
class TrialSpec:
    gait: str
    speed_m_s: float
    distance_m: float = 2.0
    time_factor: float = 2.0
    warmup_s: float = 1.0            # excluded from speed/SSM/slip/CoT (acceleration, first cycle)
    tilt_deg: float = 30.0
    min_z_m: float = 0.060
    # Terrain the trial walks on (E3-E5): a plane through ground_point rising at slope_deg
    # along the world direction slope_yaw_deg (E4 ramps; 0 = level). Progress/success are
    # measured along the slope, the fall tilt relative to it, the fall height above it.
    slope_deg: float = 0.0
    slope_yaw_deg: float = 0.0
    ground_point: tuple = (0.0, 0.0, 0.0)
    measure_sink: bool = False       # E3 soft floor: foot sinkage below ground_point's height

    @property
    def time_limit_s(self) -> float:
        return self.time_factor * self.distance_m / self.speed_m_s


# -- geometry helpers ------------------------------------------------------

def quat_to_matrix(quat) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array((
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    ))


def quat_to_rpy(quat) -> tuple[float, float, float]:
    x, y, z, w = (float(v) for v in quat)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Axis-angle vector of a rotation matrix (log map)."""
    cos_angle = max(-1.0, min(1.0, (float(np.trace(rotation)) - 1.0) / 2.0))
    angle = math.acos(cos_angle)
    vee = np.array((rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]))
    if angle < 1e-6:
        return 0.5 * vee
    return angle / (2.0 * math.sin(angle)) * vee


def foot_radius_from_urdf(xml: str, link: str = "front_left_foot_link", default: float = 0.0163182) -> float:
    root = ET.fromstring(xml)
    for element in root.findall("link"):
        if element.attrib.get("name") == link:
            sphere = element.find("collision/geometry/sphere")
            if sphere is not None:
                return float(sphere.attrib["radius"])
    return default


def plane_normal(slope_deg: float, slope_yaw_deg: float = 0.0) -> np.ndarray:
    a, psi = math.radians(slope_deg), math.radians(slope_yaw_deg)
    return np.array((-math.sin(a) * math.cos(psi), -math.sin(a) * math.sin(psi), math.cos(a)))


def fall_reason(pos, quat, tilt_deg: float, min_z_m: float, slope_deg: float = 0.0,
                ground_point=(0.0, 0.0, 0.0), slope_yaw_deg: float = 0.0) -> str | None:
    """'roll' / 'pitch' / 'height' when the fall criterion is met, else None.

    On a slope the tilt is taken relative to the slope (walking up a 25 deg ramp
    normally is not a fall: the body is pitched -25 deg) and the height is measured
    along the plane normal above the ground plane.
    """
    roll, pitch, _ = quat_to_rpy(quat)
    if abs(math.degrees(roll)) > tilt_deg:
        return "roll"
    if abs(math.degrees(pitch) + slope_deg) > tilt_deg:
        return "pitch"
    height = float((np.asarray(pos, dtype=float) - np.asarray(ground_point, dtype=float))
                   @ plane_normal(slope_deg, slope_yaw_deg))
    if height < min_z_m:
        return "height"
    return None


# -- kinematic post-processing ------------------------------------------------

def body_quantities(samples: TrialSamples, model: WholeBodyModel):
    """World foot centres (n,6,3), foot rotations (n,6,3,3) and CoM (n,3).

    One tree walk per sample (~0.1 ms); a 160 s trial at 50 Hz takes < 1 s.
    """
    n = len(samples.t)
    centres = np.zeros((n, 6, 3))
    rotations = np.zeros((n, 6, 3, 3))
    com = np.zeros((n, 3))
    feet = [f"{leg}_foot_link" for leg in LEGS]
    for k in range(n):
        body = np.eye(4)
        body[:3, :3] = quat_to_matrix(samples.quat[k])
        body[:3, 3] = samples.pos[k]
        poses = model.link_transforms(dict(zip(JOINTS, samples.q[k])))
        com[k] = (body @ np.append(model.com(poses=poses), 1.0))[:3]
        for i, link in enumerate(feet):
            world = body @ poses[link]
            centres[k, i] = world[:3, 3]
            rotations[k, i] = world[:3, :3]
    return centres, rotations, com


def support_mask(contact: np.ndarray, swing_plan: np.ndarray, centres: np.ndarray,
                 tolerance_m: float = 0.002, plane: tuple | None = None, foot_radius: float = 0.0) -> np.ndarray:
    """Feet that support the body: in contact, or within `tolerance_m` of the height at
    which that foot last touched the ground (a foot that low catches the body at once).

    Why not contact alone: with position-controlled legs on rigid ground the body is
    statically indeterminate - sub-millimetre differences decide which 3 of the 4-6
    stance feet carry the load at a given instant, the body rocks by ~0.1 deg between
    them, and a contact-only polygon shows CoM "outside" while the unloaded feet hover
    < 1 mm above the ground (E2 25 Sep 2026: Wave averaged 2.9 feet in contact of 5 in
    stance, contact-only SSM min -6 mm, max tilt 0.3 deg).

    Why not the planned swing flags (first version): LocomotionState.leg_in_swing was
    taken at the planner's look-ahead phase, ~65 ms ahead of the measured feet, so at
    every hand-over it dropped the leg that was still on the ground and kept the one
    still 4 mm in the air (E2 Ripple: SSM min 1.2 mm, 10 % of samples < 20 mm). The
    geometric test uses measured positions only. `swing_plan` is accepted for API
    compatibility and ignored.

    Why also the terrain plane (`plane` = (point, unit normal), ball within `tolerance_m`
    of it): on a ramp every new foothold lies stride x sin(slope) above the previous
    one (4-22 mm for 5-25 deg), so a foot that had just landed but carried no load yet
    was > 2 mm above its *last* touch-down and dropped from the polygon - Ripple/Wave
    showed SSM ~0 mm half of the time on every ramp (E4 26 Sep 2026). A foot counts if
    any of the three tests holds; the last touch-down height still covers footholds off
    the plane (E5 obstacle top).
    """
    n, legs = contact.shape
    out = contact.copy()
    if plane is not None:
        point, normal = (np.asarray(v, dtype=float) for v in plane)
        clearance = (centres - point) @ normal - foot_radius      # ball bottom above the plane
        out |= clearance <= tolerance_m
    ground = np.full(legs, np.nan)
    for k in range(n):
        for leg in range(legs):
            if contact[k, leg]:
                ground[leg] = centres[k, leg, 2]
            elif np.isfinite(ground[leg]):
                out[k, leg] |= centres[k, leg, 2] - ground[leg] <= tolerance_m
    return out


def measured_ssm(com: np.ndarray, centres: np.ndarray, contact: np.ndarray) -> np.ndarray:
    """SSM per sample from the feet in contact (-inf when no foot touches)."""
    out = np.empty(len(com))
    for k in range(len(com)):
        stance = centres[k, contact[k], :2]
        out[k] = stability_margin(stance, com[k, :2])
    return out


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Inclusive (start, end) index pairs of consecutive True values."""
    runs, start = [], None
    for i, value in enumerate(mask):
        if value and start is None:
            start = i
        elif not value and start is not None:
            runs.append((start, i - 1))
            start = None
    if start is not None:
        runs.append((start, len(mask) - 1))
    return runs


def stance_slip(centres, rotations, contact, foot_radius: float, *, min_samples: int = 4, trim: int = 1):
    """Slip of every stance phase: list of (leg, start idx, end idx, net m, path m, centre m).

    net/path: sliding of the material contact point (the slip definition).
    centre: horizontal displacement of the ball centre over the stance. The two
    differ by the ball's rotation: the planner holds the foot *centre* still
    while the tibia turns, so even ideal kinematics has a small net slip
    (~r x tibia rotation: ~2 mm at a 12 mm Tripod stride, ~5-10 mm at 50 mm) -
    report it next to the Gazebo value as the kinematic floor.

    `trim` samples are dropped at both ends of a contact run (touch-down impact
    and lift-off are not steady stance). Runs shorter than min_samples after
    trimming are ignored.
    """
    down = np.array((0.0, 0.0, -foot_radius))
    result = []
    for leg in range(contact.shape[1]):
        for start, end in _runs(contact[:, leg]):
            a, b = start + trim, end - trim
            if b - a + 1 < min_samples:
                continue
            steps = []
            for k in range(a, b):
                delta_centre = centres[k + 1, leg] - centres[k, leg]
                phi = rotation_vector(rotations[k + 1, leg] @ rotations[k, leg].T)
                steps.append((delta_centre + np.cross(phi, down))[:2])
            steps = np.asarray(steps)
            result.append((leg, a, b, float(np.linalg.norm(steps.sum(axis=0))),
                           float(np.linalg.norm(steps, axis=1).sum()),
                           float(np.linalg.norm((centres[b, leg] - centres[a, leg])[:2]))))
    return result


def mechanical_energy(t, tau, qd) -> float:
    """Sum_j integral |tau_j * qd_j| dt (J); NaN when < 90 % of torque samples exist."""
    if len(t) < 2:
        return float("nan")
    finite = np.isfinite(tau)
    if finite.mean() < 0.9:
        return float("nan")
    power = np.nansum(np.abs(np.where(finite, tau, 0.0) * qd), axis=1)
    dt = np.diff(t)
    return float(np.sum(0.5 * (power[1:] + power[:-1]) * dt))


# -- whole-trial evaluation ---------------------------------------------------

def evaluate(samples: TrialSamples, spec: TrialSpec, model: WholeBodyModel, *,
             t_command: float, foot_radius: float = 0.0163182,
             effort_limit: float = 3.29, velocity_limit: float = 3.65) -> dict:
    """All E2 metrics of one trial. t_command = sim time the walk command started."""
    n = len(samples.t)
    out: dict = {"gait": spec.gait, "speed_cmd_m_s": spec.speed_m_s, "n_samples": n}
    if n < 3:
        out.update(success=False, fell=False, fall_reason="no_data")
        return out
    t = samples.t
    rpy = np.array([quat_to_rpy(qt) for qt in samples.quat])

    # fall: first sample after the command that meets the criterion
    fall_index, reason = None, None
    for k in np.nonzero(t >= t_command)[0]:
        reason = fall_reason(samples.pos[k], samples.quat[k], spec.tilt_deg, spec.min_z_m,
                             spec.slope_deg, spec.ground_point, spec.slope_yaw_deg)
        if reason:
            fall_index = int(k)
            break

    # measurement window: from the end of the warm-up to the goal / fall / end
    after_warmup = np.nonzero(t >= t_command + spec.warmup_s)[0]
    if after_warmup.size == 0:
        out.update(success=False, fell=fall_index is not None, fall_reason=reason or "too_short")
        return out
    i0 = int(after_warmup[0])
    yaw0 = rpy[i0, 2]
    heading = np.array((math.cos(yaw0), math.sin(yaw0)))
    lateral_axis = np.array((-heading[1], heading[0]))
    alpha = math.radians(spec.slope_deg)
    along = np.array((math.cos(alpha) * heading[0], math.cos(alpha) * heading[1], math.sin(alpha)))
    progress = (samples.pos - samples.pos[i0]) @ along   # along the slope (= horizontal when level)
    reached = np.nonzero((np.arange(n) > i0) & (progress >= spec.distance_m))[0]
    goal_index = int(reached[0]) if reached.size else None
    in_time = goal_index is not None and (t[goal_index] - t[i0]) <= spec.time_limit_s + 1e-9
    fell_before_goal = fall_index is not None and (goal_index is None or fall_index <= goal_index)
    success = bool(in_time and not fell_before_goal)
    end = goal_index if goal_index is not None else n - 1
    if fall_index is not None:
        end = min(end, fall_index)
    end = max(end, i0 + 1)
    w = slice(i0, end + 1)
    duration = float(t[end] - t[i0])
    displacement = samples.pos[end, :2] - samples.pos[i0, :2]
    distance = float(np.linalg.norm(samples.pos[end] - samples.pos[i0])) if alpha else float(np.linalg.norm(displacement))

    out.update(
        success=success,
        fell=fall_index is not None,
        fall_reason=reason if fall_index is not None else "",
        time_to_goal_s=float(t[goal_index] - t[i0]) if goal_index is not None else float("nan"),
        time_limit_s=spec.time_limit_s,
        window_s=duration,
        progress_m=float(progress[end]),
        speed_m_s=float(progress[end] / duration) if duration > 0 else float("nan"),
        speed_ratio=float(progress[end] / duration / spec.speed_m_s) if duration > 0 else float("nan"),
        lateral_drift_m=float(displacement @ lateral_axis),
        yaw_drift_deg=math.degrees(math.atan2(math.sin(rpy[end, 2] - yaw0), math.cos(rpy[end, 2] - yaw0))),
        roll_rms_deg=float(np.degrees(np.sqrt(np.mean(rpy[w, 0] ** 2)))),
        pitch_rms_deg=float(np.degrees(np.sqrt(np.mean(rpy[w, 1] ** 2)))),
        tilt_max_deg=float(np.degrees(np.max(np.abs(rpy[w, :2])))),
        body_z_min_m=float(np.min(samples.pos[w, 2])),
        # E4: absolute (gravity-referenced) body attitude - what posture control levels - and
        # the tilt relative to the slope (what the fall rule uses)
        roll_mean_deg=float(np.degrees(np.mean(rpy[w, 0]))),
        pitch_mean_deg=float(np.degrees(np.mean(rpy[w, 1]))),
        pitch_abs_max_deg=float(np.degrees(np.max(np.abs(rpy[w, 1])))),
        tilt_rel_max_deg=float(max(np.degrees(np.max(np.abs(rpy[w, 0]))),
                                   np.degrees(np.max(np.abs(rpy[w, 1] + alpha))))),
    )
    est = samples.att_est[w]
    if np.any(np.isfinite(est)):
        err = np.degrees(est - rpy[w, :2])
        out.update(att_err_roll_rms_deg=float(np.sqrt(np.nanmean(err[:, 0] ** 2))),
                   att_err_pitch_rms_deg=float(np.sqrt(np.nanmean(err[:, 1] ** 2))),
                   att_err_max_deg=float(np.nanmax(np.abs(err))))
    post = samples.posture[w]
    if np.any(np.isfinite(post)):
        out.update(posture_roll_mean_deg=float(np.degrees(np.nanmean(post[:, 0]))),
                   posture_pitch_mean_deg=float(np.degrees(np.nanmean(post[:, 1]))))

    centres, rotations, com = body_quantities(samples, model)
    support = support_mask(samples.contact, samples.swing_plan, centres,
                           plane=(spec.ground_point, plane_normal(spec.slope_deg, spec.slope_yaw_deg)),
                           foot_radius=foot_radius)
    ssm = measured_ssm(com[w], centres[w], support[w])
    finite = ssm[np.isfinite(ssm)]
    ssm_contact = measured_ssm(com[w], centres[w], samples.contact[w])
    finite_contact = ssm_contact[np.isfinite(ssm_contact)]
    out.update(
        ssm_min_m=float(finite.min()) if finite.size else float("nan"),
        ssm_mean_m=float(finite.mean()) if finite.size else float("nan"),
        ssm_p5_m=float(np.percentile(finite, 5)) if finite.size else float("nan"),
        ssm_negative_frac=float(np.mean(~(ssm > 0))),
        planned_ssm_min_m=float(np.nanmin(samples.planned_ssm[w])) if np.any(np.isfinite(samples.planned_ssm[w])) else float("nan"),
        ssm_contact_min_m=float(finite_contact.min()) if finite_contact.size else float("nan"),
        contact_feet_mean=float(samples.contact[w].sum(axis=1).mean()),
        support_feet_mean=float(support[w].sum(axis=1).mean()),
        stance_feet_planned_mean=float((~samples.swing_plan[w]).sum(axis=1).mean()),
    )

    if spec.measure_sink:
        # lowest foot-ball bottom while in contact, below the (flat) ground height
        bottoms = centres[w][:, :, 2] - foot_radius
        in_contact = samples.contact[w]
        if np.any(in_contact):
            out["foot_sink_max_m"] = float(spec.ground_point[2] - np.min(bottoms[in_contact]))
            out["foot_sink_mean_m"] = float(spec.ground_point[2] - np.mean(bottoms[in_contact]))

    slips = stance_slip(centres[w], rotations[w], samples.contact[w], foot_radius)
    net = np.array([s[3] for s in slips])
    centre = np.array([s[5] for s in slips])
    out.update(
        stance_count=len(slips),
        centre_slip_mean_m=float(centre.mean()) if centre.size else float("nan"),
        slip_mean_m=float(net.mean()) if net.size else float("nan"),
        slip_max_m=float(net.max()) if net.size else float("nan"),
        slip_per_m=float(net.sum() / distance) if net.size and distance > 0 else float("nan"),
    )

    energy = mechanical_energy(t[w], samples.tau[w], samples.qd[w])
    out.update(
        energy_j=energy,
        cot=float(energy / (model.total_mass * G * distance)) if distance > 0 and math.isfinite(energy) else float("nan"),
    )
    for i, seg in enumerate(SEGMENTS):
        cols = [3 * j + i for j in range(6)]
        qd_peak = float(np.max(np.abs(samples.qd[w][:, cols])))
        tau_seg = np.abs(samples.tau[w][:, cols])
        tau_peak = float(np.nanmax(tau_seg)) if np.any(np.isfinite(tau_seg)) else float("nan")
        out[f"qd_peak_{seg}_rad_s"] = qd_peak
        out[f"qd_peak_{seg}_pct"] = 100.0 * qd_peak / velocity_limit
        out[f"tau_peak_{seg}_nm"] = tau_peak
        out[f"tau_peak_{seg}_pct"] = 100.0 * tau_peak / effort_limit
        # p95: the peak alone is dominated by single-sample touch-down transients
        tau_p95 = float(np.nanpercentile(tau_seg, 95)) if np.any(np.isfinite(tau_seg)) else float("nan")
        out[f"tau_p95_{seg}_pct"] = 100.0 * tau_p95 / effort_limit
        out[f"tau_sat_{seg}_frac"] = float(np.nanmean(tau_seg >= 0.98 * effort_limit)) if np.any(np.isfinite(tau_seg)) else float("nan")
    return out
