"""Continuous-phase tripod/ripple/wave walking planner for the CAD model.

Ported from 49x_Hexapod_test's CadTripodPlanner (renamed CadWalkingPlanner
since it now generalizes across gaits, not just tripod) with two behavior
changes called out in the project plan:

  1. Gait switches blend through GaitTransitionManager instead of resetting
     phase to 0 -- see gait_transition.py's module docstring for why the
     old behavior produced a visible foot jump.
  2. The phase accumulator freezes (rather than resets to 0) whenever the
     commanded velocity is ~zero, so stopping and resuming does not
     produce that same kind of jump either.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from hexapod_kinematics import LEGS, CadRobotKinematics, WholeBodyModel, stability_margin

from .cadence import cadence_scale, max_step_height_for
from .gait_transition import GaitTransitionManager
from .trajectory import smoothstep5, vector_foot_trajectory


@dataclass(frozen=True)
class WalkingSettings:
    cycle_time: float = 0.95
    stance_height: float = 0.10
    step_height: float = 0.020
    max_stride: float = 0.050
    max_linear_x: float = 0.065
    max_linear_y: float = 0.065
    max_angular_z: float = 0.400
    smoothing_time: float = 0.18
    blend_cycles: float = 1.0
    # IK solver (E1, chapter 4.1.1): "analytic" = closed form, 100 % convergence
    # over the reachable workspace and ~7x faster; "dls" = the original damped
    # least squares. Legs whose geometry the closed form does not support
    # always use DLS.
    ik_solver: str = "analytic"
    # Branch-flip guard for the closed form: a per-tick joint change larger
    # than this is treated as a possible jump to another IK branch (knee up
    # <-> knee down differ by roughly 1 rad or more, coxa forward <-> backward
    # by ~pi). 0.3 rad/tick at 50 Hz = 15 rad/s: about 2x the fastest joint
    # motion the planner itself commands (Wave at 0.1 m/s peaks ~7.3 rad/s),
    # so legitimate fast swings never trip it. This is a flip detector, not
    # a servo speed limit - see ik_counts / joint speed checks for that.
    ik_branch_jump_rad: float = 0.3
    # Per-gait cycle time (s). A gait missing here uses cycle_time. Wave and
    # Ripple swing each leg for only 1/6 and 1/3 of the cycle, so with the
    # Tripod cycle their swing legs would need joint speeds beyond the servo
    # (E1 follow-up: Wave peaked at 187 % of 3.27 rad/s at 0.10 m/s). Longer
    # cycles keep every gait inside the servo limit; the price is a lower
    # top speed for that gait, which is the expected trade-off (Wave = slow,
    # stable). Values: hexapod_evaluation joint-speed sweep, see gait.yaml.
    cycle_time_by_gait: dict = field(default_factory=dict)
    # Raised steps / other body heights (E2b, 25 Sep 2026): the step height is
    # clamped to what the legs reach at the current body height
    # (cadence.max_step_height_for, <= max_step_height) and the cycle is stretched
    # by cadence.cadence_scale() so the joint speed stays <= 90 % of the servo
    # limit. auto_cadence=False disables the stretch (tools/cadence_table.py needs
    # the raw planner to measure it).
    max_step_height: float = 0.050
    auto_cadence: bool = True
    # Body-height range the legs are validated for (gait.yaml min/max_stance_height_m);
    # the posture correction keeps every foot inside it (see set_tilt_correction).
    min_stance_height: float = 0.070
    max_stance_height: float = 0.135


class CadWalkingPlanner:
    def __init__(
        self,
        robot: CadRobotKinematics,
        settings: WalkingSettings = WalkingSettings(),
        gait_name: str = "tripod",
        mass_model: WholeBodyModel | None = None,
    ) -> None:
        if settings.cycle_time <= 0 or any(t <= 0 for t in settings.cycle_time_by_gait.values()):
            raise ValueError("invalid gait timing")
        if (
            settings.stance_height <= 0
            or settings.step_height < 0
            or settings.max_stride <= 0
        ):
            raise ValueError("invalid gait dimensions")
        if settings.ik_solver not in ("analytic", "dls"):
            raise ValueError(f"unknown ik_solver '{settings.ik_solver}' (analytic | dls)")
        if settings.ik_branch_jump_rad <= 0:
            raise ValueError("ik_branch_jump_rad must be positive")
        self.robot = robot
        # Per-solver call counts (analytic, dls, branch_guard) and the worst IK
        # residual of the last update() - published in LocomotionState.
        self.ik_counts = {"analytic": 0, "dls": 0, "branch_guard": 0}
        self.last_ik_residual_max = 0.0
        # LocomotionState.leg_in_swing / stability_margin_m (Step 2). The margin is the
        # *planned* static stability margin: commanded joint angles, level body, feet the
        # gait keeps in stance - a feed-forward quantity the gait selector can use. The
        # measured SSM (real orientation + contact sensors) is computed by hexapod_evaluation.
        self.mass_model = mass_model
        self.leg_in_swing = {leg: False for leg in LEGS}
        # Posture control (E4): body roll/pitch correction applied as per-foot height
        # offsets, requested by locomotion_node's IMU loop, clamped here to the reach.
        self.tilt_correction = (0.0, 0.0)   # applied (roll, pitch) rad
        self._foot_dz = {leg: 0.0 for leg in LEGS}
        self._tilt_slope = (0.0, 0.0)       # (kx, ky): dz = -kx * x + ky * y at the foot's current target
        self.last_stability_margin = math.nan
        self.settings = settings
        self.stance_height = float(settings.stance_height)
        self.step_height = min(float(settings.step_height), self.max_step_height())
        self.transition = GaitTransitionManager(gait_name, settings.blend_cycles)
        self.filtered_command = np.zeros(3)
        self.neutral_angles: dict[str, np.ndarray] = {}
        self.nominal_feet: dict[str, np.ndarray] = {}
        for leg in LEGS:
            target = robot.legs[leg].forward(np.zeros(3))
            target[2] = -settings.stance_height
            neutral = self._solve(leg, target, None)
            if not neutral.converged:
                raise ValueError(f"neutral CAD stance is unreachable for {leg}")
            self.nominal_feet[leg] = target
            self.neutral_angles[leg] = neutral.angles
        self.angles = {leg: angles.copy() for leg, angles in self.neutral_angles.items()}
        self.last_targets = {leg: point.copy() for leg, point in self.nominal_feet.items()}

    def _solve(self, leg: str, target, seed, dt: float | None = None):  # dt: set only for walking ticks
        """One leg's IK with the configured solver.

        Real-time note: the closed form costs a fixed ~65 us per leg on the
        development PC (E1), DLS 450-600 us and grows with the number of
        iterations, so the analytic path also keeps the 50 Hz tick jitter low.
        """
        chain = self.robot.legs[leg]
        if self.settings.ik_solver == "dls" or not chain.has_analytic_ik:
            self.ik_counts["dls"] += 1
            return chain.inverse(target, seed)
        result = chain.inverse_analytic(target, seed)
        self.ik_counts["analytic"] += 1
        if dt is None or seed is None or not result.converged:
            # no previous tick to compare with, or unreachable (DLS cannot do
            # better on a target the closed form proves out of reach)
            return result
        # Branch-flip guard. The closed form returns the in-limit branch
        # nearest the seed; if that branch just left the joint limits the
        # nearest remaining one can be far away, which would command a jump
        # no servo can follow. DLS warm-started from the seed stays on the
        # current branch (or fails), so prefer it when it is closer.
        step = np.abs(result.angles - np.asarray(seed, dtype=float))
        if np.all(step <= self.settings.ik_branch_jump_rad):
            return result
        self.ik_counts["branch_guard"] += 1
        alternative = chain.inverse(target, seed)
        if alternative.converged and np.max(np.abs(alternative.angles - seed)) < np.max(step):
            return alternative
        return result

    @property
    def active_gait(self) -> str:
        return self.transition.current_name

    @property
    def transitioning(self) -> bool:
        return self.transition.blending

    def max_step_height(self) -> float:
        return max_step_height_for(self.stance_height, self.settings.max_step_height)

    def set_step_height(self, height: float) -> float:
        """Set the swing apex height (m); returns the value applied after clamping to
        [0, max_step_height()] for the current body height."""
        if not math.isfinite(height):
            raise ValueError("step height must be finite")
        self.step_height = float(min(max(height, 0.0), self.max_step_height()))
        return self.step_height

    def cadence_scale_of(self, gait: str) -> float:
        if not self.settings.auto_cadence:
            return 1.0
        return cadence_scale(gait, self.stance_height, self.step_height)

    def cycle_time_of(self, gait: str) -> float:
        """Cycle time actually used: gait.yaml value x cadence scale (step/body height)."""
        base = float(self.settings.cycle_time_by_gait.get(gait, self.settings.cycle_time))
        return base * self.cadence_scale_of(gait)

    def current_cycle_time(self) -> float:
        """Cycle time now, blended across a gait switch with the offset/duty profile."""
        current = self.cycle_time_of(self.transition.current_name)
        target = self.transition.target_name
        if target is None:
            return current
        return current + self.transition.blend_weight() * (self.cycle_time_of(target) - current)

    def set_gait(self, name: str) -> bool:
        return self.transition.request_gait(name)

    def set_tilt_correction(self, roll: float, pitch: float) -> tuple[float, float]:
        """Level the body by (roll, pitch) rad through per-foot height offsets.

        Foot i moves by dz_i = -x_i tan(pitch) + y_i tan(roll) in the body frame
        (x_i, y_i = its nominal point): a positive pitch correction lowers the front
        feet (longer front legs -> nose up), which cancels a measured nose-down pitch;
        a positive roll correction raises the left feet (left side down), which
        cancels a measured left-side-up roll (REP-103 signs).
        Every foot must stay inside the validated body-height range, with room for the
        current step height (the same reach rule as cadence.max_step_height_for), so
        the whole correction is scaled down uniformly when it does not fit - the
        direction is kept, the magnitude saturates. Returns the applied (roll, pitch).

        The offset is applied at the foot's *current* target (x, y) every tick, not at its
        nominal point: with the body levelled on a slope the ground is the plane
        z = -x tan(pitch) + y tan(roll) in the body frame, so the stance stroke must follow
        that plane. A constant per-leg offset made the stroke horizontal: on a 10 deg ramp
        the feet were +-3.5 mm off the ground at the ends of Wave's 50 mm stroke, unloaded
        feet dragged back ~14 mm per stance and Wave lost 45 % of its speed (E4 26 Sep 2026).
        The reach check therefore uses the corners of the stroke (+- max_stride / 2).
        """
        if not (math.isfinite(roll) and math.isfinite(pitch)):
            raise ValueError("tilt correction must be finite")
        kx, ky = math.tan(pitch), math.tan(roll)
        raw = {leg: -self.nominal_feet[leg][0] * kx + self.nominal_feet[leg][1] * ky for leg in LEGS}
        half = 0.5 * self.settings.max_stride
        extremes = [-(self.nominal_feet[leg][0] + sx) * kx + (self.nominal_feet[leg][1] + sy) * ky
                    for leg in LEGS for sx in (-half, half) for sy in (-half, half)]
        # effective body height of foot i = stance - dz_i
        lowest_body = self.settings.min_stance_height + max(0.0, self.step_height - 0.02) * 0.75
        dz_up = self.stance_height - lowest_body                       # max positive dz
        dz_down = self.stance_height - self.settings.max_stance_height  # max negative dz (<= 0)
        scale = 1.0
        for dz in extremes:
            if dz > 1e-12:
                scale = min(scale, max(0.0, dz_up) / dz)
            elif dz < -1e-12:
                scale = min(scale, max(0.0, -dz_down) / -dz)
        self._foot_dz = {leg: scale * dz for leg, dz in raw.items()}   # at the nominal points
        self._tilt_slope = (scale * kx, scale * ky)
        self.tilt_correction = (scale * roll, scale * pitch) if scale > 0 else (0.0, 0.0)
        return self.tilt_correction

    def set_stance_height(self, height: float) -> None:
        if not math.isfinite(height) or height <= 0.0:
            raise ValueError("stance height must be positive and finite")
        next_feet, next_neutral = {}, {}
        for leg in LEGS:
            target = self.nominal_feet[leg].copy()
            target[2] = -height
            neutral = self._solve(leg, target, self.neutral_angles[leg])
            if not neutral.converged:
                raise ValueError(f"stance height is unreachable for {leg}")
            next_feet[leg] = target
            next_neutral[leg] = neutral.angles
        self.nominal_feet.update(next_feet)
        self.neutral_angles.update(next_neutral)
        self.stance_height = float(height)
        # a lower body leaves less room to lift the foot
        self.step_height = min(self.step_height, self.max_step_height())

    @property
    def neutral_positions(self) -> np.ndarray:
        return np.concatenate([self.neutral_angles[leg] for leg in LEGS])

    def set_joint_seed(self, positions) -> None:
        values = np.asarray(positions, dtype=float)
        if values.shape != (18,) or not np.all(np.isfinite(values)):
            raise ValueError("joint seed must contain 18 finite positions")
        for index, leg in enumerate(LEGS):
            self.angles[leg] = values[3 * index : 3 * index + 3].copy()

    def gesture_positions(self, name: str, progress: float, start_positions=None):
        """Return a safe joint pose for a normalized, neutral-to-neutral gesture."""
        normalized = name.strip().lower()
        if normalized not in ("bow", "wave", "sway"):
            raise ValueError(f"unsupported gesture '{name}'")
        if not math.isfinite(progress) or not 0.0 <= progress <= 1.0:
            raise ValueError("gesture progress must be within [0, 1]")
        envelope = math.sin(math.pi * progress) ** 2
        targets = {leg: point.copy() for leg, point in self.nominal_feet.items()}
        wave_angles = None
        if normalized == "bow":
            for leg in ("front_left", "front_right"):
                targets[leg][2] += 0.040 * envelope
            for leg in ("rear_left", "rear_right"):
                targets[leg][2] -= 0.015 * envelope
        elif normalized == "wave":
            wave_angles = self.neutral_angles["front_right"].copy()
            wave_angles[0] += math.radians(22.0) * math.sin(6.0 * math.pi * progress) * envelope
            # Absolute joint targets, so they depend on the URDF sign convention.
            # hexapod_description (final1.SLDASM): femur + = lift knee, tibia + = foot
            # outward/up. (The draft-1 URDF was the opposite: -40 / -15 deg.)
            wave_angles[1] += (math.radians(40.0) - wave_angles[1]) * envelope
            wave_angles[2] += (math.radians(15.0) - wave_angles[2]) * envelope
        else:
            lateral = 0.035 * math.sin(2.0 * math.pi * smoothstep5(progress))
            for leg in LEGS:
                targets[leg][1] += lateral

        solved = {}
        for leg in LEGS:
            if normalized == "wave" and leg == "front_right":
                solved[leg] = wave_angles
            else:
                result = self._solve(leg, targets[leg], self.angles[leg])
                if not result.converged:
                    raise RuntimeError(f"gesture IK failed for {leg}")
                solved[leg] = result.angles
        positions = np.concatenate([solved[leg] for leg in LEGS])
        if start_positions is not None and progress < 0.2:
            start = np.asarray(start_positions, dtype=float)
            if start.shape != (18,) or not np.all(np.isfinite(start)):
                raise ValueError("gesture start must contain 18 finite positions")
            blend = smoothstep5(progress / 0.2)
            positions = start + blend * (positions - start)
        self.set_joint_seed(positions)
        for index, leg in enumerate(LEGS):
            self.last_targets[leg] = self.robot.legs[leg].forward(positions[3 * index : 3 * index + 3])
        # the waving front-right foot is in the air; every other gesture keeps all six down
        self.leg_in_swing = {leg: (normalized == "wave" and leg == "front_right") for leg in LEGS}
        if self.mass_model is not None:
            self.last_stability_margin = self.planned_stability_margin()
        return positions

    def _clamped_command(self, command) -> np.ndarray:
        value = np.asarray(command, dtype=float)
        if value.shape != (3,) or not np.all(np.isfinite(value)):
            raise ValueError("command must be finite (linear_x, linear_y, angular_z)")
        limits = np.array((self.settings.max_linear_x, self.settings.max_linear_y, self.settings.max_angular_z))
        return np.clip(value, -limits, limits)

    def update(self, command, dt: float, *, lookahead: float = 0.0) -> np.ndarray:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive and finite")
        if not math.isfinite(lookahead) or lookahead < 0:
            raise ValueError("lookahead must be finite and non-negative")
        desired = self._clamped_command(command)
        if self.settings.smoothing_time > 0:
            alpha = 1.0 - math.exp(-dt / self.settings.smoothing_time)
            self.filtered_command += alpha * (desired - self.filtered_command)
        else:
            self.filtered_command = desired

        limits = np.array((self.settings.max_linear_x, self.settings.max_linear_y, self.settings.max_angular_z))
        motion_ratio = float(np.max(np.abs(self.filtered_command) / limits))
        moving = motion_ratio > 1e-3

        # Freeze (don't reset) the phase accumulator while stopped, so
        # resuming motion continues smoothly instead of snapping to phase 0.
        cycle_time = self.current_cycle_time()  # per-gait, blended during a switch
        dt_phase = (dt / cycle_time) if moving else 0.0
        if not moving and self.transition.blending:
            # standing still: all feet are down at their nominal points, so the
            # switch can finish at once instead of waiting for the (frozen) phase
            self.transition.complete()
        sample = self.transition.advance(dt_phase)
        if not moving:
            self.filtered_command[:] = 0.0

        sample_phase = self.transition.phase % 1.0
        if moving:
            sample_phase = (sample_phase + lookahead / cycle_time) % 1.0

        vx, vy, wz = self.filtered_command
        stance_time = sample.duty_factor * cycle_time
        lift = self.step_height * min(1.0, 2.0 * motion_ratio)
        next_angles, next_targets = {}, {}
        residual_max = 0.0
        for leg in LEGS:
            nominal = self.nominal_feet[leg]
            velocity_at_foot = np.array((vx - wz * nominal[1], vy + wz * nominal[0]))
            stride = velocity_at_foot * stance_time
            stride_norm = float(np.linalg.norm(stride))
            if stride_norm > self.settings.max_stride:
                stride *= self.settings.max_stride / stride_norm
            leg_phase = (sample_phase + sample.offsets[leg]) % 1.0
            # Swing flag at the *current* phase, not the look-ahead one used for the
            # commanded target: the trajectory is sent `lookahead` s early so the feet
            # arrive on time, and flags taken at that phase led the real feet by ~65 ms
            # (E2 25 Sep 2026: the gait selector / SSM saw a leg in swing that was still
            # on the ground).
            now_phase = (self.transition.phase + sample.offsets[leg]) % 1.0
            self.leg_in_swing[leg] = moving and now_phase >= sample.duty_factor
            offset = vector_foot_trajectory(leg_phase, stride, lift, sample.duty_factor)
            target = nominal + offset
            # posture correction (0 unless enabled): follow the ground plane seen from the
            # levelled body at the foot's current target, see set_tilt_correction
            target[2] += -self._tilt_slope[0] * target[0] + self._tilt_slope[1] * target[1]
            result = self._solve(leg, target, self.angles[leg], dt)
            if not result.converged:
                raise RuntimeError(f"IK failed for {leg}: residual {result.error:.6g} m")
            residual_max = max(residual_max, result.error)
            next_angles[leg] = result.angles
            next_targets[leg] = target

        self.last_ik_residual_max = residual_max
        self.angles.update(next_angles)
        self.last_targets.update(next_targets)
        if self.mass_model is not None:
            self.last_stability_margin = self.planned_stability_margin()
        return np.concatenate([self.angles[leg] for leg in LEGS])

    def planned_stability_margin(self) -> float:
        """SSM (m) of the current command: CoM from the commanded joint angles,
        support polygon from the commanded feet not in swing, body assumed level.

        Cost ~0.1 ms (WholeBodyModel.com, pure numpy) - fine at 50 Hz.
        """
        if self.mass_model is None:
            return math.nan
        positions = {
            f"{leg}_{segment}_joint": float(self.angles[leg][i])
            for leg in LEGS for i, segment in enumerate(("coxa", "femur", "tibia"))
        }
        com = self.mass_model.com(positions)
        stance = [self.last_targets[leg][:2] for leg in LEGS if not self.leg_in_swing[leg]]
        return stability_margin(stance, com[:2])
