"""Foot paths: constant-velocity stance and a velocity-matched quintic swing.

Ported from 49x_Hexapod_test. 25 Sep 2026 (E2): the stance became linear and the
swing a quintic Hermite whose end velocities match the stance, so the foot
velocity is continuous around the whole cycle (see vector_foot_trajectory).
"""

import math
import numpy as np


def smoothstep5(phase: float) -> float:
    if not 0 <= phase <= 1:
        raise ValueError("phase must be in [0,1]")
    return 10 * phase**3 - 15 * phase**4 + 6 * phase**5


def swing_profile(time: float, end_velocity: float) -> float:
    """Quintic Hermite -0.5 -> +0.5 on time in [0, 1] with the given velocity at both
    ends (units: stride per unit swing time) and zero end acceleration."""
    t = time
    t3, t4, t5 = t**3, t**4, t**5
    h_p0 = 1 - 10 * t3 + 15 * t4 - 6 * t5
    h_v0 = t - 6 * t3 + 8 * t4 - 3 * t5
    h_p1 = 10 * t3 - 15 * t4 + 6 * t5
    h_v1 = -4 * t3 + 7 * t4 - 3 * t5
    return -0.5 * h_p0 + 0.5 * h_p1 + end_velocity * (h_v0 + h_v1)


def vector_foot_trajectory(
    phase: float, stride_xy, step_height: float, duty_factor: float
) -> np.ndarray:
    """Return a body-frame XYZ offset for a two-dimensional stride vector."""
    if not 0 < duty_factor < 1:
        raise ValueError("duty_factor must be in (0,1)")
    stride = np.asarray(stride_xy, dtype=float)
    if stride.shape != (2,) or not np.all(np.isfinite(stride)):
        raise ValueError("stride_xy must contain two finite values")
    if not math.isfinite(step_height) or step_height < 0:
        raise ValueError("step_height must be finite and non-negative")
    normalized = phase % 1.0
    if normalized < duty_factor:
        # Stance: constant velocity (linear in phase). Every stance foot must move
        # at the same velocity relative to the body, otherwise the feet on the
        # ground push against each other. The earlier quintic (smoothstep) stance
        # profile was only consistent for Tripod, whose stance legs are in phase;
        # in Ripple/Wave the stance feet moved at 0 ... 1.875 x the mean speed at
        # the same instant (E2 quick run 25 Sep 2026: Ripple walked at 140 % of the
        # command, Wave at 75 % with 20 mm drift per 0.5 m, only ~3 of 4-5 stance
        # feet in contact, joint torques at the servo limit). The swing below starts
        # and ends with this same velocity, so the foot velocity stays continuous.
        time = normalized / duty_factor
        scale = 0.5 - time
        height = 0.0
    else:
        # Swing: quintic Hermite from -0.5 to +0.5 stride whose end velocities equal
        # the stance velocity (d scale / d time = -(1 - duty) / duty in swing time),
        # zero end accelerations. The foot therefore leaves and meets the ground
        # moving exactly like the other stance feet: no foot drags at lift-off or
        # touch-down while the new stance legs already push (with a zero-velocity
        # swing end, Tripod lost ~10 % speed and joint torques spiked at every
        # tripod change - E2 quick run 25 Sep 2026). Cost: a higher mid-swing
        # speed (x 2.75 / 1.875 for Tripod, x 2.31 / 1.875 Ripple, x 2.05 / 1.875
        # Wave) and a small backward overshoot at the swing ends - the per-gait
        # cycle times in gait.yaml keep the joint speed under the servo limit.
        time = (normalized - duty_factor) / (1.0 - duty_factor)
        scale = swing_profile(time, -(1.0 - duty_factor) / duty_factor)
        height = step_height * math.sin(math.pi * time) ** 2
    return np.array((stride[0] * scale, stride[1] * scale, height))
