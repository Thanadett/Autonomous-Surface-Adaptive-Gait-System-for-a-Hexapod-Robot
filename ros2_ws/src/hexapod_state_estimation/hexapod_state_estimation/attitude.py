"""Body roll/pitch from the IMU: a complementary filter (pure Python, testable).

Why a filter at all: the real IMU is a BNO055, which also outputs its own fused
orientation - but the controller must not depend on the simulator's ground-truth
orientation, so in simulation roll/pitch are estimated from the same raw signals the
BNO055 fuses (gyro + accelerometer, datasheet noise, hexapod_description).

    angle_k = alpha * (angle_{k-1} + gyro * dt) + (1 - alpha) * angle_from_gravity
    alpha   = tau / (tau + dt)        tau = 0.5 s by default

Gravity direction (REP-103, IMU at rest measures +g along its z): roll = atan2(ay, az),
pitch = atan2(-ax, sqrt(ay^2 + az^2)). Positive roll = left side up, positive pitch =
nose down. Walking accelerations at <= 0.1 m/s are < 0.5 m/s^2 and are what tau
filters out; the gyro carries the fast part. The small-angle Euler-rate approximation
(roll_rate = gx, pitch_rate = gy) is adequate below ~30 deg of tilt.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass
class ComplementaryFilter:
    tau_s: float = 0.5
    roll: float = 0.0
    pitch: float = 0.0
    initialised: bool = False
    last_t: float | None = None

    @staticmethod
    def gravity_angles(ax: float, ay: float, az: float) -> tuple[float, float]:
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.hypot(ay, az))
        return roll, pitch

    def update(self, t: float, gyro, accel) -> tuple[float, float]:
        """gyro (rad/s) and accel (m/s^2) in the IMU/body frame; returns (roll, pitch) rad."""
        roll_acc, pitch_acc = self.gravity_angles(*accel)
        if not self.initialised or self.last_t is None:
            self.roll, self.pitch = roll_acc, pitch_acc
            self.initialised = True
            self.last_t = t
            return self.roll, self.pitch
        dt = t - self.last_t
        self.last_t = t
        if not (0.0 < dt < 0.5):  # clock jump (sim reset / teleport): restart from gravity
            self.roll, self.pitch = roll_acc, pitch_acc
            return self.roll, self.pitch
        alpha = self.tau_s / (self.tau_s + dt)
        self.roll = alpha * (self.roll + gyro[0] * dt) + (1.0 - alpha) * roll_acc
        self.pitch = alpha * (self.pitch + gyro[1] * dt) + (1.0 - alpha) * pitch_acc
        return self.roll, self.pitch
