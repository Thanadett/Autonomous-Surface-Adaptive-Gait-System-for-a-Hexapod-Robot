"""Forward and inverse kinematics read directly from the expanded CAD URDF.

Two IK solvers share the same FK:

* inverse() - damped least squares (ported from 49x_Hexapod_test, where it
  was validated against the CAD joint frames). Generic for any 3-joint
  chain; kept as the planner's fallback (ik_solver: dls, unsupported legs,
  branch-flip guard).
* inverse_analytic() - closed form (coxa angle from the leg plane, then a
  planar 2R arm), added for E1 and the planner's default since then. On the final1.SLDASM model it solves 100 %
  of reachable targets to ~1e-13 m and is ~7x faster than inverse(), which
  converges on only ~75 % of uniform joint-space samples (it gets trapped at
  joint limits) - see hexapod_evaluation's E1 report. Both return the same
  branch on the walking workspace (max difference ~1e-6 rad).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import numpy as np

# Canonical leg order shared by every hexapod_* package: kinematics,
# locomotion, state estimation, and the interfaces that carry six-leg
# arrays (e.g. hexapod_interfaces/msg/FootContacts) all index legs this way.
LEGS = (
    "front_left",
    "middle_left",
    "rear_left",
    "front_right",
    "middle_right",
    "rear_right",
)

SEGMENTS = ("coxa", "femur", "tibia")


def _rotation_rpy(value: str) -> np.ndarray:
    roll, pitch, yaw = (float(item) for item in value.split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )
    )


def _origin_matrix(joint: ET.Element) -> np.ndarray:
    result = np.eye(4)
    origin = joint.find("origin")
    if origin is not None:
        result[:3, :3] = _rotation_rpy(origin.attrib.get("rpy", "0 0 0"))
        result[:3, 3] = np.fromstring(
            origin.attrib.get("xyz", "0 0 0"), sep=" ", dtype=float
        )
    return result


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis
    skew = np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)))
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


@dataclass(frozen=True)
class JointGeometry:
    name: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    velocity: float = math.inf  # URDF <limit velocity>, rad/s (inf if absent)


@dataclass(frozen=True)
class IKResult:
    angles: np.ndarray
    converged: bool
    error: float
    iterations: int


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _rot2(angle: float, vector: tuple[float, float]) -> tuple[float, float]:
    """Counter-clockwise rotation in the (x, z) plane of a leg."""
    c, s = math.cos(angle), math.sin(angle)
    return (c * vector[0] - s * vector[1], s * vector[0] + c * vector[1])


class _PlanarLegModel:
    """Closed-form IK for a coxa-femur-tibia chain (see LegKinematics.inverse_analytic).

    Structural assumptions, checked by build(): the coxa axis is parallel to
    the coxa mount frame's z axis; the femur and tibia axes are both
    parallel to their frame's y axis and the femur/tibia joint origins only
    rotate about y. Then every point below the coxa lies in one vertical
    plane of the coxa frame at a constant lateral offset y_off, and the
    femur/tibia pair is a planar 2R arm in the (x, z) coordinates of that
    plane. Positive rotation about -y is counter-clockwise in (x, z).
    """

    def __init__(self, mount_inv, s1, y_off, femur_xz, gamma1, s2, tibia_xz, gamma2, s3, foot_xz):
        self.mount_inv = mount_inv
        self.s1, self.s2, self.s3 = s1, s2, s3
        self.y_off = y_off
        self.femur_xz = femur_xz
        self.gamma1, self.gamma2 = gamma1, gamma2
        self.tibia_xz = tibia_xz
        self.foot_xz = foot_xz
        self.len_t = math.hypot(*tibia_xz)
        self.len_f = math.hypot(*foot_xz)
        self.ang_t = math.atan2(tibia_xz[1], tibia_xz[0])
        self.ang_f = math.atan2(foot_xz[1], foot_xz[0])

    @staticmethod
    def _about_y(rotation: np.ndarray) -> float | None:
        if not (np.allclose(rotation[1], (0.0, 1.0, 0.0), atol=1e-9) and np.allclose(rotation[:, 1], (0.0, 1.0, 0.0), atol=1e-9)):
            return None
        return math.atan2(rotation[0, 2], rotation[0, 0])  # Ry(beta)

    @classmethod
    def build(cls, joints, foot_origin) -> "_PlanarLegModel | None":
        coxa, femur, tibia = joints
        if not np.allclose(np.abs(coxa.axis), (0.0, 0.0, 1.0), atol=1e-9):
            return None
        if not (np.allclose(np.abs(femur.axis), (0.0, 1.0, 0.0), atol=1e-9) and np.allclose(np.abs(tibia.axis), (0.0, 1.0, 0.0), atol=1e-9)):
            return None
        beta1 = cls._about_y(femur.origin[:3, :3])
        beta2 = cls._about_y(tibia.origin[:3, :3])
        if beta1 is None or beta2 is None:
            return None
        t_f, t_t, foot = femur.origin[:3, 3], tibia.origin[:3, 3], foot_origin[:3, 3]
        return cls(
            mount_inv=np.linalg.inv(coxa.origin),
            s1=float(np.sign(coxa.axis[2])),
            y_off=float(t_f[1] + t_t[1] + foot[1]),
            # Ry(beta) turns (x, z) clockwise by beta -> planar angle -beta;
            # rotation by q about -y is counter-clockwise by q -> sign -axis_y.
            femur_xz=(float(t_f[0]), float(t_f[2])), gamma1=-beta1, s2=float(-np.sign(femur.axis[1])),
            tibia_xz=(float(t_t[0]), float(t_t[2])), gamma2=-beta2, s3=float(-np.sign(tibia.axis[1])),
            foot_xz=(float(foot[0]), float(foot[2])),
        )

    def candidates(self, target: np.ndarray) -> list[tuple[np.ndarray, bool]]:
        """All (angles, exact) solutions; exact=False means the target was out of
        reach and the closest configuration in the leg plane is returned."""
        px, py, pz = (self.mount_inv @ np.append(target, 1.0))[:3]
        radius = math.hypot(px, py)
        phi = math.atan2(py, px)
        exact = radius >= abs(self.y_off)
        ratio = max(-1.0, min(1.0, self.y_off / radius)) if radius > 0.0 else 0.0
        out = []
        for theta in (phi - math.asin(ratio), phi - (math.pi - math.asin(ratio))):
            x_plane = px * math.cos(theta) + py * math.sin(theta)
            w = (x_plane - self.femur_xz[0], pz - self.femur_xz[1])
            reach = w[0] * w[0] + w[1] * w[1]
            k = (reach - self.len_t ** 2 - self.len_f ** 2) / (2.0 * self.len_t * self.len_f)
            in_reach = exact and -1.0 <= k <= 1.0
            k = max(-1.0, min(1.0, k))
            for sign in (1.0, -1.0):
                a = self.ang_t - self.ang_f + sign * math.acos(k)
                v = _rot2(a, self.foot_xz)
                v = (self.tibia_xz[0] + v[0], self.tibia_xz[1] + v[1])
                b = math.atan2(w[1], w[0]) - math.atan2(v[1], v[0])
                q = np.array((
                    _wrap(theta / self.s1),
                    _wrap((b - self.gamma1) / self.s2),
                    _wrap((a - self.gamma2) / self.s3),
                ))
                out.append((q, in_reach))
        return out


class LegKinematics:
    """Three-revolute-joint chain expressed in the body_link frame."""

    def __init__(
        self, name: str, joints: tuple[JointGeometry, ...], foot_origin: np.ndarray
    ) -> None:
        if len(joints) != 3:
            raise ValueError(f"{name} must have exactly three actuated joints")
        self.name = name
        self.joints = joints
        self.foot_origin = foot_origin
        self._planar = _PlanarLegModel.build(joints, foot_origin)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.joints)

    def forward(
        self, angles, *, with_jacobian: bool = False
    ) -> np.ndarray | tuple[np.ndarray, np.ndarray]:
        q = np.asarray(angles, dtype=float)
        if q.shape != (3,) or not np.all(np.isfinite(q)):
            raise ValueError("joint angles must contain three finite values")

        transform = np.eye(4)
        joint_positions = []
        axes_in_body = []
        for joint, angle in zip(self.joints, q):
            transform = transform @ joint.origin
            joint_positions.append(transform[:3, 3].copy())
            axes_in_body.append(transform[:3, :3] @ joint.axis)
            rotation = np.eye(4)
            rotation[:3, :3] = _axis_rotation(joint.axis, float(angle))
            transform = transform @ rotation

        position = (transform @ self.foot_origin)[:3, 3]
        if not with_jacobian:
            return position
        jacobian = np.column_stack(
            [
                np.cross(axis, position - joint_position)
                for axis, joint_position in zip(axes_in_body, joint_positions)
            ]
        )
        return position, jacobian

    def inverse(
        self,
        target,
        seed=None,
        *,
        tolerance: float = 1e-7,
        max_iterations: int = 40,
        damping: float = 1e-3,
        max_step: float = 0.15,
    ) -> IKResult:
        desired = np.asarray(target, dtype=float)
        if desired.shape != (3,) or not np.all(np.isfinite(desired)):
            raise ValueError("IK target must contain three finite values")
        angles = np.zeros(3) if seed is None else np.asarray(seed, dtype=float).copy()
        if angles.shape != (3,) or not np.all(np.isfinite(angles)):
            raise ValueError("IK seed must contain three finite values")
        lower = np.array([joint.lower for joint in self.joints])
        upper = np.array([joint.upper for joint in self.joints])
        angles = np.clip(angles, lower, upper)

        for iteration in range(max_iterations + 1):
            position, jacobian = self.forward(angles, with_jacobian=True)
            error_vector = desired - position
            error = float(np.linalg.norm(error_vector))
            if error <= tolerance:
                return IKResult(angles, True, error, iteration)
            if iteration == max_iterations:
                break
            system = jacobian @ jacobian.T + damping * damping * np.eye(3)
            delta = jacobian.T @ np.linalg.solve(system, error_vector)
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > max_step:
                delta *= max_step / delta_norm
            angles = np.clip(angles + delta, lower, upper)

        final_error = float(np.linalg.norm(desired - self.forward(angles)))
        return IKResult(angles, False, final_error, max_iterations)

    @property
    def has_analytic_ik(self) -> bool:
        return self._planar is not None

    def inverse_analytic(self, target, seed=None, *, tolerance: float = 1e-7) -> IKResult:
        """Closed-form IK (coxa angle from the leg plane, then a planar 2R arm).

        Up to four solutions exist (coxa forward/backward x knee up/down).
        Solutions outside the URDF joint limits are dropped and the one
        closest to `seed` (default: the zero pose) is returned, which keeps
        consecutive ticks on the same branch the way the DLS solver's warm
        start does. Unreachable targets return converged=False with the
        in-limit candidate that lands closest to the target.

        Added for E1: unlike inverse() it cannot get trapped at a joint
        limit and costs a fixed handful of trig calls per leg.
        """
        if self._planar is None:
            raise ValueError(f"{self.name}: chain geometry not supported by the analytic solver")
        desired = np.asarray(target, dtype=float)
        if desired.shape != (3,) or not np.all(np.isfinite(desired)):
            raise ValueError("IK target must contain three finite values")
        reference = np.zeros(3) if seed is None else np.asarray(seed, dtype=float)
        lower = np.array([joint.lower for joint in self.joints])
        upper = np.array([joint.upper for joint in self.joints])

        # Exact candidates are exact by construction, so FK is only evaluated
        # once for the chosen one (as a check) - keeps the solver at a few
        # trig calls per leg instead of four FK evaluations.
        candidates = self._planar.candidates(desired)
        exact = [
            angles for angles, in_reach in candidates
            if in_reach and np.all(angles >= lower - 1e-9) and np.all(angles <= upper + 1e-9)
        ]
        if exact:
            angles = min(exact, key=lambda q: max(abs(_wrap(d)) for d in q - reference))
            angles = np.clip(angles, lower, upper)
            error = float(np.linalg.norm(self.forward(angles) - desired))
            return IKResult(angles, error <= tolerance, error, 0)
        # Unreachable (or reachable only outside the limits): closest in-limit pose.
        scored = []
        for angles, _ in candidates:
            clipped = np.clip(angles, lower, upper)
            error = float(np.linalg.norm(self.forward(clipped) - desired))
            scored.append((error, max(abs(_wrap(d)) for d in clipped - reference), clipped))
        error, _, angles = min(scored, key=lambda item: (item[0], item[1]))
        exact = error <= tolerance
        return IKResult(angles, exact, error, 0)


class CadRobotKinematics:
    """Six CAD leg chains parsed from the robot_description XML."""

    def __init__(self, legs: dict[str, LegKinematics]) -> None:
        if set(legs) != set(LEGS):
            raise ValueError("robot description must contain all six canonical legs")
        self.legs = legs

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(f"{leg}_{segment}_joint" for leg in LEGS for segment in SEGMENTS)

    @classmethod
    def from_urdf(cls, xml: str) -> "CadRobotKinematics":
        root = ET.fromstring(xml)
        joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        legs = {}
        for leg in LEGS:
            geometries = []
            expected_parent = "body_link"
            for segment in SEGMENTS:
                name = f"{leg}_{segment}_joint"
                if name not in joints:
                    raise ValueError(f"robot description is missing {name}")
                joint = joints[name]
                parent = joint.find("parent").attrib["link"]
                child = joint.find("child").attrib["link"]
                if parent != expected_parent:
                    raise ValueError(f"unexpected parent for {name}: {parent}")
                axis = np.fromstring(joint.find("axis").attrib["xyz"], sep=" ")
                norm = float(np.linalg.norm(axis))
                if axis.shape != (3,) or norm <= 1e-12:
                    raise ValueError(f"invalid axis for {name}")
                limit = joint.find("limit")
                geometries.append(
                    JointGeometry(
                        name=name,
                        origin=_origin_matrix(joint),
                        axis=axis / norm,
                        lower=float(limit.attrib["lower"]),
                        upper=float(limit.attrib["upper"]),
                        velocity=float(limit.attrib.get("velocity", "inf")),
                    )
                )
                expected_parent = child

            foot_name = f"{leg}_foot_joint"
            if foot_name not in joints:
                raise ValueError(f"robot description is missing {foot_name}")
            foot_joint = joints[foot_name]
            if foot_joint.find("parent").attrib["link"] != expected_parent:
                raise ValueError(f"unexpected parent for {foot_name}")
            legs[leg] = LegKinematics(
                leg, tuple(geometries), _origin_matrix(foot_joint)
            )
        return cls(legs)
