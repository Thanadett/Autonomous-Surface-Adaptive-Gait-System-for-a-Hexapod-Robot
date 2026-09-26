"""Whole-body model from the expanded URDF: link poses, centre of mass and
the static stability margin (SSM).

Used by
  * hexapod_locomotion (LocomotionState.stability_margin_m: the *planned*
    SSM - commanded joint angles, level body, feet the gait keeps in stance)
  * hexapod_evaluation (E2+: the *measured* SSM - measured joint angles,
    ground-truth body orientation, feet the contact sensors report - and
    foot slip, which needs the foot-link orientation as well as its position).

Definitions (simulation plan, "นิยามร่วม"):
  SSM = signed horizontal distance from the CoM projected along gravity to
  the nearest edge of the support polygon (convex hull of the stance feet);
  > 0 inside the polygon, < 0 outside. With fewer than three stance feet
  there is no polygon: the margin is minus the distance to the segment or
  point (always <= 0), so a statically unstable instant is never reported
  as stable.

Real-time note: link_transforms() is a plain tree walk (32 links on the
final1 model, one 4x4 product per joint); com() on the development PC takes
~0.1-0.2 ms in pure numpy, i.e. < 1 % of the 20 ms locomotion tick.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import numpy as np

from .cad_kinematics import _axis_rotation, _origin_matrix, _rotation_rpy


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray  # 4x4, parent link -> joint frame at q = 0
    axis: np.ndarray    # unit axis in the joint (= child link) frame


class WholeBodyModel:
    """Kinematic tree + link inertials of the robot_description."""

    def __init__(self, root: str, joints: list[_Joint], inertials: dict[str, tuple[float, np.ndarray]]):
        self.root = root
        self.inertials = inertials  # link -> (mass kg, CoM in link frame)
        self.total_mass = float(sum(m for m, _ in inertials.values()))
        if self.total_mass <= 0:
            raise ValueError("robot description has no mass")
        # joints in parent-before-child order, so one pass computes every link pose
        children: dict[str, list[_Joint]] = {}
        for joint in joints:
            children.setdefault(joint.parent, []).append(joint)
        ordered, stack = [], [root]
        while stack:
            link = stack.pop()
            for joint in children.get(link, []):
                ordered.append(joint)
                stack.append(joint.child)
        self.joints = ordered
        self.links = [root] + [j.child for j in ordered]
        # masses / local CoMs as arrays in link order, for a vectorised com()
        self._masses = np.array([inertials.get(link, (0.0, None))[0] for link in self.links])
        self._local_com = np.array(
            [np.append(inertials[link][1], 1.0) if link in inertials else (0.0, 0.0, 0.0, 1.0) for link in self.links]
        )

    @classmethod
    def from_urdf(cls, xml: str, root: str = "base_link") -> "WholeBodyModel":
        tree = ET.fromstring(xml)
        joints = []
        for element in tree.findall("joint"):
            kind = element.attrib.get("type", "fixed")
            if kind not in ("fixed", "revolute", "continuous", "prismatic"):
                raise ValueError(f"unsupported joint type {kind}")
            axis_element = element.find("axis")
            axis = np.array((1.0, 0.0, 0.0)) if axis_element is None else np.fromstring(axis_element.attrib["xyz"], sep=" ")
            norm = float(np.linalg.norm(axis))
            joints.append(_Joint(
                name=element.attrib["name"], kind=kind,
                parent=element.find("parent").attrib["link"], child=element.find("child").attrib["link"],
                origin=_origin_matrix(element), axis=axis / norm if norm > 0 else axis,
            ))
        inertials = {}
        for link in tree.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            mass = float(inertial.find("mass").attrib["value"])
            origin = inertial.find("origin")
            com = np.zeros(3) if origin is None else np.fromstring(origin.attrib.get("xyz", "0 0 0"), sep=" ")
            if mass > 0:
                inertials[link.attrib["name"]] = (mass, com)
        # the world -> base_link joint of fixed_base:=true is not part of the robot
        joints = [j for j in joints if j.parent != "world"]
        return cls(root, joints, inertials)

    def link_transforms(self, positions: dict[str, float] | None = None) -> dict[str, np.ndarray]:
        """Pose (4x4) of every link in the root frame; missing joints are at 0."""
        positions = positions or {}
        poses = {self.root: np.eye(4)}
        for joint in self.joints:
            transform = poses[joint.parent] @ joint.origin
            q = float(positions.get(joint.name, 0.0))
            if joint.kind in ("revolute", "continuous") and q != 0.0:
                motion = np.eye(4)
                motion[:3, :3] = _axis_rotation(joint.axis, q)
                transform = transform @ motion
            elif joint.kind == "prismatic" and q != 0.0:
                motion = np.eye(4)
                motion[:3, 3] = joint.axis * q
                transform = transform @ motion
            poses[joint.child] = transform
        return poses

    def com(self, positions: dict[str, float] | None = None, poses: dict[str, np.ndarray] | None = None) -> np.ndarray:
        """Whole-body centre of mass in the root frame."""
        poses = poses if poses is not None else self.link_transforms(positions)
        stacked = np.stack([poses[link] for link in self.links])            # (n, 4, 4)
        world = np.einsum("nij,nj->ni", stacked, self._local_com)[:, :3]   # (n, 3)
        return (self._masses[:, None] * world).sum(axis=0) / self.total_mass


# -- support polygon ---------------------------------------------------------

def convex_hull(points: np.ndarray) -> np.ndarray:
    """2D convex hull, counter-clockwise, collinear points dropped (monotone chain)."""
    pts = sorted(set(map(tuple, np.asarray(points, dtype=float).reshape(-1, 2))))
    if len(pts) <= 2:
        return np.array(pts, dtype=float).reshape(-1, 2)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 1e-12:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 1e-12:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1], dtype=float)


def _segment_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(ab @ ab)
    t = 0.0 if denom <= 0 else min(1.0, max(0.0, float((p - a) @ ab) / denom))
    return float(np.linalg.norm(p - (a + t * ab)))


def stability_margin(stance_xy, com_xy) -> float:
    """Signed distance (m) from com_xy to the support polygon edge; > 0 inside.

    stance_xy: (k, 2) horizontal positions of the feet in stance (any order).
    Returns -inf when no foot is in stance.
    """
    p = np.asarray(com_xy, dtype=float).reshape(2)
    feet = np.asarray(stance_xy, dtype=float).reshape(-1, 2)
    if len(feet) == 0:
        return -math.inf
    hull = convex_hull(feet)
    if len(hull) == 1:
        return -float(np.linalg.norm(p - hull[0]))
    if len(hull) == 2:
        return -_segment_distance(p, hull[0], hull[1])
    edges = [(hull[i], hull[(i + 1) % len(hull)]) for i in range(len(hull))]
    distance = min(_segment_distance(p, a, b) for a, b in edges)
    inside = all(
        (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0.0 for a, b in edges
    )
    return distance if inside else -distance


def rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return _rotation_rpy(f"{roll} {pitch} {yaw}")
