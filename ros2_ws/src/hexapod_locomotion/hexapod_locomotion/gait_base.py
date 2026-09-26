"""Shared gait phase behavior and canonical leg ordering.

Ported unchanged from 49x_Hexapod_test -- this part of the old design was
already correct. What changed is how gaits are switched (see
gait_transition.py): the old planner reset phase to 0 on every gait change,
which produced a visible foot-position jump. This module still only
describes one gait's own per-leg phase offsets; the transition logic that
blends between two of these lives next to it.
"""

from abc import ABC

from hexapod_kinematics import LEGS

__all__ = ["LEGS", "GaitBase"]


class GaitBase(ABC):
    duty_factor = 0.5
    offsets: dict[str, float]

    def leg_phase(self, leg: str, phase: float) -> float:
        if leg not in self.offsets:
            raise KeyError(leg)
        return (phase + self.offsets[leg]) % 1.0
