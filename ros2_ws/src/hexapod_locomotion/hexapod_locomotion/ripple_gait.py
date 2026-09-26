from .gait_base import GaitBase

LEG_SEQUENCE = (
    "rear_right",
    "middle_left",
    "front_right",
    "rear_left",
    "middle_right",
    "front_left",
)


class RippleGait(GaitBase):
    """Alternating sides, one-sixth of a cycle apart -- medium stability."""

    duty_factor = 2 / 3
    offsets = {name: index / 6 for index, name in enumerate(LEG_SEQUENCE)}
