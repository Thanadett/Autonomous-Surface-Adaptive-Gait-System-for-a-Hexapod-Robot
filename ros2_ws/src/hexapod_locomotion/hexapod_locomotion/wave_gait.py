from .gait_base import GaitBase

LEG_SEQUENCE = (
    "rear_right",
    "middle_right",
    "front_right",
    "front_left",
    "middle_left",
    "rear_left",
)


class WaveGait(GaitBase):
    """One leg swings at a time -- slowest gait, most stable (rough terrain)."""

    duty_factor = 5 / 6
    offsets = {name: index / 6 for index, name in enumerate(LEG_SEQUENCE)}
