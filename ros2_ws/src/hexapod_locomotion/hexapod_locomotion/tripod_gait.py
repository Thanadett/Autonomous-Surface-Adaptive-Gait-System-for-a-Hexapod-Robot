from .gait_base import GaitBase


class TripodGait(GaitBase):
    """Two 3-leg tripods, alternating -- fastest gait, least stable."""

    duty_factor = 0.5
    offsets = {
        "front_left": 0.0,
        "middle_right": 0.0,
        "rear_left": 0.0,
        "front_right": 0.5,
        "middle_left": 0.5,
        "rear_right": 0.5,
    }
