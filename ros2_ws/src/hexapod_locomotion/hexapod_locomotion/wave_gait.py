from .gait_base import GaitBase

# Lift-off order (one leg per 1/6 cycle): rear -> front on the right side, then rear ->
# front on the left side, i.e. R3 -> R2 -> R1 -> L3 -> L2 -> L1.
LEG_SEQUENCE = (
    "rear_right", "middle_right", "front_right",
    "rear_left", "middle_left", "front_left",
)


class WaveGait(GaitBase):
    """One leg swings at a time -- slowest gait, most stable (rough terrain)."""

    duty_factor = 5 / 6
    # A leg swings while (phase + offset) % 1 >= duty_factor, so it lifts off at
    # phase = (duty_factor - offset) % 1: a LARGER offset lifts EARLIER. The offsets
    # therefore step by -1/6 along LEG_SEQUENCE so each leg follows the previous one.
    # (Up to 28 Sep 2026 they stepped by +1/6, which ran the sequence backwards:
    # LR -> LM -> LF -> RF -> RM -> RR.)
    offsets = {name: (-index / 6) % 1.0 for index, name in enumerate(LEG_SEQUENCE)}
