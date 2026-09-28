"""Lift-off order actually produced by the gait offsets (the planner's swing rule).

The written LEG_SEQUENCE is not automatically the walking order: planner.py lets a leg
swing while (phase + offset) % 1 >= duty_factor, so the order depends on the sign of the
offsets. These tests read the order back from that rule.
"""
import pytest

from hexapod_locomotion.ripple_gait import RippleGait
from hexapod_locomotion.tripod_gait import TripodGait
from hexapod_locomotion.wave_gait import WaveGait


def lift_off_order(gait, steps: int = 6000):
    """Legs in the order they start swinging over one cycle (planner.py swing rule).
    Phases are sampled half a step off the 1/6 grid so no lift-off falls exactly on a sample."""
    def swinging(leg, phase):
        return (phase + gait.offsets[leg]) % 1.0 >= gait.duty_factor
    events = {}
    for leg in gait.offsets:
        prev = swinging(leg, -0.5 / steps)
        for k in range(steps):
            phase = (k + 0.5) / steps
            now = swinging(leg, phase)
            if now and not prev:
                events[leg] = phase
                break
            prev = now
    return [leg for leg, _ in sorted(events.items(), key=lambda item: item[1])]


def rotate_to(order, first):
    i = order.index(first)
    return order[i:] + order[:i]


def test_wave_lifts_rear_to_front_right_side_then_left_side():
    order = rotate_to(lift_off_order(WaveGait()), "rear_right")
    assert order == ["rear_right", "middle_right", "front_right", "rear_left", "middle_left", "front_left"]


def test_wave_lift_offs_are_one_sixth_apart_one_leg_at_a_time():
    gait = WaveGait()
    starts = sorted(((gait.duty_factor - o) % 1.0) for o in gait.offsets.values())
    gaps = [b - a for a, b in zip(starts, starts[1:])] + [1.0 - starts[-1] + starts[0]]
    assert gaps == pytest.approx([1 / 6] * 6)


def test_ripple_order_unchanged():
    # Ripple is deliberately not changed (E2-E5 ran with it): LF -> RM -> LR -> RF -> LM -> RR
    order = rotate_to(lift_off_order(RippleGait()), "front_left")
    assert order == ["front_left", "middle_right", "rear_left", "front_right", "middle_left", "rear_right"]


def test_tripod_swings_alternate_triangles():
    gait = TripodGait()
    a = {leg for leg, o in gait.offsets.items() if o == gait.offsets["front_left"]}
    assert a == {"front_left", "middle_right", "rear_left"}
