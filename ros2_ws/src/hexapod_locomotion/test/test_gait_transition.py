import pytest

from hexapod_locomotion.gait_transition import GaitTransitionManager


def test_starts_on_requested_gait_not_blending():
    mgr = GaitTransitionManager("tripod")
    sample = mgr.advance(0.01)
    assert sample.name == "tripod"
    assert not sample.blending
    assert sample.duty_factor == pytest.approx(0.5)


def test_phase_never_resets_across_a_gait_change():
    mgr = GaitTransitionManager("tripod")
    mgr.advance(0.3)
    phase_before = mgr.phase
    mgr.request_gait("wave")
    mgr.advance(0.1)
    # The accumulator only ever grows -- no reset on request_gait().
    assert mgr.phase == pytest.approx(phase_before + 0.1)


def test_blend_starts_at_old_offsets_and_ends_at_new_offsets():
    mgr = GaitTransitionManager("tripod", blend_cycles=1.0)
    mgr.request_gait("wave")

    start = mgr.advance(1e-6)
    assert start.blending
    for leg, value in start.offsets.items():
        assert value == pytest.approx(0.0 if leg in ("front_left", "middle_right", "rear_left") else 0.5, abs=1e-3)

    # Advance almost a full cycle: should be nearly at wave's offsets.
    end = mgr.advance(0.9998)
    assert end.blending
    from hexapod_locomotion.wave_gait import WaveGait

    wave_offsets = WaveGait().offsets
    for leg, value in end.offsets.items():
        assert value == pytest.approx(wave_offsets[leg], abs=1e-3)


def test_blend_completes_and_settles_on_target_gait():
    mgr = GaitTransitionManager("tripod", blend_cycles=1.0)
    mgr.request_gait("ripple")
    for _ in range(20):
        sample = mgr.advance(0.06)
    assert not mgr.blending
    assert sample.name == "ripple"
    assert sample.duty_factor == pytest.approx(2 / 3)


def test_requesting_the_same_gait_again_is_a_no_op():
    mgr = GaitTransitionManager("tripod")
    assert mgr.request_gait("tripod") is False
    assert not mgr.blending


def test_retargeting_mid_blend_is_allowed():
    mgr = GaitTransitionManager("tripod", blend_cycles=1.0)
    mgr.request_gait("wave")
    mgr.advance(0.3)
    assert mgr.request_gait("ripple") is True
    sample = mgr.advance(0.01)
    assert sample.name == "ripple"
    assert sample.blending


def test_dt_phase_must_be_finite():
    mgr = GaitTransitionManager("tripod")
    with pytest.raises(ValueError):
        mgr.advance(float("nan"))
