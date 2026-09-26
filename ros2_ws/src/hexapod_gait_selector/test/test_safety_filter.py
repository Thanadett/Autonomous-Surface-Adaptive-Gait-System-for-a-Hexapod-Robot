import pytest

from hexapod_gait_selector.safety_filter import RuleBasedFallbackInputs, SafetyFilter, SafetyFilterConfig


def test_confident_fresh_candidate_is_accepted_after_dwell():
    filt = SafetyFilter(SafetyFilterConfig(min_dwell_s=0.5), initial_gait="tripod")
    filt.step(dt=1.0, candidate_gait="tripod", candidate_confidence=1.0, feature_age_s=0.0, tilt_variance=0.0)
    result = filt.step(dt=0.6, candidate_gait="wave", candidate_confidence=1.0, feature_age_s=0.0, tilt_variance=0.0)
    assert result.gait == "wave"
    assert result.accepted_candidate


def test_min_dwell_time_blocks_a_too_soon_switch():
    filt = SafetyFilter(SafetyFilterConfig(min_dwell_s=2.0), initial_gait="tripod")
    result = filt.step(dt=0.5, candidate_gait="wave", candidate_confidence=1.0, feature_age_s=0.0, tilt_variance=0.0)
    assert result.gait == "tripod"
    assert not result.accepted_candidate
    assert result.reason == "min_dwell_hold"


def test_low_confidence_holds_current_gait():
    filt = SafetyFilter(SafetyFilterConfig(confidence_threshold=0.7), initial_gait="tripod")
    result = filt.step(dt=5.0, candidate_gait="wave", candidate_confidence=0.4, feature_age_s=0.0, tilt_variance=0.0)
    assert result.gait == "tripod"
    assert result.reason == "low_confidence_hold"


def test_stale_features_fall_back_to_rule_based():
    filt = SafetyFilter(SafetyFilterConfig(max_feature_age_s=0.5), initial_gait="tripod")
    fallback = RuleBasedFallbackInputs(slope_pitch_rad=0.5, slope_roll_rad=0.0, roughness_m=0.0)
    result = filt.step(
        dt=1.0,
        candidate_gait="tripod",
        candidate_confidence=1.0,
        feature_age_s=1.0,
        tilt_variance=0.0,
        rule_based_fallback=fallback,
    )
    assert result.gait == "wave"  # rule_based.select_gait on a 0.5 rad slope
    assert result.reason == "stale_features_fallback"


def test_stale_features_with_no_fallback_holds_current_gait():
    filt = SafetyFilter(SafetyFilterConfig(max_feature_age_s=0.5), initial_gait="ripple")
    result = filt.step(
        dt=1.0, candidate_gait="tripod", candidate_confidence=1.0, feature_age_s=1.0, tilt_variance=0.0
    )
    assert result.gait == "ripple"
    assert result.reason == "stale_features_hold"


def test_high_tilt_variance_overrides_to_wave_immediately():
    filt = SafetyFilter(SafetyFilterConfig(high_tilt_variance_override=0.05), initial_gait="tripod")
    result = filt.step(dt=0.1, candidate_gait="tripod", candidate_confidence=1.0, feature_age_s=0.0, tilt_variance=0.2)
    assert result.gait == "wave"
    assert result.reason == "tilt_variance_override"


def test_switching_gait_resets_dwell_timer():
    filt = SafetyFilter(SafetyFilterConfig(min_dwell_s=1.0), initial_gait="tripod")
    filt.step(dt=2.0, candidate_gait="wave", candidate_confidence=1.0, feature_age_s=0.0, tilt_variance=0.0)
    assert filt.current_gait == "wave"
    assert filt.time_in_current_gait == pytest.approx(0.0)
    # Immediately trying to switch again should be blocked by min dwell.
    result = filt.step(dt=0.1, candidate_gait="ripple", candidate_confidence=1.0, feature_age_s=0.0, tilt_variance=0.0)
    assert result.gait == "wave"
    assert result.reason == "min_dwell_hold"


def test_negative_dt_is_rejected():
    filt = SafetyFilter()
    with pytest.raises(ValueError):
        filt.step(dt=-1.0, candidate_gait="tripod", candidate_confidence=1.0, feature_age_s=0.0, tilt_variance=0.0)
