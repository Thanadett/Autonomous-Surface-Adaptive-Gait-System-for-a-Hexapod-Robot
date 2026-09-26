from hexapod_gait_selector.rule_based import RuleThresholds, select_gait


def test_flat_ground_selects_tripod():
    assert select_gait(0.0, 0.0, 0.0, 0.0) == "tripod"


def test_moderate_slope_selects_ripple():
    thresholds = RuleThresholds()
    slope = (thresholds.ripple_slope_rad + thresholds.wave_slope_rad) / 2
    assert select_gait(slope, 0.0, 0.0, 0.0) == "ripple"


def test_steep_slope_selects_wave():
    thresholds = RuleThresholds()
    assert select_gait(thresholds.wave_slope_rad + 0.05, 0.0, 0.0, 0.0) == "wave"


def test_roughness_alone_can_trigger_wave():
    thresholds = RuleThresholds()
    assert select_gait(0.0, 0.0, thresholds.wave_roughness_m + 0.01, 0.0) == "wave"


def test_high_tilt_variance_forces_wave_even_on_flat_ground():
    thresholds = RuleThresholds()
    assert select_gait(0.0, 0.0, 0.0, thresholds.high_tilt_variance + 0.01) == "wave"


def test_roll_and_pitch_both_considered_via_max():
    thresholds = RuleThresholds()
    # Large roll, zero pitch should behave like large pitch, zero roll.
    assert select_gait(0.0, thresholds.wave_slope_rad + 0.05, 0.0, 0.0) == "wave"
