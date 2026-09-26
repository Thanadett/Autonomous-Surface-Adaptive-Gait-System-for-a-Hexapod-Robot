"""Rule-based gait selection ("Option 0" in the project plan's model table).

This is not a placeholder: the plan calls for this as the permanent
fallback AUTO uses whenever the learned model's input is stale or its
confidence is too low (see safety_filter.py), and as the baseline every
learned option (A-D) must beat in evaluation. It needs no training data
and is what AUTO runs on until a model is trained in P4.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RuleThresholds:
    # Placeholders -- tune against the terrain curriculum once P3's
    # dataset exists; see the project plan's open-questions table (#7).
    wave_slope_rad: float = 0.26  # ~15 deg: favor Wave beyond this
    wave_roughness_m: float = 0.02
    ripple_slope_rad: float = 0.12  # ~7 deg: favor Ripple beyond this
    ripple_roughness_m: float = 0.01
    high_tilt_variance: float = 0.05  # rad^2, forces Wave regardless of slope


def select_gait(
    slope_pitch_rad: float,
    slope_roll_rad: float,
    roughness_m: float,
    tilt_variance: float,
    thresholds: RuleThresholds = RuleThresholds(),
) -> str:
    """Pure function: terrain features -> one of "tripod" | "ripple" | "wave".

    Intentionally simple and legible (no hysteresis, no dwell time here --
    that belongs to the safety filter, which wraps this and any future
    learned model identically) so it stays trustworthy as the fallback.
    """
    slope = max(abs(slope_pitch_rad), abs(slope_roll_rad))

    if tilt_variance >= thresholds.high_tilt_variance:
        return "wave"
    if slope >= thresholds.wave_slope_rad or roughness_m >= thresholds.wave_roughness_m:
        return "wave"
    if slope >= thresholds.ripple_slope_rad or roughness_m >= thresholds.ripple_roughness_m:
        return "ripple"
    return "tripod"
