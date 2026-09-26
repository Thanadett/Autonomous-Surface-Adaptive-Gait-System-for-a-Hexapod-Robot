"""Gates any gait-selection source (rule-based today, a learned model once
P4 lands) behind hysteresis, a minimum dwell time, and a staleness/
confidence fallback -- see the project plan section 7's "every option
must pass a safety filter" requirement.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .rule_based import RuleThresholds, select_gait

VALID_GAITS = ("tripod", "ripple", "wave")


@dataclass
class SafetyFilterConfig:
    confidence_threshold: float = 0.7
    min_dwell_s: float = 1.9  # ~2 gait cycles at cycle_time=0.95s
    max_feature_age_s: float = 0.5
    high_tilt_variance_override: float = 0.05  # forces Wave regardless of source


@dataclass
class SafetyFilterResult:
    gait: str
    accepted_candidate: bool  # False if the candidate was overridden/rejected
    reason: str


class SafetyFilter:
    """Stateful: tracks how long the current gait has been active so a
    candidate switch can be rejected for dwelling too briefly.
    """

    def __init__(self, config: SafetyFilterConfig = SafetyFilterConfig(), initial_gait: str = "tripod") -> None:
        if initial_gait not in VALID_GAITS:
            raise ValueError(f"unknown gait '{initial_gait}'")
        self.config = config
        self.current_gait = initial_gait
        self.time_in_current_gait = 0.0

    def step(
        self,
        dt: float,
        candidate_gait: str,
        candidate_confidence: float,
        feature_age_s: float,
        tilt_variance: float,
        rule_based_fallback: "RuleBasedFallbackInputs | None" = None,
    ) -> SafetyFilterResult:
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("dt must be finite and non-negative")
        self.time_in_current_gait += dt

        # 1. High measured tilt variance overrides everything -- widen the
        #    base of support immediately regardless of what selected the
        #    candidate.
        if tilt_variance >= self.config.high_tilt_variance_override:
            return self._commit("wave", accepted=False, reason="tilt_variance_override")

        # 2. Stale input: don't trust a candidate computed from old
        #    features -- fall back to the rule-based baseline if terrain
        #    features are given, else just hold the current gait.
        if feature_age_s > self.config.max_feature_age_s:
            if rule_based_fallback is not None:
                fallback = select_gait(
                    rule_based_fallback.slope_pitch_rad,
                    rule_based_fallback.slope_roll_rad,
                    rule_based_fallback.roughness_m,
                    tilt_variance,
                )
                return self._commit(fallback, accepted=False, reason="stale_features_fallback")
            return self._commit(self.current_gait, accepted=False, reason="stale_features_hold")

        # 3. Low confidence: hold the current gait rather than switch on a
        #    weak signal.
        if candidate_confidence < self.config.confidence_threshold:
            return self._commit(self.current_gait, accepted=False, reason="low_confidence_hold")

        # 4. Minimum dwell time: even a confident candidate can't switch
        #    the gait again too soon after the last switch (hysteresis).
        if candidate_gait != self.current_gait and self.time_in_current_gait < self.config.min_dwell_s:
            return self._commit(self.current_gait, accepted=False, reason="min_dwell_hold")

        if candidate_gait not in VALID_GAITS:
            raise ValueError(f"unknown gait '{candidate_gait}'")
        return self._commit(candidate_gait, accepted=True, reason="accepted")

    def _commit(self, gait: str, *, accepted: bool, reason: str) -> SafetyFilterResult:
        if gait != self.current_gait:
            self.current_gait = gait
            self.time_in_current_gait = 0.0
        return SafetyFilterResult(gait=gait, accepted_candidate=accepted, reason=reason)


@dataclass
class RuleBasedFallbackInputs:
    slope_pitch_rad: float
    slope_roll_rad: float
    roughness_m: float
