"""AUTO gait selection: rule-based today, ONNX inference is a documented
follow-up (see README.md P4). Subscribes hexapod_interfaces/TerrainFeatures
(from hexapod_state_estimation, P2 -- not implemented yet), applies the
safety filter, and publishes hexapod_interfaces/GaitSelection for
hexapod_locomotion's AUTO mode to consume.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node

from hexapod_interfaces.msg import GaitSelection, TerrainFeatures

from .rule_based import select_gait
from .safety_filter import RuleBasedFallbackInputs, SafetyFilter, SafetyFilterConfig

GAIT_CODES = {"tripod": GaitSelection.TRIPOD, "ripple": GaitSelection.RIPPLE, "wave": GaitSelection.WAVE}


class GaitSelectorNode(Node):
    def __init__(self) -> None:
        super().__init__("gait_selector_node")
        self.declare_parameter("min_dwell_s", 1.9)
        self.declare_parameter("confidence_threshold", 0.7)
        self.declare_parameter("max_feature_age_s", 0.5)

        config = SafetyFilterConfig(
            confidence_threshold=float(self.get_parameter("confidence_threshold").value),
            min_dwell_s=float(self.get_parameter("min_dwell_s").value),
            max_feature_age_s=float(self.get_parameter("max_feature_age_s").value),
        )
        self.filter = SafetyFilter(config)
        self.last_features: TerrainFeatures | None = None
        self.last_update_time = self.get_clock().now()

        self.publisher = self.create_publisher(GaitSelection, "/gait_selection", 10)
        self.create_subscription(TerrainFeatures, "/terrain_features", self.on_terrain_features, 10)
        self.get_logger().info(
            "gait_selector_node started (rule-based; ONNX inference not yet wired -- see README.md)"
        )

    def on_terrain_features(self, message: TerrainFeatures) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now
        if dt < 0:
            dt = 0.0

        candidate = select_gait(
            message.slope_pitch_rad,
            message.slope_roll_rad,
            message.roughness_m,
            message.tilt_variance,
        )
        stamp = message.header.stamp
        age_s = max(0.0, (now.nanoseconds - (stamp.sec * 10**9 + stamp.nanosec)) * 1e-9)

        result = self.filter.step(
            dt=dt,
            candidate_gait=candidate,
            # The rule-based source has no learned confidence score; treat
            # a valid, fresh feature vector as fully confident so the
            # safety filter's dwell/hysteresis logic still applies. A
            # learned model (P4) reports its own softmax confidence here.
            candidate_confidence=1.0 if message.valid else 0.0,
            feature_age_s=age_s,
            tilt_variance=message.tilt_variance,
            rule_based_fallback=RuleBasedFallbackInputs(
                message.slope_pitch_rad, message.slope_roll_rad, message.roughness_m
            ),
        )

        output = GaitSelection()
        output.header.stamp = now.to_msg()
        output.gait = GAIT_CODES[result.gait]
        output.probabilities = [0.0, 0.0, 0.0]
        output.probabilities[output.gait] = 1.0
        output.confidence = 1.0 if result.accepted_candidate else 0.5
        output.model_version = "rule_based_v0"
        output.inference_ms = 0.0
        output.safety_override = not result.accepted_candidate
        output.source = "rule_based"
        self.publisher.publish(output)


def main() -> None:
    rclpy.init()
    node = GaitSelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
