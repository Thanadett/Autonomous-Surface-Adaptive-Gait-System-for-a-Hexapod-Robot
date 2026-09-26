# hexapod_gait_selector

**Working now:** `rule_based.select_gait()` (Option 0 from the project
plan's model table) and `SafetyFilter` (hysteresis, minimum dwell time,
confidence gate, staleness fallback, tilt-variance override) are real,
tested logic -- see `test/`. `gait_selector_node` wires them together:
it subscribes `/terrain_features` (`hexapod_interfaces/TerrainFeatures`)
and publishes `/gait_selection` (`hexapod_interfaces/GaitSelection`) for
`hexapod_locomotion`'s AUTO mode.

**Not yet implemented (P4):** the learned model itself (Option A/B in the
plan -- a tabular or temporal classifier trained on P3's dataset,
exported to ONNX, loaded here via ONNX Runtime). Until then,
`gait_selection_node` always sets `source: "rule_based"` and
`model_version: "rule_based_v0"`. Swapping in the trained model means:
load the `.onnx` file, run inference instead of (or alongside, for
comparison) `select_gait()`, and pass the model's own softmax output as
`candidate_confidence` into `SafetyFilter.step()` -- the filter itself
does not need to change.

**Also blocked on P2:** nothing publishes `/terrain_features` yet
(`hexapod_perception` + `hexapod_state_estimation` are still stubs), so
this node currently has no real input to react to.
