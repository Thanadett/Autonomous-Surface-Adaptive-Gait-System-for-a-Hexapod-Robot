# 49x_Hexapod

ROS 2 Jazzy + Gazebo Harmonic hexapod workspace with real-time adaptive
gait selection (Tripod / Ripple / Wave) driven by Depth Camera + IMU +
per-leg limit-switch fusion. This is a from-scratch rewrite of the earlier
`49x_Hexapod_test` prototype -- see the project plan doc for the full
design and roadmap. The robot model was replaced on 2026-09-25 by the
`final1.SLDASM` package (see `src/hexapod_description/README.md`); the
previous SolidWorks draft-1 model is kept in `../backup_before_final1_urdf/`.

## Status (P0/P1 of the roadmap)

**Working and tested now:**
- `hexapod_description`: generated from `final1.SLDASM` (STEP AP214 ->
  `tools/gen.py`): identical legs (coxa 56.85 / femur 80.00 / tibia
  130.03 mm), convex-hull collision meshes, sphere feet, masses from CAD
  volume x PETG fill factor (placeholder). Joint zero = coxa radial, femur
  horizontal, tibia straight down; femur + lifts the knee (opposite sign to
  the old draft-1 model). IMU, depth camera and six foot-contact sensors
  are wired in (placeholder poses/noise). Leg/joint/topic names follow the
  workspace interface listed in its README.
- `hexapod_interfaces`: the shared messages/services other packages build on.
- `hexapod_kinematics`: numerical FK/IK for the six 3-DOF CAD leg chains
  (9 unit tests).
- `hexapod_locomotion`: the tripod/ripple/wave planner, now with
  **phase-continuous gait switching** (`gait_transition.py`) instead of
  the old prototype's hard phase reset, which produced a visible foot
  jump every time the gait changed (15 unit tests, including one that
  directly checks a gait switch mid-stride does not jump).
- `hexapod_gait_selector`: the rule-based baseline + safety filter
  (hysteresis, minimum dwell time, confidence gate, staleness fallback)
  that AUTO mode runs on until a learned model exists (14 unit tests).
- `hexapod_bringup` + `hexapod_simulation`: launch `ros2 launch
  hexapod_bringup sim.launch.py` to walk the CAD model in Gazebo.

**Package skeletons only, explicitly not implemented (see each
package's README.md):**
- `hexapod_perception` (P2: depth camera -> terrain features)
- `hexapod_state_estimation` (P2: IMU/contact fusion -> terrain features)
- `hexapod_hardware` (P6: real servo/limit-switch SystemInterface plugin)
- the learned gait-selection model itself (P4: trained on P3's dataset,
  not yet collected)

## Build and test

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
./scripts/install_dependencies.sh
./scripts/build.sh
source install/setup.bash
./scripts/test.sh
./scripts/validate_urdf.sh
```

Whole-stack check (build, launch Gazebo, stand, walk tripod + wave, sensor
rates); writes `sim_smoke_test_result.txt`:

```bash
bash scripts/sim_smoke_test.sh --build
```

Gazebo's ros2_control bridge and joint trajectory controller (same as
the prototype workspace):

```bash
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-joint-trajectory-controller ros-jazzy-ros-gz-interfaces
```

## Walk in Gazebo

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hexapod_bringup sim.launch.py
```

Send a slow test command once "CAD gait ready" and the controllers are
active:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.03}}' -r 10
```

Switch gaits (blends smoothly over one gait cycle -- see
`hexapod_locomotion/hexapod_locomotion/gait_transition.py`):

```bash
ros2 service call /set_gait_mode hexapod_interfaces/srv/SetGaitMode "{mode: 'MANUAL:wave'}"
```

Or leave it on `AUTO` (the default) and let `hexapod_gait_selector`'s
rule-based baseline choose -- though until `hexapod_perception` and
`hexapod_state_estimation` (P2) exist, nothing publishes
`/terrain_features` yet, so AUTO currently has no real input to react to.

## Known CAD quirk

`front_right_tibia_link`'s frame is rotated 180 deg about X relative to
the other five tibia links -- an artifact of the SolidWorks export, not
a bug in this workspace's code. It's compensated for explicitly in
`hexapod_description/urdf/feet.xacro` and documented in
`hexapod_description/meshes/README.md`. Re-verify in RViz
(`ros2 launch hexapod_description display.launch.py`) before trusting
any new geometry added under that link, and before mapping servo
direction for that joint on real hardware.
