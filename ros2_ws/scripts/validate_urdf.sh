#!/usr/bin/env bash
# Checks that hexapod_description's xacro expands cleanly for the argument
# sets the launch files use, and that check_urdf accepts the result (single
# tree rooted at base_link). Run before every commit that touches
# hexapod_description. Needs a sourced workspace (for $(find ...)).
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(dirname -- "$script_dir")"
pkg_dir="$workspace_dir/src/hexapod_description"
out="${TMPDIR:-/tmp}/hexapod_expanded"

xacro "$pkg_dir/urdf/hexapod.urdf.xacro" -o "$out.urdf"                       # mock hardware (default)
xacro "$pkg_dir/urdf/hexapod.urdf.xacro" use_sim:=true -o "$out.sim.urdf"     # Gazebo
xacro "$pkg_dir/urdf/hexapod.urdf.xacro" use_sim:=false use_ros2_control:=false -o "$out.display.urdf"
xacro "$pkg_dir/urdf/hexapod.urdf.xacro" use_sim:=true use_sensors:=false fixed_base:=true -o "$out.e1.urdf"  # E1
xacro "$pkg_dir/urdf/hexapod.urdf.xacro" use_sim:=true use_camera:=false -o "$out.e2.urdf"  # E2/E3 (no camera rendering)
grep -q 'type="force_torque"' "$out.sim.urdf" || { echo "joint force/torque sensors missing"; exit 1; }
check_urdf "$out.urdf"
check_urdf "$out.sim.urdf"

echo "validate_urdf.sh: OK"
