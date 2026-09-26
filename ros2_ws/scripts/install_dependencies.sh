#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(dirname -- "$script_dir")"
cd -- "$workspace_dir"
source /opt/ros/jazzy/setup.bash
set -u
# Pure-Python packages declare the ament_python build type, which is not a
# rosdep system key on this ROS installation.
rosdep install --from-paths src --ignore-src -r -y --skip-keys ament_python
