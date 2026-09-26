#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(dirname -- "$script_dir")"
cd -- "$workspace_dir"

# hexapod_description is now generated off-line from final1.SLDASM by
# src/hexapod_description/tools/gen.py (see its README), so there is no
# per-build URDF generation step any more.
# Override with COLCON_BUILD_BASE / COLCON_INSTALL_BASE to build into fresh
# directories (scripts/sim_smoke_test.sh does this automatically when the
# default build/ still holds a non-symlink build that --symlink-install
# cannot overwrite).
# ROS 2 Jazzy's Python modules (catkin_pkg, em/empy, rclpy, rosidl_*) exist
# only for the system interpreter /usr/bin/python3 (3.12). Another python3
# earlier on PATH (e.g. ~/.local/bin/python3.11) makes CMake pick it and the
# build fails with "No module named 'catkin_pkg'" / "'em'", so pin it.
export PATH="/usr/bin:$PATH"
colcon build --symlink-install \
  --build-base "${COLCON_BUILD_BASE:-build}" \
  --install-base "${COLCON_INSTALL_BASE:-install}" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_EXECUTABLE=/usr/bin/python3
