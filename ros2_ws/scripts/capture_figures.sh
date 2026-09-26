#!/usr/bin/env bash
# Gazebo screenshots for the thesis in one command (~15 min, 66 images, several views per scene):
#   bash scripts/capture_figures.sh      -> results/figures_gazebo/*.png (1920 x 1080)
# Starts sim.launch.py (flat_world, no RViz, robot depth camera off and its box hidden), runs
# scripts/capture_figures.py and stops everything again (sim_guard.sh).
# The Gazebo window also opens - leave it alone while the script runs.
set -uo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws="$(dirname -- "$script_dir")"
cd -- "$ws"

export PATH="/usr/bin:$PATH"
build_base="build"; install_base="install"
stale="$ws/build/hexapod_interfaces/ament_cmake_python/hexapod_interfaces/hexapod_interfaces"
if [[ -d "$ws/build_final1" || ( -d "$stale" && ! -L "$stale" ) ]]; then build_base="build_final1"; install_base="install_final1"; fi
export COLCON_BUILD_BASE="$build_base" COLCON_INSTALL_BASE="$install_base"

out_dir="$ws/results/figures_gazebo"
mkdir -p "$out_dir" "$ws/log"
# images of an earlier capture go to previous/ (the file names change with the views)
if compgen -G "$out_dir/*.png" > /dev/null; then
  mkdir -p "$out_dir/previous" && mv -f "$out_dir"/*.png "$out_dir/previous/"
fi
log_file="$out_dir/capture_log.txt"
: > "$log_file"
log() { echo "$@" | tee -a "$log_file"; }
log "=== capture_figures $(date -Iseconds) install=$install_base ==="

set +u; source /opt/ros/jazzy/setup.bash; set -u
log "--- build ---"
if ! bash ./scripts/build.sh > "$ws/log/capture_figures_build.log" 2>&1; then
  tail -n 20 "$ws/log/capture_figures_build.log" | tee -a "$log_file"; log "BUILD FAILED"; exit 1
fi
set +u; source "$ws/$install_base/setup.bash"; set -u
source "$script_dir/sim_guard.sh"
sim_guard_start "$ws" 2>&1 | tee -a "$log_file"
if [[ ${PIPESTATUS[0]} -ne 0 ]]; then log "ABORT: stale simulation still running"; exit 1; fi
export GZ_PARTITION="hexapod_$$_fig"
_SIM_GUARD_WS="$ws"

launch_log="$ws/log/capture_figures_launch.log"
setsid ros2 launch hexapod_bringup sim.launch.py world:=flat_world.sdf use_rviz:=false use_camera:=false camera_visual:=false \
  > "$launch_log" 2>&1 &
launch_pid=$!
cleanup() { sim_guard_stop "$launch_pid" 2>&1 | tee -a "$log_file"; }
trap cleanup EXIT

PYTHONUNBUFFERED=1 python3 "$script_dir/capture_figures.py" --out "$out_dir" 2>&1 | tee -a "$log_file"
status=${PIPESTATUS[0]}
log "results: $out_dir (exit $status)"
exit "$status"
