#!/usr/bin/env bash
# E1 (IK/FK verification, chapter 4.1.1) in one command:
#   part A  e1_ik_offline  - IK accuracy/convergence/timing over the walking grid
#                            and the whole joint space, both solvers (no Gazebo)
#   part B  e1_fixed_base  - Gazebo, body bolted 0.30 m above the floor, all six
#                            legs driven through the grid, foot-tip error of
#                            IK + controller (the Gazebo window closes itself)
#
#   bash scripts/run_e1.sh --build            # colcon build first
#   bash scripts/run_e1.sh                    # use the current install
#   bash scripts/run_e1.sh --quick            # 5 grid poses in Gazebo, 200 samples offline
#   bash scripts/run_e1.sh --solver dls       # IK used for the Gazebo part (default analytic)
#
# Results: ros2_ws/results/e1/  (e1_offline_summary.md, e1_sim_summary.md, CSVs,
# run_e1_log.txt with everything this script printed)
set -uo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws="$(dirname -- "$script_dir")"
cd -- "$ws"

build=0; quick=0; solver="analytic"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) build=1 ;;
    --quick) quick=1 ;;
    --solver) solver="$2"; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

# Same environment rules as build.sh / sim_smoke_test.sh: system Python 3.12
# for ROS 2 Jazzy, and build_final1/install_final1 when build/ is the old
# non-symlink build.
export PATH="/usr/bin:$PATH"
build_base="build"; install_base="install"
stale="$ws/build/hexapod_interfaces/ament_cmake_python/hexapod_interfaces/hexapod_interfaces"
if [[ -d "$ws/build_final1" || ( -d "$stale" && ! -L "$stale" ) ]]; then
  build_base="build_final1"; install_base="install_final1"
fi
export COLCON_BUILD_BASE="$build_base" COLCON_INSTALL_BASE="$install_base"

out_dir="$ws/results/e1"
mkdir -p "$out_dir" "$ws/log"
log_file="$out_dir/run_e1_log.txt"
: > "$log_file"
start_marker="$ws/log/.run_e1_started"
touch "$start_marker"
log() { echo "$@" | tee -a "$log_file"; }
log "=== run_e1 $(date -Iseconds)  build=$build_base install=$install_base solver=$solver quick=$quick ==="

set +u; source /opt/ros/jazzy/setup.bash; set -u
if [[ $build -eq 1 ]]; then
  log "--- build ---"
  bash ./scripts/build.sh > "$ws/log/run_e1_build.log" 2>&1
  status=$?
  tail -n 15 "$ws/log/run_e1_build.log" | tee -a "$log_file"
  if [[ $status -ne 0 ]]; then log "BUILD FAILED (exit $status)"; exit 1; fi
fi
if [[ ! -f "$ws/$install_base/setup.bash" ]]; then log "no $install_base/setup.bash - run with --build"; exit 1; fi
set +u; source "$ws/$install_base/setup.bash"; set -u

samples=2000; max_targets=0
if [[ $quick -eq 1 ]]; then samples=200; max_targets=5; fi

log "--- part A: offline IK ($samples joint-space samples per leg) ---"
ros2 run hexapod_evaluation e1_ik_offline --out "$out_dir" --samples "$samples" 2>&1 | tee -a "$log_file"
status_a=${PIPESTATUS[0]}

log "--- part B: Gazebo fixed base (max_targets=$max_targets, 0 = all 100) ---"
launch_log="$ws/log/run_e1_launch.log"
# Clear leftovers of earlier runs (a surviving gz server + controller_manager
# answers this run's spawner - see sim_guard.sh) and isolate gz transport.
source "$script_dir/sim_guard.sh"
sim_guard_start "$ws" 2>&1 | tee -a "$log_file"
if [[ ${PIPESTATUS[0]} -ne 0 ]]; then log "ABORT: stale simulation still running"; exit 1; fi
export GZ_PARTITION="hexapod_$$_e1"   # exported here: the tee pipeline above ran sim_guard_start in a subshell
log "GZ_PARTITION=$GZ_PARTITION"
_SIM_GUARD_WS="$ws"
# ~1.8 s sim per pose x 100 poses + start-up; generous wall-clock limit for slow real-time factors.
# setsid: own process group, so gz server/GUI can be killed with the launch.
setsid timeout --signal=INT --kill-after=20 1800 \
  ros2 launch hexapod_evaluation e1_fixed_base.launch.py out_dir:="$out_dir" solver:="$solver" max_targets:="$max_targets" \
  > "$launch_log" 2>&1 &
launch_pid=$!
trap 'sim_guard_stop "$launch_pid" > /dev/null 2>&1' INT TERM
wait "$launch_pid"
status_b=$?
sim_guard_stop "$launch_pid" 2>&1 | tee -a "$log_file"
trap - INT TERM
if [[ -f "$out_dir/e1_sim_summary.md" && "$out_dir/e1_sim_summary.md" -nt "$start_marker" ]]; then
  tee -a "$log_file" < "$out_dir/e1_sim_summary.md"
else
  log "part B produced no new e1_sim_summary.md (launch exit $status_b)"
fi
log ""
log "--- launch log: errors / warnings (last 40 lines) ---"
grep -aiE "error|exception|traceback|fail|unable|could not" "$launch_log" | tail -n 40 | tee -a "$log_file"
log ""
log "results: $out_dir"
[[ $status_a -eq 0 && "$out_dir/e1_sim_summary.md" -nt "$start_marker" ]]
