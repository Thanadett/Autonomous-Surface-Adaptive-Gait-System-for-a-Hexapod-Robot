#!/usr/bin/env bash
# One experiment plan (E2, E2b, E3 calibration, E3, E4, E5) in one command - run_e2.sh
# plus terrain/camera/check/resume options (a separate file so that a run_e2.sh that is
# still executing is never modified underneath bash). scripts/run_all.sh calls this.
# (Header below kept from run_e2.sh:)
# E2 (basic gaits on flat ground, chapter 4.1.2) in one command:
#   sim.launch.py (flat_world, no RViz, no depth-camera rendering) + e2_runner, which
#   runs every trial in the same simulation (teleport to the start between trials) and
#   writes the tables. Plan: src/hexapod_evaluation/config/e2.yaml.
#
#   bash scripts/run_exp.sh --build               # colcon build first
#   bash scripts/run_exp.sh --quick               # 2 trials x 0.5 m per condition (~5-10 min)
#   bash scripts/run_exp.sh                       # full plan: 5 conditions x 10 trials x 2 m (~1-1.5 h)
#   bash scripts/run_exp.sh --plan e2b            # E2b: body height x step height (config/e2b.yaml, ~2 h)
#   bash scripts/run_exp.sh --plan e3 --camera    # terrain plans record camera snapshots (E6 dataset)
#   bash scripts/run_exp.sh --plan e4 --check     # 1 short trial of the first/last condition
#   bash scripts/run_exp.sh --plan e4 --resume    # continue an interrupted run
#   bash scripts/run_exp.sh -- --gaits wave --n 3 --bag   # anything after -- goes to e2_runner
#
# Results: ros2_ws/results/<plan>/ (e2_summary.md, e2_trials.csv, e2_meta.json, samples/, run_e2_log.txt);
# full launch output: ros2_ws/log/run_e2_launch.log
set -uo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws="$(dirname -- "$script_dir")"
cd -- "$ws"

build=0; quick=0; plan="e2"; camera="false"; mode_args=(); runner_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) build=1 ;;
    --quick) quick=1 ;;
    --plan) plan="$2"; shift ;;
    --camera) camera="true" ;;
    --check) mode_args+=(--check) ;;
    --resume) mode_args+=(--resume) ;;
    --out-suffix) out_suffix="$2"; shift ;;
    --) shift; runner_args=("$@"); break ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

# Same environment rules as build.sh / run_e1.sh: system Python 3.12 for ROS 2 Jazzy,
# build_final1/install_final1 when build/ is the old non-symlink build.
export PATH="/usr/bin:$PATH"
build_base="build"; install_base="install"
stale="$ws/build/hexapod_interfaces/ament_cmake_python/hexapod_interfaces/hexapod_interfaces"
if [[ -d "$ws/build_final1" || ( -d "$stale" && ! -L "$stale" ) ]]; then
  build_base="build_final1"; install_base="install_final1"
fi
export COLCON_BUILD_BASE="$build_base" COLCON_INSTALL_BASE="$install_base"

out_dir="$ws/results/$plan${out_suffix:-}"
mkdir -p "$out_dir" "$ws/log"
log_file="$out_dir/run_e2_log.txt"
[[ " ${mode_args[*]:-} " == *" --resume "* ]] || : > "$log_file"
log() { echo "$@" | tee -a "$log_file"; }
log "=== run_e2 $(date -Iseconds)  plan=$plan camera=$camera mode=${mode_args[*]:-full} build=$build_base install=$install_base quick=$quick args=${runner_args[*]:-} ==="

set +u; source /opt/ros/jazzy/setup.bash; set -u
if [[ $build -eq 1 ]]; then
  log "--- build ---"
  bash ./scripts/build.sh > "$ws/log/run_e2_build.log" 2>&1
  status=$?
  tail -n 15 "$ws/log/run_e2_build.log" | tee -a "$log_file"
  if [[ $status -ne 0 ]]; then log "BUILD FAILED (exit $status)"; exit 1; fi
fi
if [[ ! -f "$ws/$install_base/setup.bash" ]]; then log "no $install_base/setup.bash - run with --build"; exit 1; fi
set +u; source "$ws/$install_base/setup.bash"; set -u

# Leftover Gazebo/controller_manager from an earlier run would answer this run (sim_guard.sh)
source "$script_dir/sim_guard.sh"
sim_guard_start "$ws" 2>&1 | tee -a "$log_file"
if [[ ${PIPESTATUS[0]} -ne 0 ]]; then log "ABORT: stale simulation still running"; exit 1; fi
export GZ_PARTITION="hexapod_$$_e2"   # e2_runner's `gz service` teleport inherits it
_SIM_GUARD_WS="$ws"
log "GZ_PARTITION=$GZ_PARTITION"

launch_log="$ws/log/run_e2_launch.log"
setsid ros2 launch hexapod_bringup sim.launch.py world:=flat_world.sdf use_rviz:=false use_camera:="$camera" \
  > "$launch_log" 2>&1 &
launch_pid=$!
cleanup() { sim_guard_stop "$launch_pid" 2>&1 | tee -a "$log_file"; }
trap cleanup EXIT

extra=(--config "$(ros2 pkg prefix --share hexapod_evaluation)/config/$plan.yaml")
[[ $quick -eq 1 ]] && extra+=(--quick)
extra+=("${mode_args[@]}")
log "--- e2_runner ${extra[*]:-} ${runner_args[*]:-} ---"
PYTHONUNBUFFERED=1 ros2 run hexapod_evaluation e2_runner \
  --out "$out_dir" "${extra[@]}" "${runner_args[@]}" 2>&1 | tee -a "$log_file"
status=${PIPESTATUS[0]}

log ""
log "--- launch log: errors / warnings (last 40 lines) ---"
grep -aiE "error|exception|traceback|fail|unable|could not" "$launch_log" | grep -v "gz_frame_id" | tail -n 40 | tee -a "$log_file"
log ""
log "results: $out_dir (exit $status)"
exit "$status"
