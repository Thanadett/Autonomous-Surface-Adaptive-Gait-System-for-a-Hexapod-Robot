#!/usr/bin/env bash
# Overnight batch after E2/E2b: E3 calibration, E3, E4, E5 - one command, unattended.
# The E6 dataset is recorded on the way (RGB + depth snapshots of every E3-E5 trial).
#
#   bash scripts/run_all.sh                 # build, check every plan, then run the full plans
#   bash scripts/run_all.sh --plans "e4 e5" # a subset
#   bash scripts/run_all.sh --no-check      # skip the ~15 min check stage
#   bash scripts/run_all.sh                 # again after an interruption: every full run resumes
#
# Stages (each plan in its own Gazebo session, started and cleaned up by run_exp.sh):
#   1. colcon build (once)
#   2. check: every plan with --check (1 short trial of its first and last condition)
#      into results/<plan>_check/; a plan whose check fails is NOT run in stage 3
#   3. full: every checked plan with --resume into results/<plan>/ - an interrupted night
#      continues where it stopped when the script is started again
# Rough duration with the real-time factor ~1: check ~15 min, e3_calib ~5 min, E3 ~2.5 h,
# E4 ~2.5 h, E5 ~3 h. Do not start another Gazebo meanwhile: sim_guard.sh stops leftovers.
# Log: results/run_all_log.txt; status table: results/run_all_status.md
set -uo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws="$(dirname -- "$script_dir")"
cd -- "$ws"

plans="e3_calib e3 e4 e5"
do_check=1
while [[ $# -gt 0 ]]; do
  case "$1" in
    --plans) plans="$2"; shift ;;
    --no-check) do_check=0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

# Same environment as run_e2.sh / run_exp.sh: system Python 3.12 for ROS 2 Jazzy, and
# build_final1/install_final1 when build/ is the old non-symlink build (colcon
# --symlink-install cannot overwrite it: "existing path cannot be removed: Is a directory").
# build.sh reads COLCON_BUILD_BASE/COLCON_INSTALL_BASE, run_exp.sh picks the same pair.
export PATH="/usr/bin:$PATH"
build_base="build"; install_base="install"
stale="$ws/build/hexapod_interfaces/ament_cmake_python/hexapod_interfaces/hexapod_interfaces"
if [[ -d "$ws/build_final1" || ( -d "$stale" && ! -L "$stale" ) ]]; then
  build_base="build_final1"; install_base="install_final1"
fi
export COLCON_BUILD_BASE="$build_base" COLCON_INSTALL_BASE="$install_base"
set +u; source /opt/ros/jazzy/setup.bash; set -u

mkdir -p results log
log_file="$ws/results/run_all_log.txt"
status_file="$ws/results/run_all_status.md"
log() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$log_file"; }
camera_for() { [[ "$1" == e3_calib ]] && echo "" || echo "--camera"; }

log "=== run_all $(date -Iseconds) plans: $plans check=$do_check ==="
{ echo "# run_all status ($(date '+%Y-%m-%d %H:%M'))"; echo; echo "| plan | check | full run | results |"; echo "|---|---|---|---|"; } > "$status_file"

log "--- build ($build_base / $install_base) ---"
if ! bash ./scripts/build.sh > "$ws/log/run_all_build.log" 2>&1; then
  tail -n 30 "$ws/log/run_all_build.log" | tee -a "$log_file"
  log "BUILD FAILED - nothing run"; echo "| (build) | FAILED | - | log/run_all_build.log |" >> "$status_file"; exit 1
fi
log "build OK"

declare -A check_status
for plan in $plans; do
  if [[ $do_check -eq 0 || "$plan" == e3_calib ]]; then check_status[$plan]="skipped"; continue; fi
  log "--- check $plan ---"
  if bash ./scripts/run_exp.sh --plan "$plan" $(camera_for "$plan") --check --out-suffix _check >> "$log_file" 2>&1; then
    check_status[$plan]="passed"; log "check $plan PASSED"
  else
    check_status[$plan]="FAILED"; log "check $plan FAILED - see results/${plan}_check/run_e2_log.txt"
  fi
done

for plan in $plans; do
  if [[ "${check_status[$plan]}" == FAILED ]]; then
    echo "| $plan | FAILED | not run | results/${plan}_check/ |" >> "$status_file"; continue
  fi
  log "--- full $plan ---"
  start=$(date +%s)
  if bash ./scripts/run_exp.sh --plan "$plan" $(camera_for "$plan") --resume >> "$log_file" 2>&1; then
    result="done"
  else
    result="exit $?"
  fi
  minutes=$(( ($(date +%s) - start) / 60 ))
  log "full $plan: $result after $minutes min"
  echo "| $plan | ${check_status[$plan]} | $result ($minutes min) | results/$plan/ |" >> "$status_file"
done
log "=== run_all finished ==="
cat "$status_file" | tee -a "$log_file"
