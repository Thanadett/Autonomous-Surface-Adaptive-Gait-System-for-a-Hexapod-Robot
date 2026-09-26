#!/usr/bin/env bash
# One-command check of the full simulation stack after a URDF / launch change.
#
#   bash scripts/sim_smoke_test.sh --build        # colcon build first, then test
#   bash scripts/sim_smoke_test.sh                # test the current install/
#   bash scripts/sim_smoke_test.sh --world obstacle_course.sdf
#
# Steps: (optional) scripts/build.sh -> scripts/validate_urdf.sh ->
# ros2 launch hexapod_bringup sim.launch.py use_rviz:=false (Gazebo GUI still
# opens) -> scripts/sim_smoke_test.py -> shut the launch down.
# Everything is written to ros2_ws/sim_smoke_test_result.txt (the full launch
# output goes to ros2_ws/log/sim_smoke_test_launch.log).
set -uo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ws="$(dirname -- "$script_dir")"
cd -- "$ws"

build=0
world="flat_world.sdf"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --build) build=1 ;;
    --world) world="$2"; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

# A build/ produced earlier WITHOUT --symlink-install makes every later
# --symlink-install build fail in hexapod_interfaces ("failed to create
# symbolic link ... Is a directory"). Instead of deleting it, build into a
# separate build_final1/ + install_final1/ pair and use that from then on.
build_base="build"
install_base="install"
stale="$ws/build/hexapod_interfaces/ament_cmake_python/hexapod_interfaces/hexapod_interfaces"
if [[ -d "$ws/build_final1" || ( -d "$stale" && ! -L "$stale" ) ]]; then
  build_base="build_final1"
  install_base="install_final1"
fi
export COLCON_BUILD_BASE="$build_base" COLCON_INSTALL_BASE="$install_base"

# Same reason as in build.sh: ROS 2 Jazzy nodes (including scripts with a
# "#!/usr/bin/env python3" shebang) must run on the system Python 3.12.
user_python="$(command -v python3 || true)"
export PATH="/usr/bin:$PATH"

out="$ws/sim_smoke_test_result.txt"
mkdir -p "$ws/log"
launch_log="$ws/log/sim_smoke_test_launch.log"
: > "$out"
log() { echo "$@" | tee -a "$out"; }

log "=== sim_smoke_test $(date -Iseconds)  world=$world  build=$build_base install=$install_base ==="
log "python3 on your PATH: $user_python ($("$user_python" --version 2>&1)); using /usr/bin/python3 ($(/usr/bin/python3 --version 2>&1))"
set +u
source /opt/ros/jazzy/setup.bash
set -u

if [[ $build -eq 1 ]]; then
  log "--- build ---"
  bash ./scripts/build.sh > "$ws/log/sim_smoke_test_build.log" 2>&1
  status=$?
  tail -n 25 "$ws/log/sim_smoke_test_build.log" | tee -a "$out"
  if [[ $status -ne 0 ]]; then log "BUILD FAILED (exit $status)"; exit 1; fi
fi

set +u
if [[ ! -f "$ws/$install_base/setup.bash" ]]; then
  log "no $install_base/setup.bash - run with --build first"; exit 1
fi
source "$ws/$install_base/setup.bash"
set -u

log "--- validate_urdf ---"
bash ./scripts/validate_urdf.sh 2>&1 | tail -n 15 | tee -a "$out"

log "--- simulation ---"
# Leftover Gazebo/controller_manager from an earlier run would answer this
# run's spawner and topics (see sim_guard.sh) - clear them and isolate gz.
source "$script_dir/sim_guard.sh"
sim_guard_start "$ws" 2>&1 | tee -a "$out"
if [[ ${PIPESTATUS[0]} -ne 0 ]]; then log "ABORT: stale simulation still running"; exit 1; fi
export GZ_PARTITION="hexapod_$$_smoke"   # exported here: the tee pipeline above ran sim_guard_start in a subshell
log "GZ_PARTITION=$GZ_PARTITION"
# setsid: own process group, so the whole launch tree (gz sim, bridge,
# controller spawners, python nodes) can be stopped together afterwards.
setsid ros2 launch hexapod_bringup sim.launch.py world:="$world" use_rviz:=false > "$launch_log" 2>&1 &
launch_pid=$!
_SIM_GUARD_WS="$ws"
cleanup() { sim_guard_stop "$launch_pid" 2>&1 | tee -a "$out"; }
trap cleanup EXIT

/usr/bin/python3 "$script_dir/sim_smoke_test.py" 2>&1 | tee -a "$out"
status=${PIPESTATUS[0]}

log ""
log "--- launch log: errors / warnings (last 60 lines) ---"
grep -aiE "error|exception|traceback|warn|fail|unable|could not" "$launch_log" | tail -n 60 | tee -a "$out"
log ""
log "result: $out"
exit "$status"
