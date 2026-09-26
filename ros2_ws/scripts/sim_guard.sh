# shellcheck shell=bash
# Shared guard for the simulation scripts (sourced by sim_smoke_test.sh and run_e1.sh).
#
# Why: when `ros2 launch` stops Gazebo, the `gz sim` Ruby wrapper sometimes
# ignores SIGINT; launch then SIGTERMs the wrapper and the forked gz server /
# GUI survive as orphans - still running the old world, the old robot and its
# /controller_manager. The next test then talks to that stale robot:
#   * spawner: "Controller already loaded ... Failed to configure controller"
#   * sim_smoke_test: joint_states OK but no IMU/camera/contacts, base z = 0
#     (the survivor was E1's fixed-base, sensor-less robot)
# The guard therefore
#   1. kills leftovers of earlier runs (this user only) before starting,
#   2. gives every run its own GZ_PARTITION so a survivor cannot answer the
#      new run's gz services (spawn, clock, sensors),
#   3. refuses to start while a /controller_manager is still on the ROS graph,
#   4. kills everything the run started (process group + leftovers) at exit.
#
# Usage:   source "$script_dir/sim_guard.sh"; sim_guard_start "$ws" || exit 1
#          export GZ_PARTITION=...; setsid ros2 launch ... & ; sim_guard_stop "$!"

# 5. Gazebo transport on the loopback interface. Without GZ_IP gz-transport uses the first
#    network interface; when that goes down (Wi-Fi drop / power saving, night of 25-26 Sep
#    2026) Gazebo still executes `gz service` requests (create / remove / set_pose) but
#    cannot send the reply ("NodeShared::RecvSrvRequest() error sending response: Host
#    unreachable"), so the caller sees a timeout for a request that did happen. The whole
#    simulation runs on this machine, so loopback is enough. Export GZ_IP to override.
export GZ_IP="${GZ_IP:-127.0.0.1}"

# Command-line patterns of processes that belong to a simulation run.
_sim_guard_patterns() {
  local ws="$1"
  printf '%s\n' \
    'gz sim' \
    'gz-sim' \
    'ros2 launch hexapod_' \
    'ros_gz_bridge/parameter_bridge' \
    'ros_gz_sim/create' \
    'robot_state_publisher/robot_state_publisher' \
    'controller_manager/spawner' \
    "$ws/install"
}

# Kill leftover simulation processes of this user. $1 = workspace, $2 = label for the log.
sim_guard_kill_leftovers() {
  local ws="$1" label="${2:-before start}" pids="" pattern p
  while IFS= read -r pattern; do
    for p in $(pgrep -u "$(id -u)" -f -- "$pattern" 2>/dev/null); do
      # never the calling shell or its parents (the script itself matches "$ws/install" via source paths)
      [[ "$p" == "$$" || "$p" == "$PPID" || "$p" == "${BASHPID:-$$}" ]] && continue
      pids+=" $p"
    done
  done < <(_sim_guard_patterns "$ws")
  pids="$(tr ' ' '\n' <<< "$pids" | sed '/^$/d' | sort -un | paste -sd' ')"
  [[ -z "$pids" ]] && return 0
  echo "sim_guard ($label): stopping leftover simulation processes:"
  # shellcheck disable=SC2086
  ps -o pid=,etime=,args= -p ${pids// /,} 2>/dev/null | cut -c1-160 | sed 's/^/  /'
  # shellcheck disable=SC2086
  kill -INT $pids 2>/dev/null
  for _ in 1 2 3 4 5; do
    sleep 1
    # shellcheck disable=SC2086
    kill -0 $pids 2>/dev/null || break
  done
  # shellcheck disable=SC2086
  kill -KILL $pids 2>/dev/null
  sleep 0.5
  return 0
}

# Fail (return 1) while another /controller_manager is still visible on the ROS graph.
sim_guard_check_graph() {
  ros2 daemon stop > /dev/null 2>&1   # drop the daemon's cached view of dead nodes
  local nodes
  nodes="$(timeout 15 ros2 node list --no-daemon 2>/dev/null)"
  if grep -qx '/controller_manager' <<< "$nodes"; then
    echo "sim_guard: a /controller_manager is still running on ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} after cleanup."
    echo "           Close any other Gazebo/ROS session (other terminals, another user) and run again."
    echo "           Nodes seen:"; sed 's/^/             /' <<< "$nodes"
    return 1
  fi
  return 0
}

sim_guard_start() {
  local ws="$1"
  _SIM_GUARD_WS="$ws"
  sim_guard_kill_leftovers "$ws" "before start"
  sim_guard_check_graph
  # The caller exports GZ_PARTITION itself (this function usually runs inside a
  # "| tee" subshell, where an export would be lost).
}

# $1 = PID of a process-group leader started with setsid (optional).
sim_guard_stop() {
  local group="${1:-}"
  if [[ -n "$group" ]]; then
    kill -INT -- -"$group" 2>/dev/null
    for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 -- -"$group" 2>/dev/null || break; sleep 1; done
    kill -KILL -- -"$group" 2>/dev/null
  fi
  # orphans that left the process group (gz server/GUI re-parented to init)
  sim_guard_kill_leftovers "${_SIM_GUARD_WS:-$PWD}" "after run"
}
