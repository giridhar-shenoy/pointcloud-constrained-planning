#!/usr/bin/env bash
# Run a command against a live simulation and always tear the simulation down afterwards.
#
#   scripts/with_sim.sh <command> [args...]
#
# Starts `ros2 launch pcp_bringup sim.launch.py ${PCP_SIM_ARGS}` in its own session/process
# group, runs the command, then sends SIGINT to the whole group, escalates to SIGKILL after
# 15 s, and verifies that no process of that group (gz sim included) is left. Exit status is
# the command's. Locally, wrap the call in the Gazebo lock:
#   flock ~/work/.locks/gazebo.lock timeout 600 scripts/with_sim.sh <command>
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="${PCP_LOG_DIR:-${REPO}/log/sim}"
mkdir -p "${LOG_DIR}"
SIM_LOG="${LOG_DIR}/sim_$(date +%Y%m%d_%H%M%S).log"
SIM_PGID=""

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source "${REPO}/install/setup.bash"
set -u

# One Gazebo at a time on this machine (MACHINE.md): wait for any other gz sim to exit first.
# "[b]in/gz sim" matches the real server ("ruby .../bin/gz sim ...") but not shells whose command
# text merely mentions gz sim (a plain "gz sim" pattern matched this script's own parent shell).
waited=0
while pgrep -f "[b]in/gz sim" >/dev/null 2>&1; do
  if [ "${waited}" -ge "${PCP_GZ_WAIT:-1800}" ]; then
    echo "[with_sim] another gz sim is still running after ${waited} s; giving up" >&2
    exit 3
  fi
  [ "${waited}" -eq 0 ] && echo "[with_sim] another gz sim is running; waiting for it to exit"
  sleep 10
  waited=$((waited + 10))
done

# shellcheck disable=SC2086
setsid ros2 launch pcp_bringup sim.launch.py ${PCP_SIM_ARGS:-} >"${SIM_LOG}" 2>&1 &
SIM_PGID=$!
echo "[with_sim] sim pgid=${SIM_PGID}, log=${SIM_LOG}"

cleanup() {
  local rc=$?
  trap - EXIT INT TERM
  if [ -z "${SIM_PGID:-}" ]; then
    exit "${rc}"
  fi
  if kill -0 -- "-${SIM_PGID}" 2>/dev/null; then
    kill -INT -- "-${SIM_PGID}" 2>/dev/null || true
    for _ in $(seq 1 15); do
      kill -0 -- "-${SIM_PGID}" 2>/dev/null || break
      sleep 1
    done
    kill -KILL -- "-${SIM_PGID}" 2>/dev/null || true
    sleep 1
  fi
  if pgrep -g "${SIM_PGID}" >/dev/null 2>&1; then
    echo "[with_sim] ERROR: processes of group ${SIM_PGID} still alive:" >&2
    pgrep -a -g "${SIM_PGID}" >&2 || true
    rc=1
  else
    echo "[with_sim] simulation group ${SIM_PGID} fully stopped"
  fi
  exit "${rc}"
}
trap cleanup EXIT INT TERM

"$@"
