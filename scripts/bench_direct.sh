#!/usr/bin/env bash
# Direct OMPL + Pinocchio/coal pipeline vs MoveIt 2 on the same queries, shelf boxes, planner
# settings and controller: N queries x {direct, moveit} x RRTConnect x unconstrained x 1 rep.
#
#   scripts/bench_direct.sh [N] [OUT_CSV] [--plan-only | --octomap]
#   defaults: 100, results/bench_direct.csv, with execution in Gazebo
#
# --plan-only needs no Gazebo (CI mini-benchmark): plan, parameterise and cross-validate only.
# --octomap (SHOULD) starts the sim for move_group's camera octomap and plans both pipelines on
#   that one frozen map (no boxes, no execution).
# With execution the sim (v0.1 sim.launch.py) is started and torn down by scripts/with_sim.sh.
# Timings are published, so on the shared machine run it alone, under all three locks:
#   flock ~/work/.locks/heavy.lock flock ~/work/.locks/build.lock \
#     flock ~/work/.locks/gazebo.lock scripts/bench_direct.sh
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO}"
N="${1:-100}"
OUT="${2:-results/bench_direct.csv}"
MODE="${3:-}"
QUERIES="results/queries_boxes.json"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}" GZ_PARTITION="${GZ_PARTITION:-r2}"
GIT=(git -c safe.directory="${REPO}")  # CI runs as root on a runner-owned checkout
COMMIT="$("${GIT[@]}" rev-parse --short HEAD)"
if ! "${GIT[@]}" diff --quiet HEAD -- src "${QUERIES}"; then
  COMMIT="${COMMIT}-dirty"
fi
mkdir -p "$(dirname "${OUT}")"
# Write <OUT>.partial and rename at the end, so a git operation on the tracked CSV can never swap
# the file under the running node (it happened once in v0.1; NOTES.md).
PARTIAL="${REPO}/${OUT}.partial"
rm -f "${PARTIAL}" "${PARTIAL}.meta.json"
# PCP_CONSTRAINT=orientation plans both pipelines with v0.1's keep-upright constraint (SHOULD).
# PCP_PROBE_QUERIES=k first solves k queries in MoveIt's own planning context with a counting
# validity checker, and with the direct planner (results in the .meta.json only).
LAUNCH=(ros2 launch pcp_direct bench_direct.launch.py queries_file:="${REPO}/${QUERIES}"
  output_csv:="${PARTIAL}" commit:="${COMMIT}" max_queries:="${N}"
  probe_queries:="${PCP_PROBE_QUERIES:-0}" constraint:="${PCP_CONSTRAINT:-none}")
start=$(date +%s)
if [ "${MODE}" = "--plan-only" ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/lyrical/setup.bash
  # shellcheck disable=SC1091
  source "${REPO}/install/setup.bash"
  set -u
  timeout "${PCP_BENCH_TIMEOUT:-1800}" "${LAUNCH[@]}" execute:=false
elif [ "${MODE}" = "--octomap" ]; then
  timeout "${PCP_BENCH_TIMEOUT:-900}" "${REPO}/scripts/with_sim.sh" "${LAUNCH[@]}" execute:=false \
    scene:=octomap
else
  timeout "${PCP_BENCH_TIMEOUT:-6000}" "${REPO}/scripts/with_sim.sh" "${LAUNCH[@]}" execute:=true
fi
if [ ! -f "${PARTIAL}.meta.json" ]; then
  echo "[bench_direct] benchmark did not finish (no ${PARTIAL}.meta.json); ${OUT} left unchanged" >&2
  exit 1
fi
mv "${PARTIAL}" "${REPO}/${OUT}"
mv "${PARTIAL}.meta.json" "${REPO}/${OUT}.meta.json"
rows=$(($(wc -l <"${OUT}") - 1))
echo "[bench_direct] ${rows} rows in ${OUT} (commit ${COMMIT}) in $(( $(date +%s) - start )) s"
