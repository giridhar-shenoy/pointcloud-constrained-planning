#!/usr/bin/env bash
# Planner benchmark: RRTConnect x {none, orientation} x constrained state space x 1 rep, 10 s
# per cell, on the 100 seeded shelf queries, against a live octomap from the simulated camera.
#
#   scripts/bench_planners.sh [N_QUERIES] [OUT_CSV]       (defaults: 100, results/bench.csv)
#
# Locally (shared machine): flock ~/work/.locks/gazebo.lock scripts/bench_planners.sh
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO}"
N="${1:-100}"
OUT="${2:-results/bench.csv}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}" GZ_PARTITION="${GZ_PARTITION:-r2}"
GIT=(git -c safe.directory="${REPO}")  # CI runs as root on a runner-owned checkout
COMMIT="$("${GIT[@]}" rev-parse --short HEAD)"
if ! "${GIT[@]}" diff --quiet HEAD -- src results/queries.json; then
  COMMIT="${COMMIT}-dirty"
fi
mkdir -p "$(dirname "${OUT}")"
start=$(date +%s)
"${REPO}/scripts/with_sim.sh" ros2 launch pcp_benchmark benchmark.launch.py \
  queries_file:="${REPO}/results/queries.json" output_csv:="${REPO}/${OUT}" \
  commit:="${COMMIT}" max_queries:="${N}" ompl_log_level:="${PCP_OMPL_LOG_LEVEL:-info}"
rows=$(($(wc -l <"${OUT}") - 1))
echo "[bench_planners] ${rows} rows in ${OUT} (commit ${COMMIT}) in $(( $(date +%s) - start )) s"
