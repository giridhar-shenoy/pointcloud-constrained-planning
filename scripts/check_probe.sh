#!/usr/bin/env bash
# Where the planning time goes: solve each of the first N queries in MoveIt's own OMPL planning
# context with MoveIt's validity checker subclassed to count and time every call, and with the
# direct planner; one query per process (reusing MoveIt's cached context after swapping its
# checker crashed). Plan only, no Gazebo. Merges the per-query results into OUT.
#
#   scripts/check_probe.sh [N] [OUT]      defaults: 20, results/check_probe.json
#
# Timings: on the shared machine run it under all three locks (see bench_direct.sh).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO}"
N="${1:-20}"
OUT="${2:-results/check_probe.json}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source "${REPO}/install/setup.bash"
set -u
GIT=(git -c safe.directory="${REPO}")
COMMIT="$("${GIT[@]}" rev-parse --short HEAD)"
if ! "${GIT[@]}" diff --quiet HEAD -- src results/queries_boxes.json; then
  COMMIT="${COMMIT}-dirty"
fi
WORK="${REPO}/results/tmp/probe"
rm -rf "${WORK}" && mkdir -p "${WORK}"
for i in $(seq 0 $((N - 1))); do
  timeout 300 ros2 launch pcp_direct bench_direct.launch.py \
    queries_file:="${REPO}/results/queries_boxes.json" output_csv:="${WORK}/q${i}.csv" \
    commit:="${COMMIT}" max_queries:=0 first_query:="${i}" probe_queries:=1 >"${WORK}/q${i}.log" 2>&1
  test -f "${WORK}/q${i}.csv.meta.json"
done
python3 - "${WORK}" "${N}" "${REPO}/${OUT}" "${COMMIT}" <<'PY'
import json, pathlib, sys
work, n, out, commit = pathlib.Path(sys.argv[1]), int(sys.argv[2]), sys.argv[3], sys.argv[4]
entries = []
for i in range(n):
    entries += json.loads((work / f"q{i}.csv.meta.json").read_text())["moveit_check_probe"]
pathlib.Path(out).write_text(json.dumps(
    {"commit": commit, "queries": n, "one_process_per_query": True,
     "moveit_check_probe": entries}, indent=2) + "\n")
print(f"[check_probe] {len(entries)} queries -> {out} (commit {commit})")
PY
