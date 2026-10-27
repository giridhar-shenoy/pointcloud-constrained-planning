#!/usr/bin/env bash
# Regenerate every number and GIF in the README from scratch:
#   1. build the workspace and run the unit tests (incl. the queries.json determinism test),
#   2. regenerate results/queries.json from seed 0 and fail if it differs from the committed file,
#   3. run the smoke test, the 100-query planner benchmark, the aggregation and the GIF.
# Gazebo steps take the shared-machine lock when ~/work/.locks exists (no-op elsewhere).
# Measured runtime on the i5-1135G7 laptop is recorded in NOTES.md.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}" GZ_PARTITION="${GZ_PARTITION:-r2}" MAKEFLAGS="${MAKEFLAGS:--j4}"
start=$(date +%s)
lock() {  # lock <name> <cmd...>
  local name="$1"; shift
  if [ -d "${HOME}/work/.locks" ]; then flock "${HOME}/work/.locks/${name}.lock" "$@"; else "$@"; fi
}

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
set -u
lock build colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
set +u
# shellcheck disable=SC1091
source install/setup.bash
set -u
colcon test --packages-select pcp_scenes pcp_benchmark && colcon test-result --verbose
python3 -m pytest -q scripts/tests

mkdir -p results/tmp
src/pcp_benchmark/scripts/make_robot_files.sh results/tmp/ur5e_rgbd.urdf results/tmp/ur5e.srdf
install/pcp_benchmark/lib/pcp_benchmark/generate_queries --urdf results/tmp/ur5e_rgbd.urdf \
  --srdf results/tmp/ur5e.srdf --scene src/pcp_scenes/config/shelf.yaml \
  --out results/tmp/queries.json --seed 0 --count 100
cmp results/tmp/queries.json results/queries.json
echo "[regenerate_all] queries.json regenerates byte for byte"

lock gazebo scripts/ci_smoke.sh
lock gazebo scripts/bench_planners.sh 100 results/bench.csv
python3 scripts/aggregate.py --csv results/bench.csv
lock gazebo scripts/record_gifs.sh results/bench.csv
echo "[regenerate_all] done in $(( $(date +%s) - start )) s"
