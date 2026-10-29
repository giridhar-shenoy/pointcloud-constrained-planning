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
lock3() {  # heavy, build and gazebo together, in that order (timing-sensitive runs)
  local d="${HOME}/work/.locks"
  if [ -d "${d}" ]; then
    flock "${d}/heavy.lock" flock "${d}/build.lock" flock "${d}/gazebo.lock" "$@"
  else
    "$@"
  fi
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
colcon test --packages-select pcp_scenes pcp_benchmark pcp_direct && colcon test-result --verbose
python3 -m pytest -q scripts/tests

mkdir -p results/tmp
src/pcp_benchmark/scripts/make_robot_files.sh results/tmp/ur5e_rgbd.urdf results/tmp/ur5e.srdf
install/pcp_benchmark/lib/pcp_benchmark/generate_queries --urdf results/tmp/ur5e_rgbd.urdf \
  --srdf results/tmp/ur5e.srdf --scene src/pcp_scenes/config/shelf.yaml \
  --out results/tmp/queries.json --seed 0 --count 100
cmp results/tmp/queries.json results/queries.json
install/pcp_benchmark/lib/pcp_benchmark/generate_queries --urdf results/tmp/ur5e_rgbd.urdf \
  --srdf results/tmp/ur5e.srdf --scene src/pcp_scenes/config/shelf.yaml \
  --out results/tmp/queries_seed1.json --seed 1 --count 20
cmp results/tmp/queries_seed1.json results/tuning/queries_seed1.json
# v0.2: the box-scene query set is the same generator, scene and seed (so the same bytes).
cmp results/tmp/queries.json results/queries_boxes.json
echo "[regenerate_all] queries.json, queries_boxes.json and the tuning set regenerate byte for byte"

lock gazebo scripts/ci_smoke.sh
# Tuning set (seed 1): how RRTConnect's range was chosen.
lock gazebo scripts/bench_planners.sh 20 results/tuning/tuning.csv \
  results/tuning/queries_seed1.json results/tuning/tuning.yaml
lock gazebo scripts/bench_planners.sh 20 results/tuning/tuning_bitrrt.csv \
  results/tuning/queries_seed1.json results/tuning/tuning_bitrrt.yaml
# Evaluation set (seed 0): the README table, BiTRRT column, octomap resolution sweep.
lock gazebo scripts/bench_planners.sh 100 results/bench.csv
lock gazebo scripts/bench_planners.sh 100 results/bench_bitrrt.csv results/queries.json \
  results/bitrrt.yaml
lock gazebo scripts/bench_octomap_resolution.sh 20 0.02 0.05
# The 0.005-resolution ablation is regenerated at its own commit:
#   git checkout 6c12eeb && scripts/bench_planners.sh 100 results/ablation/bench_lvsf0.005.csv
python3 scripts/aggregate.py
lock gazebo scripts/record_gifs.sh results/bench.csv
# v0.2: collision agreement, direct vs MoveIt with execution, the check-count probe, tables.
# Timings are published, so these take the heavy, build and gazebo locks together.
lock3 scripts/collision_agreement.sh 500
lock3 scripts/bench_direct.sh 100
lock3 scripts/check_probe.sh 20
lock3 env PCP_CONSTRAINT=orientation scripts/bench_direct.sh 100 results/bench_direct_upright.csv \
  --plan-only
lock3 scripts/bench_direct.sh 100 results/bench_direct_octomap.csv --octomap
python3 scripts/aggregate_direct.py
echo "[regenerate_all] done in $(( $(date +%s) - start )) s"
