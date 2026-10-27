#!/usr/bin/env bash
# SHOULD item: octomap resolution sweep. Runs the first N evaluation queries (default 20) with the
# benchmark planner under both constraint types at each resolution, one sim per resolution.
#
#   scripts/bench_octomap_resolution.sh [N] [RES...]      (defaults: 20, 0.02 0.05)
#
# Locally (shared machine): flock ~/work/.locks/gazebo.lock scripts/bench_octomap_resolution.sh
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO}"
N="${1:-20}"
shift || true
RESOLUTIONS=("$@")
[ "${#RESOLUTIONS[@]}" -gt 0 ] || RESOLUTIONS=(0.02 0.05)
mkdir -p results/sweep
for res in "${RESOLUTIONS[@]}"; do
  PCP_SIM_ARGS="octomap_resolution:=${res}" \
    "${REPO}/scripts/bench_planners.sh" "${N}" "results/sweep/octomap_${res}.csv"
done
