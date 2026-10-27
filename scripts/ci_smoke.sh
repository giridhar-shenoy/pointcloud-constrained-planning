#!/usr/bin/env bash
# CI smoke test: bring up the sim headless, wait for the octomap built from the simulated depth
# camera to appear in /monitored_planning_scene, plan one unconstrained RRTConnect query, and
# assert success. Hard limit 240 s. Tears the simulation down (whole process group) on exit.
#
# Locally (shared machine): flock ~/work/.locks/gazebo.lock scripts/ci_smoke.sh
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}" GZ_PARTITION="${GZ_PARTITION:-r2}"
start=$(date +%s)
timeout --signal=TERM "${PCP_SMOKE_TIMEOUT:-240}" \
  "${REPO}/scripts/with_sim.sh" ros2 run pcp_bringup smoke_check.py
echo "[ci_smoke] PASS in $(( $(date +%s) - start )) s"
