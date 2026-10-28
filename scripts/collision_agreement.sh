#!/usr/bin/env bash
# Collision agreement: MoveIt's planning scene (FCL) vs the direct Pinocchio/coal checker on N
# seeded uniform configurations, plus a boundary probe. No Gazebo.
#
#   scripts/collision_agreement.sh [N] [OUT_CSV] [SEED]
#   defaults: 500, results/collision_agreement.csv, 0
#   writes OUT_CSV, OUT_CSV.meta.json and <OUT without .csv>_boundary.csv
#
# The per-check times it reports are timings: on the shared machine run it under all three locks
#   flock ~/work/.locks/heavy.lock flock ~/work/.locks/build.lock flock ~/work/.locks/gazebo.lock \
#     scripts/collision_agreement.sh
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO}"
N="${1:-500}"
OUT="${2:-results/collision_agreement.csv}"
SEED="${3:-0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source "${REPO}/install/setup.bash"
set -u
GIT=(git -c safe.directory="${REPO}")
COMMIT="$("${GIT[@]}" rev-parse --short HEAD)"
if ! "${GIT[@]}" diff --quiet HEAD -- src; then
  COMMIT="${COMMIT}-dirty"
fi
mkdir -p "$(dirname "${OUT}")"
timeout 600 ros2 launch pcp_direct agreement.launch.py output_csv:="${REPO}/${OUT}" \
  count:="${N}" seed:="${SEED}" commit:="${COMMIT}"
test -f "${REPO}/${OUT}.meta.json"
python3 - "${REPO}/${OUT}.meta.json" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
print(f"[collision_agreement] {m['agree']}/{m['count']} agree ({m['agreement_pct']:.2f} %), "
      f"boundary {m['boundary_agree']}/{m['boundary_brackets']}, pair sets equal "
      f"{m['pair_sets_equal']}, commit {m['commit']}")
PY
