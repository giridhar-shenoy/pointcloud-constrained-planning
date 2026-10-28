#!/usr/bin/env bash
# Record docs/assets/constrained_vs_unconstrained.gif from Gazebo (headless rendering, no RViz).
#
# Query choice is mechanical, not hand-picked: the lowest query id in results/bench.csv that was
# solved both without a constraint and with the keep-upright constraint, with the constrained cell
# solved within the median constrained planning time (so the re-plan reliably succeeds; query 0
# took 7.6 s in the benchmark and failed three re-plans). That query is re-planned
# once per constraint by the C++ benchmark node (trajectories saved as JSON), then both
# trajectories are executed in Gazebo while a demo camera records, and ffmpeg stacks them.
# Re-planning means the GIF shows *a* solution of that query, not the benchmark cell's exact path.
# Playback: executed 15x slower than the time-optimal trajectory (gz_ros2_control tracks position
# commands through a soft proportional loop; faster replays lag 0.2 rad and clip the shelf), GIF
# shown at 3x the recorded speed.
#
# Locally (shared machine): flock ~/work/.locks/gazebo.lock scripts/record_gifs.sh
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}" GZ_PARTITION="${GZ_PARTITION:-r2}"
CSV="${1:-results/bench.csv}"
OUT="docs/assets/constrained_vs_unconstrained.gif"
WORK="results/tmp/gif"
rm -rf "${WORK}" && mkdir -p "${WORK}/traj" "${WORK}/frames"

QID=$(python3 - "${CSV}" <<'PY'
import csv, sys
import statistics
rows = list(csv.DictReader(open(sys.argv[1])))
t_ori = {int(r["query_id"]): float(r["planning_time_s"]) for r in rows
         if r["constraint"] == "orientation" and r["success"] == "1"}
median = statistics.median(t_ori.values()) if t_ori else 0.0
ok = {}
for r in rows:
    ok.setdefault(int(r["query_id"]), set())
    if r["success"] == "1":
        ok[int(r["query_id"])].add(r["constraint"])
both = [q for q, s in sorted(ok.items())
        if {"none", "orientation"} <= s and t_ori[q] <= median]
print(both[0] if both else -1)
PY
)
if [ "${QID}" -lt 0 ]; then
  echo "[record_gifs] no query solved under both constraint types in ${CSV}" >&2
  exit 1
fi
PLANNER=$(python3 -c "import csv,sys; print(next(csv.DictReader(open(sys.argv[1])))['planner'])" "${CSV}")
echo "[record_gifs] query ${QID}, planner ${PLANNER}"

cat >"${WORK}/inner.sh" <<INNER
#!/usr/bin/env bash
set -euo pipefail
for attempt in 1 2 3 4 5; do
  ros2 launch pcp_benchmark benchmark.launch.py queries_file:=${REPO}/results/queries.json \
    output_csv:=${REPO}/${WORK}/replan.csv first_query:=${QID} max_queries:=1 \
    trajectory_dir:=${REPO}/${WORK}/traj commit:=gif
  if [ -f ${REPO}/${WORK}/traj/q${QID}_none_${PLANNER}.json ] && [ -f ${REPO}/${WORK}/traj/q${QID}_orientation_${PLANNER}.json ]; then
    break
  fi
  echo "[record_gifs] re-plan attempt \${attempt} did not solve both cells"
done
ros2 run pcp_bringup record_demo.py --out-dir ${REPO}/${WORK}/frames --time-scale 15 \
  --traj ${REPO}/${WORK}/traj/q${QID}_none_${PLANNER}.json --traj ${REPO}/${WORK}/traj/q${QID}_orientation_${PLANNER}.json
INNER
chmod +x "${WORK}/inner.sh"
"${REPO}/scripts/with_sim.sh" "${WORK}/inner.sh"

tilt() { python3 -c "import json,sys; print(f\"{json.load(open(sys.argv[1]))['max_tilt_deg']:.0f}\")" "$1"; }
T_NONE=$(tilt "${WORK}/traj/q${QID}_none_${PLANNER}.json")
T_ORI=$(tilt "${WORK}/traj/q${QID}_orientation_${PLANNER}.json")
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf
LABEL="fontfile=${FONT}:fontsize=17:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=6:x=10:y=10"
ffmpeg -loglevel error -y \
  -framerate 30 -i "${WORK}/frames/q${QID}_none_${PLANNER}/frame_%04d.png" \
  -framerate 30 -i "${WORK}/frames/q${QID}_orientation_${PLANNER}/frame_%04d.png" \
  -filter_complex "[0:v]drawtext=${LABEL}:text='no constraint - max tool tilt ${T_NONE} deg'[a];\
[1:v]drawtext=${LABEL}:text='keep upright (5 deg) - max tilt ${T_ORI} deg'[b];\
[a][b]hstack=inputs=2,fps=10,scale=800:-1:flags=lanczos,split[s0][s1];\
[s0]palettegen=max_colors=96:stats_mode=diff[p];[s1][p]paletteuse=dither=bayer:bayer_scale=4" \
  "${OUT}"
echo "[record_gifs] wrote ${OUT} ($(du -h "${OUT}" | cut -f1)), query ${QID}, tilt ${T_NONE} / ${T_ORI} deg"
