#!/usr/bin/env bash
# Record the README's three GIFs from one simulation run:
#   docs/assets/hero.gif              point cloud -> octomap -> planned path -> the arm follows it
#   docs/assets/keep_upright.gif      the same query without a constraint and with keep-upright
#   docs/assets/direct_vs_moveit.gif  the same query planned and executed by both pipelines
#
#   scripts/record_gifs.sh [BENCH_CSV] [DIRECT_CSV]
#   defaults: results/bench.csv results/bench_direct.csv
#
# Query choice is mechanical, never by looks: ONE query for all three GIFs, the lowest query id
# (1) whose start and goal lie in different compartments (the GIFs must show a path out of one
#     opening and into another; a same-compartment query showed no tilt either way),
# (2) that BENCH_CSV solved both without a constraint and with the keep-upright constraint, the
#     constrained cell within the median constrained planning time (so the re-plan reliably
#     succeeds; a 7.6 s query once failed three re-plans), and
# (3) that DIRECT_CSV planned and executed to the goal with both pipelines.
# Paths, tilts and times are never looked at.
#
# Recording (one simulation, started and stopped by scripts/with_sim.sh):
#  1. scripts/gif/capture_map.py saves one depth-camera point cloud and move_group's octomap once
#     the map has settled (arm at its start-up pose, as when the benchmark freezes its scene).
#  2. The v0.1 benchmark node re-plans the query, unconstrained and keep-upright, and saves both
#     trajectories as JSON; the GIFs show *a* solution of the query, not the benchmark cell's
#     exact path (up to 5 re-plans until both are solved).
#  3. record_demo.py executes both in Gazebo 15x slower than the time-optimal trajectory
#     (gz_ros2_control tracks position commands through a soft loop; faster replays lag past the
#     controller's 0.2 rad path tolerance), recording a demo camera (20 Hz sim time, every 4th
#     frame kept) and the joint positions at every frame. An execution that MoveIt reports as
#     CONTROL_FAILED is retried from the start, up to 3 attempts in all, each attempt logged.
#  4. record_demo.py records while pcp_direct's bench_direct plans the same query once with each
#     pipeline (MoveIt 2 and the direct rebuild, same box shelf and settings) and executes both
#     plans at full speed on the same controller (one query, not a benchmark run). The planning
#     times printed come from that run's CSV. Up to 3 attempts in all, until both executions
#     reached the goal and both start-to-goal motions are found in the recording.
# Rendering (no simulation):
#  5. scripts/gif/render_hero.py draws the hero in RViz on a virtual display (xvfb-run): the
#     camera points move_group builds the map from (its self-filtered cloud: the robot's own
#     points removed), the octomap, the planned tool path, and the arm moving through the joint
#     positions recorded in step 3 (scripts/gif/hero.rviz).
#  6. scripts/gif/compose.py burns in captions and numbers (tilt by forward kinematics from the
#     recorded joint positions; planning times from the CSV) and encodes with ffmpeg.
#  7. Every GIF must be <= 3 MB (3,000,000 bytes), <= 15 s and 640-800 px wide; otherwise the
#     script exits 1.
# Playback: the hero's motion fills 5 s of GIF and the keep-upright motion 11.5 s, whatever the
# trajectory's duration; the direct-vs-MoveIt executions are shown at half speed. Each caption
# prints the resulting speed relative to the planned trajectory.
#
# Locally (shared machine), with Gazebo and RViz under the locks:
#   flock ~/work/.locks/heavy.lock flock ~/work/.locks/gazebo.lock scripts/record_gifs.sh
# PCP_GIF_STAGE=compose re-renders and re-encodes from the last recording in results/tmp/gif.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${REPO}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}" GZ_PARTITION="${GZ_PARTITION:-r2}"
CSV="${1:-results/bench.csv}"
DIRECT_CSV="${2:-results/bench_direct.csv}"
STAGE="${PCP_GIF_STAGE:-all}"
WORK="${REPO}/results/tmp/gif"
ASSETS="${REPO}/docs/assets"
QUERIES="${REPO}/results/queries.json"
QUERIES_BOXES="${REPO}/results/queries_boxes.json"  # same bytes, checked by regenerate_all.sh
GIF="${REPO}/scripts/gif"

set +u
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
# shellcheck disable=SC1091
source "${REPO}/install/setup.bash"
set -u

QID=$(python3 - "${CSV}" "${DIRECT_CSV}" <<'PY'
import csv
import statistics
import sys

rows = list(csv.DictReader(open(sys.argv[1])))
t_ori = {int(r["query_id"]): float(r["planning_time_s"]) for r in rows
         if r["constraint"] == "orientation" and r["success"] == "1"}
median = statistics.median(t_ori.values()) if t_ori else 0.0
ok = {}
for r in rows:
    ok.setdefault(int(r["query_id"]), set())
    if r["success"] == "1":
        ok[int(r["query_id"])].add(r["constraint"])
cross = {int(r["query_id"]) for r in rows if r["start_region"] != r["goal_region"]}
executed = {}
for r in csv.DictReader(open(sys.argv[2])):
    if r["success"] == "1" and r["exec_success"] == "1":
        executed.setdefault(int(r["query"]), set()).add(r["pipeline"])
picks = [q for q, s in sorted(ok.items())
         if q in cross and {"none", "orientation"} <= s and t_ori[q] <= median
         and {"direct", "moveit"} <= executed.get(q, set())]
print(picks[0] if picks else -1)
PY
)
if [ "${QID}" -lt 0 ]; then
  echo "[record_gifs] no query meets the selection rule in ${CSV} and ${DIRECT_CSV}" >&2
  exit 1
fi
PLANNER=$(python3 -c "import csv,sys; print(next(csv.DictReader(open(sys.argv[1])))['planner'])" "${CSV}")
NONE="q${QID}_none_${PLANNER}"
UPRIGHT="q${QID}_orientation_${PLANNER}"
echo "[record_gifs] query ${QID} (rule over ${CSV} and ${DIRECT_CSV}), planner ${PLANNER}"

if [ "${STAGE}" = "all" ]; then
  rm -rf "${WORK}"
  mkdir -p "${WORK}/traj" "${WORK}/frames"
  src/pcp_benchmark/scripts/make_robot_files.sh "${WORK}/ur5e_rgbd.urdf" "${WORK}/ur5e.srdf"
  cat >"${WORK}/inner.sh" <<INNER
#!/usr/bin/env bash
set -euo pipefail
python3 ${GIF}/capture_map.py --urdf ${WORK}/ur5e_rgbd.urdf --out ${WORK}/map.npz
for attempt in 1 2 3 4 5; do
  ros2 launch pcp_benchmark benchmark.launch.py queries_file:=${QUERIES} \
    output_csv:=${WORK}/replan.csv first_query:=${QID} max_queries:=1 \
    trajectory_dir:=${WORK}/traj commit:=gif
  if [ -f ${WORK}/traj/${NONE}.json ] && [ -f ${WORK}/traj/${UPRIGHT}.json ]; then
    break
  fi
  echo "[record_gifs] re-plan attempt \${attempt}/5 did not solve both cells"
done
test -f ${WORK}/traj/${NONE}.json && test -f ${WORK}/traj/${UPRIGHT}.json
ros2 run pcp_bringup record_demo.py --out-dir ${WORK}/frames --time-scale 15 --fps 20 \
  --keep-every 4 --attempts 3 \
  --traj ${WORK}/traj/${NONE}.json --traj ${WORK}/traj/${UPRIGHT}.json
for attempt in 1 2 3; do
  rm -f ${WORK}/direct.csv ${WORK}/direct.csv.meta.json
  echo "[record_gifs] direct-vs-MoveIt recording, attempt \${attempt}/3"
  if ros2 run pcp_bringup record_demo.py --out-dir ${WORK}/frames/live --fps 20 --during \
      ros2 launch pcp_direct bench_direct.launch.py queries_file:=${QUERIES_BOXES} \
      output_csv:=${WORK}/direct.csv commit:=gif first_query:=${QID} max_queries:=1 \
      execute:=true \
    && python3 ${GIF}/compose.py check-live --live ${WORK}/frames/live --csv ${WORK}/direct.csv \
      --queries ${QUERIES_BOXES} --query ${QID}; then
    exit 0
  fi
  echo "[record_gifs] direct-vs-MoveIt attempt \${attempt}/3 failed (kept in frames/live_failed\${attempt})"
  rm -rf ${WORK}/frames/live_failed\${attempt}
  mv ${WORK}/frames/live ${WORK}/frames/live_failed\${attempt} || true
  cp ${WORK}/direct.csv ${WORK}/frames/live_failed\${attempt}/ || true
done
exit 1
INNER
  chmod +x "${WORK}/inner.sh"
  timeout "${PCP_GIF_TIMEOUT:-1800}" "${REPO}/scripts/with_sim.sh" "${WORK}/inner.sh"
elif [ "${STAGE}" != "compose" ]; then
  echo "[record_gifs] PCP_GIF_STAGE must be all or compose" >&2
  exit 2
fi

# RViz on a virtual display, on its own ROS domain (the hero publishes its own /joint_states).
rm -rf "${WORK}/hero"
ROS_DOMAIN_ID="${PCP_RENDER_DOMAIN_ID:-$(((ROS_DOMAIN_ID + 50) % 101))}" \
  xvfb-run -a -s "-screen 0 1600x900x24" python3 "${GIF}/render_hero.py" \
  --urdf "${WORK}/ur5e_rgbd.urdf" --map "${WORK}/map.npz" --traj "${WORK}/traj/${NONE}.json" \
  --executed "${WORK}/frames/${NONE}/frames.json" --rviz "${GIF}/hero.rviz" \
  --out-dir "${WORK}/hero"
python3 "${GIF}/compose.py" hero --frames "${WORK}/hero" --out "${ASSETS}/hero.gif"
python3 "${GIF}/compose.py" keep_upright --urdf "${WORK}/ur5e_rgbd.urdf" \
  --none "${WORK}/frames/${NONE}" --upright "${WORK}/frames/${UPRIGHT}" \
  --out "${ASSETS}/keep_upright.gif"
python3 "${GIF}/compose.py" direct_vs_moveit --live "${WORK}/frames/live" \
  --csv "${WORK}/direct.csv" --queries "${QUERIES_BOXES}" --query "${QID}" \
  --out "${ASSETS}/direct_vs_moveit.gif"

python3 - "${ASSETS}/hero.gif" "${ASSETS}/keep_upright.gif" "${ASSETS}/direct_vs_moveit.gif" <<'PY'
import json
import os
import subprocess
import sys

bad = 0
for f in sys.argv[1:]:
    probe = json.loads(subprocess.run(
        ["ffprobe", "-v", "error", "-count_frames", "-show_entries",
         "stream=width,height,nb_read_frames:format=duration", "-of", "json", f],
        capture_output=True, text=True, check=True).stdout)
    s, dur, size = probe["streams"][0], float(probe["format"]["duration"]), os.path.getsize(f)
    ok = size <= 3_000_000 and dur <= 15.0 and 640 <= s["width"] <= 800
    bad += not ok
    print(f"[record_gifs] {f}: {size} bytes, {dur:.1f} s, {s['width']}x{s['height']} px,"
          f" {s['nb_read_frames']} frames: {'ok' if ok else 'BREAKS A LIMIT (3 MB, 15 s, 640-800 px)'}")
sys.exit(1 if bad else 0)
PY
echo "[record_gifs] done: query ${QID}"
