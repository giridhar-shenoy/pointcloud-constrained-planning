# NOTES — pointcloud-constrained-planning (R2)

Session log, decisions, unverified items and commands that worked. A fresh session reads this file,
CHECKPOINTS.md, MORNING.md and `git log -20` before doing anything.

## Session 1 — 2026-10-28, overnight mode (03:30–09:00 IST, Giri asleep)

Environment for every shell: `export ROS_DOMAIN_ID=12 GZ_PARTITION=r2 MAKEFLAGS=-j4`, then
`source /opt/ros/lyrical/setup.bash` and `source install/setup.bash`.
Builds: `flock ~/work/.locks/build.lock colcon build --parallel-workers 2 ...`.
Anything that starts Gazebo: `flock ~/work/.locks/gazebo.lock timeout <s> <cmd>`.

### Plan (scope = SPRINT OVERRIDE MUST list; the feature prompt's wider grid goes to "What's next")

1. Plumbing skeleton: LICENSE, README stub + CI badge, NOTES/CHANGELOG/THIRD_PARTY, Dockerfile
   (`ros:lyrical-ros-base`, verified in SHARED.md via the registry API), devcontainer, CI workflow
   (docker build in the job, then colcon build/test inside the image), pre-commit (ruff, ruff-format,
   clang-format with the ament ROS 2 style, yamllint, markdownlint-cli2), docs and scripts stubs.
   `gh repo create pointcloud-constrained-planning --private --source . --push`, description and topics.
   CHECKPOINT 0 → CHECKPOINTS.md.
2. `pcp_description`: xacro overlay of `ur_simulation_gz/urdf/ur_gz.urdf.xacro` (ur5e) plus a fixed
   camera mount carrying a Gazebo `rgbd_camera` sensor.
3. `pcp_scenes`: `config/shelf.yaml` is the single source of truth for the shelf geometry; a script
   writes the Gazebo world (`pcp_bringup/worlds/shelf.sdf`) and the C++ query generator reads the
   same YAML for collision checking. A test checks the two agree.
4. `pcp_bringup/sim.launch.py`: includes `ur_sim_control.launch.py` (ur5e, no GUI, no RViz, our
   description and world, `--headless-rendering`), the `ros_gz_bridge` (`bridges.yaml`:
   `/camera/points` ↔ `gz.msgs.PointCloudPacked`, `/clock`), and `move_group` from
   `pcp_moveit_config`.
5. `pcp_moveit_config`: SRDF from `ur_moveit_config`, KDL kinematics, `ompl_planning.yaml`
   (RRTConnect, `enforce_constrained_state_space: true`, projection evaluator on the first two
   joints), `sensors_3d.yaml` (PointCloudOctomapUpdater, max_range 2.0, subsample 1, padding),
   `octomap_resolution: 0.02`.
6. `scripts/ci_smoke.sh`: launch under the gazebo lock in its own process group, wait for a
   non-empty `world.octomap` on `/monitored_planning_scene`, plan one unconstrained RRTConnect
   query, assert success, 240 s timeout, kill the group, confirm no `gz sim` of ours is left.
   CHECKPOINT 1.
7. `pcp_benchmark` (C++17, MoveItCpp + PlanningComponent, Eigen):
   - constraint builder (`none`, `orientation` keep-upright 5°) → `moveit_msgs::msg::Constraints`;
   - seeded query generator (100 queries, seed 0, shelf only, tool-down TCP poses inside the
     shelf, deterministic damped-least-squares IK, collision check against the shelf YAML) →
     `results/queries.json`; gtest regenerates and diffs;
   - runner: RRTConnect × {none, orientation} × constrained-state-space × 1 rep, 10 s timeout;
     metrics success, planning time, joint-space and TCP path length (+ max tilt along the path);
     `results/bench.csv` stamped with commit hash and seed;
   - gtests: constraint builder, query determinism, metrics.
8. Dry run 10 queries → project the 100-query time (stop and record in MORNING.md if > 90 min).
   Full 100-query run under the gazebo lock (≤ 1 h). CHECKPOINT 2.
9. `scripts/aggregate.py` (Wilson 95 % intervals, median/p95 time, path lengths → docs/results.md)
   and plots.
10. GIF: constrained vs unconstrained path through the shelf, recorded from a second Gazebo camera
    with headless rendering (SHARED.md decision; no RViz, so no approval needed), ffmpeg → GIF.
11. README (required shape + NUS 34489 / CMU NREC / Perceptyne L3 table), docs/architecture.md
    (Mermaid), docs/planners.md, CHANGELOG, NOTES open questions. CI: build + gtest + ci_smoke +
    5-query mini-benchmark. CHECKPOINT final; MORNING.md STATUS.

### Environment facts verified this session

| Item | Value | Command |
|---|---|---|
| Python | 3.14.4 | `python3 --version` |
| moveit (+ros-planning, ros-perception, planners-ompl/chomp/stomp, moveit-py) | 2.15.2 installed | `dpkg-query -W ros-lyrical-<pkg>` |
| ur_moveit_config / ur_simulation_gz / ur_description | 6.1.0 / 2.5.0 / 4.3.1 | same |
| ros_gz_bridge / ros_gz_sim | 3.0.10 | same |
| gz_ros2_control | 3.0.9 | same |
| pick_ik | NOT installed; candidate 1.1.3 exists on Lyrical | `apt-cache policy ros-lyrical-pick-ik` |
| trac_ik_kinematics_plugin | NOT installed; candidate 2.2.0 exists | `apt-cache policy ros-lyrical-trac-ik-kinematics-plugin` |
| moveit_visual_tools | NOT installed; candidate 4.2.0 | `apt-cache policy ros-lyrical-moveit-visual-tools` |
| MoveItCpp `plan(params, planning_scene)` overload | present | `planning_component.hpp:197` |
| OMPL constrained orientation | exponential coordinates (rotation vector of R_ee⁻¹·R_target), one position + one orientation constraint max, BOX regions only | `ompl_constraints.hpp`, strings in `libmoveit_ompl_interface.so.2.15.2` |
| matplotlib / numpy / scipy (system) | 3.10.7 / 2.3.5 / 1.16.3 | `python3 -c "import ..."` |
| pre-commit hook tags | ruff-pre-commit v0.17.3, mirrors-clang-format v23.1.3, yamllint v1.38.0, markdownlint-cli2 v0.24.0, pre-commit-hooks v6.0.0 | `git ls-remote --tags` |

### Decisions

- **No `ur_moveit.launch.py`**: it waits on `ur_robot_driver`'s `wait_for_robot_description`; our
  own `move_group` launch uses `MoveItConfigsBuilder` with the UR SRDF xacro.
- **Benchmark scene snapshot**: `move_group` owns the octomap (one updater, the same map RViz shows).
  The benchmark node's MoveItCpp lives in namespace `/pcp_benchmark` so its planning-scene publisher
  does not feed back into `move_group`, listens to `/monitored_planning_scene`, waits until the
  octomap is non-empty, then clones the scene once and plans every query against that frozen
  snapshot (`PlanningComponent::plan(params, scene)`), so all queries see the same map.
- **Orientation constraint**: OMPL's constrained state space bounds each component of the rotation
  vector. For a rotation vector r the tilt of the tool z axis satisfies tilt ≤ ‖(r_x, r_y)‖, so
  per-axis x/y tolerances of 5°/√2 = 3.536° guarantee a tilt cone of ≤ 5°; z (yaw) tolerance is π
  (free). The benchmark reports the measured max tilt along a densely re-interpolated path.
- **Queries**: every start and goal is tool-down (inside the 5° cone), so the same 100 queries serve
  `none` and `orientation`. The feature prompt's 50/50 "start violates the constraint" split only
  matters for rejection-sampling mode, which is NOT THIS WEEK; constrained-state-space mode cannot
  start from an off-manifold state.
- **IK for the generator**: own damped-least-squares IK on MoveIt's FK/Jacobian with a fixed
  iteration budget and seeds from the query RNG. The KDL plugin re-seeds from an unseeded RNG and
  stops on wall-clock timeouts, so it cannot give a byte-identical queries.json.

### Progress log (session 1)

- 03:44 skeleton pushed; CI refused by GitHub billing (MORNING.md item 1). 03:59 checkpoint 1.
- 04:07 `generate_queries` → results/queries.json (100 queries, seed 0) in 0.5 s: 1007 pose
  samples, 0 IK failures, 6151 converged-but-colliding solutions rejected (padded shelf).
- 04:09 gtests: 14/14 pass (constraints 6, metrics 5, query determinism 3) + pcp_scenes pytest 3.
- 04:12 first dry run failed: `/**` defaults in benchmark.yaml beat the launch overrides
  (rclcpp applies the wildcard after the node-specific file) → those keys removed from the YAML.
- 04:13–04:24 my with_sim.sh wait loop used `pgrep -f "gz sim"`, which matched my own parent
  shell's command text, so it held the Gazebo lock ~10 min doing nothing while R1 waited. Fixed
  with the pattern `[b]in/gz sim` (matches only the real `ruby .../bin/gz sim` server); the stuck
  run was stopped through the task manager (a direct `kill` was refused by the sandbox).

### Commands that worked

```bash
# build (uv's python3.14 shadows /usr/bin/python3 on PATH; CMake must use the system one)
flock ~/work/.locks/build.lock colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
colcon test --packages-select pcp_scenes pcp_benchmark && colcon test-result --verbose
python3 -m pytest -q scripts/tests
flock ~/work/.locks/gazebo.lock scripts/ci_smoke.sh             # 14 s locally
flock ~/work/.locks/gazebo.lock scripts/bench_planners.sh 10 results/tmp/dry10.csv
src/pcp_benchmark/scripts/make_robot_files.sh results/tmp/ur5e_rgbd.urdf results/tmp/ur5e.srdf
install/pcp_benchmark/lib/pcp_benchmark/generate_queries --urdf results/tmp/ur5e_rgbd.urdf \
  --srdf results/tmp/ur5e.srdf --scene src/pcp_scenes/config/shelf.yaml --out results/queries.json
```

### API / environment differences hit

- launch_ros YAML-parses xacro output before using it as a string: a colon in an xacro comment
  ("geometry: the camera") breaks `robot_description`. No colons in xacro comments.
- `LockedPlanningSceneRO` has no `operator*` in 2.15; bind it to a
  `const PlanningSceneConstPtr&` first.
- rosdep key for octomap on Lyrical is `liboctomap-dev` (no `octomap` key).
- gz rgbd_camera point cloud is in the sensor link frame (x forward), not the optical frame.

### Decisions after the first 100-query run (04:30–04:47)

- **Collision-check resolution bug (mine).** MoveIt's `ModelBasedStateSpace::getMaximumExtent()`
  is the SUM of joint ranges: 22π ≈ 69.1 rad for the UR5e (±2π joints, elbow ±π). With the usual
  `longest_valid_segment_fraction: 0.005`, OMPL checks motions only every ~0.35 rad, so paths can
  step over 2 cm octomap boards; MoveIt's `ValidateSolution` then rejects them (error -2). In the
  first run every unconstrained failure was -2. Fixed to 0.0003 (≈ 0.021 rad). The first run is
  kept as an ablation (`results/ablation/bench_lvsf0.005.csv`, commit 2f08131).
- **RRTConnect range.** `range: 0` means 20 % of that extent = 13.8 rad per extension. Instead of
  tuning on the evaluation queries, range is chosen on a separate tuning set:
  `results/tuning/queries_seed1.json` (seed 1, 20 queries), orientation constraint only, ranges
  {auto, 0.5, 1, 2}; the winner is written into ompl_planning.yaml as the benchmark planner and
  the seed-0 evaluation is run once with it.

### Independent code review (subagent, read-only) — 04:50

Findings acted on (commit 89533bb):

1. TOTG ran before ValidateSolution with 0.1 rad blending → validation saw resampled, blended
   waypoints. Now ValidateSolution first, `totg.path_tolerance: 0.01`, plus a dense (0.01 rad)
   collision re-check of the executed trajectory (`dense_collision_free` column).
2. Rotation-vector yaw/tilt coupling: with reference yaw 0 the admissible tilt dropped to ~2.25°
   at 180° yaw offset. Reference yaw is now the circular mean of start/goal yaw per query; test
   `ToleranceDependsOnYawOffsetFromReference` pins the coupling.
3. (Unverified, MoveIt-side) the constrained validity checker re-checks path constraints with
   exact tolerances while OMPL's projection stops at a 1e-4 residual → projected states may be
   rejected. Not investigated tonight; listed in "where it fails" as a hypothesis.
4–7. planning_time empty on failure; mode column derived from config; trajectory file names
   include the planner; OMPL seed 1 (OMPL maps 0 → 1).
Review found no problems in metrics.cpp, the IK frames, determinism, snapshot use, CSV alignment.

The tuning run that had been queued before the review (still waiting for R1's lock) was
stopped and re-queued after the fixes, so tuning uses the final pipeline.

### Final evaluation and GIF (05:05–05:42)

- Evaluation (commit 8c81742): unconstrained 100/100, keep-upright 84/100 (Wilson 76–90 %), all
  16 failures 10 s timeouts; worst constrained tilt 3.6°; dense re-check 93/100 and 83/84.
  7.7 min wall time.
- GIF: `joint_trajectory_controller` had never initialized in the sim (ros2_control 6.10: no params
  without `--param-file`); fixed in sim.launch.py. Demo camera pose must be given to
  `ros_gz_sim create` as -x/-y/-z/-R/-P/-Y. Replays at 2× and 5× slow-down aborted on JTC path
  tolerance (0.2 rad lag, arm clipping the shelf) → 15× slow-down, shown at 3×. Query 0 (7.6 s in
  the benchmark) failed 3 re-plans → rule now requires the constrained cell within the median
  time → query 2. Possible improvement: camera framing (arm sits at the left edge).
- `ros2_control`/gz note for R1-style work: `position_proportional_gain` (gz_ros2_control,
  default 0.1) governs position tracking in sim; not changed here.

### Unverified items

- GitHub Actions: nothing has run (billing). Docker image never built (no Docker locally).
  Headless ogre2 on the runner's llvmpipe is unverified; if it fails, ci_smoke fails loudly (it
  does not skip) and the fallback is the git-lfs bag, which needs Giri's approval.
- Mermaid diagram in docs/architecture.md not render-checked (npx was not allowed).
- VS Code extension id `ms-iot.vscode-ros` in devcontainer.json not checked against the
  marketplace (that extension is deprecated upstream).

### SHOULD runs and the second-octomap bug (05:46–06:35)

- First sweep/BiTRRT chain finished 06:27. The 5 cm sweep's snapshot recorded a **2 cm** octree:
  MoveItCpp's planning-scene monitor starts a world-geometry monitor, and the benchmark node had
  been given `sensors_3d`, so it ran its own PointCloudOctomapUpdater at 2 cm. Fixed in a2441cd
  (sensor/octomap params stripped from the benchmark node). Before the fix both updaters used the
  same camera and settings (4747 voxels either way), so tuning results are kept; the evaluation,
  BiTRRT column and sweep are re-run after the fix.
- BiTRRT tuning (seed 1, orientation): auto 13/20, 2 rad 14/20, 6 rad 9/20 → BiTRRTRange2.
  Pre-fix BiTRRT evaluation: none 0/100, orientation 77/100. Hypothesis for 0/100 unconstrained:
  BiTRRT joins its trees only within one `range` step of the L1 joint metric; in the projected
  space steps are re-projected and connections happen more often. Not verified.
- I stopped my own BiTRRT run once (05:45) to let R1 run its final smoke test first.
- 06:40 the stripped benchmark node never received an octomap: in 2.15 MoveItCpp's monitor
  *listens* on `planning_scene` and *publishes* on `monitored_planning_scene`, and my remap had
  pointed its publisher at move_group's topic. Fixed (56ea7e9): `planning_scene` →
  `/monitored_planning_scene`. 2-query check: snapshot = move_group's map, 4747 voxels at 2 cm, the
  same count as the old self-built map. Re-running sweep, evaluation and BiTRRT after this fix.
