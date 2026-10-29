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
  kept as an ablation (`results/ablation/bench_lvsf0.005.csv`, commit 6c12eeb).
- **RRTConnect range.** `range: 0` means 20 % of that extent = 13.8 rad per extension. Instead of
  tuning on the evaluation queries, range is chosen on a separate tuning set:
  `results/tuning/queries_seed1.json` (seed 1, 20 queries), orientation constraint only, ranges
  {auto, 0.5, 1, 2}; the winner is written into ompl_planning.yaml as the benchmark planner and
  the seed-0 evaluation is run once with it.

### Independent code review (subagent, read-only) — 04:50

Findings acted on (commit ca8deb5):

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

- Evaluation (commit e3a1945): unconstrained 100/100, keep-upright 84/100 (Wilson 76–90 %), all
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
  been given `sensors_3d`, so it ran its own PointCloudOctomapUpdater at 2 cm. Fixed in 2951a4c
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
  pointed its publisher at move_group's topic. Fixed (db74fcc): `planning_scene` →
  `/monitored_planning_scene`. 2-query check: snapshot = move_group's map, 4747 voxels at 2 cm, the
  same count as the old self-built map. Re-running sweep, evaluation and BiTRRT after this fix.
- 06:51–06:59 **lost run (my fault):** the post-fix evaluation wrote `results/bench.csv`, a tracked
  file, while I committed the sweep. pre-commit stashes unstaged changes by checking tracked files
  out and restoring them, which replaced the file under the running node; its later rows went to
  the unlinked inode (CSV ended with 1 row, the node had run all 200 cells in 7.9 min). Rules from
  now on: no commits while a benchmark writes; evaluation re-run goes to `results/tmp/` and is
  moved into place afterwards; `bench_planners.sh` will write a `.partial` file and rename it
  (changed only after the running BiTRRT job finishes, because bash reads scripts while running).

### Runtimes measured tonight (i5-1135G7, shared machine)

| Step | Wall time |
|---|---|
| colcon build (incremental / pcp_benchmark from scratch) | 1 s / ~45 s |
| unit tests (22 colcon + 4 pytest) | ~10 s |
| `ci_smoke.sh` | 14–16 s |
| `bench_planners.sh 5` (CI mini-benchmark) | 41 s |
| tuning set, RRTConnect (80 cells) | 10.7 min |
| tuning set, BiTRRT (60 cells) | 6 min |
| evaluation, RRTConnect (200 cells) | 8.2 min |
| evaluation, BiTRRT (200 cells) | 23.7 min |
| octomap sweep (2 × 40 cells) | ~6 min |
| `record_gifs.sh` | ~5 min |
| `regenerate_all.sh` end to end | not run (≈ 60 min > 30 min rule; MORNING.md item 3) |

## CHANGELOG entry for this session

See `CHANGELOG.md` → `[Unreleased]` (Added / Fixed), written this session.

## Open questions for Giri

1. CI: fix the account's Actions billing, or make the repo public now? (MORNING.md item 1)
2. Which Perceptyne L3 JD phrase should the README table evidence? (item 2)
3. OK to run `regenerate_all.sh` end to end (~60 min, holds the Gazebo lock)? (item 3)
4. If CI cannot render the depth camera on llvmpipe, OK to record a < 50 MB git-lfs bag? (item 4)
5. The keep-upright tolerance is the square inscribed in the 5° cone (sound, conservative, yaw
   coupled). Keep it, or report the per-axis-5° variant (up to 7.1° tilt) as a second row?
6. Range tuning stopped at 2 rad for RRTConnect while success was still rising; extend the
   candidate set (e.g. 3, 4 rad) on the tuning queries next session?
7. 6/100 unconstrained trajectories graze the octomap between checked states: add a small
   collision padding for the octomap (trade-off: more queries become invalid, like the 5 cm sweep)?
8. pick_ik (1.1.3) and TRAC-IK (2.2.0) exist on Lyrical as apt candidates; KDL is used (and the
   generator has its own IK). Install either for move_group? (ask-me item; not needed for v0.1)
9. GIF framing: the arm sits at the left edge of the demo camera; worth a camera move before the
   README goes public?

### Commit-type deviation

Five result commits (987c515, 4060411, d09ddc2, a46137f, 8129897) use a `data:` type, which is not
in the repo standard's list (feat/fix/docs/ci/test/chore/refactor). History is not rewritten
(overnight rules); from now on result updates use `chore(results):`.

## Session 2 — 2026-10-29 (SPRINT OVERRIDE v0.2), Step 0: CI on main

- Repo visibility: `gh repo view --json visibility` → `PUBLIC` (made public by Giri on 29 Oct), so
  GitHub Actions now starts jobs (the billing block in MORNING.md item 1 is gone).
- First run that actually executed: run 40649038811 on b587d57,
  <https://github.com/giridhar-shenoy/pointcloud-constrained-planning/actions/runs/40649038811>,
  **success on the first attempt; no Dockerfile or ci.yml fix was needed.** Step times
  (UTC 2026-10-28): lint 1 min 18 s; image build from the Dockerfile 6 min 34 s (cold cache);
  rosdep 7 s; colcon build 53 s; colcon test 2 s (`Summary: 22 tests, 0 errors, 0 failures,
  0 skipped`); pytest 2 passed; `ci_smoke` PASS in 16 s; mini-benchmark (5 queries ×
  {none, orientation}) 54 s, 10 rows. Whole job ≈ 11 min of the 30 min cap.
- Headless rendering on the runner works: `[smoke] octomap OK: 237755 bytes, resolution 0.02 m,
  frame 'world', after 7.9 s` inside `ros:lyrical-ros-base` on the GitHub runner (Mesa, no GPU).
  So the git-lfs point-cloud bag fallback (MORNING.md item 4) is not needed.
- Warning in the run annotations, not an error: the actions pinned at `@v4/@v5/@v3/@v6`
  (checkout, setup-python, cache, build-push-action, setup-buildx-action) target Node.js 20 and
  are forced onto Node.js 24 by the runner. Left unchanged; listed as an open question.
- Commands: `gh run watch 40649038811 --exit-status` (exit 0), `gh run view 40649038811 --log`.

## Session 2 — 2026-10-29, v0.2 "direct pipeline" (SPRINT OVERRIDE v0.2)

Environment for every shell: `export ROS_DOMAIN_ID=12 GZ_PARTITION=r2 MAKEFLAGS=-j4`; builds under
`flock ~/work/.locks/build.lock`, Gazebo under `flock ~/work/.locks/gazebo.lock timeout <s>`.
All v0.2 work is on branch `feat/direct-pipeline`; v0.1 code is reused, not edited.

### Plan (numbered as in the override; CHECKPOINTs in bold)

0. CI green on main (done before this plan; see "Step 0" above). Branch `feat/direct-pipeline`.
   **CHECKPOINT 1.**
1. `src/pcp_direct` (ament_cmake, C++17, Eigen). Libraries with no MoveIt dependency:
   `pcp_direct_collision`, `pcp_direct_planner`, `pcp_direct_traj`, `pcp_direct_exec`.
   Executables that link MoveIt only to act as the reference: `collision_agreement`,
   `bench_direct`, plus the `shelf_boxes_loader` node.
2. Collision module (`collision.hpp/.cpp`):
   - `pinocchio::urdf::buildModelFromXML` + `buildGeom(..., COLLISION, ...)` on the same URDF string
     MoveIt loads (`ur5e_rgbd.urdf.xacro` expanded; collision meshes are ur_description's STLs).
   - Shelf boxes as `coal::Box` GeometryObjects on the universe joint, from
     `pcp_scenes/config/shelf.yaml` (see decision D1: that file already is the single box
     definition the Gazebo world is generated from; a second `shelf_boxes.yaml` would be a copy).
   - `addAllCollisionPairs()`, then `pinocchio::srdf::removeCollisionPairsFromXML` with the SRDF
     MoveIt loads (xacro of `ur.srdf.xacro name:=ur`).
   - Pair sets (corrected after reading the sources, see "Source facts" below): Pinocchio never
     pairs two geometries on the same joint, so universe-fixed geometries (base, ground_plane,
     shelf boxes) are never paired with each other; MoveIt's planner sends `group_name =
     ur_manipulator`, and FCL skips every pair in which neither object moves with the group.
     The two rules select the same pairs here, so nothing is added by hand; a unit test checks
     that the active pair count equals the count derived from MoveIt's ACM and group links. The
     agreement test uses the same `group_name` request as MoveIt's OMPL validity checker.
   - `isValid(q)`: bounds + `computeCollisions(..., stopAtFirstCollision=true)`; counts calls and
     records per-call time; `collidingPairs(q)` and `minDistance(q)` for the agreement report.
   - Padding 0 on both sides (MoveIt default; checked that pcp_moveit_config sets none).
   - gtests: home configuration free; a known self-colliding configuration; a configuration with
     a link inside a shelf box; SRDF-disabled pair count; active pair count == MoveIt's.
3. `shelf_boxes_loader` node: publishes the same YAML as `moveit_msgs/CollisionObject` boxes to the
   bench/agreement node's planning-scene monitor (own namespace, so move_group's scene in the sim
   is untouched).
4. Agreement test `collision_agreement` (+ `launch/agreement.launch.py`, no Gazebo):
   MoveItCpp's planning-scene monitor (no `sensors_3d`, so no octomap updater) with the boxes from
   the loader vs the direct checker, on 500 seeded uniform configurations; both checkers timed per
   call. `results/collision_agreement.csv` keeps every row (q, verdicts, colliding pairs from each,
   min distance from each); docs/results.md gets the paragraph. Nothing is tuned to raise agreement.
   **CHECKPOINT 2** (unit tests, CSV, agreement %).
5. Direct planner (`planner.hpp/.cpp`): 6-D real-vector joint space with URDF bounds, SimpleSetup,
   the collision module as validity checker, RRTConnect, 10 s, `simplifySolution(budget)`.
   Settings are READ from `pcp_moveit_config/config/ompl_planning.yaml` (default planner config of
   `ur_manipulator`, its `range`, and `longest_valid_segment_fraction`) so they cannot drift.
   Decision D2 (metric) below. Records plan time, simplify time, validity checks, median check time.
6. Queries `results/queries_boxes.json`: seed 0, 100 queries, v0.1 generator on the box shelf; gtest
   regenerates it byte for byte; every start/goal validated by both checkers at run time, rejected
   queries skipped and counted (decision D3).
7. Time parameterisation (`traj.hpp/.cpp`): R1's header-only trapezoidal generator copied into
   `include/pcp_direct/traj/trapezoidal.hpp` with an origin comment; limits from
   `pcp_direct/config/joint_limits.yaml` (decision D4); output `trajectory_msgs/JointTrajectory`
   sampled with the 1/3-rule phase boundaries included. gtests: limits respected, waypoints hit,
   durations.
8. Execution (`exec.hpp/.cpp`): FollowJointTrajectory client on `/joint_trajectory_controller`;
   MoveIt side `MoveItCpp::execute()` (trajectory execution manager, same controller). Both record
   sim-time duration, max tracking error = |/joint_states − controller reference| (reference from
   the JTC's `controller_state`, matched by stamp, the same for both pipelines), success = action
   SUCCEEDED and final error ≤ 0.01 rad. After every execution the direct planner moves the arm to
   the next run's start; failures to return are counted.
9. Cross-validation: every direct path densified to 0.01 rad and checked by MoveIt's scene; every
   MoveIt path by the direct checker (`crossval_ok`), plus each pipeline's own dense check.
10. `bench_direct` + `scripts/bench_direct.sh` (100 queries × {direct, moveit} × RRTConnect ×
    unconstrained × 1 rep; `execute:=false` mode for CI). CSV columns exactly as in the override,
    extras appended at the end. **Dry run 5 queries (needs Gazebo: ask first), project the total,
    CHECKPOINT 3; full run only on Giri's go (Friday 20:00 IST).**
11. `scripts/aggregate_direct.py`: median/p95 per column, Wilson 95 % for success, paired
    log-log scatter of planning time; tables into docs/results.md.
12. README section, fairness paragraph, "where it fails" items, architecture lane, CHANGELOG.
13. CI on the branch: collision + query-determinism gtests, 5-query plan-only mini-benchmark
    (no Gazebo, < 3 min), agreement on 50 configurations in the ci_smoke step. Merge only if CI is
    green and the 100-query table exists, and only after asking; else defer by Saturday 12:00 IST.
    **CHECKPOINT 4.**
14. SHOULD (only after MUST is green): keep-upright via `ompl::base::Constraint` +
    `ProjectedStateSpace`; shared octomap as one `coal::OcTree`.

### Decisions to confirm (D1–D5)

- D1 boxes file: reuse `src/pcp_scenes/config/shelf.yaml` (already the one definition used by the
  Gazebo world and the v0.1 generator) instead of adding `config/shelf_boxes.yaml`.
- D2 metric: MoveIt's joint state space measures distance as the SUM of per-joint distances (L1)
  and its extent as the SUM of joint ranges (69.1 rad); OMPL's plain `RealVectorStateSpace` uses
  the Euclidean norm (extent 28.8 rad). With the same `longest_valid_segment_fraction` (0.0003) and
  `range` (2 rad) the plain space would check every 0.0086 rad (L2) instead of every 0.021 rad
  (L1) and take differently sized RRT steps. Plan: subclass `RealVectorStateSpace` and override only
  `distance()` and `getMaximumExtent()` to MoveIt's definitions, so the same numbers mean the same
  thing. Plain L2 run kept as an ablation only if time allows.
- D3 queries: the v0.1 generator already sampled against the shelf BOXES (padded 3 cm), not the
  octomap, so seed 0 regenerates a `queries_boxes.json` byte-identical to `queries.json`. Keep it
  that way (separate file name per table, identical content, both determinism-tested).
- D4 limits: `joint_trajectory_controller` in `ur_simulation_gz/config/ur_controllers.yaml` has no
  velocity/acceleration limits (only path 0.2 rad / goal 0.1 rad tolerances). MoveIt's TOTG uses
  the URDF velocity (π rad/s) and `ur_moveit_config` acceleration (5 rad/s²); the direct
  pipeline's `joint_limits.yaml` uses exactly those, so both parameterisers share limits.
- D5 sim: `scripts/with_sim.sh` + v0.1 `sim.launch.py` unchanged (move_group keeps building its
  octomap in the background; v0.2 does not read its scene). Both pipelines share that background
  load.

### Source facts that shape v0.2 (read from tags moveit2 2.15.2, ompl 2.0.2, pinocchio v4.1.0, coal v3.0.3)

Read-only research by a sub-agent (shallow clones of the exact tags, installed headers diffed
against them); the items below are the ones the code depends on. File:line refs are in the tags.

- MoveIt `res.planning_time` = OMPL solve time + simplification time; interpolation and the
  response adapters (TOTG) are outside it (`model_based_planning_context.cpp` 453-461). The
  simplifier gets `allowed_planning_time − plan time` with `atLeastOnce=true`.
- After simplification MoveIt interpolates the path to every state the motion validator checks
  (`pg.interpolate()`, `validSegmentCount` per segment); `max_solution_segment_length` is never
  set in 2.15. TOTG then runs on that dense path.
- MoveIt's joint space: distance = Σ|Δq| (distance factor 1 per revolute joint), extent = Σ ranges,
  linear interpolation (`joint_model_group.cpp` 455-490, `revolute_joint_model.cpp`). OMPL
  `RealVectorStateSpace`: L2 distance, extent = diagonal norm. → decision D2.
- `longest_valid_segment_fraction` is set through `SpaceInformation::params()` and ends in
  `StateSpace::setLongestValidSegmentFraction`, i.e. the same call as
  `setStateValidityCheckingResolution`. `range` is set through the planner params and compared with
  the space's own distance.
- Motion validator: OMPL's default `DiscreteMotionValidator` (checks the end state, then bisects
  the interior states; the start is assumed valid).
- Validity checker (unconstrained): bounds → (no path constraints) → `checkCollision` with
  `group_name` set, no contacts, no distance; world check first, then self check with the ACM.
- Default collision detector is FCL; meshes are `fcl::BVHModel<OBBRSS>`; padding 0, scale 1 unless
  `robot_description_planning.default_robot_padding/scale` are set (they are not here).
- Pinocchio `addAllCollisionPairs` keeps a pair iff the two `parentJoint`s differ;
  `removeCollisionPairsFromXML` handles `<disable_collisions>` only, mapping link names to BODY
  frames (fixed-joint merges handled). coal meshes are `BVHModel<OBBRSS>`; `security_margin` 0;
  Pinocchio sets `distance_upper_bound = security_margin + 1e-6`.
- Pinocchio 4.1 `GeometryObject(name, parent_joint, parent_frame, placement, geometry)`; the old
  argument order is deprecated. `coal::Box(x, y, z)` takes full side lengths.
- MoveItCpp `PlanRequestParameters` defaults to 5 parallel attempts with hybridization when loaded
  from parameters; v0.1 and v0.2 set `planning_attempts = 1` explicitly.
- MoveIt's goal is a `GoalLazySamples` (background sampling thread) on a joint goal with
  tolerance DBL_EPSILON; the direct planner uses a plain goal state.
- `ompl::RNG::setSeed` does not seed MoveIt's state sampler (it uses
  `random_numbers::RandomNumberGenerator`), so MoveIt plans are not seed-reproducible.
- OMPL 2.0: `PathSimplifier::shortcutPath` was renamed `partialShortcutPath`; the SimpleSetup,
  RRTConnect, SpaceInformation and RealVector headers used here are otherwise unchanged from 1.6.
- The expanded URDF has no `<safety_controller>`, so MoveIt's joint bounds equal the `<limit>`
  values Pinocchio reads (±2π, elbow ±π); a test asserts it.

### Progress log (session 2, overnight)

- 02:37 Checkpoint 1 logged (overnight mode: no waiting). 02:59 Checkpoint 2 logged.
- Collision module + 8 gtests pass first time; active pairs = MoveIt's checked pairs (62 = 62);
  SRDF removes 7 geometry pairs (4 of its 11 entries involve links without geometry).
- Build gotcha: Pinocchio's CMake config loads eigenpy's own FindBoost, which leaves
  `Boost_USE_DEBUG_RUNTIME` set; MoveIt's later `find_package(Boost COMPONENTS chrono)` then
  rejects the release libraries. Fix: find pinocchio/coal last and `unset(Boost_USE_DEBUG_RUNTIME)`.
- Build gotcha: `moveit_planners_ompl` 2.15.2 exports `include/moveit_planners_ompl` as an
  include dir, which does not exist (headers are in `include/moveit_planners`) → CMake generate
  error. bench_direct links `libmoveit_ompl_interface` + that header dir directly.
- MoveItCpp needs `planning_pipelines.pipeline_names: [ompl]` as a parameter (pcp_benchmark's
  YAML has it; the agreement launch failed without it).
- Agreement timing must include forward kinematics on both sides (MoveIt's OMPL checker sets the
  state and updates transforms per check); first version timed MoveIt without FK.
- Agreement: 500/500, identical pair lists, identical min distances; boundary probe 180/194, all
  14 on forearm/wrist_3 at 1–5 nm (coal says contact, FCL says free). Commit 4412768.
- 03:09 the session was killed by systemd-oomd during a `-j4` build (swap full, three sessions).
  No committed work lost; uncommitted exec/bench files were complete on disk. From 03:35 builds
  run `MAKEFLAGS=-j2 colcon build --parallel-workers 1` after waiting for ≥ 5 GB MemAvailable.
- Plan-only smoke (3 queries, no Gazebo) works: all cross-checks clean. **Finding:** with the
  same OMPL code, the direct simplifier takes 0.33–0.45 s against MoveIt's 0.06–0.13 s, and the
  direct RRTConnect creates states more slowly; per-check cost differs only 1.3× (agreement
  test). Hypothesis: MoveIt checks fewer states, i.e. its effective check spacing is not the
  0.021 rad derived from the formula. bench_direct now reads MoveIt's configured OMPL context
  (extent, longest valid segment, range, validator) and records it in the meta file instead of
  assuming it.
- **Hypothesis rejected (04:20):** MoveIt's configured OMPL context, read from the context
  `plan()` builds, is identical to the direct planner's: extent 69.115 rad, longest valid segment
  0.02073 rad (fraction 0.0003), RRTConnect range 2, `DiscreteMotionValidator`. The "same
  settings" claim is now measured (recorded in every bench_direct `.meta.json`).
- **Where the time goes (probe, 3 dev queries):** MoveIt's planner core makes about as many
  validity checks as the direct planner (9.4k vs 10.3k, 10.8k vs 11.2k, 7.9k vs 6.3k) but each
  costs 9–21 µs inside MoveIt's planner vs 50–70 µs in the direct one. Explanation (not yet
  profiled): Pinocchio's `computeCollisions` runs coal's narrow phase on every one of the 62
  active pairs per check, while MoveIt's FCL environment culls pairs with an AABB-tree broadphase
  and only runs the narrow phase on overlapping bounding volumes. The agreement microbenchmark
  (random configurations, 69 % colliding → early exit, cold caches) understates the gap (49 vs
  61 µs). Next step (not this week): `pinocchio::BroadPhaseManager` / coal broadphase in the
  direct checker. The first probe version wrapped MoveIt's checker generically and segfaulted:
  MoveIt static-casts the space's validity checker to `ompl_interface::StateValidityChecker`, so
  the counter subclasses that type.
- **Gazebo execution check (2 queries, 04:27, gazebo lock):** both pipelines execute at full
  speed (π rad/s, 5 rad/s²) without aborts; max tracking error 0.034–0.054 rad, final error
  < 0.0005 rad, 0/4 repositioning failures, 55 s. v0.1's "needs 15× slow-down" was about
  replaying TOTG trajectories that clipped the shelf; at these limits the controller tracks.
  So decision D6 (common scaling) is not needed. MoveIt's query-0 trajectory failed the dense
  0.01 rad re-check with BOTH checkers (TOTG blends between validated waypoints), as in v0.1.
- **Commit hygiene slips (not rewritten):** dd23b65 also contains the executor
  (`exec.hpp/.cpp`) because the hook aborted its own commit; d6cafb2 also contains the CI and
  Dockerfile changes, staged during an earlier aborted commit. From now on `git reset` before
  staging each commit.
- Dry run (5 queries, all three locks) 04:45: 10/10, projection 26 min → full run 04:47.
- **Full run 1 (04:47–05:04, commit b3a170c) — kept, not published as the table:**
  `results/runs/bench_direct_run1_no_recovery.csv`. All 200 plans succeeded; executions were
  fine for queries 0–56, then at query 57 MoveIt's executed trajectory hit the physical shelf in
  Gazebo (its TOTG-blended path fails the dense 0.01 rad re-check with BOTH checkers), the
  controller aborted on its 0.2 rad path tolerance with the arm 1.84 rad from the goal and in
  contact, and my harness could not reposition from a colliding configuration ("Invalid
  start"), so every later run reports "arm not at the query start" (85/200 repositioning
  failures). The trigger is a real MoveIt result; the cascade is a harness bug. Fix: an escape
  step (sample the nearest collision-free configuration with ≥ 5 mm clearance at growing radii,
  move there in a straight line, then plan). The approved 100-query run is repeated with it.
- The check-cost probe crashed once (SIGSEGV, intermittent; the same binary completed it twice
  before and after). The chain now retries it once.
- **Full run 2 (05:19–05:47, commit 5432d60) = the published table** (`results/bench_direct.csv`,
  3e8b566): both 100/100 planned; executed to the goal direct 100/100, MoveIt 99/100 (query 63:
  path clean on both dense checks, but the arm lagged 0.207 rad > the 0.2 rad path tolerance →
  ABORTED, arm in contact → escape 0.010 rad). One direct REPOSITIONING move (q91 → q92 start)
  aborted the same way (escape 0.009 rad). 2 escapes, 0 failures. Solve + simplify median
  0.402 s vs 0.123 s (paired ratio 3.25, direct faster on 0/100); request → trajectory 0.402 s
  vs 0.169 s; MoveIt's `plan()` wall minus `planning_time` = 0.048 s median. Simplification >
  solve on 93/100 direct plans. Dense 0.01 rad re-checks: 97/100 direct and 91/100 MoveIt
  clean, and cross-validation equals the own-checker result on every row → no checker
  disagreement along any path; all misses are discretisation.
- **Probe (`scripts/check_probe.sh 20`, 7560eec):** checks per request median 9,564 direct vs
  8,768 MoveIt; median check 48.6 µs vs 12.4 µs; checker share of solve + simplify 99.6 % vs
  92.5 %. Two crash modes when one process probed many queries (SIGSEGV; then
  `free(): invalid pointer` even after restoring MoveIt's checker and keeping counters alive):
  reusing MoveIt's cached planning context after swapping its checker is not safe, so the
  script runs one query per process.
- Commit-message slip: 3e8b566 says "one escape"; the meta file counts 2 (above).
- **SHOULD keep-upright run (06:05–06:21, commit 1d8e13a, plan only, all three locks):** direct
  58/100 vs MoveIt 88/100; solve + simplify median 4.0 s vs 1.0 s; worst tilt 1.8° vs 3.2°.
  Dry run projected 17.9 min (budget 20); actual 16.3 min.
- **Slip (mine):** I edited `scripts/bench_direct.sh` (the `--octomap` branch) while that run was
  executing the script; bash reads scripts incrementally and hit a syntax error after the node had
  finished cleanly, so the final `.partial` → CSV rename did not run. The node's CSV and meta were
  complete (200 rows, `done:` logged); I did the two `mv`s by hand. Rule restated: never edit a
  script that a running job executes (NOTES session 1 already said so).

### Session 2 summary (until ≈ 06:40 IST)

- MUST list done and merged (b9e3d19, main CI green). SHOULD items both done on the branch:
  keep-upright (4a6186f, 1d8e13a, results 227d6a3) and shared octomap (b334471, f1e0a46,
  bba7c00, results 8816c04).
- Runtimes: collision agreement 500 ≈ 10 s; bench_direct 100 queries with execution 28.2 min;
  check probe 20 queries ≈ 61 s; keep-upright 100 plan-only 16.3 min; octomap 100 plan-only
  2.2 min incl. sim start; pcp_direct build 30–70 s (-j2); CI job ≈ 12 min.
- Commands that worked: `scripts/collision_agreement.sh 500`, `scripts/bench_direct.sh 100`,
  `scripts/check_probe.sh 20`, `PCP_CONSTRAINT=orientation scripts/bench_direct.sh 100 OUT
  --plan-only`, `scripts/bench_direct.sh 100 OUT --octomap`, `scripts/aggregate_direct.py`, all
  under `flock heavy flock build flock gazebo` with the 5 GB memory wait.

### CHANGELOG entry for session 2

See `CHANGELOG.md` → `[Unreleased]` → "Added / Fixed / Changed (v0.2)".

### Open questions for Giri (session 2)

1. Confirm D1–D5 (NOTES "Decisions to confirm"); D2 (L1 metric) and D4 (limits) change numbers.
2. The Perceptyne L3 phrase for the README row (still `TODO(Giri)`).
3. Branch protection on main, and bumping the Node.js-20 GitHub Actions?
4. The direct checker's per-check cost (no broadphase) explains the 3.3× planning gap; worth a
   v0.3 with `pinocchio::BroadPhaseManager`, re-run on the same queries?
5. One repetition per cell again: run 3 repetitions of the 100-query table (≈ 85 min of
   Gazebo, over the 75-min single-run budget) to put spread on the timing columns?
6. Keep the harness "escape" step as is (2 escapes in 400 executions + repositions), or report
   aborted executions without recovery?
7. `regenerate_all.sh` end to end is now ≈ 60 min (v0.1) + ≈ 50 min (v0.2): approve one run?

### Corrections after the summary

- **Correction (06:50, 5061abd):** the main table's "collision check" row had put the direct
  planner's in-planner median (50.8 µs) beside the agreement microbenchmark's MoveIt median
  (48.9 µs) as "like for like". It was not: the microbenchmark uses random, mostly colliding
  configurations with cold caches. The row now shows the probe (both inside their own planner,
  same 20 queries): 48.6 vs 12.4 µs. The CHECKPOINT 3 log keeps the original table as logged;
  MORNING.md notes the correction under its copy.
