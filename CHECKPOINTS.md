# CHECKPOINTS — pointcloud-constrained-planning (R2)

## CHECKPOINT 0 — plumbing (2026-10-28 03:46 IST, overnight mode, not waiting)

Wall-clock spent so far: ~16 min (session start 03:30 IST).

### Checklist

| Item | Status |
|---|---|
| `docker build .` succeeds locally | Replaced by the MACHINE ADDENDUM with "the GitHub Actions job builds the image and is green". **BLOCKED**: GitHub refuses to start jobs on this account (billing), see MORNING.md item 1. The Dockerfile is not built anywhere yet. |
| CI run on GitHub is green on the skeleton | **BLOCKED (billing)**, run 40593288166: "The job was not started because recent account payments have failed or your spending limit needs to be increased". |
| `pre-commit run --all-files` passes | PASS (output below) |
| LICENSE has my name; THIRD_PARTY/NOTES/CHANGELOG/docs/scripts exist | PASS |
| Conventional commits, none dated before today | PASS |

Also run locally: an empty colcon workspace gives `colcon build` rc=0 and `colcon test-result --verbose`
rc=0 ("0 tests"), which is what the CI build and test steps would see on the skeleton.

GitHub: `gh repo create pointcloud-constrained-planning --private --source . --push` done;
description set; topics robotics, ros2, moveit2, gazebo, motion-planning, manipulation.
No branch protection, no visibility change (overnight rules).

### Command output

```text
$ .venv/bin/pre-commit run --all-files
fix end of files.........................................................Passed
trim trailing whitespace.................................................Passed
check for added large files..............................................Passed
check for merge conflicts................................................Passed
ruff check...........................................(no files to check)Skipped
ruff format..............................................................Passed
clang-format.........................................(no files to check)Skipped
yamllint.................................................................Passed
markdownlint-cli2........................................................Passed
$ grep -c "Giridhar J Shenoy" LICENSE
1
$ ls THIRD_PARTY.md NOTES.md CHANGELOG.md docs/architecture.md docs/results.md scripts/regenerate_all.sh
CHANGELOG.md
NOTES.md
THIRD_PARTY.md
docs/architecture.md
docs/results.md
scripts/regenerate_all.sh
$ git log --format="%h %ad %s" --date=iso
bfd430e 2026-10-28 03:44:15 +0530 chore: add repository skeleton for pointcloud-constrained-planning
$ gh run list --limit 1
completed   failure chore: add repository skeleton for pointcloud-constrained-planning  CI  main    push    40593288166 4s  2026-10-27T22:14:37Z
$ git status --short
?? MORNING.md
```

### MUST list remaining

Everything except the plumbing: sim + octomap + ci_smoke, C++ benchmark (constraints, queries,
runner, gtests), 100-query run, aggregate + results.md, GIF, README/docs, CI steps for smoke and
mini-benchmark.

### Does 1 Nov hold?

Yes for the code and local results. CI being green depends on the billing decision (MORNING.md).

## CHECKPOINT 1 — single-arm sim, octomap, ci_smoke (2026-10-28 03:59 IST, not waiting)

Wall-clock spent so far: ~29 min.

### Checklist (feature prompt step 2)

| Item | Status |
|---|---|
| `pcp_bringup/sim.launch.py` launches `ur_sim_control.launch.py` (ur5e, headless) with our description | PASS |
| Gazebo `rgbd_camera` → `/camera/points` (`sensor_msgs/PointCloud2` ← `gz.msgs.PointCloudPacked`) | PASS, 2.0 Hz (`ros2 topic hz`, 19-sample window), frame `camera_link` |
| `sensors_3d.yaml`: PointCloudOctomapUpdater, max_range 2.0, subsample 1, resolution 0.02, self-filter on | PASS |
| Octomap non-empty in `/monitored_planning_scene` | PASS, 219 925 bytes, res 0.02, frame `world`, ~7 s after launch |
| Cloud lands on the shelf | PASS: transformed into `world`, 44.9 % of points within 1.5 cm of a shelf box, 43.9 % on the floor, 11.4 % on the robot (x ≈ 0, removed by the self-filter) |
| Octomap takes part in collision checking | PASS: goal with the tool inside the middle board is rejected; move_group log: `Found a contact between '<octomap>' (type 'Object') and 'forearm_link'` |
| `scripts/ci_smoke.sh` plans one unconstrained RRTConnect query, asserts success, 240 s cap | PASS in 14 s |
| No Gazebo left running | PASS (`pgrep -f "gz sim"` empty; with_sim.sh checks the process group) |
| CI runs ci_smoke | Step added to ci.yml; cannot run (billing, MORNING.md item 1) |

Headless depth rendering works locally with `--headless-rendering` (EGL on the Iris Xe), as
SHARED.md recorded, so no point-cloud bag / git-lfs is needed locally. Whether it works on the
GitHub runner's llvmpipe is unverified until CI can run.

### Command output

```text
$ flock ~/work/.locks/gazebo.lock scripts/ci_smoke.sh
[with_sim] sim pgid=138096, log=/home/giri/work/r2/log/sim/sim_20261028_035754.log
[smoke] octomap OK: 219925 bytes, resolution 0.02 m, frame 'world', after 7.6 s
[smoke] plan to free goal OK: error_code 1, planning_time 0.046 s, 29 waypoints
[smoke] plan into the octomap board REJECTED (OK): error_code 99999
[with_sim] simulation group 138096 fully stopped
[ci_smoke] PASS in 14 s
exit=0
$ pgrep -af "gz sim" | grep -v pgrep || echo "no gz sim running"
no gz sim running
$ colcon test --packages-select pcp_scenes && colcon test-result --verbose
Summary: 3 tests, 0 errors, 0 failures, 0 skipped
$ git status --short
 M .github/workflows/ci.yml
 M Dockerfile
$ git log --oneline | head -5
32530ab feat(bringup): add headless sim launch, camera bridge and CI smoke test
4756d24 feat(moveit_config): add OMPL constrained-planning and octomap config
d66443b feat(description): add UR5e xacro overlay with a fixed Gazebo rgbd_camera
f9dce38 feat(scenes): add shelf scene as one YAML shared by Gazebo and the benchmark
98fcfc4 docs: expand tabs in checkpoint log so markdownlint passes
```

### MUST list remaining

C++ benchmark node (constraint builder, seeded query generator + determinism gtest, runner,
metrics gtest), dry run + 100-query run, aggregate.py + docs/results.md, GIF, README, docs
(architecture, planners), CI mini-benchmark step.

### Does 1 Nov hold?

Yes. CI green still depends on the billing decision.

## CHECKPOINT 2 — bench.csv and aggregate table (2026-10-28 05:46 IST, not waiting)

Wall-clock spent so far: ~2 h 16 min (since 03:30).

### What happened between checkpoints 1 and 2

1. First 100-query run (commit 6c12eeb, `longest_valid_segment_fraction` 0.005, automatic range):
   unconstrained 83/100, every failure a `ValidateSolution` rejection (-2); orientation 12/100.
   Diagnosed as my config: MoveIt's extent is the sum of joint ranges (69.1 rad for ±2π joints), so
   0.005 = a motion check every 0.35 rad. Kept as `results/ablation/bench_lvsf0.005.csv`.
2. Independent read-only code review (subagent) found two semantic problems — validation ran on
   TOTG-resampled waypoints, and the rotation-vector tolerance depended on yaw offset — plus four
   reporting issues. All fixed in ca8deb5; NOTES.md lists them.
3. RRTConnect `range` chosen on a separate tuning set (seed 1, 20 queries, orientation only):
   auto (13.8 rad) 1/20, 0.5 → 4/20, 1 → 6/20, 2 → 11/20 ⇒ range 2 rad. The evaluation queries were
   not used to choose it.
4. Final evaluation run (commit e3a1945), table below. Then the GIF (query 2, tilt 37° vs 2°).

### Checklist

| Item | Status |
|---|---|
| `results/bench.csv` exists, 100 queries × {none, orientation}, commit + seeds stamped | PASS (200 rows, commit e3a1945, query seed 0, OMPL seed 1) |
| Runtime ≤ 1 h | PASS: 7.7 min (dry run of 10 projected 16 min) |
| Constrained state space really used | PASS: `planning_context_manager: Using OMPL's constrained state space for planning.` once per orientation request at debug level; `mode` column derived from config |
| gtests: constraint builder, query determinism (regenerate + byte diff), metrics | PASS (22 colcon tests) |
| ci_smoke after the launch change | PASS (14 s) |
| GIF ≤ 3 MB, no RViz | PASS (1.6 MB, Gazebo camera + headless rendering) |
| CI | BLOCKED (billing, MORNING.md) |

### Command output

```text
$ flock ~/work/.locks/gazebo.lock scripts/ci_smoke.sh
[with_sim] sim pgid=216505, log=/home/giri/work/r2/log/sim/sim_20261028_054215.log
[smoke] octomap OK: 219925 bytes, resolution 0.02 m, frame 'world', after 7.0 s
[smoke] plan to free goal OK: error_code 1, planning_time 0.232 s, 29 waypoints
[smoke] plan into the octomap board REJECTED (OK): error_code 99999
[with_sim] simulation group 216505 fully stopped
[ci_smoke] PASS in 14 s
$ colcon test --packages-select pcp_scenes pcp_benchmark && colcon test-result --verbose
Summary: 22 tests, 0 errors, 0 failures, 0 skipped
$ python3 -m pytest -q scripts/tests
4 passed in 0.01s
$ .venv/bin/pre-commit run --all-files | grep -c Failed
0
$ pgrep -af "[b]in/gz sim" || echo none
$ python3 scripts/aggregate.py --csv results/bench.csv --print-only
Source: `results/bench.csv` (200 rows, 100 queries, commit `e3a1945`, query seed 0, OMPL seed 1, modes `constrained_state_space`, `joint_space`, 10 s per cell, octomap 4747 occupied 2 cm voxels, total wall time 7.7 min).
| Planner | Constraint | Success (95 % Wilson CI) | Planning time median / p95 (s) | Joint path median (rad) | TCP path median (m) | Max tool tilt median / worst (deg) | Executed path collision-free (dense check) |
|---|---|---|---|---|---|---|---|
| RRTConnect (range 2 rad) | none | 100/100 = 100 % (96 %–100 %) | 0.382 / 0.915 | 4.75 | 1.14 | 51.9 / 177.2 | 93/100 |
| RRTConnect (range 2 rad) | orientation | 84/100 = 84 % (76 %–90 %) | 2.014 / 8.257 | 5.00 | 1.55 | 1.4 / 3.6 | 83/84 |
$ git status --short
$ git log --oneline | head -12
4060411 data: 100-query evaluation with RRTConnect range 2 rad (commit e3a1945)
d36d94f feat(gif): constrained vs unconstrained GIF of evaluation query 2
15a9e89 fix(bringup): spawn the trajectory controller with its parameter file
6716297 chore(scripts): regenerate tuning, BiTRRT and sweep results in regenerate_all
daae907 feat(bench): octomap resolution sweep (2 cm vs 5 cm) tooling
58d47a3 feat(bench): prepare BiTRRT column and multi-planner aggregation
e3a1945 feat(bench): use RRTConnect range 2 rad, chosen on the seed-1 tuning set
7a7f105 docs: document success semantics, yaw coupling and range selection
ca8deb5 fix(benchmark): validate what is executed, centre the keep-upright yaw
987c515 data: keep the first 100-query run as the 0.005-resolution ablation
2793d5d docs: add architecture (Mermaid), planners and scripts pages
78a2448 feat(bench): tuning-set support and ablation sections in results.md
```

### MUST list remaining

None on the code/results side. CI-green cannot be verified (billing). Perceptyne L3 JD phrase
needs Giri (MORNING.md item 2). Next: SHOULD items (BiTRRT column, 2 cm vs 5 cm sweep), then final
docs and checkpoint.

### Does 1 Nov hold?

Yes. The only open MUST acceptance item is CI-green, which needs the billing/visibility decision.

## CHECKPOINT FINAL — MUST complete except CI-green; SHOULD done (2026-10-28 07:42 IST)

Wall-clock spent: ~4 h 12 min (03:30–07:42).

### Since checkpoint 2

- SHOULD: BiTRRT column (range chosen on the tuning set) and 2 cm vs 5 cm octomap sweep.
- The sweep exposed two wiring bugs: the benchmark node had been running its own octomap updater
  (it was given `sensors_3d`), and its scene subscription was remapped the wrong way (MoveItCpp
  listens on `planning_scene`, publishes on `monitored_planning_scene`). Fixed in 2951a4c and
  db74fcc; the evaluation, BiTRRT and sweep were re-run on move_group's map (same 4747 voxels).
- One post-fix evaluation was lost to a pre-commit stash swapping the tracked CSV under the running
  node; re-run into `results/tmp/` and `bench_planners.sh` now writes `.partial` then renames.
- CI workflow bug fixed by reading it (dfb0119): `docker run --rm` per step discarded rosdep's
  installs; now one long-lived container + `docker exec`.
- GIF re-recorded for the final CSV with a cross-compartment rule (query 5, tilt 29° vs 1°).

### Feature acceptance (MUST scope of the SPRINT OVERRIDE)

| Item | Status |
|---|---|
| CI green (build, gtest, ci_smoke, 5-query mini-benchmark) | **BLOCKED (billing)**; every step run locally and passing (below) |
| `results/queries.json`: 100 queries, reproducible from the seed (test regenerates and diffs) | PASS (`QueryDeterminism.CommittedQueriesRegenerateExactly`) |
| `results/bench.csv`: RRTConnect × {none, orientation} × constrained state space × 1 rep, 10 s | PASS, 200 rows, commit 00e1d72: none 100/100, orientation 79/100 (70–86 %) |
| ≤ 1 h benchmark | PASS: 8.2 min |
| docs/results.md: success with CIs, planning time median/p95, path lengths | PASS (generated by aggregate.py) |
| GIF: constrained vs unconstrained through the shelf, no RViz | PASS (2.3 MB) |
| README shape + NUS/CMU/Perceptyne table; architecture.md Mermaid; planners.md | PASS, except the Perceptyne JD phrase (MORNING.md item 2) |
| No planner parameters hidden in code | PASS: all in `ompl_planning.yaml`; request-level protocol values (1 attempt, scaling 1.0) are in the node and listed in planners.md |
| `regenerate_all.sh` reproduces everything; runtime in NOTES | Script complete; end-to-end run NOT done (≈ 60 min > 30 min ⇒ needs approval, MORNING.md item 3); component runtimes recorded in NOTES |
| Dual-arm, TAMP, post-processing, line/box, PRM, 3 reps | NOT THIS WEEK (README "What I'd do next") |

### Command output (CI-equivalent sequence, run locally at 07:40)

```text
$ .venv/bin/pre-commit run --all-files
fix end of files Passed
trim trailing whitespace Passed
check for added large files Passed
check for merge conflicts Passed
ruff check Passed
ruff format Passed
clang-format Passed
yamllint Passed
markdownlint-cli2 Passed
$ colcon build --symlink-install --parallel-workers 2
Summary: 5 packages finished [0.80s]
$ colcon test && colcon test-result --verbose
Summary: 22 tests, 0 errors, 0 failures, 0 skipped
$ python3 -m pytest -q scripts/tests
4 passed in 0.01s
$ scripts/ci_smoke.sh
[smoke] octomap OK: 219925 bytes, resolution 0.02 m, frame 'world', after 8.9 s
[smoke] plan to free goal OK: error_code 1, planning_time 0.294 s, 38 waypoints
[smoke] plan into the octomap board REJECTED (OK): error_code 99999
[with_sim] simulation group 245513 fully stopped
[ci_smoke] PASS in 16 s
$ scripts/bench_planners.sh 5 results/tmp/ci_bench5.csv
[with_sim] simulation group 245939 fully stopped
[bench_planners] 10 rows in results/tmp/ci_bench5.csv (commit e1b0aaa) in 41 s
elapsed 41 s
rows: 10
results/tmp/ci_bench5.csv
results/tmp/ci_bench5.csv.meta.json
|---|---|---|---|---|---|---|---|
| RRTConnect (range 2 rad) | none | 5/5 = 100 % (57 %–100 %) | 0.270 / 0.299 | 3.71 | 1.02 | 42.1 / 74.0 | 5/5 |
| RRTConnect (range 2 rad) | orientation | 4/5 = 80 % (38 %–96 %) | 3.702 / 4.017 | 3.98 | 1.24 | 0.9 / 1.8 | 4/4 |
$ pgrep -af "[b]in/gz sim" | grep r2 || echo none
none
```

### MUST list remaining

Only CI-green (billing) and the Perceptyne phrase. Both need Giri.

### Does 1 Nov hold?

Yes: code, results and docs are done; 1 Nov needs only the two decisions in MORNING.md.

## v0.2 CHECKPOINT 1 — CI green on main; branch created (2026-10-29 02:37 IST, overnight mode)

Giri went to sleep before answering; per the overnight rules this report is logged and work
continues. Items that needed his answer are under WAITING FOR GIRI in MORNING.md.

```text
$ gh repo view --json visibility --jq .visibility
PUBLIC
$ gh run view 40649038811 --json status,conclusion,headSha    # first run that started, b587d57
completed success b587d57720a2a5dd17d1d90525f48ba9a1beb9ea
  colcon test-result: Summary: 22 tests, 0 errors, 0 failures, 0 skipped
  [smoke] octomap OK: 237755 bytes, resolution 0.02 m, frame 'world', after 7.9 s   (GitHub runner, Mesa)
  [ci_smoke] PASS in 16 s; mini-benchmark 10 rows
$ gh run view 40767818396 ...   # main 30677c4 (Step 0 notes)
completed success; Summary: 22 tests, 0 errors, 0 failures, 0 skipped; [ci_smoke] PASS in 13 s;
[bench_planners] 10 rows in results/tmp/ci_bench5.csv (commit 30677c4) in 26 s
$ gh run view 40767886104 ...   # feat/direct-pipeline dd5333a (plan)
completed success
$ git branch -vv
* feat/direct-pipeline dd5333a [origin/feat/direct-pipeline] docs: plan the v0.2 direct OMPL + Pinocchio/coal pipeline
  main                 30677c4 [origin/main] docs: record the first CI run on main (green, no fixes needed)
$ git status
On branch feat/direct-pipeline; nothing to commit, working tree clean
```

- Step 0 needed no Dockerfile or ci.yml fix (NOTES.md, "Session 2 … Step 0").
- Plan + decisions D1–D5 in NOTES.md (branch); source facts from the research sub-agent recorded
  (fe1c03b). D1–D5 proceed on the recommendation; listed in MORNING.md for confirmation.
- Wall-clock so far: ≈ 1 h (CI watch, reading v0.1, research, plan).
- Left: the whole v0.2 MUST list (collision module → agreement → planner → traj → exec → bench →
  docs → CI → merge).
- Does Sunday 1 Nov hold? Yes, if the dry run projects ≤ 75 min tonight.

## v0.2 CHECKPOINT 2 — collision module, agreement (2026-10-29 03:25 IST, overnight mode)

```text
$ build/pcp_direct/test_collision
[==========] 8 tests from 1 test suite ran.
[  PASSED  ] 8 tests.
  LoadsTheSevenShelfBoxes, JointBoundsMatchMoveIt, SrdfRemovesTheDisabledGeometryPairs (7),
  ActivePairsAreExactlyThePairsMoveItChecks (62 = 62), HomeConfigurationIsFree,
  FoldedElbowSelfCollides, ToolInsideTheMiddleBoardCollidesWithIt, IsValidCountsAndTimesCalls
$ colcon test --packages-select pcp_benchmark pcp_direct && colcon test-result
Summary: 31 tests, 0 errors, 0 failures, 0 skipped
$ flock heavy flock build flock gazebo scripts/collision_agreement.sh 500 results/collision_agreement.csv 0
[collision_agreement] 500/500 agree (100.00 %), boundary 180/194, pair sets equal True, commit 374b246
  agreement 500/500 = 100.00 % (MoveIt colliding 343, direct colliding 343);
  median check MoveIt 48.9 us, direct 61.2 us   (both incl. forward kinematics)
  boundary probe: 180/194 brackets agree (92.78 %)
```

**Agreement % (the item to show Giri): 100.00 % (500/500).** Detail:

- Both checkers name the identical pair list on all 500 configurations, and on the 157 free ones
  their minimum distances are equal to the 1e-6 m printed (FCL in MoveIt vs coal through
  Pinocchio; coal is a fork of FCL via hpp-fcl, so the two share algorithms).
- Uniform samples never come closer than 5.4 mm to contact, so a supplementary boundary probe
  bisects every verdict change between consecutive samples to a 1e-7 rad bracket: 180/194 agree.
  All 14 differences are the same mesh pair, forearm_link/wrist_3_link, where FCL measures
  1–5 nm of separation and coal says "contact". That is 200–1000× below coal's GJK tolerance
  (1e-6 m) and below Pinocchio's `distance_upper_bound` (1e-6 m). Nothing was tuned.
- MoveIt is faster per check (48.9 vs 61.2 µs median); see NOTES.md for what each includes.

Files: `results/collision_agreement.csv` (+ `.meta.json`), `results/collision_agreement_boundary.csv`
(commit 4412768). Wall-clock this checkpoint: ≈ 50 min. Left: planner, traj, exec, bench, docs,
CI, merge. 1 Nov holds.
