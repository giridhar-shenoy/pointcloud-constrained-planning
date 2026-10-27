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
e47c849 2026-10-28 03:44:15 +0530 chore: add repository skeleton for pointcloud-constrained-planning
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
ac65c0b feat(bringup): add headless sim launch, camera bridge and CI smoke test
95c7665 feat(moveit_config): add OMPL constrained-planning and octomap config
f317975 feat(description): add UR5e xacro overlay with a fixed Gazebo rgbd_camera
de83912 feat(scenes): add shelf scene as one YAML shared by Gazebo and the benchmark
d1dab14 docs: expand tabs in checkpoint log so markdownlint passes
```

### MUST list remaining

C++ benchmark node (constraint builder, seeded query generator + determinism gtest, runner,
metrics gtest), dry run + 100-query run, aggregate.py + docs/results.md, GIF, README, docs
(architecture, planners), CI mini-benchmark step.

### Does 1 Nov hold?

Yes. CI green still depends on the billing decision.
