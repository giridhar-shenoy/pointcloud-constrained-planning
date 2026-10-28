# Changelog

All notable changes to this project are documented in this file.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project
adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Repository skeleton: MIT license, Dockerfile on `ros:lyrical-ros-base`, devcontainer, GitHub
  Actions CI, pre-commit (ruff, clang-format, yamllint, markdownlint), docs and scripts stubs.
- `pcp_scenes`: shelf scene as one YAML shared by the Gazebo world generator and the query
  generator, with a drift test.
- `pcp_description`: UR5e (`ur_simulation_gz`) plus a fixed Gazebo `rgbd_camera`.
- `pcp_moveit_config`: OMPL with `enforce_constrained_state_space`, PointCloudOctomapUpdater
  (2 cm, 2.0 m range, self-filter), KDL.
- `pcp_bringup`: headless `sim.launch.py`, camera bridge, smoke check and demo recorder.
- `pcp_benchmark` (C++17, MoveItCpp): keep-upright constraint builder, deterministic
  damped-least-squares query generator (`results/queries.json`, 100 queries, seed 0), runner on a
  frozen octomap snapshot, path metrics (joint/TCP length, max tilt on the densified path), gtests.
- Scripts: `with_sim.sh` (process-group teardown), `ci_smoke.sh`, `bench_planners.sh`,
  `aggregate.py` (Wilson intervals, tables, plots), `record_gifs.sh`, `regenerate_all.sh`.
- Results: `results/bench.csv` (RRTConnect, range 2 rad × {none, orientation} × 100 queries),
  tuning set `results/tuning/` (seed 1, 20 queries) used to choose `range`, ablation
  `results/ablation/bench_lvsf0.005.csv`.
- GIF `docs/assets/constrained_vs_unconstrained.gif` recorded from Gazebo with headless rendering.
- SHOULD items: BiTRRT column (range chosen on the tuning set) and a 2 cm vs 5 cm octomap sweep.
- Docs: README, `docs/architecture.md`, `docs/planners.md`, `docs/results.md`.

### Fixed

- `longest_valid_segment_fraction` 0.005 → 0.0003: MoveIt's extent is the sum of joint ranges
  (69.1 rad for the UR5e), so 0.005 checked motions only every 0.35 rad.
- `ValidateSolution` now runs before time parameterization (TOTG blending capped at 0.01 rad), and
  executed trajectories are re-checked at 0.01 rad (`dense_collision_free`).
- Keep-upright reference yaw centred between start and goal (rotation-vector yaw/tilt coupling).
- `joint_trajectory_controller` is spawned with `--param-file` (it never initialized under
  ros2_control 6.10 through `ur_sim_control`).
- Benchmark node exits with `_Exit` (MoveIt 2.15.2 `~MoveItCpp` segfault), launch overrides beat
  YAML wildcards, `with_sim.sh` waits only on real `gz sim` processes.
