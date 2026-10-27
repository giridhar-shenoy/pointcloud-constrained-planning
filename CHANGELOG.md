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
- Results: `results/bench.csv` (RRTConnect × {none, orientation} × 100 queries).
- Docs: README, `docs/architecture.md`, `docs/planners.md`, `docs/results.md`.
