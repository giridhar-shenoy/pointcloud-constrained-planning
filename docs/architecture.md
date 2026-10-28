# Architecture

Language split: **C++17** for everything on the planning path (the benchmark node, constraint
builder, query generator, metrics; MoveIt and OMPL themselves); **Python** only for launch files,
the smoke-test orchestrator, the shelf-world generator and `scripts/aggregate.py` (tables and
plots). The planner never runs through Python.

## Nodes, topics and data flow

```mermaid
flowchart LR
  subgraph GZ["Gazebo Jetty (gz sim -s --headless-rendering)"]
    world["shelf.sdf<br/>(generated from pcp_scenes/config/shelf.yaml)"]
    cam["rgbd_camera sensor<br/>320x240, 2 Hz, on camera_link"]
    ur["UR5e + gz_ros2_control"]
  end

  subgraph ROS["ROS 2 Lyrical (ROS_DOMAIN_ID=12)"]
    bridge["ros_gz_bridge<br/>bridges.yaml"]
    rsp["robot_state_publisher"]
    cm["controller_manager<br/>joint_state_broadcaster<br/>joint_trajectory_controller"]
    subgraph MG["move_group (pcp_moveit_config)"]
      upd["PointCloudOctomapUpdater<br/>max_range 2.0 m, 2 cm voxels,<br/>self-filter 3 cm"]
      psm["PlanningSceneMonitor"]
      ompl["OMPL pipeline<br/>enforce_constrained_state_space"]
    end
    subgraph BN["pcp_benchmark (C++, MoveItCpp, ns /pcp_benchmark)"]
      snap["frozen scene snapshot"]
      pc["PlanningComponent<br/>RRTConnect x {none, orientation}"]
      met["metrics: joint / TCP length,<br/>max tilt on densified path"]
    end
  end

  queries[("results/queries.json<br/>generate_queries, seed 0")]
  csv[("results/bench.csv<br/>+ .meta.json")]
  agg["scripts/aggregate.py"]
  docs["docs/results.md, README table,<br/>docs/assets/*.png"]

  cam -- "gz.msgs.PointCloudPacked" --> bridge
  bridge -- "/camera/points (PointCloud2)" --> upd
  ur --> cm
  cm -- "/joint_states" --> rsp
  rsp -- "/tf" --> upd
  upd -- "octomap" --> psm
  psm -- "/monitored_planning_scene" --> snap
  snap --> pc
  queries --> pc
  pc --> ompl2["OMPL (in-process)"]
  ompl2 --> met
  met --> csv
  csv --> agg --> docs
  world --- cam
```

## Why it is wired this way

- **One octomap owner.** `move_group` runs the only `PointCloudOctomapUpdater`; the benchmark node
  receives the map through `/monitored_planning_scene`. RViz, the smoke test and the benchmark
  therefore see the same map, and the CPU pays for one updater. This needs care: MoveItCpp's
  planning-scene monitor starts a world-geometry monitor of its own, so if the benchmark node is
  given the `sensors_3d` parameters it silently builds a second octomap from the same camera.
  It was, until a 5 cm resolution sweep recorded a 2 cm octree in the benchmark's snapshot;
  `benchmark.launch.py` now strips those parameters, and every run records the snapshot's octree
  resolution in its `.meta.json`.
- **Frozen snapshot.** The benchmark waits until the number of occupied voxels has not changed for
  3 s, clones the planning scene once, and passes that clone to every
  `PlanningComponent::plan(params, scene)` call. All 200 cells see the identical map, so differences
  between constraint types are not differences between octomap updates.
- **Own namespace.** MoveItCpp's planning-scene monitor publishes its scene on the relative topic
  `planning_scene`. In the root namespace that is `/planning_scene`, which `move_group` applies as a
  diff, which it then republishes on `/monitored_planning_scene`, which MoveItCpp applies, and so on.
  The node runs in `/pcp_benchmark` with `monitored_planning_scene` and `joint_states` remapped to the
  global topics.
- **Queries without a ROS graph.** `generate_queries` is a plain C++ program: URDF + SRDF + shelf
  YAML + seed in, JSON out. That is what makes the determinism test possible (the test runs the same
  function and compares to the committed file byte for byte).
- **One shelf definition.** `pcp_scenes/config/shelf.yaml` drives both the Gazebo world (through
  `make_world.py`, checked by a test) and the generator's collision model.
- **Teardown.** Every script that starts Gazebo goes through `scripts/with_sim.sh`, which starts the
  launch in its own process group, kills the whole group afterwards and fails if anything survives.

## Frames

`world` is the root of the URDF (UR base at the origin). `camera_link` is fixed to `world` at
(−0.5, −0.6, 1.3) m looking at the shelf; the Gazebo point cloud arrives in `camera_link`
(x forward), which was checked by transforming a live cloud into `world` (45 % of points within
1.5 cm of a shelf box, 44 % on the floor, the rest on the arm). The octomap frame is `world`.
