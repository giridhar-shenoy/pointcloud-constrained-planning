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
- **Own namespace, one remap.** MoveItCpp's planning-scene monitor *listens* on the relative topic
  `planning_scene` and *publishes* its maintained scene on `monitored_planning_scene`. The node runs
  in `/pcp_benchmark` so its publisher cannot collide with `move_group`'s, and `planning_scene` is
  remapped to `/monitored_planning_scene` so it receives `move_group`'s full scene and diffs,
  octomap included (RViz is wired the same way). An earlier version remapped the publisher instead
  and planned on a second, self-built octomap; see the bullet above.
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

## v0.2: the direct pipeline beside MoveIt 2

Language split is unchanged: the direct pipeline (`src/pcp_direct`: collision checker, planner,
parameterisation, executor, benchmark node, box loader, agreement test) is C++17; only
`scripts/aggregate_direct.py` (tables and the figure) is Python.

```mermaid
flowchart LR
  yaml[("pcp_scenes/config/shelf.yaml<br/>7 boxes")]
  urdf[("robot_description<br/>robot_description_semantic<br/>(same strings MoveIt loads)")]
  omplyaml[("pcp_moveit_config/config/ompl_planning.yaml<br/>RRTConnectRange2, lvsf 0.0003")]
  limits[("pcp_direct/config/joint_limits.yaml<br/>pi rad/s, 5 rad/s^2")]
  q[("results/queries_boxes.json<br/>seed 0, 100 queries")]

  subgraph DIRECT["direct lane (pcp_direct libraries, no MoveIt)"]
    coll["CollisionChecker<br/>Pinocchio model + GeometryModel,<br/>coal meshes + coal boxes,<br/>addAllCollisionPairs - SRDF pairs"]
    plan["DirectPlanner<br/>SimpleSetup + RRTConnect on JointSpaceL1<br/>(MoveIt's L1 metric and extent),<br/>simplifySolution(10 s - plan time)"]
    traj["parameterise()<br/>R1 trapezoids, rest-to-rest,<br/>-> trajectory_msgs/JointTrajectory"]
    exe["TrajectoryExecutor<br/>FollowJointTrajectory client"]
  end

  subgraph MOVEIT["MoveIt lane (MoveItCpp in the same process, ns /pcp_direct)"]
    loader["shelf_boxes_loader<br/>CollisionObject boxes"]
    psm["PlanningSceneMonitor<br/>(no 3-D sensors: no octomap)<br/>frozen scene snapshot"]
    pc["PlanningComponent::plan()<br/>OMPL pipeline + adapters + TOTG"]
    tem["MoveItCpp::execute()<br/>TrajectoryExecutionManager"]
  end

  subgraph SIM["Gazebo (v0.1 sim.launch.py, unchanged)"]
    jtc["joint_trajectory_controller<br/>/follow_joint_trajectory<br/>/controller_state"]
    js["/joint_states"]
  end

  mon["tracking monitor<br/>/joint_states vs controller reference<br/>(same for both lanes)"]
  xval["cross-validation<br/>direct path -> MoveIt checker,<br/>MoveIt path -> direct checker,<br/>0.01 rad steps"]
  csv[("results/bench_direct.csv<br/>+ .meta.json")]
  agg["scripts/aggregate_direct.py"]

  yaml --> coll
  yaml --> loader --> psm
  urdf --> coll
  urdf --> psm
  omplyaml --> plan
  omplyaml --> pc
  limits --> traj
  q --> plan
  q --> pc
  coll --> plan --> traj --> exe --> jtc
  psm --> pc --> tem --> jtc
  jtc --> js --> mon
  jtc --> mon
  plan --> xval
  pc --> xval
  coll --> xval
  psm --> xval
  xval --> csv
  mon --> csv
  exe --> csv
  tem --> csv
  csv --> agg
```

- **One process, one scene.** `bench_direct` hosts both lanes. MoveIt plans on a clone of its
  planning scene taken once all seven boxes have arrived from `shelf_boxes_loader`; the direct
  checker is built from the same URDF/SRDF strings and the same YAML. A scene with an octomap is
  refused.
- **Settings are read, then verified.** The direct planner reads MoveIt's `ompl_planning.yaml`;
  the node aborts if MoveIt's loaded parameters differ, and records the OMPL context MoveIt
  actually configures (extent, check spacing, range) in the `.meta.json`.
- **Same controller, same measurement.** Both lanes end at the sim's
  `joint_trajectory_controller`; tracking error for both is `/joint_states` against the
  controller's own reference (`controller_state`), matched by stamp. Before every run the direct
  planner moves the arm to that run's start; the order of the two lanes alternates per query.
- **Agreement test** (`collision_agreement`, no Gazebo): the same MoveItCpp scene and direct checker,
  500 seeded uniform configurations plus a boundary probe.
