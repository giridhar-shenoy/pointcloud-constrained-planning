# Planners and constraints

This page holds the settings, equations and method details behind the README: the v0.1 MoveIt 2
planner and its keep-upright constraint, the v0.2 direct pipeline and what "like for like" means
between the two, and the long form of every item in the README's
[Where it fails](../README.md#where-it-fails). Measured numbers are not typed here; each one is
linked to the generated table in [results.md](results.md) that holds it.

All v0.1 planner parameters live in
[`src/pcp_moveit_config/config/ompl_planning.yaml`](../src/pcp_moveit_config/config/ompl_planning.yaml);
the benchmark node sets only the request-level values listed under
[v0.1 pipeline and parameters](#v01-pipeline-and-parameters), from
[`src/pcp_benchmark/config/benchmark.yaml`](../src/pcp_benchmark/config/benchmark.yaml). The v0.2
direct planner reads the same `ompl_planning.yaml` (see [v0.2: the direct pipeline](#v02-the-direct-pipeline)).

## RRTConnect (the benchmark planner)

RRTConnect [Kuffner & LaValle 2000] grows two rapidly-exploring random trees, one from the start and
one from the goal, and after every extension tries to connect them greedily with straight-line
steps. It is probabilistically complete, not asymptotically optimal, and on easy queries it is
usually the fastest planner in OMPL because the greedy connect step closes the gap as soon as the
two trees can see each other. MoveIt then shortcuts the raw path (OMPL path simplification) and
re-interpolates it, which is why reported joint-space lengths are much shorter than raw RRT paths
but still not optimal.

With a path constraint and `enforce_constrained_state_space: true`, MoveIt plans in OMPL's
`ProjectedStateSpace` [Kingston, Moll & Kavraki 2019]: every sample and every interpolated state is
projected onto the constraint manifold `F(q) = 0` by Newton iterations using the constraint
Jacobian, and edges are built by stepping along the manifold (`delta`) and re-projecting. The
`connect` step of RRTConnect therefore becomes a walk along the manifold that can stall at
singularities or in narrow passages of the manifold, which is the likely cause of most
keep-upright failures ([F1](#f1-keep-upright-planning-fails-often)). Without path constraints the
same request uses the ordinary joint-space state space.

## BiTRRT (second planner column)

BiTRRT [Devaurs, Siméon & Cortés 2013] is the bidirectional version of Transition-based RRT: two
trees grow as in RRT, but each extension must pass a *transition test* that accepts a move to a
costlier state only with a probability set by a self-adjusting temperature, which makes the trees
follow valleys of a cost map. Two properties matter here. First, the trees are joined only when a
new node of one tree is within a single `range` step of the other tree; RRTConnect instead walks
greedily towards the other tree until it is blocked. Second, with MoveIt's default objective there
is no meaningful state cost, so the transition test mostly adds rejections without guidance. Both
predict that BiTRRT needs many more nodes than RRTConnect to close the gap, and in the projected
state space each node costs a projection. The first attempt reused RRTConnect's 2 rad range and
timed out on every unconstrained query it reached; BiTRRT's range is therefore chosen on the
tuning set by the same rule ([results.md](results.md#how-bitrrts-range-was-chosen-tuning-set)).
The chosen 2 rad (`BiTRRTRange2`, selected by
[`results/bitrrt.yaml`](../results/bitrrt.yaml)) keeps BiTRRT level with RRTConnect on the
keep-upright queries, but it still solves no unconstrained query within 10 s
([F2](#f2-bitrrt-is-not-a-drop-in-alternative)).

## Keep-upright orientation constraint

| Field | Value | Why |
|---|---|---|
| link | `tool0` | the flange; the queries' tool poses are flange poses |
| target | z axis along world −z, yaw = circular mean of the query's start and goal yaws | "keep a cup upright" under a top grasp; why the yaw matters: see below |
| parameterization | `ROTATION_VECTOR` | the only one OMPL's constrained state space implements (exponential coordinates of R_ee⁻¹·R_target, `ompl_constraints.hpp`) |
| x / y tolerance | 5°/√2 = 3.536° | for a rotation vector r, the tilt of the z axis is ≤ ‖(r_x, r_y)‖, so this square of tolerances lies inside the 5° cone: everything MoveIt accepts tilts ≤ 5° (checked by `test_constraints`, 2000 sampled states). It is conservative: a pure 4.9° tilt about x alone is rejected. |
| z tolerance | π | yaw is free |

**Formulation.** With r(q) = log(R_tool(q)ᵀ · R_target) (the rotation vector, angle × axis) and
per-axis tolerances t = (5°/√2, 5°/√2, π), the constraint is the equality F(q) = 0 with a dead band
per component: F_i(q) = r_i − t_i if r_i > t_i, −t_i − r_i if r_i < −t_i, and 0 otherwise.
Co-dimension 3 on the 6 joints. MoveIt's OMPL constrained state space uses exactly this form
(`OrientationConstraint` in `ompl_constraints.cpp` with `Bounds::penalty`); the v0.2 direct planner
reimplements it ([below](#keep-upright-in-the-direct-planner)).

**Yaw/tilt coupling.** The rotation vector mixes yaw into the x/y components: at a yaw offset ψ
from the reference, ‖(r_x, r_y)‖ ≈ tilt · (ψ/2)/sin(ψ/2). With a fixed reference yaw the admissible
tilt would shrink to ≈ 2.25° for a tool yawed by 180°, and half of the uniformly-yawed queries would
face a tighter constraint than advertised (found in code review; `test_constraints` pins it: a 3°
tilt is accepted with the reference at the tool's yaw and rejected with it 180° away). The reference
yaw is therefore set per query halfway between start and goal yaw, so |ψ| ≤ 90° at both ends
(factor ≤ 1.11). Effective admissible tilt: between ≈ 3.2° and 5° depending on tilt direction and
yaw offset, never more than 5°. Along the path yaw can still drift further from the reference
([F8](#f8-the-rotation-vector-couples-yaw-and-tilt)).

**Which constraints MoveIt can project.** At most one position constraint and one orientation
constraint, rotation-vector form only; the details are under
[F7](#f7-moveits-constrained-state-space-supports-a-subset-of-constraints).

## v0.1 pipeline and parameters

| Parameter | Value | Where |
|---|---|---|
| planner config | `RRTConnectRange2`: `type: geometric::RRTConnect`, `range: 2.0` (rad, in MoveIt's L1 joint metric); it is the group's `default_planner_config`. `RRTConnectkConfigDefault` (`range: 0.0`, OMPL's default of 20 % of the space extent, 13.8 rad here) and the 0.5 and 1 rad configs stay in the file for the tuning comparison. | `ompl_planning.yaml` (`planner_configs`; `ur_manipulator.default_planner_config`), selected by `benchmark.yaml` (`bench.planners`) |
| second column | `BiTRRTRange2`: `type: geometric::BiTRRT`, `range: 2.0`, MoveIt Setup Assistant defaults otherwise (`temp_change_factor` 0.1, `init_temperature` 100, `frontier_threshold` 0, `frontier_node_ratio` 0.1, `cost_threshold` 1e300) | `ompl_planning.yaml`, selected by `results/bitrrt.yaml` |
| `enforce_constrained_state_space` | `true` | `ompl_planning.yaml` (group `ur_manipulator`) |
| `projection_evaluator` | `joints(shoulder_pan_joint,shoulder_lift_joint)` | `ompl_planning.yaml` |
| `longest_valid_segment_fraction` | 0.0003 (MoveIt sums the joint ranges into the extent, 22π ≈ 69.1 rad for the UR5e, so this is ≈ 0.021 rad between collision checks; 0.005 would be 0.35 rad) | `ompl_planning.yaml` |
| request adapters | ResolveConstraintFrames, ValidateWorkspaceBounds, CheckStartStateBounds, CheckStartStateCollision | `ompl_planning.yaml` |
| response adapters | ValidateSolution, then AddTimeOptimalParameterization, DisplayMotionPath | `ompl_planning.yaml` |
| time parameterization | TOTG (time-optimal trajectory generation): `totg.path_tolerance` 0.01 rad (default 0.1), `resample_dt` 0.1 s, `min_angle_change` 0.001 | `ompl_planning.yaml` |
| planning time per cell | 10 s | `benchmark.yaml` (`bench.planning_time`) |
| planning attempts | 1 | `benchmark_main.cpp` (`PlanRequestParameters`, fixed by the benchmark protocol) |
| velocity / acceleration scaling | 1.0 / 1.0 | `benchmark_main.cpp` (only affects time parameterization, not the path) |
| OMPL seed | 1 (`ompl::RNG::setSeed` before the planner plugin loads; OMPL replaces 0 by 1) | `benchmark.yaml` (`bench.ompl_seed`) |
| repetitions | 1 per cell | `benchmark.yaml` (`bench.repetitions`) |
| octomap resolution | 0.02 m | `pcp_moveit_config/config/octomap.yaml` |
| octomap snapshot | frozen once the occupied-voxel count has not changed for 3 s (`octomap_settle`), within 180 s (`octomap_timeout`) | `benchmark.yaml` |

**What "success" means.** `ValidateSolution` runs first, on the planner's own waypoints (OMPL has
already checked the motions between them at the `longest_valid_segment_fraction` resolution), so
a success is a path that is collision-free against the snapshot and inside the path constraint at
every waypoint. Time parameterization runs afterwards with corner blending capped at 0.01 rad. In
the first version the order was reversed with MoveIt's default 0.1 rad blending, so validation saw
0.1 s resamples of a blended path rather than the planned one (found in code review). The node
also re-checks the time-parameterized trajectory (what a controller would execute) at 0.01 rad
joint steps and reports it as `dense_collision_free`. v0.1 does not execute; v0.2 does.

**Collision-check resolution.** MoveIt's state-space extent is the *sum* of the joint ranges. Five
UR5e joints span ±2π and the elbow ±π, so the extent is 5·4π + 2π = 22π ≈ 69.1 rad. MoveIt's usual
`longest_valid_segment_fraction: 0.005` therefore means a motion check only every
0.005 × 69.1 ≈ 0.35 rad, enough for a link to step over a 2 cm board; in the first 100-query run
every unconstrained failure was a `ValidateSolution` rejection
([ablation](results.md#ablation-collision-check-resolution-0005-moveits-usual-value)).
0.0003 gives 0.0003 × 69.1 ≈ 0.021 rad ([F6](#f6-the-default-collision-check-resolution-is-wrong-for-this-arm)).

**Query classes.** Every query is labelled by how its start and goal compartments relate: same
compartment, across the central divider, across the middle board, or diagonal. The per-class
tables in results.md use these labels.

## How `range` was chosen

RRTConnect's `range: 0` means 20 % of the same extent, 0.2 × 69.1 ≈ 13.8 rad per extension. Four
settings (automatic, 0.5, 1 and 2 rad) were compared on a separate tuning set (20 queries
generated with seed 1, disjoint from the seed-0 evaluation queries) with the orientation constraint
only; the winner (highest success, ties broken by median time) is the benchmark planner. The
evaluation set was not used to choose it. Table:
[results.md](results.md#how-rrtconnects-range-was-chosen-tuning-set-not-the-evaluation-set).
`regenerate_all.sh` regenerates the tuning CSVs but does not re-apply the rule
([F3](#f3-planning-is-very-sensitive-to-the-rrtconnect-range)).

## v0.2: the direct pipeline

The same 100 shelf queries planned twice: once by MoveIt 2 (MoveItCpp, as in v0.1, but on the box
shelf instead of the octomap), once by a pipeline written directly on the libraries in C++17
(`src/pcp_direct`): Pinocchio + coal for collision checking, OMPL's RRTConnect and path simplifier,
a trapezoidal time parameterisation and a FollowJointTrajectory client. It answers what MoveIt's
machinery costs over OMPL and a collision checker used directly, and whether two collision
checkers agree on the same scene. The node and data-flow diagram is in
[architecture.md](architecture.md#v02-the-direct-pipeline-beside-moveit-2).

### Components

- **Collision checker** (`collision.cpp`). A Pinocchio model and `GeometryModel` built from the
  same `robot_description` string MoveIt loads (coal collision meshes), plus the seven shelf boxes
  from `pcp_scenes/config/shelf.yaml` as coal boxes. Pairs: `addAllCollisionPairs()`, then the
  SRDF's disabled pairs removed with `pinocchio::srdf::removeCollisionPairsFromXML`
  (`collision.cpp:137–140`). The SRDF disables 11 link pairs; 4 of them involve links without
  collision geometry, so 7 geometry pairs are removed. What remains is the same set of 62 pairs
  that MoveIt's OMPL validity checker tests (`test_collision.cpp:103–109` compares the sets;
  `results/collision_agreement.csv.meta.json` records `direct_active_pairs` and
  `moveit_checked_pairs`). A validity check is `pinocchio::computeCollisions` with
  stop-at-first-collision, including forward kinematics, timed per call (`collision.cpp:162–171`).
  No padding, no broadphase.
- **Planner** (`planner.cpp`). OMPL `SimpleSetup` + `geometric::RRTConnect` in `JointSpaceL1`, a
  `RealVectorStateSpace` whose distance is Σ|Δq_i| and whose extent is Σ(upper_i − lower_i), the
  metric and extent of MoveIt's `JointModelGroup` (`planner.cpp:50–68`), so `range` and
  `longest_valid_segment_fraction` mean the same step and the same check spacing as in MoveIt. A
  fresh `SimpleSetup` per request, like MoveIt's planning context per request. Success is
  `EXACT_SOLUTION` within the 10 s timeout; the path is then simplified with MoveIt's budget rule,
  `simplifySolution` for max(0, timeout − solve time) (`planner.cpp:154–160`).
- **Time parameterisation** (`traj.cpp`, `traj/trapezoidal.hpp`). Joint-synchronised,
  rest-to-rest trapezoids at every simplified waypoint, from the profile generator of
  ur5e-teaching-controllers ([THIRD_PARTY.md](../THIRD_PARTY.md)), sampled every 0.1 s into a
  `trajectory_msgs/JointTrajectory`. Each segment uses the 1/3 rule (accelerate for T/3, cruise for
  T/3, decelerate for T/3). For a joint distance D over a duration T this needs a peak velocity of
  1.5 D/T and an acceleration of 4.5 D/T², so the shortest admissible duration for joint j is
  T_j = max(1.5 |D_j| / v_max, √(4.5 |D_j| / a_max)), and all joints of a segment share
  T = max_j T_j. It is not time-optimal.
- **Executor and tracking monitor** (`exec.cpp`). A FollowJointTrajectory action client to the
  simulation's `joint_trajectory_controller`, and a monitor that records the maximum over samples
  and joints of |q − q_ref|, with q from `/joint_states` and q_ref from the controller's own
  `controller_state`, matched by stamp. The same monitor measures MoveIt's executions.
- **Benchmark node** (`bench_direct_main.cpp`). Hosts both pipelines in one process. MoveIt
  (MoveItCpp, namespace `/pcp_direct`) plans with `PlanningComponent::plan()` through the same
  OMPL pipeline, adapters and TOTG as v0.1, on a planning-scene clone frozen once all seven boxes
  have arrived from `shelf_boxes_loader`, and executes with `MoveItCpp::execute()` (its
  trajectory execution manager). Before every run the direct planner moves the arm to that run's
  start (a repositioning move, not a measured run); the order of the two pipelines alternates per
  query. Each path is re-checked at 0.01 rad joint steps by its own checker and by the other
  pipeline's checker (cross-validation).

### Settings

| Setting | Value | Where |
|---|---|---|
| planner and range | MoveIt's `default_planner_config` for `ur_manipulator` (`RRTConnectRange2`, `range: 2.0`); any other planner type is refused | read from `pcp_moveit_config/config/ompl_planning.yaml` (`planner.cpp:21–47`) |
| `longest_valid_segment_fraction` | 0.0003 of the 69.1 rad extent ≈ 0.0207 rad between checks | same file; the node aborts if MoveIt's loaded value or default config differ (`bench_direct_main.cpp:471–481`) and records MoveIt's configured context (extent, check spacing, range) in the `.meta.json` |
| planning time | 10 s per plan, both pipelines (MoveIt: `allowed_planning_time`) | `pcp_direct/config/bench_direct.yaml` |
| simplification budget | timeout − solve time (MoveIt's rule) | `planner.cpp` |
| planning attempts (MoveIt) | 1 (the loaded default would be 5 parallel attempts) | `bench_direct_main.cpp:751` |
| velocity / acceleration scaling (MoveIt) | 1.0 / 1.0 | `bench_direct_main.cpp:753–754` |
| joint limits (trapezoids) | π rad/s and 5 rad/s² per joint: the URDF velocity limit and `ur_moveit_config`'s acceleration limit, i.e. the limits MoveIt's TOTG uses (unit-tested against MoveIt's robot model). The controller itself has no velocity or acceleration limits. | `pcp_direct/config/joint_limits.yaml` |
| trapezoid sampling step | 0.1 s (TOTG's `resample_dt`) | `bench_direct.yaml` (`sample_dt`) |
| goal tolerance | 0.01 rad: "reached the goal" and "at the start" (MoveIt's start tolerance) | `bench_direct.yaml` (`goal_tolerance`) |
| dense / cross-validation step | 0.01 rad | `bench_direct.yaml` (`dense_step`) |
| OMPL seed | 1, direct planner only (`ompl::RNG::setSeed`); MoveIt's sampler is not seeded ([F24](#f24-moveits-plans-are-not-seed-reproducible)) | `bench_direct.yaml` (`ompl_seed`) |
| controller | the simulation's `joint_trajectory_controller`, position command interface, path tolerance 0.2 rad, goal tolerance 0.1 rad | `ur_simulation_gz/config/ur_controllers.yaml` |
| padding | none in either checker | |

### Keep-upright in the direct planner

`KeepUprightConstraint` (`constraint.hpp`, `constraint.cpp`) is an `ompl::base::Constraint` with
ambient dimension 6 and co-dimension 3, with the [formulation](#keep-upright-orientation-constraint)
above: `error()` is the rotation vector of R_toolᵀ · R_target (`constraint.cpp:65–69`) and
`function()` the per-axis dead band (`constraint.cpp:71–80`), with the same tolerances and the
same per-query reference yaw (circular mean of start and goal yaws, `bench_direct_main.cpp:680`).
The planner wraps `JointSpaceL1` in OMPL's `ProjectedStateSpace` with a
`ConstrainedSpaceInformation` and OMPL's default `delta` (0.05), `lambda` (2), projection
tolerance (1e-4) and iteration limit (50), which is what MoveIt's
`enforce_constrained_state_space` uses as well (`planner.cpp:101–108`).

One difference from MoveIt is not in the formulation but in the Jacobian. `jacobian()`
(`constraint.cpp:82–103`) is central differences with h = 1e-6: one forward-kinematics call for the
active-bound signs (`:85`) plus two per joint (`:96`), so 13 calls per Jacobian, and two
heap-allocated `Eigen::VectorXd` per call (`:91–92`). MoveIt's `OrientationConstraint` computes its
Jacobian analytically, from the angular-velocity-to-rotation-vector map and the geometric Jacobian.
The unit test `JacobianMatchesFiniteDifferencesOutsideTheBand` compares this Jacobian with central
differences at a different step, so it checks the loop, not the error function. This is the first
suspect for the keep-upright gap ([F17](#f17-keep-upright-planning-is-weaker-in-the-direct-pipeline)).

Plan only: `PCP_CONSTRAINT=orientation scripts/bench_direct.sh 100
results/bench_direct_upright.csv --plan-only`.

### Octomap scene

With `--octomap` both pipelines plan on the octomap `move_group` built from the simulated camera
instead of the boxes. The direct checker reads `world.octomap` of the frozen scene's message with
`octomap_msgs::msgToMap` (MoveIt writes a full, not a binary, map) and adds it as one
`coal::OcTree`, so both pipelines plan on the identical map, and the v0.1 query set is reused
(`queries.json` and `queries_boxes.json` are byte-identical). Plan only, unconstrained:
`scripts/bench_direct.sh 100 results/bench_direct_octomap.csv --octomap`. On the octomap both
pipelines plan every query and the planning-time gap is smaller than on the boxes
([octomap table](results.md#both-pipelines-on-move_groups-octomap)). One reading is that FCL's
checks also get more expensive against an octree, but MoveIt's per-check cost on the octomap is
not exposed, so that is inferred from planning time, not measured
([F16](#f16-the-direct-pipeline-is-slower-at-planning)).

### Collision agreement test

`collision_agreement` (no Gazebo) builds the same MoveItCpp planning scene and direct checker and
asks both for a verdict on seeded configurations sampled uniformly within the URDF joint bounds
(seed 0, rounded to 1e-6 rad), using the request MoveIt's OMPL validity checker sends (arm group,
no padding). It also compares the colliding pair names and, on free configurations, the minimum
distance. Because uniform samples rarely come close to contact, a boundary probe bisects every
verdict change between consecutive samples to a bracket below 1e-7 rad per joint and asks the
direct checker at both ends. Both checks include forward kinematics; timing is one call per
configuration, wall clock, in the same process. Coal is a fork of FCL (through hpp-fcl), so the two
share their BVH and GJK algorithms: agreement is a consistency check, not independent verification.
Results: [results.md](results.md#collision-agreement-moveits-planning-scene-vs-the-direct-checker).

### Check-count probe

MoveIt's `planning_time` hides its internals, so `scripts/check_probe.sh 20` solves the first 20
queries again, one query per process, in the OMPL planning context MoveIt builds for `plan()`
(adapters bypassed), with MoveIt's `StateValidityChecker` subclassed to count and time every
`isValid` call (`CountingValidityChecker`, `bench_direct_main.cpp:214–250`), and with the direct
planner. MoveIt's `isValid` first returns a cached verdict when the state already carries one
(`isValidityKnown`); the probe counts and times those cached returns as checks too, while the
direct checker has no cache. That inflates MoveIt's check count and lowers its per-check time by an
amount that was not measured. Results:
[results.md](results.md#where-the-planning-time-goes-probe).

### What is identical, what differs, what MoveIt gives you

**What is identical.**

- Robot and URDF: the same `robot_description` string.
- The SRDF's disabled pairs: both checkers test the same 62 pairs, and a unit test compares the
  sets.
- The seven shelf boxes: one YAML, loaded as coal boxes and published to MoveIt as
  `CollisionObject`s. Zero padding in both.
- The planner and its settings: RRTConnect, range 2 rad, `longest_valid_segment_fraction` 0.0003,
  read by the direct planner from MoveIt's own `ompl_planning.yaml`, in a state space that uses
  MoveIt's L1 joint metric and extent, so those numbers mean the same step and the same 0.021 rad
  check spacing.
- The 10 s timeout and the simplification budget rule (timeout minus planning time).
- The velocity and acceleration limits.
- The controller that executes, and how tracking error is measured.

**What differs.**

- The collision library: coal through Pinocchio vs MoveIt's default FCL plugin. Coal is a fork of
  FCL, so they share algorithms. Pinocchio tests every active pair in a loop; MoveIt's FCL
  environment runs a broadphase first.
- Time parameterisation: rest-to-rest trapezoids at every simplified waypoint vs TOTG on MoveIt's
  interpolated path.
- The keep-upright constraint Jacobian: central differences vs analytic (above).
- MoveIt's planning scene monitor, planning context set-up, request/response adapters and
  trajectory execution manager, which the direct pipeline does not have.

**What MoveIt gives you that the direct pipeline does not.** Live octomap updates from the camera,
attached objects, the request/response adapters (start-state repair, solution validation, time
parameterisation), path constraints without writing them yourself (v0.1's keep-upright), and
controller management.

**Like for like.** The success rates; "OMPL solve + simplification" (MoveIt's `planning_time` is
exactly that, as in `ModelBasedPlanningContext::solve`); "request → time-parameterised
trajectory" (wall time in the same process); path lengths; cross-validation; and tracking error.
**Not like for like:** trajectory duration and execution time (different parameterisers, and the
slower rest-to-rest trapezoids lag the reference less, which also makes "executed and reached the
goal" a comparison of two parameterisers of different aggressiveness), and the per-request check
counts (MoveIt does not expose them; the probe estimates them). MoveIt's machinery outside the
planner is measured as `plan()` wall time minus `planning_time`; the direct pipeline's equivalent
is request → trajectory minus solve + simplification (path copy, statistics and
parameterisation), not the parameterisation alone.

## Where it fails, in full

The long form of every item in the README's [Where it fails](../README.md#where-it-fails), in the
README's numbering. Measured values are in [results.md](results.md); the 30 Oct and 31 Oct runs
are compared in [Run-to-run](results.md#run-to-run-30-oct-vs-31-oct).

### F1. Keep-upright planning fails often

Holding the tool within 5° of vertical, RRTConnect in OMPL's projected state space solves clearly
fewer shelf queries in 10 s than unconstrained planning
([v0.1 table](results.md#v01-planner--constraint)), and the gap is largest when the arm has to leave one
compartment and enter another ([per-class table](results.md#v01-by-query-class)). The projected space
walks a 4-D manifold of tool-down configurations (six joints minus the two tilt directions; yaw
is free), and the compartment openings are narrow passages of it: the `connect` step stalls where
the manifold squeezes through an opening. A MoveIt-side contributor I have not verified: the
constrained validity checker re-checks the path constraint with exact tolerances, while OMPL's
projection stops at a 1e-4 residual.

### F2. BiTRRT is not a drop-in alternative

With its range chosen on the constrained tuning queries (2 rad), BiTRRT matches RRTConnect on the
keep-upright queries but solves none of the unconstrained ones in 10 s
([v0.1 table](results.md#v01-planner--constraint)). My working explanation, not verified: BiTRRT only
joins its trees when they come within one `range` step in MoveIt's L1 joint metric, while in the
projected state space the re-projected steps meet more often (see
[BiTRRT](#bitrrt-second-planner-column)). A range tuned on unconstrained queries was not tried.
The range choice itself is fragile: applied to the tuning set regenerated on 31 Oct, the same rule
would pick the automatic range instead of 2 rad
([Run-to-run](results.md#run-to-run-30-oct-vs-31-oct); the regenerated CSV is in
`results/rerun_2026-10-31/tuning/`), and `regenerate_all.sh` does not re-apply the rule.

### F3. Planning is very sensitive to the RRTConnect range

OMPL's automatic range is 20 % of MoveIt's joint-space extent. For the UR5e (five joints ±2π, the
elbow ±π; extent = sum of the ranges = 22π ≈ 69.1 rad) that is a 13.8 rad step, longer than most
whole paths. On a separate tuning set (20 queries, seed 1, keep-upright only) the automatic range
solved the fewest queries and 2 rad the most, so 2 rad is what the benchmark uses
([tuning table](results.md#how-rrtconnects-range-was-chosen-tuning-set-not-the-evaluation-set),
[how it was chosen](#how-range-was-chosen)). The margin between 2 rad and 1 rad is small on 20
queries, and it shrank when the tuning set was regenerated on 31 Oct, although the rule still
picks 2 rad ([Run-to-run](results.md#run-to-run-30-oct-vs-31-oct)). Ranges above 2 rad were not
tried.

### F4. Discrete collision checking misses grazing contacts

Re-checking every time-parameterised trajectory at 0.01 rad joint steps finds some successful plans
that touch an octomap voxel between checked states (dense-check column of the
[v0.1 table](results.md#v01-planner--constraint)). OMPL checks motions every ≈ 0.021 rad, and
`ValidateSolution` checks only the waypoints; TOTG's corner blending (capped at 0.01 rad) also moves
the trajectory off the checked segments. A collision-padding margin or continuous collision
checking would close this; neither is in v0.1. v0.2 shows the same effect in both pipelines
([F20](#f20-dense-re-checks-find-discretisation-misses-not-checker-disagreements)).

### F5. Executing plans in Gazebo needs slow playback

`gz_ros2_control` tracks position commands through a soft proportional loop. Time-optimal
trajectories played back 2–5× slower than planned still lag far enough to reach the controller's
0.2 rad path tolerance and clip the shelf, so the hero and keep-upright GIFs are recorded from a
much slower replay (the factor is set in [`scripts/record_gifs.sh`](../scripts/record_gifs.sh)); the
direct-vs-MoveIt GIF records `bench_direct`'s own full-speed executions of one query. Even that is not always
enough: in the 31 Oct regeneration the first GIF recording aborted with MoveIt's `CONTROL_FAILED`
(a path-tolerance abort on the shoulder pan joint) and stopped `regenerate_all.sh`; the same step
passed on a retry with a re-planned trajectory. The GIF recorder
([`record_demo.py`](../src/pcp_bringup/scripts/record_demo.py), run by `record_gifs.sh`) now
retries a `CONTROL_FAILED` execution from the start, up to three attempts in all, and logs every
attempt. The v0.1 benchmark only plans; v0.2 executes
both pipelines ([F21](#f21-execution-in-gazebo-is-not-always-clean)).

### F6. The default collision-check resolution is wrong for this arm

With MoveIt's usual `longest_valid_segment_fraction: 0.005`, the 69.1 rad extent means a motion
check only every 0.35 rad. In the first full run every unconstrained failure was a path that
stepped over a 2 cm octomap board and was caught by `ValidateSolution` (MoveIt error -2,
`INVALID_MOTION_PLAN`; [ablation table](results.md#ablation-collision-check-resolution-0005-moveits-usual-value)).
The benchmark uses 0.0003 (≈ 0.021 rad) and re-checks every time-parameterised trajectory at
0.01 rad ([collision-check resolution](#v01-pipeline-and-parameters)). The ablation is not
regenerated by `regenerate_all.sh`: it needs the old commit (`git checkout 6c12eeb`).

### F7. MoveIt's constrained state space supports a subset of constraints

Observed in MoveIt 2.15.2's `ompl_constraints.hpp` and the library's messages: the constrained state
space supports at most one position constraint (box regions only, with an equality-constraint hack
for very thin boxes) and one orientation constraint (rotation-vector parameterization only). Joint
constraints and visibility constraints in `path_constraints` are not represented in the projected
space, and additional position or orientation constraints are ignored with a warning ("Only a
single position constraint is supported. Using the first one."). Line and box position
constraints and rejection-sampling mode are not in v0.1; they are on the README's next-steps list.

### F8. The rotation vector couples yaw and tilt

MoveIt bounds each rotation-vector component separately, so the x/y tolerance is 5°/√2 and the
admissible tilt depends on how far the tool has yawed from the reference: at a yaw offset ψ,
‖(r_x, r_y)‖ ≈ tilt · (ψ/2)/sin(ψ/2). With the reference yaw centred between start and goal, the
admissible tilt is between ≈ 3.2° and 5°, never more than 5°. With a fixed reference it would drop
to ≈ 2.25° for a tool yawed by 180°. Along the path the yaw can still drift further from the
reference, which tightens the constraint there. Derivation and test:
[Yaw/tilt coupling](#keep-upright-orientation-constraint).

### F9. Unconstrained paths would spill the cup

Without the constraint the tool tilts far from vertical on the way, even though start and goal are
both tool-down (max-tilt column of the [v0.1 table](results.md#v01-planner--constraint) and the tilt
plot in [results.md](results.md#plots)). The tilt is measured on the path densified to 0.01 rad
joint steps, which is what a joint trajectory controller executes between waypoints.

### F10. Coarser octomaps break queries before they speed planning up

2 cm was chosen from the thinnest obstacle (2 cm boards): it is the coarsest resolution that still
represents every board as a solid layer. In the 2 cm vs 5 cm sweep on the first 20 evaluation
queries, 5 cm voxels put the start or goal of many queries in collision (MoveIt errors -10,
`START_STATE_IN_COLLISION`, and -27, `GOAL_STATE_INVALID`, listed under the
[5 cm table](results.md#octomap-resolution-sweep-5-cm-first-20-evaluation-queries)), because one
voxel swallows the generator's 3 cm clearance, while unconstrained planning time barely changes
([2 cm table](results.md#octomap-resolution-sweep-2-cm-first-20-evaluation-queries)).

### F11. One fixed camera

Occluded space is unknown, and MoveIt treats unknown as free: the back corners of the compartments
behind the boards are not in the map. A second viewpoint, or treating unknown space as occupied,
would close this; neither is in this repository.

### F12. No sensor noise

The Gazebo depth camera is noise-free, fixed and needs no calibration. A real camera's speckle and
edge artefacts would add phantom voxels next to the shelf, and a camera-to-base calibration error
would shift the whole map.

### F13. Query generation avoids the hard edges

Start and goal poses are sampled inside inset compartment regions with 3 cm clearance from the
shelf, so IK failures at the shelf edges are not exercised (the generator's counters are in
`results/queries.json`).

### F14. Not bit-reproducible, and one repetition

Queries are byte-identical from the seed (tested); plans are not, because OMPL stops on wall-clock
time, and MoveIt's sampler is not seeded at all ([F24](#f24-moveits-plans-are-not-seed-reproducible)).
An earlier run of an equivalent configuration (commit 4060411's `results/bench.csv`) solved more
keep-upright queries than the published run, within the Wilson intervals. A second full run on
31 Oct ([Run-to-run](results.md#run-to-run-30-oct-vs-31-oct)) kept the success counts inside the
30 Oct Wilson intervals except MoveIt's executed-to-goal count, and moved the timing medians in
both directions; the timings carry no stated variance because each cell ran once. The regenerated
tuning set would also have picked a different BiTRRT range
([F2](#f2-bitrrt-is-not-a-drop-in-alternative)). The 31 Oct regeneration was two runs:
`regenerate_all.sh` stopped at the GIF step with `CONTROL_FAILED`, and its v0.2 steps then ran as a
continuation. Seeds and the commit hash are stamped on every row. Three repetitions per cell are
on the next-steps list.

### F15. MoveItCpp crashes on shutdown

MoveIt 2.15.2's `~MoveItCpp` segfaults on shutdown in this setup (exit code −11, after "Deleting
MoveItCpp"), even with the executor stopped. The CSV and metadata files are closed by then, so the
benchmark node flushes and exits with `std::_Exit` instead of running the destructor
(`benchmark_main.cpp:371–375`).

### F16. The direct pipeline is slower at planning

On the same queries with the same OMPL settings, the direct pipeline's solve + simplification takes
several times MoveIt's `planning_time` (paired median of per-query ratios in the
[v0.2 table](results.md#v02-direct-ompl--coal-pipeline-vs-moveit-2-same-queries-boxes-settings-controller)).

*What is measured.* The [probe](#check-count-probe) shows that both planners make a similar number
of validity checks per request, but that a check costs several times more inside the direct planner
than inside MoveIt's planner, and that the checker takes almost all of the direct planner's solve +
simplify time ([probe table](results.md#where-the-planning-time-goes-probe)). MoveIt's machinery
outside the planner (planning context set-up, adapters, TOTG: `plan()` wall time minus
`planning_time`) is too small to close the gap: request → trajectory is still slower for the direct
pipeline. The probe counts MoveIt's cached verdicts as checks, which biases its per-check time
down by an amount that was not measured.

*What is not measured: why a direct check costs more.* This is a hypothesis. Pinocchio's
`computeCollisions` (`collision.cpp:166`) runs coal's narrow phase on the active pairs one by one
until the first collision, so on every collision-free state it tests all 62 pairs (the count is in
`results/collision_agreement.csv.meta.json`), with no broadphase; MoveIt's FCL environment culls
pairs with a broadphase first. Two observations are consistent with it but do not test it: the gap
is wider inside the planners, where most states are free, than in the
[collision agreement](results.md#collision-agreement-moveits-planning-scene-vs-the-direct-checker)
test, where most configurations collide and both checkers stop early; and on the octomap the
planning-time gap shrinks ([octomap table](results.md#both-pipelines-on-move_groups-octomap)),
although MoveIt's per-check cost on the octomap is not exposed, so that is inferred from planning
time only.

*The test, which is next.* `pinocchio::BroadPhaseManager` ships with the installed Pinocchio 4.1.0
(`/opt/ros/lyrical/include/pinocchio/collision/broadphase-manager.hpp`). The ablation runs the
same queries with the direct checker behind a broadphase manager, three repetitions per cell, and
counts MoveIt's cached and evaluated checks separately. If the hypothesis is right, the direct
per-check cost drops towards MoveIt's on free states and the planning-time gap closes.

### F17. Keep-upright planning is weaker in the direct pipeline

With the same constraint definition, the direct planner solves fewer keep-upright queries within
10 s than MoveIt and takes longer on the ones it solves
([keep-upright table](results.md#keep-upright-in-both-pipelines)).

*The cause is not measured.* It is not the collision checker's per-check cost (an earlier version
of the README said so): in keep-upright mode the direct planner makes far fewer checks than
unconstrained, and checks × per-check time is a small fraction of its solve + simplify time (the
estimated checker share in the [keep-upright table](results.md#keep-upright-in-both-pipelines),
from `validity_checks` and `check_time_us_median` in `results/bench_direct_upright.csv`). The time
goes elsewhere, most likely into the projection. The first suspect is the direct pipeline's central-difference constraint
Jacobian (`KeepUprightConstraint::jacobian`, `src/pcp_direct/src/constraint.cpp:82–103`): 13
forward-kinematics calls (`:85` once, `:96` twice per joint) and two heap allocations (`:91–92`)
per call, called at every Newton iteration of every projection, against MoveIt's analytic
Jacobian. A less accurate Jacobian can also make projection fail more often, not only run slower.
The losses concentrate on across-divider and diagonal queries
([per-class counts](results.md#keep-upright-in-both-pipelines);
[F27](#f27-the-direct-keep-upright-planner-fails-across-compartments)). The test is an analytic
Jacobian (`pinocchio::computeFrameJacobian` and the rotation-vector map) on the same queries, with
a profile of projection time and Newton iterations.

*Observation, not an explanation.* The direct keep-upright paths tilt less than MoveIt's and have
far fewer waypoints (median waypoint counts:
[keep-upright table](results.md#keep-upright-in-both-pipelines)). MoveIt interpolates along
constrained geodesics; the direct path is a few vertices joined by segments. Neither difference has
been traced to a cause.

### F18. Simplification dominates direct planning time

Simplification took longer than the OMPL solve on most direct plans (paired bullets under the
[v0.2 table](results.md#v02-direct-ompl--coal-pipeline-vs-moveit-2-same-queries-boxes-settings-controller)).
The simplifier is OMPL's, the same code as in MoveIt, but every one of its thousands of motion
checks pays the direct checker's per-check cost ([F16](#f16-the-direct-pipeline-is-slower-at-planning)).
The simplifier also stops on wall-clock time (the budget is timeout − solve time), so its output
depends on machine load.

### F19. The two collision checkers disagree only at nanometres

Every uniform configuration gets the same verdict from both checkers, but uniform samples never
come within millimetres of contact. The boundary probe, bisected to 1e-7 rad, finds a few brackets
where coal reports contact and FCL measures a separation of a few nanometres, all on
forearm_link/wrist_3_link
([collision agreement](results.md#collision-agreement-moveits-planning-scene-vs-the-direct-checker)).
That is below coal's default GJK tolerance (1e-6 m, `GJK_DEFAULT_TOLERANCE` in coal's
narrowphase defaults), so either verdict is inside the precision coal works to. Coal is a fork of FCL, so
this is not independent verification of either. Nothing was tuned. The verdicts are deterministic:
the 31 Oct run reproduced them exactly ([Run-to-run](results.md#run-to-run-30-oct-vs-31-oct)).

### F20. Dense re-checks find discretisation misses, not checker disagreements

Some MoveIt trajectories fail the direct checker at 0.01 rad, and some direct paths fail MoveIt's
("Path clean for the OTHER pipeline's checker" in the
[v0.2 table](results.md#v02-direct-ompl--coal-pipeline-vs-moveit-2-same-queries-boxes-settings-controller)).
On every path the other pipeline's checker agrees with the path's own checker, so these are
discretisation misses, not disagreements: OMPL checks motions every 0.021 rad (L1) and some
segments graze the shelf between checked states, and MoveIt's TOTG also blends corners between the
waypoints `ValidateSolution` checked, as in v0.1 ([F4](#f4-discrete-collision-checking-misses-grazing-contacts)).

### F21. Execution in Gazebo is not always clean

Some executions abort because the arm lags the controller's reference by more than the 0.2 rad
path tolerance; the controller stops, MoveIt reports `CONTROL_FAILED`, and the run counts as not
reaching the goal ("Executed and reached the goal" in the
[v0.2 table](results.md#v02-direct-ompl--coal-pipeline-vs-moveit-2-same-queries-boxes-settings-controller)).
These path-tolerance aborts hit MoveIt's TOTG trajectories, including paths that were clean on both
dense checks (each abort is listed in [Execution in Gazebo](results.md#execution-in-gazebo-aborts-repositioning-tracking-error)); their number changed
between the 30 Oct and 31 Oct runs ([Run-to-run](results.md#run-to-run-30-oct-vs-31-oct)). None
of the direct pipeline's measured executions (slower rest-to-rest trapezoids) aborted in either
run.

The direct planner's repositioning moves (to each run's start, not measured runs) can abort the
same way. In the 30 Oct run one did and left the arm in contact with the shelf. The repositioning
counter ([Execution in Gazebo](results.md#execution-in-gazebo-aborts-repositioning-tracking-error)) counts a move as failed only if it is still not at its
target after its second attempt, and each attempt escapes from a colliding configuration first,
so an abort that the retry recovers does not appear in that counter.

In the first full run such an abort (a MoveIt trajectory that grazed the shelf) left the arm in
contact, and the harness could not plan away from a colliding start, so the run cascaded (kept in
`results/runs/`). The harness now escapes first: it samples configurations in boxes of growing size
around the measured one, takes the closest collision-free sample with at least 5 mm clearance and
moves there in a straight joint-space line that is not collision-checked
(`bench_direct_main.cpp:571–612`). That is acceptable in simulation and would not be on hardware.
The number of escapes per run is `escapes` in `results/bench_direct.csv.meta.json`, printed in
[Execution in Gazebo](results.md#execution-in-gazebo-aborts-repositioning-tracking-error).

### F22. Tracking error

Max joint tracking error is `/joint_states` against the controller's own reference
(`controller_state`), matched by stamp, the same measurement for the direct trapezoids and MoveIt's
TOTG trajectories (median / p95 in the
[v0.2 table](results.md#v02-direct-ompl--coal-pipeline-vs-moveit-2-same-queries-boxes-settings-controller),
the full distribution in [Execution in Gazebo](results.md#execution-in-gazebo-aborts-repositioning-tracking-error)). It stays within the controller's 0.2 rad path tolerance except on the path-tolerance
aborts ([F21](#f21-execution-in-gazebo-is-not-always-clean)); when several runs abort, as on
31 Oct ([Run-to-run](results.md#run-to-run-30-oct-vs-31-oct)), MoveIt's p95 itself can reach the
tolerance. The path tolerance is about stopping the
controller, not about collisions: [F26](#f26-tracking-error-exceeds-the-collision-margin).

### F23. No queries were skipped

No start or goal was rejected by either checker, because the generator keeps 3 cm clearance from
the boxes. The count of skipped queries is printed in the source line of the
[v0.2 table](results.md#v02-direct-ompl--coal-pipeline-vs-moveit-2-same-queries-boxes-settings-controller)
and recorded per row (`status` = `skipped:…`) in `results/bench_direct.csv`.

### F24. MoveIt's plans are not seed-reproducible

`ompl::RNG::setSeed` seeds the direct planner, but MoveIt's state sampler draws from its own
`random_numbers` generator (`bench_direct_main.cpp:367–369`), so MoveIt plans differ from run to
run. Even the seeded direct planner does not repeat exactly when execution is on: the repositioning
moves draw from the same random number generator, and simplification stops on wall-clock time. In
plan-only, one-query-per-process runs (the probe) the direct check counts repeat exactly.

### F25. CI does not run the 100-query benchmarks

GitHub Actions builds the image, runs every unit test, the smoke test (headless depth rendering
works on the runner's Mesa), the collision agreement on 50 configurations and two 5-query
mini-benchmarks on every push. The full benchmarks need Gazebo for tens of minutes and run locally,
stamped with the commit. The mini-benchmarks only check that the expected number of rows was
written, so a run in which every plan fails still passes; CI asserts no result in the README.

### F26. Tracking error exceeds the collision margin

Executed paths are not the checked paths. The p95 max tracking error of both pipelines is several
times the 0.01 rad dense-check step (the ratio is printed in [Execution in
Gazebo](results.md#execution-in-gazebo-aborts-repositioning-tracking-error)). Tracking error is a
joint angle, and its distance at the tool depends on which joint lags and on the pose. The same
section prints an upper bound: if all of the p95 lag were at the shoulder, with the tool as far from
it as the UR5e's links allow, it would move the tool by more than the generator's 3 cm clearance.
That is a bound, not a measured distance, so the lag could exceed the clearance; whether it did is
not known. Execution-time contact is not measured at all: there is no contact sensor and no contact
column. "Executed and reached the goal" therefore says nothing about collision-free execution. The
30 Oct run shows it: a MoveIt path that was clean on both dense checks aborted and stopped in
contact with the shelf ([F21](#f21-execution-in-gazebo-is-not-always-clean)). Closing this needs a
clearance margin tied to the tracking error, a dense check on the recorded `/joint_states` rather
than on the plan, and Gazebo contact recording.

### F27. The direct keep-upright planner fails across compartments

Per query class, the direct keep-upright planner's losses against MoveIt are concentrated on
across-divider and diagonal queries ([per-class counts](results.md#keep-upright-in-both-pipelines)).
Its failures end as OMPL `Approximate solution` or `Timeout` (the `status` column of
`results/bench_direct_upright.csv`), not as rejected starts or goals. These are the queries whose
tool-down path has to cross the divider or go diagonally through the compartment openings, the
narrow passages of [F1](#f1-keep-upright-planning-fails-often), where every projection step pays
the finite-difference Jacobian of [F17](#f17-keep-upright-planning-is-weaker-in-the-direct-pipeline).
Why the direct planner fails structurally there while MoveIt does not is not measured.

## References

- J. J. Kuffner and S. M. LaValle, "RRT-Connect: An efficient approach to single-query path
  planning," ICRA 2000.
- D. Devaurs, T. Siméon and J. Cortés, "Enhancing the transition-based RRT to deal with complex
  cost spaces," ICRA 2013.
- Z. Kingston, M. Moll and L. E. Kavraki, "Exploring implicit spaces for constrained
  sampling-based planning," IJRR 38(10–11), 2019.
- I. A. Şucan, M. Moll and L. E. Kavraki, "The Open Motion Planning Library," IEEE RAM 19(4), 2012.
- E. B. Wilson, "Probable inference, the law of succession, and statistical inference," JASA 22,
  1927 (confidence intervals in results.md).
