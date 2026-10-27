# Planners and constraints

All planner parameters live in
[`src/pcp_moveit_config/config/ompl_planning.yaml`](../src/pcp_moveit_config/config/ompl_planning.yaml);
the benchmark node sets only the request-level values listed at the end of this page, from
[`src/pcp_benchmark/config/benchmark.yaml`](../src/pcp_benchmark/config/benchmark.yaml).

## RRTConnect (the only planner in v0.1)

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
singularities or in narrow passages of the manifold — the dominant cause of the orientation
failures in [results.md](results.md). Without path constraints the same request uses the ordinary
joint-space state space.

BiTRRT (second column) is a SHOULD item and is not in v0.1.

## Keep-upright orientation constraint

| Field | Value | Why |
|---|---|---|
| link | `tool0` | the flange; the queries' tool poses are flange poses |
| target | z axis along world −z, yaw 0 (quaternion x=1, w=0) | "keep a cup upright" under a top grasp |
| parameterization | `ROTATION_VECTOR` | the only one OMPL's constrained state space implements (exponential coordinates of R_ee⁻¹·R_target, `ompl_constraints.hpp`) |
| x / y tolerance | 5°/√2 = 3.536° | for a rotation vector r, the tilt of the z axis is ≤ ‖(r_x, r_y)‖, so this square of tolerances lies inside the 5° cone: everything MoveIt accepts tilts ≤ 5° (checked by `test_constraints`, 2000 sampled states). It is conservative: a pure 4.9° tilt about x alone is rejected. |
| z tolerance | π | yaw is free |

**MoveIt limitation (v2.15.2, observed in `ompl_constraints.hpp` and the library's messages).**
The constrained state space supports at most one position constraint (box regions only, with an
equality-constraint hack for very thin boxes) and one orientation constraint (rotation-vector
parameterization only). Joint constraints and visibility constraints in `path_constraints` are not
represented in the projected space; additional position/orientation constraints are ignored with a
warning ("Only a single position constraint is supported. Using the first one."). Line and box
constraints and rejection-sampling mode are on the "What's next" list.

## Pipeline and parameters

| Parameter | Value | Where |
|---|---|---|
| planner config | `RRTConnectkConfigDefault`, `type: geometric::RRTConnect`, `range: 0.0` (OMPL default: 20 % of the space extent) | `ompl_planning.yaml` |
| `enforce_constrained_state_space` | `true` | `ompl_planning.yaml` (group `ur_manipulator`) |
| `projection_evaluator` | `joints(shoulder_pan_joint,shoulder_lift_joint)` | `ompl_planning.yaml` |
| `longest_valid_segment_fraction` | 0.0003 (MoveIt sums the joint ranges into the extent, 22π ≈ 69.1 rad for the UR5e, so this is ≈ 0.021 rad between collision checks; 0.005 would be 0.35 rad) | `ompl_planning.yaml` |
| request adapters | ResolveConstraintFrames, ValidateWorkspaceBounds, CheckStartStateBounds, CheckStartStateCollision | `ompl_planning.yaml` |
| response adapters | AddTimeOptimalParameterization, ValidateSolution, DisplayMotionPath | `ompl_planning.yaml` |
| planning time per cell | 10 s | `benchmark.yaml` (`bench.planning_time`) |
| planning attempts | 1 | `benchmark_main.cpp` (`PlanRequestParameters`, fixed by the benchmark protocol) |
| velocity / acceleration scaling | 1.0 / 1.0 | `benchmark_main.cpp` (only affects time parameterization, not the path) |
| OMPL seed | 0 (`ompl::RNG::setSeed` before the planner plugin loads) | `benchmark.yaml` (`bench.ompl_seed`) |
| octomap resolution | 0.02 m | `pcp_moveit_config/config/octomap.yaml` |

`ValidateSolution` is kept on purpose: a cell counts as a success only if the trajectory MoveIt
returns is collision-free against the snapshot and satisfies the path constraint at every
waypoint, not merely if OMPL reported an exact solution.

## References

- J. J. Kuffner and S. M. LaValle, "RRT-Connect: An efficient approach to single-query path
  planning," ICRA 2000.
- Z. Kingston, M. Moll and L. E. Kavraki, "Exploring implicit spaces for constrained
  sampling-based planning," IJRR 38(10–11), 2019.
- I. A. Şucan, M. Moll and L. E. Kavraki, "The Open Motion Planning Library," IEEE RAM 19(4), 2012.
- E. B. Wilson, "Probable inference, the law of succession, and statistical inference," JASA 22,
  1927 (confidence intervals in results.md).
