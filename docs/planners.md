# Planners and constraints

All planner parameters live in
[`src/pcp_moveit_config/config/ompl_planning.yaml`](../src/pcp_moveit_config/config/ompl_planning.yaml);
the benchmark node sets only the request-level values listed at the end of this page, from
[`src/pcp_benchmark/config/benchmark.yaml`](../src/pcp_benchmark/config/benchmark.yaml).

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
singularities or in narrow passages of the manifold — the dominant cause of the orientation
failures in [results.md](results.md). Without path constraints the same request uses the ordinary
joint-space state space.

## BiTRRT (second column, SHOULD item)

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
tuning set by the same rule (results.md). The chosen 2 rad keeps BiTRRT level with RRTConnect on
the keep-upright queries, but it still solves no unconstrained query within 10 s; the single-step
tree joining in an L1 joint metric is my working explanation, not a verified one.

## Keep-upright orientation constraint

| Field | Value | Why |
|---|---|---|
| link | `tool0` | the flange; the queries' tool poses are flange poses |
| target | z axis along world −z, yaw = circular mean of the query's start and goal yaws | "keep a cup upright" under a top grasp; why the yaw matters: see below |
| parameterization | `ROTATION_VECTOR` | the only one OMPL's constrained state space implements (exponential coordinates of R_ee⁻¹·R_target, `ompl_constraints.hpp`) |
| x / y tolerance | 5°/√2 = 3.536° | for a rotation vector r, the tilt of the z axis is ≤ ‖(r_x, r_y)‖, so this square of tolerances lies inside the 5° cone: everything MoveIt accepts tilts ≤ 5° (checked by `test_constraints`, 2000 sampled states). It is conservative: a pure 4.9° tilt about x alone is rejected. |
| z tolerance | π | yaw is free |

**Yaw/tilt coupling.** The rotation vector mixes yaw into the x/y components: at a yaw offset ψ
from the reference, ‖(r_x, r_y)‖ ≈ tilt · (ψ/2)/sin(ψ/2). With a fixed reference yaw the admissible
tilt would shrink to ≈ 2.25° for a tool yawed by 180°, and half of the uniformly-yawed queries would
face a tighter constraint than advertised (found in code review; `test_constraints` pins it: a 3°
tilt is accepted with the reference at the tool's yaw and rejected with it 180° away). The reference
yaw is therefore set per query halfway between start and goal yaw, so |ψ| ≤ 90° at both ends
(factor ≤ 1.11). Effective admissible tilt: between ≈ 3.2° and 5° depending on tilt direction and
yaw offset, never more than 5°. Along the path yaw can still drift further from the reference.

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
| response adapters | ValidateSolution, then AddTimeOptimalParameterization, DisplayMotionPath | `ompl_planning.yaml` |
| time parameterization | `totg.path_tolerance` 0.01 rad (default 0.1), `resample_dt` 0.1 s | `ompl_planning.yaml` |
| planning time per cell | 10 s | `benchmark.yaml` (`bench.planning_time`) |
| planning attempts | 1 | `benchmark_main.cpp` (`PlanRequestParameters`, fixed by the benchmark protocol) |
| velocity / acceleration scaling | 1.0 / 1.0 | `benchmark_main.cpp` (only affects time parameterization, not the path) |
| OMPL seed | 1 (`ompl::RNG::setSeed` before the planner plugin loads; OMPL replaces 0 by 1) | `benchmark.yaml` (`bench.ompl_seed`) |
| octomap resolution | 0.02 m | `pcp_moveit_config/config/octomap.yaml` |

**What "success" means.** `ValidateSolution` runs first, on the planner's own waypoints (OMPL has
already checked the motions between them at the `longest_valid_segment_fraction` resolution), so
a success is a path that is collision-free against the snapshot and inside the path constraint at
every waypoint. Time parameterization runs afterwards with corner blending capped at 0.01 rad. In
the first version the order was reversed with MoveIt's default 0.1 rad blending, so validation saw
0.1 s resamples of a blended path rather than the planned one (found in code review). The node
also re-checks the executed, time-parameterized trajectory at 0.01 rad joint steps and reports it
as `dense_collision_free`.

**Collision-check resolution.** MoveIt's state-space extent is the *sum* of the joint ranges,
22π ≈ 69.1 rad for the UR5e (±2π joints, elbow ±π). MoveIt's usual
`longest_valid_segment_fraction: 0.005` therefore means a motion check only every ≈ 0.35 rad,
enough for a link to step over a 2 cm board; in the first 100-query run all 17 unconstrained
failures were `ValidateSolution` rejections (results.md, ablation). 0.0003 gives ≈ 0.021 rad.

## How `range` was chosen

RRTConnect's `range: 0` means 20 % of the same extent, 13.8 rad per extension. Four settings
(automatic, 0.5, 1 and 2 rad) were compared on a separate tuning set — 20 queries generated with
seed 1, disjoint from the seed-0 evaluation queries — with the orientation constraint only; the
winner (highest success, ties broken by median time) is the benchmark planner. The evaluation set
was not used to choose it. Table: [results.md](results.md#how-rrtconnects-range-was-chosen-tuning-set-not-the-evaluation-set).

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
