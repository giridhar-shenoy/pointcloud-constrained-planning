# MORNING — pointcloud-constrained-planning (R2)

## STATUS (v0.2, 29 Oct 2026, ≈ 06:50 IST)

**Done — sprint override v0.2, MUST list and both SHOULD items, merged into main:**

- Step 0: CI green on main on the first run once the repo was public (run 40649038811, b587d57; no
  Dockerfile/ci.yml fix needed; the runner renders the depth camera). NOTES entry 30677c4.
- `src/pcp_direct` (C++17), the direct pipeline:
  - Pinocchio/coal collision checker, 62 pairs = MoveIt's (unit-tested): 134f81a.
  - Collision agreement: **500/500 = 100 %**, boundary probe 180/194 (all 14 differences at
    1–5 nm on forearm/wrist_3): 4412768.
  - RRTConnect planner in a `RealVectorStateSpace` with MoveIt's L1 metric: 7ac371c. Its settings
    are measured identical in MoveIt's own context: dd23b65.
  - R1's trapezoid parameteriser (copied, THIRD_PARTY.md): ea0a771. Executor, tracking monitor
    and `bench_direct`: dd23b65. Escape fix: 2ace597.
- **100 queries, direct vs MoveIt 2, with execution in Gazebo** (`results/bench_direct.csv`, 3e8b566):
  - Both planned 100/100.
  - Executed to goal: direct 100/100, MoveIt 99/100.
  - Solve + simplify median 0.402 s vs 0.123 s (paired 3.3×). Request → trajectory 0.402 s vs
    0.169 s.
  - Tracking error 0.033 vs 0.046 rad.
  - Dense re-checks: other checker = own checker on every path.
- Probe (`results/check_probe.json`, 7560eec): same number of checks, 48.6 µs vs 12.4 µs per
  check. The gap is the direct checker (no broadphase), not MoveIt's machinery.
- SHOULD, keep-upright (4a6186f/1d8e13a, 227d6a3): direct 58/100 vs MoveIt 88/100.
- SHOULD, octomap (b334471/f1e0a46/bba7c00, 8816c04): both 100/100, gap 1.3×.
- Docs: README v0.2 section, fairness paragraph, "where it fails", architecture lane, results
  block, CHANGELOG, `regenerate_all.sh` (8958d60, 95ea063, 4ba747c, 26366b7).
- **Merged into main:** b9e3d19 (MUST; main CI run 40794749550 green) and ee232d7 (SHOULD).

**CI status:**

- Branch, latest code: green, 49 gtests
  (<https://github.com/giridhar-shenoy/pointcloud-constrained-planning/actions/runs/40797577403>).
- Main after the MUST merge: green
  (<https://github.com/giridhar-shenoy/pointcloud-constrained-planning/actions/runs/40794749550>).
- Main after the SHOULD merge: green, 49 gtests, smoke and agreement 50/50
  (<https://github.com/giridhar-shenoy/pointcloud-constrained-planning/actions/runs/40798341717>,
  ee232d7). Later commits on main are docs-only (this file).

**Things I got wrong tonight and fixed** (NOTES.md has each with its commit):

- I timed MoveIt's checks without forward kinematics.
- A generic probe wrapper crashed MoveIt.
- The harness had no recovery from an aborted execution: run 1 cascaded, and it is kept in
  `results/runs/`.
- I edited a script while it was running.
- Two commits bundled extra files.
- My hypothesis that MoveIt checks less often was wrong: it is measured equal.

**Blocked — decisions needed (WAITING FOR GIRI below):**

1. Confirm D1–D5.
2. The Perceptyne L3 phrase.
3. Branch protection / Node-20 actions.
4. Approve a `regenerate_all.sh` end-to-end run (≈ 110 min).
5. Optional: 3 repetitions of the 100-query table.

**Next three steps:**

1. A broadphase (`pinocchio::BroadPhaseManager`) in the direct checker, then re-run the same 100
   queries. The probe predicts most of the 3.3× gap closes.
2. Run `regenerate_all.sh` once end to end (item 4) and record its runtime.
3. Look into the keep-upright gap (58 vs 88): profile projection vs checking in the constrained
   space, and MoveIt's exact-tolerance re-check (v0.1 hypothesis).

## STATUS (v0.1, 28 Oct, kept for history)

**Done (MUST list), all local, all committed and pushed to the private repo:**

- Skeleton, Dockerfile (`ros:lyrical-ros-base`), devcontainer, CI workflow, pre-commit — bfd430e, CI fix dfb0119.
- Headless sim: UR5e + Gazebo `rgbd_camera` → `/camera/points` → move_group octomap (2 cm) in
  `/monitored_planning_scene`; `scripts/ci_smoke.sh` passes in 16 s and also proves the octomap is
  in collision checking — 32530ab.
- C++ benchmark node (MoveItCpp): keep-upright constraint builder, deterministic seeded query
  generator (`results/queries.json`, byte-for-byte regeneration test), runner on a frozen snapshot
  of move_group's map, metrics incl. max tilt and a dense collision re-check; 22 gtests/pytests
  pass — c84969c, fixes ca8deb5, 2951a4c, db74fcc.
- Results (8129897), all tables from `scripts/aggregate.py`:
  - RRTConnect (range 2 rad): unconstrained 100/100, keep-upright **79/100** (Wilson 70–86 %),
    median 2.2 s, worst tilt 4.1°, 8.2 min for 200 cells.
  - BiTRRT (SHOULD): unconstrained 0/100, keep-upright 83/100.
  - `range` chosen on a separate seed-1 tuning set, never on the evaluation queries (e3a1945).
  - Ablation of MoveIt's usual `longest_valid_segment_fraction` 0.005 (69 rad extent → check every
    0.35 rad → 17 % of unconstrained plans invalid), fixed in ee95adb.
  - Octomap sweep (SHOULD): at 5 cm half the queries start/end in collision (a46137f).
- GIF `docs/assets/constrained_vs_unconstrained.gif` from Gazebo (no RViz): query 5, tilt 29° vs 1° — e6db299.
- README (GIF, results, a long "where it fails"), docs/architecture.md (Mermaid), docs/planners.md,
  docs/results.md, CHANGELOG, NOTES, CHECKPOINTS (0, 1, 2, final).

**CI status:** never ran — GitHub refuses to start jobs on this account (billing). Latest run:
<https://github.com/giridhar-shenoy/pointcloud-constrained-planning/actions/runs/40619295823>
(final push, 534ad1f; "The job was not started because recent account payments have failed…"). The identical steps pass locally (CHECKPOINTS.md, final).

**Things I got wrong overnight and fixed (worth knowing before the hour of questions):** the
collision-check resolution, the validate/time-parameterize order, the yaw-dependent tolerance, a
second hidden octomap in the benchmark node, the scene-topic remap, and one evaluation lost to a
pre-commit stash. Each is in NOTES.md with the commit that fixed it; all published numbers come from
runs after the fixes (tuning and ablation note that they predate the octomap wiring fix).

**Blocked — decisions needed (details below):** (v0.1; item 1 resolved on 29 Oct)

1. CI: fix billing or make the repo public. RESOLVED: public, CI green.
2. README: the Perceptyne L3 JD phrase.
3. Approve one end-to-end `scripts/regenerate_all.sh` run (~60 min of Gazebo, > 30 min rule).
4. Only if CI's llvmpipe cannot render the depth camera: approve a git-lfs point-cloud bag.

**Next three steps:**

1. Unblock CI (item 1), watch the first run (`gh run watch`), fix whatever the container build or
   headless rendering on llvmpipe trips over.
2. Run `regenerate_all.sh` once end to end (item 3) and record its runtime in NOTES.md.
3. Investigate the keep-upright failures (21/100 timeouts): MoveIt's exact-tolerance re-check of
   projected states, and a `range` sweep above 2 rad on the tuning set; then BiTRRT's 0/100
   unconstrained.

## v0.2 CHECKPOINT 2 RESULT — collision agreement (you asked to see this before continuing)

**500/500 = 100 %** of seeded uniform configurations get the same verdict from MoveIt (FCL) and the
direct checker (coal through Pinocchio), with identical pair lists; boundary probe 180/194, all 14
differences on forearm_link/wrist_3_link at 1–5 nm separation (below coal's 1e-6 m GJK tolerance).
Details: CHECKPOINTS.md "v0.2 CHECKPOINT 2". Work continued per the overnight rules.

## v0.2 CHECKPOINT 3 RESULT — direct vs MoveIt, 100 queries with execution (you asked to see the table before plotting)

| Metric | Direct (OMPL + coal) | MoveIt 2 (MoveItCpp) | Like for like? |
|---|---|---|---|
| Planned (Wilson 95 %) | 100/100 = 100 % (96 %–100 %) | 100/100 = 100 % (96 %–100 %) | yes |
| **OMPL solve + simplification**, median / p95 (s) | 0.402 / 1.039 | 0.123 / 0.274 | **yes**: MoveIt's `planning_time` is solve + simplify |
| OMPL solve only, median / p95 (s) | 0.081 / 0.531 | inside `planning_time` | direct only |
| Simplification, median / p95 (s) | 0.320 / 0.688 | inside `planning_time` | direct only |
| Time parameterisation, median / p95 (s) | 0.000025 / 0.000047 | inside `plan()` (TOTG adapter) | no: trapezoid vs TOTG |
| **Request → time-parameterised trajectory**, median / p95 (s) | 0.402 / 1.040 | 0.169 / 0.471 | **yes**: wall time, same process |
| Validity checks per request, median / p95 | 7668 / 19644 | not exposed | direct only |
| Collision check, median (µs) | 50.8 | 48.9 (agreement test, not in the plan) | yes (agreement test: same configurations, both with FK) |
| Trajectory duration, median / p95 (s) | 3.85 / 6.60 | 3.14 / 5.61 | no: rest-to-rest trapezoids vs TOTG on the interpolated path |
| Joint path length, median (rad) | 4.71 | 4.51 | yes (same metric) |
| TCP path length, median (m) | 1.18 | 1.13 | yes (same metric) |
| Path clean for the OTHER pipeline's checker at 0.01 rad (cross-validation) | 97/100 | 91/100 | yes |
| Path clean for its OWN checker at 0.01 rad | 97/100 | 91/100 | yes (a failure here is a discretisation miss, not a checker disagreement) |
| Executed and reached the goal (≤ 0.01 rad) | 100/100 = 100 % (96 %–100 %) | 99/100 = 99 % (95 %–100 %) | yes: same controller |
| Execution time, median / p95 (sim s) | 3.87 / 6.61 | 3.18 / 5.60 | no: follows the trajectory duration; MoveIt adds its execution manager |
| Max joint tracking error, median / p95 (rad) | 0.0330 / 0.0538 | 0.0457 / 0.0578 | yes: same controller, same measurement |

Source: `results/bench_direct.csv` (200 rows, 100 queries, 0 skipped, commit `5432d60`, query seed 0, OMPL seed 1 (direct only), with execution, 10 s per plan, total wall time 28.2 min). Direct: RRTConnectRange2, range 2.00 rad, check every 0.0207 rad (extent 69.12 rad). MoveIt's configured context: range 2, check every 0.0207 rad (extent 69.12 rad).

Details and the first (cascaded) run: CHECKPOINTS.md "v0.2 CHECKPOINT 3".

Correction after this table was logged: its "Collision check, median" row put the direct planner's
in-planner median next to the agreement test's random-configuration median for MoveIt, which is not
like for like. The published table (README, docs/results.md) now shows both measured inside their
planner on the same 20 probe queries: 48.6 µs direct vs 12.4 µs MoveIt.

## WAITING FOR GIRI

### v0.2 (29 Oct night) — decisions taken on my recommendation, please confirm

- **D1–D5** (NOTES.md, Session 2 plan): reuse `pcp_scenes/config/shelf.yaml` as the one box file;
  an L1 joint metric/extent in the direct OMPL space to match MoveIt's; `queries_boxes.json`
  identical to `queries.json` (the v0.1 generator already sampled against the boxes); velocity
  π rad/s and acceleration 5 rad/s² for both parameterisers (the JTC config has no limits); v0.1
  `sim.launch.py` unchanged with move_group idle in the background. Work proceeds on these; if
  you disagree, D2 and D4 change benchmark numbers, the others do not.

### v0.2 — open items that need you

- **README on main says "CI has not run yet"**, which is false since run 40649038811. Fix that
  bullet on main now (docs commit) or let the v0.2 merge carry the fix? Recommendation: let the
  merge carry it; if the merge is deferred, approve a one-line fix on main.
- **Perceptyne L3 phrase**: no application notes are in the repo, so the README row stays
  `TODO(Giri)`. Paste the phrase.
- **Branch protection on main** (not set; repo settings were not touched) and **GitHub Actions on
  Node.js 20** (`checkout@v4`, `setup-python@v5`, `cache@v4`, `build-push-action@v6`,
  `setup-buildx-action@v3`; the runner forces Node 24 and warns). Recommendation: protection
  requiring the CI check, and bump the actions when GitHub stops forcing.

- **`regenerate_all.sh` end to end** — what: one run of every v0.1 and v0.2 number (≈ 60 min for
  v0.1 and ≈ 50 min for v0.2 of Gazebo/timing, all under the locks). Why you: over the 20-min and 75-min
  rules. Recommendation: run it once on a quiet machine. Blocks: only the "runtime recorded"
  item.
- **Three repetitions of the 100-query direct-vs-MoveIt table** — what: ≈ 85 min of Gazebo. Why
  you: over the 75-min single-run budget. Recommendation: yes, before the README is used in an
  interview, so the timing columns have a spread. Blocks: nothing; the table is one repetition
  and says so.
- **Keep the harness escape step?** — It moved the arm out of shelf contact twice in the
  published run (after MoveIt's query-63 abort and after one direct repositioning abort).
  Recommendation: keep it and keep reporting the aborts (they are in the table and in "where it
  fails"). Blocks: nothing.

### v0.1 items (28 Oct)

Items 1 (billing) and 4 (git-lfs bag) are resolved: the repo is public, CI runs, and the runner
renders the depth camera (run 40649038811). Items 2 and 3 are still open.

### 1. GitHub Actions will not start on this private repo (billing)

- **What:** every CI run on `giridhar-shenoy/pointcloud-constrained-planning` ends in ~3 s with the
  annotation *"The job was not started because recent account payments have failed or your spending
  limit needs to be increased. Please check the 'Billing & plans' section in your settings"*
  (first seen on run 40593288166, 2026-10-28 03:44 IST).
- **Why it needs you:** the fix is either the account's billing / Actions spending limit, or making
  the repo public (public repos get free Actions minutes). Overnight rules forbid changing visibility,
  and billing is your account.
- **Recommendation:** make the repo public when you are happy with it (the original plumbing prompt
  planned `--public` anyway); otherwise fix the payment method or raise the spending limit. Then re-run
  the latest workflow with `gh run rerun <id>` or push any commit.
- **What it blocks:** the acceptance items "CI run on GitHub is green" (plumbing) and "CI green"
  (feature). Meanwhile I run the same build/lint/test/smoke commands natively on the laptop and log
  their output in CHECKPOINTS.md, and keep pushing only at checkpoints.

### 2. Perceptyne L3 row in the README job-description table

- **What:** the SPRINT OVERRIDE asks for a "How this maps to NUS 34489 / CMU NREC / Perceptyne L3"
  table. The NUS and CMU phrases are quoted in the feature prompt; no Perceptyne L3 wording was in
  my inputs, and I will not invent a JD quote.
- **Why it needs you:** only you have the JD text.
- **Recommendation:** paste the one phrase you most want to evidence (e.g. its motion-planning or
  perception line); the README row currently reads "TODO(Giri): paste the exact JD phrase".
- **What it blocks:** nothing else; one README row.

### 3. End-to-end `scripts/regenerate_all.sh` run

- **What:** the feature prompt wants regenerate_all.sh to reproduce every number and its runtime in
  NOTES.md. Every component ran tonight (tuning 11 min, BiTRRT tuning 6 min, evaluation 8 min,
  BiTRRT 24 min, sweep 5 min, GIF 5 min, smoke/tests 2 min) but never as one ≈ 60 min run.
- **Why it needs you:** "ask me before any run longer than 30 minutes"; it also holds the shared
  Gazebo lock for an hour.
- **Recommendation:** run it once when R1 does not need Gazebo:
  `flock`-aware already, so just `scripts/regenerate_all.sh` from the repo root.
- **What it blocks:** only the "actual runtime recorded in NOTES.md" acceptance item. Note that it
  re-runs the planners, so the numbers will move within their confidence intervals.

### 4. git-lfs point-cloud bag for CI (only if needed)

- **What:** headless depth rendering works on this laptop (EGL on the Iris Xe). On GitHub's runner
  it would be Mesa llvmpipe, which is untested. ci_smoke fails loudly if no octomap appears.
- **Why it needs you:** the sprint brief says to ask before storing bags with git-lfs.
- **Recommendation:** wait for the first CI run; only if it fails on rendering, record one bag
  (< 50 MB) and replay it in CI.
- **What it blocks:** nothing until CI runs.
