# MORNING — pointcloud-constrained-planning (R2)

## STATUS

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

**Blocked — decisions needed (details below):**

1. CI: fix billing or make the repo public.
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
