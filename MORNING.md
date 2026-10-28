# MORNING — pointcloud-constrained-planning (R2)

## STATUS

**Done (MUST list), all local, all committed and pushed to the private repo:**

- Skeleton, Dockerfile (`ros:lyrical-ros-base`), devcontainer, CI workflow, pre-commit — e47c849, CI fix 8c41f37.
- Headless sim: UR5e + Gazebo `rgbd_camera` → `/camera/points` → move_group octomap (2 cm) in
  `/monitored_planning_scene`; `scripts/ci_smoke.sh` passes in 16 s and also proves the octomap is
  in collision checking — ac65c0b.
- C++ benchmark node (MoveItCpp): keep-upright constraint builder, deterministic seeded query
  generator (`results/queries.json`, byte-for-byte regeneration test), runner on a frozen snapshot
  of move_group's map, metrics incl. max tilt and a dense collision re-check; 22 gtests/pytests
  pass — 7e3488a, fixes 89533bb, a2441cd, 56ea7e9.
- Results (f44225f), all tables from `scripts/aggregate.py`:
  - RRTConnect (range 2 rad): unconstrained 100/100, keep-upright **79/100** (Wilson 70–86 %),
    median 2.2 s, worst tilt 4.1°, 8.2 min for 200 cells.
  - BiTRRT (SHOULD): unconstrained 0/100, keep-upright 83/100.
  - `range` chosen on a separate seed-1 tuning set, never on the evaluation queries (8c81742).
  - Ablation of MoveIt's usual `longest_valid_segment_fraction` 0.005 (69 rad extent → check every
    0.35 rad → 17 % of unconstrained plans invalid), fixed in d7027c0.
  - Octomap sweep (SHOULD): at 5 cm half the queries start/end in collision (4345452).
- GIF `docs/assets/constrained_vs_unconstrained.gif` from Gazebo (no RViz): query 5, tilt 29° vs 1° — c0bf4f6.
- README (GIF, results, a long "where it fails"), docs/architecture.md (Mermaid), docs/planners.md,
  docs/results.md, CHANGELOG, NOTES, CHECKPOINTS (0, 1, 2, final).

**CI status:** never ran — GitHub refuses to start jobs on this account (billing). Latest run:
<https://github.com/giridhar-shenoy/pointcloud-constrained-planning/actions/runs/40607321438>
(pushes after it will show the same). The identical steps pass locally (CHECKPOINTS.md, final).

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
