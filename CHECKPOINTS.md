# CHECKPOINTS — pointcloud-constrained-planning (R2)

## CHECKPOINT 0 — plumbing (2026-10-28 03:46 IST, overnight mode, not waiting)

Wall-clock spent so far: ~16 min (session start 03:30 IST).

### Checklist

| Item | Status |
|---|---|
| `docker build .` succeeds locally | Replaced by the MACHINE ADDENDUM with "the GitHub Actions job builds the image and is green". **BLOCKED**: GitHub refuses to start jobs on this account (billing), see MORNING.md item 1. The Dockerfile is not built anywhere yet. |
| CI run on GitHub is green on the skeleton | **BLOCKED (billing)**, run 40593288166: "The job was not started because recent account payments have failed or your spending limit needs to be increased". |
| `pre-commit run --all-files` passes | PASS (output below) |
| LICENSE has my name; THIRD_PARTY/NOTES/CHANGELOG/docs/scripts exist | PASS |
| Conventional commits, none dated before today | PASS |

Also run locally: an empty colcon workspace gives `colcon build` rc=0 and `colcon test-result --verbose`
rc=0 ("0 tests"), which is what the CI build and test steps would see on the skeleton.

GitHub: `gh repo create pointcloud-constrained-planning --private --source . --push` done;
description set; topics robotics, ros2, moveit2, gazebo, motion-planning, manipulation.
No branch protection, no visibility change (overnight rules).

### Command output

```text
$ .venv/bin/pre-commit run --all-files
fix end of files.........................................................Passed
trim trailing whitespace.................................................Passed
check for added large files..............................................Passed
check for merge conflicts................................................Passed
ruff check...........................................(no files to check)Skipped
ruff format..............................................................Passed
clang-format.........................................(no files to check)Skipped
yamllint.................................................................Passed
markdownlint-cli2........................................................Passed
$ grep -c "Giridhar J Shenoy" LICENSE
1
$ ls THIRD_PARTY.md NOTES.md CHANGELOG.md docs/architecture.md docs/results.md scripts/regenerate_all.sh
CHANGELOG.md
NOTES.md
THIRD_PARTY.md
docs/architecture.md
docs/results.md
scripts/regenerate_all.sh
$ git log --format="%h %ad %s" --date=iso
e47c849 2026-10-28 03:44:15 +0530 chore: add repository skeleton for pointcloud-constrained-planning
$ gh run list --limit 1
completed	failure	chore: add repository skeleton for pointcloud-constrained-planning	CI	main	push	40593288166	4s	2026-10-27T22:14:37Z
$ git status --short
?? MORNING.md
```

### MUST list remaining

Everything except the plumbing: sim + octomap + ci_smoke, C++ benchmark (constraints, queries,
runner, gtests), 100-query run, aggregate + results.md, GIF, README/docs, CI steps for smoke and
mini-benchmark.

### Does 1 Nov hold?

Yes for the code and local results. CI being green depends on the billing decision (MORNING.md).
