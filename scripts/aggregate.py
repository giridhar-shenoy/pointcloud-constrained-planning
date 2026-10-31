#!/usr/bin/env python3
"""Aggregate the v0.1 results (results/bench.csv + results/bench_bitrrt.csv) into docs/results.md,
results/summary.json and the v0.1 plots.

    scripts/aggregate.py [--csv results/bench.csv] [--no-plots] [--print-only] [--check]
                         [--results-md docs/results.md]

docs/results.md is written by this script except the block between `<!-- direct:start -->` and
`<!-- direct:end -->`, which scripts/aggregate_direct.py owns and this script keeps. The README
blocks are all written by scripts/aggregate_direct.py (they combine v0.1 and v0.2 numbers and
compute the v0.1 ones with this module's functions from the same CSVs), so the two scripts can
run in either order. `--check` writes nothing and makes no plots; it exits 1 if docs/results.md
(outside the direct block) or results/summary.json differs from what the CSVs give. Nothing is
typed by hand.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import statistics
import sys
from collections import defaultdict

REPO = pathlib.Path(__file__).resolve().parent.parent
DIRECT_START = "<!-- direct:start -->"
DIRECT_END = "<!-- direct:end -->"
Z95 = 1.959963984540054

# Reference categorical palette, slots 1-2 (validated adjacent pair), light surface.
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e4e3df"
SERIES = {"none": "#2a78d6", "orientation": "#eb6834"}
PLANNER_LABEL = {
    "RRTConnectkConfigDefault": "RRTConnect (auto range, 13.8 rad)",
    "RRTConnectRange0_5": "RRTConnect (range 0.5 rad)",
    "RRTConnectRange1": "RRTConnect (range 1 rad)",
    "RRTConnectRange2": "RRTConnect (range 2 rad)",
    "BiTRRTkConfigDefault": "BiTRRT (auto range, 13.8 rad)",
    "BiTRRTRange2": "BiTRRT (range 2 rad)",
    "BiTRRTRange6": "BiTRRT (range 6 rad)",
}


def wilson(k: int, n: int, z: float = Z95) -> tuple[float, float]:
    """Wilson score interval for a binomial proportion k/n."""
    if n == 0:
        return (math.nan, math.nan)
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return (max(0.0, centre - half), min(1.0, centre + half))


def percentile(values: list[float], q: float) -> float:
    """Linear-interpolation percentile (numpy's default), q in [0, 100]."""
    if not values:
        return math.nan
    xs = sorted(values)
    pos = (len(xs) - 1) * q / 100.0
    lo, hi = math.floor(pos), math.ceil(pos)
    return xs[lo] + (xs[hi] - xs[lo]) * (pos - lo)


def compartment_change(start: str, goal: str) -> str:
    """Classify a query by how the start and goal compartments relate."""
    s_level, s_side = start.split("_")
    g_level, g_side = goal.split("_")
    if (s_level, s_side) == (g_level, g_side):
        return "same compartment"
    if s_level == g_level:
        return "across divider"
    if s_side == g_side:
        return "across board"
    return "diagonal"


def load_rows(path: pathlib.Path) -> list[dict]:
    with path.open() as f:
        return list(csv.DictReader(f))


def summarise(rows: list[dict]) -> list[dict]:
    groups: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in rows:
        groups[(r["planner"], r["constraint"])].append(r)
    out = []
    # The benchmark planner (RRTConnect) first, then the others.
    order = sorted(groups.items(), key=lambda kv: (not kv[0][0].startswith("RRT"), kv[0]))
    for (planner, constraint), rs in order:
        ok = [r for r in rs if r["success"] == "1"]
        times_ok = [float(r["planning_time_s"]) for r in ok]
        lo, hi = wilson(len(ok), len(rs))
        out.append(
            {
                "planner": PLANNER_LABEL.get(planner, planner),
                "constraint": constraint,
                "n": len(rs),
                "successes": len(ok),
                "success_rate": len(ok) / len(rs),
                "ci_lo": lo,
                "ci_hi": hi,
                "time_median_s": statistics.median(times_ok) if ok else math.nan,
                "time_p95_s": percentile(times_ok, 95),
                "joint_path_median_rad": statistics.median(float(r["joint_path_rad"]) for r in ok)
                if ok
                else math.nan,
                "tcp_path_median_m": statistics.median(float(r["tcp_path_m"]) for r in ok)
                if ok
                else math.nan,
                "tilt_median_deg": statistics.median(float(r["max_tilt_deg"]) for r in ok)
                if ok
                else math.nan,
                "tilt_max_deg": max(float(r["max_tilt_deg"]) for r in ok) if ok else math.nan,
                # Executed trajectory re-checked at 0.01 rad steps (column absent in old CSVs).
                "dense_ok": sum(1 for r in ok if r.get("dense_collision_free") == "1"),
                "dense_checked": sum(1 for r in ok if r.get("dense_collision_free", "") != ""),
                "invalid_queries": sum(
                    1 for r in rs if r["start_valid"] != "1" or r["goal_valid"] != "1"
                ),
            }
        )
    return out


def by_query_class(rows: list[dict]) -> list[dict]:
    groups: dict[tuple[str, str, str], list[dict]] = defaultdict(list)
    for r in rows:
        cls = compartment_change(r["start_region"], r["goal_region"])
        groups[(cls, r["planner"], r["constraint"])].append(r)
    out = []
    for (cls, planner, constraint), rs in sorted(groups.items()):
        k = sum(1 for r in rs if r["success"] == "1")
        lo, hi = wilson(k, len(rs))
        times = [float(r["planning_time_s"]) for r in rs if r["success"] == "1"]
        out.append(
            {
                "class": cls,
                "planner": PLANNER_LABEL.get(planner, planner),
                "constraint": constraint,
                "n": len(rs),
                "successes": k,
                "ci_lo": lo,
                "ci_hi": hi,
                "time_median_s": statistics.median(times) if times else math.nan,
            }
        )
    return out


def f(x: float, digits: int = 2) -> str:
    return "–" if isinstance(x, float) and math.isnan(x) else f"{x:.{digits}f}"


def pct(x: float) -> str:
    return "–" if math.isnan(x) else f"{100 * x:.0f} %"


def main_table(summary: list[dict]) -> str:
    lines = [
        "| Planner | Constraint | Success (95 % Wilson CI) | Planning time median / p95 (s) "
        "| Joint path median (rad) | TCP path median (m) | Max tool tilt median / worst (deg) "
        "| Executed path collision-free (dense check) |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for s in summary:
        lines.append(
            f"| {s['planner']} | {s['constraint']} | {s['successes']}/{s['n']} = "
            f"{pct(s['success_rate'])} ({pct(s['ci_lo'])}–{pct(s['ci_hi'])}) | "
            f"{f(s['time_median_s'], 3)} / {f(s['time_p95_s'], 3)} | "
            f"{f(s['joint_path_median_rad'])} | {f(s['tcp_path_median_m'])} | "
            f"{f(s['tilt_median_deg'], 1)} / {f(s['tilt_max_deg'], 1)} | "
            + (f"{s['dense_ok']}/{s['dense_checked']}" if s["dense_checked"] else "not recorded")
            + " |"
        )
    return "\n".join(lines)


def class_table(classes: list[dict]) -> str:
    lines = [
        "| Query class | Planner | Constraint | Success (95 % Wilson CI) "
        "| Planning time median (s) |",
        "|---|---|---|---|---|",
    ]
    for c in classes:
        rate = c["successes"] / c["n"]
        lines.append(
            f"| {c['class']} | {c['planner']} | {c['constraint']} | {c['successes']}/{c['n']} = "
            f"{pct(rate)} "
            f"({pct(c['ci_lo'])}–{pct(c['ci_hi'])}) | {f(c['time_median_s'], 3)} |"
        )
    return "\n".join(lines)


def make_plots(rows: list[dict], out_dir: pathlib.Path) -> list[pathlib.Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "figure.facecolor": SURFACE,
            "axes.facecolor": SURFACE,
            "axes.edgecolor": TEXT_2,
            "axes.labelcolor": TEXT,
            "xtick.color": TEXT_2,
            "ytick.color": TEXT_2,
            "text.color": TEXT,
            "axes.grid": True,
            "grid.color": GRID,
            "grid.linewidth": 0.8,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "font.size": 10,
        }
    )
    out_dir.mkdir(parents=True, exist_ok=True)
    planners = sorted({r["planner"] for r in rows}, key=lambda p: (not p.startswith("RRT"), p))
    combos = [
        (pl, c)
        for pl in planners
        for c in SERIES
        if any(r["planner"] == pl and r["constraint"] == c for r in rows)
    ]
    styles = ["-", "--", ":"]

    def short(pl: str) -> str:
        return PLANNER_LABEL.get(pl, pl).split(" (")[0]

    paths = []

    # 1. ECDF of planning time; failures sit at +inf so the curve tops out at the success rate.
    fig, ax = plt.subplots(figsize=(8.0, 3.6), dpi=150)
    for pl, c in combos:
        rs = [r for r in rows if r["planner"] == pl and r["constraint"] == c]
        times = sorted(float(r["planning_time_s"]) for r in rs if r["success"] == "1")
        n = len(rs)
        xs, ys = [], []
        for i, t in enumerate(times, start=1):
            xs.append(t)
            ys.append(i / n)
        label = f"{short(pl)}, {c}" if len(planners) > 1 else c
        ax.step(
            xs,
            ys,
            where="post",
            color=SERIES[c],
            linestyle=styles[planners.index(pl) % len(styles)],
            linewidth=2,
            label=label,
        )
        if xs:
            ax.annotate(
                f"{100 * ys[-1]:.0f} %",
                (xs[-1], ys[-1]),
                xytext=(6, 0),
                textcoords="offset points",
                va="center",
                color=TEXT,
            )
    ax.set_xscale("log")
    ax.set_ylim(0, 1.02)
    ax.set_xlabel("planning time (s, log scale)")
    ax.set_ylabel("fraction of queries solved")
    ax.set_title("Octomap shelf, 100 queries: time to solve", loc="left", color=TEXT)
    ax.legend(frameon=False, loc="upper left", bbox_to_anchor=(1.0, 1.0))
    fig.tight_layout()
    p = out_dir / "bench_planning_time_ecdf.png"
    fig.savefig(p)
    plt.close(fig)
    paths.append(p)

    # 2. Max tool tilt along each successful path, per constraint type (strip plot).
    fig, ax = plt.subplots(figsize=(6.4, 2.8), dpi=150)
    for i, (pl, c) in enumerate(combos):
        tilts = [
            float(r["max_tilt_deg"])
            for r in rows
            if r["planner"] == pl and r["constraint"] == c and r["success"] == "1"
        ]
        jitter = [((k * 0.6180339887) % 1.0 - 0.5) * 0.5 for k in range(len(tilts))]
        ax.scatter(
            tilts,
            [i + j for j in jitter],
            s=14,
            color=SERIES[c],
            edgecolors=SURFACE,
            linewidths=0.6,
        )
    ax.axvline(5.0, color=TEXT_2, linewidth=1, linestyle="--")
    ax.annotate(
        "5° limit",
        (5.0, len(combos) - 0.45),
        xytext=(4, 0),
        textcoords="offset points",
        color=TEXT_2,
    )
    ax.set_yticks(
        range(len(combos)),
        [f"{short(pl)}, {c}" if len(planners) > 1 else c for pl, c in combos],
    )
    ax.set_xscale("symlog", linthresh=10)
    ax.set_xlabel("max tool tilt from vertical along the executed path (deg)")
    ax.set_title("Does the cup stay upright?", loc="left", color=TEXT)
    fig.tight_layout()
    p = out_dir / "bench_max_tilt.png"
    fig.savefig(p)
    plt.close(fig)
    paths.append(p)
    return paths


# Secondary CSVs rendered as their own sections: (file, title, explanation, regenerate command).
EXTRA_SECTIONS = [
    (
        "results/tuning/tuning.csv",
        "How RRTConnect's range was chosen (tuning set, not the evaluation set)",
        "20 queries generated with seed 1 (`results/tuning/queries_seed1.json`), disjoint from the "
        "seed-0 evaluation queries, orientation constraint only, four `range` settings. The best "
        "success rate (ties: lower median time) became the benchmark planner configuration."
        " Run before db74fcc, when the benchmark node built its own octomap from the same camera "
        "with identical settings (4747 voxels, the same count as move_group's map).",
        "`scripts/bench_planners.sh 20 results/tuning/tuning.csv "
        "results/tuning/queries_seed1.json results/tuning/tuning.yaml`",
    ),
    (
        "results/tuning/tuning_bitrrt.csv",
        "How BiTRRT's range was chosen (tuning set)",
        "Same protocol as for RRTConnect: the 20 seed-1 tuning queries, orientation constraint "
        "only, three `range` settings; the highest success rate became the BiTRRT column."
        " Run before db74fcc, when the benchmark node built its own octomap from the same camera "
        "with identical settings (4747 voxels, the same count as move_group's map).",
        "`scripts/bench_planners.sh 20 results/tuning/tuning_bitrrt.csv "
        "results/tuning/queries_seed1.json results/tuning/tuning_bitrrt.yaml`",
    ),
    (
        "results/sweep/octomap_0.02.csv",
        "Octomap resolution sweep: 2 cm (first 20 evaluation queries)",
        "The first 20 seed-0 queries with the benchmark planner, octomap voxels of "
        "2 cm (the default) in a fresh simulation; compare with the 5 cm section below.",
        "`scripts/bench_octomap_resolution.sh 20 0.02 0.05`",
    ),
    (
        "results/sweep/octomap_0.05.csv",
        "Octomap resolution sweep: 5 cm (first 20 evaluation queries)",
        "Same 20 queries, octomap voxels of 5 cm. Coarser voxels inflate obstacles by up to a "
        "voxel, which eats the generator's 3 cm clearance; `start_valid`/`goal_valid` in the CSV "
        "show how many queries become invalid.",
        "`scripts/bench_octomap_resolution.sh 20 0.02 0.05`",
    ),
    (
        "results/ablation/bench_lvsf0.005.csv",
        "Ablation: collision-check resolution 0.005 (MoveIt's usual value)",
        "The same 100 evaluation queries with the first configuration: "
        "`longest_valid_segment_fraction: 0.005`, OMPL's automatic range, time parameterization "
        "before `ValidateSolution` (0.1 rad blending), keep-upright reference yaw fixed at 0, OMPL "
        "seed 0. On the UR5e's 69 rad extent 0.005 means a collision check only every 0.35 rad; "
        "the unconstrained failures here are `ValidateSolution` rejections (error -2), not planner "
        "timeouts."
        " Run before db74fcc, when the benchmark node built its own octomap from the same camera "
        "with identical settings (4747 voxels, the same count as move_group's map).",
        "`git checkout 6c12eeb && scripts/bench_planners.sh 100 "
        "results/ablation/bench_lvsf0.005.csv`",
    ),
]


def extra_sections() -> tuple[str, list[str]]:
    md, regen = [], []
    for rel, title, text, cmd in EXTRA_SECTIONS:
        path = REPO / rel
        if not path.exists():
            continue
        rows = load_rows(path)
        codes = defaultdict(int)
        for r in rows:
            if r["success"] != "1":
                codes[(r["constraint"], r["error_code"])] += 1
        failures = ", ".join(f"{c} code {e}: {n}" for (c, e), n in sorted(codes.items()))
        commits = ", ".join(sorted({f"`{r['commit']}`" for r in rows}))
        md.append(
            f"## {title}\n\n{text}\n\nSource: `{rel}` ({len(rows)} rows, commit {commits}).\n\n"
            f"{main_table(summarise(rows))}\n\n"
            f"Failures by MoveIt error code: {failures or 'none'}.\n"
        )
        for srow in summarise(rows):
            regen.append(
                f"| {title.split(':')[0].split(' (')[0]}, {srow['planner']}, {srow['constraint']} "
                f"| success rate | {pct(srow['success_rate'])} | {cmd} |"
            )
    return "\n".join(md), regen


PLOT_FILES = ("bench_planning_time_ecdf.png", "bench_max_tilt.png")

# Static table of contents; scripts/tests checks that every anchor exists in docs/results.md.
CONTENTS = [
    ("v0.1: planner × constraint", "v01-planner--constraint"),
    ("v0.1: by query class", "v01-by-query-class"),
    (
        "How RRTConnect's range was chosen",
        "how-rrtconnects-range-was-chosen-tuning-set-not-the-evaluation-set",
    ),
    ("Octomap resolution sweep", "octomap-resolution-sweep-2-cm-first-20-evaluation-queries"),
    (
        "v0.2: direct pipeline vs MoveIt 2",
        "v02-direct-ompl--coal-pipeline-vs-moveit-2-same-queries-boxes-settings-controller",
    ),
    ("Where the planning time goes (probe)", "where-the-planning-time-goes-probe"),
    ("Keep-upright in both pipelines", "keep-upright-in-both-pipelines"),
    ("Both pipelines on move_group's octomap", "both-pipelines-on-move_groups-octomap"),
    (
        "Execution in Gazebo: aborts, repositioning, tracking error",
        "execution-in-gazebo-aborts-repositioning-tracking-error",
    ),
    (
        "Collision agreement",
        "collision-agreement-moveits-planning-scene-vs-the-direct-checker",
    ),
    ("Run-to-run: 30 Oct vs 31 Oct", "run-to-run-30-oct-vs-31-oct"),
]


def wall_time_text(csv_paths: list[pathlib.Path], metas: list[dict]) -> str:
    """Each CSV's own wall time, labelled, and their sum (one meta file per CSV)."""
    parts = [
        (p.name, m["total_wall_time_s"])
        for p, m in zip(csv_paths, metas, strict=True)
        if m.get("total_wall_time_s") is not None
    ]
    if not parts:
        return ""
    each = " + ".join(f"{t / 60:.1f} min (`{name}`)" for name, t in parts)
    total = sum(t for _, t in parts)
    return f"wall time {each}" + (f" = {total / 60:.1f} min" if len(parts) > 1 else "")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--csv",
        action="append",
        help="benchmark CSV (repeatable); default results/bench.csv plus results/bench_bitrrt.csv "
        "when it exists",
    )
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument(
        "--print-only", action="store_true", help="print the table, write nothing (CI mini-bench)"
    )
    ap.add_argument(
        "--check",
        action="store_true",
        help="write nothing, make no plots; exit 1 if docs/results.md (outside the direct block) "
        "or results/summary.json is stale",
    )
    ap.add_argument("--results-md", default="docs/results.md")
    ap.add_argument("--summary", default="results/summary.json")
    args = ap.parse_args(argv)
    csv_paths = (
        [pathlib.Path(c) for c in args.csv]
        if args.csv
        else [
            p
            for p in (REPO / "results" / "bench.csv", REPO / "results" / "bench_bitrrt.csv")
            if p.exists()
        ]
    )
    rows = [r for p in csv_paths for r in load_rows(p)]
    if not rows:
        raise SystemExit(f"{csv_paths} have no rows")
    metas = [
        json.loads(m.read_text()) if (m := pathlib.Path(str(p) + ".meta.json")).exists() else {}
        for p in csv_paths
    ]
    summary = summarise(rows)
    classes = by_query_class(rows)

    def relpath(p: pathlib.Path) -> str:
        return str(p.resolve().relative_to(REPO) if p.resolve().is_relative_to(REPO) else p)

    rel = " + ".join(relpath(p) for p in csv_paths)
    commits = sorted({r["commit"] for r in rows})
    queries = sorted({int(r["query_id"]) for r in rows})
    modes = ", ".join(f"`{m}`" for m in sorted({r["mode"] for r in rows}))
    voxels = sorted({m["octomap_voxels"] for m in metas if "octomap_voxels" in m})
    wall = wall_time_text(csv_paths, metas)
    header = (
        f"Source: `{rel}` ({len(rows)} rows, {len(queries)} queries, commit "
        f"{', '.join(f'`{c}`' for c in commits)}, query seed {rows[0]['query_seed']}, OMPL seed "
        f"{rows[0]['ompl_seed']}, modes {modes}, 10 s per cell"
        + (f", octomap {' / '.join(str(v) for v in voxels)} occupied 2 cm voxels" if voxels else "")
        + (f", {wall}" if wall else "")
        + ")."
    )
    table = main_table(summary)
    if args.print_only:
        print(header)
        print(table)
        return 0

    extra_md, extra_regen = extra_sections()
    contents = "\n".join(f"- [{t}](#{a})" for t, a in CONTENTS)
    results_md = (
        f"""# Results

<!-- GENERATED by scripts/aggregate.py and scripts/aggregate_direct.py; do not edit by hand. -->

Every number here comes from the CSVs under `results/` through `scripts/aggregate.py` (v0.1) and
`scripts/aggregate_direct.py` (v0.2 and the run-to-run comparison). Regenerate the CSVs with
`scripts/regenerate_all.sh`, then run both scripts; `--check` on either reports a stale file.

{contents}

{header}

## v0.1: planner × constraint

{table}

- *Success* = MoveIt returned a trajectory that passed the `ValidateSolution` response adapter
  (collision-free against the frozen octomap snapshot and, for `orientation`, inside the path
  constraint at every waypoint) within the 10 s budget. Wilson 95 % intervals.
- *Planning time* and *path lengths* are over successful cells only. Planning time is MoveIt's
  `MotionPlanResponse::planning_time` (OMPL solve + simplification); failed cells leave it empty.
- *Executed path collision-free*: of the successful trajectories, how many stay collision-free
  against the snapshot when the time-parameterized trajectory is re-checked at 0.01 rad joint
  steps (`ValidateSolution` only checks waypoints).
- *Max tool tilt* is the angle between `tool0`'s z axis and world −z, evaluated on the path
  densified to 0.01 rad joint steps (what a joint trajectory controller executes between
  waypoints). The `orientation` constraint allows at most 5° (see docs/planners.md).
- Queries whose start or goal collides with the octomap snapshot: """
        + ", ".join(f"{s['constraint']} {s['invalid_queries']}" for s in summary)
        + f""".

## v0.1: by query class

How the start and goal compartments relate (same compartment, across the central divider,
across the middle board, or diagonal).

{class_table(classes)}

{extra_md}
## Regenerate

| Benchmark | Metric | Value | Regenerate with |
|---|---|---|---|
"""
        + "\n".join(
            f"| shelf, {s['planner']}, {s['constraint']} | success rate | {pct(s['success_rate'])} "
            f"| `scripts/bench_planners.sh && scripts/aggregate.py` |"
            for s in summary
        )
        + "\n"
        + "".join(line + "\n" for line in extra_regen)
        + "\n## Plots\n\n"
        + "\n".join(f"![{pathlib.Path(n).stem}](assets/{n})" for n in PLOT_FILES)
        + "\n"
    )
    results_path = REPO / args.results_md
    # v0.2: scripts/aggregate_direct.py owns the block between these markers (direct pipeline vs
    # MoveIt, collision agreement, run-to-run); regenerating the v0.1 tables must keep it.
    old = results_path.read_text() if results_path.exists() else ""
    if old.count(DIRECT_START) > 1 or old.count(DIRECT_END) > 1:
        raise SystemExit(
            f"{args.results_md}: {DIRECT_START} / {DIRECT_END} must appear at most once "
            f"(found {old.count(DIRECT_START)} / {old.count(DIRECT_END)})"
        )
    if (DIRECT_START in old) != (DIRECT_END in old):
        raise SystemExit(f"{args.results_md}: only one of {DIRECT_START} / {DIRECT_END} present")
    if DIRECT_START in old:
        block = old[old.index(DIRECT_START) : old.index(DIRECT_END) + len(DIRECT_END)]
        results_md += "\n" + block + "\n"
    summary_json = json.dumps(summary, indent=2) + "\n"
    summary_path = REPO / args.summary

    if args.check:
        stale = [
            name
            for name, before, after in (
                (args.results_md, old, results_md),
                (
                    args.summary,
                    summary_path.read_text() if summary_path.exists() else "",
                    summary_json,
                ),
            )
            if before != after
        ]
        if stale:
            print("stale (re-run scripts/aggregate.py): " + ", ".join(stale), file=sys.stderr)
            return 1
        print("aggregate: docs/results.md (v0.1 part) and results/summary.json current")
        return 0

    summary_path.write_text(summary_json)
    if not args.no_plots:
        make_plots(rows, REPO / "docs" / "assets")
    results_path.write_text(results_md)
    print(table)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
