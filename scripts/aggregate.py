#!/usr/bin/env python3
"""Aggregate results/bench.csv into tables (docs/results.md, README section) and plots.

    scripts/aggregate.py [--csv results/bench.csv] [--no-plots]

Everything in docs/results.md and between the README's `results:start/end` markers is written
by this script from the CSV; nothing is typed by hand.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import statistics
from collections import defaultdict

REPO = pathlib.Path(__file__).resolve().parent.parent
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
    for (planner, constraint), rs in sorted(groups.items()):
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
        "success rate (ties: lower median time) became the benchmark planner configuration.",
        "`scripts/bench_planners.sh 20 results/tuning/tuning.csv "
        "results/tuning/queries_seed1.json results/tuning/tuning.yaml`",
    ),
    (
        "results/sweep/octomap_0.02.csv",
        "Octomap resolution sweep: 2 cm (first 20 evaluation queries)",
        "SHOULD item. The first 20 seed-0 queries with the benchmark planner, octomap voxels of "
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
        "timeouts.",
        "`git checkout 2f08131 && scripts/bench_planners.sh 100 "
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


def replace_between(text: str, start: str, end: str, body: str) -> str:
    i, j = text.index(start) + len(start), text.index(end)
    return text[:i] + "\n" + body + "\n" + text[j:]


def main() -> int:
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
    args = ap.parse_args()
    csv_paths = (
        [pathlib.Path(c) for c in args.csv]
        if args.csv
        else [
            p
            for p in (REPO / "results" / "bench.csv", REPO / "results" / "bench_bitrrt.csv")
            if p.exists()
        ]
    )
    csv_path = csv_paths[0]
    rows = [r for p in csv_paths for r in load_rows(p)]
    if not rows:
        raise SystemExit(f"{csv_paths} have no rows")
    meta_path = pathlib.Path(str(csv_path) + ".meta.json")
    meta = json.loads(meta_path.read_text()) if meta_path.exists() else {}
    summary = summarise(rows)
    classes = by_query_class(rows)
    rel = (
        csv_path.resolve().relative_to(REPO)
        if csv_path.resolve().is_relative_to(REPO)
        else csv_path
    )

    commits = sorted({r["commit"] for r in rows})
    queries = sorted({int(r["query_id"]) for r in rows})
    modes = ", ".join(f"`{m}`" for m in sorted({r["mode"] for r in rows}))
    if len(csv_paths) > 1:
        rel = " + ".join(
            f"{p.resolve().relative_to(REPO) if p.resolve().is_relative_to(REPO) else p}"
            for p in csv_paths
        )
    header = (
        f"Source: `{rel}` ({len(rows)} rows, {len(queries)} queries, commit "
        f"{', '.join(f'`{c}`' for c in commits)}, query seed {rows[0]['query_seed']}, OMPL seed "
        f"{rows[0]['ompl_seed']}, modes {modes}, 10 s per cell"
        + (
            f", octomap {meta['octomap_voxels']} occupied 2 cm voxels, "
            f"total wall time {meta['total_wall_time_s'] / 60:.1f} min"
            if meta
            else ""
        )
        + ")."
    )
    table = main_table(summary)
    if args.print_only:
        print(header)
        print(table)
        return 0
    (REPO / "results" / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    plots = [] if args.no_plots else make_plots(rows, REPO / "docs" / "assets")

    extra_md, extra_regen = extra_sections()
    results_md = (
        f"""# Results

<!-- GENERATED by scripts/aggregate.py from {rel}; do not edit by hand. -->

Every number here comes from `{rel}` through `scripts/aggregate.py`. Regenerate the CSV with
`flock ~/work/.locks/gazebo.lock scripts/bench_planners.sh` (or the whole set with
`scripts/regenerate_all.sh`), then run `scripts/aggregate.py`.

{header}

## Planner × constraint (feature prompt step 6)

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

## By query class

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
    )
    if plots:
        results_md += (
            "\n## Plots\n\n" + "\n".join(f"![{p.stem}](assets/{p.name})" for p in plots) + "\n"
        )
    (REPO / "docs" / "results.md").write_text(results_md)

    readme = REPO / "README.md"
    text = readme.read_text()
    if "<!-- results:start -->" in text:
        readme.write_text(
            replace_between(
                text,
                "<!-- results:start -->",
                "<!-- results:end -->",
                f"<!-- generated by scripts/aggregate.py -->\n\n{table}\n\n{header}\n",
            )
        )
    print(table)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
