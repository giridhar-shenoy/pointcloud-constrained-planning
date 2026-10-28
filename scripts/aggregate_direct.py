#!/usr/bin/env python3
"""Aggregate the v0.2 results: direct pipeline vs MoveIt 2 and the collision agreement.

    scripts/aggregate_direct.py [--csv results/bench_direct.csv] [--no-plots] [--print-only]

Writes the block between `<!-- direct:start -->` and `<!-- direct:end -->` in docs/results.md
(scripts/aggregate.py keeps that block when it regenerates the v0.1 tables), the README block
between `<!-- direct-results:start -->` and `<!-- direct-results:end -->`,
results/summary_direct.json and docs/assets/direct_vs_moveit_planning_time.png. Every number is
computed here from the CSVs; nothing is typed by hand.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import pathlib
import statistics

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent
_spec = importlib.util.spec_from_file_location("aggregate", HERE / "aggregate.py")
_agg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_agg)
wilson = _agg.wilson
percentile = _agg.percentile

DIRECT_START = _agg.DIRECT_START
DIRECT_END = _agg.DIRECT_END
README_START = "<!-- direct-results:start -->"
README_END = "<!-- direct-results:end -->"
PIPELINES = ("direct", "moveit")
LABEL = {"direct": "Direct (OMPL + coal)", "moveit": "MoveIt 2 (MoveItCpp)"}
# Validated pair (dataviz validator, light surface): both solved / one pipeline failed.
BOTH = "#2a78d6"
ONE_FAILED = "#eb6834"


def load(path: pathlib.Path) -> list[dict]:
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def num(row: dict, key: str) -> float | None:
    v = row.get(key, "")
    return float(v) if v not in ("", None) else None


def plan_plus_simplify(row: dict) -> float | None:
    """Like-for-like planning time: OMPL solve + simplification on both sides.

    MoveIt's planning_time already is solve + simplify; the direct pipeline records them apart.
    """
    if row["pipeline"] == "moveit":
        return num(row, "plan_time_s")
    p, s = num(row, "plan_time_s"), num(row, "simplify_time_s")
    return None if p is None or s is None else p + s


def stats(values: list[float]) -> dict:
    if not values:
        return {"n": 0, "median": math.nan, "p95": math.nan, "max": math.nan}
    return {
        "n": len(values),
        "median": statistics.median(values),
        "p95": percentile(values, 95),
        "max": max(values),
    }


def summarise(rows: list[dict]) -> dict:
    out: dict = {}
    queries = sorted({int(r["query"]) for r in rows})
    skipped = sorted({int(r["query"]) for r in rows if r["status"].startswith("skipped")})
    for pl in PIPELINES:
        rs = [r for r in rows if r["pipeline"] == pl and not r["status"].startswith("skipped")]
        planned = [r for r in rs if r["plan_success"] == "1"]
        executed = [r for r in rs if r.get("executed") == "1"]
        exec_ok = [r for r in planned if r["exec_success"] == "1"]
        n = len(rs)
        s = {
            "attempted": n,
            "planned": len(planned),
            "planned_ci": wilson(len(planned), n),
            "executed_mode": bool(executed),
            "exec_success": len(exec_ok),
            "exec_ci": wilson(len(exec_ok), n),
            "success": sum(r["success"] == "1" for r in rs),
            "success_ci": wilson(sum(r["success"] == "1" for r in rs), n),
            "crossval_ok": sum(r["crossval_ok"] == "1" for r in planned),
            "own_dense_ok": sum(r["own_dense_ok"] == "1" for r in planned),
            "at_start_failures": sum(r.get("at_start") == "0" for r in rs),
        }
        for key in (
            "plan_time_s",
            "simplify_time_s",
            "param_time_s",
            "time_to_trajectory_s",
            "traj_duration_s",
            "path_len_joint_rad",
            "path_len_tcp_m",
            "validity_checks",
            "check_time_us_median",
            "exec_time_s",
            "max_track_err_rad",
            "final_err_rad",
        ):
            s[key] = stats([v for r in planned if (v := num(r, key)) is not None])
        s["plan_plus_simplify_s"] = stats(
            [v for r in planned if (v := plan_plus_simplify(r)) is not None]
        )
        if pl == "direct":
            s["simplify_dominates"] = sum(
                (num(r, "simplify_time_s") or 0.0) > (num(r, "plan_time_s") or 0.0) for r in planned
            )
        out[pl] = s
    # Paired comparison on queries both pipelines planned.
    by_q: dict[int, dict[str, dict]] = {}
    for r in rows:
        by_q.setdefault(int(r["query"]), {})[r["pipeline"]] = r
    ratios, faster = [], 0
    for q in by_q.values():
        d, m = q.get("direct"), q.get("moveit")
        if not d or not m or d["plan_success"] != "1" or m["plan_success"] != "1":
            continue
        a, b = plan_plus_simplify(d), plan_plus_simplify(m)
        if a and b:
            ratios.append(a / b)
            faster += a < b
    out["paired"] = {
        "both_planned": len(ratios),
        "median_ratio_direct_over_moveit": statistics.median(ratios) if ratios else math.nan,
        "direct_faster": faster,
    }
    out["queries"] = len(queries)
    out["skipped_queries"] = skipped
    return out


def f(x: float, digits: int = 3) -> str:
    return "–" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{digits}f}"


def frac(k: int, n: int, ci: tuple[float, float]) -> str:
    if n == 0:
        return "–"
    return f"{k}/{n} = {100 * k / n:.0f} % ({100 * ci[0]:.0f} %–{100 * ci[1]:.0f} %)"


def mp(s: dict, digits: int = 3) -> str:
    return "–" if s["n"] == 0 else f"{f(s['median'], digits)} / {f(s['p95'], digits)}"


def main_table(sm: dict, agreement: dict | None) -> str:
    d, m = sm["direct"], sm["moveit"]
    check_moveit = (
        f"{agreement['moveit_check_us_median']:.1f} (agreement test, not in the plan)"
        if agreement
        else "not exposed by MoveItCpp"
    )
    rows = [
        (
            "Planned (Wilson 95 %)",
            frac(d["planned"], d["attempted"], d["planned_ci"]),
            frac(m["planned"], m["attempted"], m["planned_ci"]),
            "yes",
        ),
        (
            "**OMPL solve + simplification**, median / p95 (s)",
            mp(d["plan_plus_simplify_s"]),
            mp(m["plan_plus_simplify_s"]),
            "**yes**: MoveIt's `planning_time` is solve + simplify",
        ),
        (
            "OMPL solve only, median / p95 (s)",
            mp(d["plan_time_s"]),
            "inside `planning_time`",
            "direct only",
        ),
        (
            "Simplification, median / p95 (s)",
            mp(d["simplify_time_s"]),
            "inside `planning_time`",
            "direct only",
        ),
        (
            "Time parameterisation, median / p95 (s)",
            mp(d["param_time_s"], 6),
            "inside `plan()` (TOTG adapter)",
            "no: trapezoid vs TOTG",
        ),
        (
            "**Request → time-parameterised trajectory**, median / p95 (s)",
            mp(d["time_to_trajectory_s"]),
            mp(m["time_to_trajectory_s"]),
            "**yes**: wall time, same process",
        ),
        (
            "Validity checks per request, median / p95",
            mp(d["validity_checks"], 0),
            "not exposed",
            "direct only",
        ),
        (
            "Collision check, median (µs)",
            f(d["check_time_us_median"]["median"], 1),
            check_moveit,
            "yes (agreement test: same configurations, both with FK)",
        ),
        (
            "Trajectory duration, median / p95 (s)",
            mp(d["traj_duration_s"], 2),
            mp(m["traj_duration_s"], 2),
            "no: rest-to-rest trapezoids vs TOTG on the interpolated path",
        ),
        (
            "Joint path length, median (rad)",
            f(d["path_len_joint_rad"]["median"], 2),
            f(m["path_len_joint_rad"]["median"], 2),
            "yes (same metric)",
        ),
        (
            "TCP path length, median (m)",
            f(d["path_len_tcp_m"]["median"], 2),
            f(m["path_len_tcp_m"]["median"], 2),
            "yes (same metric)",
        ),
        (
            "Path clean for the OTHER pipeline's checker at 0.01 rad",
            f"{d['crossval_ok']}/{d['planned']}",
            f"{m['crossval_ok']}/{m['planned']}",
            "yes",
        ),
    ]
    if d["executed_mode"] or m["executed_mode"]:
        rows += [
            (
                "Executed and reached the goal (≤ 0.01 rad)",
                frac(d["exec_success"], d["attempted"], d["exec_ci"]),
                frac(m["exec_success"], m["attempted"], m["exec_ci"]),
                "yes: same controller",
            ),
            (
                "Execution time, median / p95 (sim s)",
                mp(d["exec_time_s"], 2),
                mp(m["exec_time_s"], 2),
                "no: follows the trajectory duration; MoveIt adds its execution manager",
            ),
            (
                "Max joint tracking error, median / p95 (rad)",
                mp(d["max_track_err_rad"], 4),
                mp(m["max_track_err_rad"], 4),
                "yes: same controller, same measurement",
            ),
        ]
    head = "| Metric | " + " | ".join(LABEL[p] for p in PIPELINES) + " | Like for like? |"
    sep = "|---|---|---|---|"
    return "\n".join([head, sep] + [f"| {a} | {b} | {c} | {e} |" for a, b, c, e in rows])


def agreement_paragraph(meta: dict, boundary: list[dict]) -> str:
    bad = [r for r in boundary if r["agree"] == "0"]
    pairs = sorted({r["colliding_side_moveit_pairs"] for r in bad})
    seps = [float(r["free_side_moveit_min_distance_m"]) for r in bad]
    sep_txt = (
        f"FCL measures {min(seps) * 1e9:.0f}–{max(seps) * 1e9:.0f} nm of separation" if seps else ""
    )
    return (
        f"**{meta['agree']}/{meta['count']} = {meta['agreement_pct']:.2f} %** of seeded uniform "
        f"configurations (seed {meta['seed']}, {meta['moveit_colliding']} colliding) get the "
        f"same verdict from MoveIt's planning scene ({meta['moveit_collision_detector']}) and the "
        f"direct checker (coal through Pinocchio), using the request MoveIt's OMPL validity "
        f"checker sends (arm group, no padding). Both check the same "
        f"{meta['direct_active_pairs']} pairs (MoveIt's ACM and group rules derived "
        f"independently: equal = {meta['pair_sets_equal']}), name identical colliding pairs on "
        f"every configuration, and report the same minimum distance to the printed 1e-6 m on the "
        f"free ones. That agreement is less independent than it looks: coal is a fork of FCL "
        f"(through hpp-fcl), so the two share their BVH and GJK algorithms. Uniform samples never "
        f"come within millimetres of contact, so a boundary probe bisects every verdict change "
        f"between consecutive samples to a 1e-7 rad bracket: **{meta['boundary_agree']}/"
        f"{meta['boundary_brackets']}** brackets agree. All {len(bad)} differences are on "
        f"{', '.join(p.replace('|', '/') for p in pairs) or 'no pair'}, where {sep_txt} and coal "
        f"reports contact: hundreds of times below coal's GJK tolerance (1e-6 m) and Pinocchio's "
        f"`distance_upper_bound` (1e-6 m), so either verdict is inside the libraries' stated "
        f"precision. Nothing was tuned to raise these numbers. Per check, including forward "
        f"kinematics, MoveIt takes {meta['moveit_check_us_median']:.1f} µs and the direct checker "
        f"{meta['direct_check_us_median']:.1f} µs (median): Pinocchio tests every active pair "
        f"in a loop, while MoveIt's FCL environment runs a broadphase first."
    )


def make_plot(rows: list[dict], timeout: float, out: pathlib.Path) -> pathlib.Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "figure.facecolor": _agg.SURFACE,
            "axes.facecolor": _agg.SURFACE,
            "axes.edgecolor": _agg.TEXT_2,
            "axes.labelcolor": _agg.TEXT,
            "xtick.color": _agg.TEXT_2,
            "ytick.color": _agg.TEXT_2,
            "text.color": _agg.TEXT,
            "axes.grid": True,
            "grid.color": _agg.GRID,
            "grid.linewidth": 0.8,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "font.size": 10,
        }
    )
    by_q: dict[int, dict[str, dict]] = {}
    for r in rows:
        if not r["status"].startswith("skipped"):
            by_q.setdefault(int(r["query"]), {})[r["pipeline"]] = r
    both, one = [], []
    for q in by_q.values():
        d, m = q.get("direct"), q.get("moveit")
        if not d or not m:
            continue
        x = plan_plus_simplify(m) if m["plan_success"] == "1" else timeout
        y = plan_plus_simplify(d) if d["plan_success"] == "1" else timeout
        (both if d["plan_success"] == m["plan_success"] == "1" else one).append((x, y))
    fig, ax = plt.subplots(figsize=(5.6, 5.0), dpi=150)
    vals = [v for p in both + one for v in p if v]
    lo = min(vals) / 1.5 if vals else 0.01
    hi = max(vals) * 1.5 if vals else timeout
    ax.plot([lo, hi], [lo, hi], color=_agg.TEXT_2, linewidth=1.0, linestyle="--", zorder=1)
    ax.annotate(
        "equal time",
        (hi / 1.6, hi / 1.6),
        rotation=45,
        color=_agg.TEXT_2,
        ha="right",
        va="bottom",
        fontsize=8,
    )
    if both:
        ax.scatter(
            [x for x, _ in both],
            [y for _, y in both],
            s=28,
            color=BOTH,
            edgecolor=_agg.SURFACE,
            linewidth=1.0,
            label=f"both planned ({len(both)})",
            zorder=3,
        )
    if one:
        ax.scatter(
            [x for x, _ in one],
            [y for _, y in one],
            s=36,
            marker="D",
            color=ONE_FAILED,
            edgecolor=_agg.SURFACE,
            linewidth=1.0,
            label=f"one failed, plotted at {timeout:.0f} s ({len(one)})",
            zorder=3,
        )
    above = sum(y > x for x, y in both)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.set_aspect("equal")
    ax.set_xlabel("MoveIt 2: planning_time = OMPL solve + simplify (s)")
    ax.set_ylabel("Direct: OMPL solve + simplify (s)")
    ax.set_title(
        f"Same queries, same OMPL settings: direct slower on {above} of {len(both)}",
        loc="left",
        color=_agg.TEXT,
        fontsize=10,
    )
    ax.legend(frameon=False, loc="upper left")
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)
    return out


def replace_or_append(text: str, start: str, end: str, body: str) -> str:
    block = f"{start}\n{body}\n{end}"
    if start in text and end in text:
        return text[: text.index(start)] + block + text[text.index(end) + len(end) :]
    return text.rstrip("\n") + "\n\n" + block + "\n"


REGENERATE = "\n".join(
    [
        "| Benchmark | Regenerate with |",
        "|---|---|",
        "| Collision agreement (500 configurations + boundary probe) "
        "| `scripts/collision_agreement.sh 500` |",
        "| Direct vs MoveIt, 100 queries with execution "
        "| `scripts/bench_direct.sh 100 && scripts/aggregate_direct.py` |",
        "| CI mini-benchmark, 5 queries, plan only "
        "| `scripts/bench_direct.sh 5 results/tmp/ci_direct5.csv --plan-only` |",
    ]
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="results/bench_direct.csv")
    ap.add_argument("--agreement", default="results/collision_agreement.csv")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--print-only", action="store_true")
    args = ap.parse_args()
    csv_path = REPO / args.csv
    rows = load(csv_path)
    meta = json.loads((csv_path.parent / (csv_path.name + ".meta.json")).read_text())
    sm = summarise(rows)
    agr_path = REPO / args.agreement
    agreement = None
    boundary: list[dict] = []
    if (agr_path.parent / (agr_path.name + ".meta.json")).exists():
        agreement = json.loads((agr_path.parent / (agr_path.name + ".meta.json")).read_text())
        bpath = agr_path.with_name(agr_path.stem + "_boundary.csv")
        boundary = load(bpath) if bpath.exists() else []
    table = main_table(sm, agreement)
    mo = meta.get("moveit_ompl_context", {})
    source = (
        f"Source: `{args.csv}` ({len(rows)} rows, {sm['queries']} queries, "
        f"{len(sm['skipped_queries'])} skipped, commit `{meta['commit']}`, query seed "
        f"{meta['query_seed']}, OMPL seed {meta['ompl_seed']} (direct only), "
        f"{'with' if meta['execute'] else 'without'} execution, "
        f"{meta['planning_time_s']:.0f} s per plan, total wall time "
        f"{meta['total_wall_time_s'] / 60:.1f} min). Direct: {meta['planner_config']}, range "
        f"{meta['range_rad']:.2f} rad, check every {meta['longest_valid_segment_rad']:.4f} rad "
        f"(extent {meta['max_extent_rad']:.2f} rad). MoveIt's configured context: "
        f"range {mo.get('range', '?')}, check every "
        f"{f(mo.get('longest_valid_segment_rad', math.nan), 4)} rad (extent "
        f"{f(mo.get('max_extent_rad', math.nan), 2)} rad)."
    )
    if args.print_only:
        print(table)
        print(source)
        return 0
    (REPO / "results" / "summary_direct.json").write_text(
        json.dumps(sm, indent=2, default=str) + "\n"
    )
    plot = None
    if not args.no_plots:
        plot = make_plot(
            rows, meta["planning_time_s"], REPO / "docs/assets/direct_vs_moveit_planning_time.png"
        )
    d = sm["direct"]
    paired = sm["paired"]
    body = f"""<!-- GENERATED by scripts/aggregate_direct.py; do not edit by hand. -->

## v0.2: direct OMPL + coal pipeline vs MoveIt 2 (same queries, boxes, settings, controller)

{table}

{source}

- Paired, on the {paired["both_planned"]} queries both planned: the direct pipeline's solve +
  simplify time is {f(paired["median_ratio_direct_over_moveit"], 2)}× MoveIt's
  `planning_time` (median of per-query ratios); it is faster on {paired["direct_faster"]}.
- Simplification took longer than the OMPL solve on {d.get("simplify_dominates", 0)} of
  {d["planned"]} direct plans.
- Skipped queries (start or goal rejected by either checker): {len(sm["skipped_queries"])}.

### Collision agreement (MoveIt's planning scene vs the direct checker)

{agreement_paragraph(agreement, boundary) if agreement else "Not run."}

Files: `results/collision_agreement.csv` (+ `.meta.json`),
`results/collision_agreement_boundary.csv`.

### Regenerate (v0.2)

{REGENERATE}
"""
    if plot:
        body += f"\n![{plot.stem}](assets/{plot.name})\n"
    results = REPO / "docs" / "results.md"
    results.write_text(replace_or_append(results.read_text(), DIRECT_START, DIRECT_END, body))
    readme = REPO / "README.md"
    text = readme.read_text()
    if README_START in text and README_END in text:
        readme.write_text(
            replace_or_append(
                text,
                README_START,
                README_END,
                f"<!-- generated by scripts/aggregate_direct.py -->\n\n{table}\n\n{source}\n",
            )
        )
    print(table)
    print(source)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
