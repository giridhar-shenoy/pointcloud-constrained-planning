#!/usr/bin/env python3
"""Aggregate the v0.2 results (direct pipeline vs MoveIt 2, collision agreement), the run-to-run
comparison, and every generated README block.

    scripts/aggregate_direct.py [--csv results/bench_direct.csv] [--no-plots] [--print-only]
                                [--check] [--readme README.md] [--results-md docs/results.md]

Writes
- the block between `<!-- direct:start -->` and `<!-- direct:end -->` in docs/results.md
  (scripts/aggregate.py writes the rest of that file and keeps this block);
- the README blocks `<!-- NAME:start -->` ... `<!-- NAME:end -->` for every NAME in
  README_BLOCKS. They combine v0.1 and v0.2 numbers; the v0.1 numbers are computed here with
  aggregate.py's own functions from the same CSVs, so the two scripts can run in either order;
- results/summary_direct.json and the plots docs/assets/direct_vs_moveit_*.png.

`--check` writes nothing and makes no plots: it exits 1 if the direct block of docs/results.md,
a README block or results/summary_direct.json differs from what the CSVs give. Every README
marker pair in README_BLOCKS must appear exactly once; the retired `direct-results` pair must
be gone. Every number is computed here from the CSVs; nothing is typed by hand.
"""

from __future__ import annotations

import argparse
import csv
import datetime
import importlib.util
import json
import math
import pathlib
import statistics
import sys

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent
_spec = importlib.util.spec_from_file_location("aggregate", HERE / "aggregate.py")
_agg = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_agg)
wilson = _agg.wilson
percentile = _agg.percentile

DIRECT_START = _agg.DIRECT_START
DIRECT_END = _agg.DIRECT_END
# README blocks this script owns, in the order the README must place them.
README_BLOCKS = ("headline", "breath", "results", "reproducibility")
# The numbers behind the "Where it fails" items, in either form: one table for every item
# (`failure-numbers`, at the end of that section) and/or small blocks under single items. Each is
# written when present (exactly once); at least one of them must be present.
OPTIONAL_README_BLOCKS = (
    "failure-numbers",
    "f16-numbers",
    "f17-numbers",
    "f26-numbers",
    "f27-numbers",
)
RETIRED_README_BLOCKS = ("direct-results",)
PUBLISHED = "results"
RERUN = "results/rerun_2026-10-31"
PUB_LABEL = "30 Oct"
RERUN_LABEL = "31 Oct"
PIPELINES = ("direct", "moveit")
LABEL = {"direct": "Direct (OMPL + coal)", "moveit": "MoveIt 2 (MoveItCpp)"}
SHORT = {"direct": "direct", "moveit": "MoveIt"}
# Configuration, not measurements: the simulated controller's path tolerance (ur_simulation_gz's
# ur_controllers.yaml) and the dense re-check step every benchmark uses.
PATH_TOLERANCE_RAD = 0.2
DENSE_STEP_RAD = 0.01
# coal's default GJK tolerance: GJK_DEFAULT_TOLERANCE in coal/narrowphase/narrowphase_defaults.h.
GJK_TOLERANCE_M = 1e-6
# UR5e link offsets (m) from the shoulder-lift joint to tool0: the forearm, wrist_1, wrist_2 and
# wrist_3 translations in ur_description's config/ur5e/default_kinematics.yaml (tool0 adds none).
# Their summed lengths bound the distance from the shoulder axes to the tool in any pose
# (scripts/tests compares them with the installed file when it exists).
UR5E_LINK_OFFSETS_M = (
    (-0.425, 0.0, 0.0),
    (-0.3922, 0.0, 0.1333),
    (0.0, -0.0997, 0.0),
    (0.0, 0.0996, 0.0),
)
QUERIES_FILE = "results/queries_boxes.json"


def lever_arm_bound_m() -> float:
    return sum(math.sqrt(x * x + y * y + z * z) for x, y, z in UR5E_LINK_OFFSETS_M)


def query_clearance_m() -> float | None:
    """The query generator's clearance from the shelf (configuration, recorded with the queries)."""
    path = REPO / QUERIES_FILE
    if not path.exists():
        return None
    return json.loads(path.read_text()).get("generator", {}).get("collision_padding_m")


def tool_bound_sentence(dt: dict, mt: dict) -> str:
    """Upper bound on the tool displacement from the p95 tracking error, against the clearance."""
    arm = lever_arm_bound_m()
    clr = query_clearance_m()
    d_cm, m_cm = 100 * dt["p95"] * arm, 100 * mt["p95"] * arm
    txt = (
        f"As an upper bound at the tool, if all of that lag were at the shoulder with the tool as "
        f"far from it as the UR5e's links allow ({arm:.2f} m, the sum of the arm's link lengths "
        f"in the UR5e's standard model), the p95 lag would move the tool {d_cm:.1f} cm "
        f"(direct) and {m_cm:.1f} cm (MoveIt)"
    )
    if clr is None:
        return txt + "."

    def cmp(x_cm: float) -> str:
        return "more than" if x_cm > 100 * clr else "less than"

    same = cmp(d_cm) == cmp(m_cm)
    where = f"the queries' {100 * clr:.0f} cm clearance from the shelf (`{QUERIES_FILE}`)"
    return txt + (
        f", {cmp(d_cm)} {where}; a bound, not a measured distance."
        if same
        else f": {cmp(d_cm)} {where} for direct and {cmp(m_cm)} for MoveIt; a bound, not a "
        f"measured distance."
    )


CONSTRAINT_WORD = {"none": "unconstrained", "orientation": "keep-upright"}
CLASS_ORDER = ("same compartment", "across board", "across divider", "diagonal")
PLAIN_PLOT = "docs/assets/direct_vs_moveit_per_query.png"
LOG_PLOT = "docs/assets/direct_vs_moveit_planning_time.png"
# Validated pair (dataviz validator, light surface): both solved / one pipeline failed.
BOTH = "#2a78d6"
ONE_FAILED = "#eb6834"
COLOR = {"moveit": BOTH, "direct": ONE_FAILED}


# --------------------------------------------------------------------------------------------
# Loading and statistics
# --------------------------------------------------------------------------------------------


def load(path: pathlib.Path) -> list[dict]:
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def load_meta(csv_path: pathlib.Path) -> dict:
    p = csv_path.parent / (csv_path.name + ".meta.json")
    return json.loads(p.read_text()) if p.exists() else {}


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
            "crossval_vs_own_disagreements": sum(
                r["crossval_ok"] != r["own_dense_ok"] for r in planned
            ),
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
            "n_waypoints",
        ):
            s[key] = stats([v for r in planned if (v := num(r, key)) is not None])
        s["plan_plus_simplify_s"] = stats(
            [v for r in planned if (v := plan_plus_simplify(r)) is not None]
        )
        # Time from request to trajectory that is not OMPL solve + simplify: MoveIt's planning
        # context set-up, adapters and TOTG (plan() wall time minus planning_time); for the
        # direct pipeline the path copy, statistics and the trapezoid.
        s["outside_planner_s"] = stats(
            [
                t - p
                for r in planned
                if (t := num(r, "time_to_trajectory_s")) is not None
                and (p := plan_plus_simplify(r)) is not None
            ]
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


def execution(rows: list[dict]) -> dict:
    """Executions that did not reach the goal, by class, plus tracking-error statistics."""
    out: dict = {}
    for pl in PIPELINES:
        rs = [
            r
            for r in rows
            if r["pipeline"] == pl and r.get("executed") == "1" and r["plan_success"] == "1"
        ]
        failed = [r for r in rs if r["exec_success"] != "1"]
        tol = [r for r in failed if (num(r, "max_track_err_rad") or 0.0) >= PATH_TOLERANCE_RAD]
        track = [v for r in rs if (v := num(r, "max_track_err_rad")) is not None]
        track_ok = [
            v for r in rs if r["exec_success"] == "1" and (v := num(r, "max_track_err_rad"))
        ]
        out[pl] = {
            "executed": len(rs),
            "failed": [int(r["query"]) for r in failed],
            "failed_rows": failed,
            "tolerance_aborts": len(tol),
            "tolerance_aborts_clean_both": sum(
                r["crossval_ok"] == "1" and r["own_dense_ok"] == "1" for r in tol
            ),
            "track": stats(track),
            "track_reached_goal": stats(track_ok),
            "track_above_dense_step": sum(v > DENSE_STEP_RAD for v in track),
        }
    return out


def by_class(rows: list[dict]) -> dict[str, dict[str, tuple[int, int]]]:
    """Planned / attempted per query class and pipeline."""
    out: dict[str, dict[str, list[int]]] = {}
    for r in rows:
        if r["status"].startswith("skipped"):
            continue
        cls = _agg.compartment_change(r["start_region"], r["goal_region"])
        k_n = out.setdefault(cls, {}).setdefault(r["pipeline"], [0, 0])
        k_n[0] += r["plan_success"] == "1"
        k_n[1] += 1
    order = [c for c in CLASS_ORDER if c in out] + sorted(set(out) - set(CLASS_ORDER))
    return {c: {p: tuple(out[c].get(p, [0, 0])) for p in PIPELINES} for c in order}


def overlap(rows: list[dict]) -> dict[str, int]:
    by_q: dict[str, dict[str, bool]] = {}
    for r in rows:
        if not r["status"].startswith("skipped"):
            by_q.setdefault(r["query"], {})[r["pipeline"]] = r["plan_success"] == "1"
    v = [q for q in by_q.values() if len(q) == 2]
    return {
        "direct_only_failed": sum(not q["direct"] and q["moveit"] for q in v),
        "moveit_only_failed": sum(q["direct"] and not q["moveit"] for q in v),
        "both_failed": sum(not q["direct"] and not q["moveit"] for q in v),
    }


def checker_share(rows: list[dict], solved: bool) -> float:
    """Median estimated share of direct solve + simplify spent in the collision checker:
    validity checks x median check time / (solve + simplify), per query."""
    vals = []
    for r in rows:
        if r["pipeline"] != "direct" or (r["plan_success"] == "1") != solved:
            continue
        c, us = num(r, "validity_checks"), num(r, "check_time_us_median")
        p, s = num(r, "plan_time_s"), num(r, "simplify_time_s")
        if None in (c, us, p) or (p or 0) + (s or 0) <= 0:
            continue
        vals.append(c * us * 1e-6 / (p + (s or 0.0)))
    return statistics.median(vals) if vals else math.nan


def status_counts(rows: list[dict], pipeline: str) -> dict[str, int]:
    out: dict[str, int] = {}
    for r in rows:
        if r["pipeline"] == pipeline and r["plan_success"] != "1":
            out[r["status"]] = out.get(r["status"], 0) + 1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))


def probe_ok(probe: list[dict]) -> list[dict]:
    return [e for e in probe if e["moveit"].get("success") and e["direct"].get("success")]


def probe_medians(probe: list[dict]) -> dict:
    ok = probe_ok(probe)
    if not ok:
        return {}

    def med(side: str, key: str) -> float:
        return statistics.median(float(e[side][key]) for e in ok)

    def share(side: str) -> float:
        return statistics.median(
            float(e[side]["check_time_total_s"])
            / (float(e[side]["plan_time_s"]) + float(e[side]["simplify_time_s"]))
            for e in ok
        )

    def mean_us(side: str) -> float:
        return statistics.median(
            1e6 * float(e[side]["check_time_total_s"]) / float(e[side]["validity_checks"])
            for e in ok
        )

    return {
        "n": len(ok),
        **{
            f"{side}_{key}": med(side, key)
            for side in PIPELINES
            for key in ("validity_checks", "check_time_us_median", "plan_time_s", "simplify_time_s")
        },
        "direct_share": share("direct"),
        "moveit_share": share("moveit"),
        "direct_mean_us": mean_us("direct"),
        "moveit_mean_us": mean_us("moveit"),
    }


def first_run_cascade(rows: list[dict]) -> dict | None:
    """The first full run without recovery: the abort after which no run started at its start."""
    rs = sorted(rows, key=lambda r: (int(r["query"]), int(r.get("order") or 0)))
    for i, r in enumerate(rs):
        if r.get("executed") == "1" and r["plan_success"] == "1" and r["exec_success"] != "1":
            later = rs[i + 1 :]
            if later and all(x.get("at_start") == "0" for x in later):
                return {
                    "query": int(r["query"]),
                    "pipeline": r["pipeline"],
                    "later_runs": len(later),
                }
    return None


def tuning_choice(rows: list[dict]) -> dict:
    """The stated rule: best success rate, ties broken by the lower median time."""
    sm = _agg.summarise(rows)
    best = max(
        sm,
        key=lambda s: (
            s["successes"],
            -(s["time_median_s"] if not math.isnan(s["time_median_s"]) else 1e9),
        ),
    )
    return {"choice": best["planner"], "cells": sm}


class Run:
    """Every CSV of one full run (the published one in results/, or a rerun directory)."""

    def __init__(self, root: pathlib.Path, label: str, direct_csv: pathlib.Path | None = None):
        self.root, self.label = root, label
        v01 = [root / "bench.csv", root / "bench_bitrrt.csv"]
        self.v01_paths = [p for p in v01 if p.exists()]
        self.v01_rows = [r for p in self.v01_paths for r in load(p)]
        self.v01 = _agg.summarise(self.v01_rows) if self.v01_rows else []
        self.v01_metas = [load_meta(p) for p in self.v01_paths]
        self.direct_csv = direct_csv or root / "bench_direct.csv"
        self.direct_rows = load(self.direct_csv)
        self.direct_meta = load_meta(self.direct_csv)
        self.direct = summarise(self.direct_rows)
        self.exec = execution(self.direct_rows)
        self.upright_csv = root / "bench_direct_upright.csv"
        self.upright_rows = load(self.upright_csv) if self.upright_csv.exists() else []
        self.upright_meta = load_meta(self.upright_csv)
        self.upright = summarise(self.upright_rows) if self.upright_rows else None
        self.octomap_csv = root / "bench_direct_octomap.csv"
        self.octomap_rows = load(self.octomap_csv) if self.octomap_csv.exists() else []
        self.octomap_meta = load_meta(self.octomap_csv)
        self.octomap = summarise(self.octomap_rows) if self.octomap_rows else None
        agr = root / "collision_agreement.csv"
        self.agreement = load_meta(agr) or None
        b = root / "collision_agreement_boundary.csv"
        self.boundary = load(b) if b.exists() else []
        probe_path = root / "check_probe.json"
        probe = json.loads(probe_path.read_text()) if probe_path.exists() else {}
        self.probe = probe.get("moveit_check_probe", [])
        self.probe_commit = probe.get("commit", "?")
        self.probe_md = probe_medians(self.probe)
        self.tuning = {
            name: tuning_choice(load(root / "tuning" / f"{name}.csv"))
            for name in ("tuning", "tuning_bitrrt")
            if (root / "tuning" / f"{name}.csv").exists()
        }
        self.sweep = {
            res: load(root / "sweep" / f"octomap_{res}.csv")
            for res in ("0.02", "0.05")
            if (root / "sweep" / f"octomap_{res}.csv").exists()
        }

    def v01_cell(self, planner_prefix: str, constraint: str) -> dict | None:
        for s in self.v01:
            if s["planner"].startswith(planner_prefix) and s["constraint"] == constraint:
                return s
        return None

    def commits(self) -> list[str]:
        cs = {m.get("commit") for m in self.v01_metas + [self.direct_meta]}
        cs |= {self.upright_meta.get("commit"), self.octomap_meta.get("commit")}
        cs |= {(self.agreement or {}).get("commit"), self.probe_commit}
        return sorted(c for c in cs if c and c != "?")

    def finished_dates(self) -> tuple[str, str]:
        extra = [
            load_meta(p)
            for p in [self.root / "tuning" / f"{n}.csv" for n in ("tuning", "tuning_bitrrt")]
            + [self.root / "sweep" / f"octomap_{r}.csv" for r in ("0.02", "0.05")]
        ]
        ts = [
            m["finished_utc"]
            for m in self.v01_metas
            + [self.direct_meta, self.upright_meta, self.octomap_meta]
            + extra
            if m.get("finished_utc")
        ]
        days = sorted(datetime.datetime.fromisoformat(t.replace("Z", "+00:00")) for t in ts)
        fmt = "%-d %b"
        return (days[0].strftime(fmt), days[-1].strftime(fmt)) if days else ("?", "?")


# --------------------------------------------------------------------------------------------
# Formatting
# --------------------------------------------------------------------------------------------


def f(x: float | None, digits: int = 3) -> str:
    return "–" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{digits}f}"


def frac(k: int, n: int, ci: tuple[float, float]) -> str:
    if n == 0:
        return "–"
    return f"{k}/{n} = {100 * k / n:.0f} % ({100 * ci[0]:.0f} %–{100 * ci[1]:.0f} %)"


def kn(k: int, n: int) -> str:
    return f"{k}/{n}"


def ci_txt(k: int, n: int) -> str:
    lo, hi = wilson(k, n)
    return f"{100 * lo:.0f} %–{100 * hi:.0f} %"


def mp(s: dict, digits: int = 3) -> str:
    return "–" if s["n"] == 0 else f"{f(s['median'], digits)} / {f(s['p95'], digits)}"


def signed_pct(new: float, old: float) -> str:
    p = 100.0 * (new / old - 1.0)
    return "0 %" if round(p) == 0 else f"{p:+.0f} %".replace("-", "−")


def range_name(planner_label: str) -> str:
    """'RRTConnect (range 2 rad)' -> 'range 2 rad'; auto range -> 'the automatic range (...)'."""
    inner = planner_label.split(" (", 1)[1].rstrip(")") if " (" in planner_label else planner_label
    if inner.startswith("auto range"):
        return f"the automatic range ({inner.split(', ', 1)[1]})" if ", " in inner else inner
    return inner


def range_key(planner_label: str) -> float:
    inner = range_name(planner_label)
    try:
        return float(inner.split()[1])
    except (IndexError, ValueError):
        return math.inf


def pct_value(new: float, old: float) -> float:
    return 100.0 * (new / old - 1.0)


def about(ratio: float) -> str:
    """A ratio for prose: one repetition does not support two decimals."""
    return f"about {ratio:.0f}×" if ratio >= 2 else f"about {ratio:.1f}×"


def join_words(items: list[str]) -> str:
    if len(items) <= 1:
        return "".join(items)
    return ", ".join(items[:-1]) + " and " + items[-1]


def table(head: list[str], rows: list[list[str]]) -> str:
    lines = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    lines += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(lines)


# --------------------------------------------------------------------------------------------
# docs/results.md: v0.2 sections
# --------------------------------------------------------------------------------------------


def main_table(sm: dict, agreement: dict | None, probe: list[dict] | None = None) -> str:
    d, m = sm["direct"], sm["moveit"]
    # Per-check cost inside each planner comes from the probe (same queries, both measured inside
    # their own planner). The agreement test's microbenchmark is NOT comparable to an in-planner
    # median (random, mostly colliding configurations, cold caches), so it is not used here.
    ok = probe_ok(probe or [])
    if ok:
        check_row = (
            f"Collision check inside the planner, median over {len(ok)} probe queries (µs)",
            f(statistics.median(e["direct"]["check_time_us_median"] for e in ok), 1),
            f(statistics.median(e["moveit"]["check_time_us_median"] for e in ok), 1),
            "yes: same queries, each measured inside its own planner (probe; see its source "
            "below the table)",
        )
    else:
        check_row = (
            "Collision check inside the planner, median (µs)",
            f(d["check_time_us_median"]["median"], 1),
            "not exposed by MoveItCpp (run scripts/check_probe.sh)",
            "direct only",
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
        check_row,
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
            "Path clean for the OTHER pipeline's checker at 0.01 rad (cross-validation)",
            f"{d['crossval_ok']}/{d['planned']}",
            f"{m['crossval_ok']}/{m['planned']}",
            "yes",
        ),
        (
            "Path clean for its OWN checker at 0.01 rad",
            f"{d['own_dense_ok']}/{d['planned']}",
            f"{m['own_dense_ok']}/{m['planned']}",
            "yes (a failure here is a discretisation miss, not a checker disagreement)",
        ),
    ]
    if d["executed_mode"] or m["executed_mode"]:
        rows += [
            (
                "Executed and reached the goal (≤ 0.01 rad)",
                frac(d["exec_success"], d["attempted"], d["exec_ci"]),
                frac(m["exec_success"], m["attempted"], m["exec_ci"]),
                "same controller, but two parameterisers of different aggressiveness (see the "
                "duration row)",
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
    below = (
        f"at least {GJK_TOLERANCE_M / max(seps):.0f} times below coal's default GJK tolerance "
        f"(1e-6 m, `GJK_DEFAULT_TOLERANCE` in coal's `narrowphase_defaults.h`), so either verdict "
        f"is inside that tolerance"
        if seps and max(seps) > 0
        else "inside coal's default GJK tolerance (1e-6 m)"
    )
    diffs = (
        f"All {len(bad)} differences are on "
        f"{', '.join(p.replace('|', '/') for p in pairs) or 'no pair'}, where {sep_txt} and coal "
        f"reports contact: {below}. "
        if bad
        else ""
    )
    return (
        f"**{meta['agree']}/{meta['count']} = {meta['agreement_pct']:.2f} %** of seeded uniform "
        f"configurations (seed {meta['seed']}, {meta['moveit_colliding']} colliding, commit "
        f"`{meta.get('commit', '?')}`) get the "
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
        f"{meta['boundary_brackets']}** brackets agree. {diffs}Nothing was tuned to raise these "
        f"numbers. On these random configurations "
        f"(two thirds colliding, so both stop early; cold caches) a check including forward "
        f"kinematics takes {meta['moveit_check_us_median']:.1f} µs in MoveIt and "
        f"{meta['direct_check_us_median']:.1f} µs in the direct checker (median). Inside the "
        f"planners, where most states are free, the gap is different (the probe above); why is "
        f"not measured (the hypothesis is in docs/planners.md)."
    )


def upright_table(run: Run, meta: dict) -> str:
    """Keep-upright (v0.1's orientation constraint) planned by both pipelines."""
    rows = run.upright_rows
    sm = run.upright
    tilt = {
        pl: [
            float(r["max_tilt_deg"])
            for r in rows
            if r["pipeline"] == pl and r["plan_success"] == "1" and r.get("max_tilt_deg")
        ]
        for pl in PIPELINES
    }
    paired = sm["paired"]
    lines = [
        "| Keep-upright (5°), plan only | " + " | ".join(LABEL[p] for p in PIPELINES) + " |",
        "|---|---|---|",
        "| Planned (Wilson 95 %) | "
        + " | ".join(
            frac(sm[p]["planned"], sm[p]["attempted"], sm[p]["planned_ci"]) for p in PIPELINES
        )
        + " |",
        "| OMPL solve + simplification, median / p95 (s), each over its own solved queries | "
        + " | ".join(mp(sm[p]["plan_plus_simplify_s"]) for p in PIPELINES)
        + " |",
        f"| Paired on the {paired['both_planned']} queries both solved: direct / MoveIt "
        f"solve + simplify, median of per-query ratios | "
        f"{f(paired['median_ratio_direct_over_moveit'], 2)}× (direct faster on "
        f"{paired['direct_faster']}) | – |",
        "| Request → trajectory, median / p95 (s) | "
        + " | ".join(mp(sm[p]["time_to_trajectory_s"]) for p in PIPELINES)
        + " |",
        "| Max tool tilt along the path, median / worst (deg) | "
        + " | ".join(
            f"{f(statistics.median(tilt[p]), 1)} / {f(max(tilt[p]), 1)}" if tilt[p] else "–"
            for p in PIPELINES
        )
        + " |",
        "| Path waypoints, median | "
        + " | ".join(f(sm[p]["n_waypoints"]["median"], 1) for p in PIPELINES)
        + " |",
        "| Path clean for the other pipeline's checker | "
        + " | ".join(f"{sm[p]['crossval_ok']}/{sm[p]['planned']}" for p in PIPELINES)
        + " |",
        "| Validity checks per request, median (solved queries) | "
        + f(sm["direct"]["validity_checks"]["median"], 0)
        + " | not exposed |",
        "| Estimated share of solve + simplify in the collision checker, median (solved / failed "
        "queries) | "
        + f"{100 * checker_share(rows, True):.1f} % / {100 * checker_share(rows, False):.2f} %"
        + " | not exposed |",
    ]
    source = (
        f"Source: `{meta.get('csv', '')}` ({len(rows)} rows, commit `{meta['commit']}`, query seed "
        f"{meta.get('query_seed', '?')}, OMPL seed {meta.get('ompl_seed', '?')} (direct only), "
        f"{meta['planning_time_s']:.0f} s per plan, total wall time "
        f"{meta['total_wall_time_s'] / 60:.1f} min)."
    )
    return "\n".join(lines) + "\n\n" + source


def upright_details(run: Run) -> str:
    rows = run.upright_rows
    cls = by_class(rows)
    ov = overlap(rows)
    head = ["Query class", "Direct planned", "MoveIt planned"]
    body = []
    for c, v in cls.items():
        body.append(
            [c]
            + [
                f"{v[p][0]}/{v[p][1]} ({ci_txt(*v[p])})" if v[p][1] else "–"
                for p in ("direct", "moveit")
            ]
        )
    st = status_counts(rows, "direct")
    gap = largest_gap_classes(cls)
    return (
        "By query class (how the start and goal compartments relate; Wilson 95 % intervals):\n\n"
        + table(head, body)
        + "\n\n"
        + f"- Failure overlap: the direct planner fails {ov['direct_only_failed']} queries MoveIt "
        f"solves; MoveIt fails {ov['moveit_only_failed']} the direct planner solves; both fail "
        f"{ov['both_failed']}.\n"
        + (f"- The direct planner's deficit is largest {join_words(gap)}.\n" if gap else "")
        + "- Direct failures by OMPL status: "
        + ", ".join(f"{k} {v}" for k, v in st.items())
        + ".\n"
        "- The checker share is estimated as validity checks × median check time / (solve + "
        "simplify) per query, because the CSV records the median check time, not the total. It "
        "is small in keep-upright mode, so the per-check cost cannot explain the direct "
        "planner's lower success rate; the cause is not measured (docs/planners.md).\n"
        "- The waypoint counts are an observation, not an explanation: OMPL's simplifier "
        "leaves the direct paths with few vertices, while MoveIt's constrained paths keep the "
        "interpolated geodesic states."
    )


def largest_gap_classes(cls: dict) -> list[str]:
    """Classes whose direct-vs-MoveIt success-rate gap is at least half the largest gap."""
    gaps = {
        c: (v["moveit"][0] - v["direct"][0]) / v["direct"][1]
        for c, v in cls.items()
        if v["direct"][1]
    }
    top = max(gaps.values(), default=0.0)
    if top <= 0:
        return []
    names = {"across divider": "across the divider", "across board": "across the board"}
    return [names.get(c, c) for c, g in gaps.items() if g >= top / 2]


def octomap_table(run: Run, meta: dict) -> str:
    """Both pipelines on move_group's octomap (one frozen map, plan only)."""
    sm = run.octomap
    rows = run.octomap_rows
    lines = [
        "| Octomap scene, plan only | " + " | ".join(LABEL[p] for p in PIPELINES) + " |",
        "|---|---|---|",
        "| Planned (Wilson 95 %) | "
        + " | ".join(
            frac(sm[p]["planned"], sm[p]["attempted"], sm[p]["planned_ci"]) for p in PIPELINES
        )
        + " |",
        "| OMPL solve + simplification, median / p95 (s) | "
        + " | ".join(mp(sm[p]["plan_plus_simplify_s"]) for p in PIPELINES)
        + " |",
        f"| Paired on the {sm['paired']['both_planned']} queries both planned: direct / MoveIt, "
        f"median of per-query ratios | {f(sm['paired']['median_ratio_direct_over_moveit'], 2)}× "
        "| – |",
        "| Collision check, median (µs) | "
        + f(sm["direct"]["check_time_us_median"]["median"], 1)
        + " | not exposed |",
        "| Path clean for the other pipeline's checker / its own (0.01 rad) | "
        + " | ".join(
            f"{sm[p]['crossval_ok']} / {sm[p]['own_dense_ok']} of {sm[p]['planned']}"
            for p in PIPELINES
        )
        + " |",
    ]
    skipped = len(sm["skipped_queries"])
    source = (
        f"Source: `{meta.get('csv', '')}` ({len(rows)} rows, {skipped} queries skipped because "
        f"a start or goal collides with the map for either checker, commit `{meta['commit']}`, "
        f"query seed {meta.get('query_seed', '?')}, OMPL seed {meta.get('ompl_seed', '?')} "
        f"(direct only), "
        f"{meta.get('octomap_occupied_leaves', '?')} occupied "
        f"{100 * meta.get('octomap_resolution_m', 0):.0f} cm leaves, total wall time "
        f"{meta['total_wall_time_s'] / 60:.1f} min)."
    )
    return "\n".join(lines) + "\n\n" + source


def probe_table(probe: list[dict], commit: str) -> str:
    """Where the planning time goes: MoveIt's planner core (its own planning context with a
    counting validity checker) vs the direct planner, on the same queries."""
    md = probe_medians(probe)
    if not md:
        return "Probe not run."
    rows = [
        (
            "Validity checks per request",
            f(md["direct_validity_checks"], 0),
            f(md["moveit_validity_checks"], 0),
        ),
        (
            "Time per check inside the planner, median (µs)",
            f(md["direct_check_time_us_median"], 1),
            f(md["moveit_check_time_us_median"], 1),
        ),
        (
            "Time per check, total / count (µs)",
            f(md["direct_mean_us"], 1),
            f(md["moveit_mean_us"], 1),
        ),
        ("OMPL solve (s)", f(md["direct_plan_time_s"]), f(md["moveit_plan_time_s"])),
        (
            "Simplification (s)",
            f(md["direct_simplify_time_s"]),
            f(md["moveit_simplify_time_s"]),
        ),
        (
            "Share of solve + simplify spent in the checker",
            f"{100 * md['direct_share']:.1f} %",
            f"{100 * md['moveit_share']:.1f} %",
        ),
    ]
    return (
        "\n".join(
            [
                f"| Median over {md['n']} queries | Direct planner | MoveIt's planner core |",
                "|---|---|---|",
            ]
            + [f"| {a} | {b} | {c} |" for a, b, c in rows]
        )
        + f"\n\nSource: `results/check_probe.json` ({len(probe)} queries, one process per query, "
        f"commit `{commit}`).\n\n"
        "MoveIt's validity checker returns a cached verdict when a state's validity is already "
        "known (`isValidityKnown`); the probe counts and times those returns as checks, while "
        "the direct checker has no cache. MoveIt's check count is therefore biased up and its "
        "per-check time down, by an amount the probe does not measure."
    )


def execution_section(run: Run) -> str:
    ex = run.exec
    meta = run.direct_meta
    lines = []
    for pl in PIPELINES:
        e = ex[pl]
        if not e["executed"]:
            continue
        fail_rows = e["failed_rows"]
        if fail_rows:
            desc = "; ".join(
                f"Query {r['query']}: {r['exec_error'] or 'failed'}, max tracking error "
                f"{f(num(r, 'max_track_err_rad'), 4)} rad, final error "
                f"{f(num(r, 'final_err_rad'), 4)} rad, clean on both dense checks: "
                f"{'yes' if r['crossval_ok'] == r['own_dense_ok'] == '1' else 'no'}"
                for r in fail_rows
            )
            ab = e["tolerance_aborts"]
            which = (
                ("it was" if ab == 1 else "all were")
                if ab == len(fail_rows)
                else f"{ab} of them {'was' if ab == 1 else 'were'}"
            )
            lines.append(
                f"- {LABEL[pl]}: {len(fail_rows)} of {e['executed']} executions did not reach "
                f"the goal; {which} a path-tolerance abort (tracking error at or above the "
                f"controller's {PATH_TOLERANCE_RAD} rad path tolerance). {desc}."
            )
        else:
            lines.append(
                f"- {LABEL[pl]}: all {e['executed']} executions reached the goal; none aborted."
            )
    lines.append(repositioning_sentence(meta))
    cascade = (
        first_run_cascade(load(REPO / "results/runs/bench_direct_run1_no_recovery.csv"))
        if (REPO / "results/runs/bench_direct_run1_no_recovery.csv").exists()
        else None
    )
    if cascade:
        lines.append(
            f"- The first full run, before the harness could free the arm "
            f"(`results/runs/bench_direct_run1_no_recovery.csv`): {SHORT[cascade['pipeline']]}'s "
            f"query {cascade['query']} did not reach the goal, and all {cascade['later_runs']} "
            f"later runs failed because the arm was not at their start."
        )
    track_rows = []
    for pl in PIPELINES:
        e = ex[pl]
        if not e["executed"]:
            continue
        t, tg = e["track"], e["track_reached_goal"]
        track_rows.append(
            [
                LABEL[pl],
                f"{f(t['median'], 4)} / {f(t['p95'], 4)} / {f(t['max'], 4)}",
                f"{f(tg['median'], 4)} / {f(tg['p95'], 4)} / {f(tg['max'], 4)}",
                f"{e['track_above_dense_step']}/{e['executed']}",
                f"{t['p95'] / DENSE_STEP_RAD:.1f}×",
            ]
        )
    track = table(
        [
            "Max joint tracking error (rad)",
            "All executions: median / p95 / max",
            "Executions that reached the goal: median / p95 / max",
            f"Executions above the {DENSE_STEP_RAD} rad dense-check step",
            "p95 / dense-check step",
        ],
        track_rows,
    )
    return (
        "\n".join(lines)
        + "\n\n"
        + track
        + "\n\nTracking error is `/joint_states` against the controller's own reference, the "
        "largest joint deviation along each execution. "
        + (
            tool_bound_sentence(ex["direct"]["track"], ex["moveit"]["track"]) + " "
            if ex["direct"]["executed"] and ex["moveit"]["executed"]
            else ""
        )
        + "Execution-time contact is not measured (no contact sensor, no column), so reaching the "
        "goal says nothing about whether the executed motion touched the shelf."
    )


def repositioning_sentence(meta: dict) -> str:
    att, fail = meta.get("repositions_attempted", 0), meta.get("reposition_failures", 0)
    esc, esc_f = meta.get("escapes", 0), meta.get("escape_failures", 0)
    return (
        f"- Repositioning moves (direct planner, to each run's start, not measured runs): {att} "
        f"attempted; {fail} still off their start after a second attempt. An attempt that aborts "
        f"is retried and is not counted as a failure, so this count hides aborts. The harness "
        f"had to free the arm from contact with the shelf {esc} time{'s' if esc != 1 else ''} "
        f"before a repositioning move (an unchecked straight joint-space move to the nearest "
        f"collision-free configuration; {esc_f} failed)."
    )


# --------------------------------------------------------------------------------------------
# Run-to-run comparison
# --------------------------------------------------------------------------------------------


class Comparison:
    """30 Oct (published) vs 31 Oct (rerun): count rows judged against the published Wilson
    interval, timing rows reported as raw changes."""

    def __init__(self, pub: Run, re: Run):
        self.pub, self.re = pub, re
        self.counts: list[dict] = []  # success counts
        self.rechecks: list[dict] = []  # dense-check / cross-check counts
        self.timings: list[dict] = []
        self.other: list[list[str]] = []
        self._build()

    def _count(
        self, kind: str, section: str, label: str, kp, np_, kr, nr, key: str = "", say: str = ""
    ):
        lo, hi = wilson(kp, np_)
        rate = kr / nr if nr else math.nan
        row = {
            "section": section,
            "label": label,
            "pub": (kp, np_),
            "re": (kr, nr),
            "inside": lo - 1e-12 <= rate <= hi + 1e-12,
            "key": key,
            "say": say or label,
        }
        (self.counts if kind == "success" else self.rechecks).append(row)

    def _timing(
        self, section: str, label: str, a: float, b: float, digits: int = 3, short: str = ""
    ):
        if a is None or b is None or math.isnan(a) or math.isnan(b) or a == 0:
            return
        self.timings.append(
            {
                "section": section,
                "label": label,
                "pub": a,
                "re": b,
                "digits": digits,
                "short": short or label,
            }
        )

    def _build(self):
        p, r = self.pub, self.re
        for planner in ("RRTConnect", "BiTRRT"):
            for c in ("none", "orientation"):
                a, b = p.v01_cell(planner, c), r.v01_cell(planner, c)
                if not a or not b:
                    continue
                name = f"{a['planner']}, {CONSTRAINT_WORD.get(c, c)}"
                self._count(
                    "success",
                    "v0.1",
                    f"{name}: solved",
                    a["successes"],
                    a["n"],
                    b["successes"],
                    b["n"],
                    key=f"v01_{planner}_{c}",
                )
                pname = a["planner"].split(" (")[0]
                self._timing(
                    "v0.1",
                    f"{name}: planning time median (s)",
                    a["time_median_s"],
                    b["time_median_s"],
                    short=f"{CONSTRAINT_WORD.get(c, c)} planning time of "
                    + ("RRTConnect" if pname == "RRTConnect" else f"{pname}, v0.1's second planner")
                    + " (v0.1)",
                )
                if a["dense_checked"] and b["dense_checked"]:
                    self._count(
                        "recheck",
                        "v0.1",
                        f"{name}: solved paths clean on the dense check",
                        a["dense_ok"],
                        a["dense_checked"],
                        b["dense_ok"],
                        b["dense_checked"],
                    )
        for pl in PIPELINES:
            a, b = p.direct[pl], r.direct[pl]
            self._count(
                "success",
                "v0.2",
                f"{SHORT[pl]}: planned",
                a["planned"],
                a["attempted"],
                b["planned"],
                b["attempted"],
                key=f"v02_planned_{pl}",
            )
            if a["executed_mode"] and b["executed_mode"]:
                self._count(
                    "success",
                    "v0.2",
                    f"{SHORT[pl]}: executed and reached the goal",
                    a["exec_success"],
                    a["attempted"],
                    b["exec_success"],
                    b["attempted"],
                    key=f"v02_exec_{pl}",
                    say=f"{'MoveIt' if pl == 'moveit' else 'the direct pipeline'}'s "
                    "executed-to-goal count",
                )
        for pl in PIPELINES:
            a, b = p.direct[pl], r.direct[pl]
            self._timing(
                "v0.2",
                f"{SHORT[pl]}: OMPL solve + simplification median (s)",
                a["plan_plus_simplify_s"]["median"],
                b["plan_plus_simplify_s"]["median"],
                short=f"{SHORT[pl]} planning time (v0.2)",
            )
            self._timing(
                "v0.2",
                f"{SHORT[pl]}: request → trajectory median (s)",
                a["time_to_trajectory_s"]["median"],
                b["time_to_trajectory_s"]["median"],
                short=f"{SHORT[pl]} request-to-trajectory time (v0.2)",
            )
            self._count(
                "recheck",
                "v0.2",
                f"{SHORT[pl]}: path clean for its own checker",
                a["own_dense_ok"],
                a["planned"],
                b["own_dense_ok"],
                b["planned"],
            )
        if p.probe_md and r.probe_md:
            for pl in PIPELINES:
                self._timing(
                    "v0.2",
                    f"{SHORT[pl]}: collision check inside the planner, probe median (µs)",
                    p.probe_md[f"{pl}_check_time_us_median"],
                    r.probe_md[f"{pl}_check_time_us_median"],
                    digits=1,
                    short=f"{SHORT[pl]} time per collision check (v0.2 probe)",
                )
        a, b = p.direct["paired"], r.direct["paired"]
        self.other.append(
            [
                "v0.2",
                "paired planning-time ratio, direct / MoveIt (median of per-query ratios)",
                f"{f(a['median_ratio_direct_over_moveit'], 2)}× (direct faster on "
                f"{a['direct_faster']})",
                f"{f(b['median_ratio_direct_over_moveit'], 2)}× (direct faster on "
                f"{b['direct_faster']})",
                "derived from the timings above",
            ]
        )
        for tag, x, y in (
            ("keep-upright", p.upright, r.upright),
            ("octomap", p.octomap, r.octomap),
        ):
            if not x or not y:
                continue
            for pl in PIPELINES:
                self._count(
                    "success",
                    f"v0.2 {tag}",
                    f"{tag}, {SHORT[pl]}: planned",
                    x[pl]["planned"],
                    x[pl]["attempted"],
                    y[pl]["planned"],
                    y[pl]["attempted"],
                    key=f"{tag}_planned_{pl}",
                )
            for pl in PIPELINES:
                self._timing(
                    f"v0.2 {tag}",
                    f"{tag}, {SHORT[pl]}: OMPL solve + simplification median (s)",
                    x[pl]["plan_plus_simplify_s"]["median"],
                    y[pl]["plan_plus_simplify_s"]["median"],
                    short=f"{SHORT[pl]} keep-upright planning time (v0.2)"
                    if tag == "keep-upright"
                    else f"{SHORT[pl]} planning time on the octomap (v0.2)",
                )
        if p.agreement and r.agreement:
            self._count(
                "success",
                "v0.2 agreement",
                "collision agreement: same verdict",
                p.agreement["agree"],
                p.agreement["count"],
                r.agreement["agree"],
                r.agreement["count"],
                key="agreement",
            )
            self._count(
                "success",
                "v0.2 agreement",
                "collision agreement: boundary brackets that agree",
                p.agreement["boundary_agree"],
                p.agreement["boundary_brackets"],
                r.agreement["boundary_agree"],
                r.agreement["boundary_brackets"],
                key="boundary",
            )
        for name, title in (("tuning", "RRTConnect"), ("tuning_bitrrt", "BiTRRT")):
            if name in p.tuning and name in r.tuning:
                a, b = p.tuning[name]["choice"], r.tuning[name]["choice"]
                self.other.append(
                    [
                        "v0.1 tuning",
                        f"{title} range chosen by the tuning rule (20 tuning queries)",
                        a,
                        b,
                        "same" if a == b else "**differs**",
                    ]
                )

    # -- summaries ---------------------------------------------------------------------------

    def outside(self, rows: list[dict]) -> list[dict]:
        return [c for c in rows if not c["inside"]]

    def timing_range(self) -> tuple[float, float]:
        ps = [pct_value(t["re"], t["pub"]) for t in self.timings]
        return (min(ps), max(ps)) if ps else (math.nan, math.nan)

    def success_sentence(self) -> str:
        out = self.outside(self.counts)
        if not out:
            return (
                f"Every success count of the {RERUN_LABEL} run is inside its {PUB_LABEL} Wilson "
                f"interval."
            )
        parts = []
        for c in out:
            txt = f"{c['say']} ({kn(*c['pub'])} → {kn(*c['re'])})"
            if c["key"].startswith("v02_exec_"):
                pl = c["key"].rsplit("_", 1)[1]
                e = self.re.exec[pl]
                nf = len(e["failed"])
                ab = e["tolerance_aborts"]
                if nf and ab == nf:
                    txt += (
                        f"; all {ab} of its {RERUN_LABEL} failures are path-tolerance aborts "
                        f"(the arm lagged its reference by at least the controller's "
                        f"{PATH_TOLERANCE_RAD} rad path tolerance)"
                    )
                elif nf:
                    txt += f", {ab} of whose {nf} {RERUN_LABEL} failures are path-tolerance aborts"
            parts.append(txt)
        return (
            f"Every success count stays inside its {PUB_LABEL} Wilson interval except "
            f"{join_words(parts)}."
        )

    def timing_sentence(self, readme: bool = False) -> str:
        lo, hi = self.timing_range()
        if math.isnan(lo):
            return "No timing could be compared."
        where = (
            "listed in the [full comparison](docs/results.md#run-to-run-30-oct-vs-31-oct)"
            if readme
            else "in the table above"
        )
        extremes = " (the extremes are the last two rows)" if readme else ""
        return (
            f"Over the {len(self.timings)} timing medians {where}, the changes run from "
            f"{f'{lo:+.0f}'.replace('-', '−')} % to {f'{hi:+.0f}'.replace('-', '−')} %"
            f"{extremes}: timings are one repetition, so they carry no stated variance, and only "
            f"the success counts can be judged against an interval."
        )

    def recheck_sentence(self) -> str:
        out = self.outside(self.rechecks)
        if not out:
            return f"Every dense-check count is also inside its {PUB_LABEL} Wilson interval."
        parts = [f"{c['label']} ({kn(*c['pub'])} → {kn(*c['re'])})" for c in out]
        better = all(c["re"][0] / c["re"][1] > c["pub"][0] / c["pub"][1] for c in out)
        tail = ", all towards fewer misses" if better else ""
        return (
            f"Dense-check counts outside their {PUB_LABEL} Wilson interval{tail}: "
            f"{join_words(parts)}."
        )

    def two_runs_statement(self, files: bool = True) -> str:
        commits = self.re.commits()
        return (
            f"The {RERUN_LABEL} regeneration (commit {', '.join(f'`{c}`' for c in commits)}) was "
            "two runs: `scripts/regenerate_all.sh` stopped at the GIF step, where MoveIt reported "
            "CONTROL_FAILED (its error code for a controller that aborted the motion, here a "
            "path-tolerance abort while replaying a trajectory for the GIF), and the remaining "
            "v0.2 steps of the same script then ran unchanged as a continuation."
            + (
                f" Its files are in `{RERUN}/`; the published tables keep the {PUB_LABEL} files."
                if files
                else ""
            )
        )

    # -- tables ------------------------------------------------------------------------------

    def full_table(self) -> str:
        rows = []
        for c in self.counts + self.rechecks:
            kp, np_ = c["pub"]
            rows.append(
                [
                    c["section"],
                    c["label"],
                    f"{kp}/{np_} ({ci_txt(kp, np_)})",
                    kn(*c["re"]),
                    "inside" if c["inside"] else "**outside**",
                ]
            )
        for t in self.timings:
            rows.append(
                [
                    t["section"],
                    t["label"],
                    f(t["pub"], t["digits"]),
                    f(t["re"], t["digits"]),
                    signed_pct(t["re"], t["pub"]),
                ]
            )
        rows += self.other
        order = {
            "v0.1": 0,
            "v0.1 tuning": 1,
            "v0.2": 2,
            "v0.2 keep-upright": 3,
            "v0.2 octomap": 4,
            "v0.2 agreement": 5,
        }
        rows.sort(key=lambda r: order.get(r[0], 9))
        return table(
            [
                "Set",
                "Metric",
                f"{PUB_LABEL} (published; Wilson 95 % for counts)",
                f"{RERUN_LABEL} (rerun)",
                f"Inside the {PUB_LABEL} interval? / change",
            ],
            rows,
        )

    def short_table(self) -> str:
        by_key = {c["key"]: c for c in self.counts}
        rows = []
        for key, label in (
            ("v01_RRTConnect_none", "Unconstrained queries solved (v0.1, MoveIt)"),
            ("v01_RRTConnect_orientation", "Keep-upright queries solved (v0.1, MoveIt)"),
            ("v02_exec_direct", "Executed to goal, direct (v0.2)"),
            ("v02_exec_moveit", "Executed to goal, MoveIt (v0.2)"),
            ("keep-upright_planned_direct", "Keep-upright planned, direct (v0.2)"),
            ("keep-upright_planned_moveit", "Keep-upright planned, MoveIt (v0.2)"),
            ("agreement", "Same collide / no-collide verdict on random configurations (v0.2)"),
        ):
            c = by_key.get(key)
            if c:
                rows.append(
                    [label, kn(*c["pub"]), kn(*c["re"]), "yes" if c["inside"] else "**no**"]
                )
        t = {x["label"]: x for x in self.timings}
        a = t.get("direct: OMPL solve + simplification median (s)")
        b = t.get("MoveIt: OMPL solve + simplification median (s)")
        if a and b:
            rows.append(
                [
                    "Planning time median, direct / MoveIt (s, v0.2)",
                    f"{f(a['pub'])} / {f(b['pub'])}",
                    f"{f(a['re'])} / {f(b['re'])}",
                    f"no interval (one repetition); {signed_pct(a['re'], a['pub'])} / "
                    f"{signed_pct(b['re'], b['pub'])}",
                ]
            )
        if self.timings:
            ranked = sorted(self.timings, key=lambda x: pct_value(x["re"], x["pub"]))
            for tag, x in (("drop", ranked[0]), ("rise", ranked[-1])):
                rows.append(
                    [
                        f"Biggest {tag} among the timing medians: {x['short']} (s)",
                        f(x["pub"], x["digits"]),
                        f(x["re"], x["digits"]),
                        f"no interval (one repetition); {signed_pct(x['re'], x['pub'])}",
                    ]
                )
        return table(
            ["Measured", PUB_LABEL, RERUN_LABEL, f"Inside the {PUB_LABEL} Wilson interval?"], rows
        )


def run_to_run_section(cmp: Comparison) -> str:
    first, last = cmp.pub.finished_dates()
    return (
        f"## Run-to-run: {PUB_LABEL} vs {RERUN_LABEL}\n\n"
        f'"{PUB_LABEL}" is the published set: the files in `results/` as committed on '
        f"{PUB_LABEL}, measured between {first} and {last} (UTC, from the `.meta.json` files) at "
        f"commits {', '.join(f'`{c}`' for c in cmp.pub.commits())}. "
        f"{cmp.two_runs_statement()}\n\n"
        f"{cmp.full_table()}\n\n"
        f"- {cmp.success_sentence()}\n"
        f"- {cmp.recheck_sentence()}\n"
        f"- {cmp.timing_sentence()}\n"
        "- The tuning rule (best success rate on the 20 tuning queries, ties: lower median "
        "time) is applied here to both tuning sets; `scripts/regenerate_all.sh` regenerates the "
        "tuning CSVs but does not re-apply the rule to the benchmark configuration.\n"
        "- Not regenerated by `scripts/regenerate_all.sh`: the ablation "
        "(`results/ablation/`, an earlier commit) and the first full v0.2 run "
        "(`results/runs/`)."
    )


# --------------------------------------------------------------------------------------------
# README blocks
# --------------------------------------------------------------------------------------------


def settings_matched(pub: Run) -> bool:
    """Planner, range and collision-check spacing equal on both sides (recorded in the meta)."""
    meta = pub.direct_meta
    mo = meta.get("moveit_ompl_context", {})
    try:
        return (
            mo.get("planner", "").endswith(f"[{meta['planner_config']}]")
            and abs(float(mo["range"]) - float(meta["range_rad"])) < 1e-9
            and abs(float(mo["longest_valid_segment_rad"]) - meta["longest_valid_segment_rad"])
            < 1e-12
        )
    except (KeyError, TypeError, ValueError):
        return False


def boundary_disagreements(pub: Run) -> tuple[int, int, list[float]]:
    agr = pub.agreement or {}
    bad = [r for r in pub.boundary if r["agree"] == "0"]
    seps = [float(r["free_side_moveit_min_distance_m"]) for r in bad]
    return len(bad), agr.get("boundary_brackets", 0), seps


def headline_block(pub: Run) -> str:
    none = pub.v01_cell("RRTConnect", "none")
    up = pub.v01_cell("RRTConnect", "orientation")
    solved_none = (
        "every unconstrained query"
        if none["successes"] == none["n"]
        else f"{none['successes']}/{none['n']} unconstrained queries"
    )
    sm = pub.direct
    m = sm["moveit"]
    layers, plan = m["outside_planner_s"]["median"], m["plan_plus_simplify_s"]["median"]
    # Older CSVs and metas do not record the constraint; bench_direct's default is "none".
    cons = {r.get("constraint") for r in pub.direct_rows if r.get("constraint")}
    kind = CONSTRAINT_WORD[
        pub.direct_meta.get("constraint") or (cons.pop() if len(cons) == 1 else "none")
    ]
    ratio = sm["paired"]["median_ratio_direct_over_moveit"]
    pm = pub.probe_md
    costlier = bool(pm) and pm["direct_check_time_us_median"] > pm["moveit_check_time_us_median"]
    pairs = pub.direct_meta.get("direct_active_pairs")
    speed = f"{about(ratio)} slower" if ratio >= 1.5 else "slower" if ratio > 1 else "faster"
    if layers < plan / 2:
        share = "less than half"
    elif layers > plan / 2:
        share = "more than half"
    else:
        share = "half"
    setup = (
        ", because each of its collision checks costs more (measured); why is a hypothesis: it "
        "uses coal instead of FCL (MoveIt's collision library) and tests all "
        f"{pairs} pairs of arm parts without a broadphase (a quick pre-filter of far-apart "
        "pairs). So its time is not a measure of MoveIt's layers "
        "([Where it fails](#where-it-fails))."
        if costlier and pairs and ratio > 1
        else "."
    )
    bullets = [
        f"- On the camera's octomap, MoveIt solved {solved_none} and "
        f"{up['successes']}/{up['n']} keep-upright ones within 10 s.",
        "- MoveIt's own layers, timed inside MoveIt (everything it does per request outside the "
        "planning library, including its adapters and the time parameterisation that gives "
        "each point of the path its time), took "
        f"{f(layers)} s against {f(plan)} s of planning, {share} the planning time (medians over "
        f"the {sm['queries']} {kind} queries, with the shelf given as boxes).",
        "- The direct rebuild shows the same planning can be done without MoveIt, "
        + ("but" if ratio > 1 else "and")
        + f" it was {speed} (median of per-query ratios){setup}",
    ]
    agr = pub.agreement or {}
    m_bad, k_brackets, seps = boundary_disagreements(pub)
    if agr:
        random_part = (
            f"on all {agr['count']} random configurations they agreed"
            if agr["agree"] == agr["count"]
            else f"on {agr['agree']} of {agr['count']} random configurations they agreed"
        )
        if m_bad and seps and max(seps) < GJK_TOLERANCE_M:
            near = (
                "near contact, where configurations were moved step by step to the point where "
                "two parts just touch, they gave the same collide / no-collide verdict in "
                f"{k_brackets - m_bad} of {k_brackets} cases; in the other {m_bad} one reports "
                "contact and the other a gap of a few nanometres, below the 1e-6 m tolerance coal "
                "computes to"
            )
        elif m_bad:
            near = (
                "near contact, where configurations were moved step by step to the point where "
                f"two parts just touch, they disagreed in {m_bad} of {k_brackets} cases"
            )
        else:
            near = (
                "near contact, where configurations were moved step by step to the point where "
                f"two parts just touch, they agreed in all {k_brackets} cases"
            )
        bullets.append(f"- The two collision checkers: {near}; {random_part}.")
    return "\n".join(bullets)


def breath_block(pub: Run) -> str:
    none = pub.v01_cell("RRTConnect", "none")
    up = pub.v01_cell("RRTConnect", "orientation")
    d, m = pub.direct["direct"], pub.direct["moveit"]
    paired = pub.direct["paired"]
    ratio = paired["median_ratio_direct_over_moveit"]
    pm = pub.probe_md
    probe = (
        f" The per-check cost is where the two differ: a collision check took "
        f"{f(pm['direct_check_time_us_median'], 1)} µs inside the direct planner and "
        f"{f(pm['moveit_check_time_us_median'], 1)} µs inside MoveIt's (medians from a "
        f"separate probe run that counts every collision check and times each one, on "
        f"{pm['n']} queries); why is not yet measured."
        if pm
        else ""
    )
    upr = pub.upright
    upright = (
        f" With the keep-upright constraint on the same boxes, the direct pipeline planned "
        f"{upr['direct']['planned']}/{upr['direct']['attempted']} queries and MoveIt "
        f"{upr['moveit']['planned']}/{upr['moveit']['attempted']} (MoveIt's "
        f"{up['successes']}/{up['n']} above was on the octomap)."
        if upr
        else ""
    )
    tilt = (
        f" Along every solved keep-upright path, checked at every {DENSE_STEP_RAD} rad joint "
        f"step, the tool stayed within {f(up['tilt_max_deg'], 1)}° of vertical on the octomap"
        + (
            ", and within "
            + " and ".join(
                f"{f(v, 1)}° ({SHORT[pl]})"
                for pl, v in (
                    (
                        pl,
                        max(
                            (
                                float(r["max_tilt_deg"])
                                for r in pub.upright_rows
                                if r["pipeline"] == pl
                                and r["plan_success"] == "1"
                                and r.get("max_tilt_deg")
                            ),
                            default=math.nan,
                        ),
                    )
                    for pl in PIPELINES
                )
            )
            + " on the boxes"
            if upr
            else ""
        )
        + " (the limit is 5°)."
        if not math.isnan(up["tilt_max_deg"])
        else ""
    )
    agr = pub.agreement
    m_bad, k_brackets, seps = boundary_disagreements(pub)
    agreement = (
        (
            " The last question was whether the two collision checkers (MoveIt's FCL and coal) "
            "agree: they gave the same collide / no-collide verdict on "
            + (
                f"all {agr['count']}"
                if agr["agree"] == agr["count"]
                else f"{agr['agree']} of {agr['count']}"
            )
            + " random arm configurations"
            + (
                f". Taking the random configurations as a sequence, wherever two neighbours fall "
                f"on opposite sides (one colliding, one not), the step between them was halved "
                f"until both ends were within "
                f"nanometres of the point where two parts touch: in {m_bad} of those "
                f"{k_brackets} cases one checker reports contact and the other a gap of at most "
                f"{max(seps) * 1e9:.0f} nm, below the 1e-6 m tolerance coal computes to."
                if seps
                else "."
            )
        )
        if agr
        else ""
    )
    return (
        f"v0.1 asked whether MoveIt can plan on a map built only from the camera: on the octomap "
        f"it solved {none['successes']}/{none['n']} queries without a constraint and "
        f"{up['successes']}/{up['n']} with the keep-upright constraint, each within 10 s. v0.2 "
        f"asked what MoveIt's layers cost, on the shelf given as seven boxes. Timed inside MoveIt, "
        f"everything it does per request outside the planning library took "
        f"{f(m['outside_planner_s']['median'])} s (median). Planning the same queries without "
        f"MoveIt, the direct pipeline's planning time (OMPL's search plus "
        f"simplification) was {f(d['plan_plus_simplify_s']['median'])} s against MoveIt's "
        f"{f(m['plan_plus_simplify_s']['median'])} s (medians; paired, {about(ratio)} over the "
        f"{paired['both_planned']} queries both planned), so it was "
        f"{'slower' if ratio > 1 else 'faster'}.{probe}{upright}{tilt}{agreement} Two caveats: "
        f"every number is one repetition, and a second full run on 31 Oct moved some of them (see "
        f"[Reproducibility](#reproducibility))."
    )


def results_block(pub: Run) -> str:
    none = pub.v01_cell("RRTConnect", "none")
    up = pub.v01_cell("RRTConnect", "orientation")
    d, m = pub.direct["direct"], pub.direct["moveit"]
    paired = pub.direct["paired"]
    ex = pub.exec
    agr = pub.agreement
    octo = pub.octomap
    bad = [r for r in pub.boundary if r["agree"] == "0"]
    seps = [float(r["free_side_moveit_min_distance_m"]) for r in bad]
    fails_up = up["n"] - up["successes"]
    rows = [
        [
            "Unconstrained queries solved (MoveIt, octomap)",
            f"{none['successes']}/{none['n']} (Wilson interval "
            f"{ci_txt(none['successes'], none['n'])})",
            (
                "the planner found a path on the camera map for every query"
                if none["successes"] == none["n"]
                else f"{none['n'] - none['successes']} queries found no path within 10 s"
            )
            + (
                f"; the dense check later found a contact on "
                f"{none['dense_checked'] - none['dense_ok']}/{none['dense_checked']} of those "
                f"paths (see Where it fails)"
                if none["dense_checked"]
                else ""
            ),
        ],
        [
            "Keep-upright queries solved (MoveIt, octomap)",
            f"{up['successes']}/{up['n']} (Wilson interval {ci_txt(up['successes'], up['n'])})",
            f"the constraint works, at a cost: {fails_up} queries found no upright path within "
            "10 s",
        ],
        [
            "Planning time, median (boxes)",
            f"direct {f(d['plan_plus_simplify_s']['median'])} s, MoveIt "
            f"{f(m['plan_plus_simplify_s']['median'])} s; paired ratio "
            f"{f(paired['median_ratio_direct_over_moveit'], 2)}×",
            (
                f"without MoveIt it is slower, not faster (direct faster on "
                f"{paired['direct_faster']}/{paired['both_planned']} queries)"
            )
            if paired["median_ratio_direct_over_moveit"] > 1
            else "without MoveIt it is faster",
        ],
    ]
    if d["executed_mode"] and m["executed_mode"]:
        mf = ex["moveit"]
        df = ex["direct"]
        what = []
        for pl, e in (("direct", df), ("moveit", mf)):
            nf = len(e["failed"])
            if nf:
                what.append(
                    f"{SHORT[pl]}'s {nf} failure{'s' if nf != 1 else ''}: "
                    + (
                        "path-tolerance abort" + ("s" if nf != 1 else "")
                        if e["tolerance_aborts"] == nf
                        else f"{e['tolerance_aborts']} path-tolerance aborts"
                    )
                )
        rows.append(
            [
                "Executed to goal (same controller)",
                f"direct {d['exec_success']}/{d['attempted']}, MoveIt "
                f"{m['exec_success']}/{m['attempted']}",
                "the plans run on the simulated controller"
                + (f"; {'; '.join(what)}" if what else "")
                + " (reaching the goal says nothing about contact on the way)",
            ]
        )
    if agr:
        rows.append(
            [
                "Same collide / no-collide verdict (FCL vs coal)",
                f"{agr['agree']}/{agr['count']} random configurations; in cases narrowed to "
                f"within nanometres of contact, "
                + (
                    f"{len(bad)}/{agr['boundary_brackets']} disagree, by "
                    f"{min(seps) * 1e9:.0f}–{max(seps) * 1e9:.0f} nm"
                    if seps
                    else f"all {agr['boundary_brackets']} agree"
                ),
                "the two checkers differ only below the 1e-6 m tolerance coal computes to"
                if agr["agree"] == agr["count"] and (not seps or max(seps) < GJK_TOLERANCE_M)
                else "the two checkers disagree beyond their precision",
            ]
        )
    if octo:
        r = octo["paired"]["median_ratio_direct_over_moveit"]
        rows.append(
            [
                "Planning time on the camera's octomap, median",
                f"direct {f(octo['direct']['plan_plus_simplify_s']['median'])} s, MoveIt "
                f"{f(octo['moveit']['plan_plus_simplify_s']['median'])} s; paired ratio "
                f"{f(r, 2)}×",
                (
                    "the gap is smaller on the octomap than on the boxes; why is not measured"
                    if r < paired["median_ratio_direct_over_moveit"]
                    else "the gap is not smaller on the octomap"
                )
                + ". The boxes row above is the main comparison of the two pipelines",
            ]
        )
    commits = ", ".join(f"`{c}`" for c in pub.commits())
    caption_side = (
        "right"
        if d["plan_plus_simplify_s"]["median"] > m["plan_plus_simplify_s"]["median"]
        else "left"
    )
    return (
        table(["Measured", "Result", "What it means"], rows)
        + "\n\n"
        + f"Sources: `results/bench.csv`, `results/bench_direct.csv`, "
        f"`results/bench_direct_octomap.csv`, `results/collision_agreement.csv` "
        f"(commits {commits}, "
        f'one repetition, 10 s per plan). "Boxes": the shelf given to both pipelines as seven '
        f'boxes; "paired ratio": the median, over the queries both planned, of direct time '
        f"divided by MoveIt time.\n\n"
        f"![Each of the {paired['both_planned']} queries as one dot per pipeline: time to find "
        f"and simplify a path, linear axis](docs/assets/direct_vs_moveit_per_query.png)\n\n"
        f"*Each dot is one query: the time OMPL took to find and simplify a path. Look at where "
        f"each row's dots cluster: the direct row sits to the {caption_side} of MoveIt's; the "
        f"vertical bar in each row is its median. The same data on log axes, and every other "
        f"table, are in [docs/results.md](docs/results.md).*"
    )


def reproducibility_block(cmp: Comparison) -> str:
    return (
        f"{cmp.short_table()}\n\n"
        f"{cmp.success_sentence()} {cmp.timing_sentence(readme=True)}\n\n"
        f"{cmp.two_runs_statement(files=False)}"
    )


def f16_block(pub: Run) -> str:
    """Numbers for F16 (the speed gap), placed under that item."""
    m = pub.direct["moveit"]
    paired = pub.direct["paired"]
    pm = pub.probe_md
    txt = (
        f"Measured: over the {paired['both_planned']} queries both pipelines planned, the direct "
        f"search plus simplification took {f(paired['median_ratio_direct_over_moveit'], 2)}× "
        f"MoveIt's time (paired, median of the per-query ratios); the direct pipeline was faster "
        f"on {paired['direct_faster']}/{paired['both_planned']}."
    )
    if pm:
        txt += (
            f" Probe, medians over {pm['n']} queries: {f(pm['direct_validity_checks'], 0)} vs "
            f"{f(pm['moveit_validity_checks'], 0)} collision checks per request, "
            f"{f(pm['direct_check_time_us_median'], 1)} vs "
            f"{f(pm['moveit_check_time_us_median'], 1)} µs per check."
        )
    txt += (
        f" MoveIt's layers outside the planner add {f(m['outside_planner_s']['median'])} s per "
        f"request (median), against {f(m['plan_plus_simplify_s']['median'])} s of search plus "
        f"simplification."
    )
    if pub.octomap:
        txt += (
            f" On the octomap the paired ratio is "
            f"{f(pub.octomap['paired']['median_ratio_direct_over_moveit'], 2)}×."
        )
    return txt


def f17_block(pub: Run) -> str:
    """Numbers for F17 (keep-upright in the direct pipeline), placed under that item."""
    u = pub.upright
    if not u:
        return "Keep-upright comparison not run."
    share = checker_share(pub.upright_rows, True)
    return (
        f"Measured: planned within 10 s, direct {u['direct']['planned']}/"
        f"{u['direct']['attempted']} vs MoveIt {u['moveit']['planned']}/"
        f"{u['moveit']['attempted']}. Collision checking took an estimated "
        f"{100 * share:.1f} % of the direct planner's search plus simplification time (median "
        f"over its solved queries; checks × median check time, since the total is not recorded). "
        f"Waypoints per path, median: {f(u['direct']['n_waypoints']['median'], 1)} direct vs "
        f"{f(u['moveit']['n_waypoints']['median'], 1)} MoveIt."
    )


def f26_block(pub: Run) -> str:
    """Numbers for F26 (tracking error vs the dense-check step), placed under that item."""
    ex = pub.exec
    if not ex["moveit"]["executed"]:
        return "No executed runs."
    dt, mt = ex["direct"]["track"], ex["moveit"]["track"]
    return (
        f"Measured: the 95th percentile (p95, the value 95 % of executions stay below) of each "
        f"execution's largest tracking error is {f(dt['p95'], 4)} rad for the direct pipeline and "
        f"{f(mt['p95'], 4)} rad for MoveIt, {dt['p95'] / DENSE_STEP_RAD:.1f}× and "
        f"{mt['p95'] / DENSE_STEP_RAD:.1f}× the {DENSE_STEP_RAD} rad dense-check step; "
        f"{ex['direct']['track_above_dense_step']}/{ex['direct']['executed']} direct and "
        f"{ex['moveit']['track_above_dense_step']}/{ex['moveit']['executed']} MoveIt executions "
        f"exceeded that step. {tool_bound_sentence(dt, mt)}"
    )


def f27_block(pub: Run) -> str:
    """Numbers for F27 (keep-upright per query class), placed under that item."""
    if not pub.upright:
        return "Keep-upright comparison not run."
    cls = by_class(pub.upright_rows)
    ov = overlap(pub.upright_rows)
    gap = largest_gap_classes(cls)
    rows = [[c, kn(*v["direct"]), kn(*v["moveit"])] for c, v in cls.items()]
    return (
        table(["Keep-upright planned, by query class", "Direct", "MoveIt"], rows)
        + f"\n\nThe direct planner fails {ov['direct_only_failed']} queries MoveIt solves and "
        f"solves {ov['moveit_only_failed']} that MoveIt fails; both fail {ov['both_failed']}"
        + (f". Its deficit is largest {join_words(gap)}." if gap else ".")
    )


def failure_numbers_block(pub: Run, re: Run | None) -> str:
    rows: list[list[str]] = []
    # F2, F3: BiTRRT and the RRTConnect range (v0.1 + tuning sets).
    bn, bo = pub.v01_cell("BiTRRT", "none"), pub.v01_cell("BiTRRT", "orientation")
    up = pub.v01_cell("RRTConnect", "orientation")
    if bn and bo:
        txt = (
            f"BiTRRT (range 2 rad) solved {bn['successes']}/{bn['n']} unconstrained and "
            f"{bo['successes']}/{bo['n']} keep-upright queries; RRTConnect "
            f"{up['successes']}/{up['n']} keep-upright."
        )
        if "tuning_bitrrt" in pub.tuning:
            a = pub.tuning["tuning_bitrrt"]["choice"]
            b = re.tuning["tuning_bitrrt"]["choice"] if re and "tuning_bitrrt" in re.tuning else a
            txt += (
                " The tuning rule (best success rate on 20 separate tuning queries) picks "
                f"BiTRRT's "
                f"{range_name(a)} on the published tuning set"
                + (
                    f" and {range_name(b)} on its {RERUN_LABEL} rerun."
                    if b != a
                    else f", and the same on its {RERUN_LABEL} rerun."
                )
            )
        rows.append(["F2", txt])
    if "tuning" in pub.tuning:

        def cells(run: Run) -> str:
            return ", ".join(
                f"{range_name(s['planner'])} {s['successes']}/{s['n']}"
                for s in sorted(
                    run.tuning["tuning"]["cells"], key=lambda s: range_key(s["planner"])
                )
            )

        txt = f"Keep-upright queries of the 20-query tuning set solved by RRTConnect: {cells(pub)}."
        if re and "tuning" in re.tuning:
            txt += f" {RERUN_LABEL} rerun: {cells(re)}."
        rows.append(["F3", txt])
    # F4: v0.1 dense-check misses.
    parts = []
    for s in pub.v01:
        if s["dense_checked"]:
            parts.append(
                f"{s['planner'].split(' (')[0]} "
                f"{CONSTRAINT_WORD.get(s['constraint'], s['constraint'])}: "
                f"{s['dense_checked'] - s['dense_ok']}/{s['dense_checked']}"
            )
    if parts:
        rows.append(
            [
                "F4",
                "Solved v0.1 paths with a contact found by the dense check: "
                + ", ".join(parts)
                + ".",
            ]
        )
    # F10: octomap resolution sweep.
    if {"0.02", "0.05"} <= set(pub.sweep):

        def invalid(rows_: list[dict]) -> tuple[int, int]:
            qs = {r["query_id"] for r in rows_}
            bad = {
                r["query_id"] for r in rows_ if r["start_valid"] != "1" or r["goal_valid"] != "1"
            }
            return len(bad), len(qs)

        def med_none(rows_: list[dict]) -> float:
            t = [
                float(r["planning_time_s"])
                for r in rows_
                if r["constraint"] == "none" and r["success"] == "1"
            ]
            return statistics.median(t) if t else math.nan

        i2, n2 = invalid(pub.sweep["0.02"])
        i5, n5 = invalid(pub.sweep["0.05"])
        rows.append(
            [
                "F10",
                f"Queries with a start or goal in collision: {i2}/{n2} with 2 cm cubes, {i5}/{n5} "
                f"with 5 cm. Unconstrained median planning time: "
                f"{f(med_none(pub.sweep['0.02']))} s (2 cm) vs {f(med_none(pub.sweep['0.05']))} s "
                f"(5 cm), over the queries solved.",
            ]
        )
    # F16: the speed gap.
    d, m = pub.direct["direct"], pub.direct["moveit"]
    paired = pub.direct["paired"]
    pm = pub.probe_md
    txt = (
        f"Paired over the {paired['both_planned']} queries both planned, the direct planning time "
        f"(OMPL search + path simplification) is "
        f"{f(paired['median_ratio_direct_over_moveit'], 2)}× "
        f"MoveIt's (median of per-query ratios; direct faster on {paired['direct_faster']})."
    )
    if pm:
        txt += (
            f" Probe, {pm['n']} queries, medians: {f(pm['direct_validity_checks'], 0)} vs "
            f"{f(pm['moveit_validity_checks'], 0)} collision checks per request and "
            f"{f(pm['direct_check_time_us_median'], 1)} vs "
            f"{f(pm['moveit_check_time_us_median'], 1)}"
            f" µs per check (MoveIt's count includes cached verdicts)."
        )
    txt += (
        " MoveIt's own layers (its time per request outside the planning library): "
        f"{f(m['outside_planner_s']['median'])} s median, "
        f"{f(m['outside_planner_s']['p95'])} s 95th percentile; the direct pipeline's "
        f"equivalent: {f(d['outside_planner_s']['median'], 4)} s."
    )
    if pub.octomap:
        txt += (
            f" On the camera's octomap the paired ratio is "
            f"{f(pub.octomap['paired']['median_ratio_direct_over_moveit'], 2)}×."
        )
    rows.append(["F16", txt])
    # F17: keep-upright in both pipelines.
    if pub.upright:
        u = pub.upright
        tilt = {
            pl: max(
                (
                    float(r["max_tilt_deg"])
                    for r in pub.upright_rows
                    if r["pipeline"] == pl and r["plan_success"] == "1" and r.get("max_tilt_deg")
                ),
                default=math.nan,
            )
            for pl in PIPELINES
        }
        rows.append(
            [
                "F17",
                f"Keep-upright planned within 10 s: direct {u['direct']['planned']}/"
                f"{u['direct']['attempted']}, MoveIt {u['moveit']['planned']}/"
                f"{u['moveit']['attempted']}; planning time median "
                f"{f(u['direct']['plan_plus_simplify_s']['median'], 2)} s vs "
                f"{f(u['moveit']['plan_plus_simplify_s']['median'], 2)} s (each over its own "
                "solved "
                f"queries); worst tool tilt {f(tilt['direct'], 1)}° vs {f(tilt['moveit'], 1)}°. "
                f"Estimated share of the direct planning time spent in the collision checker: "
                f"{100 * checker_share(pub.upright_rows, True):.1f} % (median over solved "
                "queries). "
                f"Path waypoints, median: {f(u['direct']['n_waypoints']['median'], 1)} direct vs "
                f"{f(u['moveit']['n_waypoints']['median'], 1)} MoveIt.",
            ]
        )
    # F18: simplification dominates.
    rows.append(
        [
            "F18",
            f"Simplification took longer than the OMPL search on {d.get('simplify_dominates', 0)}/"
            f"{d['planned']} direct plans.",
        ]
    )
    # F19: agreement at the boundary.
    agr = pub.agreement
    if agr:
        bad = [r for r in pub.boundary if r["agree"] == "0"]
        seps = [float(r["free_side_moveit_min_distance_m"]) for r in bad]
        pairs = sorted({r["colliding_side_moveit_pairs"].replace("|", "/") for r in bad})
        rows.append(
            [
                "F19",
                f"{agr['agree']}/{agr['count']} random configurations get the same verdict. Where "
                f"consecutive samples switch between collide and no-collide, the step between "
                f"them was halved until both ends were within nanometres of contact: "
                f"{agr['boundary_agree']}/{agr['boundary_brackets']} of those cases agree"
                + (
                    f"; in the other {len(bad)} coal reports contact where FCL measures "
                    f"{min(seps) * 1e9:.0f}–{max(seps) * 1e9:.0f} nm of separation, all on "
                    f"{join_words(pairs)}."
                    if bad
                    else "."
                ),
            ]
        )
    # F20: dense re-checks.
    rows.append(
        [
            "F20",
            f"Paths that fail the other pipeline's dense check at 0.01 rad: MoveIt "
            f"{m['planned'] - m['crossval_ok']}/{m['planned']}, direct "
            f"{d['planned'] - d['crossval_ok']}/{d['planned']}; "
            + (
                "on every path the other checker agrees with the path's own checker."
                if d["crossval_vs_own_disagreements"] == m["crossval_vs_own_disagreements"] == 0
                else f"the other checker disagrees with the path's own checker on "
                f"{d['crossval_vs_own_disagreements'] + m['crossval_vs_own_disagreements']} paths."
            ),
        ]
    )
    # F21: execution failures, repositioning.
    ex = pub.exec
    if ex["moveit"]["executed"]:
        parts = []
        for pl in PIPELINES:
            e = ex[pl]
            nf = len(e["failed"])
            if nf == 0:
                parts.append(f"{SHORT[pl]} 0")
            elif nf == 1 and e["tolerance_aborts"] == 1:
                clean = e["tolerance_aborts_clean_both"] == 1
                parts.append(
                    f"{SHORT[pl]} 1, a path-tolerance abort (tracking error at the "
                    f"{PATH_TOLERANCE_RAD} rad path tolerance) on a path "
                    + ("clean" if clean else "not clean")
                    + " on both dense checks"
                )
            else:
                which = "all" if e["tolerance_aborts"] == nf else f"{e['tolerance_aborts']} of them"
                parts.append(
                    f"{SHORT[pl]} {nf}, {which} path-tolerance aborts (tracking error at the "
                    f"{PATH_TOLERANCE_RAD} rad path tolerance), "
                    f"{e['tolerance_aborts_clean_both']} of those on a path clean on both dense "
                    "checks"
                )
        meta = pub.direct_meta
        txt = (
            "Executions that did not reach the goal: "
            + "; ".join(parts)
            + f". Repositioning moves (one before each execution, both pipelines): "
            f"{meta.get('repositions_attempted', 0)} attempted, "
            + (
                "every one ended at its start (an attempt that aborted was retried and is not "
                "counted as a failure)"
                if not meta.get("reposition_failures", 0)
                else f"{meta.get('reposition_failures', 0)} still off their start after a retry "
                "(an attempt that aborted was retried and is not counted as a failure)"
            )
            + f"; the harness freed the arm from contact {meta.get('escapes', 0)} times."
        )
        runs1 = REPO / "results/runs/bench_direct_run1_no_recovery.csv"
        cascade = first_run_cascade(load(runs1)) if runs1.exists() else None
        if cascade:
            c1 = load_meta(runs1).get("commit", "?")
            txt += (
                f" In an earlier full run of the same benchmark (commit `{c1}`, before the harness "
                f"could free the arm; kept in `results/runs/`, not one of the published runs), all "
                f"{cascade['later_runs']} runs after the first failed execution failed."
            )
        rows.append(["F21", txt])
        # F22, F26: tracking error.
        dt, mt = ex["direct"]["track"], ex["moveit"]["track"]
        mg = ex["moveit"]["track_reached_goal"]
        rows.append(
            [
                "F22",
                f"Max joint tracking error, median / 95th percentile over all executions: direct "
                f"{f(dt['median'], 4)} / {f(dt['p95'], 4)} rad, MoveIt {f(mt['median'], 4)} / "
                f"{f(mt['p95'], 4)} rad; worst MoveIt execution that reached the goal "
                f"{f(mg['max'], 4)} rad.",
            ]
        )
        rows.append(
            [
                "F26",
                f"95th percentile (p95) of the max tracking error: direct {f(dt['p95'], 4)} rad, "
                f"MoveIt "
                f"{f(mt['p95'], 4)} rad, i.e. {dt['p95'] / DENSE_STEP_RAD:.1f}× and "
                f"{mt['p95'] / DENSE_STEP_RAD:.1f}× the {DENSE_STEP_RAD} rad dense-check step; "
                f"executions above that step: direct {ex['direct']['track_above_dense_step']}/"
                f"{ex['direct']['executed']}, MoveIt {ex['moveit']['track_above_dense_step']}/"
                f"{ex['moveit']['executed']}. {tool_bound_sentence(dt, mt)}",
            ]
        )
    # F27: per-class keep-upright counts.
    if pub.upright:
        cls = by_class(pub.upright_rows)
        ov = overlap(pub.upright_rows)
        gap = largest_gap_classes(cls)
        rows.append(
            [
                "F27",
                "Keep-upright planned, direct vs MoveIt, by query class (how start and goal "
                "compartments relate): "
                + "; ".join(f"{c} {kn(*v['direct'])} vs {kn(*v['moveit'])}" for c, v in cls.items())
                + f". The direct planner fails {ov['direct_only_failed']} queries MoveIt solves "
                f"and solves {ov['moveit_only_failed']} MoveIt fails"
                + (f"; its deficit is largest {join_words(gap)}." if gap else "."),
            ]
        )
    return (
        table(["Item", "Measured (published run)"], rows)
        + "\n\nSources: the CSVs under `results/`; per-run details in "
        "[docs/results.md](docs/results.md)."
    )


# --------------------------------------------------------------------------------------------
# Plots
# --------------------------------------------------------------------------------------------


def _style():
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
    return plt


def make_plot(rows: list[dict], timeout: float, out: pathlib.Path) -> pathlib.Path:
    """Per-query scatter on log axes (docs/results.md)."""
    plt = _style()
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
        (lo * 2.2, lo * 2.2),
        xytext=(6, -10),
        textcoords="offset points",
        rotation=45,
        color=_agg.TEXT_2,
        ha="left",
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
        f"Direct slower than MoveIt on {above} of {len(both)} queries",
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


def make_plain_plot(rows: list[dict], out: pathlib.Path) -> pathlib.Path:
    """README chart: one row of dots per pipeline on a linear seconds axis, median marked."""
    plt = _style()
    times = {
        pl: [
            v
            for r in rows
            if r["pipeline"] == pl
            and r["plan_success"] == "1"
            and (v := plan_plus_simplify(r)) is not None
        ]
        for pl in PIPELINES
    }
    fig, ax = plt.subplots(figsize=(7.2, 2.9), dpi=150)
    order = ("moveit", "direct")  # MoveIt on the bottom row, the direct pipeline above it
    names = {"moveit": "MoveIt 2", "direct": "Direct (no MoveIt)"}
    for i, pl in enumerate(order):
        ts = times[pl]
        jitter = [((k * 0.6180339887) % 1.0 - 0.5) * 0.36 for k in range(len(ts))]
        ax.scatter(
            ts,
            [i + j for j in jitter],
            s=20,
            color=COLOR[pl],
            edgecolors=_agg.SURFACE,
            linewidths=0.6,
            zorder=3,
        )
        if ts:
            med = statistics.median(ts)
            ax.plot([med, med], [i - 0.3, i + 0.3], color=_agg.TEXT, linewidth=2, zorder=4)
            ax.annotate(
                f"median {med:.2f} s",
                (med, i + 0.3),
                xytext=(4, 2),
                textcoords="offset points",
                color=_agg.TEXT,
                fontsize=9,
                va="bottom",
            )
    ax.set_yticks(range(len(order)), [names[p] for p in order])
    ax.set_ylim(-0.6, len(order) - 0.25)
    hi = max((max(v) for v in times.values() if v), default=1.0)
    ax.set_xlim(0, hi * 1.05)
    ax.grid(axis="y", visible=False)
    ax.set_xlabel("time to find and simplify a path (s), one dot per query")
    md = {pl: statistics.median(v) if v else math.nan for pl, v in times.items()}
    verdict = "takes longer" if md["direct"] > md["moveit"] else "is faster"
    ax.set_title(
        f"Same {min(len(v) for v in times.values())} queries: without MoveIt, planning {verdict}",
        loc="left",
        color=_agg.TEXT,
        fontsize=10,
    )
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)
    return out


# --------------------------------------------------------------------------------------------
# Markers and files
# --------------------------------------------------------------------------------------------


def markers(name: str) -> tuple[str, str]:
    return f"<!-- {name}:start -->", f"<!-- {name}:end -->"


def require_pair(text: str, start: str, end: str, where: str) -> None:
    a, b = text.count(start), text.count(end)
    if a != 1 or b != 1 or text.index(start) > text.index(end):
        raise SystemExit(
            f"{where}: the marker pair {start} ... {end} must appear exactly once, start before "
            f"end (found {a} start, {b} end). Place it once; see the README block list in "
            f"scripts/aggregate_direct.py."
        )


def require_absent(text: str, name: str, where: str) -> None:
    start, end = markers(name)
    if start in text or end in text:
        raise SystemExit(
            f"{where}: the retired block {start} ... {end} is still present; remove it (its "
            f"tables are in docs/results.md now)."
        )


def replace_block(text: str, start: str, end: str, body: str) -> str:
    """Replace what lies between the markers. A start marker indented under a list item keeps the
    block inside the item: every body line and the end marker get the same indentation."""
    i = text.index(start)
    line = text.rfind("\n", 0, i) + 1
    indent = text[line:i] if not text[line:i].strip() else ""
    j = text.index(end)
    j_line = text.rfind("\n", 0, j) + 1
    j = j_line if not text[j_line:j].strip() else j
    body = "\n".join(indent + ln if ln else ln for ln in body.split("\n"))
    tail = text[j:] if text[j:].startswith(indent + end) else indent + text[j:].lstrip(" ")
    return text[: i + len(start)] + "\n" + body + "\n" + tail


def replace_or_append(text: str, start: str, end: str, body: str) -> str:
    block = f"{start}\n{body}\n{end}"
    if start in text and end in text:
        return text[: text.index(start)] + block + text[text.index(end) + len(end) :]
    return text.rstrip("\n") + "\n\n" + block + "\n"


def readme_text(old: str, blocks: dict[str, str], where: str) -> str:
    for name in RETIRED_README_BLOCKS:
        require_absent(old, name, where)
    text = old
    optional = [n for n in OPTIONAL_README_BLOCKS if markers(n)[0] in text or markers(n)[1] in text]
    if not optional:
        raise SystemExit(
            f"{where}: no block for the failure items' numbers; place "
            + " or ".join(markers(n)[0] for n in OPTIONAL_README_BLOCKS)
            + " (with its end marker)."
        )
    for name in list(README_BLOCKS) + optional:
        start, end = markers(name)
        require_pair(text, start, end, where)
        text = replace_block(
            text,
            start,
            end,
            f"<!-- generated by scripts/aggregate_direct.py -->\n\n{blocks[name]}\n",
        )
    return text


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
        "| Check-count probe, 20 queries | `scripts/check_probe.sh 20` |",
        "| Keep-upright, both pipelines, 100 queries, plan only "
        "| `PCP_CONSTRAINT=orientation scripts/bench_direct.sh 100 "
        "results/bench_direct_upright.csv --plan-only` |",
        "| Octomap scene, both pipelines, 100 queries, plan only "
        "| `scripts/bench_direct.sh 100 results/bench_direct_octomap.csv --octomap` |",
        "| Run-to-run comparison | `scripts/regenerate_all.sh` into a fresh clone, copy its "
        "CSV/JSON files into `results/rerun_<date>/`, then `scripts/aggregate_direct.py` |",
        "| Check that docs/results.md and every README block are current "
        "| `scripts/aggregate.py --check && scripts/aggregate_direct.py --check` |",
    ]
)


def direct_body(pub: Run, args_csv: str, cmp: Comparison | None) -> str:
    sm = pub.direct
    table_ = main_table(sm, pub.agreement, pub.probe)
    source = direct_source(pub, args_csv)
    d = sm["direct"]
    m = sm["moveit"]
    paired = sm["paired"]
    upright = "Not run."
    if pub.upright:
        umeta = dict(pub.upright_meta, csv="results/bench_direct_upright.csv")
        upright = upright_table(pub, umeta) + "\n\n" + upright_details(pub)
    octo = "Not run."
    if pub.octomap:
        ometa = dict(pub.octomap_meta, csv="results/bench_direct_octomap.csv")
        octo = octomap_table(pub, ometa)
    executed = d["executed_mode"] or m["executed_mode"]
    body = f"""<!-- GENERATED by scripts/aggregate_direct.py; do not edit by hand. -->

## v0.2: direct OMPL + coal pipeline vs MoveIt 2 (same queries, boxes, settings, controller)

{table_}

{source}

- Paired, on the {paired["both_planned"]} queries both planned: the direct pipeline's solve +
  simplify time is {f(paired["median_ratio_direct_over_moveit"], 2)}× MoveIt's
  `planning_time` (median of per-query ratios); it is faster on {paired["direct_faster"]}.
- Simplification took longer than the OMPL solve on {d.get("simplify_dominates", 0)} of
  {d["planned"]} direct plans.
- Time from request to trajectory outside OMPL solve + simplify: MoveIt
  {f(m["outside_planner_s"]["median"])} s median, {f(m["outside_planner_s"]["p95"])} s p95
  (`plan()` wall time minus `planning_time`: planning context set-up, adapters, TOTG); direct
  {f(d["outside_planner_s"]["median"], 4)} s median (path copy, statistics and the trapezoid,
  not the trapezoid alone, which is the time parameterisation row).
- Skipped queries (start or goal rejected by either checker): {len(sm["skipped_queries"])}.

![Per-query planning time, direct vs MoveIt, log axes](assets/direct_vs_moveit_planning_time.png)

![Per-query planning time, direct vs MoveIt, linear axis](assets/direct_vs_moveit_per_query.png)

### Where the planning time goes (probe)

MoveIt's `planning_time` hides its internals, so a separate probe solves the first queries again
in the OMPL planning context MoveIt builds for `plan()`, with MoveIt's own validity checker
subclassed to count and time every call, and with the direct planner
(`scripts/check_probe.sh 20`, one query per process; the pipeline's adapters are bypassed).

{probe_table(pub.probe, pub.probe_commit)}

### Keep-upright in both pipelines

The direct planner gets v0.1's constraint as an `ompl::base::Constraint` (the rotation vector of
R_tool^T R_target with per-axis dead bands, co-dimension 3, as MoveIt formulates it) on OMPL's
`ProjectedStateSpace`; MoveIt gets v0.1's `OrientationConstraint` with
`enforce_constrained_state_space`. Plan only, same queries. The direct constraint's Jacobian is
central differences, MoveIt's is analytic.

{upright}

### Both pipelines on move_group's octomap

The direct checker reads the octomap MoveIt built from the simulated camera (`world.octomap` of
the frozen scene's message, `octomap_msgs::msgToMap`, since MoveIt writes a full rather than
binary map) as one `coal::OcTree`, so both pipelines plan on
the identical map and the v0.1 query set (`queries.json` = `queries_boxes.json`) is reused.
Plan only, unconstrained (`scripts/bench_direct.sh 100 results/bench_direct_octomap.csv
--octomap`). MoveIt's per-check cost on the octomap is not exposed.

{octo}
"""
    if executed:
        body += f"""
### Execution in Gazebo: aborts, repositioning, tracking error

{execution_section(pub)}
"""
    body += f"""
### Collision agreement (MoveIt's planning scene vs the direct checker)

{agreement_paragraph(pub.agreement, pub.boundary) if pub.agreement else "Not run."}

Files: `results/collision_agreement.csv` (+ `.meta.json`),
`results/collision_agreement_boundary.csv`.
"""
    if cmp:
        body += "\n" + run_to_run_section(cmp) + "\n"
    body += f"""
## Regenerate (v0.2)

{REGENERATE}
"""
    return body


def direct_source(pub: Run, args_csv: str) -> str:
    sm, meta = pub.direct, pub.direct_meta
    mo = meta.get("moveit_ompl_context", {})
    probe = (
        f" The per-check row comes from `results/check_probe.json` ({len(pub.probe)} queries, "
        f"commit `{pub.probe_commit}`), not from this CSV."
        if probe_ok(pub.probe)
        else ""
    )
    return (
        f"Source: `{args_csv}` ({len(pub.direct_rows)} rows, {sm['queries']} queries, "
        f"{len(sm['skipped_queries'])} skipped, commit `{meta['commit']}`, query seed "
        f"{meta['query_seed']}, OMPL seed {meta['ompl_seed']} (direct only), "
        f"{'with' if meta['execute'] else 'without'} execution, "
        f"{meta['planning_time_s']:.0f} s per plan, total wall time "
        f"{meta['total_wall_time_s'] / 60:.1f} min). Direct: {meta['planner_config']}, range "
        f"{meta['range_rad']:.2f} rad, check every {meta['longest_valid_segment_rad']:.4f} rad "
        f"(extent {meta['max_extent_rad']:.2f} rad). MoveIt's configured context: "
        f"range {mo.get('range', '?')}, check every "
        f"{f(mo.get('longest_valid_segment_rad', math.nan), 4)} rad (extent "
        f"{f(mo.get('max_extent_rad', math.nan), 2)} rad).{probe}"
    )


def readme_blocks(pub: Run, cmp: Comparison | None) -> dict[str, str]:
    blocks = {
        "headline": headline_block(pub),
        "breath": breath_block(pub),
        "results": results_block(pub),
        "f16-numbers": f16_block(pub),
        "f17-numbers": f17_block(pub),
        "f26-numbers": f26_block(pub),
        "f27-numbers": f27_block(pub),
        "failure-numbers": failure_numbers_block(pub, cmp.re if cmp else None),
    }
    blocks["reproducibility"] = (
        reproducibility_block(cmp) if cmp else f"No rerun found in `{RERUN}/`; nothing to compare."
    )
    return blocks


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default="results/bench_direct.csv")
    ap.add_argument("--rerun", default=RERUN, help="directory of the rerun's CSV/JSON files")
    ap.add_argument("--readme", default="README.md")
    ap.add_argument("--results-md", default="docs/results.md")
    ap.add_argument("--summary", default="results/summary_direct.json")
    ap.add_argument("--no-plots", action="store_true")
    ap.add_argument("--print-only", action="store_true")
    ap.add_argument(
        "--check",
        action="store_true",
        help="write nothing, make no plots; exit 1 if docs/results.md, a README block or the "
        "summary JSON is stale",
    )
    args = ap.parse_args(argv)
    csv_path = REPO / args.csv
    if args.print_only:
        rows = load(csv_path)
        meta = load_meta(csv_path)
        sm = summarise(rows)
        print(main_table(sm, None, None))
        mo = meta.get("moveit_ompl_context", {})
        print(
            f"Source: `{args.csv}` ({len(rows)} rows, {sm['queries']} queries, "
            f"{len(sm['skipped_queries'])} skipped, commit `{meta.get('commit', '?')}`, "
            f"MoveIt range {mo.get('range', '?')})."
        )
        return 0
    pub = Run(REPO / PUBLISHED, PUB_LABEL, direct_csv=csv_path)
    rerun_dir = REPO / args.rerun
    cmp = (
        Comparison(pub, Run(rerun_dir, RERUN_LABEL))
        if (rerun_dir / "bench_direct.csv").exists()
        else None
    )
    body = direct_body(pub, args.csv, cmp)
    results_path = REPO / args.results_md
    old_results = results_path.read_text() if results_path.exists() else ""
    if old_results.count(DIRECT_START) > 1 or old_results.count(DIRECT_END) > 1:
        raise SystemExit(f"{args.results_md}: {DIRECT_START} / {DIRECT_END} appear more than once")
    new_results = replace_or_append(old_results, DIRECT_START, DIRECT_END, body)
    readme_path = REPO / args.readme
    old_readme = readme_path.read_text()
    new_readme = readme_text(old_readme, readme_blocks(pub, cmp), args.readme)
    summary_path = REPO / args.summary
    summary = json.dumps(pub.direct, indent=2, default=str) + "\n"

    if args.check:
        stale = [
            name
            for name, old, new in (
                (args.results_md, old_results, new_results),
                (args.readme, old_readme, new_readme),
                (args.summary, summary_path.read_text() if summary_path.exists() else "", summary),
            )
            if old != new
        ]
        if stale:
            print(
                "stale (re-run scripts/aggregate_direct.py): " + ", ".join(stale), file=sys.stderr
            )
            return 1
        print("aggregate_direct: docs/results.md (v0.2 block), README blocks and summary current")
        return 0

    summary_path.write_text(summary)
    if not args.no_plots:
        make_plot(pub.direct_rows, pub.direct_meta["planning_time_s"], REPO / LOG_PLOT)
        make_plain_plot(pub.direct_rows, REPO / PLAIN_PLOT)
    results_path.write_text(new_results)
    readme_path.write_text(new_readme)
    print(main_table(pub.direct, pub.agreement, pub.probe))
    print(direct_source(pub, args.csv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
