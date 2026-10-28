"""Unit tests for scripts/aggregate_direct.py (pure Python; `python3 -m pytest scripts/tests`)."""

import importlib.util
import math
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "aggregate_direct", HERE.parent / "aggregate_direct.py"
)
agd = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agd)


def row(query, pipeline, plan_ok="1", plan="0.1", simplify="", status="Exact solution", **kw):
    r = {
        "query": str(query),
        "pipeline": pipeline,
        "success": plan_ok,
        "plan_success": plan_ok,
        "exec_success": "",
        "executed": "0",
        "status": status,
        "plan_time_s": plan if plan_ok == "1" else "",
        "simplify_time_s": simplify,
        "crossval_ok": "1" if plan_ok == "1" else "",
        "own_dense_ok": "1" if plan_ok == "1" else "",
        "at_start": "",
    }
    r.update(kw)
    return r


def test_like_for_like_time_adds_simplification_only_for_direct():
    assert agd.plan_plus_simplify(row(0, "direct", plan="0.2", simplify="0.3")) == 0.5
    assert agd.plan_plus_simplify(row(0, "moveit", plan="0.4")) == 0.4
    assert agd.plan_plus_simplify(row(0, "direct", plan_ok="0", plan="", simplify="")) is None


def test_summary_pairs_queries_and_skips_rejected_ones():
    rows = [
        row(0, "direct", plan="0.1", simplify="0.1"),
        row(0, "moveit", plan="0.1"),
        row(1, "direct", plan="0.3", simplify="0.1"),
        row(1, "moveit", plan="0.2"),
        row(2, "direct", plan_ok="0"),
        row(2, "moveit", plan="0.2"),
        row(3, "direct", plan_ok="0", status="skipped:direct_start;"),
        row(3, "moveit", plan_ok="0", status="skipped:direct_start;"),
    ]
    sm = agd.summarise(rows)
    assert sm["queries"] == 4
    assert sm["skipped_queries"] == [3]
    assert sm["direct"]["attempted"] == 3
    assert sm["direct"]["planned"] == 2
    assert sm["moveit"]["planned"] == 3
    # Paired on queries 0 and 1 only: ratios 2.0 and 2.0.
    assert sm["paired"]["both_planned"] == 2
    assert math.isclose(sm["paired"]["median_ratio_direct_over_moveit"], 2.0)
    assert sm["paired"]["direct_faster"] == 0
    assert sm["direct"]["simplify_dominates"] == 0
    assert math.isclose(sm["direct"]["plan_plus_simplify_s"]["median"], 0.3)


def test_replace_or_append_is_idempotent():
    text = "# Results\n\nv0.1 tables\n"
    once = agd.replace_or_append(text, agd.DIRECT_START, agd.DIRECT_END, "body 1")
    assert once.count(agd.DIRECT_START) == 1 and "body 1" in once
    twice = agd.replace_or_append(once, agd.DIRECT_START, agd.DIRECT_END, "body 2")
    assert twice.count(agd.DIRECT_START) == 1
    assert "body 1" not in twice and "body 2" in twice
    assert twice.startswith("# Results\n\nv0.1 tables\n")
