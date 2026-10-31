"""Unit tests for scripts/aggregate_direct.py (pure Python; `python3 -m pytest scripts/tests`)."""

import importlib.util
import math
import pathlib

import pytest

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


REPO = HERE.parent.parent
STALE = [
    "3.3×",
    "query 63",
    "0.207 rad",
    "9 of 100",
    "3 of 100",
    "2 escapes",
    "0.033 / 0.054",
    "0.046 / 0.058",
    "11 of 20",
]


def scratch_readme(tmp_path, names=agd.README_BLOCKS + agd.OPTIONAL_README_BLOCKS, extra=""):
    body = "# Title\n\n" + "".join(
        f"<!-- {n}:start -->\n<!-- {n}:end -->\n\nprose\n\n" for n in names
    )
    path = tmp_path / "README.md"
    path.write_text(body + extra)
    return path


def run(tmp_path, readme, *extra):
    results = tmp_path / "results.md"
    if not results.exists():
        results.write_text((REPO / "docs" / "results.md").read_text())
    return agd.main(
        [
            "--readme",
            str(readme),
            "--results-md",
            str(results),
            "--summary",
            str(tmp_path / "summary_direct.json"),
            "--no-plots",
            *extra,
        ]
    )


def test_check_mode_detects_a_stale_readme_block(tmp_path, capsys):
    readme = scratch_readme(tmp_path)
    assert run(tmp_path, readme) == 0
    assert run(tmp_path, readme, "--check") == 0
    text = readme.read_text()
    for n in agd.README_BLOCKS + agd.OPTIONAL_README_BLOCKS:
        start, end = agd.markers(n)
        assert text.count(start) == 1 and text.count(end) == 1
        assert text.index(start) < text.index(end)
    # Hand-edit one generated number: --check must fail and name the README.
    readme.write_text(text.replace("79/100", "80/100", 1))
    assert run(tmp_path, readme, "--check") == 1
    assert "README.md" in capsys.readouterr().err


def test_readme_blocks_carry_no_stale_hand_typed_strings(tmp_path):
    readme = scratch_readme(tmp_path)
    run(tmp_path, readme)
    text = readme.read_text()
    assert not [s for s in STALE if s in text]


@pytest.mark.parametrize("drop", agd.README_BLOCKS)
def test_missing_marker_pair_fails_clearly(tmp_path, drop):
    readme = scratch_readme(
        tmp_path, [n for n in agd.README_BLOCKS + ("failure-numbers",) if n != drop]
    )
    with pytest.raises(SystemExit, match=f"<!-- {drop}:start -->"):
        run(tmp_path, readme, "--check")


def test_duplicated_or_retired_marker_fails_clearly(tmp_path):
    readme = scratch_readme(tmp_path, extra="<!-- results:start -->\n<!-- results:end -->\n")
    with pytest.raises(SystemExit, match="exactly once"):
        run(tmp_path, readme, "--check")
    readme = scratch_readme(
        tmp_path, extra="<!-- direct-results:start -->\n<!-- direct-results:end -->\n"
    )
    with pytest.raises(SystemExit, match="retired"):
        run(tmp_path, readme, "--check")


def test_success_sentence_is_data_gated():
    cmp = object.__new__(agd.Comparison)
    cmp.counts = []
    cmp.rechecks = []
    cmp._count("success", "v0.2", "a: planned", 99, 100, 98, 100)
    assert cmp.success_sentence().startswith("Every success count of the 31 Oct run is inside")
    cmp._count("success", "v0.2", "b: planned", 99, 100, 90, 100)
    s = cmp.success_sentence()
    assert "except b: planned (99/100 → 90/100)" in s


def test_repo_readme_and_results_are_current():
    """Every README number is generated: the committed README blocks and docs/results.md must be
    what the scripts produce from the committed CSVs."""
    text = (REPO / "README.md").read_text()
    if not all(agd.markers(n)[0] in text for n in agd.README_BLOCKS):
        pytest.skip("README has not placed every generated block yet")
    agg = agd._agg
    assert agg.main(["--check"]) == 0
    assert agd.main(["--check"]) == 0


def test_indented_block_stays_inside_its_list_item():
    text = "- item\n\n  <!-- x:start -->\n  old\n  <!-- x:end -->\n- next\n"
    start, end = agd.markers("x")
    once = agd.replace_block(text, start, end, "line 1\n\n| a |\n|---|")
    assert once == (
        "- item\n\n  <!-- x:start -->\n  line 1\n\n  | a |\n  |---|\n  <!-- x:end -->\n- next\n"
    )
    assert agd.replace_block(once, start, end, "line 1\n\n| a |\n|---|") == once


def test_failure_number_blocks_are_optional_but_one_is_needed(tmp_path):
    readme = scratch_readme(tmp_path, agd.README_BLOCKS + ("f26-numbers",))
    run(tmp_path, readme)
    text = readme.read_text()
    assert "failure-numbers" not in text and "dense-check step" in text
    readme = scratch_readme(tmp_path, agd.README_BLOCKS + ("failure-numbers",))
    run(tmp_path, readme)
    assert "| F27 |" in readme.read_text()
    readme = scratch_readme(tmp_path, agd.README_BLOCKS)
    with pytest.raises(SystemExit, match="failure items"):
        run(tmp_path, readme, "--check")
    dup = "<!-- f27-numbers:start -->\n<!-- f27-numbers:end -->\n"
    readme = scratch_readme(tmp_path, agd.README_BLOCKS + ("f27-numbers",), extra=dup)
    with pytest.raises(SystemExit, match="exactly once"):
        run(tmp_path, readme, "--check")


def test_lever_arm_constant_matches_ur_description():
    """The tool-distance bound uses ur_description's UR5e link offsets; check the copy."""
    yaml = pytest.importorskip("yaml")

    path = pathlib.Path("/opt/ros/lyrical/share/ur_description/config/ur5e/default_kinematics.yaml")
    if not path.exists():
        pytest.skip("ur_description not installed")
    k = yaml.safe_load(path.read_text())["kinematics"]
    want = [
        tuple(k[link][a] for a in "xyz") for link in ("forearm", "wrist_1", "wrist_2", "wrist_3")
    ]
    for got, ref in zip(agd.UR5E_LINK_OFFSETS_M, want, strict=True):
        assert all(abs(g - r) < 1e-6 for g, r in zip(got, ref, strict=True))


def test_tool_bound_wording_is_data_gated():
    small, large = {"p95": 0.001}, {"p95": 0.2}
    assert "less than the queries' 3 cm clearance" in agd.tool_bound_sentence(small, small)
    assert "more than the queries' 3 cm clearance" in agd.tool_bound_sentence(large, large)
    mixed = agd.tool_bound_sentence(small, large)
    assert "less than" in mixed and "more than for MoveIt" in mixed
