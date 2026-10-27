"""Unit tests for scripts/aggregate.py (pure Python; run with `python3 -m pytest scripts/tests`)."""

import importlib.util
import math
import pathlib

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("aggregate", HERE.parent / "aggregate.py")
agg = importlib.util.module_from_spec(spec)
spec.loader.exec_module(agg)


def test_wilson_known_values():
    # Reference values from the closed form (Wilson 1927), z = 1.96.
    lo, hi = agg.wilson(50, 100)
    assert math.isclose(lo, 0.4038, abs_tol=1e-4)
    assert math.isclose(hi, 0.5962, abs_tol=1e-4)
    lo, hi = agg.wilson(100, 100)
    assert math.isclose(lo, 0.9630, abs_tol=1e-4)
    assert hi == 1.0
    lo, hi = agg.wilson(0, 10)
    assert lo == 0.0
    assert math.isclose(hi, 0.2775, abs_tol=1e-4)
    assert all(math.isnan(v) for v in agg.wilson(0, 0))


def test_percentile_matches_linear_interpolation():
    xs = [1.0, 2.0, 3.0, 4.0, 5.0]
    assert agg.percentile(xs, 50) == 3.0
    assert math.isclose(agg.percentile(xs, 95), 4.8)
    assert agg.percentile([7.0], 95) == 7.0
    assert math.isnan(agg.percentile([], 50))


def test_compartment_classes():
    assert agg.compartment_change("lower_left", "lower_left") == "same compartment"
    assert agg.compartment_change("lower_left", "lower_right") == "across divider"
    assert agg.compartment_change("lower_left", "upper_left") == "across board"
    assert agg.compartment_change("lower_left", "upper_right") == "diagonal"


def _row(constraint, success, t, tilt="1.0"):
    return {
        "planner": "RRTConnectkConfigDefault",
        "constraint": constraint,
        "success": "1" if success else "0",
        "planning_time_s": str(t),
        "joint_path_rad": "2.0" if success else "",
        "tcp_path_m": "0.5" if success else "",
        "max_tilt_deg": tilt if success else "",
        "start_valid": "1",
        "goal_valid": "1",
    }


def test_summary_uses_successes_only_for_time_and_length():
    rows = [_row("none", True, 0.1), _row("none", True, 0.3), _row("none", False, 10.0)]
    (s,) = agg.summarise(rows)
    assert s["n"] == 3 and s["successes"] == 2
    assert math.isclose(s["time_median_s"], 0.2)
    assert s["joint_path_median_rad"] == 2.0
    assert s["planner"] == "RRTConnect (auto range, 13.8 rad)"
