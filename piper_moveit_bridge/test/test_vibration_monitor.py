import math

from piper_moveit_bridge.vibration_monitor import error_stats, period_stats


def test_period_stats_empty():
    assert period_stats([]) == {
        "count": 0, "mean_ms": None, "std_ms": None, "max_ms": None
    }


def test_period_stats_reports_mean_jitter_and_max():
    result = period_stats([0.01,  0.011, 0.009])
    assert result["count"] == 3
    assert math.isclose(result["mean_ms"], 10.0)
    assert result["max_ms"] == 11.0
    assert result["std_ms"] > 0


def test_error_stats_rms_and_peak():
    result = error_stats([0.0, 0.003, -0.004])
    assert result["count"] == 3
    assert math.isclose(result["rms_rad"], math.sqrt(25e-6 / 3))
    assert result["max_abs_rad"] == 0.004


def test_error_stats_empty():
    assert error_stats([]) == {
        "count": 0, "rms_rad": None, "max_abs_rad": None
    }
