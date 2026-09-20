"""Tests for response metric definitions."""

import pytest

from pid_controller import SimulationRecord, SimulationResult, calculate_metrics
from pid_controller.exceptions import PIDInputError


def make_result(
    measurements: tuple[float, ...],
    *,
    setpoints: tuple[float, ...] | None = None,
) -> SimulationResult:
    """Build a minimal result with one-second spacing for metric tests."""
    active_setpoints = setpoints or (10.0,) * len(measurements)
    records = tuple(
        SimulationRecord(
            time=float(index),
            setpoint=active_setpoints[index],
            measurement=value,
            error=active_setpoints[index] - value,
            proportional=0.0,
            integral=0.0,
            derivative=0.0,
            raw_output=0.0,
            output=0.0,
            saturated=False,
        )
        for index, value in enumerate(measurements)
    )
    return SimulationResult(records, 1.0)


def test_metrics_for_known_positive_step_response() -> None:
    """Metric calculations should match a hand-checkable response."""
    metrics = calculate_metrics(make_result((0.0, 2.0, 9.0, 11.0, 10.1, 10.0)))
    assert metrics.rise_time == 1.0
    assert metrics.settling_time == 4.0
    assert metrics.overshoot_percent == pytest.approx(10.0)
    assert metrics.peak_value == 11.0
    assert metrics.steady_state_error == 0.0
    assert metrics.iae == pytest.approx(15.1)
    assert metrics.ise == pytest.approx(116.01)


def test_metrics_support_negative_step() -> None:
    """Progress and overshoot direction should follow a negative amplitude."""
    metrics = calculate_metrics(
        make_result((0.0, -5.0, -11.0, -10.0), setpoints=(-10.0,) * 4)
    )
    assert metrics.rise_time == 1.0
    assert metrics.overshoot_percent == 10.0
    assert metrics.peak_value == -11.0


def test_zero_amplitude_has_undefined_rise_and_overshoot() -> None:
    """A response with no commanded step should not invent step metrics."""
    metrics = calculate_metrics(make_result((10.0, 10.0, 10.0)))
    assert metrics.rise_time is None
    assert metrics.overshoot_percent is None
    assert metrics.settling_time == 0.0


def test_unreached_response_has_no_rise_or_settling_time() -> None:
    """Non-reaching traces should report missing time metrics."""
    metrics = calculate_metrics(make_result((0.0, 0.5, 1.0)))
    assert metrics.rise_time is None
    assert metrics.settling_time is None


@pytest.mark.parametrize("tolerance", [0.0, 1.0, float("inf")])
def test_invalid_settling_tolerance_is_rejected(tolerance: float) -> None:
    """Tolerance must be a finite fraction."""
    with pytest.raises(PIDInputError):
        calculate_metrics(make_result((0.0, 10.0)), settling_tolerance=tolerance)


@pytest.mark.parametrize("bounds", [(-0.1, 0.9), (0.9, 0.1), (0.5, 0.5), (0.1, 1.1)])
def test_invalid_rise_bounds_are_rejected(bounds: tuple[float, float]) -> None:
    """Rise bounds must be ordered normalized fractions."""
    with pytest.raises(PIDInputError):
        calculate_metrics(make_result((0.0, 10.0)), rise_bounds=bounds)


def test_setpoint_change_requires_segmentation() -> None:
    """A single metric set must not mix distinct command steps."""
    with pytest.raises(PIDInputError):
        calculate_metrics(make_result((0.0, 1.0), setpoints=(1.0, 2.0)))


def test_metrics_require_two_samples() -> None:
    """Numerical integration needs an interval."""
    with pytest.raises(PIDInputError):
        calculate_metrics(make_result((0.0,)))
