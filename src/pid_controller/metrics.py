"""Control-response metrics with explicit definitions and edge-case behavior."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from pid_controller.exceptions import PIDInputError
from pid_controller.simulation import SimulationResult


@dataclass(frozen=True, slots=True)
class ResponseMetrics:
    """Summary metrics for a constant-setpoint closed-loop response."""

    rise_time: float | None
    settling_time: float | None
    overshoot_percent: float | None
    peak_value: float
    steady_state_error: float
    iae: float
    ise: float


def _first_time_at_or_above(
    times: tuple[float, ...], values: tuple[float, ...], threshold: float
) -> float | None:
    """Return first timestamp at which normalized progress reaches a threshold."""
    for time, value in zip(times, values, strict=True):
        if value >= threshold:
            return time
    return None


def _trapezoid(
    times: tuple[float, ...], values: tuple[float, ...]
) -> float:
    """Integrate sampled values with the trapezoidal rule."""
    return sum(
        (times[index] - times[index - 1])
        * (values[index] + values[index - 1])
        / 2.0
        for index in range(1, len(times))
    )


def calculate_metrics(
    result: SimulationResult,
    *,
    settling_tolerance: float = 0.02,
    rise_bounds: tuple[float, float] = (0.1, 0.9),
) -> ResponseMetrics:
    """Calculate standard metrics for a constant-setpoint response.

    Rise time uses the supplied normalized lower/upper bounds. Settling time is
    the first sample after the last tolerance-band violation. Metrics spanning
    multiple setpoints are rejected because a single value would be ambiguous.
    """
    if len(result.records) < 2:
        raise PIDInputError("metrics require at least two samples")
    if not isfinite(settling_tolerance) or not 0.0 < settling_tolerance < 1.0:
        raise PIDInputError("settling_tolerance must be between zero and one")
    lower_bound, upper_bound = rise_bounds
    if not 0.0 <= lower_bound < upper_bound <= 1.0:
        raise PIDInputError("rise_bounds must satisfy 0 <= lower < upper <= 1")

    times = result.signal("time")
    setpoints = result.signal("setpoint")
    measurements = result.signal("measurement")
    if any(value != setpoints[0] for value in setpoints[1:]):
        raise PIDInputError("segment setpoint changes before calculating metrics")

    target = setpoints[0]
    initial = measurements[0]
    amplitude = target - initial
    errors = tuple(target - value for value in measurements)
    absolute_errors = tuple(abs(value) for value in errors)
    squared_errors = tuple(value * value for value in errors)

    if amplitude == 0.0:
        rise_time = None
        overshoot_percent = None
        peak_value = max(measurements)
    else:
        progress = tuple((value - initial) / amplitude for value in measurements)
        lower_time = _first_time_at_or_above(times, progress, lower_bound)
        upper_time = _first_time_at_or_above(times, progress, upper_bound)
        rise_time = (
            None
            if lower_time is None or upper_time is None
            else upper_time - lower_time
        )
        if amplitude > 0.0:
            peak_value = max(measurements)
            overshoot = max(0.0, peak_value - target)
        else:
            peak_value = min(measurements)
            overshoot = max(0.0, target - peak_value)
        overshoot_percent = 100.0 * overshoot / abs(amplitude)

    tolerance = settling_tolerance * max(abs(amplitude), 1.0)
    outside = [index for index, error in enumerate(errors) if abs(error) > tolerance]
    if not outside:
        settling_time = times[0]
    elif outside[-1] >= len(times) - 1:
        settling_time = None
    else:
        settling_time = times[outside[-1] + 1]

    return ResponseMetrics(
        rise_time=rise_time,
        settling_time=settling_time,
        overshoot_percent=overshoot_percent,
        peak_value=peak_value,
        steady_state_error=errors[-1],
        iae=_trapezoid(times, absolute_errors),
        ise=_trapezoid(times, squared_errors),
    )

