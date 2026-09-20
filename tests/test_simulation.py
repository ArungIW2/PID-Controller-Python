"""Tests for deterministic closed-loop orchestration."""

from math import inf

import pytest

from pid_controller import (
    FirstOrderProcess,
    PIDConfig,
    PIDController,
    SetpointChange,
    run_closed_loop,
)
from pid_controller.exceptions import PIDInputError


def test_closed_loop_records_initial_and_final_samples() -> None:
    """The runner should preserve signal ordering and deterministic length."""
    result = run_closed_loop(
        PIDController(PIDConfig(kp=1.0, sample_time=0.5)),
        FirstOrderProcess(),
        duration=1.0,
        setpoint=2.0,
    )

    assert len(result.records) == 3
    assert result.sample_time == 0.5
    assert result.signal("time") == (0.0, 0.5, 1.0)
    assert result.records[0].measurement == 0.0
    assert result.records[0].output == 2.0
    assert result.records[1].measurement == pytest.approx(1.0)


def test_setpoint_schedule_changes_active_value() -> None:
    """Ordered setpoint changes should become active at their sample."""
    result = run_closed_loop(
        PIDController(PIDConfig(kp=0.0, sample_time=0.5)),
        FirstOrderProcess(),
        duration=1.0,
        setpoint=1.0,
        setpoint_changes=(SetpointChange(0.5, 3.0), SetpointChange(1.0, -1.0)),
    )
    assert result.signal("setpoint") == (1.0, 3.0, -1.0)


def test_unknown_or_boolean_signal_is_rejected() -> None:
    """Signal extraction should expose only known numeric record fields."""
    result = run_closed_loop(
        PIDController(PIDConfig(kp=1.0)),
        FirstOrderProcess(),
        duration=0.1,
        setpoint=1.0,
    )
    with pytest.raises(KeyError):
        result.signal("missing")
    with pytest.raises(KeyError):
        result.signal("saturated")


@pytest.mark.parametrize("duration", [0.0, -1.0, inf])
def test_invalid_duration_is_rejected(duration: float) -> None:
    """A run must have finite positive duration."""
    with pytest.raises(PIDInputError):
        run_closed_loop(
            PIDController(PIDConfig(kp=1.0)),
            FirstOrderProcess(),
            duration=duration,
            setpoint=1.0,
        )


def test_duration_must_align_with_sample_time() -> None:
    """Fixed-step runs must not silently shorten or extend duration."""
    with pytest.raises(PIDInputError):
        run_closed_loop(
            PIDController(PIDConfig(kp=1.0, sample_time=0.3)),
            FirstOrderProcess(),
            duration=1.0,
            setpoint=1.0,
        )


def test_non_finite_initial_setpoint_is_rejected() -> None:
    """Initial setpoint must be finite."""
    with pytest.raises(PIDInputError):
        run_closed_loop(
            PIDController(PIDConfig(kp=1.0)),
            FirstOrderProcess(),
            duration=1.0,
            setpoint=inf,
        )


@pytest.mark.parametrize(
    "changes",
    [
        (SetpointChange(-0.1, 1.0),),
        (SetpointChange(1.1, 1.0),),
        (SetpointChange(inf, 1.0),),
        (SetpointChange(0.5, inf),),
        (SetpointChange(0.8, 1.0), SetpointChange(0.2, 2.0)),
    ],
)
def test_invalid_setpoint_schedule_is_rejected(
    changes: tuple[SetpointChange, ...],
) -> None:
    """Schedules must be finite, ordered, and contained in the run."""
    with pytest.raises(PIDInputError):
        run_closed_loop(
            PIDController(PIDConfig(kp=1.0)),
            FirstOrderProcess(),
            duration=1.0,
            setpoint=0.0,
            setpoint_changes=changes,
        )

