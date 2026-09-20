"""Tests for mathematical process models."""

from math import inf, nan

import pytest

from pid_controller import FirstOrderProcess, FirstOrderProcessConfig
from pid_controller.exceptions import PIDConfigurationError, PIDInputError


def test_first_order_process_matches_one_euler_step() -> None:
    """Model update should match its documented discrete equation."""
    process = FirstOrderProcess(
        FirstOrderProcessConfig(gain=2.0, time_constant=4.0, initial_value=1.0)
    )
    assert process.output == 1.0
    assert process.step(control_input=3.0, sample_time=0.5) == pytest.approx(1.625)


def test_default_process_and_reset_behaviour() -> None:
    """Reset should support configured and explicitly supplied states."""
    process = FirstOrderProcess()
    process.step(1.0, 0.1)
    process.reset()
    assert process.output == 0.0
    process.reset(2.5)
    assert process.output == 2.5


@pytest.mark.parametrize(
    "values",
    [
        {"gain": inf},
        {"time_constant": inf},
        {"initial_value": nan},
        {"time_constant": 0.0},
        {"time_constant": -1.0},
    ],
)
def test_invalid_process_configuration_is_rejected(values: dict[str, float]) -> None:
    """Model parameters must define a finite process with positive tau."""
    with pytest.raises(PIDConfigurationError):
        FirstOrderProcessConfig(**values)


@pytest.mark.parametrize(
    ("control_input", "sample_time"),
    [(inf, 0.1), (nan, 0.1), (1.0, 0.0), (1.0, -0.1), (1.0, inf)],
)
def test_invalid_model_step_is_rejected(
    control_input: float, sample_time: float
) -> None:
    """Invalid inputs must not contaminate mathematical model state."""
    with pytest.raises(PIDInputError):
        FirstOrderProcess().step(control_input, sample_time)


def test_invalid_reset_value_is_rejected() -> None:
    """Reset state must be finite."""
    with pytest.raises(PIDInputError):
        FirstOrderProcess().reset(nan)

