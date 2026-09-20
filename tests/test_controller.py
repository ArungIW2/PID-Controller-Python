"""Tests for the discrete controller core."""

from math import inf, nan

import pytest

from pid_controller import (
    PIDConfig,
    PIDConfigurationError,
    PIDController,
    PIDInputError,
)


def test_proportional_update_reports_all_observable_values() -> None:
    """P control should multiply signed error by proportional gain."""
    result = PIDController(PIDConfig(kp=2.5)).update(10.0, 6.0)

    assert result.setpoint == 10.0
    assert result.measurement == 6.0
    assert result.error == 4.0
    assert result.proportional == 10.0
    assert result.integral == 0.0
    assert result.derivative == 0.0
    assert result.raw_output == 10.0
    assert result.output == 10.0
    assert result.saturated is False


def test_proportional_update_preserves_negative_error_sign() -> None:
    """A measurement above setpoint should produce negative P action."""
    result = PIDController(PIDConfig(kp=3.0)).update(2.0, 5.0)
    assert result.error == -3.0
    assert result.output == -9.0


def test_zero_gain_produces_zero_output() -> None:
    """A zero proportional gain disables proportional action."""
    assert PIDController(PIDConfig(kp=0.0)).update(10.0, 1.0).output == 0.0


def test_controller_exposes_immutable_configuration() -> None:
    """The exact validated configuration should remain inspectable."""
    config = PIDConfig(kp=1.0, sample_time=0.2)
    assert PIDController(config).config is config


@pytest.mark.parametrize("field", ["kp", "ki", "kd"])
@pytest.mark.parametrize("value", [-1.0, inf, -inf, nan])
def test_invalid_gain_is_rejected(field: str, value: float) -> None:
    """Gains must be finite and non-negative."""
    values = {"kp": 1.0, "ki": 0.0, "kd": 0.0, field: value}
    with pytest.raises(PIDConfigurationError):
        PIDConfig(**values)


@pytest.mark.parametrize("sample_time", [0.0, -0.1, inf, nan])
def test_invalid_sample_time_is_rejected(sample_time: float) -> None:
    """The fixed time step must be positive and finite."""
    with pytest.raises(PIDConfigurationError):
        PIDConfig(kp=1.0, sample_time=sample_time)


@pytest.mark.parametrize(
    ("setpoint", "measurement"),
    [(nan, 0.0), (0.0, nan), (inf, 0.0), (0.0, -inf)],
)
def test_non_finite_runtime_input_is_rejected(
    setpoint: float, measurement: float
) -> None:
    """Runtime inputs must not silently contaminate controller state."""
    controller = PIDController(PIDConfig(kp=1.0))
    with pytest.raises(PIDInputError):
        controller.update(setpoint, measurement)


def test_controller_requires_pid_config() -> None:
    """Reject an accidental mapping or unrelated configuration object."""
    with pytest.raises(TypeError):
        PIDController({"kp": 1.0})  # type: ignore[arg-type]


def test_reset_is_safe_for_stateless_proportional_control() -> None:
    """Reset should preserve deterministic P-only behavior."""
    controller = PIDController(PIDConfig(kp=2.0))
    before = controller.update(1.0, 0.0)
    controller.reset()
    after = controller.update(1.0, 0.0)
    assert before == after
