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


@pytest.mark.parametrize("filter_tau", [-0.1, inf, nan])
def test_invalid_derivative_filter_time_constant_is_rejected(
    filter_tau: float,
) -> None:
    """Derivative filter time constant must be finite and non-negative."""
    with pytest.raises(PIDConfigurationError):
        PIDConfig(kp=1.0, derivative_filter_tau=filter_tau)


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


def test_integral_accumulates_error_over_fixed_time_steps() -> None:
    """PI control should retain the discrete integral contribution."""
    controller = PIDController(PIDConfig(kp=2.0, ki=0.5, sample_time=0.2))

    first = controller.update(5.0, 3.0)
    second = controller.update(5.0, 3.0)

    assert first.integral == pytest.approx(0.2)
    assert first.output == pytest.approx(4.2)
    assert second.integral == pytest.approx(0.4)
    assert second.output == pytest.approx(4.4)


def test_integral_respects_error_sign_and_sample_time() -> None:
    """Negative error should reduce the integral using the configured step."""
    controller = PIDController(PIDConfig(kp=0.0, ki=2.0, sample_time=0.25))
    assert controller.update(0.0, 4.0).integral == pytest.approx(-2.0)


def test_zero_integral_gain_disables_integral_action() -> None:
    """P-only operation should not accumulate hidden integral state."""
    controller = PIDController(PIDConfig(kp=1.0, ki=0.0))
    controller.update(10.0, 0.0)
    assert controller.update(10.0, 0.0).integral == 0.0


def test_reset_clears_integral_state() -> None:
    """Reset should restart PI control from a zero integral contribution."""
    controller = PIDController(PIDConfig(kp=0.0, ki=1.0, sample_time=0.5))
    controller.update(2.0, 0.0)
    controller.reset()
    assert controller.update(2.0, 0.0).integral == pytest.approx(1.0)


def test_derivative_on_error_uses_error_difference() -> None:
    """Error-based derivative should respond to a setpoint change."""
    controller = PIDController(
        PIDConfig(
            kp=0.0,
            kd=2.0,
            sample_time=0.5,
            derivative_on_measurement=False,
        )
    )

    assert controller.update(1.0, 0.0).derivative == 0.0
    assert controller.update(2.0, 0.0).derivative == pytest.approx(4.0)


def test_derivative_on_measurement_avoids_setpoint_kick() -> None:
    """Changing only setpoint should not create derivative-on-measurement kick."""
    controller = PIDController(PIDConfig(kp=0.0, kd=3.0, sample_time=0.25))
    controller.update(1.0, 0.0)
    assert controller.update(4.0, 0.0).derivative == 0.0
    assert controller.update(4.0, 1.0).derivative == pytest.approx(-12.0)


def test_derivative_filter_applies_first_order_smoothing() -> None:
    """A positive filter constant should attenuate a derivative step."""
    controller = PIDController(
        PIDConfig(
            kp=0.0,
            kd=2.0,
            sample_time=0.1,
            derivative_filter_tau=0.1,
        )
    )
    controller.update(0.0, 0.0)
    result = controller.update(0.0, 1.0)
    assert result.derivative == pytest.approx(-10.0)


def test_reset_clears_derivative_history() -> None:
    """The first update after reset should use the zero-derivative policy."""
    controller = PIDController(PIDConfig(kp=0.0, kd=1.0))
    controller.update(0.0, 0.0)
    assert controller.update(0.0, 1.0).derivative != 0.0
    controller.reset()
    assert controller.update(0.0, 5.0).derivative == 0.0
