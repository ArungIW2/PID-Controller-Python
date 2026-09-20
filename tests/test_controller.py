"""Tests for the discrete controller core."""

from math import inf, nan

import pytest

from pid_controller import (
    AntiWindupMode,
    OperatingMode,
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


def test_output_limits_report_saturation() -> None:
    """Controller output should be clamped while preserving raw output."""
    controller = PIDController(PIDConfig(kp=10.0, output_limits=(-2.0, 3.0)))
    high = controller.update(1.0, 0.0)
    low = controller.update(-1.0, 0.0)
    assert (high.raw_output, high.output, high.saturated) == (10.0, 3.0, True)
    assert (low.raw_output, low.output, low.saturated) == (-10.0, -2.0, True)


def test_clamping_anti_windup_stops_integral_pushing_into_limit() -> None:
    """Conditional integration should stop further saturation pressure."""
    controller = PIDController(
        PIDConfig(kp=0.0, ki=2.0, sample_time=1.0, output_limits=(0.0, 5.0))
    )
    for _ in range(5):
        result = controller.update(10.0, 0.0)
    assert result.integral == 0.0
    assert result.output == 0.0


def test_clamping_allows_integral_to_recover_from_existing_state() -> None:
    """Integral movement away from saturation should remain enabled."""
    controller = PIDController(
        PIDConfig(kp=0.0, ki=1.0, sample_time=1.0, output_limits=(-5.0, 5.0))
    )
    controller.update(4.0, 0.0)
    recovered = controller.update(-2.0, 0.0)
    assert recovered.integral == pytest.approx(2.0)


def test_no_anti_windup_preserves_unlimited_integral_state() -> None:
    """NONE mode should expose classic windup for comparison experiments."""
    controller = PIDController(
        PIDConfig(
            kp=0.0,
            ki=2.0,
            sample_time=1.0,
            output_limits=(0.0, 5.0),
            anti_windup=AntiWindupMode.NONE,
        )
    )
    result = controller.update(10.0, 0.0)
    assert result.integral == 20.0
    assert result.output == 5.0


def test_back_calculation_moves_integral_toward_limited_output() -> None:
    """Back calculation should feed saturation difference into integral state."""
    controller = PIDController(
        PIDConfig(
            kp=0.0,
            ki=2.0,
            sample_time=0.5,
            output_limits=(0.0, 5.0),
            anti_windup=AntiWindupMode.BACK_CALCULATION,
            back_calculation_gain=1.0,
        )
    )
    result = controller.update(10.0, 0.0)
    assert result.integral == pytest.approx(7.5)
    assert result.output == 5.0


def test_integral_limits_are_applied_without_output_saturation() -> None:
    """Independent integral bounds should constrain stored contribution."""
    controller = PIDController(
        PIDConfig(
            kp=0.0,
            ki=10.0,
            sample_time=1.0,
            integral_limits=(-2.0, 2.0),
        )
    )
    assert controller.update(1.0, 0.0).integral == 2.0
    assert controller.update(-1.0, 0.0).integral == -2.0


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("output_limits", (2.0, 2.0)),
        ("output_limits", (3.0, 2.0)),
        ("output_limits", (None, inf)),
        ("integral_limits", (-inf, None)),
        ("output_limits", (0.0,)),
        ("output_limits", [0.0, 1.0]),
    ],
)
def test_invalid_limit_configuration_is_rejected(field: str, value: object) -> None:
    """Limit pairs must be finite, ordered two-element tuples."""
    with pytest.raises(PIDConfigurationError):
        PIDConfig(kp=1.0, **{field: value})  # type: ignore[arg-type]


@pytest.mark.parametrize("gain", [-1.0, inf, nan])
def test_invalid_back_calculation_gain_is_rejected(gain: float) -> None:
    """Back-calculation feedback gain must be finite and non-negative."""
    with pytest.raises(PIDConfigurationError):
        PIDConfig(kp=1.0, back_calculation_gain=gain)


def test_invalid_anti_windup_mode_is_rejected() -> None:
    """Configuration should require a named anti-windup strategy."""
    with pytest.raises(PIDConfigurationError):
        PIDConfig(kp=1.0, anti_windup="clamp")  # type: ignore[arg-type]


def test_manual_mode_clamps_output_and_freezes_integral() -> None:
    """Manual operation should use operator output without integrating error."""
    controller = PIDController(
        PIDConfig(kp=1.0, ki=1.0, output_limits=(0.0, 5.0))
    )
    controller.set_mode(OperatingMode.MANUAL, manual_output=8.0)
    result = controller.update(10.0, 0.0)
    assert controller.mode is OperatingMode.MANUAL
    assert result.raw_output == 8.0
    assert result.output == 5.0
    assert result.integral == 0.0
    assert result.saturated is True


def test_return_to_automatic_is_bumpless() -> None:
    """First automatic output should match the last achievable manual output."""
    controller = PIDController(
        PIDConfig(kp=2.0, ki=1.0, sample_time=1.0, output_limits=(0.0, 10.0))
    )
    controller.set_mode(OperatingMode.MANUAL, manual_output=4.0)
    controller.update(3.0, 1.0)
    controller.set_mode(OperatingMode.AUTOMATIC)
    result = controller.update(3.0, 1.0)
    assert controller.mode is OperatingMode.AUTOMATIC
    assert result.output == pytest.approx(4.0)


@pytest.mark.parametrize("manual_output", [None, inf, nan])
def test_manual_mode_requires_finite_output(manual_output: float | None) -> None:
    """Manual operation requires an explicit finite command."""
    with pytest.raises(PIDInputError):
        PIDController(PIDConfig(kp=1.0)).set_mode(
            OperatingMode.MANUAL, manual_output=manual_output
        )


def test_invalid_operating_mode_is_rejected() -> None:
    """Operating mode must use the public enum."""
    with pytest.raises(PIDInputError):
        PIDController(PIDConfig(kp=1.0)).set_mode("automatic")  # type: ignore[arg-type]


def test_reset_restores_automatic_mode() -> None:
    """Reset should clear manual and transfer state."""
    controller = PIDController(PIDConfig(kp=1.0))
    controller.set_mode(OperatingMode.MANUAL, manual_output=1.0)
    controller.reset()
    assert controller.mode is OperatingMode.AUTOMATIC
