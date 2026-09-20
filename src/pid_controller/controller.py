"""Discrete-time PID controller implemented from first principles."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite

from pid_controller.exceptions import PIDConfigurationError, PIDInputError


def _require_finite(name: str, value: float, error_type: type[ValueError]) -> None:
    """Reject NaN and infinite values at public numeric boundaries."""
    if not isfinite(value):
        raise error_type(f"{name} must be finite, got {value!r}")


@dataclass(frozen=True, slots=True)
class PIDConfig:
    """Immutable controller configuration.

    Phase 2 provides proportional control. Integral and derivative gains are
    present with zero defaults so later controller modes extend this stable API.
    """

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    sample_time: float = 0.1
    derivative_on_measurement: bool = True
    derivative_filter_tau: float = 0.0

    def __post_init__(self) -> None:
        """Validate gains and the fixed controller sample time."""
        for name, value in (("kp", self.kp), ("ki", self.ki), ("kd", self.kd)):
            _require_finite(name, value, PIDConfigurationError)
            if value < 0.0:
                raise PIDConfigurationError(f"{name} must be non-negative")
        _require_finite("sample_time", self.sample_time, PIDConfigurationError)
        if self.sample_time <= 0.0:
            raise PIDConfigurationError("sample_time must be greater than zero")
        _require_finite(
            "derivative_filter_tau", self.derivative_filter_tau, PIDConfigurationError
        )
        if self.derivative_filter_tau < 0.0:
            raise PIDConfigurationError("derivative_filter_tau must be non-negative")


@dataclass(frozen=True, slots=True)
class PIDResult:
    """Observable values produced by one controller update."""

    setpoint: float
    measurement: float
    error: float
    proportional: float
    integral: float
    derivative: float
    raw_output: float
    output: float
    saturated: bool = False


class PIDController:
    """Deterministic discrete-time controller with explicit internal state."""

    def __init__(self, config: PIDConfig) -> None:
        """Create a controller from validated immutable configuration."""
        if not isinstance(config, PIDConfig):
            raise TypeError("config must be a PIDConfig instance")
        self._config = config
        self._integral = 0.0
        self._previous_error: float | None = None
        self._previous_measurement: float | None = None
        self._derivative_state = 0.0

    @property
    def config(self) -> PIDConfig:
        """Return the controller configuration."""
        return self._config

    def update(self, setpoint: float, measurement: float) -> PIDResult:
        """Calculate proportional control output for one fixed-time step."""
        _require_finite("setpoint", setpoint, PIDInputError)
        _require_finite("measurement", measurement, PIDInputError)
        error = setpoint - measurement
        proportional = self._config.kp * error
        self._integral += self._config.ki * error * self._config.sample_time
        derivative = self._calculate_derivative(error, measurement)
        raw_output = proportional + self._integral + derivative
        self._previous_error = error
        self._previous_measurement = measurement
        return PIDResult(
            setpoint=setpoint,
            measurement=measurement,
            error=error,
            proportional=proportional,
            integral=self._integral,
            derivative=derivative,
            raw_output=raw_output,
            output=raw_output,
        )

    def _calculate_derivative(self, error: float, measurement: float) -> float:
        """Calculate optional filtered derivative contribution."""
        if self._previous_error is None or self._previous_measurement is None:
            return 0.0

        if self._config.derivative_on_measurement:
            rate = -(
                measurement - self._previous_measurement
            ) / self._config.sample_time
        else:
            rate = (error - self._previous_error) / self._config.sample_time

        if self._config.derivative_filter_tau > 0.0:
            alpha = self._config.sample_time / (
                self._config.derivative_filter_tau + self._config.sample_time
            )
            self._derivative_state += alpha * (rate - self._derivative_state)
        else:
            self._derivative_state = rate
        return self._config.kd * self._derivative_state

    def reset(self) -> None:
        """Reset controller state.

        This clears accumulated integral action and makes the next result
        equivalent to a new controller with the same configuration.
        """
        self._integral = 0.0
        self._previous_error = None
        self._previous_measurement = None
        self._derivative_state = 0.0
