"""Discrete-time PID controller implemented from first principles."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import isfinite

from pid_controller.exceptions import PIDConfigurationError, PIDInputError


def _require_finite(name: str, value: float, error_type: type[ValueError]) -> None:
    """Reject NaN and infinite values at public numeric boundaries."""
    if not isfinite(value):
        raise error_type(f"{name} must be finite, got {value!r}")


class AntiWindupMode(str, Enum):
    """Supported strategies for integral behavior during saturation."""

    NONE = "none"
    CLAMP = "clamp"
    BACK_CALCULATION = "back_calculation"


class OperatingMode(str, Enum):
    """Controller operating modes."""

    AUTOMATIC = "automatic"
    MANUAL = "manual"


@dataclass(frozen=True, slots=True)
class PIDConfig:
    """Immutable controller configuration with validated numeric boundaries."""

    kp: float
    ki: float = 0.0
    kd: float = 0.0
    sample_time: float = 0.1
    derivative_on_measurement: bool = True
    derivative_filter_tau: float = 0.0
    output_limits: tuple[float | None, float | None] = (None, None)
    integral_limits: tuple[float | None, float | None] = (None, None)
    anti_windup: AntiWindupMode = AntiWindupMode.CLAMP
    back_calculation_gain: float = 1.0

    def __post_init__(self) -> None:
        """Validate gains, timing, filtering, limits, and anti-windup settings."""
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
        self._validate_limits("output_limits", self.output_limits)
        self._validate_limits("integral_limits", self.integral_limits)
        if not isinstance(self.anti_windup, AntiWindupMode):
            raise PIDConfigurationError("anti_windup must be an AntiWindupMode")
        _require_finite(
            "back_calculation_gain",
            self.back_calculation_gain,
            PIDConfigurationError,
        )
        if self.back_calculation_gain < 0.0:
            raise PIDConfigurationError("back_calculation_gain must be non-negative")

    @staticmethod
    def _validate_limits(
        name: str, limits: tuple[float | None, float | None]
    ) -> None:
        """Validate an optional lower/upper limit pair."""
        if not isinstance(limits, tuple) or len(limits) != 2:
            raise PIDConfigurationError(f"{name} must be a (lower, upper) tuple")
        lower, upper = limits
        for value in limits:
            if value is not None and not isfinite(value):
                raise PIDConfigurationError(f"{name} values must be finite")
        if lower is not None and upper is not None and lower >= upper:
            raise PIDConfigurationError(f"{name} lower limit must be below upper")


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
    """Deterministic discrete-time PID controller with explicit state."""

    def __init__(self, config: PIDConfig) -> None:
        """Create a controller from validated immutable configuration."""
        if not isinstance(config, PIDConfig):
            raise TypeError("config must be a PIDConfig instance")
        self._config = config
        self._integral = 0.0
        self._previous_error: float | None = None
        self._previous_measurement: float | None = None
        self._derivative_state = 0.0
        self._mode = OperatingMode.AUTOMATIC
        self._manual_raw_output = 0.0
        self._manual_output = 0.0
        self._last_output = 0.0
        self._bumpless_pending = False

    @property
    def config(self) -> PIDConfig:
        """Return the controller configuration."""
        return self._config

    @property
    def mode(self) -> OperatingMode:
        """Return the current controller operating mode."""
        return self._mode

    def update(self, setpoint: float, measurement: float) -> PIDResult:
        """Calculate one deterministic discrete-time control update."""
        _require_finite("setpoint", setpoint, PIDInputError)
        _require_finite("measurement", measurement, PIDInputError)
        error = setpoint - measurement
        proportional = self._config.kp * error
        derivative = self._calculate_derivative(error, measurement)
        self._previous_error = error
        self._previous_measurement = measurement

        if self._mode is OperatingMode.MANUAL:
            return PIDResult(
                setpoint=setpoint,
                measurement=measurement,
                error=error,
                proportional=proportional,
                integral=self._integral,
                derivative=derivative,
                raw_output=self._manual_raw_output,
                output=self._manual_output,
                saturated=self._manual_raw_output != self._manual_output,
            )

        previous_integral = self._integral
        if self._bumpless_pending:
            candidate_integral = self._limit_value(
                self._last_output - proportional - derivative,
                self._config.integral_limits,
            )
            self._bumpless_pending = False
        else:
            candidate_integral = self._limit_value(
                self._integral
                + self._config.ki * error * self._config.sample_time,
                self._config.integral_limits,
            )

        raw_output = proportional + candidate_integral + derivative
        output = self._limit_value(raw_output, self._config.output_limits)
        if self._config.anti_windup is AntiWindupMode.CLAMP:
            pushes_high = output < raw_output and error > 0.0
            pushes_low = output > raw_output and error < 0.0
            self._integral = (
                previous_integral if pushes_high or pushes_low else candidate_integral
            )
        elif self._config.anti_windup is AntiWindupMode.BACK_CALCULATION:
            self._integral = self._limit_value(
                candidate_integral
                + self._config.back_calculation_gain
                * (output - raw_output)
                * self._config.sample_time,
                self._config.integral_limits,
            )
        else:
            self._integral = candidate_integral

        raw_output = proportional + self._integral + derivative
        output = self._limit_value(raw_output, self._config.output_limits)
        self._last_output = output
        return PIDResult(
            setpoint=setpoint,
            measurement=measurement,
            error=error,
            proportional=proportional,
            integral=self._integral,
            derivative=derivative,
            raw_output=raw_output,
            output=output,
            saturated=raw_output != output,
        )

    @staticmethod
    def _limit_value(
        value: float, limits: tuple[float | None, float | None]
    ) -> float:
        """Apply optional lower and upper limits to a scalar value."""
        lower, upper = limits
        if lower is not None and value < lower:
            return lower
        if upper is not None and value > upper:
            return upper
        return value

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

    def set_mode(
        self, mode: OperatingMode, *, manual_output: float | None = None
    ) -> None:
        """Select manual or automatic operation with bumpless return to auto."""
        if not isinstance(mode, OperatingMode):
            raise PIDInputError("mode must be an OperatingMode")
        if mode is OperatingMode.MANUAL:
            if manual_output is None or not isfinite(manual_output):
                raise PIDInputError("manual mode requires a finite manual_output")
            self._manual_raw_output = manual_output
            self._manual_output = self._limit_value(
                manual_output, self._config.output_limits
            )
            self._last_output = self._manual_output
            self._mode = mode
            return
        if self._mode is OperatingMode.MANUAL:
            self._bumpless_pending = True
        self._mode = mode

    def reset(self) -> None:
        """Reset dynamic, operating-mode, and transfer state."""
        self._integral = 0.0
        self._previous_error = None
        self._previous_measurement = None
        self._derivative_state = 0.0
        self._mode = OperatingMode.AUTOMATIC
        self._manual_raw_output = 0.0
        self._manual_output = 0.0
        self._last_output = 0.0
        self._bumpless_pending = False
