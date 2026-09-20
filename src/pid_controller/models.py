"""Mathematical process models used for software-only control studies."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from typing import Protocol

from pid_controller.exceptions import PIDConfigurationError, PIDInputError


class ProcessModel(Protocol):
    """Protocol implemented by deterministic scalar process models."""

    @property
    def output(self) -> float:
        """Return the current process output."""

    def step(self, control_input: float, sample_time: float) -> float:
        """Advance the model by one sample and return its new output."""

    def reset(self, value: float | None = None) -> None:
        """Reset process state to an optional output value."""


@dataclass(frozen=True, slots=True)
class FirstOrderProcessConfig:
    """Parameters for ``dy/dt = (gain * u - y) / time_constant``."""

    gain: float = 1.0
    time_constant: float = 1.0
    initial_value: float = 0.0

    def __post_init__(self) -> None:
        """Validate finite parameters and a positive time constant."""
        for name, value in (
            ("gain", self.gain),
            ("time_constant", self.time_constant),
            ("initial_value", self.initial_value),
        ):
            if not isfinite(value):
                raise PIDConfigurationError(f"{name} must be finite")
        if self.time_constant <= 0.0:
            raise PIDConfigurationError("time_constant must be greater than zero")


class FirstOrderProcess:
    """Forward-Euler implementation of a stable first-order process equation."""

    def __init__(self, config: FirstOrderProcessConfig | None = None) -> None:
        """Create the mathematical model from validated parameters."""
        self.config = config or FirstOrderProcessConfig()
        self._output = self.config.initial_value

    @property
    def output(self) -> float:
        """Return the current model output."""
        return self._output

    def step(self, control_input: float, sample_time: float) -> float:
        """Advance the documented differential equation using forward Euler."""
        if not isfinite(control_input):
            raise PIDInputError("control_input must be finite")
        if not isfinite(sample_time) or sample_time <= 0.0:
            raise PIDInputError("sample_time must be finite and greater than zero")
        derivative = (
            self.config.gain * control_input - self._output
        ) / self.config.time_constant
        self._output += sample_time * derivative
        return self._output

    def reset(self, value: float | None = None) -> None:
        """Reset state to the configured initial value or an explicit value."""
        new_value = self.config.initial_value if value is None else value
        if not isfinite(new_value):
            raise PIDInputError("reset value must be finite")
        self._output = new_value

