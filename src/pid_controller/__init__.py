"""Educational feedback-control components implemented in Python."""

from pid_controller.controller import (
    AntiWindupMode,
    OperatingMode,
    PIDConfig,
    PIDController,
    PIDResult,
)
from pid_controller.exceptions import PIDConfigurationError, PIDInputError
from pid_controller.models import FirstOrderProcess, FirstOrderProcessConfig
from pid_controller.simulation import (
    SetpointChange,
    SimulationRecord,
    SimulationResult,
    run_closed_loop,
)

__all__ = [
    "AntiWindupMode",
    "OperatingMode",
    "PIDConfig",
    "PIDConfigurationError",
    "PIDController",
    "PIDInputError",
    "PIDResult",
    "FirstOrderProcess",
    "FirstOrderProcessConfig",
    "SetpointChange",
    "SimulationRecord",
    "SimulationResult",
    "__version__",
    "run_closed_loop",
]

__version__ = "0.1.0"
