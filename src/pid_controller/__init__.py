"""Educational feedback-control components implemented in Python."""

from pid_controller.controller import PIDConfig, PIDController, PIDResult
from pid_controller.exceptions import PIDConfigurationError, PIDInputError

__all__ = [
    "PIDConfig",
    "PIDConfigurationError",
    "PIDController",
    "PIDInputError",
    "PIDResult",
    "__version__",
]

__version__ = "0.1.0"
