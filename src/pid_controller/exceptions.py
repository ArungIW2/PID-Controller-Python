"""Domain-specific exceptions for controller validation."""


class PIDConfigurationError(ValueError):
    """Raised when PID configuration values are invalid."""


class PIDInputError(ValueError):
    """Raised when a runtime controller input is invalid."""

