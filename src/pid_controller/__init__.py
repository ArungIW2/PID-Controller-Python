"""Educational feedback-control components implemented in Python."""

from pid_controller.controller import (
    AntiWindupMode,
    OperatingMode,
    PIDConfig,
    PIDController,
    PIDResult,
)
from pid_controller.exceptions import PIDConfigurationError, PIDInputError
from pid_controller.experiments import (
    ExperimentRun,
    ExperimentScenario,
    export_experiment,
    run_controller_comparison,
    run_gain_sweep,
)
from pid_controller.metrics import ResponseMetrics, calculate_metrics
from pid_controller.models import FirstOrderProcess, FirstOrderProcessConfig
from pid_controller.simulation import (
    SetpointChange,
    SimulationRecord,
    SimulationResult,
    run_closed_loop,
)
from pid_controller.tuning import (
    TuningCandidate,
    TuningWeights,
    evaluate_robustness,
    parameter_sweep,
    score_metrics,
    ziegler_nichols_closed_loop,
)
from pid_controller.visualization import plot_comparison, plot_simulation

__all__ = [
    "AntiWindupMode",
    "OperatingMode",
    "PIDConfig",
    "PIDConfigurationError",
    "PIDController",
    "PIDInputError",
    "PIDResult",
    "ResponseMetrics",
    "FirstOrderProcess",
    "FirstOrderProcessConfig",
    "ExperimentRun",
    "ExperimentScenario",
    "SetpointChange",
    "SimulationRecord",
    "SimulationResult",
    "TuningCandidate",
    "TuningWeights",
    "__version__",
    "calculate_metrics",
    "export_experiment",
    "evaluate_robustness",
    "parameter_sweep",
    "plot_comparison",
    "plot_simulation",
    "run_closed_loop",
    "run_controller_comparison",
    "run_gain_sweep",
    "score_metrics",
    "ziegler_nichols_closed_loop",
]

__version__ = "1.0.0"
