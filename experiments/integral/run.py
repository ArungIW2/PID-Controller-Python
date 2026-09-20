"""Run the integral-gain experiment."""

from pid_controller import (
    ExperimentScenario,
    PIDConfig,
    export_experiment,
    run_gain_sweep,
)

runs = run_gain_sweep(
    "ki",
    (0.0, 0.2, 0.5, 1.0),
    base_config=PIDConfig(kp=1.5, sample_time=0.05, output_limits=(0.0, 2.0)),
    scenario=ExperimentScenario(),
)
export_experiment(runs, "results/integral", name="integral-gain")
