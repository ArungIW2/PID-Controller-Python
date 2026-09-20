"""Run the proportional-gain experiment."""

from pid_controller import (
    ExperimentScenario,
    PIDConfig,
    export_experiment,
    run_gain_sweep,
)

runs = run_gain_sweep(
    "kp",
    (0.5, 1.0, 2.0, 4.0),
    base_config=PIDConfig(kp=1.0, sample_time=0.05),
    scenario=ExperimentScenario(),
)
export_experiment(runs, "results/proportional", name="proportional-gain")
