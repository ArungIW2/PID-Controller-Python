"""Run the derivative-gain experiment."""

from pid_controller import (
    ExperimentScenario,
    PIDConfig,
    export_experiment,
    run_gain_sweep,
)

runs = run_gain_sweep(
    "kd",
    (0.0, 0.05, 0.1, 0.2),
    base_config=PIDConfig(
        kp=2.0,
        ki=0.5,
        sample_time=0.05,
        derivative_filter_tau=0.1,
    ),
    scenario=ExperimentScenario(),
)
export_experiment(runs, "results/derivative", name="derivative-gain")
