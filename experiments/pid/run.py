"""Run the P, PI, and PID comparison experiment."""

from pid_controller import (
    ExperimentScenario,
    PIDConfig,
    export_experiment,
    run_controller_comparison,
)

sample_time = 0.05
runs = run_controller_comparison(
    {
        "P": PIDConfig(kp=2.0, sample_time=sample_time),
        "PI": PIDConfig(kp=2.0, ki=0.5, sample_time=sample_time),
        "PID": PIDConfig(
            kp=2.0,
            ki=0.5,
            kd=0.1,
            sample_time=sample_time,
            derivative_filter_tau=0.1,
        ),
    },
    scenario=ExperimentScenario(),
)
export_experiment(runs, "results/pid", name="controller-comparison")
