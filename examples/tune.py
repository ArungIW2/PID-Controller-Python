"""Find and evaluate a PID tuning on mathematical process variations."""

from pid_controller import (
    ExperimentScenario,
    FirstOrderProcessConfig,
    PIDConfig,
    evaluate_robustness,
    parameter_sweep,
)

sample_time = 0.05
scenario = ExperimentScenario(duration=20.0, setpoint=1.0)
candidates = parameter_sweep(
    kp_values=(1.0, 2.0, 3.0),
    ki_values=(0.2, 0.5, 0.8),
    kd_values=(0.0, 0.1),
    base_config=PIDConfig(kp=1.0, sample_time=sample_time),
    scenario=scenario,
)
best = candidates[0]
robustness = evaluate_robustness(
    best.config,
    {
        "nominal": scenario,
        "higher-gain": ExperimentScenario(
            duration=20.0,
            setpoint=1.0,
            process=FirstOrderProcessConfig(gain=1.2, time_constant=1.0),
        ),
        "slower": ExperimentScenario(
            duration=20.0,
            setpoint=1.0,
            process=FirstOrderProcessConfig(gain=1.0, time_constant=1.5),
        ),
    },
)

print("Best configuration:", best.config)
for name, run in robustness.items():
    print(name, run.metrics)

