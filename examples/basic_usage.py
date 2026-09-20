"""Run and inspect a basic software-only PID response."""

from pid_controller import (
    FirstOrderProcess,
    PIDConfig,
    PIDController,
    calculate_metrics,
    run_closed_loop,
)

controller = PIDController(
    PIDConfig(
        kp=2.0,
        ki=0.5,
        kd=0.1,
        sample_time=0.05,
        output_limits=(0.0, 2.0),
        derivative_filter_tau=0.1,
    )
)
result = run_closed_loop(
    controller,
    FirstOrderProcess(),
    duration=20.0,
    setpoint=1.0,
)

print(calculate_metrics(result))

