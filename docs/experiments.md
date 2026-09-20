# Experiments and Result Interpretation

## Reproducibility contract

Every experiment declares gains, sample time, duration, setpoint, process gain,
and process time constant. Each run starts with a new controller and process
instance. No randomness is used by the current mathematical model.

## Proportional gain

The proportional experiment varies only `Kp`. Increasing `Kp` generally speeds
the response and reduces proportional offset, but can increase controller
effort, overshoot, or saturation depending on constraints.

## Integral gain

The integral experiment varies only `Ki`. Integral action reduces steady-state
error, while excessive integral action can increase overshoot and settling time.
Output-limited experiments expose the effect of anti-windup.

## Derivative gain

The derivative experiment varies only `Kd` with derivative-on-measurement and a
declared filter. Derivative action can improve damping, but fast measurement
changes make an unfiltered derivative sensitive.

## P, PI, and PID comparison

The curated comparison uses the same stable first-order model and command for
all controllers. Its plot and summary table can be regenerated with:

```bash
python scripts/generate_release_assets.py
```

Metric values describe this mathematical scenario only. They are evidence that
the software behaves consistently under its declared equations, not proof of
performance on industrial equipment.

## Metric definitions

- Rise time: time from 10% to 90% normalized progress.
- Settling time: first sample after the final 2% tolerance violation.
- Overshoot: excursion beyond target divided by commanded step amplitude.
- Peak value: maximum for positive steps, minimum for negative steps.
- Steady-state error: final setpoint minus final measurement.
- IAE: trapezoidal integral of absolute error.
- ISE: trapezoidal integral of squared error.

For multiple setpoint steps, metric calculation requires segmentation. Mixing
several commands into one metric value would be ambiguous.

