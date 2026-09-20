# Requirements and Scope

## 1. Purpose

Build a software-only PID control learning project that demonstrates control
engineering fundamentals and disciplined Python engineering. The repository is
intended for study and portfolio review, not production control.

## 2. In scope

- A controller implemented from first principles in Python.
- P, PI, optional PD, and PID configurations using one controller core.
- Configurable gains, sample time, setpoint, and output limits.
- Reset, setpoint changes, input validation, and explicit errors.
- Integral anti-windup, derivative filtering, and derivative-on-measurement.
- A deterministic first-order mathematical process model.
- Closed-loop simulation, structured data logging, plots, and CSV export.
- Response metrics: rise time, settling time, peak, overshoot,
  steady-state error, IAE, and ISE.
- Reproducible gain experiments and appropriate tuning comparisons.
- Unit and integration tests with continuous integration.

## 3. Out of scope

- Physical PLCs, sensors, actuators, motors, microcontrollers, or other hardware.
- Hardware emulation or claims of hardware-in-the-loop validation.
- A digital twin of a named industrial machine.
- Safety instrumented functions or safety certification.
- Deterministic real-time scheduling and vendor-specific PLC runtime behavior.
- Production deployment to control physical equipment.

## 4. Mathematical model decision

The initial process is a stable first-order linear model:

\[
\tau \frac{dy}{dt} + y = Ku
\]

or equivalently:

\[
\frac{dy}{dt} = \frac{Ku-y}{\tau}
\]

It will be integrated with forward Euler using the controller sample time:

\[
y_{k+1} = y_k + \Delta t\frac{Ku_k-y_k}{\tau}
\]

### Why this model

- Its parameters have understandable effects: process gain \(K\) and time
  constant \(\tau\).
- Its open-loop behavior is stable and analytically understandable.
- It is sufficient to expose proportional offset, integral elimination of
  offset, saturation, and common transient-response trade-offs.
- It keeps the learning focus on the controller and measurement methodology.

### Known limitations

The basic model has no transport delay, measurement noise, nonlinear response,
or actuator dynamics. These may be introduced later only as explicit software
scenarios. A delay-enhanced model may be used for selected tuning studies if a
method requires sustained-oscillation behavior that the basic plant does not
show cleanly.

## 5. Functional requirements

| ID | Requirement | Verification |
| --- | --- | --- |
| FR-01 | Calculate error as setpoint minus process variable. | Unit test |
| FR-02 | Calculate proportional, integral, and derivative terms. | Unit tests |
| FR-03 | Accept finite gains and a strictly positive sample time. | Validation tests |
| FR-04 | Enforce ordered optional output limits. | Unit tests |
| FR-05 | Prevent or unwind integral accumulation under saturation. | Scenario tests |
| FR-06 | Reset all controller state deterministically. | Unit test |
| FR-07 | Support setpoint changes during a run. | Integration test |
| FR-08 | Log time, SP, PV, error, P/I/D terms, raw output, and limited output. | Schema test |
| FR-09 | Simulate the documented first-order equation reproducibly. | Model tests |
| FR-10 | Calculate documented response metrics. | Unit and scenario tests |
| FR-11 | Generate comparison plots without coupling plotting to control logic. | Integration test |
| FR-12 | Reject non-finite inputs with clear exceptions. | Edge-case tests |

## 6. Non-functional requirements

- Python 3.10 or newer, type hints, and public API docstrings.
- Deterministic default experiments and explicit configuration.
- Separation of control logic, modelling, simulation, analysis, and plotting.
- Automated linting, strict type checking, tests, and coverage reporting.
- Generated results must be traceable to a scenario and parameter set.
- Documentation must distinguish mathematical results from physical validation.

## 7. Phase 1 acceptance criteria

- The package installs from `pyproject.toml`.
- The source-layout package imports and exposes a semantic version.
- Automated test, lint, and type-check commands are configured.
- CI covers supported Python versions.
- Scope, model decision, architecture, and ten-phase roadmap are documented.
- No untested or misleading PID implementation is shipped in the foundation.

