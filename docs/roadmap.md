# Development Roadmap

Each phase is sized as a reviewable Git commit and includes tests for the
behavior it introduces.

## Phase 1 — Foundation and engineering contract

**Deliverable:** Installable `src`-layout package, repository conventions,
quality configuration, CI, smoke test, scope, model choice, architecture, and
roadmap.

**Exit criteria:** Install, import, lint, type-check, and test commands succeed;
no PID functionality is implied.

**Suggested commit:** `chore: establish project foundation and engineering plan`

## Phase 2 — Proportional controller core

**Deliverable:** Validated controller configuration, typed controller result,
error calculation, proportional term, deterministic update, and reset baseline.

**Tests:** Error sign, proportional calculation, zero gain, invalid values, and
public API behavior.

**Learning outcome:** Feedback error, proportional action, stateless versus
stateful controller responsibilities.

**Suggested commit:** `feat: implement validated proportional controller core`

## Phase 3 — Integral action and PI control

**Deliverable:** Discrete integral accumulation, PI operation, reset of integral
state, and integral observability. Saturation protection is deliberately deferred
until limits are introduced.

**Tests:** Accumulation over multiple samples, sample-time effect, reset,
negative error, and disabled integral action.

**Learning outcome:** Integral state, removal of steady-state offset, and the
mechanism behind windup.

**Suggested commit:** `feat: add integral action and PI controller tests`

## Phase 4 — Derivative action and complete PID

**Deliverable:** Derivative term, first-sample policy, PD/PID configurations,
derivative-on-measurement option, and a simple low-pass derivative filter.

**Tests:** Difference quotient, derivative kick behavior, filter state, reset,
and P/PI/PD/PID mode equivalence through zero gains.

**Learning outcome:** Damping, noise sensitivity, derivative kick, and filtered
discrete derivatives.

**Suggested commit:** `feat: complete PID with configurable derivative behavior`

## Phase 5 — Process model and closed-loop runner

**Deliverable:** First-order model, fixed-step integration, setpoint schedules,
closed-loop orchestration, and typed time-series records.

**Tests:** Known model transitions, deterministic run length, signal ordering,
setpoint changes, and reproducibility.

**Learning outcome:** Plant dynamics, feedback-loop sequencing, discretization,
and the distinction between controller and process.

**Suggested commit:** `feat: add first-order model and closed-loop simulation`

## Phase 6 — Saturation, anti-windup, and operating behavior

**Deliverable:** Output limits, conditional-integration or back-calculation
anti-windup, integral bounds, and explicit manual/automatic mode if justified.
Bumpless transfer is included only with operating modes.

**Tests:** Upper/lower saturation, recovery from saturation, disabled limits,
invalid bounds, and transition behavior.

**Learning outcome:** Actuator constraints, windup, state management, and safe
mode transitions.

**Suggested commit:** `feat: add output limiting and anti-windup protection`

## Phase 7 — Metrics, data export, and visualization

**Deliverable:** Rise time, settling time, overshoot, peak, steady-state error,
IAE, ISE, Pandas export, and plots for SP/PV, output, error, and term breakdown.

**Tests:** Synthetic traces with known answers, non-reaching responses, negative
or zero setpoints, irregular invalid input, and headless plot creation.

**Learning outcome:** Quantitative response analysis and precise metric
definitions instead of visual guesswork.

**Suggested commit:** `feat: add response metrics logging and visualizations`

## Phase 8 — Reproducible experiments

**Deliverable:** Parameterized Kp, Ki, Kd, and P-vs-PI-vs-PID experiments;
machine-readable configurations; deterministic CSV/PNG outputs; concise
engineering observations.

**Tests:** Experiment configuration validation, repeatability, and output schema.

**Learning outcome:** Isolating variables, fair controller comparison, and
evidence-based interpretation.

**Suggested commit:** `feat: add reproducible PID gain experiments`

## Phase 9 — Tuning and robustness study

**Deliverable:** Manual tuning guide, bounded parameter sweep with an explicit
cost function, and a justified rule-based method. Ziegler–Nichols is used only
for a scenario that satisfies its assumptions; otherwise its exclusion is
documented. Selected disturbance/noise sensitivity cases test robustness.

**Tests:** Search bounds, deterministic ranking, invalid candidates, cost
calculation, and regression cases for chosen tunings.

**Learning outcome:** Tuning trade-offs, objective-function bias, model
dependence, and robustness limits.

**Suggested commit:** `feat: add tuning workflows and robustness analysis`

## Phase 10 — Portfolio release

**Deliverable:** Final API review, complete recruiter-friendly README, theory and
tuning guides, curated result figures, changelog, release checklist, and a
tag-ready version. Coverage, lint, typing, and CI are tightened based on the
finished surface.

**Tests:** Full suite, documentation command verification, packaging build, and
clean-environment installation.

**Learning outcome:** Technical communication, traceability, release discipline,
and honest presentation of engineering evidence.

**Suggested commit:** `docs: prepare validated portfolio release`

