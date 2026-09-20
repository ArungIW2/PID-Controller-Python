"""Transparent PID tuning and process-uncertainty evaluation utilities."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from itertools import product
from math import isfinite
from typing import Literal

from pid_controller.controller import PIDConfig, PIDController
from pid_controller.exceptions import PIDInputError
from pid_controller.experiments import ExperimentRun, ExperimentScenario
from pid_controller.metrics import ResponseMetrics, calculate_metrics
from pid_controller.models import FirstOrderProcess
from pid_controller.simulation import run_closed_loop


@dataclass(frozen=True, slots=True)
class TuningWeights:
    """Weights for an explicit multi-objective response cost."""

    iae: float = 1.0
    ise: float = 0.1
    overshoot: float = 0.1
    settling_time: float = 0.1
    steady_state_error: float = 1.0
    missing_metric_penalty: float = 1_000.0

    def __post_init__(self) -> None:
        """Require every cost coefficient to be finite and non-negative."""
        for name in self.__dataclass_fields__:
            coefficient = getattr(self, name)
            if not isfinite(coefficient) or coefficient < 0.0:
                raise PIDInputError(
                    f"{name} weight must be finite and non-negative"
                )


@dataclass(frozen=True, slots=True)
class TuningCandidate:
    """A tested configuration with its response metrics and scalar score."""

    config: PIDConfig
    metrics: ResponseMetrics
    score: float


def score_metrics(metrics: ResponseMetrics, weights: TuningWeights) -> float:
    """Calculate the documented cost used to rank parameter candidates."""
    overshoot = (
        weights.missing_metric_penalty
        if metrics.overshoot_percent is None
        else metrics.overshoot_percent
    )
    settling = (
        weights.missing_metric_penalty
        if metrics.settling_time is None
        else metrics.settling_time
    )
    return (
        weights.iae * metrics.iae
        + weights.ise * metrics.ise
        + weights.overshoot * overshoot
        + weights.settling_time * settling
        + weights.steady_state_error * abs(metrics.steady_state_error)
    )


def parameter_sweep(
    *,
    kp_values: Sequence[float],
    ki_values: Sequence[float],
    kd_values: Sequence[float],
    base_config: PIDConfig,
    scenario: ExperimentScenario,
    weights: TuningWeights | None = None,
    max_candidates: int = 10_000,
) -> tuple[TuningCandidate, ...]:
    """Evaluate a bounded Cartesian gain grid and return best score first."""
    active_weights = weights or TuningWeights()
    dimensions = (kp_values, ki_values, kd_values)
    if any(not values for values in dimensions):
        raise PIDInputError("every gain dimension requires at least one value")
    candidate_count = len(kp_values) * len(ki_values) * len(kd_values)
    if max_candidates <= 0 or candidate_count > max_candidates:
        raise PIDInputError("parameter grid exceeds the configured candidate limit")
    candidates: list[TuningCandidate] = []
    for kp, ki, kd in product(kp_values, ki_values, kd_values):
        if any(not isfinite(value) or value < 0.0 for value in (kp, ki, kd)):
            raise PIDInputError("candidate gains must be finite and non-negative")
        config = replace(base_config, kp=kp, ki=ki, kd=kd)
        simulation = run_closed_loop(
            PIDController(config),
            FirstOrderProcess(scenario.process),
            duration=scenario.duration,
            setpoint=scenario.setpoint,
        )
        metrics = calculate_metrics(simulation)
        candidates.append(
            TuningCandidate(config, metrics, score_metrics(metrics, active_weights))
        )
    return tuple(sorted(candidates, key=lambda candidate: candidate.score))


def ziegler_nichols_closed_loop(
    ultimate_gain: float,
    ultimate_period: float,
    *,
    controller_type: Literal["P", "PI", "PID"],
    sample_time: float,
) -> PIDConfig:
    """Return classic closed-loop Ziegler–Nichols gains from measured Ku and Pu.

    This function does not estimate the ultimate values. Supplying them asserts
    that a suitable sustained-oscillation study has already been performed.
    """
    if (
        not isfinite(ultimate_gain)
        or ultimate_gain <= 0.0
        or not isfinite(ultimate_period)
        or ultimate_period <= 0.0
    ):
        raise PIDInputError("ultimate gain and period must be finite and positive")
    if controller_type == "P":
        return PIDConfig(kp=0.5 * ultimate_gain, sample_time=sample_time)
    if controller_type == "PI":
        kp = 0.45 * ultimate_gain
        integral_time = ultimate_period / 1.2
        return PIDConfig(kp=kp, ki=kp / integral_time, sample_time=sample_time)
    if controller_type == "PID":
        kp = 0.6 * ultimate_gain
        integral_time = 0.5 * ultimate_period
        derivative_time = 0.125 * ultimate_period
        return PIDConfig(
            kp=kp,
            ki=kp / integral_time,
            kd=kp * derivative_time,
            sample_time=sample_time,
        )
    raise PIDInputError("controller_type must be P, PI, or PID")


def evaluate_robustness(
    config: PIDConfig,
    scenarios: Mapping[str, ExperimentScenario],
) -> dict[str, ExperimentRun]:
    """Evaluate one tuning against named mathematical process variations."""
    if not scenarios:
        raise PIDInputError("robustness evaluation requires scenarios")
    runs: dict[str, ExperimentRun] = {}
    for name, scenario in scenarios.items():
        simulation = run_closed_loop(
            PIDController(config),
            FirstOrderProcess(scenario.process),
            duration=scenario.duration,
            setpoint=scenario.setpoint,
        )
        runs[name] = ExperimentRun(config, simulation, calculate_metrics(simulation))
    return runs
