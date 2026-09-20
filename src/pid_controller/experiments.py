"""Reproducible gain sweeps and controller-comparison experiments."""

from __future__ import annotations

import json
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass, replace
from math import isfinite
from pathlib import Path
from typing import Literal

import matplotlib.pyplot as plt
import pandas as pd

from pid_controller.controller import PIDConfig, PIDController
from pid_controller.exceptions import PIDInputError
from pid_controller.metrics import ResponseMetrics, calculate_metrics
from pid_controller.models import FirstOrderProcess, FirstOrderProcessConfig
from pid_controller.simulation import SimulationResult, run_closed_loop
from pid_controller.visualization import plot_comparison

GainName = Literal["kp", "ki", "kd"]


@dataclass(frozen=True, slots=True)
class ExperimentScenario:
    """Shared process and command settings for a fair comparison."""

    duration: float = 20.0
    setpoint: float = 1.0
    process: FirstOrderProcessConfig = FirstOrderProcessConfig()

    def __post_init__(self) -> None:
        """Validate experiment time and command values."""
        if not isfinite(self.duration) or self.duration <= 0.0:
            raise PIDInputError("experiment duration must be finite and positive")
        if not isfinite(self.setpoint):
            raise PIDInputError("experiment setpoint must be finite")


@dataclass(frozen=True, slots=True)
class ExperimentRun:
    """Controller configuration, response, and metrics for one run."""

    controller: PIDConfig
    simulation: SimulationResult
    metrics: ResponseMetrics


def _execute(config: PIDConfig, scenario: ExperimentScenario) -> ExperimentRun:
    """Execute one controller under a fresh mathematical process state."""
    simulation = run_closed_loop(
        PIDController(config),
        FirstOrderProcess(scenario.process),
        duration=scenario.duration,
        setpoint=scenario.setpoint,
    )
    return ExperimentRun(config, simulation, calculate_metrics(simulation))


def run_gain_sweep(
    parameter: GainName,
    values: Sequence[float],
    *,
    base_config: PIDConfig,
    scenario: ExperimentScenario,
) -> dict[str, ExperimentRun]:
    """Vary exactly one gain while preserving model and all other settings."""
    if parameter not in ("kp", "ki", "kd"):
        raise PIDInputError("parameter must be one of: kp, ki, kd")
    if not values:
        raise PIDInputError("gain sweep requires at least one value")
    runs: dict[str, ExperimentRun] = {}
    for value in values:
        if not isfinite(value) or value < 0.0:
            raise PIDInputError("sweep gains must be finite and non-negative")
        if parameter == "kp":
            config = replace(base_config, kp=value)
        elif parameter == "ki":
            config = replace(base_config, ki=value)
        else:
            config = replace(base_config, kd=value)
        runs[f"{parameter}={value:g}"] = _execute(config, scenario)
    return runs


def run_controller_comparison(
    controllers: Mapping[str, PIDConfig],
    *,
    scenario: ExperimentScenario,
) -> dict[str, ExperimentRun]:
    """Run named P/PI/PID configurations under the same scenario."""
    if not controllers:
        raise PIDInputError("controller comparison requires configurations")
    return {name: _execute(config, scenario) for name, config in controllers.items()}


def export_experiment(
    runs: Mapping[str, ExperimentRun],
    directory: str | Path,
    *,
    name: str,
) -> tuple[Path, ...]:
    """Export traces, metrics, manifest, and a comparison plot."""
    if not runs:
        raise PIDInputError("cannot export an empty experiment")
    output_directory = Path(directory)
    output_directory.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    summary_rows: list[dict[str, float | str | None]] = []
    manifest: dict[str, dict[str, float | str | bool | tuple[object, ...]]] = {}

    for label, run in runs.items():
        safe_label = label.replace("=", "-").replace(".", "_").replace(" ", "-")
        trace_path = output_directory / f"{name}-{safe_label}.csv"
        run.simulation.to_csv(trace_path)
        paths.append(trace_path)
        summary_rows.append({"controller": label, **asdict(run.metrics)})
        manifest[label] = {
            "kp": run.controller.kp,
            "ki": run.controller.ki,
            "kd": run.controller.kd,
            "sample_time": run.controller.sample_time,
            "derivative_on_measurement": run.controller.derivative_on_measurement,
            "derivative_filter_tau": run.controller.derivative_filter_tau,
            "output_limits": run.controller.output_limits,
            "anti_windup": run.controller.anti_windup.value,
        }

    summary_path = output_directory / f"{name}-summary.csv"
    pd.DataFrame(summary_rows).to_csv(summary_path, index=False)
    paths.append(summary_path)
    manifest_path = output_directory / f"{name}-manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    paths.append(manifest_path)

    figure = plot_comparison(
        {label: run.simulation for label, run in runs.items()}, title=name
    )
    plot_path = output_directory / f"{name}-comparison.png"
    figure.savefig(plot_path, dpi=150)
    plt.close(figure)
    paths.append(plot_path)
    return tuple(paths)

