"""Tests for reproducible controller experiments."""

import json
from pathlib import Path

import pytest

from pid_controller import (
    ExperimentScenario,
    FirstOrderProcessConfig,
    PIDConfig,
    export_experiment,
    run_controller_comparison,
    run_gain_sweep,
)
from pid_controller.exceptions import PIDInputError


@pytest.fixture
def scenario() -> ExperimentScenario:
    """Return a short, stable test scenario."""
    return ExperimentScenario(
        duration=1.0,
        setpoint=1.0,
        process=FirstOrderProcessConfig(gain=1.0, time_constant=1.0),
    )


def test_gain_sweep_changes_only_selected_gain(
    scenario: ExperimentScenario,
) -> None:
    """Sweep runs should preserve base configuration except selected gain."""
    runs = run_gain_sweep(
        "kp",
        (1.0, 2.0),
        base_config=PIDConfig(kp=0.0, ki=0.3, sample_time=0.1),
        scenario=scenario,
    )
    assert tuple(runs) == ("kp=1", "kp=2")
    assert runs["kp=2"].controller.kp == 2.0
    assert runs["kp=2"].controller.ki == 0.3
    assert runs["kp=1"].simulation.records == run_gain_sweep(
        "kp",
        (1.0,),
        base_config=PIDConfig(kp=0.0, ki=0.3, sample_time=0.1),
        scenario=scenario,
    )["kp=1"].simulation.records


@pytest.mark.parametrize("parameter", ["ki", "kd"])
def test_each_supported_gain_can_be_swept(
    parameter: str, scenario: ExperimentScenario
) -> None:
    """Integral and derivative branches should create valid configurations."""
    runs = run_gain_sweep(
        parameter,  # type: ignore[arg-type]
        (0.1,),
        base_config=PIDConfig(kp=1.0, sample_time=0.1),
        scenario=scenario,
    )
    assert getattr(next(iter(runs.values())).controller, parameter) == 0.1


@pytest.mark.parametrize(
    ("parameter", "values"),
    [("bad", (1.0,)), ("kp", ()), ("kp", (-1.0,)), ("kp", (float("inf"),))],
)
def test_invalid_sweep_is_rejected(
    parameter: str, values: tuple[float, ...], scenario: ExperimentScenario
) -> None:
    """Sweep dimensions and candidates must be explicit and valid."""
    with pytest.raises(PIDInputError):
        run_gain_sweep(
            parameter,  # type: ignore[arg-type]
            values,
            base_config=PIDConfig(kp=1.0, sample_time=0.1),
            scenario=scenario,
        )


def test_named_controller_comparison(scenario: ExperimentScenario) -> None:
    """P and PI responses should run under the same scenario."""
    runs = run_controller_comparison(
        {
            "P": PIDConfig(kp=1.0, sample_time=0.1),
            "PI": PIDConfig(kp=1.0, ki=0.2, sample_time=0.1),
        },
        scenario=scenario,
    )
    assert set(runs) == {"P", "PI"}
    assert all(run.metrics.iae >= 0.0 for run in runs.values())


def test_empty_controller_comparison_is_rejected(
    scenario: ExperimentScenario,
) -> None:
    """A comparison must contain at least one controller."""
    with pytest.raises(PIDInputError):
        run_controller_comparison({}, scenario=scenario)


@pytest.mark.parametrize(
    "values",
    [{"duration": 0.0}, {"duration": float("inf")}, {"setpoint": float("inf")}],
)
def test_invalid_experiment_scenario_is_rejected(values: dict[str, float]) -> None:
    """Scenario commands and duration must be finite and usable."""
    with pytest.raises(PIDInputError):
        ExperimentScenario(**values)


def test_export_writes_trace_summary_manifest_and_plot(
    scenario: ExperimentScenario, tmp_path: Path
) -> None:
    """Experiment artifacts should be complete and machine-readable."""
    runs = run_gain_sweep(
        "kp",
        (1.0,),
        base_config=PIDConfig(kp=1.0, sample_time=0.1),
        scenario=scenario,
    )
    paths = export_experiment(runs, tmp_path / "nested", name="test")
    assert len(paths) == 4
    assert all(path.exists() for path in paths)
    manifest = json.loads((tmp_path / "nested/test-manifest.json").read_text())
    assert manifest["kp=1"]["kp"] == 1.0


def test_empty_experiment_export_is_rejected(tmp_path: Path) -> None:
    """Export should not produce misleading empty artifacts."""
    with pytest.raises(PIDInputError):
        export_experiment({}, tmp_path, name="empty")

