"""Tests for tabular export and headless plot construction."""

import matplotlib
import matplotlib.pyplot as plt
import pytest

matplotlib.use("Agg")

from pid_controller import (  # noqa: E402
    FirstOrderProcess,
    PIDConfig,
    PIDController,
    SimulationResult,
    plot_comparison,
    plot_simulation,
    run_closed_loop,
)
from pid_controller.exceptions import PIDInputError  # noqa: E402


@pytest.fixture
def result() -> SimulationResult:
    """Return a small deterministic response."""
    return run_closed_loop(
        PIDController(PIDConfig(kp=1.0, sample_time=0.5)),
        FirstOrderProcess(),
        duration=1.0,
        setpoint=1.0,
    )


def test_dataframe_and_csv_export(result: SimulationResult, tmp_path: object) -> None:
    """Every record field should survive tabular conversion and CSV export."""
    from pathlib import Path

    frame = result.to_dataframe()
    assert list(frame.columns) == [
        "time",
        "setpoint",
        "measurement",
        "error",
        "proportional",
        "integral",
        "derivative",
        "raw_output",
        "output",
        "saturated",
    ]
    path = Path(str(tmp_path)) / "trace.csv"
    result.to_csv(path)
    assert path.read_text(encoding="utf-8").startswith("time,setpoint,measurement")


def test_simulation_plot_has_four_signal_panels(result: SimulationResult) -> None:
    """Standard plot should separate response, output, error, and PID terms."""
    figure = plot_simulation(result, title="Test response")
    assert len(figure.axes) == 4
    assert figure._suptitle is not None
    plt.close(figure)


def test_comparison_plot_overlays_named_results(result: SimulationResult) -> None:
    """Comparison plot should contain setpoint plus each supplied response."""
    figure = plot_comparison({"P": result, "PID": result})
    assert len(figure.axes[0].lines) == 3
    plt.close(figure)


def test_empty_plots_are_rejected() -> None:
    """Plot functions should fail clearly without data."""
    with pytest.raises(PIDInputError):
        plot_simulation(SimulationResult((), 0.1))
    with pytest.raises(PIDInputError):
        plot_comparison({})

