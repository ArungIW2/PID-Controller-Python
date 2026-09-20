"""Plotting functions that consume completed simulation records."""

from __future__ import annotations

from collections.abc import Mapping

import matplotlib.pyplot as plt
from matplotlib.figure import Figure

from pid_controller.exceptions import PIDInputError
from pid_controller.simulation import SimulationResult


def plot_simulation(
    result: SimulationResult, *, title: str = "PID closed-loop response"
) -> Figure:
    """Plot SP/PV, output, error, and PID terms without displaying the figure."""
    if not result.records:
        raise PIDInputError("cannot plot an empty simulation")
    times = result.signal("time")
    figure, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 10))
    axes[0].plot(times, result.signal("setpoint"), "--", label="Setpoint")
    axes[0].plot(times, result.signal("measurement"), label="Process variable")
    axes[0].set_ylabel("Value")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, result.signal("output"), label="Output", color="tab:orange")
    axes[1].set_ylabel("Output")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times, result.signal("error"), label="Error", color="tab:red")
    axes[2].set_ylabel("Error")
    axes[2].grid(True, alpha=0.3)

    for signal, label in (
        ("proportional", "P"),
        ("integral", "I"),
        ("derivative", "D"),
    ):
        axes[3].plot(times, result.signal(signal), label=label)
    axes[3].set_xlabel("Time [s]")
    axes[3].set_ylabel("PID terms")
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)
    figure.suptitle(title)
    figure.tight_layout()
    return figure


def plot_comparison(
    results: Mapping[str, SimulationResult], *, title: str = "Controller comparison"
) -> Figure:
    """Overlay process-variable responses from comparable simulations."""
    if not results:
        raise PIDInputError("comparison requires at least one result")
    figure, axis = plt.subplots(figsize=(10, 5))
    first = next(iter(results.values()))
    axis.plot(first.signal("time"), first.signal("setpoint"), "k--", label="Setpoint")
    for label, result in results.items():
        axis.plot(result.signal("time"), result.signal("measurement"), label=label)
    axis.set_title(title)
    axis.set_xlabel("Time [s]")
    axis.set_ylabel("Value")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    return figure

