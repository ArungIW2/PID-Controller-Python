"""Deterministic closed-loop simulation orchestration."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from math import isclose, isfinite
from pathlib import Path
from typing import TYPE_CHECKING, cast

from pid_controller.controller import PIDController
from pid_controller.exceptions import PIDInputError
from pid_controller.models import ProcessModel

if TYPE_CHECKING:
    from pandas import DataFrame


@dataclass(frozen=True, slots=True)
class SetpointChange:
    """Setpoint value that becomes active at a simulation time."""

    time: float
    value: float


@dataclass(frozen=True, slots=True)
class SimulationRecord:
    """One immutable sample of all relevant closed-loop signals."""

    time: float
    setpoint: float
    measurement: float
    error: float
    proportional: float
    integral: float
    derivative: float
    raw_output: float
    output: float
    saturated: bool


@dataclass(frozen=True, slots=True)
class SimulationResult:
    """Completed deterministic simulation and its sample time."""

    records: tuple[SimulationRecord, ...]
    sample_time: float

    def signal(self, name: str) -> tuple[float, ...]:
        """Extract a named numeric signal from every record."""
        if name not in SimulationRecord.__dataclass_fields__ or name == "saturated":
            raise KeyError(f"unknown numeric signal: {name}")
        return tuple(float(getattr(record, name)) for record in self.records)

    def to_dataframe(self) -> DataFrame:
        """Convert all records to a Pandas DataFrame for analysis or export."""
        import pandas as pd

        return cast(
            "DataFrame",
            pd.DataFrame(
                [
                    {
                        field: getattr(record, field)
                        for field in record.__dataclass_fields__
                    }
                    for record in self.records
                ]
            ),
        )

    def to_csv(self, path: str | Path) -> None:
        """Write records to CSV without an implicit index column."""
        self.to_dataframe().to_csv(path, index=False)


def run_closed_loop(
    controller: PIDController,
    process: ProcessModel,
    *,
    duration: float,
    setpoint: float,
    setpoint_changes: Iterable[SetpointChange] = (),
) -> SimulationResult:
    """Run a fixed-step controller/model feedback loop.

    Samples include both ``t=0`` and ``t=duration``. The controller output at a
    sample advances the process to the next sample.
    """
    if not isfinite(duration) or duration <= 0.0:
        raise PIDInputError("duration must be finite and greater than zero")
    if not isfinite(setpoint):
        raise PIDInputError("setpoint must be finite")

    sample_time = controller.config.sample_time
    exact_steps = duration / sample_time
    steps = round(exact_steps)
    if not isclose(exact_steps, steps, rel_tol=0.0, abs_tol=1e-9):
        raise PIDInputError("duration must be an integer multiple of sample_time")

    changes = tuple(setpoint_changes)
    previous_time = -1.0
    for change in changes:
        if (
            not isfinite(change.time)
            or not isfinite(change.value)
            or change.time < 0.0
            or change.time > duration
        ):
            raise PIDInputError("setpoint changes must be finite and within the run")
        if change.time < previous_time:
            raise PIDInputError("setpoint changes must be ordered by time")
        previous_time = change.time

    records: list[SimulationRecord] = []
    active_setpoint = setpoint
    change_index = 0
    for index in range(steps + 1):
        time = index * sample_time
        while change_index < len(changes) and changes[change_index].time <= time:
            active_setpoint = changes[change_index].value
            change_index += 1

        control = controller.update(active_setpoint, process.output)
        records.append(
            SimulationRecord(
                time=time,
                setpoint=active_setpoint,
                measurement=process.output,
                error=control.error,
                proportional=control.proportional,
                integral=control.integral,
                derivative=control.derivative,
                raw_output=control.raw_output,
                output=control.output,
                saturated=control.saturated,
            )
        )
        if index < steps:
            process.step(control.output, sample_time)

    return SimulationResult(tuple(records), sample_time)
