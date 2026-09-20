"""Regenerate the curated comparison artifacts used by project documentation."""

from dataclasses import asdict
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from pid_controller import (
    ExperimentScenario,
    PIDConfig,
    plot_comparison,
    run_controller_comparison,
)

OUTPUT_DIRECTORY = Path("docs/assets")
SAMPLE_TIME = 0.05


def main() -> None:
    """Run the documented comparison and write its plot and metric table."""
    runs = run_controller_comparison(
        {
            "P": PIDConfig(kp=2.0, sample_time=SAMPLE_TIME),
            "PI": PIDConfig(kp=2.0, ki=0.5, sample_time=SAMPLE_TIME),
            "PID": PIDConfig(
                kp=2.0,
                ki=0.5,
                kd=0.1,
                sample_time=SAMPLE_TIME,
                derivative_filter_tau=0.1,
            ),
        },
        scenario=ExperimentScenario(duration=20.0, setpoint=1.0),
    )
    OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)
    summary = pd.DataFrame(
        [{"controller": label, **asdict(run.metrics)} for label, run in runs.items()]
    )
    summary.to_csv(OUTPUT_DIRECTORY / "controller-comparison-summary.csv", index=False)
    figure = plot_comparison(
        {label: run.simulation for label, run in runs.items()},
        title="P vs PI vs PID — first-order mathematical process",
    )
    figure.savefig(
        OUTPUT_DIRECTORY / "controller-comparison.png",
        dpi=150,
        bbox_inches="tight",
    )
    plt.close(figure)


if __name__ == "__main__":
    main()

